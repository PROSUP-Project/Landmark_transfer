#include <iostream>
#include <string>

#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/optional.hpp>

#include <itkPCAModelBuilder.h>

#include <DataManager.h>
#include <vtkStandardMeshRepresenter.h>
#include <StatisticalModel.h>

#include "FileIO.hpp"
#include "GaussModel.hpp"
#include "Bone.hpp"
#include "BoneManager.hpp"
#include "SingleBoneTypeManager.hpp"
#include "utils.hpp"

#define DEFAULT_RIGIT_ITER 1000
#define DEFAULT_GAUSS_FIT_ITER 1500
#define DEFAULT_KERNEL_LEVELS 3
#define DEFAULT_KERNEL_SIGMA 75.0
#define DEFAULT_KERNEL_SCALE 200.0
#define DEFAULT_GAUSS_COMP 250.0
#define DEFAULT_VOLUME_PERCENTAGE 0.6
#define DEFAULT_LANDMARK_VARIANCE 0.1
#define DEFAULT_ATLAS_VARIANCE 0

/**
 * Generates an atlas with the given bones.
 * @param config		the configuration for the atlas creation
 * @param bonemanager	single bone type manager for all the bones that are used to create the atlas
 * @param outputdir		deters the path to the output directory
 * @param gauss_path	path to a pre calculated Gaussian model as a boost optional
 */
void generateAtlas(boost::property_tree::ptree config, SingleBoneTypeManager* bonemanager, std::string& outputdir, boost::optional<std::string> gauss_path);

int main(int argc, char *argv[]){
	if(argc < 2){
		std::cout << "Usage " << argv[0] << " config_file" << std::endl;
		exit(-1);
	}
	
	if (!boost::filesystem::exists(argv[1]))
	{
		std::cout << "The specified configuration file does not exist...exiting" << std::endl;
		exit(-1);
	}
	
	boost::property_tree::ptree config;
	
	try{
		boost::property_tree::read_json(argv[1], config);
	}catch( boost::property_tree::json_parser::json_parser_error &e){
		std::cout << "Configuration file can not be read..." << std::endl;
		std::cout << "Message: " << e.message() << std::endl;
		std::cout << "File-name: " << e.filename() << std::endl;
		std::cout << "Line: " << e.line() << std::endl;
		std::cout << "...exiting." << std::endl;
		exit(-1);
	}
	
	boost::property_tree::ptree parameters = config.get_child("parameters");
	
	//get output directory structure	
	std::string outputdir_radius_right = parameters.get<std::string>("radius_right_folder", "datadir/radius/right");
	std::string outputdir_radius_left = parameters.get<std::string>("radius_left_folder", "datadir/radius/left");
	std::string outputdir_ulna_right = parameters.get<std::string>("ulna_right_folder", "datadir/ulna/right");
	std::string outputdir_ulna_left = parameters.get<std::string>("ulna_left_folder", "datadir/ulna/left");
	
	//load bone data
	BoneManager* bm = new BoneManager(config.get_child("bones"), outputdir_radius_right, outputdir_radius_left, outputdir_ulna_right, outputdir_ulna_left);
	
	//generate the atlases
	std::cout << "Radius Right:";
	generateAtlas(parameters, bm->getRadii_r(), outputdir_radius_right, parameters.get_optional<std::string>("gauss_path_radius_r"));
	std::cout << "Radius Left:";
	generateAtlas(parameters, bm->getRadii_l(), outputdir_radius_left, parameters.get_optional<std::string>("gauss_path_radius_l"));
	std::cout << "Ulna Right:";
	generateAtlas(parameters, bm->getUlnae_r(), outputdir_ulna_right, parameters.get_optional<std::string>("gauss_path_ulna_r"));
	std::cout << "Ulna Left:";
	generateAtlas(parameters, bm->getUlnae_l(), outputdir_ulna_left, parameters.get_optional<std::string>("gauss_path_ulna_l"));
	
	delete bm;
	
	return EXIT_SUCCESS;
}

void generateAtlas(boost::property_tree::ptree config, SingleBoneTypeManager* bonemanager, std::string& outputdir, boost::optional<std::string> gauss_path){
	//check if bones exist
	if(bonemanager->getBones().size() < 1){
		std::cout << " No bones of this type." << std::endl;
		return;
	}
	
	boost::filesystem::create_directories(outputdir);
	
	itk::StatisticalModel< itk::Mesh<float, 3> >::Pointer gaussModel = itk::StatisticalModel< itk::Mesh<float, 3> >::New();
	if(gauss_path && boost::filesystem::exists(gauss_path.get().c_str())){
		//get pre calculated Gaussian process model id it is given
		itk::StandardMeshRepresenter<float, 3>::Pointer model_representer = itk::StandardMeshRepresenter<float, 3>::New();
		gaussModel->Load(model_representer, gauss_path.get().c_str());
		std::cout << std::endl << "Found pre calculated Gaussian process model." << std::endl;
		//align the bones with the model
		std::cout << "Rigid alignment..." << std::flush;
		int rigit_iter = config.get<int>("rigid_iter", DEFAULT_RIGIT_ITER);
		bonemanager->align_bones(gaussModel, rigit_iter);
		std::cout << "Done." << std::endl;
	}else{
		//align the bones
		std::cout << std::endl << "Rigid alignment..." << std::flush;
		int rigit_iter = config.get<int>("rigid_iter", DEFAULT_RIGIT_ITER);
		bonemanager->align_bones(rigit_iter);
		std::cout << "Done." << std::endl;
		
		//generate Gaussian process model
		std::cout << "Generate GaussianProcess Model..." << std::flush;
		//get parameters from configuration if present, if not default values will be used
		double sigma = config.get<double>("kernel_sigma", DEFAULT_KERNEL_SIGMA);
		int levels = config.get<int>("kernel_levels", DEFAULT_KERNEL_LEVELS);
		int scale = config.get<int>("kernel_scale", DEFAULT_KERNEL_SCALE);
		int num_comp = config.get<int>("gauss_comp", DEFAULT_GAUSS_COMP);
		gaussModel = bonemanager->getMultiLevelGPModel(sigma, scale, levels, num_comp);
		std::cout << "Done." << std::endl;
		
		//Save the gauss model, can be removed later
		std::string gauss_model_path(outputdir + "/gaussModel.h5");
		gaussModel->Save(gauss_model_path.c_str());
	}
	
	//DataManager for all Meshes
	statismo::DataManager< itk::Mesh<float, 3> >* datamanager = bonemanager->getDatamanager();
	
	//generate and add correspondent data
	std::cout << "Generate correspondent mesh for each input file..." << std::flush;
	int gauss_fit_iter = config.get<int>("gauss_fit_iter", DEFAULT_GAUSS_FIT_ITER);
	float volume_percentage = config.get<float>("correspond_volume_percentage", DEFAULT_VOLUME_PERCENTAGE);
	boost::optional<double> landmark_variance = config.get_optional<double>("landmark_variance");
	if(landmark_variance){
		bonemanager->addCorrespondentDataToDataManager(datamanager, gaussModel, gauss_fit_iter, volume_percentage, landmark_variance.get());
	}else{
		bonemanager->addCorrespondentDataToDataManager(datamanager, gaussModel, gauss_fit_iter, volume_percentage, DEFAULT_LANDMARK_VARIANCE);
	}
	std::cout << "Done." << std::endl;
	
	//Build the statistical model
	std::cout << "Creating Statistical model..." << std::flush;
	itk::PCAModelBuilder< itk::Mesh<float, 3> >::Pointer modelBuilder = itk::PCAModelBuilder< itk::Mesh<float, 3> >::New();
	itk::StatisticalModel< itk::Mesh<float, 3> >::Pointer model;
	float atlas_variance = config.get<float>("atlas_variance", DEFAULT_ATLAS_VARIANCE);
	if(datamanager->GetData().size() < 2){
		datamanager->Delete();
		std::cout << "Less than 2 meshes in the data manager. Aborting..." << std::endl;
		return;
	}
	model = modelBuilder->BuildNewModel(datamanager->GetData(), atlas_variance);
	std::cout << "Done." << std::endl;
	
	//Save the model
	std::string statistical_model_path(outputdir + "/statisticalModel.h5");
	std::cout << "Writing: " << statistical_model_path << std::endl;
	model->Save(statistical_model_path.c_str());
	
	//get parameters for an enhanced model
	boost::optional<double> atlas_sigma = config.get_optional<double>("atlas_sigma");
	boost::optional<double> atlas_scale = config.get_optional<double>("atlas_scale");
	boost::optional<int> atlas_num_comp = config.get_optional<int>("atlas_num_comp");
	//check if all parameters are set
	if(atlas_sigma && atlas_scale && atlas_num_comp){
		//build Gaussian improved statistical model
		std::cout << "Creating enhanced Statistical model..." << std::flush;
		itk::StatisticalModel< itk::Mesh<float, 3> >::Pointer enhanced_model = GaussModel::generateImprovedAtlas(datamanager, atlas_sigma.get(), atlas_scale.get(), atlas_num_comp.get());
		std::cout << "Done." << std::endl;
		
		//Save the model
		std::string statistical_enhanced_model_path(outputdir + "/enhancedStatisticalModel.h5");
		std::cout << "Writing: " << statistical_enhanced_model_path << std::endl;
		enhanced_model->Save(statistical_enhanced_model_path.c_str());
	}
	datamanager->Delete();
}
