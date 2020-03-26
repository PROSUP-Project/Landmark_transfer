#include <string>
#include <vector>

#include <vnl/vnl_vector.h>

#include <boost/atomic/atomic.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/lexical_cast.hpp>

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkIterativeClosestPointTransform.h>
#include <vtkLandmarkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <DataManager.h>

#include <vtkTriangle.h>
#include <vtkCellArray.h>
#include <vtkLine.h>

#include <itkVector.h>
#include <itkMesh.h>
#include <itkStandardMeshRepresenter.h>
#include <itkStatisticalModel.h>
#include <itkPosteriorModelBuilder.h>
#include <itkPointsLocator.h>

#include "Bone.hpp"
#include "FileIO.hpp"
#include "GaussModel.hpp"
#include "utils.hpp"
#include "BoneType.hpp"

boost::atomic<int> Bone::name(0);

Bone::Bone(boost::property_tree::ptree boneConfig, const std::string& output_folder, BoneType::e t){
	type = t;
	out_folder = output_folder;
	not_aligned_data = 0;
	not_aligned_filename = boneConfig.get<std::string>("path");
	boost::property_tree::ptree landmark_tree = boneConfig.get_child("landmarks");
	for (boost::property_tree::ptree::iterator it = landmark_tree.begin(); it != landmark_tree.end(); ++it) {
		landmarks.insert(std::pair<std::string, Landmark*>(it->first, new Landmark(it->first, it->second)));
	}
}

Bone::~Bone(){
	free_not_aligned_data();
	for (std::map<std::string, Landmark*>::iterator it = landmarks.begin(); it != landmarks.end(); ++it) {
		delete (it->second);
	}
}

vtkPolyData* Bone::get_not_aligned_data(){
	if(not_aligned_data == 0){
		not_aligned_data = loadVTKPolydata(not_aligned_filename);
	}
	return not_aligned_data;
}

void Bone::free_not_aligned_data(){
	if(not_aligned_data != 0){
		not_aligned_data->Delete();
		not_aligned_data = 0;
	}
}

itk::Mesh<float, 3>::Pointer Bone::get_aligned_data(){
	return loadMesh(aligned_filename);
}

BoneType::e Bone::getType(){
	return type;
}

double Bone::getLength(){
	return length;
}

std::string Bone::getOutputFolder(){
	return out_folder;
}

void Bone::transformLandmarks(vtkSmartPointer<vtkIterativeClosestPointTransform> icp){
	Landmark::transformLandmarks(landmarks, icp);
}

void Bone::align_with(Bone* b, int iterations){
	if(getType() == b->getType()){
		align_with(b->get_not_aligned_data(), iterations);
	}
}

std::map<std::string, Landmark*> Bone::getLandmarks(){
	return landmarks;
}

void Bone::align_with(vtkPolyData* b, int iterations){
	// Set up rigid transform
	vtkSmartPointer<vtkIterativeClosestPointTransform> icp = getRigidTransformation(get_not_aligned_data(), b, iterations);
	
	//Apply transformation on landmarks
	transformLandmarks(icp);
	
	// Transform the source points to the target
	vtkSmartPointer<vtkPolyData> transformedData = performPolyDataTransform(get_not_aligned_data(), icp);
	
	aligned_filename = out_folder + "/aligned_" + boost::lexical_cast<std::string>(name++) + ".vtk";
	
	//Write to file
	saveVTKPolydata(transformedData, aligned_filename);
	
	//Calculate length approximation with the bounding box
	double bounds[6];
	/* xmin: bounds[0], xmax: bounds[1]
     * ymin: bounds[2], ymax: bounds[3]
     * zmin: bounds[4], zmax: bounds[5]
	 */
	transformedData->GetBounds(bounds);
	//calculate the length of a diagonal trough the bounding box
	length = sqrt((bounds[1]-bounds[0])*(bounds[1]-bounds[0])+(bounds[3]-bounds[2])*(bounds[3]-bounds[2])+(bounds[5]-bounds[4])*(bounds[5]-bounds[4]));
}

void Bone::multicore_correspond(Bone* b, statismo::DataManager< itk::Mesh<float, 3> >* datamanager, itk::StatisticalModel< itk::Mesh<float, 3> >::Pointer gaussModel, int iterations, float volume_percentage){
	b->correspond(datamanager, gaussModel, iterations, volume_percentage);
}

void Bone::multicore_correspond_with_constrains(Bone* b, statismo::DataManager< itk::Mesh<float, 3> >* datamanager, itk::StatisticalModel< itk::Mesh<float, 3> >::Pointer gaussModel, int iterations, float volume_percentage, double variance){
	b->correspond_with_constrains(datamanager, gaussModel, iterations, volume_percentage, variance);
}

void Bone::correspond(statismo::DataManager< itk::Mesh<float, 3> >* datamanager, itk::StatisticalModel< itk::Mesh<float, 3> >::Pointer gaussModel, int iterations, float volume_percentage){
	//generate a corresponding Mesh
	itk::Mesh<float, 3>::Pointer correspondingMesh = generateCorrespondingMesh(gaussModel, iterations);
	
	//Write to file
	corresponding_filename = aligned_filename;
	boost::replace_all(corresponding_filename, "aligned_", "correspondent_");	
	saveVTKFile(correspondingMesh, corresponding_filename);
	
	//check if the fitting is a plausible fitting
	//the fitting minimizes the distance between the model and the bone mesh but not the distance between the bone mesh and the model, so this distance is used as validation criteria
	if(validateFittingWithVolume(correspondingMesh, volume_percentage)){
		//Add transformed Mesh to the DataManager
		datamanager->AddDataset(correspondingMesh, corresponding_filename);
	}else{
		std::cout << std::endl << "Skipped: " << corresponding_filename << " fitting does not match the input file.";
	}
}

void Bone::correspond_with_constrains(statismo::DataManager< itk::Mesh<float, 3> >* datamanager, itk::StatisticalModel< itk::Mesh<float, 3> >::Pointer gaussModel, int iterations, float volume_percentage, double variance){
	//generate a corresponding Mesh
	itk::Mesh<float, 3>::Pointer correspondingMesh = generateCorrespondingMeshWithConstraints(gaussModel, iterations, variance);
	
	//Write to file
	corresponding_filename = aligned_filename;
	boost::replace_all(corresponding_filename, "aligned_", "correspondent_");	
	saveVTKFile(correspondingMesh, corresponding_filename);
	
	//check if the fitting is a plausible fitting
	//the fitting minimizes the distance between the model and the bone mesh but not the distance between the bone mesh and the model, so this distance is used as validation criteria
	if(validateFittingWithVolume(correspondingMesh, volume_percentage)){
		//Add transformed Mesh to the DataManager
		datamanager->AddDataset(correspondingMesh, corresponding_filename);
	}else{
		std::cout << std::endl << "Skipped: " << corresponding_filename << " fitting does not match the input file.";
	}
}

bool Bone::validateFittingWithDistance(itk::Mesh<float, 3>::Pointer mesh, float max_distance){
	//load the mesh data that is used to fit on
	itk::Mesh<float, 3>::Pointer boneMesh = get_aligned_data();
	
	//add landmarks to the mesh
	addLandmarksToMesh(boneMesh);

	//Get Points from the given mesh
	itk::PointsLocator< itk::Mesh<float, 3>::PointsContainer >::Pointer ptLocator = itk::PointsLocator< itk::Mesh<float, 3>::PointsContainer >::New();
	ptLocator->SetPoints(mesh->GetPoints());
	ptLocator->Initialize();
	
	//iterate over all points of the bone mesh
	for(itk::Mesh<float, 3>::PointsContainerIterator  it = boneMesh->GetPoints()->Begin(); it != boneMesh->GetPoints()->End(); ++it){
		int closestPointId = ptLocator->FindClosestPoint(it.Value());
		itk::Point<float,3> pointOnMesh = mesh->GetPoint(closestPointId);
		
		//compute the distance between the point on the bone mesh and the closest point on the given mesh
		double distance = pointOnMesh.EuclideanDistanceTo(it.Value());
		
		if(distance > max_distance){
			return false;
		}
	}
	
	return true;
}

bool Bone::validateFittingWithVolume(itk::Mesh<float, 3>::Pointer mesh, float min_percentage){
	//load the mesh data that is used to fit on
	itk::Mesh<float, 3>::Pointer boneMesh = get_aligned_data();
	
	//get the volumes
	double boneVolume = calculateVolume(boneMesh);
	
	int num_to_remove = 0;
	if(getType() == BoneType::radius){
		num_to_remove = 9;
	} else if(getType() == BoneType::ulna){
		num_to_remove = 11;
	}
	
	itk::Mesh<float, 3>::Pointer mesh2 = itk::Mesh<float, 3>::New();
	
	//copy all points except the landmarks into a new mesh
	for(int i = 0; i < mesh->GetNumberOfPoints()-num_to_remove; ++i){
		mesh2->SetPoint(i, mesh->GetPoint(i));
	}
	
	//copy all cells
	mesh2->SetCells(mesh->GetCells());
	
	
	double meshVolume = calculateVolume(mesh2);
	
	//calculate percentage
	double p = meshVolume / boneVolume;
	
	return p >= min_percentage;
}

itk::Mesh<float, 3>::Pointer Bone::generateCorrespondingMesh(itk::StatisticalModel< itk::Mesh<float, 3> >::Pointer gaussModel, int iterations){
	//load the mesh data that is used to fit and project on
	itk::Mesh<float, 3>::Pointer data = get_aligned_data();

	//add landmarks to the mesh
	addLandmarksToMesh(data);
	
    itk::Mesh<float, 3>::Pointer fittedMesh = fitModelToMesh(gaussModel, data, iterations);

	return fittedMesh;
}

itk::Mesh<float, 3>::Pointer Bone::generateCorrespondingMeshWithConstraints(itk::StatisticalModel< itk::Mesh<float, 3> >::Pointer gaussModel, int iterations, double variance){
	//load the mesh data that is used to fit on
	itk::Mesh<float, 3>::Pointer data = get_aligned_data();
	
	//add landmarks to the mesh
	addLandmarksToMesh(data);

	//compute the landmark constrained posterior model
    itk::StatisticalModel<itk::Mesh<float, 3> >::Pointer constraintModel = getPosteriorModel(gaussModel, data, variance);
	
	std::string constrained_model_path = aligned_filename;
	boost::replace_all(constrained_model_path, "aligned_", "constrained_model_");
	boost::replace_all(constrained_model_path, ".vtk", ".h5");
	constraintModel->Save(constrained_model_path.c_str());
	
	//call the generateCorrespondingMesh with the constrained model and return the result
	return generateCorrespondingMesh(constraintModel, iterations);
}

itk::StatisticalModel< itk::Mesh<float, 3> >::Pointer Bone::getGaussianProcessModel(double sigma1, double sigma2, double relation, int scale, int num_comp){
	itk::Mesh<float, 3>::Pointer mesh = get_aligned_data();
	
	addLandmarksToMesh(mesh);
	
	//Mesh representer for the GaussModel
	itk::StandardMeshRepresenter<float, 3>::Pointer representer = itk::StandardMeshRepresenter<float, 3>::New();
	representer->SetReference(mesh);
	
	//GaussModel for the registration
	itk::StatisticalModel< itk::Mesh<float, 3> >::Pointer gaussModel = GaussModel::generateGaussModel(representer, sigma1, sigma2, relation, scale, num_comp);
	
	return gaussModel;
}

itk::StatisticalModel< itk::Mesh<float, 3> >::Pointer Bone::getMultiLevelGaussianProcessModel(double sigma, int scale, int levels, int num_comp){
	itk::Mesh<float, 3>::Pointer mesh = get_aligned_data();
	
	addLandmarksToMesh(mesh);
	
	//Mesh representer for the GaussModel
	itk::StandardMeshRepresenter<float, 3>::Pointer representer = itk::StandardMeshRepresenter<float, 3>::New();
	representer->SetReference(mesh);
	
	//GaussModel for the registration
	itk::StatisticalModel< itk::Mesh<float, 3> >::Pointer gaussModel = GaussModel::generateMultiLevelGaussModel(representer, sigma, scale, levels, num_comp);
	
	return gaussModel;
}

itk::StatisticalModel<itk::Mesh<float, 3> >::PointValueListType Bone::getLandmarkConstraints(itk::StatisticalModel< itk::Mesh<float, 3> >::Pointer gaussModel, itk::Mesh<float, 3>::Pointer mesh){
	itk::StatisticalModel<itk::Mesh<float, 3> >::PointValueListType constraints;
	std::map<std::string, Landmark*> modelLandmarks = Landmark::getLandmarksFromMesh(gaussModel->GetRepresenter()->GetReference(), getType());
	std::map<std::string, Landmark*> meshLandmarks = Landmark::getLandmarksFromMesh(mesh, getType());
	for (std::map<std::string, Landmark*>::iterator it = modelLandmarks.begin(); it != modelLandmarks.end(); ++it) {
		if(it->second->getLandmarkPoint().EuclideanDistanceTo(meshLandmarks.find(it->second->getName())->second->getLandmarkPoint()) > 50){
			std::cout << std::endl << "Landmarks are far from each other, this could mean there are errors or just that the bones have really different sizes, has to be checked. (DEBUG OUTPUT)" << std::endl;
			std::cout << aligned_filename << " -> Model Landmark(" << it->second->getName() << "): " << it->second->getLandmarkPoint() << " Mesh Landmark(" << 
				meshLandmarks.find(it->second->getName())->second->getName() << "): " << meshLandmarks.find(it->second->getName())->second->getLandmarkPoint() << std::endl;
		}
		itk::StatisticalModel<itk::Mesh<float, 3> >::PointValuePairType pointValue(it->second->getLandmarkPoint(), meshLandmarks.find(it->second->getName())->second->getLandmarkPoint());
			constraints.push_back(pointValue);
		delete it->second;
	}
	for (std::map<std::string, Landmark*>::iterator it = meshLandmarks.begin(); it != meshLandmarks.end(); ++it) {
		delete it->second;
	}
	return constraints;
}

itk::StatisticalModel<itk::Mesh<float, 3> >::Pointer Bone::getPosteriorModel(itk::StatisticalModel< itk::Mesh<float, 3> >::Pointer gaussModel, itk::Mesh<float, 3>::Pointer mesh, double variance)
{
	itk::PosteriorModelBuilder<itk::Mesh<float, 3> >::Pointer pfmb = itk::PosteriorModelBuilder<itk::Mesh<float, 3> >::New();
	itk::StatisticalModel<itk::Mesh<float, 3> >::Pointer constraintModel = pfmb->BuildNewModelFromModel(gaussModel, getLandmarkConstraints(gaussModel, mesh), variance);
	return constraintModel;
}

void Bone::addLandmarksToMesh(itk::Mesh<float, 3>::Pointer mesh){
	Landmark::addLandmarksToMesh(mesh, landmarks, getType());
}
