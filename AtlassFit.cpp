#include <string>
#include <sys/types.h>
#include <iostream>
#include <fstream> 
#include <limits>
#include <algorithm>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/optional.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>

#include <vnl/vnl_vector.h>

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkIterativeClosestPointTransform.h>
#include <vtkLandmarkTransform.h>
#include <vtkTransformPolyDataFilter.h>

#include <itkMesh.h>
#include <itkStandardMeshRepresenter.h>
#include <itkStatisticalModel.h>

#include <DataManager.h>

#include "FileIO.hpp"
#include "utils.hpp"
#include "Landmark.hpp"

/**
 * Prints the given landmarks in the standard output.
 * @param landmarks		map containing the landmarks that should be printed
 */
void printLandmarks(std::map<std::string, Landmark*> landmarks);

/**
 * Reads the control landmarks from the given configuration and returns a map containing this landmarks.
 * If no control landmarks are provided the map will be empty.
 * @param config	the program configuration
 * @return map containing the control landmarks
 */
std::map<std::string, Landmark*> loadControlLandmarks(boost::property_tree::ptree config);

/**
 * Calculates for each landmark the distance between the calculated landmark coordinates and the 
 * control landmarks.
 * @param calculated	the map containing the calculated landmarks
 * @param control		the map containing the control landmarks
 * @return a map containing all distances
 */
std::map<std::string, double> calculateLandmarkError(std::map<std::string, Landmark*> calculated, std::map<std::string, Landmark*> control);

/**
 * Prints all errors of the landmarks and the sum of the errors.
 * @param landmarkErrors	the map containing the error data
 */
void printLandmarkErrors(std::map<std::string, double> landmarkErrors);

/**
 * Adds the error informations to the given property tree.
 * @param landmarkErrors	the map containing the error data
 * @param tree				the property tree into which the data should be inserted
 */
void addLandmarkErrorsToPTree(std::map<std::string, double> landmarkErrors, boost::property_tree::ptree* tree);

/**
 * Adds the given landmark information to the given property tree.
 * @param landmarks		the map containing the landmarks
 * @param tree			the property tree into which the data should be inserted
 */
void addLandmarksToPTree(std::map<std::string, Landmark*> landmarks, boost::property_tree::ptree* tree);

/**
 * Writes a text file that includes scala code.
 * This file can be opened as source code in scalismo lab.
 * By execution of the code a visual result of the fitting is shown.
 * @param path				the path to the file that should be created
 * @param output_vtk		the path to the vtk file with the landmarks in it
 * @param type				the bone type
 * @param controlLandmarks	the map containing the control landmarks, or an empty map
 */
void writeScalismoGUICode(const std::string& path, const std::string& output_vtk, BoneType::e type, std::map<std::string, Landmark*> controlLandmarks);

int main(int argc, char *argv[]){
	if(argc < 2){
		std::cout << "Usage " << argv[0] << " configuration_file" << std::endl;
		exit(-1);
	}
	
	if (!boost::filesystem::exists(argv[1]))
	{
		std::cout << "The specified configuration file does not exist...exiting" << std::endl;
		exit(-1);
	}
	
	//read configuration
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
	
	//get configuration values
	boost::optional<std::string> atlas_path = config.get_optional<std::string>("atlas_path");
	boost::optional<std::string> patient_path = config.get_optional<std::string>("patient_path");
	boost::optional<std::string> s_type = config.get_optional<std::string>("type");
	int align_iterations = config.get<int>("align_iter", 1000);
	int fitting_iterations = config.get<int>("fit_iter", 2000);
	
	//check configuration
	if(!atlas_path || !boost::filesystem::exists(atlas_path.get().c_str())){
		std::cout << "Atlas path not set correctly...exiting." << std::endl;
		exit(-1);
	}
	if(!patient_path || !boost::filesystem::exists(patient_path.get().c_str())){
		std::cout << "Patient path not set correctly...exiting." << std::endl;
		exit(-1);
	}
	BoneType::e type;
	if(s_type.get() == "radius"){
		type = BoneType::radius;
	} else if(s_type.get() == "ulna"){
		type = BoneType::ulna;
	} else {
		std::cout << "Unsupported bone type...exiting." << std::endl;
		exit(-1);
	}
	
	//load control landmarks if provided
	std::map<std::string, Landmark*> controlLandmarks = loadControlLandmarks(config);
	
	//load the patient data
    vtkSmartPointer<vtkPolyData> patientMesh = loadVTKPolydata(patient_path.get());

    //load the model
    itk::StatisticalModel< itk::Mesh<float, 3> >::Pointer model = itk::StatisticalModel< itk::Mesh<float, 3> >::New();
    itk::StandardMeshRepresenter<float, 3>::Pointer representer = itk::StandardMeshRepresenter<float, 3>::New();
    model->Load(representer, atlas_path.get().c_str());
	
	//get the representer of the model and transform it into a vtk mesh
	vtkSmartPointer<vtkPolyData> vtkReference = vtkFromItk(representer->GetReference());
	//vtkSmartPointer<vtkPolyData> vtkReference = vtkFromItk(representer->GetReference(), "model_reference.vtk"); //with path to save the representer as vtk file
	 
	std::cout << "Align patient data with the model..." << std::flush;
	// Set up rigid transform
	vtkSmartPointer<vtkIterativeClosestPointTransform> icp = getRigidTransformation(patientMesh, vtkReference, align_iterations);
	
	// Transform the source points to the target
	vtkSmartPointer<vtkPolyData> aligned_patientMeshVtk = performPolyDataTransform(patientMesh, icp);
	std::cout << "Done." << std::endl;
	
	std::string output_vtk_temp = config.get<std::string>("vtk_output_path", ".");
	
	//save file and transform it into a itk mesh
	std::string aligned_path = output_vtk_temp + "/fit_aligned.vtk";
	itk::Mesh<float, 3>::Pointer aligned_patientMesh = itkFromVtk(aligned_patientMeshVtk, aligned_path);

	//fit the model to the aligned mesh
	std::cout << "Fit the model to the patient data..." << std::flush;
    itk::Mesh<float, 3  >::Pointer fittedMesh = fitModelToMesh(model, aligned_patientMesh, fitting_iterations);
	std::cout << "Done." << std::endl;
	
	//save the fitted mesh, can be done to visualize this step
	//std::string fitted_path("fitted_model.vtk");
	//saveVTKFile(fittedMesh, fitted_path);
	
	//get landmarks from the mesh
	std::map<std::string, Landmark*> landmarks = Landmark::getLandmarksFromMesh(fittedMesh, type);
	
	//project landmarks on the mesh surface
	for (std::map<std::string, Landmark*>::iterator it = landmarks.begin(); it != landmarks.end(); ++it) {
		it->second->setCoordinates(getClosestPointOnMesh(aligned_patientMesh, it->second->getLandmarkPoint()));
	}
	
	//print landmarks on the mesh
	std::cout << "Landmarks projected on the patient:" << std::endl;
	printLandmarks(landmarks);
	
	int numPointsMesh = aligned_patientMesh->GetNumberOfPoints();
	Landmark::addLandmarksToMesh(aligned_patientMesh, landmarks, type);
	
	std::string result_path = output_vtk_temp + "/output.vtk";
	saveVTKFile(aligned_patientMesh, result_path);
	//transform the path into an absolute path
	boost::filesystem::path p(result_path);
	p = boost::filesystem::canonical(p);
	std::string output_vtk = p.string();
	
	//property tree for the output, if a path for it is given
	boost::property_tree::ptree output;
	
	//make landmark checks if control landmarks are given
	if(!controlLandmarks.empty()){
		//apply the alignment on the control landmarks
		Landmark::transformLandmarks(controlLandmarks, icp);
		//std::cout << "Control landmarks:" << std::endl;
		//printLandmarks(controlLandmarks);
		
		std::map<std::string, double> landmarkErrors = calculateLandmarkError(landmarks, controlLandmarks);
		std::cout << "Errors after the projection:" << std::endl;
		printLandmarkErrors(landmarkErrors);
		
		//add landmark error information to the output data
		addLandmarkErrorsToPTree(landmarkErrors, &output);
	}
	
	boost::optional<std::string> scala_output_path_temp = config.get_optional<std::string>("scalismo_gui_path");
		if(scala_output_path_temp){			
			//write code that can be executed in the scalismo-GUI for a graphical result
			std::cout << "Writing: " << scala_output_path_temp.get() << std::endl;
			writeScalismoGUICode(scala_output_path_temp.get(), output_vtk, type, controlLandmarks);
			//transform the path into an absolute path
			boost::filesystem::path p(scala_output_path_temp.get());
			p = boost::filesystem::canonical(p);
			std::string scala_output_path = p.string();
			output.put("scalismo_gui_path", scala_output_path);	
		}
	
	boost::optional<std::string> json_outputpath = config.get_optional<std::string>("json_output_path");
	if(json_outputpath){
		std::cout << "Writing: " << json_outputpath.get() << std::endl;
		output.put("vtk_path", output_vtk);
		output.put("patient", patient_path.get());
		addLandmarksToPTree(landmarks, &output);
		boost::property_tree::write_json(json_outputpath.get(), output);
	}
	
	//free the landmark memory
	for (std::map<std::string, Landmark*>::iterator it = landmarks.begin(); it != landmarks.end(); ++it) {
		delete it->second;
	}
	for (std::map<std::string, Landmark*>::iterator it = controlLandmarks.begin(); it != controlLandmarks.end(); ++it) {
		delete it->second;
	}
	
	return EXIT_SUCCESS;
}

void printLandmarks(std::map<std::string, Landmark*> landmarks){
	for (std::map<std::string, Landmark*>::iterator it = landmarks.begin(); it != landmarks.end(); ++it) {
		std::cout << it->first << ": " << it->second->getLandmarkPoint() << std::endl;
	}
}

std::map<std::string, Landmark*> loadControlLandmarks(boost::property_tree::ptree config){
	std::map<std::string, Landmark*> ret;
	boost::optional<boost::property_tree::ptree &> landmark_tree = config.get_child_optional("landmarks");
	if(landmark_tree){
		for (boost::property_tree::ptree::iterator it = landmark_tree.get().begin(); it != landmark_tree.get().end(); ++it) {
			ret.insert(std::pair<std::string, Landmark*>(it->first, new Landmark(it->first, it->second)));
		}
	}
	return ret;
}

std::map<std::string, double> calculateLandmarkError(std::map<std::string, Landmark*> calculated, std::map<std::string, Landmark*> control){
	std::map<std::string, double> ret;
	for (std::map<std::string, Landmark*>::iterator it = calculated.begin(); it != calculated.end(); ++it) {
		double distance = it->second->getDistance(control.find(it->second->getName())->second);
		ret.insert(std::pair<std::string, double>(it->first, distance));
	}
	return ret;
}

void printLandmarkErrors(std::map<std::string, double> landmarkErrors){
	double sum = 0.0;
	double squaredsum = 0.0;
	for (std::map<std::string, double>::iterator it = landmarkErrors.begin(); it != landmarkErrors.end(); ++it) {
		std::cout << it->first << ": " << it->second << std::endl;
		sum += it->second;
		squaredsum += it->second * it->second;
	}
	std::cout << "Error sum: " << sum << std::endl;
	std::cout << "Squared error sum: " << squaredsum << std::endl;
}

void addLandmarkErrorsToPTree(std::map<std::string, double> landmarkErrors, boost::property_tree::ptree* tree){
	double sum = 0.0;
	double squaredsum = 0.0;
	boost::property_tree::ptree detailedErrors;
	for (std::map<std::string, double>::iterator it = landmarkErrors.begin(); it != landmarkErrors.end(); ++it) {
		detailedErrors.put(it->first, it->second);
		sum += it->second;
		squaredsum += it->second * it->second;
	}
	tree->put("error_sum", sum);
	tree->put("squared_error_sum", squaredsum);
	tree->add_child("detailed_errors", detailedErrors);
}

void writeScalismoGUICode(const std::string& path, const std::string& output_vtk, BoneType::e type, std::map<std::string, Landmark*> controlLandmarks){
	std::ofstream outfile (path.c_str());
	//function to extract landmarks from a mesh
	outfile << "def getLandmarks(mesh:scalismo.mesh.TriangleMesh, num:Int) : Array[scalismo.geometry.Point3D] = {" << std::endl;
	outfile << "	var arr = new Array[scalismo.geometry.Point3D](num)" << std::endl;
	outfile << "	var pos = num" << std::endl;
	outfile << "	while(pos > 0){" << std::endl;
	outfile << "		arr(num-pos) = mesh.point(PointId(mesh.points.length-pos))" << std::endl;
	outfile << "		pos = pos -1" << std::endl;
	outfile << "	}" << std::endl;
	outfile << "	return arr" << std::endl;
	outfile << "}" << std::endl;
	outfile << std::endl << std::endl << std::endl;
	//load the mesh
	outfile << "val file_patient_mesh = new File(\"" << output_vtk << "\")" << std::endl;
	outfile << "val patient_mesh = MeshIO.readMesh(file_patient_mesh).get" << std::endl;
	outfile << "show(patient_mesh, \"patient_mesh\")" << std::endl;
	//get landmarks
	outfile << "val landmarks_calculated = getLandmarks(patient_mesh, " << boost::lexical_cast<std::string>((type==(BoneType::radius))?9:((type==(BoneType::ulna))?11:0)) << ")" << std::endl;
	outfile << "show(landmarks_calculated.toIndexedSeq, \"landmarks_calculated\")" << std::endl;
	//control landmarks
	if(!controlLandmarks.empty()){
		outfile << "val control_landmarks = Array(";
		for (std::map<std::string, Landmark*>::iterator it = controlLandmarks.begin(); it != controlLandmarks.end(); ++it) {
			if(it != controlLandmarks.begin()){
				outfile << ", ";
			}
			outfile << "Point(" << it->second->getX() << "f," << it->second->getY() << "f," << it->second->getZ() << "f)";
		}
		outfile << ")" << std::endl;
		outfile << "show(control_landmarks.toIndexedSeq, \"control_landmarks\")" << std::endl;
	}
	//close file
	outfile.close();
}

void addLandmarksToPTree(std::map<std::string, Landmark*> landmarks, boost::property_tree::ptree* tree){
	boost::property_tree::ptree landmark_tree;
	for (std::map<std::string, Landmark*>::iterator it = landmarks.begin(); it != landmarks.end(); ++it) {
		boost::property_tree::ptree landmark;
		landmark.put("x", it->second->getX());
		landmark.put("y", it->second->getY());
		landmark.put("z", it->second->getZ());
		landmark_tree.add_child(it->first, landmark);
	}
	tree->add_child("landmarks", landmark_tree);
}
