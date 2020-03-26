#include <iostream>
#include <string>
#include <vector>

#include <boost/property_tree/ptree.hpp>

#include <itkPoint.h>
#include <itkMesh.h>

#include <vtkMatrix4x4.h>
#include <vtkSmartPointer.h>
#include <vtkIterativeClosestPointTransform.h>

#include "Landmark.hpp"
#include "BoneType.hpp"
#include "utils.hpp"

Landmark::Landmark(std::string landmarkName, boost::property_tree::ptree landmarkCoordinates){
	t = landmarkName;
	m_x = landmarkCoordinates.get<double>("x");
	m_y = landmarkCoordinates.get<double>("y");
	m_z = landmarkCoordinates.get<double>("z");
}

Landmark::Landmark(std::string landmarkName, double x, double y, double z): m_x(x),  m_y(y), m_z(z), t(landmarkName) {}

void Landmark::transform(vtkSmartPointer<vtkMatrix4x4> transform){
	double in[4] = {m_x, m_y, m_z, 0};
	double out[4];
	transform->MultiplyPoint(in, out);
	m_x = out[0];
	m_y = out[1];
	m_z = out[2];
}

itk::Point<float, 3> Landmark::getLandmarkPoint(){
	itk::Point<float, 3> point;
	point[0] = m_x;
	point[1] = m_y;
	point[2] = m_z;
	return point;
}

std::string Landmark::getName(){
	return t;
}

double Landmark::getX(){
	return m_x;
}

double Landmark::getY(){
	return m_y;
}

double Landmark::getZ(){
	return m_z;
}

void Landmark::setCoordinates(vnl_vector<double> coordinates){
	m_x = coordinates.get(0);
	m_y = coordinates.get(1);
	m_z = coordinates.get(2);
}

void Landmark::setCoordinates(double* coordinates){
	m_x = coordinates[0];
	m_y = coordinates[1];
	m_z = coordinates[2];
}

void Landmark::setCoordinates(itk::Point<float, 3> coordinates){
	m_x = coordinates[0];
	m_y = coordinates[1];
	m_z = coordinates[2];
}

double Landmark::getDistance(Landmark* l){
	return l->getLandmarkPoint().EuclideanDistanceTo(getLandmarkPoint());
}

Landmark* Landmark::getLandmarkFromMesh(itk::Mesh<float, 3>::Pointer mesh, const std::string& name, int position){
	itk::Point<float, 3> p = mesh->GetPoint(position);
	double x = p.GetVnlVector().get(0);
	double y = p.GetVnlVector().get(1);
	double z = p.GetVnlVector().get(2);
	Landmark* l = new Landmark(name, x, y, z);
	return l;
}

std::map<std::string, Landmark*> Landmark::getLandmarksFromMesh(itk::Mesh<float, 3>::Pointer mesh, BoneType::e t){
	int numPoints = mesh->GetNumberOfPoints();
	std::map<std::string, Landmark*> landmarks;
	//get landmarks from the mesh, it is done bottom up
	if(t == BoneType::radius){
		//extract the last 9 points from the mesh
		landmarks.insert(std::pair<std::string, Landmark*>("PRUL_R", Landmark::getLandmarkFromMesh(mesh, "PRUL_R", --numPoints)));
		landmarks.insert(std::pair<std::string, Landmark*>("DRUL_R", Landmark::getLandmarkFromMesh(mesh, "DRUL_R", --numPoints)));
		landmarks.insert(std::pair<std::string, Landmark*>("DOAC_R", Landmark::getLandmarkFromMesh(mesh, "DOAC_R", --numPoints)));
		landmarks.insert(std::pair<std::string, Landmark*>("POC_R", Landmark::getLandmarkFromMesh(mesh, "POC_R", --numPoints)));
		landmarks.insert(std::pair<std::string, Landmark*>("DOB_R", Landmark::getLandmarkFromMesh(mesh, "DOB_R", --numPoints)));
		landmarks.insert(std::pair<std::string, Landmark*>("AB_R", Landmark::getLandmarkFromMesh(mesh, "AB_R", --numPoints)));
		landmarks.insert(std::pair<std::string, Landmark*>("CBP_R", Landmark::getLandmarkFromMesh(mesh, "CBP_R", --numPoints)));
		landmarks.insert(std::pair<std::string, Landmark*>("CBD_R", Landmark::getLandmarkFromMesh(mesh, "CBD_R", --numPoints)));
		landmarks.insert(std::pair<std::string, Landmark*>("RAF_P", Landmark::getLandmarkFromMesh(mesh, "RAF_P", --numPoints)));
	} else if(t == BoneType::ulna){
		//extract the last 11 points from the mesh
		landmarks.insert(std::pair<std::string, Landmark*>("PRULd_U", Landmark::getLandmarkFromMesh(mesh, "PRULd_U", --numPoints)));
		landmarks.insert(std::pair<std::string, Landmark*>("PRULs_U", Landmark::getLandmarkFromMesh(mesh, "PRULs_U", --numPoints)));
		landmarks.insert(std::pair<std::string, Landmark*>("DRULd_U", Landmark::getLandmarkFromMesh(mesh, "DRULd_U", --numPoints)));
		landmarks.insert(std::pair<std::string, Landmark*>("DRULs_U", Landmark::getLandmarkFromMesh(mesh, "DRULs_U", --numPoints)));
		landmarks.insert(std::pair<std::string, Landmark*>("DOAC_U", Landmark::getLandmarkFromMesh(mesh, "DOAC_U", --numPoints)));
		landmarks.insert(std::pair<std::string, Landmark*>("POC_U", Landmark::getLandmarkFromMesh(mesh, "POC_U", --numPoints)));
		landmarks.insert(std::pair<std::string, Landmark*>("DOB_U", Landmark::getLandmarkFromMesh(mesh, "DOB_U", --numPoints)));
		landmarks.insert(std::pair<std::string, Landmark*>("AB_U", Landmark::getLandmarkFromMesh(mesh, "AB_U", --numPoints)));
		landmarks.insert(std::pair<std::string, Landmark*>("CBP_U", Landmark::getLandmarkFromMesh(mesh, "CBP_U", --numPoints)));
		landmarks.insert(std::pair<std::string, Landmark*>("CBD_U", Landmark::getLandmarkFromMesh(mesh, "CBD_U", --numPoints)));
		landmarks.insert(std::pair<std::string, Landmark*>("RAF_D", Landmark::getLandmarkFromMesh(mesh, "RAF_D", --numPoints)));
	}
	return landmarks;
}

void Landmark::addLandmarkToMesh(itk::Mesh<float, 3>::Pointer mesh, std::map<std::string, Landmark*> landmarks, const std::string& landmarkname, int position){
	Landmark* l = landmarks.find(landmarkname)->second;
	mesh->SetPoint(position, getClosestPointOnMesh(mesh, l->getLandmarkPoint()));
}

void Landmark::addLandmarksToMesh(itk::Mesh<float, 3>::Pointer mesh, std::map<std::string, Landmark*> landmarks, BoneType::e type){
	int numPoints = mesh->GetNumberOfPoints();
	if(type == BoneType::radius){
		//the last 9 points of the mesh will be the landmarks
		addLandmarkToMesh(mesh, landmarks, "RAF_P", numPoints++);
		addLandmarkToMesh(mesh, landmarks, "CBD_R", numPoints++);
		addLandmarkToMesh(mesh, landmarks, "CBP_R", numPoints++);
		addLandmarkToMesh(mesh, landmarks, "AB_R", numPoints++);
		addLandmarkToMesh(mesh, landmarks, "DOB_R", numPoints++);
		addLandmarkToMesh(mesh, landmarks, "POC_R", numPoints++);
		addLandmarkToMesh(mesh, landmarks, "DOAC_R", numPoints++);
		addLandmarkToMesh(mesh, landmarks, "DRUL_R", numPoints++);
		addLandmarkToMesh(mesh, landmarks, "PRUL_R", numPoints++);
	} else if (type == BoneType::ulna){
		//the last 11 points of the mesh will be the landmarks
		addLandmarkToMesh(mesh, landmarks, "RAF_D", numPoints++);
		addLandmarkToMesh(mesh, landmarks, "CBD_U", numPoints++);
		addLandmarkToMesh(mesh, landmarks, "CBP_U", numPoints++);
		addLandmarkToMesh(mesh, landmarks, "AB_U", numPoints++);
		addLandmarkToMesh(mesh, landmarks, "DOB_U", numPoints++);
		addLandmarkToMesh(mesh, landmarks, "POC_U", numPoints++);
		addLandmarkToMesh(mesh, landmarks, "DOAC_U", numPoints++);
		addLandmarkToMesh(mesh, landmarks, "DRULs_U", numPoints++);
		addLandmarkToMesh(mesh, landmarks, "DRULd_U", numPoints++);
		addLandmarkToMesh(mesh, landmarks, "PRULs_U", numPoints++);
		addLandmarkToMesh(mesh, landmarks, "PRULd_U", numPoints++);
	}
}

void Landmark::transformLandmarks(std::map<std::string, Landmark*> landmarks, vtkSmartPointer<vtkIterativeClosestPointTransform> transformation){
	//Create point collection from landmarks
	vtkSmartPointer<vtkPoints> landmarkPoints = vtkSmartPointer<vtkPoints>::New();
	
	for (std::map<std::string, Landmark*>::iterator it = landmarks.begin(); it != landmarks.end(); ++it) {
		landmarkPoints->InsertNextPoint(it->second->getX(), it->second->getY(), it->second->getZ());
	}
	
	//Create poly data with the point collection
	vtkSmartPointer<vtkPolyData> landmarkPoly = vtkSmartPointer<vtkPolyData>::New();
	landmarkPoly->SetPoints(landmarkPoints);
	
	//Transform the poly data with the landmarks in it
	vtkSmartPointer<vtkPolyData> transformedLandmarkPoly = performPolyDataTransform(landmarkPoly, transformation);
	
	//Extract points from poly data
	vtkSmartPointer<vtkPoints> transformedLandmarkPoints = transformedLandmarkPoly->GetPoints();
	
	//write point data back in the landmarks
	int pos = 0;
	for (std::map<std::string, Landmark*>::iterator it = landmarks.begin(); it != landmarks.end(); ++it) {
		double* a = transformedLandmarkPoints->GetPoint(pos++);
		it->second->setCoordinates(a);
	}
}
