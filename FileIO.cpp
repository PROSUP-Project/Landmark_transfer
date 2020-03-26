//standard includes
#include <string>
//itk includes
#include <itkMeshFileWriter.h>
#include <itkMeshFileReader.h>
//vtk includes
#include <vtkPolyData.h>
#include <vtkPolyDataReader.h>
#include <vtkSTLReader.h>
#include <vtkDirectory.h>
#include <vtkPolyDataWriter.h>
#include <vtkSmartPointer.h>
//my includes
#include "FileIO.hpp"

//Reads a VTK file and returns the content as vtkPolyData pointer
vtkPolyData* loadVTKPolydataFromVTKFile(const std::string& filename) {
    vtkSmartPointer<vtkPolyDataReader> reader = vtkSmartPointer<vtkPolyDataReader>::New();
    reader->SetFileName(filename.c_str());
    reader->Update();
    vtkPolyData* pd = vtkPolyData::New();
    pd->ShallowCopy(reader->GetOutput());
    return pd;
}

//Reads a STL file and returns the content as vtkPolyData pointer
vtkPolyData* loadVTKPolydataFromSTLFile(const std::string& filename) {
    vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
    reader->SetFileName(filename.c_str());
    reader->Update();
    vtkPolyData* pd = vtkPolyData::New();
    pd->ShallowCopy(reader->GetOutput());
    return pd;
}

//Reads a STL or VTK file and returns the content as vtkPolyData pointer
vtkPolyData* loadVTKPolydata(const std::string& filename) {
	if(filename.find(".vtk") != std::string::npos){
		return loadVTKPolydataFromVTKFile(filename);
	}
	if(filename.find(".stl") != std::string::npos){
		return loadVTKPolydataFromSTLFile(filename);
	}
}

void saveVTKPolydata(vtkPolyData* data, const std::string& filename){
	vtkSmartPointer<vtkPolyDataWriter> writer = vtkSmartPointer<vtkPolyDataWriter>::New();
	writer->SetFileName(filename.c_str());
	writer->SetFileTypeToBinary();
	writer->SetInputData(data);
	writer->Write();
}


//Reads a STL or VTK file and returns the content as vtkPolyData pointer
itk::Mesh<float, 3>::Pointer loadMesh(const std::string& filename){
	itk::MeshFileReader< itk::Mesh<float, 3> >::Pointer reader = itk::MeshFileReader< itk::Mesh<float, 3> >::New();
	reader->SetFileName(filename.c_str());
    reader->Update();
	itk::Mesh<float, 3>::Pointer mesh = reader->GetOutput();
	return mesh;
}

void saveVTKFile(itk::Mesh<float, 3>* data, const std::string& filename){
	itk::MeshFileWriter< itk::Mesh<float, 3> >::Pointer writer = itk::MeshFileWriter< itk::Mesh<float, 3> >::New();
	writer->SetFileName(filename.c_str());
	writer->SetFileTypeAsASCII();
	//writer->SetFileTypeAsBINARY();
	writer->SetInput(data);
	writer->Write();
}
