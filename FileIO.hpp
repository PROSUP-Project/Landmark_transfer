#ifndef __FILE_IO_HPP__
#define __FILE_IO_HPP__
//standard includes
#include <string>
//itk includes
#include <itkMesh.h>
//vtk includes
#include <vtkPolyData.h>

/**
 * Reads a VTK file and returns the content as vtkPolyData pointer.
 * @param path to the file
 * @return pointer to the mesh data
 */
vtkPolyData* loadVTKPolydataFromVTKFile(const std::string& filename);

/**
 * Reads a STL file and returns the content as vtkPolyData pointer.
 * @param path to the file
 * @return pointer to the mesh data
 */
vtkPolyData* loadVTKPolydataFromSTLFile(const std::string& filename);

/**
 * Reads a STL or VTK file and returns the content as vtkPolyData pointer.
 * @param path to the file
 * @return pointer to the mesh data
 */
vtkPolyData* loadVTKPolydata(const std::string& filename);

/**
 * Writes the given vtk poly data into a .vtk file.
 * @param data		vtk poly data that should be written
 * @param filename	path to the file
 */
void saveVTKPolydata(vtkPolyData* data, const std::string& filename);

/**
 * Reads a 3D mesh file and returns the content as itk mesh pointer.
 * @param filename	path to the file
 * @return pointer to the mesh data
 */ 
itk::Mesh<float, 3>::Pointer loadMesh(const std::string& filename);

/**
 * Writes the given itk mesh into a file.
 * @param data		itk mesh that should be written
 * @param filename	path to the file
 */
void saveVTKFile(itk::Mesh<float, 3>* data, const std::string& filename);


#endif