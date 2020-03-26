#ifndef __MY_UTILS_HPP__
#define __MY_UTILS_HPP__

#include <vnl/vnl_vector.h>

#include <itkMesh.h>
#include <itkImage.h>
#include <itkStandardMeshRepresenter.h>
#include <itkStatisticalModel.h>

#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkIterativeClosestPointTransform.h>

/**
 * Returns a deep copy of the given mesh.
 * As itk has no function to deep copy a mesh, it is done by applying a transformation with the identity matrix on the mesh.
 * @param mesh	the mesh that should be copied
 * @return a deep copy of the given mesh
 */
itk::Mesh<float, 3>::Pointer cloneMesh(itk::Mesh<float, 3>* mesh);

/**
 * Generates a distance image of the given mesh.
 * @param mesh	the mesh from which a distance image should be created
 * @return the distance image of the mesh
 */
itk::Image<float, 3>::Pointer distanceImageFromMesh(itk::Mesh<float, 3>* mesh);

/**
 * Calculates the closest point to a given point on a triangle.
 * @param t0		first vertex of the triangle
 * @param t1		second vertex of the triangle
 * @param t2		third vertex of the triangle
 * @param point		the point which nearest point should be found
 * @return	the nearest point on the triangle
 */
vnl_vector<double> closesPointOnTriangle(vnl_vector<double>& t0, vnl_vector<double>& t1, vnl_vector<double>& t2, vnl_vector<double>& point);

/**
 * Clamps the given value between the given minimum and maximum.
 * @param a		the value that should be clamped
 * @param min	the minimal value
 * @param max	the maximal value
 * @return min if a < min, max if a > max, else a
 */
double clamp(double a, double min = 0.0, double max = 1.0);

/**
 * Creates a double vector from a float vector.
 * @param src	float vector
 * @return a new vector containing the same values as the input but as double values
 */
vnl_vector<double> toDoubleVector(const vnl_vector<float>& src);

/**
 * Transforms the given vtk mesh into a itk mesh.
 * For this the mesh is saved in a temporary file and loaded back in the other format.
 * @warning not thread safe with no given path or identical paths
 * @param input		the mesh which type should be changed
 * @param path		optional path of the file, if set the file will be kept, if not set the path will be "/tmp/temp.vtk" and after the transformation the file will be deleted
 * @return the transformed mesh
 */
itk::Mesh<float, 3>::Pointer itkFromVtk(vtkPolyData* input, const std::string& path = "");

/**
 * Transforms the given itk mesh into a vtk mesh.
 * For this the mesh is saved in a temporary file and loaded back in the other format.
 * @warning not thread safe with no given path or identical paths
 * @param input		the mesh which type should be changed
 * @param path		optional path of the file, if set the file will be kept, if not set the path will be "/tmp/temp.vtk" and after the transformation the file will be deleted
 * @return the transformed mesh
 */
vtkSmartPointer<vtkPolyData> vtkFromItk(itk::Mesh<float, 3>::Pointer input, const std::string& path = "");

/**
 * Projects the given point on the surface of the given mesh and returns the projected point.
 * @param mesh		the mesh on which the point should be projected
 * @param point		the point that should be projected on the surface of the mesh
 * @return	a new point that represents the given point projected on the surface of the given mesh
 */
itk::Point<float, 3> getClosestPointOnMesh(itk::Mesh<float, 3>::Pointer mesh, itk::Point<float, 3> point);

/**
 * Fits the given model to the given mesh and returns the fitting mesh.
 * @param model			the statistical model that should be fitted
 * @param mesh			the mesh to which the model should be fitted
 * @param iterations	maximal number of iterations for the fitting
 * @return the fitted mesh 
 */
itk::Mesh<float, 3>::Pointer fitModelToMesh(itk::StatisticalModel<itk::Mesh<float, 3> >::Pointer model, itk::Mesh<float, 3>::Pointer mesh, int iterations);

/**
 * Performs the given transformation on the given poly data and returns the result.
 * @param data				data which should be transformed
 * @param transformation	transformation that should be applied
 * @return data after the transformation
 */
vtkSmartPointer<vtkPolyData> performPolyDataTransform(vtkSmartPointer<vtkPolyData> data, vtkSmartPointer<vtkIterativeClosestPointTransform> transformation);

/**
 * Generates an iterative closest point transformation from the source to the target.
 * @param source		source of the transformation
 * @param target		target of the transformation
 * @param iterations	maximal number of iterations
 * @return transformation from the source to the target
 */
vtkSmartPointer<vtkIterativeClosestPointTransform> getRigidTransformation(vtkSmartPointer<vtkPolyData> source, vtkSmartPointer<vtkPolyData> target, int iterations);

/**
 * Calculates the volume of the given mesh.
 * @param mesh	the mesh which volume should be calculated
 * @return the volume of the mesh
 */
double calculateVolume(itk::Mesh<float, 3>::Pointer mesh);
#endif
