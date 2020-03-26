#ifndef __BONE_HPP__
#define __BONE_HPP__

#include <string>
#include <map>

#include <boost/atomic/atomic.hpp>
#include <boost/thread.hpp>
#include <boost/thread/future.hpp>

#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkIterativeClosestPointTransform.h>

#include <itkMesh.h>
#include <itkStandardMeshRepresenter.h>
#include <itkStatisticalModel.h>

#include <DataManager.h>

#include "Landmark.hpp"
#include "BoneType.hpp"

/**
 * Class that represents a single bone.
 */
class Bone{
	public:
		/**
		 * Creates a new bone from the given configuration.
		 * @param boneConfig		bone configuration, containing the landmark data
		 * @param output_folder		path to a directory where changed models of the bone should be saved
		 * @param t					type of the bone
		 */
		Bone(boost::property_tree::ptree boneConfig, const std::string& output_folder, BoneType::e t);
		
		/**
		 * Frees the memory of the aligned data buffer and all landmarks of the bone.
		 */
		~Bone();
		
		/**
		 * Returns the type of the bone.
		 * @return the bone type
		 */
		BoneType::e getType();
		
		/**
		 * Returns a string that represents the path of the output folder.
		 * @return output path as string
		 */
		std::string getOutputFolder();
		
		/**
		 * Returns the length of the bounding box.
		 * @return the length of the bounding box
		 */
		double getLength();
		
		/**
		 * Performs a rigid transformation on the bone to be aligned with the given bone.
		 * The transformation will also be applied on all landmarks.
		 * Will generate a VTK file with the new aligned mesh, the file path is stored in this object
		 * to access the file.
		 * @param b				target bone
		 * @param iterations	maximal number of iterations in the alignment
		 */
		void align_with(Bone* b, int iterations);
		
		/**
		 * Performs a rigid transformation on the bone to be aligned with the given mesh.
		 * The transformation will also be applied on all landmarks.
		 * Will generate a VTK file with the new aligned mesh, the file path is stored in this object
		 * to access the file.
		 * 
		 * @param b				target bone
		 * @param iterations	maximal number of iterations in the alignment
		 */
		void align_with(vtkPolyData* b, int iterations);
		
		/**
		 * Returns the not aligned input data of the bone.
		 * This data will be buffered in memory to avoid multiple file access.
		 * @return vtkPolyData of the original input data
		 */
		vtkPolyData* get_not_aligned_data();
		
		/**
		 * Frees the memory used to store the original input data.
		 */
		void free_not_aligned_data();
		
		/**
		 * Reads the aligned mesh from the file generated from the 'align_with' function.
		 * Requires that the bone was aligned before.
		 * @return an itkMesh containing the aligned bone data
		 */
		itk::Mesh<float, 3>::Pointer get_aligned_data();
		
		/**
		 * Fits the given Gaussian process model to match this bone, this will result in a 
		 * corresponding mesh that is saved to a file, the mesh and the file path to this 
		 * is added to the given data manager.
		 * Requires that the bone was aligned before.
		 * @param datamanager	data manager where the corresponding mesh should be added
		 * @param gaussModel	the model that should be fitted to this bone
		 * @param iterations	maximal number of iterations in the fitting process
		 * @param volume_percentage	the minimal percentage of volume that the corresponding mash has to have in relation to the original mesh
		 */
		void correspond(statismo::DataManager< itk::Mesh<float, 3> >* datamanager, itk::StatisticalModel< itk::Mesh<float, 3> >::Pointer gaussModel, int iterations, float volume_percentage);
		
		/**
		 * Fits the given Gaussian process model to match this bone, this will result in a 
		 * corresponding mesh that is saved to a file, the mesh and the file path to this 
		 * is added to the given data manager.
		 * In addition this method will constrain the given model with the landmark coordinates, for this the variance of the landmark 
		 * position is required.
		 * Requires that the bone was aligned before.
		 * @param datamanager	data manager where the corresponding mesh should be added
		 * @param gaussModel	the model that should be fitted to this bone
		 * @param iterations	maximal number of iterations in the fitting process
		 * @param volume_percentage	the minimal percentage of volume that the corresponding mash has to have in relation to the original mesh
		 * @param variance			the variance of the possible error of the landmark positions, has to be greater than 0
		 */
		void correspond_with_constrains(statismo::DataManager< itk::Mesh<float, 3> >* datamanager, itk::StatisticalModel< itk::Mesh<float, 3> >::Pointer gaussModel, int iterations, float volume_percentage, double variance);
		
		/**
		 * Helper function required to call the 'correspond' function of multiple bones in threads.
		 * @param b				bone which 'correspond' function should be called
		 * @param datamanager	data manager where the corresponding mesh should be added
		 * @param gaussModel	Gaussian process model that should be fitted on the given bone
		 * @param iterations	maximal number of iterations of the fitting step
		 * @param volume_percentage	the minimal percentage of volume that the corresponding mash has to have in relation to the original mesh
		 */
		static void multicore_correspond(Bone* b, statismo::DataManager< itk::Mesh<float, 3> >* datamanager, itk::StatisticalModel< itk::Mesh<float, 3> >::Pointer gaussModel, int iterations, float volume_percentage);
		
		/**
		 * Helper function required to call the 'correspond_with_constrains' function of multiple bones in threads.
		 * In addition this method will constrain the given model with the landmark coordinates, for this the variance of the landmark 
		 * position is required.
		 * @param b				bone which 'correspond' function should be called
		 * @param datamanager	data manager where the corresponding mesh should be added
		 * @param gaussModel	Gaussian process model that should be fitted on the given bone
		 * @param iterations	maximal number of iterations of the fitting step
		 * @param volume_percentage	the minimal percentage of volume that the corresponding mash has to have in relation to the original mesh
		 * @param variance			the variance of the possible error of the landmark positions, has to be greater than 0
		 */
		static void multicore_correspond_with_constrains(Bone* b, statismo::DataManager< itk::Mesh<float, 3> >* datamanager, itk::StatisticalModel< itk::Mesh<float, 3> >::Pointer gaussModel, int iterations, float volume_percentage, double variance);
		
		/**
		 * Generates a Gaussian process model of this bone.
		 * @param sigma1	sigma of the Gaussian kernel
		 * @param sigma2	sigma of the second Gaussian kernel
		 * @param relation	factor that deters the scale of the second kernel in relation to the first
		 * @param scale		scale of the kernel first kernel
		 * @param num_comp	number of parameters the model should have
		 * @return a two stage Gaussian process model of this bone
		 */
		itk::StatisticalModel< itk::Mesh<float, 3> >::Pointer getGaussianProcessModel(double sigma1, double sigma2, double relation, int scale, int num_comp);		
		
		/**
		 * Generates a multi level Gaussian process model of this bone.
		 * @param sigma		sigma of the Gaussian kernel
		 * @param levels	number of levels of the Gaussian kernel
		 * @param scale		scale of the kernel first kernel
		 * @param num_comp	number of parameters the model should have
		 * @return a multi level Gaussian process model of this bone
		 */
		itk::StatisticalModel< itk::Mesh<float, 3> >::Pointer getMultiLevelGaussianProcessModel(double sigma1, int scale, int levels, int num_comp);
		
		/**
		 * Appends the given landmarks of the bone to the given mesh.
		 * @param mesh	 where the landmarks should be added
		 */
		void addLandmarksToMesh(itk::Mesh<float, 3>::Pointer mesh);
		
	private:
		/**
		 * Fits the given Gaussian process model to match this bone.
		 * @param gaussModel	the model that should be fitted to this bone
		 * @param iterations	maximal number of iterations in the fitting process
		 * @return a corresponding mesh to the mesh of this bone, created with the given model
		 */
		itk::Mesh<float, 3>::Pointer generateCorrespondingMesh(itk::StatisticalModel< itk::Mesh<float, 3> >::Pointer gaussModel, int iterations);
		
		/**
		 * Second version of 'generateCorrespondingMesh'
		 * In addition the known landmarks are used to improve the fitting
		 * @param variance the landmark variance
		 * @warning requires that the gaussModel contains the landmark points and that the model has the same bone type as this bone
		 * @see generateCorrespondingMesh
		 */
		itk::Mesh<float, 3>::Pointer generateCorrespondingMeshWithConstraints(itk::StatisticalModel< itk::Mesh<float, 3> >::Pointer gaussModel, int iterations, double variance);
		
		/**
		 * Applies the given transformation on the landmarks of the bone.
		 * This is done as alternative to perform a matrix transformation,
		 * because the GetMatrix() function gave me a wrong matrix
		 * @param icp	the transformation that should be applied on the landmarks
		 */
		void transformLandmarks(vtkSmartPointer<vtkIterativeClosestPointTransform> icp);
		
		/**
		 * Returns the constraints given by the landmarks of this bone and the landmarks contained in the given model.
		 * @param gaussModel	the model, has to contain the landmarks
		 * @return the landmark constraints
		 */
		itk::StatisticalModel<itk::Mesh<float, 3> >::PointValueListType getLandmarkConstraints(itk::StatisticalModel< itk::Mesh<float, 3> >::Pointer gaussModel, itk::Mesh<float, 3>::Pointer mesh);
		
		/**
		 * Returns a model that is constrained so that the landmark points from the model match the landmark points of this bone.
		 * @param gaussModel	the model that should be constrained, has to contain the landmarks
		 * @param variance		the variance of the landmark points
		 * @return the constrained model
		 */
		itk::StatisticalModel<itk::Mesh<float, 3> >::Pointer getPosteriorModel(itk::StatisticalModel< itk::Mesh<float, 3> >::Pointer gaussModel, itk::Mesh<float, 3>::Pointer mesh, double variance);
		
		/**
		 * Checks if the distance of all points from the mesh of this bone and the given mesh
		 * is smaller than the given maximal distance.
		 * As finding the nearest point on a mesh is not a bijective function it is important that it is
		 * done for each point of the bone mesh and not for the points on the given mesh.
		 * @param mesh			the mesh which points should be checked
		 * @param max_distance	the maximal valid distance
		 * @return true if no points are further than the given distance from the given mesh
		 */
		bool validateFittingWithDistance(itk::Mesh<float, 3>::Pointer mesh, float max_distance);
		
		/**
		 * Validates the fitting with the volume.
		 * @param mesh				the mesh which volume should be checked
		 * @param min_percentage	the maximal valid distance
		 * @return true if the volume of the given mesh has a volume of at least 'min_percentage' of the bone volume
		 */
		bool validateFittingWithVolume(itk::Mesh<float, 3>::Pointer mesh, float min_percentage);
		
		/**
		 * Returns the map containing the landmarks.
		 * @return map containing the landmarks
		 */
		std::map<std::string, Landmark*> getLandmarks();
		
		//path of the input file
		std::string not_aligned_filename;
		//path of the aligned mesh
		std::string aligned_filename;
		//path of the corresponding mesh
		std::string corresponding_filename;
		//path of the output folder
		std::string out_folder;
		
		//the type of the bone
		BoneType::e type;
		
		//length of the bounding box
		double length;
		
		//map containing all landmarks
		std::map<std::string, Landmark*> landmarks;
		
		//buffer for the aligned mesh
		vtkPolyData* not_aligned_data;
		
		//static number to give each bone a new number, this number is used in the output file names
		static boost::atomic<int> name;
};

#endif
