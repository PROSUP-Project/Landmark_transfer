#ifndef __SINGLE_BONE_TYPE_MANAGER_HPP__
#define __SINGLE_BONE_TYPE_MANAGER_HPP__

#include <vector>

#include <itkMesh.h>
#include <itkStandardMeshRepresenter.h>
#include <itkStatisticalModel.h>

#include <DataManager.h>
#include <StatisticalModel.h>

#include "Bone.hpp"

class SingleBoneTypeManager{
	public:
		/**
		 * Generates a manager for a single bone type.
		 */
		SingleBoneTypeManager();
		
		/**
		 * Frees all bones in the vector.
		 */
		~SingleBoneTypeManager();
		
		/**
		 * Adds the given bone to the managed bones.
		 * @param b		the bone to add
		 */
		void addBone(Bone* b);
		
		/**
		 * Returns the vector containing all handled bones.
		 * @return all ulnae of the left side
		 */
		std::vector<Bone*> getBones();
		
		/**
		 * Returns the bone with the biggest diagonal of the bounding box.
		 * @return the longest ulna from the left side
		 */
		Bone* get_longest_bone();
		
		/**
		 * Aligns all bones managed by this manager.
		 * @param rigit_iter	maximal number of iterations for the alignment
		 */
		void align_bones(int rigit_iter);
		
		/**
		 * Aligns all bones managed by this manager with the representer of the given model.
		 * @param rigit_iter	maximal number of iterations for the alignment
		 */
		void align_bones(itk::StatisticalModel< itk::Mesh<float, 3> >::Pointer model, int rigit_iter);
		
		/**
		 * Helper function to call the 'align_bones' function in parallel.
		 * @param sbtm			pointer to the single bone type manager which function should be called
		 * @param rigit_iter	maximal number of iterations for the alignment
		 */
		static void multicore_align_bones(SingleBoneTypeManager* sbtm, int rigit_iter);
		
		/**
		 * Return a Gaussian process model of the longest handled bone.
		 * @param sigma1	sigma of the Gaussian kernel
		 * @param sigma2	sigma of the second Gaussian kernel
		 * @param relation	factor that deters the scale of the second kernel in relation to the first
		 * @param scale		scale of the kernel first kernel
		 * @param num_comp	number of parameters the model should have
		 * @return Gaussian process model
		 */
		itk::StatisticalModel< itk::Mesh<float, 3> >::Pointer getGPModel(double sigma1, double sigma2, double relation, int scale, int num_comp);
		
		/**
		 * Return a multi level Gaussian process model of the longest handled bone.
		 * @param sigma		sigma of the Gaussian kernel
		 * @param levels	number of levels of the Gaussian kernel
		 * @param num_comp	number of parameters the model should have
		 * @return Gaussian process model
		 */
		itk::StatisticalModel< itk::Mesh<float, 3> >::Pointer getMultiLevelGPModel(double sigma1, int scale, int levels, int num_comp);
		
		/**
		 * Generates a statismo data manager for the bone meshes with landmarks.
		 * @return pointer to the data manager, had to freed manually
		 */
		statismo::DataManager< itk::Mesh<float, 3> >* getDatamanager();
		
		/**
		 * The Gaussian process model is used to generate correspondent meshes for each bone managed by this manager,
		 * this meshes will then be extended by the landmark data and the resulting mesh is added to the given data manager.
		 * @param datamanager		data manager where the data should be added
		 * @param gaussModel		the Gaussian process model for the fitting
		 * @param gauss_fit_iter	maximal number of iterations for the fitting
		 * @param volume_percentage	the minimal percentage of volume that the corresponding mash has to have in relation to the original mesh
		 */
		void addCorrespondentDataToDataManager(statismo::DataManager< itk::Mesh<float, 3> >* datamanager, itk::StatisticalModel< itk::Mesh<float, 3> >::Pointer gaussModel, int gauss_fit_iter, float volume_percentage);
		
		/**
		 * The Gaussian process model is used to generate correspondent meshes for each bone managed by this manager,
		 * this meshes will then be extended by the landmark data and the resulting mesh is added to the given data manager.
		 * In addition this method will constrain the given model with the landmark coordinates, for this the variance of the landmark 
		 * position is required.
		 * @param datamanager		data manager where the data should be added
		 * @param gaussModel		the Gaussian process model for the fitting
		 * @param gauss_fit_iter	maximal number of iterations for the fitting
		 * @param volume_percentage	the minimal percentage of volume that the corresponding mash has to have in relation to the original mesh
		 * @param variance			the variance of the possible error of the landmark positions, has to be greater than 0
		 */
		void addCorrespondentDataToDataManager(statismo::DataManager< itk::Mesh<float, 3> >* datamanager, itk::StatisticalModel< itk::Mesh<float, 3> >::Pointer gaussModel, int gauss_fit_iter, float volume_percentage, double variance);
	
	private:
		
		std::vector<Bone*> bones;
};

#endif
