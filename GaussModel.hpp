#ifndef __GAUSS_MODEL_HPP__
#define __GAUSS_MODEL_HPP__

#include <itkMesh.h>
#include <itkStandardMeshRepresenter.h>
#include <itkStatisticalModel.h>

#include <DataManager.h>
#include <Kernels.h>

class GaussModel{
	public:
		/**
		 * Generates a Gaussian kernel and with this kernel a Gaussian process model for the given representer.
		 * @param representer			the representer for the model
		 * @param gaussianKernelSigma	sigma of the Gaussian kernel
		 * @param gaussianKernelSigma2	sigma of the second Gaussian kernel
		 * @param relation				factor that deters the scale of the second kernel in relation to the first
		 * @param kernelScale			scale of the kernel first kernel
		 * @param numComponents			the number of parameters the model should have
		 */
		static itk::StatisticalModel< itk::Mesh<float, 3> >::Pointer generateGaussModel(itk::StandardMeshRepresenter<float, 3>::Pointer representer, double gaussianKernelSigma, double gaussianKernelSigma2, double relation, double kernelScale, unsigned numComponents);
		
		/**
		 * Generates a multi level Gaussian kernel and with this kernel a Gaussian process model for the given representer.
		 * @param representer			the representer for the model
		 * @param gaussianKernelSigma	sigma of the Gaussian kernel
		 * @param levels				number of levels, the scale and sigma of each level is the base value divided by the level index
		 * @param kernelScale			scale of the kernel first kernel
		 * @param numComponents			the number of parameters the model should have
		 */
		static itk::StatisticalModel< itk::Mesh<float, 3> >::Pointer generateMultiLevelGaussModel(itk::StandardMeshRepresenter<float, 3>::Pointer representer, double gaussianKernelSigma, double kernelScale, unsigned levels, unsigned numComponents);
		
		/**
		 * Adds a Gaussian kernel to the given statistical model to increase the flexibility of the model.
		 * This increases the possibility to fit the statistical model to other meshes, even when the number of 
		 * samples from which the statistical model is generated is small.
		 * @param model		the statistical model that should be improved
		 * @param sigma		sigma of the Gaussian kernel that should be added
		 * @param scale
		 * @return
		 */
		static itk::StatisticalModel< itk::Mesh<float, 3> >::Pointer generateImprovedAtlas(statismo::DataManager< itk::Mesh<float, 3> >* datamanager, double sigma, double scale, int numComponents);
};
#endif
