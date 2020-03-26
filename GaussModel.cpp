#include <boost/ptr_container/ptr_vector.hpp>

#include <Kernels.h>
#include <KernelCombinators.h>
#include <statismoITKConfig.h>
#include <StatisticalModel.h>
#include <PCAModelBuilder.h>

#include <itkStatisticalModel.h>
#include <itkMesh.h>
#include <itkLowRankGPModelBuilder.h>
#include <itkStandardMeshRepresenter.h>

#include "GaussModel.hpp"
#include "FileIO.hpp"
#include "Bone.hpp"
#include "GaussianKernel.hpp"
#include "MultiscaleGaussianKernel.hpp"

itk::StatisticalModel< itk::Mesh<float, 3> >::Pointer GaussModel::generateGaussModel(itk::StandardMeshRepresenter<float, 3>::Pointer representer, double gaussianKernelSigma, double gaussianKernelSigma2, double relation, double kernelScale, unsigned numComponents){
	//Generate Gaussian kernel with the given sigma	
	GaussianKernel gk = GaussianKernel(gaussianKernelSigma);
	
	//generate matrix kernel from the Gaussian kernel	
    const statismo::MatrixValuedKernel< itk::Point<float, 3> >& mvGk = statismo::UncorrelatedMatrixValuedKernel< itk::Point<float, 3> >(&gk, representer->GetDimensions());
	
	//scale the kernel by the given value
    const statismo::MatrixValuedKernel< itk::Point<float, 3> >& scaledGk = statismo::ScaledKernel< itk::Point<float, 3> >(&mvGk, kernelScale);
	
	//Generate a second Gaussian kernel with the given sigma	
	GaussianKernel gk2 = GaussianKernel(gaussianKernelSigma2);
	
	//generate matrix kernel from the second Gaussian kernel	
    const statismo::MatrixValuedKernel< itk::Point<float, 3> >& mvGk2 = statismo::UncorrelatedMatrixValuedKernel< itk::Point<float, 3> >(&gk2, representer->GetDimensions());
	
	//scale the second kernel by the given value
    const statismo::MatrixValuedKernel< itk::Point<float, 3> >& scaledGk2 = statismo::ScaledKernel< itk::Point<float, 3> >(&mvGk2, kernelScale*relation);
	
	//combine the kernels
	const statismo::MatrixValuedKernel< itk::Point<float, 3> >& kernel = statismo::SumKernel< itk::Point<float, 3> >(&scaledGk, &scaledGk2);
	
	//generate the Gaussian process model
	itk::LowRankGPModelBuilder< itk::Mesh<float, 3> >::Pointer builder = itk::LowRankGPModelBuilder< itk::Mesh<float, 3> >::New();
	builder->SetRepresenter(representer);
	itk::StatisticalModel< itk::Mesh<float, 3> >::Pointer model = builder->BuildNewZeroMeanModel(kernel, numComponents);	
	return model;
}

itk::StatisticalModel< itk::Mesh<float, 3> >::Pointer GaussModel::generateMultiLevelGaussModel(itk::StandardMeshRepresenter<float, 3>::Pointer representer, double gaussianKernelSigma, double kernelScale, unsigned levels, unsigned numComponents){
	//the kernels
	const statismo::MatrixValuedKernel< itk::Point<float, 3> >& kernel = MultiscaleGaussianKernel(gaussianKernelSigma, kernelScale, levels);
	
	//generate the Gaussian process model
	itk::LowRankGPModelBuilder< itk::Mesh<float, 3> >::Pointer builder = itk::LowRankGPModelBuilder< itk::Mesh<float, 3> >::New();
	builder->SetRepresenter(representer);
	itk::StatisticalModel< itk::Mesh<float, 3> >::Pointer model = builder->BuildNewZeroMeanModel(kernel, numComponents);	
	return model;
}

itk::StatisticalModel< itk::Mesh<float, 3> >::Pointer GaussModel::generateImprovedAtlas(statismo::DataManager< itk::Mesh<float, 3> >* datamanager, double sigma, double scale, int numComponents){
	//build the statistical model
	statismo::PCAModelBuilder< itk::Mesh<float, 3> >* modelBuilder = statismo::PCAModelBuilder< itk::Mesh<float, 3> >::Create();
	statismo::StatisticalModel< itk::Mesh<float, 3> >* model;
	model = modelBuilder->BuildNewModel(datamanager->GetData(), 0);

	//Generate Gaussian kernel with the given sigma	
	GaussianKernel gk = GaussianKernel(sigma);
	
	//generate matrix kernel from the Gaussian kernel	
    const statismo::MatrixValuedKernel< itk::Point<float, 3> >& mvGk = statismo::UncorrelatedMatrixValuedKernel< itk::Point<float, 3> >(&gk, model->GetRepresenter()->GetDimensions());
	
	//scale the kernel by the given value
    const statismo::MatrixValuedKernel< itk::Point<float, 3> >& scaledGk = statismo::ScaledKernel< itk::Point<float, 3> >(&mvGk, scale);
	
	//kernel of the statistical model
	const statismo::MatrixValuedKernel< itk::Point<float, 3> >& statisticalKernel = statismo::StatisticalModelKernel< itk::Mesh<float, 3> >(model);
	
	//combine the kernels
	const statismo::MatrixValuedKernel< itk::Point<float, 3> >& improvedKernel = statismo::SumKernel< itk::Point<float, 3> >(&scaledGk, &statisticalKernel);
	
	//generate the Gaussian process model
	itk::LowRankGPModelBuilder< itk::Mesh<float, 3> >::Pointer builder = itk::LowRankGPModelBuilder< itk::Mesh<float, 3> >::New();
	builder->SetRepresenter(model->GetRepresenter());
	itk::StatisticalModel< itk::Mesh<float, 3> >::Pointer improvedModel = builder->BuildNewZeroMeanModel(improvedKernel, numComponents);
	
	delete modelBuilder;
	delete model;
	
	return improvedModel;
}
