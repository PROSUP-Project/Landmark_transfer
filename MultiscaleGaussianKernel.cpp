#include <string>

#include <itkMesh.h>
#include <Kernels.h>

#include <boost/lexical_cast.hpp>

#include "MultiscaleGaussianKernel.hpp"

MultiscaleGaussianKernel::MultiscaleGaussianKernel(float baseSigma, float baseScale, unsigned nLevels) : m_baseSigma(baseSigma), m_baseScale(baseScale), m_nLevels(nLevels), MatrixValuedKernel<itk::Point<float, 3> >(3) {}

inline statismo::MatrixType MultiscaleGaussianKernel::operator()(const itk::Point<float, 3>& x, const itk::Point<float, 3>& y) const{
	vnl_vector<float> xv = x.GetVnlVector();
	vnl_vector<float> yv = y.GetVnlVector();
	
	vnl_vector<float> r = yv - xv;
	
	const float minusRDotR = -dot_product(r, r);
	
	float kernelValue = 0.;
	for (unsigned i = 1 ; i <= m_nLevels; ++i) {
		const float scaleOnLevel = m_baseScale / static_cast< float >( i );
		const float sigmaOnLevel = m_baseSigma / static_cast< float >( i );
		kernelValue += scaleOnLevel * std::exp( minusRDotR / (sigmaOnLevel * sigmaOnLevel));
	}
	return statismo::MatrixType::Identity(3,3) * kernelValue;
}

std::string MultiscaleGaussianKernel::GetKernelInfo() const{
	return "MultiscaleGaussianKernel(sigma:" + boost::lexical_cast<std::string>(m_baseSigma) + ", scale:" + boost::lexical_cast<std::string>(m_baseScale) + ", levels:" + boost::lexical_cast<std::string>(m_nLevels) + ")";
}