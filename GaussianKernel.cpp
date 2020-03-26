#include<string>

#include <itkMesh.h>
#include <Kernels.h>

#include <boost/lexical_cast.hpp>

#include "GaussianKernel.hpp"

GaussianKernel::GaussianKernel(double sigma) : m_sigma(sigma), m_sigma2(sigma * sigma) {}

inline double GaussianKernel::operator()(const itk::Point<float, 3>& x, const itk::Point<float, 3>& y) const {
	vnl_vector<float> xv = x.GetVnlVector();
	vnl_vector<float> yv = y.GetVnlVector();
	
	vnl_vector<float> r = yv - xv;
	return exp(-dot_product(r, r) / m_sigma2);
}

std::string GaussianKernel::GetKernelInfo() const {
	return "GaussianKernel(" + boost::lexical_cast<std::string>(m_sigma) + ")";
}
