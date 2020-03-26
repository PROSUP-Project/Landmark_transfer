#ifndef __GAUSSIAN_KERNEL__HPP
#define __GAUSSIAN_KERNEL__HPP

#include <itkMesh.h>
#include <Kernels.h>

class GaussianKernel: public statismo::ScalarValuedKernel< itk::Point<float, 3> > {
  public:
    GaussianKernel(double sigma);

    inline double operator()(const itk::Point<float, 3>& x, const itk::Point<float, 3>& y) const;

    std::string GetKernelInfo() const;

  private:
    double m_sigma;
    double m_sigma2;
};
#endif
