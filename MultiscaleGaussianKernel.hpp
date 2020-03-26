#ifndef __MULTI__SCALE_GAUSSIAN_KERNEL__
#define __MULTI__SCALE_GAUSSIAN_KERNEL__

#include <itkMesh.h>
#include <Kernels.h>

class MultiscaleGaussianKernel: public statismo::MatrixValuedKernel< itk::Point<float, 3> > {
  public:
    MultiscaleGaussianKernel(float baseSigma, float baseScale, unsigned nLevels);

    inline statismo::MatrixType operator()(const itk::Point<float, 3>& x, const itk::Point<float, 3>& y) const;

    std::string GetKernelInfo() const;

  private:

    float m_baseSigma;
    float m_baseScale;
    unsigned m_nLevels;
};

#endif
