#include <itkMesh.h>
#include <itkImage.h>
#include <itkPointSetToImageFilter.h>
#include <itkDanielssonDistanceMapImageFilter.h>
#include <itkTransformMeshFilter.h>
#include <itkIdentityTransform.h>
#include <itkPointsLocator.h>
#include <itkStandardMeshRepresenter.h>
#include <itkLinearInterpolateImageFunction.h>
#include <itkStatisticalShapeModelTransform.h>
#include <itkStatisticalModel.h>
#include <itkMeanSquaresPointSetToImageMetric.h>
#include <itkLBFGSBOptimizer.h>
#include <itkPointSetToImageRegistrationMethod.h>
#include <itkTransformMeshFilter.h>
#include <itkSimplexMesh.h>
#include <itkTriangleMeshToSimplexMeshFilter.h>
#include <itkSimplexMeshVolumeCalculator.h>

#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkIterativeClosestPointTransform.h>
#include <vtkLandmarkTransform.h>
#include <vtkTransformPolyDataFilter.h>

#include <boost/filesystem.hpp>

#include <vnl/vnl_vector.h>

#include "utils.hpp"
#include "FileIO.hpp"

//itk has no clone mesh, so a transformation with the identity matrix is used, maybe replace with memory copy to be faster
itk::Mesh<float, 3>::Pointer cloneMesh(itk::Mesh<float, 3>* mesh) {
	itk::TransformMeshFilter<itk::Mesh<float, 3>, itk::Mesh<float, 3>, itk::IdentityTransform<float, itk::Mesh<float, 3>::PointDimension> >::Pointer transformMeshFilter = itk::TransformMeshFilter<itk::Mesh<float, 3>, itk::Mesh<float, 3>, itk::IdentityTransform<float, itk::Mesh<float, 3>::PointDimension> >::New();
	itk::IdentityTransform<float, itk::Mesh<float, 3>::PointDimension>::Pointer id = itk::IdentityTransform<float, itk::Mesh<float, 3>::PointDimension>::New();
	transformMeshFilter->SetInput(mesh);
	transformMeshFilter->SetTransform(id);
	transformMeshFilter->Update();
	return transformMeshFilter->GetOutput();
}

//from statismo examples
itk::Image<float, 3 >::Pointer distanceImageFromMesh(itk::Mesh<float, 3>* mesh) {
	// Compute a bounding box around the reference shape
    itk::BoundingBox<int, 3, float, itk::Mesh<float, 3>::PointsContainer>::Pointer bb = 
		itk::BoundingBox<int, 3, float, itk::Mesh<float, 3>::PointsContainer>::New();
    bb->SetPoints(mesh->GetPoints());
    bb->ComputeBoundingBox();

    // Compute a binary image from the point set, which is as large as the bounding box plus a margin.
    itk::PointSetToImageFilter<itk::PointSet<float, 3>,itk::Image< unsigned char, 3> >::Pointer pointsToImageFilter =
		itk::PointSetToImageFilter<itk::PointSet<float, 3>,itk::Image< unsigned char, 3> >::New();
    pointsToImageFilter->SetInput(mesh);
    itk::Image< unsigned char, 3 >::SpacingType spacing; 
    itk::Image< unsigned char, 3 >::SpacingType margin; 
    itk::Image< unsigned char, 3 >::PointType origin = bb->GetMinimum();
    itk::Image< unsigned char, 3 >::SpacingType diff = bb->GetMaximum() - bb->GetMinimum();    
    itk::Image< unsigned char, 3 >::SizeType size;
    
    for (unsigned i =0; i < 3; ++i) {
        //margin[i] = diff[i] * 0.1; // 10 % margin on each side
        margin[i] = diff[i] * 1;
        origin[i] -= margin[i];
        size[i] = 256 + margin[i];
        spacing[i] = (diff[i] + 2.0 * margin[i]) / 256;
    }
	
    pointsToImageFilter->SetSpacing(spacing);
    pointsToImageFilter->SetOrigin(origin);
    pointsToImageFilter->SetSize(size);
    pointsToImageFilter->Update();

    // compute a distance map to the points in the point set
    itk::Image< unsigned char, 3 >::Pointer binaryImage = pointsToImageFilter->GetOutput();
    itk::DanielssonDistanceMapImageFilter<itk::Image< unsigned char, 3 >, itk::Image<float, 3> >::Pointer distanceFilter = 
		itk::DanielssonDistanceMapImageFilter<itk::Image< unsigned char, 3 >, itk::Image<float, 3> >::New();
    distanceFilter->SetInput(binaryImage);
    distanceFilter->Update();
    itk::Image<float, 3>::Pointer distanceImage = distanceFilter->GetOutput();
    return distanceImage;
}

double clamp(double a, double min, double max){
	if(a < min)
		return min;
	if(a > max)
		return max;
	return a;
}

vnl_vector<double> closesPointOnTriangle(vnl_vector<double>& t0, vnl_vector<double>& t1, vnl_vector<double>& t2, vnl_vector<double>& point){
    vnl_vector<double> edge0 = t1 - t0;
    vnl_vector<double> edge1 = t2 - t0;
    vnl_vector<double> v0 = t0 - point;

    double a = dot_product(edge0, edge0);
    double b = dot_product(edge0, edge1);
    double c = dot_product(edge1, edge1);
    double d = dot_product(edge0, v0);
    double e = dot_product(edge1, v0);

    double det = a*c - b*b;
    double s = b*e - c*d;
    double t = b*d - a*e;

    if(s + t < det){
        if(s < 0.0){
            if(t < 0.0){
                if(d < 0.0){
                    s = clamp(-d/a);
                    t = 0.0;
                }else{
                    s = 0.0;
                    t = clamp(-e/c);
                }
            }else{
                s = 0.0;
                t = clamp(-e/c);
            }
        }else if(t < 0.0){
            s = clamp(-d/a);
            t = 0.0;
        }else{
            double invDet = 1.0 / det;
            s *= invDet;
            t *= invDet;
        }
    }else{
        if(s < 0.0){
            double tmp0 = b+d;
            double tmp1 = c+e;
            if(tmp1 > tmp0){
                double numer = tmp1 - tmp0;
                double denom = a-2*b+c;
                s = clamp(numer/denom);
                t = 1-s;
            }else{
                t = clamp(-e/c);
                s = 0.0;
            }
        }else if(t < 0.0){
            if(a+d > b+e){
                double numer = c+e-b-d;
                double denom = a-2*b+c;
                s = clamp(numer/denom);
                t = 1-s;
            }else{
                s = clamp(-e/c);
                t = 0.0;
            }
        }else{
            double numer = c+e-b-d;
            double denom = a-2*b+c;
            s = clamp(numer/denom);
            t = 1.0 - s;
        }
    }
    return t0 + s * edge0 + t * edge1;
}

vnl_vector<double> toDoubleVector(const vnl_vector<float>& src){
	vnl_vector<double> ret(src.size());
	for(int i = 0; i < src.size(); ++i){
		ret.put(i, double(src[i]));
	}
	return ret;
}

itk::Mesh<float, 3>::Pointer itkFromVtk(vtkPolyData* input, const std::string& path){
	std::string temp("/tmp/temp.vtk");
	if(path != ""){
		temp = path;
	}
	saveVTKPolydata(input, temp);
	itk::Mesh<float, 3>::Pointer mesh = loadMesh(temp);
	if(path == ""){
		boost::filesystem::remove(temp);
	}
	return mesh;
}

vtkSmartPointer<vtkPolyData> vtkFromItk(itk::Mesh<float, 3>::Pointer input, const std::string& path){
	std::string temp("/tmp/temp.vtk");
	if(path != ""){
		temp = path;
	}
	saveVTKFile(input, temp);
	vtkSmartPointer<vtkPolyData> vtkReference = loadVTKPolydata(temp);
	if(path == ""){
		boost::filesystem::remove(temp);
	}
	return vtkReference;
}

itk::Point<float, 3> getClosestPointOnMesh(itk::Mesh<float, 3>::Pointer mesh, itk::Point<float, 3> point){
	//Get Points from the given mesh
	itk::PointsLocator< itk::Mesh<float, 3>::PointsContainer >::Pointer ptLocator = itk::PointsLocator< itk::Mesh<float, 3>::PointsContainer >::New();
    ptLocator->SetPoints(mesh->GetPoints());
    ptLocator->Initialize();

	//project landmark on the mesh surface
	int closestPointId = ptLocator->FindClosestPoint(point);
	double dist = std::numeric_limits<double>::max();
	vnl_vector<double> bufferedCoordinates = toDoubleVector(point.GetVnlVector());
	vnl_vector<double> projectedCoordinates = bufferedCoordinates; //initialize new position with the old one
	//iterate over all cells of the mesh
	for(itk::Mesh<float, 3>::CellsContainer::Iterator cellIterator = mesh->GetCells()->Begin(); cellIterator != mesh->GetCells()->End(); ++cellIterator){
		itk::Mesh<float, 3>::CellType *cellptr = cellIterator.Value();
		std::vector<int> vertexIds;
		
		//iterate over all vertices of the cell
		for(itk::Mesh<float, 3>::CellType::PointIdIterator pointIterator = cellptr->PointIdsBegin(); pointIterator != cellptr->PointIdsEnd(); ++pointIterator){
			vertexIds.push_back(*pointIterator);
		}
		
		//cell is one around the closest mesh vertex
		if(std::find(vertexIds.begin(), vertexIds.end(), closestPointId) != vertexIds.end()){
			vnl_vector<double> v0 = toDoubleVector(mesh->GetPoint(vertexIds[0]).GetVnlVector());
			vnl_vector<double> v1 = toDoubleVector(mesh->GetPoint(vertexIds[1]).GetVnlVector());
			vnl_vector<double> v2 = toDoubleVector(mesh->GetPoint(vertexIds[2]).GetVnlVector());
			vnl_vector<double> pointOnCell = closesPointOnTriangle(v0, v1, v2, bufferedCoordinates);
			double pointDist = sqrt(((pointOnCell.get(0)-bufferedCoordinates.get(0))*(pointOnCell.get(0)-bufferedCoordinates.get(0))) + 
					((pointOnCell.get(1)-bufferedCoordinates.get(1))*(pointOnCell.get(1)-bufferedCoordinates.get(1))) + 
					((pointOnCell.get(2)-bufferedCoordinates.get(2))*(pointOnCell.get(2)-bufferedCoordinates.get(2))));
			if(pointDist < dist){
				projectedCoordinates = pointOnCell;
			}					
		}
	}
	//create point from the vnl_vector
	itk::Point<float, 3> ret;
	ret[0] = projectedCoordinates.get(0);
	ret[1] = projectedCoordinates.get(1);
	ret[2] = projectedCoordinates.get(2);
	
	return ret;
}

itk::Mesh<float, 3>::Pointer fitModelToMesh(itk::StatisticalModel<itk::Mesh<float, 3> >::Pointer model, itk::Mesh<float, 3>::Pointer mesh, int iterations){
	// create a shape model transform
    itk::StatisticalShapeModelTransform<itk::Mesh<float, 3>, double, 3>::Pointer statModelTransform = itk::StatisticalShapeModelTransform<itk::Mesh<float, 3>, double, 3>::New();
    statModelTransform->SetStatisticalModel(model);
    statModelTransform->SetIdentity();

    // The actual fitting will be done to a distance image representation of the mesh.
    itk::Image<float, 3>::Pointer distanceImage = distanceImageFromMesh(mesh);

    // set up the optimizer
    itk::LBFGSBOptimizer::Pointer optimizer = itk::LBFGSBOptimizer::New();
    optimizer->SetMaximumNumberOfIterations(iterations);
    optimizer->SetProjectedGradientTolerance(0);
    optimizer->MinimizeOn();
	
	itk::LBFGSBOptimizer::BoundSelectionType boundSelect(statModelTransform->GetNumberOfParameters());
	itk::LBFGSBOptimizer::BoundValueType upperBound(statModelTransform->GetNumberOfParameters());
	itk::LBFGSBOptimizer::BoundValueType lowerBound(statModelTransform->GetNumberOfParameters());
	boundSelect.Fill(0);
	upperBound.Fill(0.0);
	lowerBound.Fill(0.0);
	optimizer->SetBoundSelection(boundSelect);
	optimizer->SetUpperBound(upperBound);
	optimizer->SetLowerBound(lowerBound);

    itk::MeanSquaresPointSetToImageMetric<itk::PointSet<float, 3>, itk::Image<float, 3> >::Pointer metric = 
		itk::MeanSquaresPointSetToImageMetric<itk::PointSet<float, 3>, itk::Image<float, 3> >::New();
    itk::LinearInterpolateImageFunction<itk::Image<float, 3>, double>::Pointer interpolator = 
		itk::LinearInterpolateImageFunction<itk::Image<float, 3>, double>::New();

    itk::PointSet<float, 3>::Pointer fixedPointSet = itk::PointSet<float, 3>::New();
    fixedPointSet->SetPoints(model->GetRepresenter()->GetReference()->GetPoints());
    itk::PointSet<float, 3>::PointDataContainer::Pointer points = itk::PointSet<float, 3>::PointDataContainer::New();
    points->Reserve(model->GetRepresenter()->GetReference()->GetNumberOfPoints());
    for (itk::PointSet<float, 3>::PointDataContainer::Iterator it = points->Begin(); it != points->End(); ++it) {
        it->Value() = 0;
    }
    fixedPointSet->SetPointData(points);

    itk::PointSetToImageRegistrationMethod<itk::PointSet<float, 3>,itk::Image<float, 3> >::Pointer registration = 
		itk::PointSetToImageRegistrationMethod<itk::PointSet<float, 3>,itk::Image<float, 3> >::New();
    registration->SetInitialTransformParameters(statModelTransform->GetParameters());
    registration->SetMetric(metric);
    registration->SetInterpolator(interpolator);
    registration->SetOptimizer(optimizer);
    registration->SetTransform(statModelTransform);

    registration->SetFixedPointSet(fixedPointSet);
    registration->SetMovingImage(distanceImage);  
  
    try {
        registration->Update();

    } catch ( itk::ExceptionObject& o ) {
        std::cout << "Caught exception: " << o << std::endl;
    }

    // Transform the mesh according to the optimized transformation.
    itk::TransformMeshFilter<itk::Mesh<float, 3>, itk::Mesh<float, 3>, itk::StatisticalShapeModelTransform<itk::Mesh<float, 3>, double, 3> >::Pointer transformMeshFilter = 
		itk::TransformMeshFilter<itk::Mesh<float, 3>, itk::Mesh<float, 3>, itk::StatisticalShapeModelTransform<itk::Mesh<float, 3>, double, 3> >::New();
    transformMeshFilter->SetInput(model->GetRepresenter()->GetReference());
    transformMeshFilter->SetTransform(statModelTransform);
    transformMeshFilter->Update();

    itk::Mesh<float, 3>::Pointer fittedMesh = transformMeshFilter->GetOutput();
	return fittedMesh;
}

vtkSmartPointer<vtkPolyData> performPolyDataTransform(vtkSmartPointer<vtkPolyData> data, vtkSmartPointer<vtkIterativeClosestPointTransform> transformation){
	vtkSmartPointer<vtkTransformPolyDataFilter> icpTransformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	icpTransformFilter->SetInputData(data);
	icpTransformFilter->SetTransform(transformation);
	icpTransformFilter->Update();
	return icpTransformFilter->GetOutput();
}

vtkSmartPointer<vtkIterativeClosestPointTransform> getRigidTransformation(vtkSmartPointer<vtkPolyData> source, vtkSmartPointer<vtkPolyData> target, int iterations){
	vtkSmartPointer<vtkIterativeClosestPointTransform> icp = vtkSmartPointer<vtkIterativeClosestPointTransform>::New();
	icp->SetSource(source);
	icp->SetTarget(target);
	icp->GetLandmarkTransform()->SetModeToRigidBody();
	icp->SetMaximumNumberOfIterations(iterations);
	icp->Modified();
	icp->Update();
	return icp;
}

double calculateVolume(itk::Mesh<float, 3>::Pointer mesh){
	itk::TriangleMeshToSimplexMeshFilter< itk::Mesh<float, 3>, itk::SimplexMesh<float, 3> >::Pointer convert = 
		itk::TriangleMeshToSimplexMeshFilter< itk::Mesh<float, 3>, itk::SimplexMesh<float, 3> >::New();
	convert->SetInput(mesh);
	convert->Update();
 
	// Calculate the volume and area of the simplex mesh.
	itk::SimplexMeshVolumeCalculator<itk::SimplexMesh<float, 3> >::Pointer volume = 
		itk::SimplexMeshVolumeCalculator<itk::SimplexMesh<float, 3> >::New();
	volume->SetSimplexMesh(convert->GetOutput());
	volume->Compute();
	return volume->GetVolume();
}
