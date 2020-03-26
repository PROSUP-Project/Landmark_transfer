#ifndef __LANDMAKR_HPP__
#define __LANDMAKR_HPP__

#include <string>
#include <vector>

#include <boost/property_tree/ptree.hpp>

#include <itkPoint.h>

#include <vtkMatrix4x4.h>
#include <vtkSmartPointer.h>
#include <vtkIterativeClosestPointTransform.h>

#include "BoneType.hpp"

/**
 * Class that represents a single landmark.
 */
class Landmark{
	public:
		/**
		 * Creates a landmark with the given name and configuration.
		 * @param landmarkName			landmark name
		 * @param landmarkCoordinates	coordinates of the landmark
		 */
		Landmark(std::string landmarkName, boost::property_tree::ptree landmarkCoordinates);
		
		/**
		 * Creates a landmark with the given name and coordinates.
		 * @param landmarkName	landmark name
		 * @param x				x coordinate of the landmark
		 * @param y				y coordinate of the landmark
		 * @param z				z coordinate of the landmark
		 */
		Landmark(std::string landmarkName, double x, double y, double z);
		
		/**
		 * Performs a transformation with the given matrix on the coordinates of the landmark.
		 * @param transform		transformation matrix that should be applied on the landmark coordinates.
		 */
		void transform(vtkSmartPointer<vtkMatrix4x4> transform);
		
		/**
		 * Returns a point at the coordinates of the landmark.
		 * @return the landmark point
		 */
		itk::Point<float, 3> getLandmarkPoint();
		
		/**
		 * Returns the name of the landmark.
		 * @return the landmark name
		 */
		std::string getName();
		
		/**
		 * Returns the x coordinate of the landmark.
		 * @return x coordinate
		 */
		double getX();
		
		/**
		 * Returns the y coordinate of the landmark.
		 * @return y coordinate
		 */
		double getY();
		
		/**
		 * Returns the z coordinate of the landmark.
		 * @return z coordinate
		 */
		double getZ();
		
		/**
		 * Sets the coordinates to the given coordinates.
		 * @param coordinates	array of the coordinates, first position: x coordinate, second position: y coordinate, third position: z coordinate
		 */
		void setCoordinates(double* coordinates);
		
		/**
		 * Sets the coordinates to the given coordinates.
		 * @param coordinates	vnl_vector of the coordinates, first position: x coordinate, second position: y coordinate, third position: z coordinate
		 */
		void setCoordinates(vnl_vector<double> coordinates);
		
		/**
		 * Sets the coordinates to the given coordinates.
		 * @param coordinates	itk point
		 */
		void setCoordinates(itk::Point<float, 3> coordinates);
		
		/**
		 * Returns the distance from this landmark to the given one.
		 * @param l		pointer to a landmark for which the displacement to this landmark should be calculated
		 * @return the displacement
		 */
		double getDistance(Landmark* l);
		
		/**
		 * Extracts the point with the given index from a mesh and creates a landmark with the given name from the point.
		 * @param mesh			mesh from which the point should be extracted
		 * @param name			name of the landmark
		 * @param position		index of the point that should be extracted
		 * @return	a pointer to the created landmark, had to be freed manually
		 */
		static Landmark* getLandmarkFromMesh(itk::Mesh<float, 3>::Pointer mesh, const std::string& name, int position);
		
		/**
		 * Extracts all landmarks from the given mesh.
		 * Which points are extracted depends on the given bone type.
		 * @param mesh	mesh from which the last point should be extracted and parsed into landmarks
		 * @param t		type of the bone
		 * @return	map containing all landmarks of this bone type and the corresponding names
		 */
		static std::map<std::string, Landmark*> getLandmarksFromMesh(itk::Mesh<float, 3>::Pointer mesh, BoneType::e t);
		
		/**
		 * Appends the given landmarks to the given mesh.
		 * @param mesh	 		where the landmarks should be added
		 * @param landmarks 	the landmarks that should be added to the mesh
		 * @param type			type of the bone, which mesh and landmarks are given
		 */
		static void addLandmarksToMesh(itk::Mesh<float, 3>::Pointer mesh, std::map<std::string, Landmark*> landmarks, BoneType::e type);
		
		/**
		 * Applies the given transformation on the given landmark points.
		 * @param landmarks			the landmarks that should be transformed
		 * @param transformation	the transformation that should be applied on the landmarks
		 */
		static void transformLandmarks(std::map<std::string, Landmark*> landmarks, vtkSmartPointer<vtkIterativeClosestPointTransform> transformation);
		
	private:
		/**
		 * Searched for a landmark with the given name in the given map of landmarks.
		 * If the landmark exists the point which represents the landmark coordinates will be added to the
		 * at the given position.
		 * @param mesh				mesh where the point should be added
		 * @param landmarkname		name of the landmark that should be added
		 * @param position			point index where the point should be added
		 */
		static void addLandmarkToMesh(itk::Mesh<float, 3>::Pointer mesh, std::map<std::string, Landmark*> landmarks, const std::string& landmarkname, int position);
		
		//x coordinate
		double m_x;
		//y coordinate
		double m_y;
		//z coordinate
		double m_z;
		
		//landmark name
		std::string t;
};
#endif
