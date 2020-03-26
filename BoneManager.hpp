#ifndef __BONE_MANAGER_HPP__
#define __BONE_MANAGER_HPP__

#include <boost/property_tree/ptree.hpp>

#include <itkMesh.h>
#include <itkStandardMeshRepresenter.h>
#include <itkStatisticalModel.h>

#include "Bone.hpp"
#include "SingleBoneTypeManager.hpp"

/**
 * A class that manages 4 types of bones.
 * One single type manager per type and side of the bone is managed.
 * Types: ulna and radius
 * Sides: right and left
 */
class BoneManager{
	public:
		/**
		 * Sets up the managers containing all bones with the given configuration.
		 * @param bonesConfig				configuration containing all informations to create the bones
		 * @param output_folder_radius_r	path to the output folder for the right radii
		 * @param output_folder_radius_l	path to the output folder for the left radii
		 * @param output_folder_ulna_r		path to the output folder for the right ulnae
		 * @param output_folder_ulna_l		path to the output folder for the left ulnae
		 */
		BoneManager(boost::property_tree::ptree bonesConfig, const std::string& output_folder_radius_r, const std::string& output_folder_radius_l, const std::string& output_folder_ulna_r, const std::string& output_folder_ulna_l);
		
		/**
		 * Frees all bones in the vectors.
		 */
		~BoneManager();
		
		/**
		 * Returns the pointer to the manager for
		 * @return the manager for the bone type
		 */
		SingleBoneTypeManager* getRadii_r();
		
		/**
		 * Returns the pointer to the manager for
		 * @return the manager for the bone type
		 */
		SingleBoneTypeManager* getRadii_l();
		
		/**
		 * Returns the pointer to the manager for
		 * @return the manager for the bone type
		 */
		SingleBoneTypeManager* getUlnae_r();
		
		/**
		 * Returns the pointer to the manager for
		 * @return the manager for the bone type
		 */
		SingleBoneTypeManager* getUlnae_l();

	private:	
		SingleBoneTypeManager* radii_r;
		SingleBoneTypeManager* radii_l;
		SingleBoneTypeManager* ulnae_r;
		SingleBoneTypeManager* ulnae_l;
};

#endif
