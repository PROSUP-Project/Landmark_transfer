#include <vector>

#include <boost/property_tree/ptree.hpp>
#include <boost/optional.hpp>

#include "Bone.hpp"
#include "BoneManager.hpp"
#include "SingleBoneTypeManager.hpp"
#include "BoneType.hpp"

BoneManager::BoneManager(boost::property_tree::ptree bonesConfig, const std::string& output_folder_radius_r, const std::string& output_folder_radius_l, const std::string& output_folder_ulna_r, const std::string& output_folder_ulna_l){
	radii_r = new SingleBoneTypeManager();
	radii_l = new SingleBoneTypeManager();
	ulnae_r = new SingleBoneTypeManager();
	ulnae_l = new SingleBoneTypeManager();
	boost::optional<boost::property_tree::ptree &> radii_tree = bonesConfig.get_child_optional("radius");
	if(radii_tree){
		for (boost::property_tree::ptree::iterator it = radii_tree.get().begin(); it != radii_tree.get().end(); ++it) {
			if(it->second.get<std::string>("side") == "r"){
				radii_r->addBone(new Bone(it->second, output_folder_radius_r, BoneType::radius));
			} else {
				radii_l->addBone(new Bone(it->second, output_folder_radius_l, BoneType::radius));
			}
		}
	}
	boost::optional<boost::property_tree::ptree &> ulnae_tree = bonesConfig.get_child_optional("ulna");
	if(ulnae_tree){
		for (boost::property_tree::ptree::iterator it = ulnae_tree.get().begin(); it != ulnae_tree.get().end(); ++it) {
			if(it->second.get<std::string>("side") == "r"){
				ulnae_r->addBone(new Bone(it->second, output_folder_ulna_r, BoneType::ulna));
			} else {
				ulnae_l->addBone(new Bone(it->second, output_folder_ulna_l, BoneType::ulna));
			}
		}
	}
}

SingleBoneTypeManager* BoneManager::getRadii_r(){
	return radii_r;
}

SingleBoneTypeManager* BoneManager::getRadii_l(){
	return radii_l;
}

SingleBoneTypeManager* BoneManager::getUlnae_r(){
	return ulnae_r;
}

SingleBoneTypeManager* BoneManager::getUlnae_l(){
	return ulnae_l;
}

BoneManager::~BoneManager(){
	delete radii_r;
	delete radii_l;
	delete ulnae_r;
	delete ulnae_l;
}
