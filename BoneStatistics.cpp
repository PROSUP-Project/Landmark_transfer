#include <string>
#include <vector>
#include <map>
#include <utility>
#include <boost/property_tree/ptree.hpp>
#include <boost/optional.hpp>
#include <boost/lexical_cast.hpp>

#include "BoneStatistics.hpp"

BoneStatistics::BoneStatistics() : AbstractStatistics(){}

BoneStatistics::~BoneStatistics(){}

void BoneStatistics::addData(boost::property_tree::ptree data){
	boost::optional<boost::property_tree::ptree &> error_tree = data.get_child_optional("detailed_errors");
	if(error_tree){
		std::vector<double>* v = new std::vector<double>();
		for (boost::property_tree::ptree::iterator it = error_tree.get().begin(); it != error_tree.get().end(); ++it) {
			v->push_back(boost::lexical_cast<double>(it->second.data()));
		}
		dataMap.insert(std::pair<std::string, std::vector<double>* >(getBoneName(data), v));
	}
}

std::string BoneStatistics::getBoneName(boost::property_tree::ptree data){
	boost::optional<std::string> optionalBoneName = data.get_optional<std::string>("patient");
	if(optionalBoneName){
		return optionalBoneName.get();
	}
	static int nameNum(0);
	std::string ret("bone");
	if(nameNum < 10)
		ret+="0";
	if(nameNum < 100)
		ret+="0";
	ret+= boost::lexical_cast<std::string>(nameNum++);
	return ret;
}
