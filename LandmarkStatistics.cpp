#include <string>
#include <vector>
#include <map>
#include <utility>
#include <boost/property_tree/ptree.hpp>
#include <boost/optional.hpp>
#include <boost/lexical_cast.hpp>

#include "LandmarkStatistics.hpp"

LandmarkStatistics::LandmarkStatistics() : AbstractStatistics(){}

LandmarkStatistics::~LandmarkStatistics(){}

void LandmarkStatistics::addData(boost::property_tree::ptree data){
	boost::optional<boost::property_tree::ptree &> error_tree = data.get_child_optional("detailed_errors");
	if(error_tree){
		for (boost::property_tree::ptree::iterator it = error_tree.get().begin(); it != error_tree.get().end(); ++it) {
			addSingleData(it->first, boost::lexical_cast<double>(it->second.data()));
		}
	}
}

void LandmarkStatistics::addSingleData(std::string key, double value){
	std::map<std::string, std::vector<double>* >::iterator it = dataMap.find(key);
	if(it != dataMap.end()){
		it->second->push_back(value);
	}else{
		std::vector<double>* v = new std::vector<double>();
		v->push_back(value);
		dataMap.insert(std::pair<std::string, std::vector<double>* >(key, v));
	}
}
