#include <string>
#include <iostream>
#include <vector>
#include <map>
#include <utility>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/optional.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/erase.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>

#include "AbstractStatistics.hpp"

AbstractStatistics::AbstractStatistics(){
	dataMap = std::map<std::string, std::vector<double>* >();
}

AbstractStatistics::~AbstractStatistics(){
	for (std::map<std::string, std::vector<double>* >::iterator it = dataMap.begin(); it != dataMap.end(); ++it) {
		delete (it->second);
	}
}

void AbstractStatistics::addDataFromFile(const std::string& data_path){
	boost::property_tree::ptree data;
	try{
		boost::property_tree::read_json(data_path, data);
		addData(data);
	}catch( boost::property_tree::json_parser::json_parser_error &e){
		std::cout << "Configuration file can not be read..." << std::endl;
		std::cout << "Message: " << e.message() << std::endl;
		std::cout << "File-name: " << e.filename() << std::endl;
		std::cout << "Line: " << e.line() << std::endl;
	}
}

double AbstractStatistics::getMin(std::string key){
	return getMin(getVector(key));
}

double AbstractStatistics::getMin(){
	std::vector<double>* v = getVector();
	double ret  = getMin(v);
	delete v;
	return ret;
}

double AbstractStatistics::getMax(std::string key){
	return getMax(getVector(key));
}

double AbstractStatistics::getMax(){
	std::vector<double>* v = getVector();
	double ret  = getMax(v);
	delete v;
	return ret;
}

double AbstractStatistics::getMean(std::string key){
	return getMean(getVector(key));
}

double AbstractStatistics::getMean(){
	std::vector<double>* v = getVector();
	double ret  = getMean(v);
	delete v;
	return ret;
}

double AbstractStatistics::getVariance(std::string key){
	return getVariance(getVector(key));
}

double AbstractStatistics::getVariance(){
	std::vector<double>* v = getVector();
	double ret  = getVariance(v);
	delete v;
	return ret;
}

std::vector<std::string> AbstractStatistics::getKeys(){
	std::vector<std::string> v;
	for(std::map<std::string, std::vector<double>* >::iterator it = dataMap.begin(); it != dataMap.end(); ++it) {
		v.push_back(it->first);
	}
	return v;
}

std::vector<double>* AbstractStatistics::getVector(std::string key){
	std::map<std::string, std::vector<double>* >::iterator it = dataMap.find(key);
	if(it != dataMap.end()){
		return it->second;
	}else{
		return nullptr;
	}
}

std::vector<double>* AbstractStatistics::getVector(){
	std::vector<double>* v = new std::vector<double>();
	
	for(std::map<std::string, std::vector<double>* >::iterator it = dataMap.begin(); it != dataMap.end(); ++it){
		v->insert(v->end(), it->second->begin(), it->second->end());
	}
	
	return v;
}

double AbstractStatistics::getMin(std::vector<double>* v){
	double min = std::numeric_limits<double>::max();
	for(int i = 0; i < v->size(); ++i){
		if((*v)[i] < min){
			min = (*v)[i];
		}
	}
	return min;
}

double AbstractStatistics::getMax(std::vector<double>* v){
	double max = 0;
	for(int i = 0; i < v->size(); ++i){
		if((*v)[i] > max){
			max = (*v)[i];
		}
	}
	return max;
}

double AbstractStatistics::getMean(std::vector<double>* v){
	boost::accumulators::accumulator_set<double, boost::accumulators::stats<boost::accumulators::tag::mean> > acc;
	std::for_each(v->begin(), v->end(), boost::bind<void>(boost::ref(acc), _1));
	return boost::accumulators::extract::mean(acc);
}

double AbstractStatistics::getVariance(std::vector<double>* v){
	boost::accumulators::accumulator_set<double, boost::accumulators::stats<boost::accumulators::tag::variance(boost::accumulators::lazy)> > acc;
	std::for_each(v->begin(), v->end(), boost::bind<void>(boost::ref(acc), _1));
	return boost::accumulators::extract::variance(acc);
}

std::string AbstractStatistics::getRTable(const std::string &name, std::vector<std::string>* names){
	std::string ret = "";
	std::string tablename = name;
	tablename.erase(std::remove(tablename.begin(), tablename.end(), ' '), tablename.end());
	std::string table = tablename + ".data <- data.frame(";
	std::string namestr = "";
	for(int i = 0; names != nullptr && i < names->size(); ++i){
		if(i == 0){
			namestr += ", names = c(";
		}
			namestr += "'" + (*names)[i] + "'";
		if(i != names->size() - 1){
			namestr += ", ";
		}else{
			namestr += ")";
		}
	}
	std::vector<std::string> keys = getKeys();
	for(int i = 0; i < keys.size(); ++i){
		if(names != nullptr && i < names->size()){
			ret += getRVector(getVector(keys[i]), (*names)[i]);
			table += (*names)[i];
		}else{
			ret += getRVector(getVector(keys[i]), keys[i]);
			table += keys[i];
		}
		if(i != keys.size() - 1){
			table += ", ";
		}
	}
	table += ")\n";
	ret += table;
	ret += "boxplot(" + tablename + ".data, las = 2, ylab =\"Error in mm\"" + namestr + ", main = \"" + name + "\")\n";
	return ret;
}

std::string AbstractStatistics::getRVector(std::vector<double>* v, const std::string &name){
	std::string ret = name + " <- c(";
	for(int i = 0; v != nullptr && i < v->size(); ++i){
		ret += boost::lexical_cast<std::string>((*v)[i]);
		if(i != v->size() - 1){
			ret += ", ";
		}
	}
	ret += ")\n";
	return ret;
}
