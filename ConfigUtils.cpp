#include <iostream>
#include <string>
#include <fstream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>

#include "ConfigUtils.hpp"

void addPath(const std::string& input_phrase, const std::string& name, boost::property_tree::ptree* tree){
	std::string input_line;
	bool correct = false;
	while(!correct){
		std::cout << input_phrase << std::flush;
		getline(std::cin, input_line);
		boost::filesystem::path p(input_line);
		if(boost::filesystem::is_regular_file(p)){
			p = boost::filesystem::canonical(p);
			tree->put(name, p.string());
			correct = true;
		} else {
			std::cout << "File not found." << std::endl;
		}
	}
}

void addOptionalPath(const std::string& input_phrase, const std::string& name, boost::property_tree::ptree* parameters){
	std::string input_line;
	bool correct = false;
	while(!correct){
		std::cout << input_phrase << std::flush;
		getline(std::cin, input_line);
		if(input_line == ""){
			break;
		}
		boost::filesystem::path p(input_line);
		if(boost::filesystem::is_regular_file(p)){
			p = boost::filesystem::canonical(p);
			parameters->put(name, p.string());
			correct = true;
		} else {
			std::cout << "File not found." << std::endl;
		}
	}
}

void addLandmark(boost::property_tree::ptree* landmarks, const std::string& name){
	boost::property_tree::ptree landmark;
	std::string input_line;
	bool correct = false;
	while(!correct){
		std::cout << "Coordinates of the " << name << " Landmark:" << std::endl;
		try{
			addParameter<float>("x: ", "x", &landmark);
			addParameter<float>("y: ", "y", &landmark);
			addParameter<float>("z: ", "z", &landmark);
			landmarks->push_back(std::make_pair(name, landmark));
			correct = true;
		}catch( boost::bad_lexical_cast &e){
			std::cout << "Input error, please repeat." << std::endl;
		}
	}
}

void addRadiusLandmarks(boost::property_tree::ptree* config){
	boost::property_tree::ptree landmarks;
	std::cout << "Landmarks: " << std::endl;
	addLandmark(&landmarks, "RAF_P");
	addLandmark(&landmarks, "CBD_R");
	addLandmark(&landmarks, "CBP_R");
	addLandmark(&landmarks, "AB_R");
	addLandmark(&landmarks, "DOB_R");
	addLandmark(&landmarks, "POC_R");
	addLandmark(&landmarks, "DOAC_R");
	addLandmark(&landmarks, "DRUL_R");
	addLandmark(&landmarks, "PRUL_R");
	config->add_child("landmarks", landmarks);
}

void addUlnaLandmarks(boost::property_tree::ptree* config){
	boost::property_tree::ptree landmarks;
	std::cout << "Landmarks: " << std::endl;
	addLandmark(&landmarks, "RAF_D");
	addLandmark(&landmarks, "CBD_U");
	addLandmark(&landmarks, "CBP_U");
	addLandmark(&landmarks, "AB_U");
	addLandmark(&landmarks, "DOB_U");
	addLandmark(&landmarks, "POC_U");
	addLandmark(&landmarks, "DOAC_U");
	addLandmark(&landmarks, "DRULs_U");
	addLandmark(&landmarks, "DRULd_U");
	addLandmark(&landmarks, "PRULs_U");
	addLandmark(&landmarks, "PRULd_U");
	config->add_child("landmarks", landmarks);
}
