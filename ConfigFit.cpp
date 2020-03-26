#include <iostream>
#include <string>
#include <fstream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>

#include "ConfigUtils.hpp"

/**
 * An example program that shows how the configuration file could be created.
 * Asks everything as user input in the console, could take some time to configure everything.
 */
int main(int argc, char *argv[]){
	if(argc < 2){
		std::cout << "Usage " << argv[0] << " filename.json" << std::endl;
		exit(-1);
	}
	std::string filename(argv[1]);
	boost::property_tree::ptree config;
	
	addPath("Path to the atlas: ", "atlas_path", &config);
	addPath("Path to the patient data: ", "patient_path", &config);
	addPath("Path for the output mesh: ", "vtk_output_path", &config);
	std::string boneType;
	bool correct = false;
	while(!correct){
		std::cout << "Which bone type has the atlas and the patient data? (radius or ulna): " << std::flush;
		getline(std::cin, boneType);
		if(boneType == "radius" || boneType == "ulna"){
			correct = true;
		}
	}
	config.put("type", boneType);
	std::string input_line;
	std::cout << "Are the landmarks of the patient known? (y or n): " << std::flush;
	getline(std::cin, input_line);
	if(input_line == "y"){
		if(boneType == "radius"){
			addRadiusLandmarks(&config);
		}else if(boneType == "ulna"){
			addUlnaLandmarks(&config);
		}
	}
	addParameter<int>("Maximal number of iterations for the alignment between model and patient mesh: ", "align_iter", &config);
	addParameter<int>("Maximal number of iterations for the fitting of the model: ", "fit_iter", &config);
	boost::property_tree::write_json(filename, config);
	return EXIT_SUCCESS;
}
