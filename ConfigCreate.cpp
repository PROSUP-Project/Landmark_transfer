#include <iostream>
#include <string>
#include <fstream>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>

#include "ConfigUtils.hpp"

/**
 * Asks if there are some pre calculated Gaussian process models and if so the paths will be added as parameters.
 * @param parameters	property tree where the parameter should be added
 */
void addPreCalculatedModels(boost::property_tree::ptree* parameters){
	std::cout << "Does pre calculated Gaussian process models should be used? (y or n): " << std::flush;
	std::string input_line;
	getline(std::cin, input_line);
	if(input_line == "y"){
		addOptionalPath("Path to the model of the radius of the right side (can be left empty): ", "gauss_path_raius_r", parameters);
		addOptionalPath("Path to the model of the radius of the left side (can be left empty): ", "gauss_path_raius_l", parameters);
		addOptionalPath("Path to the model of the ulna of the right side (can be left empty): ", "gauss_path_ulna_r", parameters);
		addOptionalPath("Path to the model of the ulna of the left side (can be left empty): ", "gauss_path_ulna_l", parameters);
	}
}

/**
 * Asks if the resulting statistical model should be improved by a Gaussian kernel for better fitting with a 
 * low amount of input data.
 * @param parameters	property tree where the parameter should be added
 */
void addImprovement(boost::property_tree::ptree* parameters){
	std::cout << "Does a Gaussian improved statistical model should be created? (y or n): " << std::flush;
	std::string input_line;
	getline(std::cin, input_line);
	if(input_line == "y"){
		addParameter<double>("Sigma of the Gaussian kernel for the improvement: ", "atlas_sigma", parameters);
		addParameter<double>("Scale of the Gaussian kernel for the improvement: ", "atlas_scale", parameters);
		addParameter<int>("Number of parameters of the Gaussian kernel for the improvement: ", "atlas_num_comp", parameters);
	}
}

/**
 * Adds some parameters to the given property tree
 * @param parameters	the property tree into which the parameters should be added
 */
void setParameters(boost::property_tree::ptree* parameters){
	addParameter<int>("Maximal number of iterations for the rigid alignment: ", "rigid_iter", parameters);
	addParameter<double>("Sigma of the Gaussian kernel: ", "kernel_sigma", parameters);
	addParameter<double>("Scale of the Gaussian kernel: ", "kernel_scale", parameters);
	addParameter<int>("Number of levels of the Gaussian kernel: ", "kernel_levels", parameters);
	addParameter<int>("Number of components in the Gaussian process model: ", "gauss_comp", parameters);
	addParameter<int>("Maximal number of iterations for fitting of the Gaussian model: ", "gauss_fit_iter", parameters);
	addParameter<double>("Minimal threshold for the fitted volume (default = 0.6): ", "correspond_volume_percentage", parameters);
	addParameter<double>("Variance of the landmark inaccuracy, has to be greater than 0: ", "landmark_variance", parameters);
	addParameter<double>("Atlas variance: ", "atlas_variance", parameters);
	addParameter<double>("Sigma for the enhanced atlas: ", "atlas_sigma", parameters);
	addParameter<double>("Scale for the enhanced atlas: ", "atlas_scale", parameters);
	addParameter<int>("Number of components in the enhanced atlas: ", "atlas_num_comp", parameters);
	addPreCalculatedModels(parameters);
	addImprovement(parameters);
}

/**
 * Adds a new radius to the given list of bones
 * @param bones		a property tree containing a list of radii
 */
void addRadius(boost::property_tree::ptree* bones){
	boost::property_tree::ptree bone;
	bool correct = false;
	std::string input_line;
	//path
	addPath("Path to the model: ", "path", &bone);
	addParameter<std::string>("Side (l or r): ", "side", &bone);
	//landmarks
	addRadiusLandmarks(&bone);
	bones->push_back(std::make_pair("", bone));
}

/**
 * Adds a new ulna to the given list of bones
 * @param bones		a property tree containing a list of ulnae
 */
void addUlna(boost::property_tree::ptree* bones){
	boost::property_tree::ptree bone;
	bool correct = false;
	std::string input_line;
	//path
	addPath("Path to the model: ", "path", &bone);
	addParameter<std::string>("Side (l or r): ", "side", &bone);
	//landmarks
	addUlnaLandmarks(&bone);
	bones->push_back(std::make_pair("", bone));
}

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
	std::string input_line;
	boost::property_tree::ptree parameters;
	boost::property_tree::ptree bones;
	boost::property_tree::ptree radius;
	boost::property_tree::ptree ulna;
	std::cout << " --- AtlasCreation Configuration --- " << std::endl;
	bool done = false;
	while(true){
		std::cout << "Please choose:" << std::endl;
		std::cout << "0...Exit without saving" << std::endl;
		std::cout << "1...Write the json file and exit" << std::endl;
		std::cout << "2...Add a new bone model" << std::endl;
		std::cout << "3...Specify program parameters (optional)" << std::endl;
		std::cout << "Option: " << std::flush;
		int option = -1;
		getline(std::cin, input_line);
		try   {
			option = boost::lexical_cast<int>(input_line);
		} catch( boost::bad_lexical_cast &e){/*do nothing, input loop will continue with a new iteration*/}
		if(option < 0 || option > 5){
			continue;
		}
		switch(option){
			case 0:
				return 0;
			case 1:
				config.add_child("parameters", parameters);
				bones.add_child("radius", radius);
				bones.add_child("ulna", ulna);
				config.add_child("bones", bones);
				boost::property_tree::write_json(filename, config);
				return 0;
			case 2:
				done = false;
				while(!done){
					std::cout << "Which bone:" << std::endl;
					std::cout << "0...Radius" << std::endl;
					std::cout << "1...Ulna" << std::endl;
					std::cout << "Option: " << std::flush;
					option = -1;
					getline(std::cin, input_line);
					try{
						option = boost::lexical_cast<int>(input_line);
					}catch( boost::bad_lexical_cast &e){/*do nothing, input loop will continue with a new iteration*/}
					if(option == 0){
						addRadius(&radius);
						done = true;
					} else if (option == 1) {
						addUlna(&ulna);
						done = true;
					}
				}
				break;
			case 3:
				setParameters(&parameters);
				break;
		}
	}
	return EXIT_SUCCESS;
}
