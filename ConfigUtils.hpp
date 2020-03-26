#ifndef __CONFIG_UTILS_HPP__
#define __CONFIG_UTILS_HPP__

#include <iostream>
#include <string>
#include <fstream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>

/**
 * Adds a path to the given property tree.
 * The path will be requested as user input with the given input phrase.
 * @param input_phrase	text that will be shown in the console to ask for the input
 * @param name			name of the parameter in the property tree
 * @param parameters	property tree where the parameter should be added
 */
void addPath(const std::string& input_phrase, const std::string& name, boost::property_tree::ptree* tree);

/**
 * Asks the given question and adds the path as parameter with the given name.
 * If the user input is left empty this parameter will be skipped.
 * @param input_phrase	text that will be shown in the console to ask for the input
 * @param name			name of the parameter in the property tree
 * @param parameters	property tree where the parameter should be added
 */
void addOptionalPath(const std::string& input_phrase, const std::string& name, boost::property_tree::ptree* parameters);

/**
 * Adds a new landmark to the given list of landmarks
 * @param landmarks		a property tree containing a list of landmarks
 * @param name			the name of the landmark that should be added
 */
void addLandmark(boost::property_tree::ptree* landmarks, const std::string& name);

/**
 * Adds all landmarks of a radius to the given property tree.
 * @param config	property tree where the landmarks child should be added
 */
void addRadiusLandmarks(boost::property_tree::ptree* config);

/**
 * Adds all landmarks of an ulna to the given property tree.
 * @param config	property tree where the landmarks child should be added
 */
void addUlnaLandmarks(boost::property_tree::ptree* config);

/**
 * Adds a parameter to the given property tree.
 * @param <T>			the type of the parameter
 * @param input_phrase	text that will be shown in the console to ask for the input
 * @param name			name of the parameter in the property tree
 * @param parameters	property tree where the parameter should be added
 */
template<typename T>
void addParameter(const std::string& input_phrase, const std::string& name, boost::property_tree::ptree* parameters){
	std::string input_line;
	bool correct = false;
	while(!correct){
		std::cout << input_phrase << std::flush;
		try{
			getline(std::cin, input_line);
			T input = boost::lexical_cast<T>(input_line);
			parameters->put(name, input);
			correct = true;
		}catch( boost::bad_lexical_cast &e){
			std::cout << "Input error, please repeat." << std::endl;
		}
	}
}

#endif
