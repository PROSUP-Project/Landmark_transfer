#include <string>
#include <iostream>
#include <vector>
#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/optional.hpp>

#include "LandmarkStatistics.hpp"
#include "AbstractStatistics.hpp"
#include "BoneStatistics.hpp"

/**
 * Fills the given statistic object with the data specified in the given property tree.
 * The statistics of this data then will be written in the given stream.
 */
void statistics(AbstractStatistics* stat, boost::property_tree::ptree config, std::ostream& out);

int main(int argc, char *argv[]){
	if(argc < 2){
		std::cout << "Usage " << argv[0] << " configuration_file" << std::endl;
		exit(-1);
	}
	
	if (!boost::filesystem::exists(argv[1]))
	{
		std::cout << "The specified configuration file does not exist...exiting" << std::endl;
		exit(-1);
	}
	
	//read configuration
	boost::property_tree::ptree config;
	try{
		boost::property_tree::read_json(argv[1], config);
	}catch( boost::property_tree::json_parser::json_parser_error &e){
		std::cout << "Configuration file can not be read..." << std::endl;
		std::cout << "Message: " << e.message() << std::endl;
		std::cout << "File-name: " << e.filename() << std::endl;
		std::cout << "Line: " << e.line() << std::endl;
		std::cout << "...exiting." << std::endl;
		exit(-1);
	}
	
	//stream for the statistics output, a file or the console output
	std::ofstream out;
	boost::optional<std::string> stat_path = config.get_optional<std::string>("stat_file_path");
	if(stat_path){
		out.open(stat_path.get());
	}
	
	std::ofstream r_file;
	boost::optional<std::string> r_path = config.get_optional<std::string>("r_file_path");
	if(r_path){
		r_file.open(r_path.get());
	}
	
	//object for landmark statistics
	LandmarkStatistics* landStat = new LandmarkStatistics();
	stat_path?out:std::cout << "By Landmark:" << std::endl;
	statistics(landStat, config, stat_path?out:std::cout);
	if(r_path){
		//add the boxplot code to the file
		r_file << landStat->getRTable("Error by landmark");
	}
	delete landStat;

	//object for bone statistics
	BoneStatistics* boneStat = new BoneStatistics();
	stat_path?out:std::cout << std::endl << "By Bone:" << std::endl;
	statistics(boneStat, config, stat_path?out:std::cout);
	if(r_path){
		//create names for the bones
		std::vector<std::string> names;
		std::vector<std::string> keys = boneStat->getKeys();
		for(int i = 0; i < keys.size(); ++i){
			//the name is just 'bone' plus a number
			std::string tmp = "bone" + boost::lexical_cast<std::string>(i+1);
			names.push_back(tmp);
		}
		//add the boxplot code to the file, without given names file paths will be the names
		r_file << boneStat->getRTable("Error by bone", &names);
	}
	delete boneStat;
	
	//close file if stat output was a file
	if(stat_path){
		out.close();
	}
	
	//close r file if used
	if(r_path){
		r_file.close();
	}
}

void statistics(AbstractStatistics* stat, boost::property_tree::ptree config, std::ostream& out){
	//fill the statistic object with all the data defined in the configuration file
	boost::property_tree::ptree paths = config.get_child("paths");
	for (boost::property_tree::ptree::iterator it = paths.begin(); it != paths.end(); ++it) {
		stat->addDataFromFile(boost::lexical_cast<std::string>(it->second.data()));
	}
	
	//get all landmark names
	std::vector<std::string> keys = stat->getKeys();
	//print error statistics for each landmark
	for(int i = 0; i < keys.size(); ++i){
		out << keys[i] << ":" << std::endl;
		out << "	min: " << stat->getMin(keys[i]) << std::endl;
		out << "	max: " << stat->getMax(keys[i]) << std::endl;
		out << "	mean: " << stat->getMean(keys[i]) << std::endl;
		double variance = stat->getVariance(keys[i]);
		out << "	variance: " << variance << std::endl;
		out << "	standard deviation: " << sqrt(variance) << std::endl;
	}
	//print error statistics for all landmarks combined
	out << "All:" << std::endl;
	out << "	min: " << stat->getMin() << std::endl;
	out << "	max: " << stat->getMax() << std::endl;
	out << "	mean: " << stat->getMean() << std::endl;
	double variance = stat->getVariance();
	out << "	variance: " << variance << std::endl;
	out << "	standard deviation: " << sqrt(variance) << std::endl;
}
