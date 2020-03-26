#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <map>
#include <utility>

#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/optional.hpp>
#include <boost/lexical_cast.hpp>

/**
 * Runs the atlas creation and the atlas fitting with the given configuration files and returns
 * the squared error after the fitting.
 * To get this function working, the json output path in the fitting configuration file has to be set
 * and the control landmarks have to be contained in the fitting configuration.
 * @param 
 * @param 
 * @return the squared error of the fitting
 */
double testConfiguration(std::string build_config_path, std::string fit_config_path);

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
	
	
	std::string build_config_path = config.get<std::string>("build_config_path");
	std::string fit_config_path = config.get<std::string>("fit_config_path");
	
	boost::property_tree::ptree build_config;
	try{
		boost::property_tree::read_json(build_config_path, build_config);
	}catch( boost::property_tree::json_parser::json_parser_error &e){
		std::cout << "Configuration file can not be read..." << std::endl;
		std::cout << "Message: " << e.message() << std::endl;
		std::cout << "File-name: " << e.filename() << std::endl;
		std::cout << "Line: " << e.line() << std::endl;
		std::cout << "...exiting." << std::endl;
		exit(-1);
	}
	
	boost::property_tree::ptree fit_config;
	try{
		boost::property_tree::read_json(fit_config_path, fit_config);
	}catch( boost::property_tree::json_parser::json_parser_error &e){
		std::cout << "Configuration file can not be read..." << std::endl;
		std::cout << "Message: " << e.message() << std::endl;
		std::cout << "File-name: " << e.filename() << std::endl;
		std::cout << "Line: " << e.line() << std::endl;
		std::cout << "...exiting." << std::endl;
		exit(-1);
	}
	
	int type;
	std::string type_s = config.get<std::string>("type");
	std::string side_s = config.get<std::string>("side");
	std::string folder;
	if(type_s == "radius"){
		if(side_s == "r"){
			type = 1;
			folder = build_config.get<std::string>("parameters.radius_right_folder", "datadir/radius/right");
		} else if (side_s == "l"){
			type = 2;
			folder = build_config.get<std::string>("parameters.radius_left_folder", "datadir/radius/left");
		} else {
			std::cout << "The side has to be r or l...exiting" << std::endl;
			exit(-1);
		}
	} else if (type_s == "ulna"){
		if(side_s == "r"){
			type = 3;
			folder = build_config.get<std::string>("parameters.ulna_right_folder", "datadir/ulna/right");
		} else if (side_s == "l"){
			type = 4;
			folder = build_config.get<std::string>("parameters.ulna_left_folder", "datadir/ulna/left");
		} else {
			std::cout << "The side has to be r or l...exiting" << std::endl;
			exit(-1);
		}
	} else {
		std::cout << "The bone type has to be radius or ulna...exiting" << std::endl;
		exit(-1);
	}
	
	int gp_sigma_min = config.get<int>("gp_sigma_min");
	int gp_sigma_max = config.get<int>("gp_sigma_max");
	int gp_sigma_step = config.get<int>("gp_sigma_step");
	
	int at_sigma_min = config.get<int>("at_sigma_min");
	int at_sigma_max = config.get<int>("at_sigma_max");
	int at_sigma_step = config.get<int>("at_sigma_step");
	
	int levels_min = config.get<int>("levels_min");
	int levels_max = config.get<int>("levels_max");
	int levels_step = config.get<int>("levels_step");
	
	boost::filesystem::create_directories("atlas_tune");
	
	//total number of tries, used to give each folder a different name
	int try_num = 0;
	
	//map to store the results
	std::map<double, std::pair<std::string, std::string> > results;
	
	for(int sigma = gp_sigma_min; sigma < gp_sigma_max; sigma+=gp_sigma_step){
	
		//set parameter for the current iteration
		build_config.put("parameters.kernel_sigma", sigma);
		
		for(int levels = levels_min; levels < levels_max; levels+=levels_step){
		
			//erase not required parameters
			build_config.erase("parameters.gauss_path_radius_r");
			build_config.erase("parameters.gauss_path_radius_l");
			build_config.erase("parameters.gauss_path_ulna_r");
			build_config.erase("parameters.gauss_path_ulna_l");
			
			//set parameters for the current iteration
			build_config.put("parameters.kernel_levels", levels);
			build_config.put("parameters.atlas_sigma", at_sigma_min);
			
			//create directory
			boost::filesystem::create_directories(folder + "/atlas_tune" + boost::lexical_cast<std::string>(try_num));
			
			switch(type){
				case 1:
				build_config.put("parameters.radius_right_folder", folder + "/atlas_tune" + boost::lexical_cast<std::string>(try_num));
				break;
			case 2:
				build_config.put("parameters.radius_left_folder", folder + "/atlas_tune" + boost::lexical_cast<std::string>(try_num));
				break;
			case 3:
				build_config.put("parameters.ulna_right_folder", folder + "/atlas_tune" + boost::lexical_cast<std::string>(try_num));
				break;
			case 4:
				build_config.put("parameters.ulna_left_folder", folder + "/atlas_tune" + boost::lexical_cast<std::string>(try_num));
				break;
			}
			
			//create build configuration file
			std::string path("atlas_tune/build" + boost::lexical_cast<std::string>(try_num) + ".json");
			boost::property_tree::write_json(path, build_config);
			
			//set the atlas path
			fit_config.put("atlas_path", folder + "/atlas_tune" + boost::lexical_cast<std::string>(try_num) + "/enhancedStatisticalModel.h5");
			fit_config.put("json_output_path", folder + "/atlas_tune" + boost::lexical_cast<std::string>(try_num) + "/output.json");
			fit_config.put("vtk_output_path", folder + "/atlas_tune" + boost::lexical_cast<std::string>(try_num) + "/output.vtk");
			fit_config.put("scalismo_gui_path", folder + "/atlas_tune" + boost::lexical_cast<std::string>(try_num) + "/output.txt");
			
			//create fit configuration file
			std::string path2("atlas_tune/fit" + boost::lexical_cast<std::string>(try_num) + ".json");
			boost::property_tree::write_json(path2, fit_config);
			
			//calculate results and add them to the result map
			results.insert(std::pair<double, std::pair<std::string, std::string> >(testConfiguration(path, path2), std::pair<std::string, std::string>(path, path2)));

			//path of the created Gaussian process model, can be reused in the next for loop, so it has not to be created each time
			std::string gauss_folder = "/atlas_tune" + boost::lexical_cast<std::string>(try_num);
			
			++try_num;
			
			for(int atlas_sigma = at_sigma_min + at_sigma_step; atlas_sigma < at_sigma_max; atlas_sigma+=at_sigma_step){
				//set parameter for the current iteration
				build_config.put("parameters.atlas_sigma", atlas_sigma);
			
				//create directory
				boost::filesystem::create_directories(folder + "/atlas_tune" + boost::lexical_cast<std::string>(try_num));
			
				switch(type){
					case 1:
						build_config.put("parameters.radius_right_folder", folder + "/atlas_tune" + boost::lexical_cast<std::string>(try_num));
						build_config.put("parameters.gauss_path_radius_r", folder + gauss_folder + "/gaussModel.h5");
						break;
					case 2:
						build_config.put("parameters.radius_left_folder", folder + "/atlas_tune" + boost::lexical_cast<std::string>(try_num));
						build_config.put("parameters.gauss_path_radius_l", folder + gauss_folder + "/gaussModel.h5");
						break;
					case 3:
						build_config.put("parameters.ulna_right_folder", folder + "/atlas_tune" + boost::lexical_cast<std::string>(try_num));
						build_config.put("parameters.gauss_path_ulna_r", folder + gauss_folder + "/gaussModel.h5");	
						break;
					case 4:
						build_config.put("parameters.ulna_left_folder", folder + "/atlas_tune" + boost::lexical_cast<std::string>(try_num));
						build_config.put("parameters.gauss_path_ulna_l", folder + gauss_folder + "/gaussModel.h5");	
						break;
				}
				//create build configuration file
				path = "atlas_tune/build" + boost::lexical_cast<std::string>(try_num) + ".json";
				boost::property_tree::write_json(path, build_config);
			
				//set the atlas path
				fit_config.put("atlas_path", folder + "/atlas_tune" + boost::lexical_cast<std::string>(try_num) + "/enhancedStatisticalModel.h5");
				fit_config.put("json_output_path", folder + "/atlas_tune" + boost::lexical_cast<std::string>(try_num) + "/output.json");
				fit_config.put("vtk_output_path", folder + "/atlas_tune" + boost::lexical_cast<std::string>(try_num) + "/output.vtk");
				fit_config.put("scalismo_gui_path", folder + "/atlas_tune" + boost::lexical_cast<std::string>(try_num) + "/output.txt");
		
				//create fit configuration file
				path2 = "atlas_tune/fit" + boost::lexical_cast<std::string>(try_num) + ".json";
				boost::property_tree::write_json(path2, fit_config);
			
				//calculate results and add them to the result map
				results.insert(std::pair<double, std::pair<std::string, std::string> >(testConfiguration(path, path2), std::pair<std::string, std::string>(path, path2)));
				
				++try_num;
			}
		}
	}
	

	
	double min_error = std::numeric_limits<double>::max();
	std::pair<std::string, std::string> min_config;
	std::ofstream outfile ("atlas_tune/results.csv");
	outfile << "Squared Error;Kernel Sigma;Kernel Levels;Atlas Sigma" << std::endl;
	for (std::map<double, std::pair<std::string, std::string> >::iterator it = results.begin(); it != results.end(); ++it) {
		if(it->first < min_error){
			min_error = it->first;
			min_config = it->second;
		}
		
		boost::property_tree::ptree config;
		try{
		boost::property_tree::read_json(it->second.first, config);
		}catch( boost::property_tree::json_parser::json_parser_error &e){
			std::cout << "Configuration file can not be read..." << std::endl;
			std::cout << "Message: " << e.message() << std::endl;
			std::cout << "File-name: " << e.filename() << std::endl;
			std::cout << "Line: " << e.line() << std::endl;
			std::cout << "...exiting." << std::endl;
			exit(-1);
		}
		
		double build_sigma = config.get<double>("parameters.kernel_sigma");
		int build_levels = config.get<int>("parameters.kernel_levels");
		double atlas_sigma = config.get<double>("parameters.atlas_sigma");
		std::string er = boost::lexical_cast<std::string>(it->first);
		boost::replace_all(er, ".", ","); //as excel uses , as comma replace the . with a ,
		outfile << er << ";" << build_sigma << ";" << build_levels << ";" << atlas_sigma << std::endl;
	}
	outfile.close();
	
	std::cout << std::endl << std::endl << std::endl << "Best configuration: create:" << min_config.first << " fit:" << min_config.second << std::endl;
	
	return EXIT_SUCCESS;
}

double testConfiguration(std::string build_config_path, std::string fit_config_path){
	std::string command1 = "./AtlasCreation " + build_config_path;
	std::string command2 = "./FitAtlas " + fit_config_path;
	
	//open the fitting configuration to get the output path
	boost::property_tree::ptree fit_config;
	try{
		boost::property_tree::read_json(fit_config_path, fit_config);
	}catch( boost::property_tree::json_parser::json_parser_error &e){
		std::cout << "Configuration file can not be read..." << std::endl;
		std::cout << "Message: " << e.message() << std::endl;
		std::cout << "File-name: " << e.filename() << std::endl;
		std::cout << "Line: " << e.line() << std::endl;
		std::cout << "...exiting." << std::endl;
		exit(-1);
	}
	std::string result_path = fit_config.get<std::string>("json_output_path");
	
	//call the atlas generation
	if(std::system(command1.c_str()) != EXIT_SUCCESS){
		return std::numeric_limits<double>::max();
	}
	
	//call the atlas fitting
	if(std::system(command2.c_str()) != EXIT_SUCCESS){
		return std::numeric_limits<double>::max();
	}
	
	//open the result json
	boost::property_tree::ptree fit_res;
	try{
		boost::property_tree::read_json(result_path, fit_res);
	}catch( boost::property_tree::json_parser::json_parser_error &e){
		std::cout << "Configuration file can not be read..." << std::endl;
		std::cout << "Message: " << e.message() << std::endl;
		std::cout << "File-name: " << e.filename() << std::endl;
		std::cout << "Line: " << e.line() << std::endl;
		std::cout << "...exiting." << std::endl;
		exit(-1);
	}
	
	//return the squared error
	return fit_res.get<double>("squared_error_sum");
}
