#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/optional.hpp>
#include <boost/lexical_cast.hpp>

void generateTests(boost::property_tree::ptree parameterConfig, std::vector<boost::property_tree::ptree> bones, int type, std::vector<std::string>& run_scripts);

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
	
	boost::property_tree::ptree parameterConfig = config.get_child("parameters");
	
	//split bones into vectors
	std::vector<boost::property_tree::ptree> v_radii_r, v_radii_l, v_ulnae_r, v_ulnae_l;
	boost::property_tree::ptree bonesConfig = config.get_child("bones");
	boost::optional<boost::property_tree::ptree &> radii_tree = bonesConfig.get_child_optional("radius");
	if(radii_tree){
		for (boost::property_tree::ptree::iterator it = radii_tree.get().begin(); it != radii_tree.get().end(); ++it) {
			boost::property_tree::ptree radius = it->second;
			if(radius.get<std::string>("side") == "r"){
				v_radii_r.push_back(radius);
			} else {
				v_radii_l.push_back(radius);
			}
		}
	}
	boost::optional<boost::property_tree::ptree &> ulnae_tree = bonesConfig.get_child_optional("ulna");
	if(ulnae_tree){
		for (boost::property_tree::ptree::iterator it = ulnae_tree.get().begin(); it != ulnae_tree.get().end(); ++it) {
			boost::property_tree::ptree ulna = it->second;
			if(ulna.get<std::string>("side") == "r"){
				v_ulnae_r.push_back(ulna);
			} else {
				v_ulnae_l.push_back(ulna);
			}
		}
	}
	std::vector<std::string> run_scripts;
	generateTests(parameterConfig, v_radii_r, 0, run_scripts);
	generateTests(parameterConfig, v_radii_l, 1, run_scripts);
	generateTests(parameterConfig, v_ulnae_r, 2, run_scripts);
	generateTests(parameterConfig, v_ulnae_l, 3, run_scripts);
	
	
	std::string sript_path = "./runAll.sh";
	std::ofstream outfile (sript_path.c_str());
	for(int i = 0; i < run_scripts.size(); ++i){
		outfile << run_scripts[i] << std::endl;
	}
	outfile.close();
	return EXIT_SUCCESS;
}

void generateTests(boost::property_tree::ptree parameterConfig, std::vector<boost::property_tree::ptree> bones, int type, std::vector<std::string>& run_scripts){
	std::string path;
	if (bones.size() < 1){
		return;
	}
	
	switch (type){
		case 0:
			path = parameterConfig.get<std::string>("radius_right_folder", "./radius/right");
			break;
		case 1:
			path = parameterConfig.get<std::string>("radius_left_folder", "./radius/left");
			break;
		case 2:
			path = parameterConfig.get<std::string>("ulna_right_folder", "./ulna/right");
			break;
		case 3:
			path = parameterConfig.get<std::string>("ulna_left_folder", "./ulna/left");
			break;
	}
	//generate a test for each bone
	for(int i = 0; i < bones.size(); ++i){
		//adjust the path
		std::string sub_path = path + "/" + boost::lexical_cast<std::string>(i);
		boost::filesystem::create_directories(sub_path);
		sub_path = boost::filesystem::canonical(boost::filesystem::path(sub_path)).string();
		switch (type){
			case 0:
				parameterConfig.put("radius_right_folder", sub_path);
				parameterConfig.erase("radius_left_folder");
				parameterConfig.erase("ulna_right_folder");
				parameterConfig.erase("ulna_left_folder");
				break;
			case 1:
				parameterConfig.put("radius_left_folder", sub_path);
				parameterConfig.erase("radius_right_folder");
				parameterConfig.erase("ulna_right_folder");
				parameterConfig.erase("ulna_left_folder");
				break;
			case 2:
				parameterConfig.put("ulna_right_folder", sub_path);
				parameterConfig.erase("radius_right_folder");
				parameterConfig.erase("radius_left_folder");
				parameterConfig.erase("ulna_left_folder");
				break;
			case 3:
				parameterConfig.put("ulna_left_folder", sub_path);
				parameterConfig.erase("radius_right_folder");
				parameterConfig.erase("radius_left_folder");
				parameterConfig.erase("ulna_right_folder");
				break;
		}
		
		//all other bones are used as training data
		boost::property_tree::ptree bone_tree;
		for(int j = 0; j < bones.size(); ++j){
			if(i != j){
				boost::property_tree::ptree bone = bones[j];
				boost::filesystem::path path = bone.get<std::string>("path");
				path = boost::filesystem::canonical(path);
				bone.put("path", path.string());
				bone_tree.push_back(std::make_pair("", bone));
			}
		}
		
		//build atlas creation configuration out of its parts
		boost::property_tree::ptree atlasConfiguration, boneTypeTree;
		atlasConfiguration.add_child("parameters", parameterConfig);
		if(type == 0 || type == 1){
			boneTypeTree.add_child("radius", bone_tree);
		}
		if(type == 2 || type == 3){
			boneTypeTree.add_child("ulna", bone_tree);
		}
		atlasConfiguration.add_child("bones", boneTypeTree);
		
		//save the atlas creation configuration file
		std::string atlasConfigurationPath = sub_path + "/build.json";
		boost::property_tree::write_json(atlasConfigurationPath, atlasConfiguration);
		
		//create atlas fit configuration
		boost::property_tree::ptree fitConfiguration;
		fitConfiguration.put("atlas_path", sub_path + "/enhancedStatisticalModel.h5");
		boost::filesystem::path tmp(bones[i].get<std::string>("path"));
		tmp = boost::filesystem::canonical(tmp);
		fitConfiguration.put("patient_path", tmp.string());
		if(type == 0 || type == 1){
			fitConfiguration.put("type", "radius");
		}
		if(type == 2 || type == 3){
			fitConfiguration.put("type", "ulna");
		}
		fitConfiguration.put("align_iter", parameterConfig.get<std::string>("rigid_iter"));
		fitConfiguration.put("fit_iter", parameterConfig.get<std::string>("gauss_fit_iter"));
		fitConfiguration.put("vtk_output_path", sub_path);
		fitConfiguration.put("json_output_path", sub_path + "/output.json");
		fitConfiguration.put("scalismo_gui_path", sub_path + "/output.txt");
		boost::property_tree::ptree landmarks = bones[i].get_child("landmarks");
		fitConfiguration.add_child("landmarks", landmarks);
		
		//save the atlas fit configuration file
		std::string atlasFitPath = sub_path + "/fit.json";
		boost::property_tree::write_json(atlasFitPath, fitConfiguration);
		
		//get paths
		boost::filesystem::path atlascreation("./AtlasCreation");
		boost::filesystem::path fitatlas("./FitAtlas");
		boost::filesystem::path creationConfig(sub_path + "/build.json");
		boost::filesystem::path fitConfig(sub_path + "/fit.json");
		boost::filesystem::path absolut_atlas = boost::filesystem::canonical(atlascreation);
		boost::filesystem::path absolut_fit = boost::filesystem::canonical(fitatlas);
		
		//write script file
		std::string sript_path = sub_path + "/run.sh";
		std::ofstream outfile (sript_path.c_str());
		outfile << absolut_atlas.string() << " " + sub_path + "/build.json" << std::endl;
		outfile << absolut_fit.string() << " " + sub_path + "/fit.json" << std::endl;
		outfile.close();
		run_scripts.push_back(sript_path);
	}
}
