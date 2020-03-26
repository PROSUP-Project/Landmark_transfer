#ifndef __ABSTRACT_STATISTICS_HPP__
#define __ABSTRACT_STATISTICS_HPP__

#include <vector>
#include <map>
#include <string>
#include <boost/property_tree/ptree.hpp>

/**
 * Class that can collect error informations from output files generated during the atlas fitting.
 * With this informations some statistics can be calculated, the data will be grouped by landmark names.
 */
class AbstractStatistics{
	public:
		/**
		 * Creates the statistic object.
		 * Contains a map with all error informations grouped by landmark name.
		 */
		AbstractStatistics();
		
		/**
		 * Frees all error information
		 */
		~AbstractStatistics();
		
		/**
		 * Opens the file with the given path and reads the error data from it,
		 * this data is than added to the map containing the error data for statistic
		 * calculations.
		 * @note calls addData with the property tree read from the given path.
		 * @param data_path		a path to a json file containing error informations that should be analysed
		 */
		void addDataFromFile(const std::string& data_path);
		
		/**
		 * Adds the error data containing in the given property tree to the data on which the statistics
		 * are build.
		 * This function does the grouping of the data.
		 * @param data		property tree containing the error data
		 */
		virtual void addData(boost::property_tree::ptree data) = 0;
		
		/**
		 * Returns the minimal error in the group with the given name.
		 * @param key	name of the group which minimal error should be returned
		 * @return the minimal error
		 */
		double getMin(std::string key);
		
		/**
		 * Returns the minimal error of all groups.
		 * @return the minimal error
		 */
		double getMin();
		
		/**
		 * Returns the maximal error in the group with the given name.
		 * @param key	name of the group which maximal error should be returned
		 * @return the maximal error
		 */
		double getMax(std::string key);
		
		/**
		 * Returns the maximal error of all groups.
		 * @return the maximal error
		 */
		double getMax();
		
		/**
		 * Returns the mean of the error in the group with the given name.
		 * @param key	name of the group which mean should be returned
		 * @return the error mean
		 */
		double getMean(std::string key);
		
		/**
		 * Returns the mean of errors of all groups.
		 * @return the error mean
		 */
		double getMean();
		
		/**
		 * Returns the variance of the error in the group with the given name.
		 * @param key	name of the group which variance should be returned
		 * @return the error variance
		 */
		double getVariance(std::string key);
		
		/**
		 * Returns the variance of all groups.
		 * @return the error variance
		 */
		double getVariance();
		
		/**
		 * Returns a vector containing all names of groups
		 * from which error data is stored
		 * @return vector of group names
		 */
		std::vector<std::string> getKeys();
		
		/**
		 * Returns a string in R syntax to create a table containing all the error data.
		 * @param name	the name of the table
		 * @param names	vector with the names of the vectors in the R code, default are the key names
		 * @return string in R syntax
		 */
		std::string getRTable(const std::string &name, std::vector<std::string>* names = nullptr);
		
	protected:
		//map to store all error information grouped into vectors with group names
		std::map<std::string, std::vector<double>* > dataMap;
		
		/**
		 * Returns a pointer to the vector containing the error data
		 * of the group with the given name.
		 * @param key	name of the group which data should be returned
		 * @return a pointer to the vector containing the error data
		 * @warning do not free this data, it is a pointer to the vector in the map
		 */
		std::vector<double>* getVector(std::string key);
		
		/**
		 * Generates a vector containing the error data of all groups combined.
		 * @return a pointer to the vector
		 * @warning has to be freed manually
		 */
		std::vector<double>* getVector();
		
		/**
		 * Returns the minimal error stored in the given vector.
		 * @param v		vector containing the error data
		 * @return the minimal error
		 */
		double getMin(std::vector<double>* v);
		
		/**
		 * Returns the maximal error stored in the given vector.
		 * @param v		vector containing the error data
		 * @return the maximal error
		 */
		double getMax(std::vector<double>* v);
		
		/**
		 * Returns the mean error of the error data stored in the given vector.
		 * @param v		vector containing the error data
		 * @return the mean error
		 */
		double getMean(std::vector<double>* v);
		
		/**
		 * Returns the variance of the errors contained in the given vector.
		 * @param v		vector containing the error data
		 * @return the error variance
		 */
		double getVariance(std::vector<double>* v);
		
		/**
		 * Returns a string that is written in R syntax to create a vector in R.
		 * @param v		the vector from which data the string should be created
		 * @param name	the name that the vector in R should have
		 * @return a string in R syntax that represents the given vector
		 */
		std::string getRVector(std::vector<double>* v, const std::string &name);
};

#endif
