#ifndef __LANDMARK_STATISTICS_HPP__
#define __LANDMARK_STATISTICS_HPP__

#include <vector>
#include <map>
#include <string>
#include <boost/property_tree/ptree.hpp>

#include "AbstractStatistics.hpp"

/**
 * Class that can collect error informations from output files generated during the atlas fitting.
 * With this informations some statistics can be calculated, the data will be grouped by landmark names.
 */
class LandmarkStatistics : public AbstractStatistics {
	public:
		/**
		 * Creates the statistic object.
		 * Contains a map with all error informations grouped by landmark name.
		 */
		LandmarkStatistics();
		
		/**
		 * Frees all error information
		 */
		~LandmarkStatistics();
		
		/**
		 * Adds the error data containing in the given property tree to the data on which the statistics
		 * are build.
		 * Specifies the grouping by landmark name.
		 * @param data		property tree containing the error data
		 */
		virtual void addData(boost::property_tree::ptree data);		
		
	private:		
		/**
		 * Adds a single error value to the group with the given name,
		 * if the group currently not exists a new group will be created.
		 * @param key		name of the landmark
		 * @param value		error value
		 */
		void addSingleData(std::string key, double value);
};

#endif
