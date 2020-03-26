#ifndef __BONE_STATISTICS_HPP__
#define __BONE_STATISTICS_HPP__

#include <vector>
#include <map>
#include <string>
#include <boost/property_tree/ptree.hpp>

#include "AbstractStatistics.hpp"

/**
 * Class that can collect error informations from output files generated during the atlas fitting.
 * With this informations some statistics can be calculated, the data will be grouped by bone names.
 */
class BoneStatistics : public AbstractStatistics {
	public:
		/**
		 * Creates the statistic object.
		 * Contains a map with all error informations grouped by bone name.
		 */
		BoneStatistics();
		
		/**
		 * Frees all error information
		 */
		~BoneStatistics();
		
		/**
		 * Adds the error data containing in the given property tree to the data on which the statistics
		 * are build.
		 * Specifies the grouping by bone name.
		 * @param data		property tree containing the error data
		 */
		virtual void addData(boost::property_tree::ptree data);	
		
	private:
		/**
		 * Gets the name of the bone out of the given property tree.
		 * If no name is given a generated name is returned.
		 * @param		property tree that contains the bone name
		 * @return the bone name
		 */
		std::string getBoneName(boost::property_tree::ptree data);
};

#endif
