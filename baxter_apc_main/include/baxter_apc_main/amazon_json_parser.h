/*********************************************************************
 * Software License Agreement
 *
 *  Copyright (c) 2015, Dave Coleman <dave@dav.ee>
 *  All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *********************************************************************/

/* Author: Dave Coleman <dave@dav.ee>
   Desc:
*/

#ifndef BAXTER_APC_MAIN__AMAZON_JSON_PARSER
#define BAXTER_APC_MAIN__AMAZON_JSON_PARSER

// ROS
#include <ros/ros.h>

// JSON
#include <baxter_apc_main/json/json.h>
#include <baxter_apc_main/shelf.h>

// MoveIt
#include <moveit_visual_tools/moveit_visual_tools.h>

namespace baxter_apc_main
{

class AmazonJSONParser
{
private:

  // A shared node handle
  ros::NodeHandle nh_;

  // Show more visual and console output, with general slower run time.
  bool verbose_;

  // For visualizing things in rviz
  VisualsPtr visuals_;

public:

  /**
   * \brief Constructor
   * \param verbose - run in debug mode
   */
  AmazonJSONParser(bool verbose, VisualsPtr visuals);

  /**
   * \brief Destructor
   */
  ~AmazonJSONParser();

  /**
   * \brief Parse an input JSON file
   * \return true on success
   */
  bool parse(const std::string& file_path, const std::string& package_path, ShelfObjectPtr shelf, WorkOrders& orders);
  bool parseBins(const std::string& package_path, const Json::Value bin_contents, ShelfObjectPtr shelf);
  bool parseWorkOrders(const Json::Value work_orders, ShelfObjectPtr shelf, WorkOrders& orders);

}; // end class

// Create boost pointers for this class
typedef boost::shared_ptr<AmazonJSONParser> AmazonJSONParserPtr;
typedef boost::shared_ptr<const AmazonJSONParser> AmazonJSONParserConstPtr;

} // end namespace

#endif
