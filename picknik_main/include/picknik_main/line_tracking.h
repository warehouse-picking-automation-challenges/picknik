/*********************************************************************
  * Software License Agreement (BSD License)
  *
  *  Copyright (c) 2015, Dave Coleman <dave@dav.ee>
  *  All rights reserved.
  *
  * Unauthorized copying of this file, via any medium is strictly prohibited
  * Proprietary and confidential
  *********************************************************************/
/*
  Author: Dave Coleman <dave@dav.ee>
  Desc:   Holder for multiple visuals tools
*/

#ifndef PICKNIK_MAIN__LINE_TRACKING
#define PICKNIK_MAIN__LINE_TRACKING

// ROS
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

// PickNik
#include <picknik_main/namespaces.h>
#include <picknik_main/visuals.h>

namespace picknik_main
{

static const std::string ATTACH_FRAME = "finger_sensor_pad";

class LineTracking
{
public:

  /**
   * \brief Constructor
   */
  LineTracking(VisualsPtr visuals);

  void dataCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);

  void convertPixelToMeters(double &x, double &y);
  
  void publishUpdatedLine(geometry_msgs::Point& pt1, geometry_msgs::Point& pt2);

  void getToolDirection(const geometry_msgs::PoseStamped& center_point,  double theta);
  
private:

  // A shared node handle
  ros::NodeHandle nh_;

  ros::Subscriber end_effector_data_sub_;

  VisualsPtr visuals_;

}; // end class

// Create boost pointers for this class
typedef boost::shared_ptr<LineTracking> LineTrackingPtr;

} // end namespace

#endif
