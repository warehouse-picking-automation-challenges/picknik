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
  Desc:   Integrate feedback and command of a tactile sensor
*/

#ifndef PICKNIK_MAIN__TACTILE_FEEDBACK
#define PICKNIK_MAIN__TACTILE_FEEDBACK

// ROS
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

// PickNik
#include <picknik_main/namespaces.h>
#include <picknik_main/manipulation_data.h>

// Visual Tools
#include <rviz_visual_tools/rviz_visual_tools.h>

namespace picknik_main
{
static const std::string ATTACH_FRAME = "finger_sensor_pad";

/** \brief Names of data sent from Gelsight to rest of BLUE
           NOTE: this is copied from
           gelsight/include/gelsight/image_processing.hpp
 */
enum EndEffectorData
{
  SHEER_FORCE = 0,
  LINE_CENTER_X,
  LINE_CENTER_Y,
  LINE_EIGEN_VEC_X,
  LINE_EIGEN_VEC_Y,
  LINE_EIGEN_VAL,
  SHEER_DISPLACEMENT_X,
  SHEER_DISPLACEMENT_Y,
  SHEER_TORQUE,
  IMAGE_HEIGHT,
  IMAGE_WIDTH,
  ALWAYS_AT_END  // for counting array size
};

class TactileFeedback
{
public:
  /**
   * \brief Constructor
   */
  TactileFeedback(ManipulationDataPtr config);

  /** \brief Send command to remote sensor to reset itself */
  void recalibrateTactileSensor();

  double getSheerTheta() { return sheer_theta_; };
  double getSheerForce() { return end_effector_data_cached_[SHEER_FORCE]; };
  double getSheerTorque() { return end_effector_data_cached_[SHEER_TORQUE]; };
  void setEndEffectorDataCallback(std::function<void()> function)
  {
    end_effector_data_callback_ = function;
  };

private:
  void dataCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);

  void displayLineDirection();

  void displaySheerForce();

  void publishUpdatedLine(geometry_msgs::Point& pt1, geometry_msgs::Point& pt2);

  void convertPixelToMeters(geometry_msgs::Pose& pose, int height, int width);

  // A shared node handle
  ros::NodeHandle nh_;

  // Listen to incoming feedback from sensor
  ros::Subscriber end_effector_data_sub_;

  // Publish commands to re-calibrate sensor
  ros::Publisher tactile_calibration_pub_;

  double sheer_theta_;
  std::vector<double> end_effector_data_cached_;

  // Allow a callback to be added whenever new end effector data is recieved
  std::function<void()> end_effector_data_callback_;

  // Show data
  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

};  // end class

// Create boost pointers for this class
typedef boost::shared_ptr<TactileFeedback> TactileFeedbackPtr;

}  // end namespace

#endif
