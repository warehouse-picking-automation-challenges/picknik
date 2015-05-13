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

#ifndef PICKNIK_MAIN__VISUALS
#define PICKNIK_MAIN__VISUALS

// ROS
#include <ros/ros.h>

// Visual Tools
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rviz_visual_tools/tf_visual_tools.h>

// PickNik
#include <picknik_main/namespaces.h>

// Boost
#include <boost/enable_shared_from_this.hpp>

namespace picknik_main
{

MOVEIT_CLASS_FORWARD(ShelfObject);

class Visuals : public boost::enable_shared_from_this<Visuals>
{
public:

  /**
   * \brief Constructor
   * \param verbose - run in debug mode
   */
  Visuals(moveit::core::RobotModelPtr robot_model, 
          planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor);

  bool visualizeDisplayShelf(ShelfObjectPtr shelf);

  /**
   * \brief Allow visual_tools to have the correct virtual joint
   * \return true on success
   */
  bool setSharedRobotState(moveit::core::RobotStatePtr current_state);

  /**
   * \brief Check if certain key is enabled
   * \return true if enabled
   */
  bool isEnabled(const std::string& setting_name);

public:

  // Public vars
  mvt::MoveItVisualToolsPtr visual_tools_;
  mvt::MoveItVisualToolsPtr visual_tools_display_;
  mvt::MoveItVisualToolsPtr start_state_; // also used for grasp markers
  mvt::MoveItVisualToolsPtr goal_state_; // also used for trajectory lines
  mvt::MoveItVisualToolsPtr grasp_markers_; // also used for start state
  mvt::MoveItVisualToolsPtr trajectory_lines_; // also used for goal state
  mvt::MoveItVisualToolsPtr product_perception_; // for bounding boxes
  rvt::TFVisualToolsPtr tf_; // for debugging transforms

private:

  // Visualization settings
  std::map<std::string, bool> enabled_;

  // A shared node handle
  ros::NodeHandle nh_;

}; // end class

// Create boost pointers for this class
typedef boost::shared_ptr<Visuals> VisualsPtr;

} // end namespace

#endif
