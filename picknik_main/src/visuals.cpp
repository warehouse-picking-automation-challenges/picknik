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

// PickNik
#include <picknik_main/visuals.h>
#include <picknik_main/shelf.h>

// Parameter loading
#include <ros_param_shortcuts/ros_param_utilities.h>

namespace picknik_main
{

Visuals::Visuals(robot_model::RobotModelPtr robot_model, 
                 planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor)
  : nh_("~")
{
  // ------------------------------------------------------------------------------------------------------
  // Load the Robot Viz Tools for publishing to Rviz
  visual_tools_.reset(new mvt::MoveItVisualTools(robot_model->getModelFrame(), "/picknik_main/markers",
                                                 planning_scene_monitor));
  visual_tools_->loadRobotStatePub("/picknik_main/robot_state");
  visual_tools_->loadTrajectoryPub("/picknik_main/display_trajectory");
  visual_tools_->loadMarkerPub();
  visual_tools_->setAlpha(0.8);
  visual_tools_->hideRobot(); // show that things have been reset
  visual_tools_->deleteAllMarkers(); // clear all old markers
  visual_tools_->setManualSceneUpdating(true);

  // ------------------------------------------------------------------------------------------------------
  // Load the COLLISION Robot Viz Tools for publishing to Rviz
  visual_tools_display_.reset(new mvt::MoveItVisualTools(robot_model->getModelFrame(), "/picknik_main/highres_markers",
                                                         planning_scene_monitor));
  //visual_tools_display_->deleteAllMarkers(); // clear all old markers

  // ------------------------------------------------------------------------------------------------------
  // Load RobotState VisualTools for Start State
  start_state_.reset(new mvt::MoveItVisualTools(robot_model->getModelFrame(), "/picknik_main/grasping_markers",
                                                planning_scene_monitor));
  start_state_->loadMarkerPub();
  start_state_->loadRobotStatePub("/picknik_main/robot_start_state");
  ros::spinOnce();
  start_state_->deleteAllMarkers(); // clear all old markers
  start_state_->hideRobot(); // show that things have been reset
  grasp_markers_ = start_state_; // same object just renamed

  // ------------------------------------------------------------------------------------------------------
  // Load RobotState VisualTools for Goal State
  goal_state_.reset(new mvt::MoveItVisualTools(robot_model->getModelFrame(), "/picknik_main/trajectory_markers",
                                               planning_scene_monitor));
  goal_state_->loadMarkerPub();
  goal_state_->loadRobotStatePub("/picknik_main/robot_goal_state");
  ros::spinOnce();
  goal_state_->deleteAllMarkers(); // clear all old markers
  goal_state_->hideRobot(); // show that things have been reset
  trajectory_lines_ = goal_state_; // same object just renamed

  // ------------------------------------------------------------------------------------------------------
  // Load Product Perception VisualTools 
  //product_perception_.reset(new mvt::MoveItVisualTools(robot_model->getModelFrame(), "/picknik_main/product_perception",
  //                                             planning_scene_monitor));
  //product_perception_->deleteAllMarkers(); // clear all old markers

  // ------------------------------------------------------------------------------------------------------
  // Load TF Visual Tools
  tf_.reset(new rvt::TFVisualTools());

  // Load verbose/visualization settings
  const std::string parent_name = "visuals"; // for namespacing logging messages
  ros_param_utilities::getBoolMap(parent_name, nh_, "debug_level", enabled_);
}

bool Visuals::visualizeDisplayShelf(ShelfObjectPtr shelf)
{
  visual_tools_display_->deleteAllMarkers(); // clear all old markers
  visual_tools_display_->enableBatchPublishing(true);
  shelf->visualizeHighRes();
  shelf->visualizeAxis(shared_from_this());
  visual_tools_display_->triggerBatchPublishAndDisable();
  return true;
}

bool Visuals::setSharedRobotState(moveit::core::RobotStatePtr current_state)
{
  // allow visual_tools to have the correct virtual joint
  visual_tools_->getSharedRobotState() = current_state; 
  visual_tools_display_->getSharedRobotState() = current_state; 
  start_state_->getSharedRobotState() = current_state; 
  goal_state_->getSharedRobotState() = current_state; 
  return true;
}

bool Visuals::isEnabled(const std::string& setting_name)
{
  std::map<std::string,bool>::iterator it = enabled_.find(setting_name);
  if(it != enabled_.end())
  {
    // Element found;
    return it->second;
  }
  ROS_ERROR_STREAM_NAMED("visuals","isEnabled() key '" << setting_name << "' does not exist in the available configuration");
  return false;
}

} // end namespace
