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

namespace picknik_main
{

Visuals::Visuals(robot_model::RobotModelPtr robot_model, planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor)
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

  // ------------------------------------------------------------------------------------------------------
  // Load RobotState VisualTools for Goal State
  goal_state_.reset(new mvt::MoveItVisualTools(robot_model->getModelFrame(), "/picknik_main/trajectory_markers",
                                               planning_scene_monitor));
  goal_state_->loadMarkerPub();
  goal_state_->loadRobotStatePub("/picknik_main/robot_goal_state");
  ros::spinOnce();
  goal_state_->deleteAllMarkers(); // clear all old markers
  goal_state_->hideRobot(); // show that things have been reset

}

bool Visuals::visualizeDisplayShelf(ShelfObjectPtr shelf)
{
  visual_tools_display_->deleteAllMarkers(); // clear all old markers
  visual_tools_display_->enableBatchPublishing(true);
  shelf->visualize();
  shelf->visualizeAxis(shared_from_this());
  visual_tools_display_->triggerBatchPublishAndDisable();
  return true;
}



} // end namespace
