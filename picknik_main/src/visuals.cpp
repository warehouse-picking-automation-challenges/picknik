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

#include <picknik_main/visuals.h>

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
  visual_tools_display_.reset(new mvt::MoveItVisualTools(robot_model->getModelFrame(), "/amazon_shelf_markers_display",
                                                         planning_scene_monitor));
  visual_tools_display_->deleteAllMarkers(); // clear all old markers

  // ------------------------------------------------------------------------------------------------------
  // Load RobotState VisualTools for Start State
  start_state_.reset(new mvt::MoveItVisualTools(robot_model->getModelFrame(), "/amazon_start_state_markers",
                                                planning_scene_monitor));
  start_state_->loadRobotStatePub("/picknik_main/robot_start_state");
  start_state_->hideRobot(); // show that things have been reset

  // ------------------------------------------------------------------------------------------------------
  // Load RobotState VisualTools for Goal State
  goal_state_.reset(new mvt::MoveItVisualTools(robot_model->getModelFrame(), "/amazon_goal_state_markers",
                                               planning_scene_monitor));
  goal_state_->loadRobotStatePub("/picknik_main/robot_goal_state");
  goal_state_->hideRobot(); // show that things have been reset

}


} // end namespace
