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

#ifndef BAXTER_APC_MAIN__VISUALS
#define BAXTER_APC_MAIN__VISUALS

// ROS
#include <ros/ros.h>

// MoveIt
#include <moveit_visual_tools/moveit_visual_tools.h>

namespace baxter_apc_main
{

class Visuals
{
public:

  /**
   * \brief Constructor
   * \param verbose - run in debug mode
   */
  Visuals(robot_model::RobotModelPtr robot_model, planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor)
  {
    // ------------------------------------------------------------------------------------------------------
    // Load the Robot Viz Tools for publishing to Rviz
    visual_tools_.reset(new mvt::MoveItVisualTools(robot_model->getModelFrame(), "/baxter_apc_main/markers", 
                                                   planning_scene_monitor));
    visual_tools_->loadRobotStatePub("/baxter_apc_main/robot_state");
    visual_tools_->loadTrajectoryPub("/baxter_apc_main/display_trajectory");
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
    start_state_->loadRobotStatePub("/baxter_apc_main/robot_start_state");
    start_state_->hideRobot(); // show that things have been reset

    // ------------------------------------------------------------------------------------------------------
    // Load RobotState VisualTools for Goal State
    goal_state_.reset(new mvt::MoveItVisualTools(robot_model->getModelFrame(), "/amazon_start_state_markers", 
                                                 planning_scene_monitor));
    goal_state_->loadRobotStatePub("/baxter_apc_main/robot_start_state");
    goal_state_->hideRobot(); // show that things have been reset

  }

  /**
   * \brief Destructor
   */
  ~Visuals()
  {
  }

  // Public vars
  mvt::MoveItVisualToolsPtr visual_tools_;
  mvt::MoveItVisualToolsPtr visual_tools_display_;
  mvt::MoveItVisualToolsPtr start_state_;
  mvt::MoveItVisualToolsPtr goal_state_;

}; // end class

// Create boost pointers for this class
typedef boost::shared_ptr<Visuals> VisualsPtr;
typedef boost::shared_ptr<const Visuals> VisualsConstPtr;

} // end namespace

#endif
