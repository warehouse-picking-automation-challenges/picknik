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

// MoveIt
#include <moveit_visual_tools/moveit_visual_tools.h>

// PickNik
#include <picknik_main/namespaces.h>

namespace picknik_main
{

class Visuals
{
public:

  /**
   * \brief Constructor
   * \param verbose - run in debug mode
   */
  Visuals(robot_model::RobotModelPtr robot_model, planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor);

  // Public vars
  mvt::MoveItVisualToolsPtr visual_tools_;
  mvt::MoveItVisualToolsPtr visual_tools_display_;
  mvt::MoveItVisualToolsPtr start_state_;
  mvt::MoveItVisualToolsPtr goal_state_;

}; // end class

// Create boost pointers for this class
typedef boost::shared_ptr<Visuals> VisualsPtr;

} // end namespace

#endif
