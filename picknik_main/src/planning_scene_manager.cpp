/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman <dave@dav.ee>
   Desc:   Shows different versions of a shelf planning scene
*/

// PickNik
#include <picknik_main/planning_scene_manager.h>
#include <picknik_main/namespaces.h>
#include <picknik_main/visuals.h>
#include <picknik_main/shelf.h>

// ROS
#include <ros/ros.h>

namespace picknik_main
{

PlanningSceneManager::PlanningSceneManager(bool verbose, VisualsPtr visuals, ShelfObjectPtr shelf)
  : verbose_(verbose)
  , visuals_(visuals)
  , shelf_(shelf)
  , mode_(NOT_LOADED)
  , focused_bin_("")
{

  ROS_INFO_STREAM_NAMED("planning_scene_manager","PlanningSceneManager Ready.");
}

bool PlanningSceneManager::displayEmptyShelf()
{
  if (mode_ == EMPTY_SHELF)
  {
    return true;
  }
  ROS_DEBUG_STREAM_NAMED("planning_scene_manager","SWITCHING TO MODE empty_shelf");
  mode_ = EMPTY_SHELF;

  // Clear all old collision objects
  visuals_->visual_tools_->removeAllCollisionObjects();

  // Create new scene
  bool just_frame = true;
  bool show_products = false;
  shelf_->createCollisionBodies("", just_frame, show_products); // only show the frame

  // Output planning scene
  visuals_->visual_tools_->triggerPlanningSceneUpdate();
  return true;
}

bool PlanningSceneManager::displayShelfWithOpenBins(bool force)
{
  if (!force && mode_ == ALL_OPEN_BINS)
  {
    return true;
  }
  ROS_DEBUG_STREAM_NAMED("planning_scene_manager","SWITCHING TO MODE all_open_bins");
  mode_ = ALL_OPEN_BINS;

  // Clear all old collision objects
  visuals_->visual_tools_->removeAllCollisionObjects();

  // Create new scene
  bool just_frame = false;
  bool show_products = true;
  shelf_->createCollisionBodies("", just_frame, show_products); // only show the frame

  // Output planning scene
  visuals_->visual_tools_->triggerPlanningSceneUpdate();

  return true;
}

bool PlanningSceneManager::displayShelfAsWall()
{
  if (mode_ == ONLY_COLLISION_WALL)
  {
    return true;
  }
  ROS_DEBUG_STREAM_NAMED("planning_scene_manager","SWITCHING TO MODE only_collision_wall");
  mode_ = ONLY_COLLISION_WALL;

  // Clear all old collision objects
  visuals_->visual_tools_->removeAllCollisionObjects();

  // Create new scene
  shelf_->getFrontWall()->createCollisionBodies(shelf_->getBottomRight());
  shelf_->getGoalBin()->createCollisionBodies(shelf_->getBottomRight());
  shelf_->createCollisionBodiesEnvironmentObjects();

  // Output planning scene
  visuals_->visual_tools_->triggerPlanningSceneUpdate();
  ros::Duration(0.1).sleep(); // TODO remove?

  return true;
}

bool PlanningSceneManager::displayShelfOnlyBin( const std::string& bin_name )
{
  if (mode_ == FOCUSED_ON_BIN && focused_bin_ == bin_name)
  {
    return true;
  }
  ROS_DEBUG_STREAM_NAMED("planning_scene_manager","SWITCHING TO MODE focused_on_bin");
  mode_ = FOCUSED_ON_BIN;
  focused_bin_ = bin_name;

  // Clear all old collision objects
  visuals_->visual_tools_->removeAllCollisionObjects();

  // Create new scene
  bool only_show_shelf_frame = false;
  bool show_all_products = false;
  ROS_DEBUG_STREAM_NAMED("apc_manager","Showing planning scene shelf with focus on bin " << bin_name);

  shelf_->createCollisionBodies(bin_name, only_show_shelf_frame, show_all_products);

  // Output planning scene
  visuals_->visual_tools_->triggerPlanningSceneUpdate();
  ros::Duration(0.5).sleep();

  return true;
}

bool PlanningSceneManager::testAllModes()
{
  while (ros::ok())
  {
    displayShelfWithOpenBins();
    ros::Duration(4.0).sleep();

    displayShelfAsWall();
    ros::Duration(4.0).sleep();

    displayShelfOnlyBin("bin_B");
    ros::Duration(4.0).sleep();
  }
  return true;
}

} // end namespace
