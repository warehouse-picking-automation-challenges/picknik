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
   Desc:   Perception inteface to communicate with manipulation pipeline
*/

#include <picknik_perception/manipulation_interface.h>

namespace picknik_perception
{

ManipulationInterface::ManipulationInterface()
  : action_server_("recognize_objects", false) // Load the action server
  , start_perception_(false)
  , perception_in_progress_(false)
  , stop_perception_(false)
{

  // Register the goal and feeback callbacks.
  action_server_.registerGoalCallback(boost::bind(&ManipulationInterface::goalCallback, this));
  action_server_.start();

  // Register the stop command
  stop_service_ = nh_.advertiseService("stop_perception", &ManipulationInterface::stopPerception, this);

  // Allow time to publish messages
  ros::Duration(0.1).sleep();
  ros::spinOnce();
  ROS_INFO_STREAM_NAMED("manipulation_interface","ManipulationInterface Ready.");
}

bool ManipulationInterface::isReadyToStartPerception(picknik_msgs::FindObjectsGoalConstPtr& goal)
{
  if (start_perception_)
  {
    // Accept the new goal
    goal = action_server_.acceptNewGoal();
    start_perception_ = false; // we are no longer waiting to start
    perception_in_progress_ = true;
    return true;
  }
  return false;
}

bool ManipulationInterface::isReadyToStopPerception()
{
  if (stop_perception_)
  {
    stop_perception_ = false;
    return true;
  }
  return false;
}

bool ManipulationInterface::sendPerceptionResults(picknik_msgs::FindObjectsResult &result)
{
  ROS_INFO_STREAM_NAMED("manipulation_interface","Returning result back to manipulation pipeline");

  // Error check
  if (!perception_in_progress_)
  {
    ROS_ERROR_STREAM_NAMED("manipulation_interface","Perception is currently not in progress, unable to send results");
    return false;
  }
  if (stop_perception_)
  {
    ROS_ERROR_STREAM_NAMED("manipulation_interface","Perception is commanded to stop but this value has not been checked, unable to send results");
    return false;
  }
  if (start_perception_)
  {
    ROS_ERROR_STREAM_NAMED("manipulation_interface","Perception is commanded to start, unable to send results");
    return false;
  }

  perception_in_progress_ = false;

  // Mark action as completed
  action_server_.setSucceeded(result);
  return true;
}

void ManipulationInterface::goalCallback()
{
  ROS_INFO_STREAM_NAMED("manipulation_interface","Recieved request to start perception");

  // Error check
  if (perception_in_progress_)
  {
    ROS_ERROR_STREAM_NAMED("manipulation_interface","Perception is currently in progress, unable to recieve new goal");
    return;
  }
  if (stop_perception_)
  {
    ROS_ERROR_STREAM_NAMED("manipulation_interface","Perception is currently in progress and is waiting to end, unable to recieve new goal");
    return;
  }

  start_perception_ = true;
}

bool ManipulationInterface::stopPerception(picknik_msgs::StopPerception::Request&, picknik_msgs::StopPerception::Response &res)
{
  // Error check
  if (!perception_in_progress_)
  {
    ROS_ERROR_STREAM_NAMED("manipulation_interface","Perception is currently not in progress, unable to stop");
    res.stopped = false;
    return false;
  }
  if (stop_perception_)
  {
    ROS_ERROR_STREAM_NAMED("manipulation_interface","Perception is already commanded to stop, unable to stop");
    res.stopped = false;
    return false;
  }

  // Mark as stopped
  res.stopped = true;
  stop_perception_ = true;
  ROS_INFO_STREAM_NAMED("manipulation_inteface","Perception stop command has been recieved.");
  return true;
}

} // end namespace
