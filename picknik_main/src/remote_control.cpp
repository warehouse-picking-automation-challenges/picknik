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
   Desc:   Contains all hooks for remote control
*/

#include <picknik_main/remote_control.h>
#include <picknik_main/apc_manager.h>

namespace picknik_main
{

/**
 * \brief Constructor
 * \param verbose - run in debug mode
 */
RemoteControl::RemoteControl(bool verbose, ros::NodeHandle nh, APCManager* parent)
  : verbose_(verbose)
  , nh_(nh)
  , parent_(parent)
  , is_waiting_(false)
  , next_step_ready_(false)
  , autonomous_(false)
  , stop_(false)
{
  // Subscribe to remote control topic
  ROS_DEBUG_STREAM_NAMED("apc_manager","Subscribing to button topics");
  std::size_t queue_size = 10;
  remote_next_control_ = nh_.subscribe("/picknik_main/next_command", queue_size, &RemoteControl::remoteNextCallback, this);
  remote_auto_control_ = nh_.subscribe("/picknik_main/auto_command", queue_size, &RemoteControl::remoteAutoCallback, this);
  remote_stop_control_ = nh_.subscribe("/picknik_main/stop_command", queue_size, &RemoteControl::remoteStopCallback, this);
  remote_joy_ = nh_.subscribe("/joy", queue_size, &RemoteControl::joyCallback, this);

  ROS_INFO_STREAM_NAMED("remote_control","RemoteControl Ready.");
}

void RemoteControl::remoteNextCallback(const std_msgs::Bool::ConstPtr& msg)
{
  setReadyForNextStep();
}

void RemoteControl::remoteAutoCallback(const std_msgs::Bool::ConstPtr& msg)
{
  setAutonomous();
}

void RemoteControl::remoteStopCallback(const std_msgs::Bool::ConstPtr& msg)
{
  setStop();
}

void RemoteControl::joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  // Table of index number of /joy.buttons: ------------------------------------

  // 0 - A
  if (msg->buttons[0])
    setReadyForNextStep();
  // 1 - B
  if (msg->buttons[1])
    setStop();
  // 2 - X
  // 3 - Y
  // 4 - LB
  // 5 - RB
  // 6 - back
  if (msg->buttons[6])
    parent_->moveToStartPosition();
  // 7 - start
  // 8 - power
  if (msg->buttons[8])
    setAutonomous();
  // 9 - Button stick left
  // 10 - Button stick right

  // Table of index number of /joy.axis: ------------------------------------

  // 0
  // Left/Right Axis stick left
  // 1
  // Up/Down Axis stick left
  // 2
  // Left/Right Axis stick right
  // 3
  // Up/Down Axis stick right
  // 4
  // RT
  // 4
  // LT
  // 6
  // cross key left/right
  // 7
  // cross key up/down

}

bool RemoteControl::setReadyForNextStep()
{
  if (is_waiting_)
  {
    next_step_ready_ = true;
    stop_ = false;
  }
}

void RemoteControl::setAutonomous(bool autonomous)
{
  // TODO: disable this feature for final competition
  autonomous_ = autonomous;
  stop_ = false;
}

void RemoteControl::setStop(bool stop)
{
  stop_ = stop;
  autonomous_ = !stop;
}

bool RemoteControl::getStop()
{
  return stop_;
}

bool RemoteControl::getAutonomous()
{
  return autonomous_;
}

bool RemoteControl::waitForNextStep()
{
  is_waiting_ = true;
  // Wait until next step is ready
  while (!next_step_ready_ && !autonomous_ && ros::ok())
  {
    ros::Duration(0.25).sleep();
    ros::spinOnce();
  }
  if (!ros::ok())
    return false;
  next_step_ready_ = false;
  is_waiting_ = false;
  return true;
}


} // end namespace
