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

#ifndef PICKNIK_MAIN__REMOTE_CONTROL
#define PICKNIK_MAIN__REMOTE_CONTROL

// MoveIt
#include <moveit/macros/class_forward.h>

// moveit_grasps
#include <moveit_grasps/grasp_planner.h>

// ROS
#include <ros/ros.h>
#include <picknik_msgs/PickNikDashboard.h>
#include <sensor_msgs/Joy.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

// Boost
#include <boost/thread/mutex.hpp>

namespace picknik_main
{
MOVEIT_CLASS_FORWARD(PickManager);

class RemoteControl
{
public:
  /**
   * \brief Constructor
   * \param verbose - run in debug mode
   */
  RemoteControl(bool verbose, ros::NodeHandle nh, PickManager* parent);

  /**
   * \brief Remote control from Rviz
   */
  void remoteCallback(const picknik_msgs::PickNikDashboard::ConstPtr& msg);

  /**
   * \brief Recieves inputs from joystick
   * \param input - description
   * \return true on success
   */
  void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);

  /**
   * \brief Step to next step
   * \return true on success
   */
  bool setReadyForNextStep();

  /**
   * \brief Enable autonomous mode
   */
  void setAutonomous(bool autonomous = true);
  void setFullAutonomous(bool autonomous = true);

  /**
   * \brief Get the autonomous mode
   * \return true if is in autonomous mode
   */
  bool getAutonomous();
  bool getFullAutonomous();

  /**
   * \brief Stop something in pipeline
   */
  void setStop(bool stop = true);

  /**
   * \brief See if we are in stop mode
   */
  bool getStop();

  /**
   * \brief Wait until user presses a button
   * \return true on success
   */
  bool waitForNextStep(const std::string& caption = "go to next step");
  bool waitForNextFullStep(const std::string& caption = "go to next full step");

  void initializeInteractiveMarkers(const geometry_msgs::Pose& pose);

private:
  void make6DofMarker(bool fixed, unsigned int interaction_mode, const geometry_msgs::Pose& pose,
                      bool show_6dof);

  visualization_msgs::InteractiveMarkerControl&
  makeBoxControl(visualization_msgs::InteractiveMarker& msg);

  void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

  /** \brief Not sure what this does */
  // void publishInteractiveMarkers(bool pose_update);

  // Show more visual and console output, with general slower run time.
  bool verbose_;

  // A shared node handle
  ros::NodeHandle nh_;

  // The overall manager for sending commands
  PickManager* parent_;

  // Remote control
  ros::Subscriber remote_control_;
  ros::Subscriber remote_joy_;

  // Remote control
  bool autonomous_;
  bool full_autonomous_;
  bool next_step_ready_;
  bool is_waiting_;
  bool stop_;

  // Interactive markers
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> imarker_server_;
  interactive_markers::MenuHandler menu_handler_;
  bool teleoperation_ready_ = true;

  ros::Time throttle_time_;
  boost::mutex interactive_mutex_;

};  // end class

// Create boost pointers for this class
typedef boost::shared_ptr<RemoteControl> RemoteControlPtr;
typedef boost::shared_ptr<const RemoteControl> RemoteControlConstPtr;

}  // end namespace

#endif
