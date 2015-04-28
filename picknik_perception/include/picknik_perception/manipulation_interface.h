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

// ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <picknik_msgs/FindObjectsAction.h>
#include <picknik_msgs/StopPerception.h>

namespace picknik_perception
{

class ManipulationInterface
{
public:

  /**
   * \brief Constructor
   */
  ManipulationInterface();

  /**
   * \brief Ability to clear all previous progress and reset system to new
   * \return true if reset needed
   */
  bool resetIsNeeded();

  /**
   * \brief Check if perception is ready to start processing
   * \param goal - the items to look for if system is ready to start perception
   * \return true if ready to start perception
   */
  bool isReadyToStartPerception(picknik_msgs::FindObjectsGoalConstPtr& goal);

  /**
   * \brief Check if it is time to end perception, meaning camera has stopped moving and we manipulation pipeline wants results
   * \return true if camera 
   */
  bool isReadyToStopPerception();

  /**
   * \brief Send result back to manipulation pipeline
   * \param result - discovered items to return back
   * \return true on successful communication
   */
  bool sendPerceptionResults(picknik_msgs::FindObjectsResult &result);

private:

  /**
   * \brief Callback when a request is recieved for recognized objects
   * \param req - Request
   * \param res - Response
   * \return bool - true on success
   */
  void goalCallback();

  /**
   * \brief Stop callback
   * \return true on success
   */
  bool stopPerception(picknik_msgs::StopPerception::Request&, picknik_msgs::StopPerception::Response &res);

  /**
   * \brief Ability to clear all previous progress and reset system to new
   * \return true on success
   */
  bool resetPerception(picknik_msgs::StopPerception::Request&, picknik_msgs::StopPerception::Response &res);

  /**
   * \brief Initialize state machine variables
   * \return true on success
   */
  bool initialize();

  // A shared node handle
  ros::NodeHandle nh_;

  // Actionlib
  actionlib::SimpleActionServer<picknik_msgs::FindObjectsAction> action_server_;

  // Stop and reset services
  ros::ServiceServer stop_service_;
  ros::ServiceServer reset_service_;

  // State machine
  bool perception_running_; // whether we should start perception
  bool stop_perception_; // whether perception should end
  bool reset_perception_; // Whether to clear all previous progress and restart

}; // end class

// Create boost pointers for this class
typedef boost::shared_ptr<ManipulationInterface> ManipulationInterfacePtr;
typedef boost::shared_ptr<const ManipulationInterface> ManipulationInterfaceConstPtr;

} // end namespace
