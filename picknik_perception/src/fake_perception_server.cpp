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
   Desc:   Listens to object recognition requests and when one is received,
   publishes updates of collision objects to /collision_objects
*/

// ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <picknik_msgs/FindObjectsAction.h>
#include <picknik_perception/manipulation_interface.h>

#include <rviz_visual_tools/rviz_visual_tools.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "perception_server");
  ROS_INFO_STREAM_NAMED("main", "Starting FakePerceptionServer...");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // initialize random seed
  srand (time(NULL));

  // Create the ROS communication layer with the manipulation pipeline
  picknik_perception::ManipulationInterfacePtr client;
  client.reset(new picknik_perception::ManipulationInterface());

  // Main loop
  while (ros::ok())
  {
    if (client->resetIsNeeded())
      break;

    // Check if goal is recieved
    picknik_msgs::FindObjectsGoalConstPtr goal;
    if (!client->isReadyToStartPerception(goal))
    {
      ros::Duration(0.1).sleep();
      continue; // loop again
    }

    ROS_INFO_STREAM_NAMED("fake_perception_server","Starting perception, waiting for stop command");

    while (ros::ok())
    {
      // Do perception processing HERE

      // Wait until camera is done moving
      if (client->isReadyToStopPerception())
        break;

      // Check if cancel is desired
      if (client->resetIsNeeded())
        break;

      // Wait
      ros::Duration(0.1).sleep();
    }

    // Check if cancel is desired
    if (client->resetIsNeeded())
      break;

    // Finish up perception
    ROS_INFO_STREAM_NAMED("fake_perception_server","Finishing up perception");
    // TODO

    // Send dummy results
    picknik_msgs::FindObjectsResult result;

    // For each object in the bin
    for (std::size_t i = 0; i < goal->expected_objects_names.size(); ++i)
    {
      picknik_msgs::FoundObject new_product;
      new_product.object_name = goal->expected_objects_names[i];

      // Object pose
      new_product.object_pose.position.x = 0.45;
      new_product.object_pose.position.y = 0;
      new_product.object_pose.position.z = 0;
      new_product.object_pose.orientation.x = 0;
      new_product.object_pose.orientation.y = 0;
      new_product.object_pose.orientation.z = 0;
      new_product.object_pose.orientation.w = 1;

      // Value between 0 and 1 for each expected object's confidence of its pose
      new_product.expected_object_confidence = 1.0;

      // Add object to result
      result.found_objects.push_back(new_product);
    } // end for each product

      // If the camera angle was bad or some other failure, return false
    result.succeeded = true;
    //ROS_INFO_STREAM_NAMED("fake_perception_server","Sending perception result:\n" << result);
    ROS_INFO_STREAM_NAMED("fake_perception_server","Sending perception result");

    // Check if cancel is desired
    if (client->resetIsNeeded())
      break;

    client->sendPerceptionResults(result);

  } // end while each request

  ros::waitForShutdown();

  ROS_INFO_STREAM_NAMED("main", "Shutting down.");
  ros::shutdown();

  return 0;
}
