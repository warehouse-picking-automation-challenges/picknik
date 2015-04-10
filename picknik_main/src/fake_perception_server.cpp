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

#include <rviz_visual_tools/rviz_visual_tools.h>

namespace picknik_main
{

class FakePerceptionServer
{
private:

  // A shared node handle
  ros::NodeHandle nh_;

  // Actionlib
  actionlib::SimpleActionServer<picknik_msgs::FindObjectsAction> action_server_;

  // Show more visual and console output, with general slower run time.
  bool verbose_;
  
  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;


public:

  /**
   * \brief Constructor
   * \param verbose - run in debug mode
   */
  FakePerceptionServer(bool verbose)
    : verbose_(verbose),
      action_server_("recognize_objects", false) // Load the action server
  {
    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("world"));

    // Register the goal and feeback callbacks.
    action_server_.registerGoalCallback(boost::bind(&FakePerceptionServer::goalCallback, this));
    action_server_.start();

    // Allow time to publish messages
    ros::Duration(0.1).sleep();
    ros::spinOnce();
    ROS_INFO_STREAM_NAMED("perception_server","FakePerceptionServer Ready.");
  }

  /**
   * \brief Destructor
   */
  ~FakePerceptionServer()
  {
  }

  /**
   * \brief Callback when a request is recieved for recognized objects
   * \param req - Request
   * \param res - Response
   * \return bool - true on success
   */
  void goalCallback()
  {
    ROS_INFO_STREAM_NAMED("perception_server","Current recognized objected requested");

    // Accept the new goal
    picknik_msgs::FindObjectsGoalConstPtr goal;
    goal = action_server_.acceptNewGoal();

    // ================================================================
    // Return a dummy object
    // ================================================================
    picknik_msgs::FindObjectsResult result;
    geometry_msgs::Pose pose;

    // If the camera angle was bad or some other failure, return false
    result.succeeded = true;

    // For each object in the bin
    for (std::size_t i = 0; i < goal->expected_objects_names.size(); ++i)
    {
      picknik_msgs::FoundObject new_product;
      new_product.object_name = goal->expected_objects_names[i];

      // Object pose
      Eigen::Affine3d pose = Eigen::Affine3d::Identity();
      //pose.translation().z() = 0.45;
      pose.translation().y() = -0.45;
      //pose = pose 
      //  * Eigen::AngleAxisd(1.7, Eigen::Vector3d::UnitY());
        
      new_product.object_pose = visual_tools_->convertPose(pose);

      // Value between 0 and 1 for each expected object's confidence of its pose
      new_product.expected_object_confidence = 1.0; // TODO

      /*
      // Mesh - Create random verticies      
      int num_triangles = 10; // times 3
      for (std::size_t i = 0; i < num_triangles*3; ++i)
      {
        visual_tools_->generateRandomPose(pose, pose_bounds);
        new_product.bounding_mesh.vertices.push_back(pose.position);
      }
      // Attach verticies randomly
      for (std::size_t i = 0; i < num_triangles; ++i)
      {
        // Create single triangle
        shape_msgs::MeshTriangle t1;
        for (std::size_t j = 0; j < 3; ++j)
        {
          t1.vertex_indices[j] = visual_tools_->iRand(0,num_triangles*3-1);
        }

        // Add to mesh
        new_product.bounding_mesh.triangles.push_back(t1);
      }
      */

      // Add object to result
      result.found_objects.push_back(new_product);
    }

    std::cout << "result: " << result << std::endl;

    // Mark action as completed
    action_server_.setSucceeded(result);
  }


}; // end class

// Create boost pointers for this class
typedef boost::shared_ptr<FakePerceptionServer> FakePerceptionServerPtr;
typedef boost::shared_ptr<const FakePerceptionServer> FakePerceptionServerConstPtr;

} // end namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "perception_server");
  ROS_INFO_STREAM_NAMED("main", "Starting FakePerceptionServer...");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // initialize random seed
  srand (time(NULL));

  // Check for verbose flag
  bool verbose = false;
  if (argc > 1)
  {
    for (std::size_t i = 0; i < argc; ++i)
    {
      if (strcmp(argv[i], "--verbose") == 0)
      {
        ROS_INFO_STREAM_NAMED("main","Running in VERBOSE mode (slower)");
        verbose = true;
      }
    }
  }

  picknik_main::FakePerceptionServer server(verbose);

  ros::waitForShutdown();

  ROS_INFO_STREAM_NAMED("main", "Shutting down.");
  ros::shutdown();

  return 0;
}
