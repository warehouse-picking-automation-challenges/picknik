/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, University of Colorado, Boulder
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
   This allows the server to be triggered by the Rviz GUI
*/

// ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <object_recognition_msgs/ObjectRecognitionAction.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

namespace moveit_arpg_perception
{

class ObjectRecognitionServer
{
private:

  // A shared node handle
  ros::NodeHandle nh_;

  // Actionlib
  actionlib::SimpleActionServer<object_recognition_msgs::ObjectRecognitionAction> action_server_;

  // Show more visual and console output, with general slower run time.
  bool verbose_;

  // For visualizing things in rviz
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

public:

  /**
   * \brief Constructor
   * \param verbose - run in debug mode
   */
  ObjectRecognitionServer(bool verbose)
    : verbose_(verbose),
      action_server_("/recognize_objects", false) // Load the action server
  {

    // Register the goal and feeback callbacks.
    action_server_.registerGoalCallback(boost::bind(&ObjectRecognitionServer::goalCallback, this));
    action_server_.start();

    // Load collision object publisher
    visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("base","/moveit_visual_tools"));


    // Allow time to publish messages
    ros::Duration(1.0).sleep();
    ros::spinOnce();
    ROS_INFO_STREAM_NAMED("obj_recognition","ObjectRecognitionServer Ready.");
  }

  /**
   * \brief Destructor
   */
  ~ObjectRecognitionServer()
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
    ROS_INFO_STREAM_NAMED("obj_recognition","Current recognized objected requested");

    // Accept the new goal
    //object_recognition_msgs::ObjectRecognitionGoalConstPtr goal_;
    //goal_ =
    action_server_.acceptNewGoal();

    // ============================================================
    // Publish test objects
    // ============================================================
    // Here I will show several methods for publishing objects
    geometry_msgs::Pose pose;

    /*
    // Create collision block
    visual_tools_->generateRandomPose(pose);
    const double block_size = 0.1;
    visual_tools_->publishCollisionBlock(pose, "myblock", block_size);
    ros::spinOnce();

    // Create a collision wall
    visual_tools_->publishCollisionWall(1.0, 1.0, M_PI/2.0, 0.5, "wall1");
    ros::spinOnce();

    // Table
    visual_tools_->publishCollisionTable(1.0, 0, 0, 1, 1, 0.1, "table1");
    */

    // ============================================================
    // Publish Mesh
    // ============================================================

    // Header-type properties
    moveit_msgs::CollisionObject collision_obj;
    collision_obj.header.stamp = ros::Time::now();
    collision_obj.header.frame_id = visual_tools_->getBaseFrame();
    collision_obj.id = "mesh1";
    collision_obj.operation = moveit_msgs::CollisionObject::ADD;

    // Create mesh
    shape_msgs::Mesh mesh1;

    // Create random verticies
    int num_triangles = 50; // times 3
    for (std::size_t i = 0; i < num_triangles*3; ++i)
    {
      visual_tools_->generateRandomPose(pose);
      mesh1.vertices.push_back(pose.position);
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
      mesh1.triangles.push_back(t1);
    }

    collision_obj.meshes.push_back(mesh1);

    // Create pose of mesh
    geometry_msgs::Pose mesh_pose;    
    mesh_pose.position.x = 0.6;
    mesh_pose.position.y = -0.5;
    mesh_pose.position.z = -0.7;

    collision_obj.mesh_poses.push_back(mesh_pose);

    std::cout << "collision_obj "  << collision_obj << std::endl;

    visual_tools_->processCollisionObjectMsg(collision_obj);
    ros::Duration(1).sleep();
    ros::spinOnce();


    // ================================================================

    // Mark action as completed
    object_recognition_msgs::ObjectRecognitionResult result;
    action_server_.setSucceeded(result);
  }


}; // end class

// Create boost pointers for this class
typedef boost::shared_ptr<ObjectRecognitionServer> ObjectRecognitionServerPtr;
typedef boost::shared_ptr<const ObjectRecognitionServer> ObjectRecognitionServerConstPtr;

} // end namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "obj_recognition");
  ROS_INFO_STREAM_NAMED("main", "Starting ObjectRecognitionServer...");

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

  moveit_arpg_perception::ObjectRecognitionServer server(verbose);

  ros::waitForShutdown();

  ROS_INFO_STREAM_NAMED("main", "Shutting down.");
  ros::shutdown();

  return 0;
}
