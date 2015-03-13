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
#include <picknik_msgs/FindObjectsAction.h>
#include <tf/transform_datatypes.h>

//#include <moveit_visual_tools/moveit_visual_tools.h>

//ARPG includes
#include <Node/Node.h>

#include "findObject.pb.h"
#include "ObjectPose.pb.h"

using std::string;

namespace moveit_arpg_perception
{

class ObjectRecognitionServer
{
private:

  //ROS-related:
  // A shared node handle
  ros::NodeHandle nh_;

  // Actionlib
  actionlib::SimpleActionServer<picknik_msgs::FindObjectsAction> action_server_;

  // Show more visual and console output, with general slower run time.
  bool verbose_;

  // For visualizing things in rviz
  //moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;


  //ARPG Node related:
  node::node n;
  string nodeName;
  string ddtrNode;
  string ddtrGetObjects;
  int seqToDDTR;
  int seqFromDDTR;
  
public:

  /**
   * \brief Constructor
   * \param verbose - run in debug mode
   */
  ObjectRecognitionServer(bool verbose)
    : verbose_(verbose),
      action_server_("recognize_objects", false) // Load the action server
  {

    // Register the goal and feeback callbacks.
    action_server_.registerGoalCallback(boost::bind(&ObjectRecognitionServer::goalCallback, this));
    action_server_.start();

    // Load collision object publisher
    //visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("base","/moveit_visual_tools"));

    // Allow time to publish messages
    ros::Duration(0.1).sleep();
    ros::spinOnce();
    nodeName = "ObjectRecognitionServer";
    n.init(nodeName);
    ddtrNode = "ddtr";
    ddtrGetObjects = "getObjects";
    seqToDDTR = 0;
    seqFromDDTR = 0;
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
    
    // Accept the new goal
    picknik_msgs::FindObjectsGoalConstPtr goal;
    goal = action_server_.acceptNewGoal();
    ROS_INFO("obj_recognition: Current recognized objected requested: Looking for [%s]", goal->desired_object_name.c_str());

    //Tell DDTR that we need some info - pass the Node message 

    string rpc_call = ddtrNode.append("/");
    rpc_call.append(ddtrGetObjects);

    int res;
    FindObjectMsg sendMsg;
    sendMsg.set_objectname(goal->desired_object_name);
    for (int i=0; i<goal->expected_objects_names.size(); i++)
      {
	sendMsg.add_cellobjects(goal->expected_objects_names[i]);
      }
    
    sendMsg.set_sequence(seqToDDTR++);

    ObjectPoseMsg recvMsg;
    ROS_INFO("obj_recognition: Requesting pose for object [%s] from Node [%s] via RPC", goal->desired_object_name.c_str(), rpc_call.c_str());
    
    n.call_rpc(rpc_call, sendMsg, recvMsg);

    ROS_INFO("obj_recognition: Got object pose in reply : [%1.3f, %1.3f, %1.3f], seq %d",
	     recvMsg.x(),
	     recvMsg.y(),
	     recvMsg.z(),
	     recvMsg.sequence());
    // ============================================================
    // Publish test objects
    // ============================================================
    // Here I will show several methods for publishing objects

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
    /*
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
    */
    
    picknik_msgs::FindObjectsResult result;
    geometry_msgs::Pose pose;
    
    pose.position.x = recvMsg.x();
    pose.position.y = recvMsg.y();
    pose.position.z = recvMsg.z();

    pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(recvMsg.p(),
							   recvMsg.q(),
							   recvMsg.r());
    result.succeeded = true;
    /*
    // ================================================================
    // Return a dummy object
    // ================================================================
    picknik_msgs::FindObjectsResult result;
    geometry_msgs::Pose pose;
    rviz_visual_tools::RandomPoseBounds pose_bounds;

    // If the camera angle was bad or some other failure, return false
    result.succeeded = true;

    // The pose in some frame with units radians/meters
    visual_tools_->generateRandomPose(pose, pose_bounds);
    result.desired_object_pose = pose;

    // Sometimes, you can only provide a bounding box/shape, even in 3d
    // This is in the pose frame
    //result.bounding_mesh

    // For each object in the bin
    for (std::size_t i = 0; i < goal->expected_objects_names.size(); ++i)
    {
      // The poses of all the objects expected in the scene,
      visual_tools_->generateRandomPose(pose, pose_bounds);
      result.expected_objects_poses.push_back(pose);

      // Value between 0 and 1 for each expected object's confidence of its pose
      result.expected_object_confidence.push_back(1.0); // we're super confident ;-)
    }
    */
    
    // Mark action as completed
    result.expected_objects_poses.push_back(pose);
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
