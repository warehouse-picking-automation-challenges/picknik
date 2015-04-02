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
   Desc:   Interface between the perception pipeline and the manipulation pipeline
*/

// PickNik
#include <picknik_main/perception_interface.h>

// ROS
#include <tf_conversions/tf_eigen.h>
#include <std_msgs/Bool.h>

namespace picknik_main
{

PerceptionInterface::PerceptionInterface(bool verbose, VisualsPtr visuals, ShelfObjectPtr shelf, ManipulationDataPtr config, 
                                 boost::shared_ptr<tf::TransformListener> tf, ros::NodeHandle nh)
  : verbose_(verbose)
  , visuals_(visuals)
  , shelf_(shelf)
  , config_(config)
  , tf_(tf)
  , nh_(nh)
  , find_objects_action_(PERCEPTION_TOPIC)
{
  // Load ROS publisher
  std::size_t queue_size = 10;
  stop_perception_pub_ = nh_.advertise<std_msgs::Bool>( "/perception/stop_perception", queue_size );  
  

  ROS_INFO_STREAM_NAMED("perception_interface","PerceptionInterface Ready.");
}

bool PerceptionInterface::isPerceptionReady()
{
  ROS_INFO_STREAM_NAMED("apc_manager","Waiting for object perception server on topic " << PERCEPTION_TOPIC);

  find_objects_action_.waitForServer();
  return true;
}

bool PerceptionInterface::startPerception(ProductObjectPtr& product, BinObjectPtr& bin)
{
  // Setup goal
  ROS_INFO_STREAM_NAMED("perception_interface","Communicating with perception pipeline");
  picknik_msgs::FindObjectsGoal find_object_goal;
  find_object_goal.desired_object_name = product->getName();

  // Get all of the products in the bin
  bin->getProducts(find_object_goal.expected_objects_names);

  // Get the camera pose
  //moveit::core::RobotStatePtr current_state = manipulation_->getCurrentState();
  /// TODO - add a better link
  //find_object_goal.camera_pose = visuals_->visual_tools_->convertPose(current_state->getGlobalLinkTransform("jaco2_end_effector"));

  // Send request
  find_objects_action_.sendGoal(find_object_goal);
  return true;
}


bool PerceptionInterface::endPerception(ProductObjectPtr& product, BinObjectPtr& bin, moveit::core::RobotStatePtr current_state)
{
  // Tell the perception pipeline we are done moving the camera
  ROS_INFO_STREAM_NAMED("perception_interface","Sending stop command to perception server");
  std_msgs::Bool result;
  result.data = true; // meaningless value
  stop_perception_pub_.publish( result );
  ros::spinOnce(); // repeat 3 times for guarantees
  ros::Duration(0.1).sleep();
  stop_perception_pub_.publish( result );
  ros::spinOnce();
  ros::Duration(0.1).sleep();
  stop_perception_pub_.publish( result );

  ROS_INFO_STREAM_NAMED("perception_interface","Waiting for response from perception server");

  // Wait for the action to return with product pose
  if (!find_objects_action_.waitForResult(ros::Duration(20.0)))
  {
    ROS_ERROR_STREAM_NAMED("perception_interface","Percetion action did not finish before the time out.");
    return false;
  }

  // Get goal state
  actionlib::SimpleClientGoalState goal_state = find_objects_action_.getState();
  ROS_INFO_STREAM_NAMED("perception_interface","Perception action finished: " << goal_state.toString());

  // Get result
  picknik_msgs::FindObjectsResultConstPtr perception_result = find_objects_action_.getResult();
  ROS_DEBUG_STREAM_NAMED("apc_manager.perception","Perception result:\n" << *perception_result);

  if (!perception_result->succeeded)
  {
    ROS_ERROR_STREAM_NAMED("perception_interface","Perception action server reported failure");
    return false;
  }

  // -----------------------------------------------------------------------------------------------
  // Check bounds and update planning scene
  if (!processNewObjectPose(perception_result, product, bin, current_state))
  {
    ROS_ERROR_STREAM_NAMED("perception_interface","Failed to process new object pose");
    return false;
  }

  return true;
}

bool PerceptionInterface::processNewObjectPose(picknik_msgs::FindObjectsResultConstPtr result,
                                           ProductObjectPtr& product, BinObjectPtr& bin, moveit::core::RobotStatePtr current_state)
{
  std::cout << std::endl;
  std::cout << "-------------------------------------------------------" << std::endl;
  ROS_WARN_STREAM_NAMED("perception_interface","Processing new object pose");

  // Find the desired pose
  const picknik_msgs::FoundObject* desired_object = NULL;
  for (std::size_t i = 0; i < result->found_objects.size(); ++i)
  {
    std::cout << "CHECKING NAME " << result->found_objects[i].object_name << std::endl;
    if (result->found_objects[i].object_name == product->getName())
    {
      desired_object = &result->found_objects[i];
    }
  }
  std::cout << std::endl;

  // Error check
  if (!desired_object)
  {
    ROS_ERROR_STREAM_NAMED("perception_interface","The desired product was not returned in the list of found objects");
    return false;
  }
  if (result->found_objects.size() != bin->getProducts().size())
  {
    ROS_ERROR_STREAM_NAMED("perception_interface","The list of found objects does not have the same size as the expected bin contents");
    return false;
  }

  std::cout << "FoundObject: " << std::endl;
  std::cout << "  object_name: " << desired_object->object_name << std::endl;
  std::cout << "  object_pose: \n" << desired_object->object_pose;
  std::cout << "  expected_object_confidence: " << desired_object->expected_object_confidence << std::endl;
  std::cout << std::endl;

  // Update bin products with new locations
  // TODO

  // Update product with new pose
  Eigen::Affine3d camera_to_object_cv_frame = visuals_->visual_tools_->convertPose(desired_object->object_pose);
  Eigen::Affine3d camera_to_object;

  // Convert coordinate systems
  convertFrameCVToROS(camera_to_object_cv_frame, camera_to_object);

  bool publish_rviz_arrow = false;
  if (publish_rviz_arrow)
  {
    geometry_msgs::PoseStamped camera_to_object_msg;
    camera_to_object_msg.header.frame_id = "xtion_camera";
    camera_to_object_msg.header.stamp = ros::Time::now();
    camera_to_object_msg.pose = visuals_->visual_tools_->convertPose(camera_to_object);
    visuals_->visual_tools_->publishArrow(camera_to_object_msg, rvt::PURPLE, rvt::LARGE);
  }

  // Get camera pose
  tf::StampedTransform transform;
  try
  {
    ROS_INFO_STREAM_NAMED("perception_interface","Getting transform from /world to /xtion_camera");
    tf_->waitForTransform("/world", "/xtion_camera", ros::Time(0), ros::Duration(3));
    tf_->lookupTransform("/world","/xtion_camera", ros::Time(0), transform);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR_STREAM_NAMED("perception_interface", "Error: " << ex.what());
    return false;
  }

  Eigen::Affine3d world_to_camera;
  tf::transformTFToEigen(transform, world_to_camera);
  visuals_->visual_tools_->publishAxisWithLabel(world_to_camera, "world_to_camera");

  // Show camera view
  publishCameraFrame(world_to_camera);

  // Get bin location
  const Eigen::Affine3d& world_to_bin = picknik_main::transform(bin->getBottomRight(), shelf_->getBottomRight());
  visuals_->visual_tools_->publishAxisWithLabel(world_to_bin, "world_to_bin");

  // Convert to pose of bin
  //const Eigen::Affine3d world_to_object = camera_to_object.inverse() * world_to_camera.inverse();
  const Eigen::Affine3d world_to_object = world_to_camera * camera_to_object;
  visuals_->visual_tools_->publishAxisWithLabel(world_to_object, "world_to_object");
  const Eigen::Affine3d bin_to_object = world_to_bin.inverse() * world_to_object;

  // Check bounds
  if (
      bin_to_object.translation().x() < 0 ||
      bin_to_object.translation().x() > bin->getDepth() ||
      bin_to_object.translation().y() < 0 ||
      bin_to_object.translation().y() > bin->getWidth() ||
      bin_to_object.translation().z() < 0 ||
      bin_to_object.translation().z() > bin->getHeight()
      )
  {
    if (
        bin_to_object.translation().x() < -PRODUCT_POSE_WITHIN_BIN_TOLERANCE ||
        bin_to_object.translation().x() > bin->getDepth() + PRODUCT_POSE_WITHIN_BIN_TOLERANCE ||
        bin_to_object.translation().y() < -PRODUCT_POSE_WITHIN_BIN_TOLERANCE ||
        bin_to_object.translation().y() > bin->getWidth() + PRODUCT_POSE_WITHIN_BIN_TOLERANCE||
        bin_to_object.translation().z() < -PRODUCT_POSE_WITHIN_BIN_TOLERANCE ||
        bin_to_object.translation().z() > bin->getHeight() + PRODUCT_POSE_WITHIN_BIN_TOLERANCE
        )
    {
      // Do not continue because outside of tolerance
      ROS_ERROR_STREAM_NAMED("perception_interface","Product " << product->getName() << " has a reported pose from the perception pipline that is outside the tolerance of " << PRODUCT_POSE_WITHIN_BIN_TOLERANCE);
      ROS_ERROR_STREAM_NAMED("perception_interface","Pose:\n" << visuals_->visual_tools_->convertPose(bin_to_object));
      //return false;
    }
    else
    {
      // Its within error tolerance, just warn
      ROS_WARN_STREAM_NAMED("perception_interface","Product " << product->getName() << " has a reported pose from the perception pipline that is outside the bounds of the bin shape");
      ROS_WARN_STREAM_NAMED("perception_interface","Pose:\n" << visuals_->visual_tools_->convertPose(bin_to_object));
    }
  }

  // Save to the product's property
  product->setCentroid(bin_to_object);

  // Show new mesh if possible
  const shape_msgs::Mesh* mesh = &(desired_object->bounding_mesh);

  ROS_DEBUG_STREAM_NAMED("perception_interface","Recieved mesh with " << mesh->triangles.size() << " triangles and " 
                         << mesh->vertices.size() << " vertices");
                         

  if (mesh->triangles.empty() || mesh->vertices.empty())
  {
    ROS_WARN_STREAM_NAMED("perception_interface","No bounding mesh returned");

    // Show in collision and display Rvizs
    product->visualize(world_to_bin);
    product->createCollisionBodies(world_to_bin);
  }
  else
  {
    ROS_INFO_STREAM_NAMED("perception_interface","Setting new bounding mesh");
    product->setCollisionMesh(*mesh);

    // Show in collision and display Rvizs
    product->visualize(world_to_bin);
    product->createCollisionBodies(world_to_bin);
  }

  visuals_->visual_tools_->triggerPlanningSceneUpdate();

  // Allow mesh to display
  ros::spinOnce();
  ros::Duration(5.0).sleep();

  return true;
}

bool PerceptionInterface::publishCameraFrame(Eigen::Affine3d world_to_camera)
{
  const double distance_from_camera = 0.3;
  const double height = 480 * config_->camera_frame_display_scale_; // size of camera view finder
  const double width = 640 * config_->camera_frame_display_scale_; // size of camera view finder
  Eigen::Affine3d camera_view_finder_offset = Eigen::Affine3d::Identity();
  camera_view_finder_offset.translation().x() = distance_from_camera;

  Eigen::Affine3d camera_view_finder = camera_view_finder_offset * world_to_camera;


  camera_view_finder = camera_view_finder 
    * Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY()) 
    * Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitZ());


  visuals_->visual_tools_->publishWireframeRectangle(camera_view_finder, height, width, rvt::PINK, rvt::SMALL);
}

bool PerceptionInterface::convertFrameCVToROS(const Eigen::Affine3d& cv_frame, Eigen::Affine3d& ros_frame)
{
  Eigen::Matrix3d rotation;
  rotation = Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(-M_PI/2.0, Eigen::Vector3d::UnitZ());
  //std::cout << "Rotation: " << rotation << std::endl;
  ros_frame = rotation * cv_frame;

  return true;
}

} // namespace
