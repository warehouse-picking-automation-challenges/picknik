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
#include <picknik_main/perception_layer.h>

// ROS
#include <tf_conversions/tf_eigen.h>

namespace picknik_main
{

PerceptionLayer::PerceptionLayer(bool verbose, VisualsPtr visuals, ShelfObjectPtr shelf, ManipulationDataPtr config)
  : verbose_(verbose)
  , visuals_(visuals)
  , shelf_(shelf)
  , config_(config)
  , find_objects_action_(PERCEPTION_TOPIC)
{

  ROS_INFO_STREAM_NAMED("perception_layer","PerceptionLayer Ready.");
}

bool PerceptionLayer::isPerceptionReady()
{
  ROS_INFO_STREAM_NAMED("apc_manager","Waiting for object perception server on topic " << PERCEPTION_TOPIC);

  find_objects_action_.waitForServer();
  return true;
}

bool PerceptionLayer::startPerception(ProductObjectPtr& product, BinObjectPtr& bin)
{
  // Setup goal
  ROS_INFO_STREAM_NAMED("perception_layer","Communicating with perception pipeline");
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


bool PerceptionLayer::endPerception(ProductObjectPtr& product, BinObjectPtr& bin, moveit::core::RobotStatePtr current_state)
{
  ROS_INFO_STREAM_NAMED("perception_layer","Waiting for response from perception server");

  // Wait for the action to return with product pose
  if (!find_objects_action_.waitForResult(ros::Duration(20.0)))
  {
    ROS_ERROR_STREAM_NAMED("perception_layer","Percetion action did not finish before the time out.");
    return false;
  }

  // Get goal state
  actionlib::SimpleClientGoalState goal_state = find_objects_action_.getState();
  ROS_INFO_STREAM_NAMED("perception_layer","Perception action finished: " << goal_state.toString());

  // Get result
  picknik_msgs::FindObjectsResultConstPtr perception_result = find_objects_action_.getResult();
  ROS_DEBUG_STREAM_NAMED("apc_manager.perception","Perception result:\n" << *perception_result);

  if (!perception_result->succeeded)
  {
    ROS_ERROR_STREAM_NAMED("perception_layer","Perception action server reported failure");
    return false;
  }

  // -----------------------------------------------------------------------------------------------
  // Check bounds and update planning scene
  if (!processNewObjectPose(perception_result, product, bin, current_state))
  {
    ROS_ERROR_STREAM_NAMED("perception_layer","Failed to process new object pose");
    return false;
  }

  return true;
}

bool PerceptionLayer::processNewObjectPose(picknik_msgs::FindObjectsResultConstPtr result,
                                           ProductObjectPtr& product, BinObjectPtr& bin, moveit::core::RobotStatePtr current_state)
{
  std::cout << std::endl;
  std::cout << "-------------------------------------------------------" << std::endl;
  ROS_WARN_STREAM_NAMED("perception_layer","Processing new object pose");

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
    ROS_ERROR_STREAM_NAMED("perception_layer","The desired product was not returned in the list of found objects");
    return false;
  }
  if (result->found_objects.size() != bin->getProducts().size())
  {
    ROS_ERROR_STREAM_NAMED("perception_layer","The list of found objects does not have the same size as the expected bin contents");
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
  Eigen::Affine3d object_to_camera_cv_frame = visuals_->visual_tools_->convertPose(desired_object->object_pose);
  Eigen::Affine3d object_to_camera;

  // Convert coordinate systems
  convertFrameCVToROS(object_to_camera_cv_frame, object_to_camera);

  geometry_msgs::PoseStamped object_to_camera_msg;
  object_to_camera_msg.header.frame_id = "xtion_camera";
  object_to_camera_msg.header.stamp = ros::Time::now();
  object_to_camera_msg.pose = visuals_->visual_tools_->convertPose(object_to_camera);
  visuals_->visual_tools_->publishArrow(object_to_camera_msg, rvt::PURPLE, rvt::LARGE);

  // Get camera pose
  tf::StampedTransform transform;
  try
  {
    listener_.waitForTransform("/world", "/xtion_camera", ros::Time(0), ros::Duration(3));
    listener_.lookupTransform("/world", "/xtion_camera", ros::Time(0), transform);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR_STREAM_NAMED("perception_layer", "Error: " << ex.what());
    return false;
  }

  Eigen::Affine3d camera_to_world;
  tf::transformTFToEigen(transform, camera_to_world);
  visuals_->visual_tools_->publishAxisWithLabel(camera_to_world, "xtion_to_world");

  // Show camera view
  publishCameraFrame(camera_to_world);

  // Get bin location
  const Eigen::Affine3d& bin_to_world = picknik_main::transform(bin->getBottomRight(), shelf_->getBottomRight());
  visuals_->visual_tools_->publishAxisWithLabel(bin_to_world, "bin_to_world");

  // Convert to pose of bin
  const Eigen::Affine3d object_to_world = object_to_camera * camera_to_world;
  const Eigen::Affine3d object_to_bin = object_to_world * bin_to_world.inverse();

  visuals_->visual_tools_->publishAxis(object_to_world);
  visuals_->visual_tools_->publishText(object_to_world, "object_to_world", rvt::BLACK, rvt::SMALL, false);

  // Check bounds
  if (
      object_to_bin.translation().x() < 0 ||
      object_to_bin.translation().x() > bin->getDepth() ||
      object_to_bin.translation().y() < 0 ||
      object_to_bin.translation().y() > bin->getWidth() ||
      object_to_bin.translation().z() < 0 ||
      object_to_bin.translation().z() > bin->getHeight()
      )
  {
    if (
        object_to_bin.translation().x() < -PRODUCT_POSE_WITHIN_BIN_TOLERANCE ||
        object_to_bin.translation().x() > bin->getDepth() + PRODUCT_POSE_WITHIN_BIN_TOLERANCE ||
        object_to_bin.translation().y() < -PRODUCT_POSE_WITHIN_BIN_TOLERANCE ||
        object_to_bin.translation().y() > bin->getWidth() + PRODUCT_POSE_WITHIN_BIN_TOLERANCE||
        object_to_bin.translation().z() < -PRODUCT_POSE_WITHIN_BIN_TOLERANCE ||
        object_to_bin.translation().z() > bin->getHeight() + PRODUCT_POSE_WITHIN_BIN_TOLERANCE
        )
    {
      // Do not continue because outside of tolerance
      ROS_ERROR_STREAM_NAMED("perception_layer","Product " << product->getName() << " has a reported pose from the perception pipline that is outside the tolerance of " << PRODUCT_POSE_WITHIN_BIN_TOLERANCE);
      ROS_ERROR_STREAM_NAMED("perception_layer","Pose:\n" << visuals_->visual_tools_->convertPose(object_to_bin));
      //return false;
    }
    else
    {
      // Its within error tolerance, just warn
      ROS_WARN_STREAM_NAMED("perception_layer","Product " << product->getName() << " has a reported pose from the perception pipline that is outside the bounds of the bin shape");
      ROS_WARN_STREAM_NAMED("perception_layer","Pose:\n" << visuals_->visual_tools_->convertPose(object_to_bin));
    }
  }

  // Save to the product's property
  product->setCentroid(object_to_bin);

  // Show new mesh if possible
  const shape_msgs::Mesh* mesh = &(desired_object->bounding_mesh);
  if (mesh->triangles.empty() || mesh->vertices.empty())
  {
    ROS_WARN_STREAM_NAMED("perception_layer","No bounding mesh returned");

    // Show in collision and display Rvizs
    product->visualize(bin_to_world);
    product->createCollisionBodies(bin_to_world);
  }
  else
  {
    ROS_INFO_STREAM_NAMED("perception_layer","Setting new bounding mesh");
    //product->setCollisionMesh(*mesh);

    // Show in collision and display Rvizs
    product->visualize(bin_to_world);
    product->createCollisionBodies(bin_to_world);
  }

  visuals_->visual_tools_->triggerPlanningSceneUpdate();

  return true;
}

bool PerceptionLayer::publishCameraFrame(Eigen::Affine3d camera_to_world)
{
  const double distance_from_camera = 0.3;
  const double height = 480 * config_->camera_frame_display_scale_; // size of camera view finder
  const double width = 640 * config_->camera_frame_display_scale_; // size of camera view finder
  Eigen::Affine3d camera_view_finder_offset = Eigen::Affine3d::Identity();
  camera_view_finder_offset.translation().x() = distance_from_camera;

  Eigen::Affine3d camera_view_finder = camera_view_finder_offset * camera_to_world;


  camera_view_finder = camera_view_finder 
    * Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY()) 
    * Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitZ());


  visuals_->visual_tools_->publishWireframeRectangle(camera_view_finder, height, width, rvt::PINK, rvt::SMALL);
}

bool PerceptionLayer::convertFrameCVToROS(const Eigen::Affine3d& cv_frame, Eigen::Affine3d& ros_frame)
{
  Eigen::Matrix3d rotation;
  rotation = Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(-M_PI/2.0, Eigen::Vector3d::UnitZ());
  std::cout << "Rotation: " << rotation << std::endl;
  ros_frame = rotation * cv_frame;

  //    * Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY());

  return true;
}

} // namespace
