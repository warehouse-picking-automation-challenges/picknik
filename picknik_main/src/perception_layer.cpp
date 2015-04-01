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

namespace picknik_main
{

PerceptionLayer::PerceptionLayer(bool verbose, VisualsPtr visuals, ShelfObjectPtr shelf, ManipulationDataPtr config)
  : verbose_(verbose)
  , visuals_(visuals)
  , shelf_(shelf)
  , config_(config)
  , find_objects_action_("perception/recognize_objects")
{

  ROS_INFO_STREAM_NAMED("perception_layer","PerceptionLayer Ready.");
}

bool PerceptionLayer::isPerceptionReady()
{
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
  if (!find_objects_action_.waitForResult(ros::Duration(30.0)))
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
    //return false;
  }

  // -----------------------------------------------------------------------------------------------
  // Check bounds and update planning scene
  if (!processNewObjectPose(perception_result, product, bin, current_state))
    return false;

  return true;
}

bool PerceptionLayer::processNewObjectPose(picknik_msgs::FindObjectsResultConstPtr result,
                                           ProductObjectPtr& product, BinObjectPtr& bin, moveit::core::RobotStatePtr current_state)
{
  std::cout << std::endl;
  std::cout << "-------------------------------------------------------" << std::endl;

  ROS_WARN_STREAM_NAMED("perception_layer","Processing new object pose");

  // Update product with new pose
  const Eigen::Affine3d object_to_camera = visuals_->visual_tools_->convertPose(result->desired_object_pose);
  geometry_msgs::PoseStamped object_to_camera_msg;
  object_to_camera_msg.header.frame_id = "xtion_camera";
  object_to_camera_msg.header.stamp = ros::Time::now();
  object_to_camera_msg.pose = result->desired_object_pose;
  visuals_->visual_tools_->publishArrow(object_to_camera_msg, rvt::PURPLE, rvt::LARGE);

  // Get camera pose
  const Eigen::Affine3d& camera_to_world = current_state->getGlobalLinkTransform("xtion");
  printTransform(camera_to_world);
  visuals_->visual_tools_->publishAxis(camera_to_world);
  visuals_->visual_tools_->publishText(camera_to_world, "camera_pose", rvt::BLACK, rvt::SMALL, false);

  // Get bin location
  const Eigen::Affine3d& bin_to_world = transform(bin->getBottomRight(), shelf_->getBottomRight());
  visuals_->visual_tools_->publishAxis(bin_to_world);
  visuals_->visual_tools_->publishText(bin_to_world, "bin_pose", rvt::BLACK, rvt::SMALL, false);

  // Show camera view
  const double distance_from_camera = 0.3;
  const double height = 480 * config_->camera_frame_display_scale_; // size of camera view finder
  const double width = 640 * config_->camera_frame_display_scale_; // size of camera view finder
  Eigen::Affine3d camera_view_finder_offset = Eigen::Affine3d::Identity();
  camera_view_finder_offset.translation().x() = distance_from_camera;
  Eigen::Affine3d camera_view_finder = camera_view_finder_offset * camera_to_world;
  visuals_->visual_tools_->publishWireframeRectangle(camera_view_finder, height, width, rvt::PINK, rvt::SMALL);

  // Convert to pose of bin
  const Eigen::Affine3d object_to_world = object_to_camera * camera_to_world;
  const Eigen::Affine3d object_to_bin = object_to_world * bin_to_world.inverse();

  visuals_->visual_tools_->publishAxis(object_to_world);
  visuals_->visual_tools_->publishText(object_to_world, "object_pose", rvt::BLACK, rvt::SMALL, false);

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
      visuals_->visual_tools_->publishAxis(object_to_bin);
      visuals_->visual_tools_->publishText(object_to_bin, "object_pose_2", rvt::BLACK, rvt::SMALL, false);
      return false;
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

  // Get new transform from shelf to bin to product
  Eigen::Affine3d world_to_bin_transform = transform(bin->getBottomRight(), shelf_->getBottomRight());
  //global_object_pose = transform(object_to_bin, world_to_bin_transform);

  // Show new mesh if possible
  if (result->bounding_mesh.triangles.empty() || result->bounding_mesh.vertices.empty())
  {
    ROS_WARN_STREAM_NAMED("perception_layer","No bounding mesh returned");

    // Show in collision and display Rvizs
    product->visualize(world_to_bin_transform);
    product->createCollisionBodies(world_to_bin_transform);
  }
  else
  {
    product->setCollisionMesh(result->bounding_mesh);

    // Show in collision and display Rvizs
    product->visualize(world_to_bin_transform);
    product->createCollisionBodies(world_to_bin_transform);
  }

  visuals_->visual_tools_->triggerPlanningSceneUpdate();

  return true;
}

} // namespace
