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
#include <picknik_main/namespaces.h>
#include <picknik_main/manipulation_data.h>

// ROS
#include <tf_conversions/tf_eigen.h>

// PickNik Msgs
#include <picknik_msgs/StopPerception.h>

// Parameter loading
#include <ros_param_utilities/ros_param_utilities.h>

namespace picknik_main
{
PerceptionInterface::PerceptionInterface(bool verbose, VisualsPtr visuals,
                                         ManipulationDataPtr config,
                                         boost::shared_ptr<tf::TransformListener> tf,
                                         ros::NodeHandle nh)
  : nh_(nh)
  , verbose_(verbose)
  , visuals_(visuals)
  , config_(config)
  , tf_(tf)
  , find_objects_action_(PERCEPTION_TOPIC)
  , is_processing_perception_(false)
{
  // Load ROS publisher
  stop_perception_client_ =
      nh_.serviceClient<picknik_msgs::StopPerception>("/perception/stop_perception");
  reset_perception_client_ =
      nh_.serviceClient<picknik_msgs::StopPerception>("/perception/reset_perception");

  //***** WHAT IS THIS USED FOR?
  // Load camera intrinsics
  const std::string parent_name = "perception_interface";  // for namespacing logging messages
  ros_param_utilities::getDoubleParameter(parent_name, nh, "camera_intrinsics/fx", camera_fx_);
  ros_param_utilities::getDoubleParameter(parent_name, nh, "camera_intrinsics/fx", camera_fy_);
  ros_param_utilities::getDoubleParameter(parent_name, nh, "camera_intrinsics/fx", camera_cx_);
  ros_param_utilities::getDoubleParameter(parent_name, nh, "camera_intrinsics/fx", camera_cy_);
  ros_param_utilities::getDoubleParameter(parent_name, nh, "camera_intrinsics/min_depth",
                                          camera_min_depth_);
  // ros_param_utilities::getDoubleParameter(parent_name, nh, "bounding_box_reduction",
  // bounding_box_reduction_);

  ROS_INFO_STREAM_NAMED("perception_interface", "PerceptionInterface Ready.");
}

bool PerceptionInterface::isPerceptionReady()
{
  // Do initial check before showing warning
  if (!find_objects_action_.waitForServer(ros::Duration(0.1)))
  {
    ROS_WARN_STREAM_NAMED("perception_interface", "Waiting for object perception server on topic "
                                                      << PERCEPTION_TOPIC);

    find_objects_action_.waitForServer();
  }
  ROS_INFO_STREAM_NAMED("perception_interface", "Object perception server found");

  return true;
}

bool PerceptionInterface::getTFTransform(Eigen::Affine3d& world_to_frame, ros::Time& time_stamp,
                                         const std::string& frame_id)
{
  return getTFTransform(world_to_frame, time_stamp, config_->robot_base_frame_, frame_id);
}

bool PerceptionInterface::getTFTransform(Eigen::Affine3d& world_to_frame, ros::Time& time_stamp,
                                         const std::string& parent_frame_id,
                                         const std::string& frame_id)
{
  tf::StampedTransform tf_transform;

  try
  {
    // Wait to make sure a transform message has arrived
    tf_->waitForTransform(parent_frame_id, frame_id, ros::Time(0), ros::Duration(1));
    // Get latest transform available
    tf_->lookupTransform(parent_frame_id, frame_id, ros::Time(0), tf_transform);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR_STREAM_NAMED("perception_interface", "TF error: " << ex.what());
    return false;
  }

  // Copy results
  tf::transformTFToEigen(tf_transform, world_to_frame);
  time_stamp = tf_transform.stamp_;
  return true;
}

bool PerceptionInterface::publishCameraFrame(Eigen::Affine3d world_to_camera)
{
  // get the pose of the top left point which depth is 0.5 meters away
  Eigen::Vector3d top_left, top_right, bottom_left, bottom_right;
  // top left (0,0)
  top_left << -camera_min_depth_ * (camera_cy_ - 0) / camera_fx_,
      -camera_min_depth_ * (camera_cx_ - 0) / camera_fy_, camera_min_depth_;
  // top right (640,0)
  top_right << -camera_min_depth_ * (camera_cy_ - 640) / camera_fx_,
      -camera_min_depth_ * (camera_cx_ - 0) / camera_fy_, camera_min_depth_;
  // bot right (640.480)
  bottom_right << -camera_min_depth_ * (camera_cy_ - 640) / camera_fx_,
      -camera_min_depth_ * (camera_cx_ - 480) / camera_fy_, camera_min_depth_;
  // bot left (0,480)
  bottom_left << -camera_min_depth_ * (camera_cy_ - 0) / camera_fx_,
      -camera_min_depth_ * (camera_cx_ - 480) / camera_fy_, camera_min_depth_;

  // const double distance_from_camera = 0;
  // Eigen::Affine3d camera_view_finder_offset = Eigen::Affine3d::Identity();
  // camera_view_finder_offset.translation().x() = distance_from_camera;
  // Eigen::Affine3d camera_view_finder = camera_view_finder_offset * world_to_camera;

  Eigen::Affine3d camera_view_finder = Eigen::Affine3d::Identity();  // world_to_camera;

  // camera_view_finder = camera_view_finder
  //   * Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY())
  //   * Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitZ());

  visuals_->visual_tools_->publishWireframeRectangle(
      camera_view_finder, top_left, top_right, bottom_right, bottom_left, rvt::PINK, rvt::SMALL);

  // visuals_->visual_tools_->publishWireframeRectangle(camera_view_finder, height, width,
  // rvt::PINK, rvt::SMALL);
  return true;
}

}  // namespace
