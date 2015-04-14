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
   Desc:   Holds common parameters for manipulation
*/

#include <picknik_main/manipulation_data.h>
#include <picknik_main/namespaces.h>

// Parameter loading
#include <rviz_visual_tools/ros_param_utilities.h>

namespace picknik_main
{

ManipulationData::ManipulationData()
  : nh_("~")
{
}

bool ManipulationData::load(robot_model::RobotModelPtr robot_model)
{
const std::string parent_name = "manipulation_data"; // for namespacing logging messages

// Load performance variables
rvt::getDoubleParameter(parent_name, nh_, "main_velocity_scaling_factor", main_velocity_scaling_factor_);
rvt::getDoubleParameter(parent_name, nh_, "approach_velocity_scaling_factor", approach_velocity_scaling_factor_);
rvt::getDoubleParameter(parent_name, nh_, "lift_velocity_scaling_factor", lift_velocity_scaling_factor_);
rvt::getDoubleParameter(parent_name, nh_, "retreat_velocity_scaling_factor", retreat_velocity_scaling_factor_);
rvt::getDoubleParameter(parent_name, nh_, "calibration_velocity_scaling_factor", calibration_velocity_scaling_factor_);
rvt::getDoubleParameter(parent_name, nh_, "wait_before_grasp", wait_before_grasp_);
rvt::getDoubleParameter(parent_name, nh_, "wait_after_grasp", wait_after_grasp_);
rvt::getDoubleParameter(parent_name, nh_, "approach_distance_desired", approach_distance_desired_);
rvt::getDoubleParameter(parent_name, nh_, "retreat_distance_desired", retreat_distance_desired_);
rvt::getDoubleParameter(parent_name, nh_, "lift_distance_desired", lift_distance_desired_);
rvt::getDoubleParameter(parent_name, nh_, "place_goal_down_distance_desired", place_goal_down_distance_desired_);

// Load perception variables
rvt::getDoubleParameter(parent_name, nh_, "camera/x_translation_from_bin", camera_x_translation_from_bin_);
rvt::getDoubleParameter(parent_name, nh_, "camera/y_translation_from_bin", camera_y_translation_from_bin_);
rvt::getDoubleParameter(parent_name, nh_, "camera/z_translation_from_bin", camera_z_translation_from_bin_);
rvt::getDoubleParameter(parent_name, nh_, "camera/x_rotation_from_standard_grasp", camera_x_rotation_from_standard_grasp_);
rvt::getDoubleParameter(parent_name, nh_, "camera/y_rotation_from_standard_grasp", camera_y_rotation_from_standard_grasp_);
rvt::getDoubleParameter(parent_name, nh_, "camera/z_rotation_from_standard_grasp", camera_z_rotation_from_standard_grasp_);
rvt::getDoubleParameter(parent_name, nh_, "camera/lift_distance", camera_lift_distance_);
rvt::getDoubleParameter(parent_name, nh_, "camera/left_distance", camera_left_distance_);
rvt::getDoubleParameter(parent_name, nh_, "camera/camera_frame_display_scale", camera_frame_display_scale_);

// Load robot semantics
rvt::getStringParameter(parent_name, nh_, "start_pose", start_pose_);
rvt::getStringParameter(parent_name, nh_, "right_arm_dropoff_pose", right_arm_dropoff_pose_);
rvt::getStringParameter(parent_name, nh_, "left_arm_dropoff_pose", left_arm_dropoff_pose_);
rvt::getStringParameter(parent_name, nh_, "right_hand_name", right_hand_name_);
rvt::getStringParameter(parent_name, nh_, "left_hand_name", left_hand_name_);
rvt::getStringParameter(parent_name, nh_, "right_arm_name", right_arm_name_);
rvt::getStringParameter(parent_name, nh_, "left_arm_name", left_arm_name_);
rvt::getStringParameter(parent_name, nh_, "both_arms_name", both_arms_name_);

// Decide on dual arm mode we are in
int temp_value;
rvt::getIntParameter(parent_name, nh_, "dual_arm", temp_value);
 dual_arm_ = temp_value;

 // Load proper groups
 // TODO - check if joint model group exists
 if (dual_arm_)
 {
   // Load arm groups
   left_arm_ = robot_model->getJointModelGroup(left_arm_name_);
   right_arm_ = robot_model->getJointModelGroup(right_arm_name_);
   both_arms_ = robot_model->getJointModelGroup(both_arms_name_);
 }
 else
 {
   // Load arm groups
   right_arm_ = robot_model->getJointModelGroup(right_arm_name_);
 }

 ROS_INFO_STREAM_NAMED("manipulation_data","ManipulationData Ready.");

 return true;
}

} // end namespace
