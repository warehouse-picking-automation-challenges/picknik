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

namespace picknik_main
{

ManipulationData::ManipulationData(robot_model::RobotModelPtr robot_model)
  : nh_("~")
{
  // Load performance variables
  getDoubleParameter(nh_, "main_velocity_scaling_factor", main_velocity_scaling_factor_);
  getDoubleParameter(nh_, "approach_velocity_scaling_factor", approach_velocity_scaling_factor_);
  getDoubleParameter(nh_, "lift_velocity_scaling_factor", lift_velocity_scaling_factor_);
  getDoubleParameter(nh_, "retreat_velocity_scaling_factor", retreat_velocity_scaling_factor_);
  getDoubleParameter(nh_, "calibration_velocity_scaling_factor", calibration_velocity_scaling_factor_);
  getDoubleParameter(nh_, "wait_before_grasp", wait_before_grasp_);
  getDoubleParameter(nh_, "wait_after_grasp", wait_after_grasp_);
  getDoubleParameter(nh_, "approach_distance_desired", approach_distance_desired_);
  getDoubleParameter(nh_, "lift_distance_desired", lift_distance_desired_);
  getDoubleParameter(nh_, "collision_wall_safety_margin", collision_wall_safety_margin_);

  // Load perception variables
  getDoubleParameter(nh_, "camera/x_translation_from_bin", camera_x_translation_from_bin_);
  getDoubleParameter(nh_, "camera/y_translation_from_bin", camera_y_translation_from_bin_);
  getDoubleParameter(nh_, "camera/z_translation_from_bin", camera_z_translation_from_bin_);
  getDoubleParameter(nh_, "camera/x_rotation_from_standard_grasp", camera_x_rotation_from_standard_grasp_);
  getDoubleParameter(nh_, "camera/y_rotation_from_standard_grasp", camera_y_rotation_from_standard_grasp_);
  getDoubleParameter(nh_, "camera/z_rotation_from_standard_grasp", camera_z_rotation_from_standard_grasp_);
  getDoubleParameter(nh_, "camera/lift_distance", camera_lift_distance_);
  getDoubleParameter(nh_, "camera/left_distance", camera_left_distance_);

  // Load robot semantics
  getStringParameter(nh_, "start_pose", start_pose_);
  getStringParameter(nh_, "dropoff_pose", dropoff_pose_);
  getStringParameter(nh_, "right_hand_name", right_hand_name_);
  getStringParameter(nh_, "left_hand_name", left_hand_name_);
  getStringParameter(nh_, "right_arm_name", right_arm_name_);
  getStringParameter(nh_, "left_arm_name", left_arm_name_);
  getStringParameter(nh_, "both_arms_name", both_arms_name_);

  // Decide what robot we are working with
  if (robot_model->getName() == "baxter")
  {
    dual_arm_ = true;

    // Load arm groups
    left_arm_ = robot_model->getJointModelGroup(left_arm_name_);
    right_arm_ = robot_model->getJointModelGroup(right_arm_name_);
    both_arms_ = robot_model->getJointModelGroup(both_arms_name_);
  }
  else if (robot_model->getName() == "jacob")
  {
    dual_arm_ = false;

    // Load arm groups
    right_arm_ = robot_model->getJointModelGroup(right_arm_name_);
  }
  else
  {
    ROS_WARN_STREAM_NAMED("manipulation_data","Unknown type of robot '" << robot_model->getName() << "'");
  }

  ROS_INFO_STREAM_NAMED("manipulation_data","ManipulationData Ready.");
}

bool getDoubleParameter(ros::NodeHandle &nh, const std::string &param_name, double &value)
{
  // Load a param
  if (!nh.hasParam(param_name))
  {
    ROS_ERROR_STREAM_NAMED("manipulation_data","Missing parameter '" << param_name << "'. Searching in namespace: " << nh.getNamespace());
    return false;
  }
  nh.getParam(param_name, value);
  ROS_DEBUG_STREAM_NAMED("manipulation_data","Loaded parameter '" << param_name << "' with value " << value);

  return true;
}

bool getIntParameter(ros::NodeHandle &nh, const std::string &param_name, int &value)
{
  // Load a param
  if (!nh.hasParam(param_name))
  {
    ROS_ERROR_STREAM_NAMED("manipulation_data","Missing parameter '" << param_name << "'. Searching in namespace: " << nh.getNamespace());
    return false;
  }
  nh.getParam(param_name, value);
  ROS_DEBUG_STREAM_NAMED("manipulation_data","Loaded parameter '" << param_name << "' with value " << value);

  return true;
}

bool getStringParameter(ros::NodeHandle &nh, const std::string &param_name, std::string &value)
{
  // Load a param
  if (!nh.hasParam(param_name))
  {
    ROS_ERROR_STREAM_NAMED("manipulation_data","Missing parameter '" << param_name << "'. Searching in namespace: " << nh.getNamespace());
    return false;
  }
  nh.getParam(param_name, value);
  ROS_DEBUG_STREAM_NAMED("manipulation_data","Loaded parameter '" << param_name << "' with value " << value);

  return true;
}

} // end namespace
