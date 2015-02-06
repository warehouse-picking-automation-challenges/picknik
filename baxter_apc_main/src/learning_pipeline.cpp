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
   Desc:   Generates poses for the shelf to populate the database
*/

// ROS
#include <ros/ros.h>

#include <baxter_apc_main/learning_pipeline.h>

namespace baxter_apc_main
{

LearningPipeline::LearningPipeline(bool verbose, moveit_visual_tools::MoveItVisualToolsPtr visual_tools,
                 planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor,
                 ShelfObjectPtr shelf, bool use_experience, bool show_database)
  : ManipulationPipeline(verbose, visual_tools, planning_scene_monitor, shelf, use_experience, show_database)
{

  ROS_INFO_STREAM_NAMED("learning_pipeline","LearningPipeline Ready.");
}

bool LearningPipeline::generateTrainingGoals(ShelfObjectPtr shelf)
{
  // Generate all grasps
  for (BinObjectMap::const_iterator bin_it = shelf->getBins().begin(); bin_it != shelf->getBins().end(); bin_it++)
  {
    BinObjectPtr bin = bin_it->second;

    //ROS_DEBUG_STREAM_NAMED("learning_pipeline","Adding poses for bin " << bin->getName());

    // Create new entry
    bin_experience_data_[bin->getName()] = BinExperienceData();

    // Populate poses
    generateTrainingGoalsBin( shelf->bottom_right_ * bin->bottom_right_, bin_experience_data_[bin->getName()].poses);
  }

  // Display grasps
  bool valid_only = false;
  //displayGrasps(valid_only);


  // Analyze reachability
  testGrasps();

  // Testing
  //visualizePose(Eigen::Affine3d::Identity(), left_arm_);
  //visualizePose(Eigen::Affine3d::Identity(), right_arm_);
}

bool LearningPipeline::visualizePose(Eigen::Affine3d grasp_pose, const moveit::core::JointModelGroup *arm_jmg)
{
  const moveit::core::JointModelGroup* ee_jmg = robot_model_->getJointModelGroup(grasp_datas_[arm_jmg].ee_group_);

  // Rotate based on EE type
  grasp_pose = grasp_pose * grasp_datas_[arm_jmg].grasp_pose_to_eef_pose_;

  visual_tools_->publishArrow(grasp_pose);
  visual_tools_->publishEEMarkers(grasp_pose, ee_jmg, rviz_visual_tools::GREEN);
}

bool LearningPipeline::generateTrainingGoalsBin(Eigen::Affine3d bin_transpose, EigenSTL::vector_Affine3d &poses)
{
  Eigen::Affine3d pose;

  // Generate for one bin
  for (double y = POSE_DISCRETIZATION; y < BIN_WIDTH; y += POSE_DISCRETIZATION)
  {
    for (double z = POSE_DISCRETIZATION; z < BIN_HEIGHT / 1.25 - POSE_DISCRETIZATION; z += POSE_DISCRETIZATION)
    {
      //std::cout << "y: " << y << " z: " << z << std::endl;

      pose = Eigen::Affine3d::Identity();
      pose.translation().x() = 0.09; // penetration depth into shelf, this is 0.005 m beyond line (5mm)
      pose.translation().y() = y;
      pose.translation().z() = z;

      pose = bin_transpose * pose;
      poses.push_back(pose);
    }
  }
}

bool LearningPipeline::testGrasps()
{
  // TODO: test both arms
  const moveit::core::JointModelGroup* jmg = right_arm_;
  const moveit::core::JointModelGroup* ee_jmg = robot_model_->getJointModelGroup(grasp_datas_[jmg].ee_group_);

  // Convert grasp vectors to grasp msgs
  std::vector<moveit_msgs::Grasp> possible_grasps;

  // Statistics for testing
  std::size_t total_generated_grasps = 0;
  std::size_t total_valid_ik_grasps = 0;
  std::size_t total_collision_free_grasps = 0;

  for (BinExperienceDataMap::iterator bin_it = bin_experience_data_.begin(); 
       bin_it != bin_experience_data_.end(); bin_it++)
  {
    BinExperienceData &data = bin_it->second;

    for (std::size_t i = 0; i < data.poses.size(); ++i)
    {
      moveit_msgs::Grasp new_grasp;
      total_generated_grasps++;

      /* The estimated probability of success for this grasp, or some other measure of how "good" it is.
       * Here we base bias the score based on how far the wrist is from the surface, preferring a greater
       * distance to prevent wrist/end effector collision with the table
       */
      new_grasp.grasp_quality = 1; // TODO

      // A name for this grasp
      static int grasp_id = 0;
      new_grasp.id = "Grasp" + boost::lexical_cast<std::string>(grasp_id);
      ++grasp_id;

      // The internal posture of the hand for the pre-grasp only positions are used
      new_grasp.pre_grasp_posture = grasp_datas_[jmg].pre_grasp_posture_;

      // The internal posture of the hand for the grasp positions and efforts are used
      new_grasp.grasp_posture = grasp_datas_[jmg].grasp_posture_;        

      // The position of the end-effector for the grasp relative to a reference frame (that is always specified elsewhere, not in this message)
      geometry_msgs::PoseStamped grasp_pose_msg;
      grasp_pose_msg.header.stamp = ros::Time::now();
      grasp_pose_msg.header.frame_id = robot_model_->getModelFrame();

      // Transform based on EE type
      Eigen::Affine3d eigen_grasp_pose = data.poses[i] * grasp_datas_[jmg].grasp_pose_to_eef_pose_;
      tf::poseEigenToMsg(eigen_grasp_pose, grasp_pose_msg.pose);
      new_grasp.grasp_pose = grasp_pose_msg;

      // debug mode
      if (false)
      {
        visual_tools_->publishArrow(grasp_pose_msg.pose, rviz_visual_tools::RED);
        visual_tools_->publishEEMarkers(grasp_pose_msg.pose, ee_jmg);
        ros::Duration(1).sleep();
      }

      // the maximum contact force to use while grasping (<=0 to disable)
      new_grasp.max_contact_force = 0;

      // ---------------------------------------------------------------------------------------------
      // Grasp parameters

      // Create re-usable approach motion
      moveit_msgs::GripperTranslation pre_grasp_approach;
      pre_grasp_approach.direction.header.stamp = ros::Time::now();
      pre_grasp_approach.desired_distance = grasp_datas_[jmg].finger_to_palm_depth_ + 0.1; // The distance the origin of a robot link needs to travel
      pre_grasp_approach.min_distance = grasp_datas_[jmg].finger_to_palm_depth_; // half of the desired? Untested.

      // Create re-usable retreat motion
      moveit_msgs::GripperTranslation post_grasp_retreat;
      post_grasp_retreat.direction.header.stamp = ros::Time::now();
      post_grasp_retreat.desired_distance = grasp_datas_[jmg].finger_to_palm_depth_ + 0.1; // The distance the origin of a robot link needs to travel
      post_grasp_retreat.min_distance = grasp_datas_[jmg].finger_to_palm_depth_; // half of the desired? Untested.

      // Angled with pose -------------------------------------------------------------------------------------
      // Approach with respect to end effector orientation

      // Approach
      bool approach_down = false;
      if (approach_down)
      {
        pre_grasp_approach.direction.header.frame_id = robot_model_->getModelFrame();
        pre_grasp_approach.direction.vector.z = -1;
      }
      else
      {
        pre_grasp_approach.direction.header.frame_id = grasp_datas_[jmg].parent_link_name_;
        pre_grasp_approach.direction.vector.z = 1;
      }
      pre_grasp_approach.direction.vector.x = 0;
      pre_grasp_approach.direction.vector.y = 0;
      new_grasp.pre_grasp_approach = pre_grasp_approach;

      // Retreat
      if (approach_down)
      {
        post_grasp_retreat.direction.header.frame_id = robot_model_->getModelFrame();
        post_grasp_retreat.direction.vector.z = 1;
      }
      else
      {
        post_grasp_retreat.direction.header.frame_id = grasp_datas_[jmg].parent_link_name_;
        post_grasp_retreat.direction.vector.z = -1;
      }
      post_grasp_retreat.direction.vector.x = 0;
      post_grasp_retreat.direction.vector.y = 0;
      new_grasp.post_grasp_retreat = post_grasp_retreat;

      // Add to vector
      possible_grasps.push_back(new_grasp);
    }
  }

  // Filter the grasp for only the ones that are reachable
  bool filter_pregrasps = true;
  std::vector<moveit_grasps::GraspSolution> filtered_grasps;
  std::cout << std::endl;
  std::cout << std::endl;
  ROS_INFO_STREAM_NAMED("learning_pipeline","Filtering grasps using IK, may take a minute");
  grasp_filter_->filterGrasps(possible_grasps, filtered_grasps, filter_pregrasps,
                              grasp_datas_[jmg].parent_link_name_, jmg);
  total_valid_ik_grasps = filtered_grasps.size();

  // Visualize valid grasps as arrows
  if (verbose_)
  {
    ROS_INFO_STREAM_NAMED("learning_pipeline","Showing valid filtered grasp poses");
    for (std::size_t i = 0; i < filtered_grasps.size(); ++i)
    {      
      publishGraspArrow(filtered_grasps[i].grasp_.grasp_pose.pose, jmg, rviz_visual_tools::BLUE);
      ros::Duration(0.001).sleep();
    }    
  }

  // Show just one grasp - DEBUG mode
  if (false)
  {
    double animation_speed = 0.5;
    for (std::size_t i = 0; i < 20; ++i)
    {
      std::cout << std::endl;
      std::cout << "-------------------------------------------------------" << std::endl;
      std::cout << "Showing animated grasp 0 " << std::endl;
      visual_tools_->publishAnimatedGrasp(possible_grasps[0], ee_jmg, animation_speed);    
      ros::Duration(0).sleep();
    }
  }

  // Visualize animated grasps
  if (verbose_ && false)
  {
    ROS_INFO_STREAM_NAMED("learning_pipeline","Showing animated grasps");
    double animation_speed = 0.01;
    visual_tools_->publishAnimatedGrasps(possible_grasps, ee_jmg, animation_speed);
  }

  // Visualize IK solutions
  if (verbose_ && true)
  {
    // Convert the filtered_grasps into a format moveit_visual_tools can use
    std::vector<trajectory_msgs::JointTrajectoryPoint> ik_solutions;
    ik_solutions.resize(filtered_grasps.size());
    for (std::size_t i = 0; i < filtered_grasps.size(); ++i)
    {
      ik_solutions[i].positions = filtered_grasps[i].grasp_ik_solution_;
    }
    ROS_INFO_STREAM_NAMED("learning_pipeline","Showing IK solutions of grasps");
    double display_time = 2;
    visual_tools_->publishIKSolutions(ik_solutions, jmg->getName(), display_time);
  }

  ROS_INFO_STREAM_NAMED("learning_pipeline","Filtering grasps by collision");

  // Filter grasps based on collision
  {
    planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_); // Lock planning scene
    (*robot_state_) = scene->getCurrentState();
  }

  openEndEffector(true, robot_state_); // to be passed to the grasp filter

  // Filter by collision
  grasp_filter_->filterGraspsInCollision(filtered_grasps, planning_scene_monitor_, jmg, robot_state_, verbose_);
  total_collision_free_grasps = filtered_grasps.size();

  // Visualize valid grasps after collision filtering with arrows
  if (verbose_)
  {
    visual_tools_->deleteAllMarkers();
    ROS_INFO_STREAM_NAMED("learning_pipeline","Showing valid filtered grasp poses");
    for (std::size_t i = 0; i < filtered_grasps.size(); ++i)
    {      
      publishGraspArrow(filtered_grasps[i].grasp_.grasp_pose.pose, jmg, rviz_visual_tools::GREEN);
      ros::Duration(0.001).sleep();
    }    
  }

  // Output statistics
  std::cout << std::endl;
  std::cout << "-------------------------------------------------------" << std::endl;
  std::cout << "Total Generated Grasps: " << total_generated_grasps << std::endl;
  std::cout << "Grasps with valid IK:   " << total_valid_ik_grasps << std::endl;
  std::cout << "Percent valid: " << (double(total_valid_ik_grasps) / total_generated_grasps * 100.0) << " %" << std::endl;
  std::cout << std::endl;
  std::cout << "Grasps not in collision: " << total_collision_free_grasps << std::endl;
  std::cout << "Percent valid: " << (double(total_collision_free_grasps) / total_generated_grasps * 100.0) << " %" << std::endl;
  std::cout << "-------------------------------------------------------" << std::endl;
  std::cout << std::endl;

  // Visualize IK solutions after collision filtering
  if (verbose_ && true)
  {
    // Convert the filtered_grasps into a format moveit_visual_tools can use
    std::vector<trajectory_msgs::JointTrajectoryPoint> ik_solutions;
    ik_solutions.resize(filtered_grasps.size());
    for (std::size_t i = 0; i < filtered_grasps.size(); ++i)
    {
      ik_solutions[i].positions = filtered_grasps[i].grasp_ik_solution_;
    }
    ROS_INFO_STREAM_NAMED("learning_pipeline","Showing IK solutions of grasps that are not in collision");
    double display_time = 0.1;
    visual_tools_->publishIKSolutions(ik_solutions, jmg->getName(), display_time);
  }

}

bool LearningPipeline::displayGrasps(bool valid_only)
{
  for (BinExperienceDataMap::iterator bin_it = bin_experience_data_.begin(); 
       bin_it != bin_experience_data_.end(); bin_it++)
  {
    BinExperienceData &data = bin_it->second;

    for (std::size_t i = 0; i < data.poses.size(); ++i)
    {
      visual_tools_->publishArrow(data.poses[i]);
      ros::Duration(0.001).sleep();          
    }
  }
}

} // namespace
