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

LearningPipeline::LearningPipeline(bool verbose, 
                                   mvt::MoveItVisualToolsPtr visual_tools,
                                   mvt::MoveItVisualToolsPtr visual_tools_display,
                                   planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor,
                                   ShelfObjectPtr shelf, bool use_experience, bool show_database)
  : ManipulationPipeline(verbose, visual_tools, visual_tools_display, planning_scene_monitor, shelf, use_experience, show_database)
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

  const moveit::core::JointModelGroup* arm_jmg = right_arm_;

  // Analyze reachability and filter out invalid grasps
  analyzeGrasps(arm_jmg);

  // Training lightning by planning to every grasp
  planToGrasps(arm_jmg);

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
  visual_tools_->publishEEMarkers(grasp_pose, ee_jmg, rvt::GREEN);
}

bool LearningPipeline::generateTrainingGoalsBin(Eigen::Affine3d bin_transpose, EigenSTL::vector_Affine3d &poses)
{
  Eigen::Affine3d pose;

  // Generate for one bin
  for (double y = POSE_HORIZONTAL_MARGIN; y < BIN_WIDTH - POSE_HORIZONTAL_MARGIN; y += POSE_DISCRETIZATION)
  {
    for (double z = POSE_BOTTOM_MARGIN; z < POSE_TOP_MARGIN; z += POSE_DISCRETIZATION)
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

bool LearningPipeline::analyzeGrasps(const moveit::core::JointModelGroup* arm_jmg)
{
  const moveit::core::JointModelGroup* ee_jmg = robot_model_->getJointModelGroup(grasp_datas_[arm_jmg].ee_group_);

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
      new_grasp.pre_grasp_posture = grasp_datas_[arm_jmg].pre_grasp_posture_;

      // The internal posture of the hand for the grasp positions and efforts are used
      new_grasp.grasp_posture = grasp_datas_[arm_jmg].grasp_posture_;

      // The position of the end-effector for the grasp relative to a reference frame (that is always specified elsewhere, not in this message)
      geometry_msgs::PoseStamped grasp_pose_msg;
      grasp_pose_msg.header.stamp = ros::Time::now();
      grasp_pose_msg.header.frame_id = robot_model_->getModelFrame();

      // Transform based on EE type
      Eigen::Affine3d eigen_grasp_pose = data.poses[i] * grasp_datas_[arm_jmg].grasp_pose_to_eef_pose_;
      tf::poseEigenToMsg(eigen_grasp_pose, grasp_pose_msg.pose);
      new_grasp.grasp_pose = grasp_pose_msg;

      // debug mode
      if (false)
      {
        visual_tools_->publishArrow(grasp_pose_msg.pose, rvt::RED);
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
      pre_grasp_approach.desired_distance = grasp_datas_[arm_jmg].finger_to_palm_depth_ + APPROACH_DISTANCE_DESIRED; // The distance the origin of a robot link needs to travel
      pre_grasp_approach.min_distance = grasp_datas_[arm_jmg].finger_to_palm_depth_; // half of the desired? Untested.

      // Create re-usable retreat motion
      moveit_msgs::GripperTranslation post_grasp_retreat;
      post_grasp_retreat.direction.header.stamp = ros::Time::now();
      post_grasp_retreat.desired_distance = grasp_datas_[arm_jmg].finger_to_palm_depth_ + APPROACH_DISTANCE_DESIRED; // The distance the origin of a robot link needs to travel
      post_grasp_retreat.min_distance = grasp_datas_[arm_jmg].finger_to_palm_depth_; // half of the desired? Untested.

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
        pre_grasp_approach.direction.header.frame_id = grasp_datas_[arm_jmg].parent_link_name_;
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
        post_grasp_retreat.direction.header.frame_id = grasp_datas_[arm_jmg].parent_link_name_;
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
  std::cout << std::endl;
  std::cout << std::endl;
  ROS_INFO_STREAM_NAMED("learning_pipeline","Filtering grasps using IK, may take a minute");
  grasp_filter_->filterGrasps(possible_grasps, filtered_grasps_, filter_pregrasps,
                              grasp_datas_[arm_jmg].parent_link_name_, arm_jmg);
  total_valid_ik_grasps = filtered_grasps_.size();

  // Visulizations
  if (verbose_)
  {
    // Visualize valid grasps as arrows with cartesian path as well
    ROS_DEBUG_STREAM_NAMED("learning.ik_filtered_grasps","enabled" << visualizeGrasps(filtered_grasps_, arm_jmg));

    // Visualize animated grasps
    double animation_speed = 0.01;
    ROS_DEBUG_STREAM_NAMED("learning.ik_animated_grasps","Showing animated grasps" 
                           << visual_tools_->publishAnimatedGrasps(possible_grasps, ee_jmg, animation_speed));

    // Visualize IK solutions
    ROS_DEBUG_STREAM_NAMED("learning.ik_filtered_solutions","enabled" << visualizeIKSolutions(filtered_grasps_, arm_jmg));
  }

  ROS_INFO_STREAM_NAMED("learning_pipeline","Filtering grasps by collision");

  // Filter grasps based on collision
  {
    planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_); // Lock planning scene
    (*robot_state_) = scene->getCurrentState();
  }

  openEndEffector(true, robot_state_); // to be passed to the grasp filter

  // Filter by collision
  ROS_INFO_STREAM_NAMED("learning","Filtering grasps by collision checking");
  bool filter_verbose = false;  
  grasp_filter_->filterGraspsInCollision(filtered_grasps_, planning_scene_monitor_, arm_jmg, robot_state_, filter_verbose);
  total_collision_free_grasps = filtered_grasps_.size();

  // Visualize valid grasps after collision filtering with arrows
  if (verbose_)
  {
    // Visualize valid grasps as arrows with cartesian path as well
    bool show_cartesian_path = false;
    ROS_DEBUG_STREAM_NAMED("learning.collision_filtered_grasps","enabled" 
                           << visual_tools_->deleteAllMarkers()
                           << visualizeGrasps(filtered_grasps_, arm_jmg, show_cartesian_path));

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
    ROS_DEBUG_STREAM_NAMED("learning.collision_filtered_solutions","enabled" << visualizeIKSolutions(filtered_grasps_, arm_jmg));
  }

}

bool LearningPipeline::planToGrasps(const moveit::core::JointModelGroup *arm_jmg)
{
  moveit::core::RobotStatePtr start_state(new moveit::core::RobotState(*robot_state_));
  moveit::core::RobotStatePtr goal_state(new moveit::core::RobotState(*robot_state_));

  // Set start state to initial pose
  setToDefaultPosition(start_state);

  double path_length = 0;
  for (std::size_t i = 0; i < filtered_grasps_.size(); ++i)
  {
    moveit_grasps::GraspSolution& solution = filtered_grasps_[i];

    ROS_INFO_STREAM_NAMED("learning","Planning to grasp " << i);
    grasps_->publishGraspArrow(solution.grasp_.grasp_pose.pose, grasp_datas_[arm_jmg], rvt::GREEN, path_length);

    goal_state->setJointGroupPositions(arm_jmg, solution.grasp_ik_solution_);

    bool execute_trajectory = false;
    bool show_database = true;
    if (!move(start_state, goal_state, arm_jmg, verbose_, execute_trajectory, show_database))
    {
      ROS_ERROR_STREAM_NAMED("pipeline","Plan to grasp " << i << " failed");
      return false;
    }
  }

  return true;
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

bool LearningPipeline::testSingleGraspIK()
{
  /*
  const moveit::core::JointModelGroup* jmg = left_arm_;
  const moveit::core::JointModelGroup* ee_jmg = robot_model_->getJointModelGroup(grasp_datas_[jmg].ee_group_);

  BinExperienceData &data = bin_experience_data_["bin_H"];

  // Transform based on EE type
  std::size_t i = 0;
  Eigen::Affine3d eigen_grasp_pose = data.poses[i] * grasp_datas_[jmg].grasp_pose_to_eef_pose_;

  // debug mode
  if (true)
  {
    visual_tools_->publishArrow(eigen_grasp_pose, rvt::RED);
    visual_tools_->publishEEMarkers(eigen_grasp_pose, ee_jmg);
    ros::Duration(1).sleep();
  }

  // Seed state - start at zero
  std::vector<double> ik_seed_state(7); // fill with zeros
  // TODO do not assume 7 dof

  std::vector<double> grasp_ik_solution;
  std::vector<double> pregrasp_ik_solution;
  moveit_msgs::MoveItErrorCodes error_code;
  geometry_msgs::PoseStamped ik_pose;

  // Process the assigned grasps
  for( int i = ik_thread_struct.grasps_id_start_; i < ik_thread_struct.grasps_id_end_; ++i )
  {
    //ROS_DEBUG_STREAM_NAMED("filter", "Checking grasp #" << i);

    // Clear out previous solution just in case - not sure if this is needed
    grasp_ik_solution.clear(); // TODO remove
    pregrasp_ik_solution.clear(); // TODO remove

    // Transform current pose to frame of planning group
    ik_pose = ik_thread_struct.possible_grasps_[i].grasp_pose;
    Eigen::Affine3d eigen_pose;
    tf::poseMsgToEigen(ik_pose.pose, eigen_pose);
    eigen_pose = ik_thread_struct.link_transform_ * eigen_pose;
    tf::poseEigenToMsg(eigen_pose, ik_pose.pose);

    // Test it with IK
    ik_thread_struct.kin_solver_->
      searchPositionIK(ik_pose.pose, ik_seed_state, ik_thread_struct.timeout_, grasp_ik_solution, error_code);

    // Results
    if( error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS )
    {
      //ROS_INFO_STREAM_NAMED("filter","Found IK Solution");

      // Copy solution to seed state so that next solution is faster
      ik_seed_state = grasp_ik_solution;

      // Start pre-grasp section ----------------------------------------------------------
      if (ik_thread_struct.filter_pregrasp_)       // optionally check the pregrasp
      {
        // Convert to a pre-grasp
        ik_pose = Grasps::getPreGraspPose(ik_thread_struct.possible_grasps_[i], ik_thread_struct.ee_parent_link_);

        // Transform current pose to frame of planning group
        Eigen::Affine3d eigen_pose;
        tf::poseMsgToEigen(ik_pose.pose, eigen_pose);
        eigen_pose = ik_thread_struct.link_transform_ * eigen_pose;
        tf::poseEigenToMsg(eigen_pose, ik_pose.pose);

        // Test it with IK
        ik_thread_struct.kin_solver_->
          searchPositionIK(ik_pose.pose, ik_seed_state, ik_thread_struct.timeout_, pregrasp_ik_solution, error_code);

        // Results
        if( error_code.val == moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION )
        {
          ROS_WARN_STREAM_NAMED("filter","Unable to find IK solution for pre-grasp pose.");
          continue;
        }
        else if( error_code.val == moveit_msgs::MoveItErrorCodes::TIMED_OUT )
        {
          //ROS_DEBUG_STREAM_NAMED("filter","Unable to find IK solution for pre-grasp pose: Timed Out.");
          continue;
        }
        else if( error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS )
        {
          ROS_INFO_STREAM_NAMED("filter","IK solution error for pre-grasp: MoveItErrorCodes.msg = " << error_code);
          continue;
        }
      }
      else
      {
        ROS_WARN_STREAM_NAMED("pipeline","Not filtering pre-grasp - GraspSolution may have bad data");
      }
      // Both grasp and pre-grasp have passed, create the solution
      GraspSolution grasp_solution;
      grasp_solution.grasp_ = ik_thread_struct.possible_grasps_[i];
      grasp_solution.grasp_ik_solution_ = grasp_ik_solution;
      grasp_solution.pregrasp_ik_solution_ = pregrasp_ik_solution;

      // Lock the result vector so we can add to it for a second
      {
        boost::mutex::scoped_lock slock(*ik_thread_struct.lock_);
        ik_thread_struct.filtered_grasps__.push_back( grasp_solution );
      }

      // End pre-grasp section -------------------------------------------------------
    }
    else if( error_code.val == moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION )
    {
      //ROS_WARN_STREAM_NAMED("filter","Unable to find IK solution for pose: No Solution");
    }
    else if( error_code.val == moveit_msgs::MoveItErrorCodes::TIMED_OUT )
    {
      //ROS_DEBUG_STREAM_NAMED("filter","Unable to find IK solution for pose: Timed Out.");
    }
    else
      ROS_INFO_STREAM_NAMED("filter","IK solution error: MoveItErrorCodes.msg = " << error_code);
  }
  
  */
}

bool LearningPipeline::visualizeGrasps(std::vector<moveit_grasps::GraspSolution> filtered_grasps, 
                                       const moveit::core::JointModelGroup *arm_jmg,
                                       bool show_cartesian_path)
{
  ROS_INFO_STREAM_NAMED("learning_pipeline","Showing valid filtered grasp poses");

  // Publish in batch
  //visual_tools_->enableBatchPublishing(true);

  // Get the-grasp
  moveit::core::RobotStatePtr the_grasp(new moveit::core::RobotState(*robot_state_));

  Eigen::Vector3d approach_direction;
  approach_direction << -1, 0, 0; // backwards towards robot body
  double desired_approach_distance = 0.45; //0.12; //0.15;
  std::vector<robot_state::RobotStatePtr> robot_state_trajectory;
  double path_length;
  double max_path_length = 0; // statistics
  for (std::size_t i = 0; i < filtered_grasps.size(); ++i)
  {
    if (!ros::ok())
      return false;

    if (show_cartesian_path)
    {
      the_grasp->setJointGroupPositions(arm_jmg, filtered_grasps[i].grasp_ik_solution_);
      
      if (!computeStraightLinePath(approach_direction, desired_approach_distance,
                                   robot_state_trajectory, the_grasp, arm_jmg, path_length))
      {
        ROS_WARN_STREAM_NAMED("pipeline","Unable to find straight line path");
      }

      // Statistics
      if (path_length > max_path_length)
        max_path_length = path_length;

      bool blocking = false;
      double speed = 0.01;
      visual_tools_->publishTrajectoryPath(robot_state_trajectory, arm_jmg, speed, blocking);
    }
    grasps_->publishGraspArrow(filtered_grasps[i].grasp_.grasp_pose.pose, grasp_datas_[arm_jmg], rvt::BLUE, path_length);
  }
  //visual_tools_->triggerBatchPublishAndDisable();

  ROS_INFO_STREAM_NAMED("learning","Maximum path length in approach trajetory was " << max_path_length);

  return true;
}

bool LearningPipeline::visualizeIKSolutions(std::vector<moveit_grasps::GraspSolution> filtered_grasps, const moveit::core::JointModelGroup* arm_jmg)
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

  return visual_tools_->publishIKSolutions(ik_solutions, arm_jmg->getName(), display_time);
}

} // namespace
