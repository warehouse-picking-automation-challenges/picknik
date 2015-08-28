/*********************************************************************
 * Software License Agreement
 *
 *  Copyright (c) 2015, Dave Coleman <dave@dav.ee>
 *  All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *********************************************************************/

/* Author: Dave Coleman <dave@dav.ee>
   Desc:   Manage the manipulation of MoveIt
*/

// PickNik
#include <picknik_main/manipulation.h>
#include <picknik_main/product_simulator.h>

// MoveIt
#include <moveit/ompl/model_based_planning_context.h>
#include <moveit/collision_detection/world.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/macros/console_colors.h>
#include <moveit/robot_state/conversions.h>

// OMPL
#include <ompl/tools/lightning/Lightning.h>
#include <ompl/tools/thunder/Thunder.h>

// moveit_grasps
#include <moveit_grasps/grasp_generator.h>

namespace picknik_main
{
Manipulation::Manipulation(bool verbose, VisualsPtr visuals,
                           planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor,
                           ManipulationDataPtr config, moveit_grasps::GraspDatas grasp_datas,
                           RemoteControlPtr remote_control, bool fake_execution,
                           TactileFeedbackPtr tactile_feedback)
  : nh_("~")
  , verbose_(verbose)
  , visuals_(visuals)
  , planning_scene_monitor_(planning_scene_monitor)
  , config_(config)
  , grasp_datas_(grasp_datas)
  , remote_control_(remote_control)
  , tactile_feedback_(tactile_feedback)
{
  // Create initial robot state
  {
    planning_scene_monitor::LockedPlanningSceneRO scene(
        planning_scene_monitor_);  // Lock planning scene
    current_state_.reset(new moveit::core::RobotState(scene->getCurrentState()));
  }  // end scoped pointer of locked planning scene

  // Set shared robot states for all visual tools
  visuals_->setSharedRobotState(current_state_);

  // Set robot model
  robot_model_ = current_state_->getRobotModel();

  // Load execution interface
  execution_interface_.reset(new ExecutionInterface(verbose_, remote_control_, visuals_,
                                                    grasp_datas_, planning_scene_monitor_, config_,
                                                    current_state_, fake_execution));

  // Load logging capability
  if (config_->use_experience_setup_)
  {
    if (config_->experience_type_ == "thunder")
      logging_file_.open("/home/dave/ompl_storage/thunder_logging.csv",
                         std::ios::out | std::ios::app);
    else if (config_->experience_type_ == "lightning")
      logging_file_.open("/home/dave/ompl_storage/lightning_logging.csv",
                         std::ios::out | std::ios::app);
    else
      ROS_ERROR_STREAM_NAMED("manipulation",
                             "Unsupported experience type: " << config_->experience_type_);
  }

  // Load grasp generator
  grasp_generator_.reset(new moveit_grasps::GraspGenerator(visuals_->grasp_markers_));
  // setStateWithOpenEE(true, current_state_); // so that grasp filter is started up with EE open
  grasp_filter_.reset(new moveit_grasps::GraspFilter(current_state_, visuals_->grasp_markers_));
  grasp_planner_.reset(new moveit_grasps::GraspPlanner(visuals_->trajectory_lines_));
  grasp_planner_->setWaitForNextStepCallback(
      boost::bind(&picknik_main::RemoteControl::waitForNextStep, remote_control_, _1));

  // Done
  ROS_INFO_STREAM_NAMED("manipulation", "Manipulation Ready.");
}

bool Manipulation::computeCartesianWaypointPath(
    JointModelGroup* arm_jmg, const moveit::core::RobotStatePtr start_state,
    const EigenSTL::vector_Affine3d& waypoints,
    moveit_grasps::GraspTrajectories& segmented_cartesian_traj)
{
  // End effector parent link (arm tip for ik solving)
  const moveit::core::LinkModel* ik_tip_link = grasp_datas_[arm_jmg]->parent_link_;

  // Resolution of trajectory
  const double max_step = 0.01;  // The maximum distance in Cartesian space between consecutive
                                 // points on the resulting path

  // Jump threshold for preventing consequtive joint values from 'jumping' by a large amount in
  // joint space
  const double jump_threshold = config_->jump_threshold_;  // aka jump factor

  // Collision setting
  const bool collision_checking_verbose = false;
  const bool only_check_self_collision = false;

  // Reference frame setting
  const bool global_reference_frame = true;

  // Check for kinematic solver
  if (!arm_jmg->canSetStateFromIK(ik_tip_link->getName()))
  {
    ROS_ERROR_STREAM_NAMED("manipulation.waypoints", "No IK Solver loaded - make sure "
                                                     "moveit_config/kinamatics.yaml is loaded in "
                                                     "this namespace");
    return false;
  }

  // Results
  double last_valid_percentage;

  std::size_t attempts = 0;
  static const std::size_t MAX_IK_ATTEMPTS = 5;
  bool valid_path_found = false;
  while (attempts < MAX_IK_ATTEMPTS)
  {
    if (attempts > 0)
    {
      // std::cout << std::endl;
      // std::cout << "-------------------------------------------------------" << std::endl;
      ROS_DEBUG_STREAM_NAMED("manipulation.waypoints", "Attempting IK solution, attempt # "
                                                           << attempts + 1);
    }
    attempts++;

    // Collision check
    boost::scoped_ptr<planning_scene_monitor::LockedPlanningSceneRO> ls;
    ls.reset(new planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor_));
    moveit::core::GroupStateValidityCallbackFn constraint_fn = boost::bind(
        &isStateValid, static_cast<const planning_scene::PlanningSceneConstPtr&>(*ls).get(),
        collision_checking_verbose, only_check_self_collision, visuals_, _1, _2, _3);

    // Test
    moveit::core::RobotState temp_state(*start_state);

    // Compute Cartesian Path
    segmented_cartesian_traj.clear();
    last_valid_percentage = temp_state.computeCartesianPathSegmented(
        arm_jmg, segmented_cartesian_traj, ik_tip_link, waypoints, global_reference_frame, max_step,
        jump_threshold, constraint_fn, kinematics::KinematicsQueryOptions());

    ROS_DEBUG_STREAM_NAMED("manipulation.waypoints", "Cartesian last_valid_percentage: "
                                                         << last_valid_percentage
                                                         << " number of segments in trajectory: "
                                                         << segmented_cartesian_traj.size());

    double min_allowed_valid_percentage = 0.9;
    if (last_valid_percentage == 0)
    {
      ROS_DEBUG_STREAM_NAMED("manipulation.waypoints",
                             "Failed to computer cartesian path: last_valid_percentage is 0");
    }
    else if (last_valid_percentage < min_allowed_valid_percentage)
    {
      ROS_DEBUG_STREAM_NAMED("manipulation.waypoints",
                             "Resulting cartesian path is less than "
                                 << min_allowed_valid_percentage
                                 << " % of the desired distance, % valid: "
                                 << last_valid_percentage);
    }
    else
    {
      ROS_DEBUG_STREAM_NAMED("manipulation.waypoints", "Found valid cartesian path");
      valid_path_found = true;
      break;
    }
  }  // end while AND scoped pointer of locked planning scenep

  if (!valid_path_found)
  {
    ROS_INFO_STREAM_NAMED("manipulation.waypoints",
                          "UNABLE to find valid waypoint cartesian path after " << MAX_IK_ATTEMPTS
                                                                                << " attempts");
    return false;
  }

  return true;
}

bool Manipulation::moveCartesianWaypointPath(JointModelGroup* arm_jmg,
                                             EigenSTL::vector_Affine3d waypoints)
{
  // Debug
  visuals_->visual_tools_->publishAxisLabeled(waypoints.front(), "start");

  // Move to first position
  if (!moveToEEPose(waypoints.front(), config_->main_velocity_scaling_factor_, arm_jmg))
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Unable to move to starting pose");
    return false;
  }

  // Calculate remaining trajectory
  moveit_grasps::GraspTrajectories segmented_cartesian_traj;
  if (!computeCartesianWaypointPath(arm_jmg, getCurrentState(), waypoints,
                                    segmented_cartesian_traj))
  {
    ROS_INFO_STREAM_NAMED("manipulation", "Unable to plan circular path");
    return false;
  }

  // Combine segmented trajectory into single trajectory
  moveit_grasps::GraspTrajectories single_cartesian_traj;
  single_cartesian_traj.resize(1);
  for (std::size_t i = 0; i < segmented_cartesian_traj.size(); ++i)
  {
    single_cartesian_traj[0].insert(single_cartesian_traj[0].end(),
                                    segmented_cartesian_traj[i].begin(),
                                    segmented_cartesian_traj[i].end());
  }

  visuals_->visual_tools_->publishTrajectoryPoints(single_cartesian_traj[0],
                                                   grasp_datas_[arm_jmg]->parent_link_, rvt::RAND);

  if (!executeSavedCartesianPath(single_cartesian_traj, arm_jmg, 0))
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Error executing trajectory segment " << 0);
    return false;
  }

  return true;
}

bool Manipulation::moveToSRDFPose(JointModelGroup* arm_jmg, const std::string& pose_name,
                                  double velocity_scaling_factor, bool check_validity)
{
  ROS_DEBUG_STREAM_NAMED("manipulation.superdebug", "moveToSRDFPose()");

  // Set new state to current state
  getCurrentState();

  // Set goal state to initial pose
  moveit::core::RobotStatePtr goal_state(
      new moveit::core::RobotState(*current_state_));  // Allocate robot states
  if (!goal_state->setToDefaultValues(arm_jmg, pose_name))
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Failed to set pose '" << pose_name
                                                                  << "' for planning group '"
                                                                  << arm_jmg->getName() << "'");
    return false;
  }

  // Plan
  bool execute_trajectory = true;
  if (!move(current_state_, goal_state, arm_jmg, velocity_scaling_factor, verbose_,
            execute_trajectory, check_validity))
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Unable to move to new position");
    return false;
  }

  return true;
}

bool Manipulation::moveToEEPose(const Eigen::Affine3d& ee_pose, double velocity_scaling_factor,
                                JointModelGroup* arm_jmg)
{
  // Create start and goal
  getCurrentState();
  moveit::core::RobotStatePtr goal_state(new moveit::core::RobotState(*current_state_));

  if (!getRobotStateFromPose(ee_pose, goal_state, arm_jmg))
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Unable to get robot state from pose");
    return false;
  }

  // Plan to this position
  bool verbose = true;
  bool execute_trajectory = true;
  bool check_validity = true;
  if (!move(current_state_, goal_state, arm_jmg, velocity_scaling_factor, verbose,
            execute_trajectory, check_validity))
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Failed to move EE to desired pose");
    return false;
  }

  ROS_INFO_STREAM_NAMED("manipulation", "Moved EE to desired pose successfullly");

  return true;
}

bool Manipulation::move(const moveit::core::RobotStatePtr& start,
                        const moveit::core::RobotStatePtr& goal, JointModelGroup* arm_jmg,
                        double velocity_scaling_factor, bool verbose, bool execute_trajectory,
                        bool check_validity)
{
  ROS_INFO_STREAM_NAMED("manipulation.move", "Planning to new pose with velocity scale "
                                                 << velocity_scaling_factor);

  // Check validity of start and goal
  if (check_validity && !checkCollisionAndBounds(start, goal))
  {
    ROS_ERROR_STREAM_NAMED("manipulation.move", "Potential issue with start and goal state, but "
                                                "perhaps this should not fail in the future");
    return false;
  }
  else if (!check_validity)
    ROS_WARN_STREAM_NAMED("manipulation.move",
                          "Start/goal state collision checking for move() was disabled");

  // Visualize start and goal
  if (verbose)
  {
    visuals_->start_state_->publishRobotState(start, rvt::GREEN);
    visuals_->goal_state_->publishRobotState(goal, rvt::ORANGE);
  }

  // Check if already in new position
  if (statesEqual(*start, *goal, arm_jmg))
  {
    ROS_INFO_STREAM_NAMED(
        "manipulation",
        "Not planning motion because current state and goal state are close enough.");
    return true;
  }

  // Do motion plan
  moveit_msgs::RobotTrajectory trajectory_msg;
  std::size_t plan_attempts = 0;
  while (ros::ok())
  {
    if (plan_attempts > 0)
      ROS_WARN_STREAM_NAMED("manipulation", "Previous plan attempt failed, trying again on attempt "
                                                << plan_attempts);

    if (plan(start, goal, arm_jmg, velocity_scaling_factor, verbose, trajectory_msg))
    {
      // Plan succeeded
      break;
    }
    plan_attempts++;
    if (plan_attempts > 5)
    {
      ROS_ERROR_STREAM_NAMED("manipulation", "Max number of plan attempts reached, giving up");
      return false;
    }
  }

  // Hack: do not allow a two point trajectory to be executed because there is no velcity?
  if (trajectory_msg.joint_trajectory.points.size() < 3)
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Trajectory only has "
                                               << trajectory_msg.joint_trajectory.points.size()
                                               << " points");

    if (trajectory_msg.joint_trajectory.points.size() == 2)
    {
      // Remove previous parameterization
      for (std::size_t i = 0; i < trajectory_msg.joint_trajectory.points.size(); ++i)
      {
        trajectory_msg.joint_trajectory.points[i].velocities.clear();
        trajectory_msg.joint_trajectory.points[i].accelerations.clear();
      }

      // Add more waypoints
      robot_trajectory::RobotTrajectoryPtr robot_traj(
          new robot_trajectory::RobotTrajectory(robot_model_, arm_jmg));
      robot_traj->setRobotTrajectoryMsg(*current_state_, trajectory_msg);

      // Interpolate
      double discretization = 0.25;
      interpolate(robot_traj, discretization);

      // Convert trajectory back to a message
      robot_traj->getRobotTrajectoryMsg(trajectory_msg);

      std::cout << "BEFORE PARAM: \n" << trajectory_msg << std::endl;

      // Perform iterative parabolic smoothing
      iterative_smoother_.computeTimeStamps(*robot_traj, config_->main_velocity_scaling_factor_);

      // Convert trajectory back to a message
      robot_traj->getRobotTrajectoryMsg(trajectory_msg);
    }
  }

  // Execute trajectory
  bool wait_for_execution = false;
  if (execute_trajectory)  // TODO remove this feature and replace with the unit testing ability?
  {
    if (!execution_interface_->executeTrajectory(trajectory_msg, arm_jmg, wait_for_execution))
    {
      ROS_ERROR_STREAM_NAMED("manipulation", "Failed to execute trajectory");
      return false;
    }
  }
  else
  {
    ROS_WARN_STREAM_NAMED("manipulation",
                          "Trajectory not executed as because was requested not to");
  }

  // Do processing while trajectory is execute
  planPostProcessing();

  // Wait for trajectory execution to finish
  if (execute_trajectory && !wait_for_execution)
  {
    if (!execution_interface_->waitForExecution())
    {
      ROS_ERROR_STREAM_NAMED("manipulation", "Failed to execute trajectory");
      return false;
    }
  }

  return true;
}

bool Manipulation::createPlanningRequest(planning_interface::MotionPlanRequest& request,
                                         const moveit::core::RobotStatePtr& start,
                                         const moveit::core::RobotStatePtr& goal,
                                         JointModelGroup* arm_jmg, double velocity_scaling_factor)
{
  moveit::core::robotStateToRobotStateMsg(*start, request.start_state);

  // Create Goal constraint
  double tolerance_pose = 0.0001;
  moveit_msgs::Constraints goal_constraint = kinematic_constraints::constructGoalConstraints(
      *goal, arm_jmg, tolerance_pose, tolerance_pose);
  request.goal_constraints.push_back(goal_constraint);

  // Other settings e.g. OMPL
  request.planner_id = "RRTConnectkConfigDefault";
  // request.planner_id = "RRTstarkConfigDefault";
  request.group_name = arm_jmg->getName();
  if (config_->use_experience_setup_)
    request.num_planning_attempts =
        1;  // this must be one else it threads and doesn't use lightning/thunder correctly
  else
    request.num_planning_attempts = 3;  // this is also the number of threads to use
  request.allowed_planning_time = config_->planning_time_;  // seconds
  request.use_experience = config_->use_experience_setup_;
  request.experience_method = config_->experience_type_;
  request.max_velocity_scaling_factor = velocity_scaling_factor;

  // Parameters for the workspace that the planner should work inside relative to center of robot
  double workspace_size = 1;
  request.workspace_parameters.header.frame_id = robot_model_->getModelFrame();
  request.workspace_parameters.min_corner.x =
      start->getVariablePosition("virtual_joint/trans_x") - workspace_size;
  request.workspace_parameters.min_corner.y =
      start->getVariablePosition("virtual_joint/trans_y") - workspace_size;
  request.workspace_parameters.min_corner.z =
      0;  // floor start->getVariablePosition("virtual_joint/trans_z") - workspace_size;
  request.workspace_parameters.max_corner.x =
      start->getVariablePosition("virtual_joint/trans_x") + workspace_size;
  request.workspace_parameters.max_corner.y =
      start->getVariablePosition("virtual_joint/trans_y") + workspace_size;
  request.workspace_parameters.max_corner.z =
      start->getVariablePosition("virtual_joint/trans_z") + workspace_size;
  // visuals_->visual_tools_->publishWorkspaceParameters(request.workspace_parameters);

  return true;
}

bool Manipulation::plan(const moveit::core::RobotStatePtr& start,
                        const moveit::core::RobotStatePtr& goal, JointModelGroup* arm_jmg,
                        double velocity_scaling_factor, bool verbose,
                        moveit_msgs::RobotTrajectory& trajectory_msg)
{
  // Create motion planning request
  planning_interface::MotionPlanRequest request;
  planning_interface::MotionPlanResponse result;

  createPlanningRequest(request, start, goal, arm_jmg, config_->main_velocity_scaling_factor_);

  // Call pipeline
  std::vector<std::size_t> dummy;

  // SOLVE
  loadPlanningPipeline();  // always call before using planning_pipeline_
  planning_scene::PlanningScenePtr cloned_scene;
  {
    planning_scene_monitor::LockedPlanningSceneRO scene(
        planning_scene_monitor_);  // Lock planning scene
    cloned_scene = planning_scene::PlanningScene::clone(scene);
  }  // end scoped pointer of locked planning scene

  planning_pipeline_->generatePlan(cloned_scene, request, result, dummy, planning_context_handle_);

  // Get the trajectory
  moveit_msgs::MotionPlanResponse response;
  response.trajectory = moveit_msgs::RobotTrajectory();
  result.getMessage(response);
  trajectory_msg = response.trajectory;

  // Check that the planning was successful
  bool error = (result.error_code_.val != result.error_code_.SUCCESS);
  if (error)
  {
    ROS_ERROR_STREAM_NAMED("manipulation",
                           "Planning failed:: " << getActionResultString(
                               result.error_code_, trajectory_msg.joint_trajectory.points.empty()));
    return false;
  }

  return true;
}

bool Manipulation::planPostProcessing()
{
  // Save Experience Database
  if (config_->use_experience_setup_)
  {
    ROS_INFO_STREAM_NAMED("manipulation", "Performing planner post-processing");

    moveit_ompl::ModelBasedPlanningContextPtr mbpc =
        boost::dynamic_pointer_cast<moveit_ompl::ModelBasedPlanningContext>(
            planning_context_handle_);
    ompl::tools::ExperienceSetupPtr experience_setup =
        boost::dynamic_pointer_cast<ompl::tools::ExperienceSetup>(mbpc->getOMPLSimpleSetup());

    // Process new experience into database
    experience_setup->doPostProcessing();

    // Save database
    ROS_DEBUG_STREAM_NAMED("manipulation", "Saving experience db...");
    experience_setup->saveIfChanged();

    // Display logs
    if (visuals_->isEnabled("verbose_experience_database_stats"))
      experience_setup->printLogs();

    // Show experience database
    if (visuals_->isEnabled("show_experience_database"))
    {
      JointModelGroup* arm_jmg = config_->dual_arm_ ? config_->both_arms_ : config_->right_arm_;

      displayExperienceDatabase(arm_jmg);
    }
  }
  return true;
}

bool Manipulation::printExperienceLogs()
{
  if (!config_->use_experience_setup_)
    return true;

  if (!planning_context_handle_)
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Unable to print experience logs");
    return false;
  }
  moveit_ompl::ModelBasedPlanningContextPtr mbpc =
      boost::dynamic_pointer_cast<moveit_ompl::ModelBasedPlanningContext>(planning_context_handle_);
  ompl::tools::ExperienceSetupPtr experience_setup =
      boost::dynamic_pointer_cast<ompl::tools::ExperienceSetup>(mbpc->getOMPLSimpleSetup());

  // Display logs
  experience_setup->printLogs();
  return true;
}

bool Manipulation::interpolate(robot_trajectory::RobotTrajectoryPtr robot_traj,
                               const double& discretization)
{
  double dummy_dt = 1;  // dummy value until parameterization

  robot_trajectory::RobotTrajectoryPtr new_robot_traj(
      new robot_trajectory::RobotTrajectory(robot_model_, robot_traj->getGroup()));
  std::size_t original_num_waypoints = robot_traj->getWayPointCount();

  // Error check
  if (robot_traj->getWayPointCount() < 2)
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Unable to interpolate between less than two states");
    return false;
  }

  // Debug
  // for (std::size_t i = 0; i < robot_traj->getWayPointCount(); ++i)
  // {
  //   moveit::core::robotStateToStream(robot_traj->getWayPoint(i), std::cout, false);
  // }
  // std::cout << "-------------------------------------------------------" << std::endl;

  // For each set of points (A,B) in the original trajectory
  for (std::size_t i = 0; i < robot_traj->getWayPointCount() - 1; ++i)
  {
    // Add point A to final trajectory
    new_robot_traj->addSuffixWayPoint(robot_traj->getWayPoint(i), dummy_dt);

    for (double t = discretization; t < 1; t += discretization)
    {
      // Create new state
      moveit::core::RobotStatePtr interpolated_state(
          new moveit::core::RobotState(robot_traj->getFirstWayPoint()));
      // Fill in new values
      robot_traj->getWayPoint(i)
          .interpolate(robot_traj->getWayPoint(i + 1), t, *interpolated_state);
      // Add to trajectory
      new_robot_traj->addSuffixWayPoint(interpolated_state, dummy_dt);
      // std::cout << "inserting " << t << " at " << new_robot_traj->getWayPointCount() <<
      // std::endl;
    }
  }

  // Add final waypoint
  new_robot_traj->addSuffixWayPoint(robot_traj->getLastWayPoint(), dummy_dt);

  // Debug
  // for (std::size_t i = 0; i < new_robot_traj->getWayPointCount(); ++i)
  // {
  //   moveit::core::robotStateToStream(new_robot_traj->getWayPoint(i), std::cout, false);
  // }

  std::size_t modified_num_waypoints = new_robot_traj->getWayPointCount();
  ROS_DEBUG_STREAM_NAMED("manipulation.interpolation", "Interpolated trajectory from "
                                                           << original_num_waypoints << " to "
                                                           << modified_num_waypoints);

  // Copy back to original datastructure
  *robot_traj = *new_robot_traj;

  return true;
}

std::string Manipulation::getActionResultString(const moveit_msgs::MoveItErrorCodes& error_code,
                                                bool planned_trajectory_empty)
{
  if (error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    if (planned_trajectory_empty)
      return "Requested path and goal constraints are already met.";
    else
    {
      return "Solution was found and executed.";
    }
  }
  else if (error_code.val == moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME)
    return "Must specify group in motion plan request";
  else if (error_code.val == moveit_msgs::MoveItErrorCodes::PLANNING_FAILED ||
           error_code.val == moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN)
  {
    if (planned_trajectory_empty)
      return "No motion plan found. No execution attempted.";
    else
      return "Motion plan was found but it seems to be invalid (possibly due to postprocessing). "
             "Not executing.";
  }
  else if (error_code.val == moveit_msgs::MoveItErrorCodes::UNABLE_TO_AQUIRE_SENSOR_DATA)
    return "Motion plan was found but it seems to be too costly and looking around did not help.";
  else if (error_code.val ==
           moveit_msgs::MoveItErrorCodes::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE)
    return "Solution found but the environment changed during execution and the path was aborted";
  else if (error_code.val == moveit_msgs::MoveItErrorCodes::CONTROL_FAILED)
    return "Solution found but controller failed during execution";
  else if (error_code.val == moveit_msgs::MoveItErrorCodes::TIMED_OUT)
    return "Timeout reached";
  else if (error_code.val == moveit_msgs::MoveItErrorCodes::PREEMPTED)
    return "Preempted";
  else if (error_code.val == moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS)
    return "Invalid goal constraints";
  else if (error_code.val == moveit_msgs::MoveItErrorCodes::INVALID_OBJECT_NAME)
    return "Invalid object name";
  else if (error_code.val == moveit_msgs::MoveItErrorCodes::FAILURE)
    return "Catastrophic failure";
  return "Unknown event";
}

bool Manipulation::executeState(const moveit::core::RobotStatePtr goal_state, JointModelGroup* jmg,
                                double velocity_scaling_factor)
{
  // Get the start state
  getCurrentState();

  bool go_fast = true;  // reduce debug output

  // Visualize start/goal
  if (go_fast)
    visuals_->start_state_->publishRobotState(current_state_, rvt::GREEN);
  visuals_->goal_state_->publishRobotState(goal_state, rvt::ORANGE);

  // Check if already in new position
  if (statesEqual(*current_state_, *goal_state, jmg))
  {
    ROS_INFO_STREAM_NAMED("manipulation",
                          "Not executing because current state and goal state are close enough.");
    return true;
  }

  // Create trajectory
  std::vector<moveit::core::RobotStatePtr> robot_state_trajectory;
  robot_state_trajectory.push_back(current_state_);

  // Add goal state
  robot_state_trajectory.push_back(goal_state);

  // Get trajectory message
  moveit_msgs::RobotTrajectory trajectory_msg;

  // Convert trajectory to a message
  bool interpolate = false;
  if (!convertRobotStatesToTrajectory(robot_state_trajectory, trajectory_msg, jmg,
                                      velocity_scaling_factor, interpolate))
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Failed to convert to parameterized trajectory");
    return false;
  }

  // Execute
  const bool wait_for_execution = true;
  if (!execution_interface_->executeTrajectory(trajectory_msg, jmg, wait_for_execution))
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Failed to execute trajectory");
    return false;
  }
  return true;
}

bool Manipulation::moveDirectToState(const moveit::core::RobotStatePtr goal_state,
                                     JointModelGroup* jmg, double velocity_scaling_factor)
{
  // Visualize goal
  visuals_->goal_state_->publishRobotState(goal_state, rvt::ORANGE);

  // Get trajectory message
  moveit_msgs::RobotTrajectory trajectory_msg;

  // Assign joint names
  trajectory_msg.joint_trajectory.joint_names = jmg->getActiveJointModelNames();

  // Assign joint values
  trajectory_msg.joint_trajectory.points.resize(1);
  trajectory_msg.joint_trajectory.points[0].positions.resize(jmg->getActiveJointModels().size());
  goal_state->copyJointGroupPositions(jmg, trajectory_msg.joint_trajectory.points[0].positions);

  // Assign duration
  trajectory_msg.joint_trajectory.points[0].time_from_start = ros::Duration(0.05);

  // Debug
  // std::copy(trajectory_msg.joint_trajectory.joint_names.begin(),
  // trajectory_msg.joint_trajectory.joint_names.end(),
  // std::ostream_iterator<std::string>(std::cout, "\n"));
  // std::copy(  trajectory_msg.joint_trajectory.points[0].positions.begin(),
  // trajectory_msg.joint_trajectory.points[0].positions.end(),
  // std::ostream_iterator<double>(std::cout, "\n"));

  // Wait for previous trajectory to finish?
  execution_interface_->waitForExecution();

  // Execute
  const bool wait_for_execution = false;
  if (!execution_interface_->executeTrajectory(trajectory_msg, jmg, wait_for_execution))
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Failed to execute trajectory");
    return false;
  }
  return true;
}

bool Manipulation::executeApproachPath(moveit_grasps::GraspCandidatePtr chosen_grasp)
{
  ROS_WARN_STREAM_NAMED("temp", "deprecated");

  // Get start pose
  // getCurrentState();
  // Eigen::Affine3d pregrasp_pose =
  // current_state_->getGlobalLinkTransform(chosen_grasp->grasp_data_->parent_link_);

  // Get goal pose
  const geometry_msgs::PoseStamped& grasp_pose_msg = chosen_grasp->grasp_.grasp_pose;
  Eigen::Affine3d grasp_pose = visuals_->trajectory_lines_->convertPose(grasp_pose_msg.pose);

  // Create desired trajectory
  EigenSTL::vector_Affine3d waypoints;
  // waypoints.push_back(pregrasp_pose);
  waypoints.push_back(grasp_pose);

  // Visualize waypoints
  bool visualize_path_details = true;
  if (visualize_path_details)
  {
    bool static_id = false;
    // visuals_->trajectory_lines_->publishZArrow(pregrasp_pose, rvt::GREEN, rvt::SMALL);
    // visuals_->trajectory_lines_->publishText(pregrasp_pose, "pregrasp", rvt::WHITE, rvt::SMALL,
    // static_id);
    visuals_->trajectory_lines_->publishZArrow(grasp_pose, rvt::YELLOW, rvt::SMALL);
    visuals_->trajectory_lines_->publishText(grasp_pose, "grasp", rvt::WHITE, rvt::SMALL,
                                             static_id);
  }

  // Compute cartesian path
  moveit_grasps::GraspTrajectories segmented_cartesian_traj;
  if (!computeCartesianWaypointPath(chosen_grasp->grasp_data_->arm_jmg_, current_state_, waypoints,
                                    segmented_cartesian_traj))
  {
    ROS_WARN_STREAM_NAMED("manipulation", "Unable to plan approach path");
    return false;
  }

  // Feedback
  ROS_DEBUG_STREAM_NAMED("manipulation", "Found valid waypoint manipulation path for pregrasp");

  // Error check
  if (segmented_cartesian_traj.size() != 1)
  {
    ROS_ERROR_STREAM_NAMED("manipulation",
                           "Unexpected number of segmented states retured, should be 1");
    return false;
  }

  // Get trajectory message
  moveit_msgs::RobotTrajectory trajectory_msg;
  if (!convertRobotStatesToTrajectory(segmented_cartesian_traj.front(), trajectory_msg,
                                      chosen_grasp->grasp_data_->arm_jmg_,
                                      config_->approach_velocity_scaling_factor_))
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Failed to convert to parameterized trajectory");
    return false;
  }

  // Execute
  if (!execution_interface_->executeTrajectory(trajectory_msg, chosen_grasp->grasp_data_->arm_jmg_))
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Failed to execute trajectory");
    return false;
  }

  return true;
}

bool Manipulation::executeSavedCartesianPath(moveit_grasps::GraspCandidatePtr chosen_grasp,
                                             std::size_t segment_id)
{
  return executeSavedCartesianPath(chosen_grasp->segmented_cartesian_traj_,
                                   chosen_grasp->grasp_data_->arm_jmg_, segment_id);
}

bool Manipulation::executeSavedCartesianPath(
    const moveit_grasps::GraspTrajectories& segmented_cartesian_traj, JointModelGroup* arm_jmg,
    std::size_t segment_id)
{
  // Error check
  if (segment_id >= segmented_cartesian_traj.size())
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Requested segment id does not exist");
    return false;
  }
  if (segmented_cartesian_traj[segment_id].empty())
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Subtrajectory " << segment_id << " is empty!");
    return false;
  }

  // Add the current state to the trajectory
  std::vector<moveit::core::RobotStatePtr> trajectory = segmented_cartesian_traj[segment_id];
  trajectory.insert(trajectory.begin(), getCurrentState());

  // Get trajectory message
  moveit_msgs::RobotTrajectory trajectory_msg;
  if (!convertRobotStatesToTrajectory(trajectory, trajectory_msg, arm_jmg,
                                      config_->approach_velocity_scaling_factor_))
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Failed to convert to parameterized trajectory");
    return false;
  }

  // Execute
  if (!execution_interface_->executeTrajectory(trajectory_msg, arm_jmg))
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Failed to execute trajectory");
    return false;
  }

  return true;
}

bool Manipulation::generateApproachPath(moveit_grasps::GraspCandidatePtr chosen_grasp,
                                        moveit_msgs::RobotTrajectory& approach_trajectory_msg,
                                        const moveit::core::RobotStatePtr pre_grasp_state,
                                        const moveit::core::RobotStatePtr the_grasp_state,
                                        bool verbose)
{
  Eigen::Vector3d approach_direction = grasp_generator_->getPreGraspDirection(
      chosen_grasp->grasp_, chosen_grasp->grasp_data_->parent_link_->getName());
  bool reverse_path = true;

  double path_length;
  bool ignore_collision = false;
  std::vector<moveit::core::RobotStatePtr> robot_state_trajectory;
  getCurrentState();
  if (!computeStraightLinePath(
          approach_direction, chosen_grasp->grasp_data_->approach_distance_desired_,
          robot_state_trajectory, the_grasp_state, chosen_grasp->grasp_data_->arm_jmg_,
          reverse_path, path_length, ignore_collision))
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Error occured while computing straight line path");
    return false;
  }

  // Get approach trajectory message
  if (!convertRobotStatesToTrajectory(robot_state_trajectory, approach_trajectory_msg,
                                      chosen_grasp->grasp_data_->arm_jmg_,
                                      config_->approach_velocity_scaling_factor_))
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Failed to convert to parameterized trajectory");
    return false;
  }

  // Visualize trajectory in Rviz display
  // bool wait_for_trajetory = false;
  // visuals_->visual_tools_->publishTrajectoryPath(approach_trajectory_msg, current_state_,
  // wait_for_trajetory);

  // Set the pregrasp to be the first state in the trajectory. Copy value, not pointer
  *pre_grasp_state = *first_state_in_trajectory_;

  return true;
}

const moveit::core::JointModel* Manipulation::getGantryJoint()
{
  if (!config_->has_gantry_)
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Attempt to use gantry on robot that does not have one");
    return NULL;
  }

  const moveit::core::JointModel* gantry_joint = robot_model_->getJointModel("gantry_joint");
  if (!gantry_joint)
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Failed to get joint link");
    return NULL;
  }
  if (gantry_joint->getVariableCount() != 1)
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Invalid number of joints in "
                                               << gantry_joint->getName());
    return NULL;
  }
  return gantry_joint;
}

bool Manipulation::executeVerticlePath(JointModelGroup* arm_jmg,
                                       const double& desired_lift_distance,
                                       const double& velocity_scaling_factor, bool up,
                                       bool ignore_collision)
{
  ROS_INFO_STREAM_NAMED("manipulation", "Executing verticle path " << (up ? "up" : "down"));

  // Attempt to only use gantry, then fall back to IK
  if (config_->has_gantry_)
  {
    if (executeVerticlePathGantryOnly(arm_jmg, desired_lift_distance, velocity_scaling_factor, up,
                                      ignore_collision))
      return true;
    else
      ROS_INFO_STREAM_NAMED("manipulation", "Falling back to IK-based solution");
  }

  return executeVerticlePathWithIK(arm_jmg, desired_lift_distance, up, ignore_collision);
}

bool Manipulation::executeVerticlePathGantryOnly(JointModelGroup* arm_jmg,
                                                 const double& desired_lift_distance,
                                                 const double& velocity_scaling_factor, bool up,
                                                 bool ignore_collision, bool best_attempt)
{
  if (!config_->has_gantry_)
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Attempt to use gantry on robot that does not have one");
    return false;
  }

  // Find joint property
  const moveit::core::JointModel* gantry_joint = getGantryJoint();

  // Get latest state
  getCurrentState();

  std::vector<moveit::core::RobotStatePtr> robot_state_trajectory;
  robot_state_trajectory.push_back(current_state_);

  // Get current gantry joint
  const double* current_gantry_positions = current_state_->getJointPositions(gantry_joint);

  // Set new gantry joint
  double new_gantry_positions[1];
  if (up)
    new_gantry_positions[0] = current_gantry_positions[0] + desired_lift_distance;
  else
    new_gantry_positions[0] = current_gantry_positions[0] - desired_lift_distance;

  // Check joint limits
  if (!gantry_joint->satisfiesPositionBounds(new_gantry_positions))
  {
    ROS_INFO_STREAM_NAMED("manipulation", "New gantry position of "
                                              << new_gantry_positions[0]
                                              << " does not satisfy joint limit bounds.");

    if (!best_attempt)
      return false;
  }

  // Create new movemenet state
  moveit::core::RobotStatePtr new_state(new moveit::core::RobotState(*current_state_));
  new_state->setJointPositions(gantry_joint, new_gantry_positions);
  robot_state_trajectory.push_back(new_state);

  // Get approach trajectory message
  moveit_msgs::RobotTrajectory cartesian_trajectory_msg;
  if (!convertRobotStatesToTrajectory(robot_state_trajectory, cartesian_trajectory_msg, arm_jmg,
                                      velocity_scaling_factor))

  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Failed to convert to parameterized trajectory");
    return false;
  }

  // Execute
  if (!execution_interface_->executeTrajectory(cartesian_trajectory_msg, arm_jmg))
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Failed to execute trajectory");
    return false;
  }

  return true;
}

bool Manipulation::executeVerticlePathWithIK(JointModelGroup* arm_jmg,
                                             const double& desired_lift_distance, bool up,
                                             bool ignore_collision)
{
  Eigen::Vector3d approach_direction;
  approach_direction << 0, 0, (up ? 1 : -1);  // 1 is up, -1 is down
  bool reverse_path = false;

  if (!executeCartesianPath(arm_jmg, approach_direction, desired_lift_distance,
                            config_->lift_velocity_scaling_factor_, reverse_path, ignore_collision))
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Failed to execute horizontal path");
    return false;
  }
  return true;
}

bool Manipulation::executeHorizontalPath(JointModelGroup* arm_jmg,
                                         const double& desired_lift_distance, bool left,
                                         bool ignore_collision)
{
  Eigen::Vector3d approach_direction;
  approach_direction << 0, (left ? 1 : -1), 0;  // 1 is left, -1 is right
  bool reverse_path = false;

  if (!executeCartesianPath(arm_jmg, approach_direction, desired_lift_distance,
                            config_->lift_velocity_scaling_factor_, reverse_path, ignore_collision))
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Failed to execute horizontal path");
    return false;
  }
  return true;
}

bool Manipulation::executeRetreatPath(JointModelGroup* arm_jmg, double desired_retreat_distance,
                                      bool retreat, bool ignore_collision)
{
  // Compute straight line in reverse from grasp
  Eigen::Vector3d approach_direction;
  approach_direction << (retreat ? -1 : 1), 0, 0;  // backwards towards robot body
  bool reverse_path = false;

  if (!executeCartesianPath(arm_jmg, approach_direction, desired_retreat_distance,
                            config_->retreat_velocity_scaling_factor_, reverse_path,
                            ignore_collision))
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Failed to execute retreat path");
    return false;
  }
  return true;
}

bool Manipulation::executeInsertionClosedLoop(JointModelGroup* arm_jmg, double desired_distance,
                                              Eigen::Affine3d& desired_world_to_tool,
                                              bool direction_in, bool& achieved_depth)
{
  // Copy pose
  teleop_world_to_tool_ = desired_world_to_tool;

  // Move in and out
  Eigen::Vector3d approach_direction;
  approach_direction << 0, 0, (direction_in ? 1 : -1);

  // the direction can be in the local reference frame (in which case we rotate it)
  const Eigen::Vector3d rotated_direction = teleop_world_to_tool_.rotation() * approach_direction;

  // Reusable transform from robot base to world. Could be identity. Assumes that it does not change
  // This should be a constant
  Eigen::Affine3d base_to_world = getCurrentState()->getGlobalLinkTransform("base_link").inverse();

  // The target pose is built by applying a translation to the start pose for the desired direction
  // and distance
  // Eigen::Affine3d teleop_world_to_tool__base_frame;
  std::size_t num_steps = desired_distance * config_->insertion_steps_per_meter_;
  // don't allow waypoints to be reached before next goal sent
  static const double SMOOTH_FACTOR = 1.1;

  // Pre-calculate values
  double step_distance = desired_distance / double(num_steps);
  double step_duration = config_->insertion_duration_ / double(num_steps);

  std::cout << "step_distance: " << step_distance << std::endl;
  std::cout << "step_duration: " << step_duration << std::endl;

  achieved_depth = true;  // assume it works

  ros::Rate rate_limiter(1 / step_duration);  // specify the hz
  // Begin soft-real time loop
  for (std::size_t i = 0; i < num_steps; ++i)
  {
    // Check if program needs to end
    if (!ros::ok())
      break;

    if (direction_in)
      std::cout << "insertion,  step " << i << std::endl;
    else
      std::cout << "retracting, step " << i << std::endl;

    // Move target pose inward
    teleop_world_to_tool_.translation() += rotated_direction * step_distance;

    // Track if tactile sensor was used
    bool corrected = false;

    // Adjust pose based on tactile feedback
    corrected = adjustPoseFromTactile();

    // Decide when to stop inserting based on negative z-axis sheer force
    // if (direction_in && adjustPoseAndReject())
    // {
    //   std::cout << "Breaking insertion loop because sheer force too high" << std::endl;
    //   achieved_depth = false;
    //   remote_control_->waitForNextStep("reverse direction");
    //   break;
    // }

    // Visualize the pivot point for pose rotations
    visuals_->visual_tools_->publishSphere(
        visuals_->visual_tools_->convertPose(teleop_world_to_ee_), rvt::PURPLE,
        visuals_->visual_tools_->getScale(rvt::REGULAR, false, 0.1), "Sphere", 1);

    // Move pose from tips of finger (tool) back to base of EE
    teleop_world_to_ee_ = teleop_world_to_tool_ * config_->teleoperation_offset_;

    // Show pose-rotation pose
    visuals_->trajectory_lines_->publishZArrow(teleop_world_to_ee_, rvt::ORANGE, rvt::REGULAR);

    // Convert desired pose from 'world' frame to 'robot base' frame
    teleop_base_to_ee_ = base_to_world * teleop_world_to_ee_;

    if (corrected)
      remote_control_->waitForNextStep("move");

    // Move robot
    execution_interface_->executePose(teleop_base_to_ee_, arm_jmg, step_duration * SMOOTH_FACTOR);

    if (corrected)
    {
      std::cout << "CORRECTED" << std::endl;
      ros::Duration(config_->insertion_alter_pause_).sleep();
      // tactile_feedback_->recalibrateTactileSensor();
    }
    else
    {
      // Wait for next loop
      rate_limiter.sleep();
    }
  }

  // Copy pose back
  desired_world_to_tool = teleop_world_to_tool_;

  return true;
}

bool Manipulation::executeInsertionOpenLoop(JointModelGroup* arm_jmg, double desired_distance,
                                            bool in, double velocity_scaling_factor)
{
  const moveit::core::LinkModel* ik_tip_link = grasp_datas_[arm_jmg]->parent_link_;

  moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(*getCurrentState()));

  // Get current pose
  Eigen::Affine3d ee_start_pose =
      robot_state->getGlobalLinkTransform(grasp_datas_[arm_jmg]->parent_link_);

  // Visualize current pose
  visuals_->trajectory_lines_->publishZArrow(ee_start_pose, rvt::ORANGE);

  // Move in and out
  Eigen::Vector3d approach_direction;
  approach_direction << 0, 0, (in ? 1 : -1);

  // Compute Cartesian Path
  // the direction can be in the local reference frame (in which case we rotate it)
  const Eigen::Vector3d rotated_direction = ee_start_pose.rotation() * approach_direction;

  // The target pose is built by applying a translation to the start pose for the desired direction
  // and distance
  Eigen::Affine3d target_pose = ee_start_pose;
  target_pose.translation() += rotated_direction * desired_distance;
  visuals_->trajectory_lines_->publishZArrow(target_pose, rvt::GREEN);

  // Resolution of trajectory
  double max_step = 0.01;  // 0.01 // The maximum distance in Cartesian space between consecutive
                           // points on the resulting path

  // Jump threshold for preventing consequtive joint values from 'jumping' by a large amount in
  // joint space
  double jump_threshold = config_->jump_threshold_;  // aka jump factor

  std::vector<moveit::core::RobotStatePtr> robot_state_trajectory;
  double last_valid_percentage =
      robot_state->computeCartesianPath(arm_jmg, robot_state_trajectory, ik_tip_link, target_pose,
                                        true, max_step, jump_threshold, NULL);
  std::cout << "last_valid_percentage: " << last_valid_percentage << std::endl;

  visuals_->trajectory_lines_->publishTrajectoryPoints(robot_state_trajectory, ik_tip_link);

  // Get approach trajectory message
  moveit_msgs::RobotTrajectory cartesian_trajectory_msg;
  if (!convertRobotStatesToTrajectory(robot_state_trajectory, cartesian_trajectory_msg, arm_jmg,
                                      velocity_scaling_factor))
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Failed to convert to parameterized trajectory");
    return false;
  }

  // Execute
  if (!execution_interface_->executeTrajectory(cartesian_trajectory_msg, arm_jmg))
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Failed to execute trajectory");
    return false;
  }

  return true;
}

bool Manipulation::adjustPoseAndReject()
{
  // Check if overall sheer force is enough to move the arm
  if (tactile_feedback_->getSheerForce() > config_->sheer_force_rejection_max_)
  {
    std::cout << "sheer force reached max, actual: " << tactile_feedback_->getSheerForce()
              << ", max:" << config_->sheer_force_rejection_max_
              << " --------------------------------\n";
    return true;
  }

  return false;  // do not reject
}

bool Manipulation::executeCartesianPath(JointModelGroup* arm_jmg, const Eigen::Vector3d& direction,
                                        double desired_distance, double velocity_scaling_factor,
                                        bool reverse_path, bool ignore_collision)
{
  getCurrentState();

  // Debug
  // visuals_->visual_tools_->publishRobotState( current_state_, rvt::PURPLE );
  // visuals_->start_state_->hideRobot();
  // visuals_->goal_state_->hideRobot();

  double path_length;
  std::vector<moveit::core::RobotStatePtr> robot_state_trajectory;
  if (!computeStraightLinePath(direction, desired_distance, robot_state_trajectory, current_state_,
                               arm_jmg, reverse_path, path_length, ignore_collision))

  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Error occured while computing straight line path");
    return false;
  }

  // Get approach trajectory message
  moveit_msgs::RobotTrajectory cartesian_trajectory_msg;
  if (!convertRobotStatesToTrajectory(robot_state_trajectory, cartesian_trajectory_msg, arm_jmg,
                                      velocity_scaling_factor))
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Failed to convert to parameterized trajectory");
    return false;
  }

  // Execute
  if (!execution_interface_->executeTrajectory(cartesian_trajectory_msg, arm_jmg))
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Failed to execute trajectory");
    return false;
  }

  return true;
}

bool Manipulation::computeStraightLinePath(
    Eigen::Vector3d direction, double desired_distance,
    std::vector<moveit::core::RobotStatePtr>& robot_state_trajectory,
    moveit::core::RobotStatePtr robot_state, JointModelGroup* arm_jmg, bool reverse_trajectory,
    double& last_valid_percentage, bool ignore_collision)
{
  // End effector parent link (arm tip for ik solving)
  const moveit::core::LinkModel* ik_tip_link = grasp_datas_[arm_jmg]->parent_link_;

  // ---------------------------------------------------------------------------------------------
  // Show desired trajectory in BLACK
  Eigen::Affine3d tip_pose_start = robot_state->getGlobalLinkTransform(ik_tip_link);

  // Debug
  if (false)
  {
    std::cout << "Tip Pose Start \n" << tip_pose_start.translation().x() << "\t"
              << tip_pose_start.translation().y() << "\t" << tip_pose_start.translation().z()
              << std::endl;
  }

  // Visualize start and goal state
  if (verbose_)
  {
    visuals_->visual_tools_->publishSphere(tip_pose_start, rvt::RED, rvt::LARGE);

    // Get desired end pose
    Eigen::Affine3d tip_pose_end;
    straightProjectPose(tip_pose_start, tip_pose_end, direction, desired_distance);

    visuals_->visual_tools_->publishLine(tip_pose_start, tip_pose_end, rvt::BLACK, rvt::REGULAR);

    // Show start and goal states of cartesian path
    if (reverse_trajectory)
    {
      // The passed in robot state is the goal
      visuals_->start_state_->hideRobot();
      visuals_->goal_state_->publishRobotState(robot_state, rvt::ORANGE);
    }
    else
    {
      // The passed in robot state is the start (retreat)
      visuals_->goal_state_->hideRobot();
      visuals_->start_state_->publishRobotState(robot_state, rvt::GREEN);
    }
  }

  // ---------------------------------------------------------------------------------------------
  // Settings for computeCartesianPath

  // Resolution of trajectory
  double max_step = 0.01;  // 0.01 // The maximum distance in Cartesian space between consecutive
                           // points on the resulting path

  // Error check
  if (desired_distance < max_step)
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Not enough: desired_distance ("
                                               << desired_distance << ")  < max_step (" << max_step
                                               << ")");
    return false;
  }

  // Jump threshold for preventing consequtive joint values from 'jumping' by a large amount in
  // joint space
  double jump_threshold = config_->jump_threshold_;  // aka jump factor

  bool collision_checking_verbose = false;

  // Reference frame setting
  bool global_reference_frame = true;

  // Check for kinematic solver
  if (!arm_jmg->canSetStateFromIK(ik_tip_link->getName()))
    ROS_ERROR_STREAM_NAMED("manipulation", "No IK Solver loaded - make sure "
                                           "moveit_config/kinamatics.yaml is loaded in this "
                                           "namespace");

  std::size_t attempts = 0;
  static const std::size_t MAX_IK_ATTEMPTS = 4;
  while (attempts < MAX_IK_ATTEMPTS)
  {
    if (attempts > 0)
    {
      std::cout << std::endl;
      ROS_INFO_STREAM_NAMED("manipulation", "Attempting IK solution, attempts # " << attempts);
    }
    if (attempts > MAX_IK_ATTEMPTS - 2)
    {
      ROS_WARN_STREAM_NAMED("manipulation", "Enabling collision check verbose");
      collision_checking_verbose = true;
    }
    attempts++;

    bool only_check_self_collision = false;
    if (ignore_collision)
    {
      only_check_self_collision = true;
      ROS_INFO_STREAM_NAMED("manipulation", "computeStraightLinePath() is ignoring collisions with "
                                            "world objects (but not robot links)");
    }

    // Collision check
    boost::scoped_ptr<planning_scene_monitor::LockedPlanningSceneRO> ls;
    ls.reset(new planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor_));
    moveit::core::GroupStateValidityCallbackFn constraint_fn = boost::bind(
        &isStateValid, static_cast<const planning_scene::PlanningSceneConstPtr&>(*ls).get(),
        collision_checking_verbose, only_check_self_collision, visuals_, _1, _2, _3);

    // Compute Cartesian Path
    // this is the Cartesian pose we start from, and have to move in the direction indicated
    const Eigen::Affine3d& start_pose = robot_state->getGlobalLinkTransform(ik_tip_link);

    // the direction can be in the local reference frame (in which case we rotate it)
    const Eigen::Vector3d rotated_direction =
        global_reference_frame ? direction : start_pose.rotation() * direction;

    // The target pose is built by applying a translation to the start pose for the desired
    // direction and distance
    Eigen::Affine3d target_pose = start_pose;
    target_pose.translation() += rotated_direction * desired_distance;

    robot_state_trajectory.clear();
    last_valid_percentage =
        robot_state->computeCartesianPath(arm_jmg, robot_state_trajectory, ik_tip_link, target_pose,
                                          true, max_step, jump_threshold, constraint_fn);

    ROS_DEBUG_STREAM_NAMED("manipulation", "Cartesian last_valid_percentage: "
                                               << last_valid_percentage
                                               << ", number of states in trajectory: "
                                               << robot_state_trajectory.size());

    double min_allowed_valid_percentage = 0.9;
    if (last_valid_percentage == 0)
    {
      ROS_WARN_STREAM_NAMED("manipulation",
                            "Failed to computer cartesian path: last_valid_percentage is 0");
    }
    else if (last_valid_percentage < min_allowed_valid_percentage)
    {
      ROS_DEBUG_STREAM_NAMED("manipulation.waypoints",
                             "Resulting cartesian path is less than "
                                 << min_allowed_valid_percentage
                                 << " % of the desired distance, % valid: "
                                 << last_valid_percentage);
    }
    else
    {
      ROS_INFO_STREAM_NAMED("manipulation", "Found valid cartesian path");
      break;
    }
  }  // end while AND scoped pointer of locked planning scene

  // Check if we never found a path
  if (attempts >= MAX_IK_ATTEMPTS)
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Never found a valid cartesian path, aborting");
    return false;
  }

  // Reverse the trajectory if neeeded
  if (reverse_trajectory)
  {
    std::reverse(robot_state_trajectory.begin(), robot_state_trajectory.end());

    // Also, save the first state so that generateApproachPath() can use it
    first_state_in_trajectory_ = robot_state_trajectory.front();
  }

  // Debug
  if (verbose_)
  {
    // Super debug
    if (false)
    {
      std::cout << "Tip Pose Result: \n";
      for (std::size_t i = 0; i < robot_state_trajectory.size(); ++i)
      {
        const Eigen::Affine3d tip_pose_start =
            robot_state_trajectory[i]->getGlobalLinkTransform(ik_tip_link);
        std::cout << tip_pose_start.translation().x() << "\t" << tip_pose_start.translation().y()
                  << "\t" << tip_pose_start.translation().z() << std::endl;
      }
    }

    // Show actual trajectory in GREEN
    ROS_INFO_STREAM_NAMED("manipulation", "Displaying cartesian trajectory in green");
    const Eigen::Affine3d& tip_pose_end =
        robot_state_trajectory.back()->getGlobalLinkTransform(ik_tip_link);
    visuals_->visual_tools_->publishLine(tip_pose_start, tip_pose_end, rvt::LIME_GREEN, rvt::LARGE);
    visuals_->visual_tools_->publishSphere(tip_pose_end, rvt::ORANGE, rvt::LARGE);

    // Visualize end effector position of cartesian path
    ROS_INFO_STREAM_NAMED("manipulation", "Visualize end effector position of cartesian path");
    visuals_->visual_tools_->publishTrajectoryPoints(robot_state_trajectory, ik_tip_link);

    // Show start and goal states of cartesian path
    if (reverse_trajectory)
    {
      // The passed in robot state is the goal
      visuals_->start_state_->publishRobotState(robot_state_trajectory.front(), rvt::GREEN);
    }
    else
    {
      // The passed in robot state is the start (retreat)
      visuals_->goal_state_->publishRobotState(robot_state_trajectory.back(), rvt::ORANGE);
    }
  }

  return true;
}

JointModelGroup* Manipulation::chooseArm(const Eigen::Affine3d& ee_pose)
{
  // Single Arm
  if (!config_->dual_arm_)
  {
    return config_->right_arm_;  // right is always the default arm for single arm robots
  }
  // Dual Arm
  else if (ee_pose.translation().y() < 0)
  {
    ROS_DEBUG_STREAM_NAMED("manipulation", "Using right arm for task");
    return config_->right_arm_;
  }
  else
  {
    ROS_DEBUG_STREAM_NAMED("manipulation", "Using left arm for task");
    return config_->left_arm_;
  }
}

bool Manipulation::getRobotStateFromPose(const Eigen::Affine3d& ee_pose,
                                         moveit::core::RobotStatePtr& robot_state,
                                         JointModelGroup* arm_jmg, bool use_consistency_limits)
{
  // Setup collision checking with a locked planning scene
  {
    bool collision_checking_verbose = false;
    if (collision_checking_verbose)
      ROS_WARN_STREAM_NAMED("manipulation",
                            "moveToEEPose() has collision_checking_verbose turned on");
    boost::scoped_ptr<planning_scene_monitor::LockedPlanningSceneRO> ls;
    ls.reset(new planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor_));
    bool only_check_self_collision = true;
    moveit::core::GroupStateValidityCallbackFn constraint_fn = boost::bind(
        &isStateValid, static_cast<const planning_scene::PlanningSceneConstPtr&>(*ls).get(),
        collision_checking_verbose, only_check_self_collision, visuals_, _1, _2, _3);

    // Solve IK problem for arm
    std::size_t attempts = 0;  // use default
    double timeout = 0;        // use default

    // Create consistency limits TODO cache
    std::vector<double> consistency_limits;
    if (use_consistency_limits)
      for (std::size_t i = 0; i < arm_jmg->getActiveJointModels().size();
           ++i)  // TODO hard coded njoints
        consistency_limits.push_back(0.5);

    const moveit::core::LinkModel* ik_tip_link = grasp_datas_[arm_jmg]->parent_link_;
    if (!robot_state->setFromIK(arm_jmg, ee_pose, ik_tip_link->getName(), consistency_limits,
                                attempts, timeout, constraint_fn))
    {
      visuals_->visual_tools_->publishZArrow(ee_pose, rvt::RED);
      ROS_WARN_STREAM_NAMED("manipulation", "Unable to find arm solution for desired pose");
      return false;
    }
  }  // end scoped pointer of locked planning scene

  // ROS_DEBUG_STREAM_NAMED("manipulation","Found solution to pose request");
  return true;
}

bool Manipulation::straightProjectPose(const Eigen::Affine3d& original_pose,
                                       Eigen::Affine3d& new_pose, const Eigen::Vector3d direction,
                                       double distance)
{
  ROS_DEBUG_STREAM_NAMED("manipulation.superdebug", "straightProjectPose()");

  // Assume everything is in world coordinates
  new_pose = original_pose;
  Eigen::Vector3d longer_direction = direction * distance;
  new_pose.translation() += longer_direction;

  return true;
}

bool Manipulation::convertRobotStatesToTrajectory(
    const std::vector<moveit::core::RobotStatePtr>& robot_state_traj,
    moveit_msgs::RobotTrajectory& trajectory_msg, JointModelGroup* jmg,
    const double& velocity_scaling_factor, bool use_interpolation)
{
  ROS_DEBUG_STREAM_NAMED("manipulation.superdebug", "convertRobotStatesToTrajectory()");

  // Copy the vector of RobotStates to a RobotTrajectory
  robot_trajectory::RobotTrajectoryPtr robot_traj(
      new robot_trajectory::RobotTrajectory(robot_model_, jmg));

  // -----------------------------------------------------------------------------------------------
  // Convert to RobotTrajectory datatype
  for (std::size_t k = 0; k < robot_state_traj.size(); ++k)
  {
    double duration_from_previous = 1;  // this is overwritten and unimportant
    robot_traj->addSuffixWayPoint(robot_state_traj[k], duration_from_previous);
  }

  // Interpolate any path with two few points
  if (use_interpolation)
  {
    static const std::size_t MIN_TRAJECTORY_POINTS = 20;
    if (robot_traj->getWayPointCount() < MIN_TRAJECTORY_POINTS)
    {
      ROS_INFO_STREAM_NAMED("manipulation", "Interpolating trajectory because two few points ("
                                                << robot_traj->getWayPointCount() << ")");

      // Interpolate between each point
      double discretization = 0.25;
      interpolate(robot_traj, discretization);
    }
  }

  // Perform iterative parabolic smoothing
  iterative_smoother_.computeTimeStamps(*robot_traj, velocity_scaling_factor);

  // Convert trajectory to a message
  robot_traj->getRobotTrajectoryMsg(trajectory_msg);

  return true;
}

bool Manipulation::openEEs(bool open)
{
  ROS_DEBUG_STREAM_NAMED("manipulation.superdebug", "openEEs()");

  // First arm
  if (!openEE(open, config_->right_arm_))
    return false;

  // Second arm if applicable
  if (config_->dual_arm_)
    if (!openEE(open, config_->left_arm_))
      return false;

  return true;
}

bool Manipulation::openEE(bool open, JointModelGroup* arm_jmg)
{
  if (open)
  {
    return setEEGraspPosture(grasp_datas_[arm_jmg]->pre_grasp_posture_, arm_jmg);
  }
  else
  {
    return setEEGraspPosture(grasp_datas_[arm_jmg]->grasp_posture_, arm_jmg);
  }
}

bool Manipulation::setEEJointPosition(double joint_position, JointModelGroup* arm_jmg)
{
  std::vector<double> joint_positions;
  for (std::size_t i = 0; i < grasp_datas_[arm_jmg]->ee_jmg_->getVariableCount(); ++i)
  {
    joint_positions.push_back(joint_position);
    // TODO ignore tip joints
  }

  trajectory_msgs::JointTrajectory grasp_posture;
  grasp_datas_[arm_jmg]->jointPositionsToGraspPosture(joint_positions, grasp_posture);

  return setEEGraspPosture(grasp_posture, arm_jmg);
}

bool Manipulation::setEEGraspPosture(trajectory_msgs::JointTrajectory grasp_posture,
                                     JointModelGroup* arm_jmg)
{
  ROS_DEBUG_STREAM_NAMED("manipulation.end_effector", "Moving to grasp posture:\n"
                                                          << grasp_posture);

  // Check status
  if (!config_->isEnabled("end_effector_enabled"))
  {
    ROS_WARN_STREAM_NAMED("manipulation", "Gripping is disabled");
    return true;
  }

  getCurrentState();
  robot_trajectory::RobotTrajectoryPtr ee_trajectory(
      new robot_trajectory::RobotTrajectory(robot_model_, grasp_datas_[arm_jmg]->ee_jmg_));

  ROS_INFO_STREAM_NAMED("manipulation", "Sending command to end effector "
                                            << grasp_datas_[arm_jmg]->ee_jmg_->getName());

  // Add goal state
  ee_trajectory->setRobotTrajectoryMsg(*current_state_, grasp_posture);

  // Add start state to trajectory
  double dummy_dt = 1;
  ee_trajectory->addPrefixWayPoint(current_state_, dummy_dt);

  // Check if already in new position
  if (statesEqual(ee_trajectory->getFirstWayPoint(), ee_trajectory->getLastWayPoint(),
                  grasp_datas_[arm_jmg]->ee_jmg_))
  {
    ROS_INFO_STREAM_NAMED(
        "manipulation",
        "Not executing motion because current state and goal state are close enough for group "
            << grasp_datas_[arm_jmg]->ee_jmg_->getName());

    return true;
  }

  // Interpolate between each point
  double discretization = 0.1;
  interpolate(ee_trajectory, discretization);

  // Perform iterative parabolic smoothing
  double ee_velocity_scaling_factor = 0.1;
  iterative_smoother_.computeTimeStamps(*ee_trajectory, ee_velocity_scaling_factor);

  // Show the change in end effector
  if (verbose_)
  {
    visuals_->start_state_->publishRobotState(current_state_, rvt::GREEN);
    visuals_->goal_state_->publishRobotState(ee_trajectory->getLastWayPoint(), rvt::ORANGE);
  }

  // Convert trajectory to a message
  moveit_msgs::RobotTrajectory trajectory_msg;
  ee_trajectory->getRobotTrajectoryMsg(trajectory_msg);

  // Execute trajectory
  if (!execution_interface_->executeTrajectory(trajectory_msg, arm_jmg))
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Failed to execute grasp trajectory");
    return false;
  }

  return true;
}

// bool Manipulation::setStateWithOpenEE(bool open, moveit::core::RobotStatePtr robot_state)
// {
//   ROS_DEBUG_STREAM_NAMED("manipulation.superdebug","setStateWithOpenEE()");

//   if (open)
//   {
//     grasp_datas_[config_->right_arm_]->setRobotStatePreGrasp( robot_state );
//     if (config_->dual_arm_)
//       grasp_datas_[config_->left_arm_]->setRobotStatePreGrasp( robot_state );
//   }
//   else
//   {
//     grasp_datas_[config_->right_arm_]->setRobotStateGrasp( robot_state );
//     if (config_->dual_arm_)
//       grasp_datas_[config_->left_arm_]->setRobotStateGrasp( robot_state );
//   }
//   return true;
// }

ExecutionInterfacePtr Manipulation::getExecutionInterface() { return execution_interface_; }
bool Manipulation::fixCollidingState(planning_scene::PlanningScenePtr cloned_scene)
{
  ROS_DEBUG_STREAM_NAMED("manipulation.superdebug", "fixCollidingState()");

  // Turn off auto mode
  // remote_control_->setFullAutonomous(false);

  // Open hand to ensure we aren't holding anything anymore
  if (!openEEs(true))
  {
    ROS_WARN_STREAM_NAMED("manipulation", "Unable to open end effectors");
    // return false;
  }

  JointModelGroup* arm_jmg = config_->dual_arm_ ? config_->left_arm_ : config_->right_arm_;

  // Decide what direction is needed to fix colliding state, using the cloned scene
  collision_detection::CollisionResult::ContactMap contacts;
  cloned_scene->getCollidingPairs(contacts);

  std::string colliding_world_object = "";
  for (collision_detection::CollisionResult::ContactMap::const_iterator contact_it =
           contacts.begin();
       contact_it != contacts.end(); contact_it++)
  {
    // const std::string& body_id_1 = contact_it->first.first;
    // const std::string& body_id_2 = contact_it->first.second;
    // std::cout << "body_id_1: " << body_id_1 << std::endl;
    // std::cout << "body_id_2: " << body_id_2 << std::endl;

    const std::vector<collision_detection::Contact>& contacts = contact_it->second;

    for (std::size_t i = 0; i < contacts.size(); ++i)
    {
      const collision_detection::Contact& contact = contacts[i];

      // Find the world object that is the problem
      if (contact.body_type_1 == collision_detection::BodyTypes::WORLD_OBJECT)
      {
        colliding_world_object = contact.body_name_1;
        break;
      }
      if (contact.body_type_2 == collision_detection::BodyTypes::WORLD_OBJECT)
      {
        colliding_world_object = contact.body_name_2;
        break;
      }
    }
    if (!colliding_world_object.empty())
      break;
  }

  if (colliding_world_object.empty())
  {
    ROS_WARN_STREAM_NAMED("manipulation",
                          "Did not find any world objects in collision. Attempting to move home");
    bool check_validity = false;
    return moveToStartPosition(NULL, check_validity);
  }

  ROS_INFO_STREAM_NAMED("manipulation", "World object " << colliding_world_object
                                                        << " in collision");

  // Categorize this world object
  bool reverse_out = false;
  bool raise_up = false;
  bool move_in_right = false;
  bool move_in_left = false;
  std::cout << "substring is: " << colliding_world_object.substr(0, 7) << std::endl;

  // if shelf or product, reverse out
  if (colliding_world_object.substr(0, 7) == "product")
  {
    reverse_out = true;
  }
  else if (colliding_world_object.substr(0, 7) == "front_w")  // front_wall
  {
    reverse_out = true;
  }
  else if (colliding_world_object.substr(0, 7) == "shelf")  // TODO string name
  {
    reverse_out = true;
  }
  // if goal bin, raise up
  else if (colliding_world_object.substr(0, 7) == "goal_bin")  // TODO string name
  {
    raise_up = true;
  }
  // Right wall
  else if (colliding_world_object.substr(0, 7) == "right_w")
  {
    move_in_left = true;
  }
  // Left wall
  else if (colliding_world_object.substr(0, 7) == "left_w")
  {
    move_in_right = true;
  }
  else
  {
    int mode = iRand(0, 3);
    ROS_WARN_STREAM_NAMED(
        "manipulation",
        "Unknown object, not sure how to handle. Performing random action using mode " << mode);

    if (mode == 0)
      reverse_out = true;
    else if (mode == 1)
      raise_up = true;
    else if (mode == 2)
      move_in_left = true;
    else  // mode = 3
      move_in_right = true;
  }

  if (raise_up)
  {
    ROS_INFO_STREAM_NAMED("manipulation", "Raising up");
    double desired_distance = 0.2;
    bool up = true;
    bool ignore_collision = true;
    if (!executeVerticlePath(arm_jmg, desired_distance, config_->lift_velocity_scaling_factor_, up,
                             ignore_collision))
    {
      return false;
    }
  }
  else if (reverse_out)
  {
    ROS_INFO_STREAM_NAMED("manipulation", "Reversing out");
    double desired_distance = 0.1;
    bool restreat = true;
    bool ignore_collision = true;
    if (!executeRetreatPath(arm_jmg, desired_distance, restreat, ignore_collision))
    {
      return false;
    }
  }
  else if (move_in_left)
  {
    ROS_INFO_STREAM_NAMED("manipulation", "Moving in left");
    double desired_distance = 0.2;
    bool left = true;
    bool ignore_collision = true;
    if (!executeHorizontalPath(arm_jmg, desired_distance, left, ignore_collision))
    {
      return false;
    }
  }
  else if (move_in_right)
  {
    ROS_INFO_STREAM_NAMED("manipulation", "Moving in right");
    double desired_distance = 0.2;
    bool left = false;
    bool ignore_collision = true;
    if (!executeHorizontalPath(arm_jmg, desired_distance, left, ignore_collision))
    {
      return false;
    }
  }

  return true;
}

bool Manipulation::moveToStartPosition(JointModelGroup* arm_jmg, bool check_validity)
{
  // Choose which planning group to use
  if (arm_jmg == NULL)
    arm_jmg = config_->dual_arm_ ? config_->both_arms_ : config_->right_arm_;
  return moveToSRDFPose(arm_jmg, config_->start_pose_, config_->main_velocity_scaling_factor_,
                        check_validity);
}

void Manipulation::loadPlanningPipeline()
{
  ROS_DEBUG_STREAM_NAMED("manipulation.superdebug", "loadPlanningPipeline()");

  if (!planning_pipeline_)
  {
    // Setup planning pipeline
    planning_pipeline_.reset(new planning_pipeline::PlanningPipeline(
        robot_model_, nh_, "planning_plugin", "request_adapters"));
  }
}

bool Manipulation::statesEqual(const moveit::core::RobotState& s1,
                               const moveit::core::RobotState& s2, JointModelGroup* jmg)
{
  static const double STATES_EQUAL_THRESHOLD = 0.01;

  double s1_vars[jmg->getVariableCount()];
  double s2_vars[jmg->getVariableCount()];
  s1.copyJointGroupPositions(jmg, s1_vars);
  s2.copyJointGroupPositions(jmg, s2_vars);

  for (std::size_t i = 0; i < jmg->getVariableCount(); ++i)
  {
    const moveit::core::JointModel* this_joint = jmg->getJointModel(jmg->getVariableNames()[i]);

    std::vector<const moveit::core::JointModel*>::const_iterator joint_it;
    joint_it = std::find(jmg->getActiveJointModels().begin(), jmg->getActiveJointModels().end(),
                         this_joint);

    // Make sure joint is active
    if (joint_it != jmg->getActiveJointModels().end())
    {
      if (fabs(s1_vars[i] - s2_vars[i]) > STATES_EQUAL_THRESHOLD)
      {
        // std::cout << "    statesEqual: Variable " << jmg->getVariableNames()[i] << " beyond
        // threshold, diff: "
        // << fabs(s1_vars[i] - s2_vars[i]) << std::endl;
        return false;
      }
    }
  }

  return true;
}

ompl::tools::ExperienceSetupPtr Manipulation::getExperienceSetup(JointModelGroup* arm_jmg)
{
  // Get manager
  loadPlanningPipeline();  // always call before using planning_pipeline_
  const planning_interface::PlannerManagerPtr planner_manager =
      planning_pipeline_->getPlannerManager();

  // Create dummy request
  planning_interface::MotionPlanRequest request;
  createPlanningRequest(request, current_state_, current_state_, arm_jmg,
                        config_->main_velocity_scaling_factor_);

  // Get context
  moveit_msgs::MoveItErrorCodes error_code;
  planning_context_handle_ = planner_manager->getPlanningContext(
      planning_scene_monitor_->getPlanningScene(), request, error_code);

  // Convert to model based planning context
  moveit_ompl::ModelBasedPlanningContextPtr mbpc =
      boost::dynamic_pointer_cast<moveit_ompl::ModelBasedPlanningContext>(planning_context_handle_);

  // Get experience setup
  ompl::tools::ExperienceSetupPtr experience_setup =
      boost::dynamic_pointer_cast<ompl::tools::ExperienceSetup>(mbpc->getOMPLSimpleSetup());

  return experience_setup;
}

bool Manipulation::displayExperienceDatabase(JointModelGroup* arm_jmg)
{
  ompl::tools::ExperienceSetupPtr experience_setup = getExperienceSetup(arm_jmg);

  // Create a state space describing our robot's planning group
  moveit_ompl::ModelBasedStateSpacePtr model_state_space =
      boost::dynamic_pointer_cast<moveit_ompl::ModelBasedStateSpace>(
          experience_setup->getStateSpace());

  // Get all of the graphs in the database
  std::vector<ompl::base::PlannerDataPtr> graphs;
  experience_setup->getAllPlannerDatas(graphs);

  ROS_INFO_STREAM_NAMED("manipulation", "Number of paths/graphs to publish: " << graphs.size());

  // Error check
  if (!graphs.size())
  {
    ROS_WARN_STREAM_NAMED("manipulation",
                          "Unable to show first state of robot because graph is empty");
    return false;
  }

  // Load the OMPL visualizer
  if (!ompl_visual_tools_)
  {
    ompl_visual_tools_.reset(new ovt::OmplVisualTools(
        robot_model_->getModelFrame(), "/ompl_experience_database", planning_scene_monitor_));
    ompl_visual_tools_->loadRobotStatePub("/picknik_main/robot_state");
  }
  ompl_visual_tools_->deleteAllMarkers();  // clear all old markers
  ompl_visual_tools_->setStateSpace(model_state_space);

  // Ensure visual tools is loaded
  ros::Duration(0.1).sleep();
  ros::spinOnce();

  // Get tip links for this setup
  std::vector<const robot_model::LinkModel*> tips;
  arm_jmg->getEndEffectorTips(tips);

  if (config_->experience_type_ == "lightning")
  {
    bool show_trajectory_animated = false;

    // Loop through each path
    for (std::size_t path_id = 0; path_id < graphs.size(); ++path_id)
    {
      // std::cout << "Processing path " << path_id << std::endl;
      ompl_visual_tools_->publishRobotPath(graphs[path_id], arm_jmg, tips,
                                           show_trajectory_animated);
    }
  }
  else if (config_->experience_type_ == "thunder")
  {
    ompl_visual_tools_->publishRobotGraph(graphs[0], tips);

    // Show robot in some pose
    // for (std::size_t i = 0; i < graphs[0]->numVertices() && ros::ok(); ++i)
    // {
    //   ompl_visual_tools_->publishRobotState(graphs[0]->getVertex(i).getState());
    //   ros::Duration(0.1).sleep();
    // }
  }
  else
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Unrecognized experience type");
    return false;
  }

  return true;
}

// bool Manipulation::visualizeGrasps(std::vector<moveit_grasps::GraspCandidatePtr>
// grasp_candidates,
//                                    JointModelGroup *arm_jmg, bool show_cartesian_path)
// {
//   ROS_INFO_STREAM_NAMED("manipulation","Showing " << grasp_candidates.size() << " valid filtered
//   grasp poses");

//   // Publish in batch
//   //visuals_->visual_tools_->enableBatchPublishing(true);

//   // Get the-grasp
//   moveit::core::RobotStatePtr the_grasp_state(new moveit::core::RobotState(*current_state_));

//   Eigen::Vector3d direction;
//   direction << -1, 0, 0; // backwards towards robot body
//   double desired_distance = 0.45; //0.12; //0.15;
//   std::vector<moveit::core::RobotStatePtr> robot_state_trajectory;
//   double path_length;
//   double max_path_length = 0; // statistics
//   bool reverse_path = false;

//   for (std::size_t i = 0; i < grasp_candidates.size(); ++i)
//   {
//     if (!ros::ok())
//       return false;

//     if (show_cartesian_path)
//     {
//       the_grasp_state->setJointGroupPositions(arm_jmg, grasp_candidates[i]->grasp_ik_solution_);

//       if (!computeStraightLinePath(direction, desired_distance,
//                                    robot_state_trajectory, the_grasp_state, arm_jmg,
//                                    reverse_path, path_length))
//       {
//         ROS_WARN_STREAM_NAMED("manipulation","Unable to find straight line path");
//       }

//       // Statistics
//       if (path_length > max_path_length)
//         max_path_length = path_length;

//       bool blocking = false;
//       double speed = 0.01;
//       visuals_->visual_tools_->publishTrajectoryPath(robot_state_trajectory, arm_jmg, speed,
//       blocking);
//     }
//     std::cout << "grasp_candidates[i]->grasp_.grasp_pose: " <<
//     grasp_candidates[i]->grasp_.grasp_pose << std::endl;
//     visuals_->visual_tools_->publishZArrow(grasp_candidates[i]->grasp_.grasp_pose, rvt::RED);
//     //grasp_generator_->publishGraspArrow(grasp_candidates[i]->grasp_.grasp_pose.pose,
//     grasp_datas_[arm_jmg],
//     //                                              rvt::BLUE, path_length);

//     bool show_score = false;
//     if (show_score)
//     {
//       const geometry_msgs::Pose& pose = grasp_candidates[i]->grasp_.grasp_pose.pose;
//       double roll = atan2(2*(pose.orientation.x*pose.orientation.y +
//       pose.orientation.w*pose.orientation.z), pose.orientation.w*pose.orientation.w +
//       pose.orientation.x*pose.orientation.x - pose.orientation.y*pose.orientation.y -
//       pose.orientation.z*pose.orientation.z);
//       double yall = asin(-2*(pose.orientation.x*pose.orientation.z -
//       pose.orientation.w*pose.orientation.y));
//       double pitch = atan2(2*(pose.orientation.y*pose.orientation.z +
//       pose.orientation.w*pose.orientation.x), pose.orientation.w*pose.orientation.w -
//       pose.orientation.x*pose.orientation.x - pose.orientation.y*pose.orientation.y +
//       pose.orientation.z*pose.orientation.z);
//       std::cout << "ROLL: " << roll << " YALL: " << yall << " PITCH: " << pitch << std::endl;

//       visuals_->visual_tools_->publishText(pose, boost::lexical_cast<std::string>(yall),
//       rvt::BLACK, rvt::SMALL, false);
//       //visuals_->visual_tools_->publishAxis(pose);
//     }

//   }
//   //visuals_->visual_tools_->triggerBatchPublishAndDisable();

//   ROS_INFO_STREAM_NAMED("learning","Maximum path length in approach trajetory was " <<
//   max_path_length);

//   return true;
// }

moveit::core::RobotStatePtr Manipulation::getCurrentState()
{
  // Pass down to the exection interface layer so that we can catch the getCurrentState with a fake
  // one
  // if we are unit testing
  current_state_ = execution_interface_->getCurrentState();
  return current_state_;
}

bool Manipulation::waitForRobotToStop(const double& timeout)
{
  ROS_INFO_STREAM_NAMED("manipulation", "Waiting for robot to stop moving");
  ros::Time when_to_stop = ros::Time::now() + ros::Duration(timeout);

  static const double UPDATE_RATE = 0.1;  // how often to check if robot is stopped
  static const double POSITION_ERROR_THRESHOLD = 0.002;
  // static const std::size_t REQUIRED_STABILITY_PASSES = 4; // how many times it must be within
  // threshold in a row
  std::size_t stability_passes = 0;
  double error;
  // Get the current position
  moveit::core::RobotState previous_position = *getCurrentState();  // copy the memory
  moveit::core::RobotState current_position = previous_position;    // copy the memory

  while (ros::ok())
  {
    ros::Duration(UPDATE_RATE).sleep();
    ros::spinOnce();

    current_position = *getCurrentState();  // copy the memory

    // Check if all positions are near zero
    bool stopped = true;
    for (std::size_t i = 0; i < current_position.getVariableCount(); ++i)
    {
      error = fabs(previous_position.getVariablePositions()[i] -
                   current_position.getVariablePositions()[i]);

      if (error > POSITION_ERROR_THRESHOLD)
      {
        ROS_DEBUG_STREAM_NAMED("manipulation", "Robot is still moving with error "
                                                   << error << " on variable " << i);
        stopped = false;
      }
    }
    if (stopped)
    {
      stability_passes++;
      // ROS_INFO_STREAM_NAMED("manipulation","On stability pass " << stability_passes);
    }
    else
    {
      // Reset the stability passes because this round didn't pass
      stability_passes = 0;
    }

    if (stability_passes > 2)
      return true;

    // Check timeout
    if (ros::Time::now() > when_to_stop)
    {
      ROS_WARN_STREAM_NAMED("manipulation", "Timed out while waiting for robot to stop");
      return false;
    }

    // Copy newest positions
    previous_position = current_position;  // copy the memory
  }

  return false;
}

bool Manipulation::fixCurrentCollisionAndBounds(JointModelGroup* arm_jmg)
{
  // ROS_INFO_STREAM_NAMED("manipulation","Checking current collision and bounds");

  bool result = true;

  // Copy planning scene that is locked
  planning_scene::PlanningScenePtr cloned_scene;
  {
    planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_);
    cloned_scene = planning_scene::PlanningScene::clone(scene);
    (*current_state_) = scene->getCurrentState();
  }

  // Check for collisions
  bool verbose = false;
  if (cloned_scene->isStateColliding(*current_state_, arm_jmg->getName(), verbose))
  {
    result = false;

    // std::cout << std::endl;
    // std::cout << "-------------------------------------------------------" << std::endl;
    ROS_WARN_STREAM_NAMED("manipulation", "State is colliding, attempting to fix...");

    // Show collisions
    visuals_->visual_tools_->publishContactPoints(*current_state_, cloned_scene.get());
    visuals_->visual_tools_->publishRobotState(current_state_, rvt::RED);

    // Attempt to fix collision state
    if (!fixCollidingState(cloned_scene))
    {
      ROS_ERROR_STREAM_NAMED("manipulation", "Unable to fix colliding state");
    }
  }
  else
  {
    ROS_INFO_STREAM_NAMED("manipulation", "State is not colliding");
  }

  // Check if satisfies bounds
  if (!current_state_->satisfiesBounds(arm_jmg, fix_state_bounds_.getMaxBoundsError()))
  {
    std::cout << std::endl;
    std::cout << "-------------------------------------------------------" << std::endl;
    ROS_WARN_STREAM_NAMED("manipulation", "State does not satisfy bounds, attempting to fix...");
    std::cout << "-------------------------------------------------------" << std::endl;

    moveit::core::RobotStatePtr new_state(new moveit::core::RobotState(*current_state_));

    if (!fix_state_bounds_.fixBounds(*new_state, arm_jmg))
    {
      ROS_WARN_STREAM_NAMED("manipulation", "Unable to fix state bounds or change not required");
    }
    else
    {
      // Alert human to error
      remote_control_->setAutonomous(false);

      // State was modified, send to robot
      if (!executeState(new_state, arm_jmg, config_->main_velocity_scaling_factor_))
      {
        ROS_ERROR_STREAM_NAMED("manipulation", "Unable to exceute state bounds fix");
      }
      result = false;
    }
  }
  else
  {
    ROS_INFO_STREAM_NAMED("manipulation", "State satisfies bounds");
  }
  return result;
}

bool Manipulation::checkCollisionAndBounds(const moveit::core::RobotStatePtr& start_state,
                                           const moveit::core::RobotStatePtr& goal_state,
                                           bool verbose)
{
  bool result = true;

  // Check if satisfies bounds  --------------------------------------------------------

  // Start
  if (!start_state->satisfiesBounds(fix_state_bounds_.getMaxBoundsError()))
  {
    if (verbose)
    {
      ROS_WARN_STREAM_NAMED("manipulation", "Start state does not satisfy bounds");

      // For debugging in console
      showJointLimits(config_->right_arm_);
    }
    result = false;
  }

  // Goal
  if (goal_state && !goal_state->satisfiesBounds(fix_state_bounds_.getMaxBoundsError()))
  {
    if (verbose)
    {
      ROS_WARN_STREAM_NAMED("manipulation", "Goal state does not satisfy bounds");

      // For debugging in console
      showJointLimits(config_->right_arm_);
    }

    result = false;
  }

  // Check for collisions --------------------------------------------------------
  JointModelGroup* arm_jmg = config_->dual_arm_ ? config_->both_arms_ : config_->right_arm_;

  // Get planning scene lock
  {
    planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_);
    // Start
    if (scene->isStateColliding(*start_state, arm_jmg->getName(), verbose))
    {
      if (verbose)
      {
        ROS_WARN_STREAM_NAMED("manipulation.checkCollisionAndBounds", "Start state is colliding");
        // Show collisions
        visuals_->visual_tools_->publishContactPoints(
            *start_state, planning_scene_monitor_->getPlanningScene().get());
        visuals_->visual_tools_->publishRobotState(*start_state, rvt::RED);
      }
      result = false;
    }

    // Goal
    if (goal_state)
    {
      goal_state->update();

      if (scene->isStateColliding(*goal_state, arm_jmg->getName(), verbose))
      {
        if (verbose)
        {
          ROS_WARN_STREAM_NAMED("manipulation.checkCollisionAndBounds", "Goal state is colliding");
          // Show collisions
          visuals_->visual_tools_->publishContactPoints(
              *goal_state, planning_scene_monitor_->getPlanningScene().get());
          visuals_->visual_tools_->publishRobotState(*goal_state, rvt::RED);
        }
        result = false;
      }
    }
  }

  return result;
}

bool Manipulation::getPose(Eigen::Affine3d& pose, const std::string& frame_name)
{
  tf::StampedTransform camera_transform;
  try
  {
    // Wait to make sure a transform message has arrived
    planning_scene_monitor_->getTFClient()->waitForTransform(config_->robot_base_frame_, frame_name,
                                                             ros::Time(0), ros::Duration(1));
    // Get latest transform available
    planning_scene_monitor_->getTFClient()->lookupTransform(config_->robot_base_frame_, frame_name,
                                                            ros::Time(0), camera_transform);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "TF error: " << ex.what());
    return false;
  }

  // Copy results
  tf::transformTFToEigen(camera_transform, pose);

  return true;
}

double Manipulation::getMaxJointLimit(const moveit::core::JointModel* joint)
{
  // Assume all joints have only one variable
  if (joint->getVariableCount() > 1)
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Unable to handle joints with more than one variable");
    return 0;
  }

  const moveit::core::VariableBounds& bound = joint->getVariableBounds()[0];

  return bound.max_position_;
}

double Manipulation::getMinJointLimit(const moveit::core::JointModel* joint)
{
  // Assume all joints have only one variable
  if (joint->getVariableCount() > 1)
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Unable to handle joints with more than one variable");
    return 0;
  }

  const moveit::core::VariableBounds& bound = joint->getVariableBounds()[0];

  return bound.min_position_;
}

bool Manipulation::showJointLimits(JointModelGroup* jmg)
{
  const std::vector<const moveit::core::JointModel*>& joints = jmg->getActiveJointModels();

  std::cout << std::endl;

  // Loop through joints
  for (std::size_t i = 0; i < joints.size(); ++i)
  {
    // Assume all joints have only one variable
    if (joints[i]->getVariableCount() > 1)
    {
      ROS_ERROR_STREAM_NAMED("manipulation", "Unable to handle joints with more than one var");
      return false;
    }
    getCurrentState();
    double current_value = current_state_->getVariablePosition(joints[i]->getName());

    // check if bad position
    bool out_of_bounds = !current_state_->satisfiesBounds(joints[i]);

    const moveit::core::VariableBounds& bound = joints[i]->getVariableBounds()[0];

    if (out_of_bounds)
      std::cout << MOVEIT_CONSOLE_COLOR_RED;

    std::cout << "   " << std::fixed << std::setprecision(5) << bound.min_position_ << "\t";
    double delta = bound.max_position_ - bound.min_position_;
    // std::cout << "delta: " << delta << " ";
    double step = delta / 20.0;

    bool marker_shown = false;
    for (double value = bound.min_position_; value < bound.max_position_; value += step)
    {
      // show marker of current value
      if (!marker_shown && current_value < value)
      {
        std::cout << "|";
        marker_shown = true;
      }
      else
        std::cout << "-";
    }
    // show max position
    std::cout << " \t" << std::fixed << std::setprecision(5) << bound.max_position_ << "  \t"
              << joints[i]->getName() << " current: " << std::fixed << std::setprecision(5)
              << current_value << std::endl;

    if (out_of_bounds)
      std::cout << MOVEIT_CONSOLE_COLOR_RESET;
  }

  return true;
}

bool Manipulation::beginTouchControl()
{
  JointModelGroup* arm_jmg = config_->dual_arm_ ? config_->both_arms_ : config_->right_arm_;

  // Initialize the teleop pose
  teleop_world_to_tool_ =
      getCurrentState()->getGlobalLinkTransform(grasp_datas_[arm_jmg]->parent_link_);

  // Move the teleop pose forward from EE base to finger tips
  teleop_world_to_tool_ = teleop_world_to_tool_ * config_->teleoperation_offset_.inverse();

  tactile_feedback_->setEndEffectorDataCallback(
      std::bind(&Manipulation::updateTouchControl, this, arm_jmg));

  ros::spin();

  return true;
}

// Multi-use function for adjusting teleoperating and insertion tasks
bool Manipulation::adjustPoseFromTactile()
{
  // Two experimental modes - translation and pitch rotation
  bool translation_teleop = false;
  if (translation_teleop)
  {
    // Check if overall sheer force is enough to move the arm
    if (tactile_feedback_->getSheerForce() < config_->sheer_force_threshold_)
    {
      std::cout << "force to low: " << tactile_feedback_->getSheerForce() << "/"
                << config_->sheer_force_threshold_ << " --------------------------------\n";
      return false;
    }

    // Calculate amount to translate
    teleop_direction_ << -1 * sin(tactile_feedback_->getSheerTheta()), 0,
        cos(tactile_feedback_->getSheerTheta());
    teleop_rotated_direction_ = teleop_world_to_tool_.rotation() * teleop_direction_;

    // Calculate translation amount based on sheer force
    static const double SHEER_FORCE_MAX = 20;  // ignore sheer force higher than this
    const double sheer_force = std::min(SHEER_FORCE_MAX, tactile_feedback_->getSheerForce());
    static const double SHEER_RATIO = config_->touch_teleop_max_translation_step_ / SHEER_FORCE_MAX;
    const double sheer_force_gain = sheer_force * SHEER_RATIO;

    // The target pose is built by applying a translation to the target pose for the desired
    // direction and distance
    teleop_world_to_tool_.translation() += teleop_rotated_direction_ * sheer_force_gain;
    return true;
  }
  else
  {
    // rotate based on torque
    bool verbose_torque = true;

    // Max threhold, absolute
    // Notes: getSheerTorque() will generally return values between -360 -> 0 -> 360 but can exceed
    // those values also. 0 is the calibrated position, i.e. no toruqe
    double torque = std::min(tactile_feedback_->getSheerTorque(), config_->insertion_torque_max_);
    torque = std::max(tactile_feedback_->getSheerTorque(), -config_->insertion_torque_max_);

    // Debug
    if (verbose_torque && false)
    {
      std::cout << "raw torque: " << tactile_feedback_->getSheerTorque() << std::endl;
      std::cout << "capped torque: " << torque << std::endl;
    }

    // Min threshold, absolute
    if (fabs(torque) < config_->insertion_torque_min_)
    {
      const bool show = false;
      showDirectionArrow(torque, show);

      if (verbose_torque && false)
        std::cout << "Ignoring torque because below threshold: " << torque << std::endl;
      return false;
    }

    // Remove bottom of torque so that there are no jumps after the min
    if (torque > 0)
      torque -= config_->insertion_torque_min_;
    else
      torque += config_->insertion_torque_min_;

    if (verbose_torque)
      std::cout << "  Pre-scaled torque: " << torque << std::endl;

    // Scale torque rotation to output rotation
    torque *= config_->insertion_torque_scale_;
    if (verbose_torque)
      std::cout << "  Scaled (reduced) torque: " << torque << std::endl;

    // Show pre-rotation pose
    // visuals_->trajectory_lines_->publishZArrow(
    // teleop_world_to_tool_ * config_->teleoperation_offset_, rvt::GREEN, rvt::REGULAR);

    // Show arrow in direction of movement
    const bool show = true;
    showDirectionArrow(torque, show);

    // Apply torque to EE pose
    Eigen::Affine3d rotation;
    rotation = Eigen::AngleAxisd(torque, Eigen::Vector3d::UnitY());
    teleop_world_to_tool_ = teleop_world_to_tool_ * rotation;
    return true;
  }
}

void Manipulation::showDirectionArrow(double torque, bool show)
{
  // Move pose from tips of finger (tool) back to base of EE
  Eigen::Affine3d visualize_direction_arrow =  // teleop_world_to_tool_;
      teleop_world_to_tool_ * config_->teleoperation_offset_;

  // Rotate to be z-aligned
  visualize_direction_arrow =
      visualize_direction_arrow * Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitY());

  // Rotate 90
  Eigen::Affine3d rotation;
  if (torque > 0)
    torque = -1.57;
  else
    torque = 1.57;
  rotation = Eigen::AngleAxisd(torque, Eigen::Vector3d::UnitY());
  visualize_direction_arrow = visualize_direction_arrow * rotation;

  // Set the frame ID and timestamp.
  visualization_msgs::Marker arrow_marker;
  arrow_marker.header.stamp = ros::Time::now();
  arrow_marker.header.frame_id = config_->robot_base_frame_;

  // Pose
  arrow_marker.pose = visuals_->trajectory_lines_->convertPose(visualize_direction_arrow);

  // Set the namespace and id for this marker.  This serves to create a unique ID
  arrow_marker.ns = "Movement Direction";
  arrow_marker.id = 2;

  // Other properites
  arrow_marker.type = visualization_msgs::Marker::ARROW;
  if (show)
    arrow_marker.action = visualization_msgs::Marker::ADD;
  else
    arrow_marker.action = visualization_msgs::Marker::DELETE;
  arrow_marker.lifetime = ros::Duration(0.0);
  arrow_marker.color = visuals_->trajectory_lines_->getColor(rvt::BLACK);
  arrow_marker.scale = visuals_->trajectory_lines_->getScale(rvt::LARGE, true);
  arrow_marker.scale.x = 0.1;  // overrides previous x scale specified

  // Publish
  visuals_->trajectory_lines_->publishMarker(arrow_marker);
}

// This function is called from tactile_feedback.cpp as a binded function
void Manipulation::updateTouchControl(JointModelGroup* arm_jmg)
{
  // Make sure a previous call isn't currently blocking
  if (remote_control_->isWaiting())
    return;

  if (remote_control_->getStop())
  {
    std::cout << "Error: remote control stopped " << std::endl;
    return;
  }

  // Adjust pose based on tactile feedback
  adjustPoseFromTactile();

  // Move pose from tips of finger (tool) back to base of EE
  teleop_world_to_ee_ = teleop_world_to_tool_ * config_->teleoperation_offset_;

  // Show pose-rotation pose
  visuals_->trajectory_lines_->publishZArrow(teleop_world_to_ee_, rvt::ORANGE, rvt::REGULAR);

  // Convert deisred pose from 'world' frame to 'robot base' frame
  Eigen::Affine3d base_to_world = getCurrentState()->getGlobalLinkTransform("base_link").inverse();
  teleop_base_to_ee_ = base_to_world * teleop_world_to_ee_;

  remote_control_->waitForNextStep("move");

  // Move robot
  execution_interface_->executePose(teleop_base_to_ee_, arm_jmg);

  ros::Duration(1.0).sleep();
}

// Note: deprecated function
bool Manipulation::teleoperation(const Eigen::Affine3d& ee_pose, bool move,
                                 JointModelGroup* arm_jmg)
{
  // NOTE this is in a separate thread, so we should only use visuals_->trajectory_lines_ for
  // debugging!

  // Solve IK
  bool use_consistency_limits = true;
  if (!getRobotStateFromPose(ee_pose, teleop_state_, arm_jmg, use_consistency_limits))
    return false;

  // Execute robot pose
  if (move)
  {
    const double velocity_scaling_factor = 1.0;
    if (!moveDirectToState(teleop_state_, arm_jmg, velocity_scaling_factor))
    {
      ROS_ERROR_STREAM_NAMED("pick_manager", "Failed to execute state");
      return false;
    }
  }
  else
  {
    // Visualize what we would have done
    visuals_->goal_state_->publishRobotState(teleop_state_, rvt::BLUE);
  }

  return true;
}

bool Manipulation::enableTeleoperation()
{
  teleop_state_.reset(new moveit::core::RobotState(*getCurrentState()));
  return true;
}

void Manipulation::transformGlobalToBaseLink(Eigen::Affine3d& pose)
{
  const Eigen::Affine3d& world_to_base = getCurrentState()->getGlobalLinkTransform("base_link");
  pose = world_to_base.inverse() * pose;
}

}  // end namespace

namespace
{
bool isStateValid(const planning_scene::PlanningScene* planning_scene, bool verbose,
                  bool only_check_self_collision, picknik_main::VisualsPtr visuals,
                  moveit::core::RobotState* robot_state, JointModelGroup* group,
                  const double* ik_solution)
{
  robot_state->setJointGroupPositions(group, ik_solution);
  robot_state->update();

  if (!planning_scene)
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "No planning scene provided");
    return false;
  }
  if (only_check_self_collision)
  {
    // No easy API exists for only checking self-collision, so we do it here. TODO: move this big
    // into planning_scene.cpp
    collision_detection::CollisionRequest req;
    req.verbose = false;
    req.group_name = group->getName();
    collision_detection::CollisionResult res;
    planning_scene->checkSelfCollision(req, res, *robot_state);
    if (!res.collision)
      return true;  // not in collision
  }
  else if (!planning_scene->isStateColliding(*robot_state, group->getName()))
    return true;  // not in collision

  // Display more info about the collision
  if (verbose)
  {
    visuals->visual_tools_->publishRobotState(*robot_state, rvt::RED);
    planning_scene->isStateColliding(*robot_state, group->getName(), true);
    visuals->visual_tools_->publishContactPoints(*robot_state, planning_scene);
    ros::Duration(0.4).sleep();
  }
  return false;
}

}  // end annonymous namespace
