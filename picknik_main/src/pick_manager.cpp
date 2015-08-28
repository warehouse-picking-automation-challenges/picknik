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
   Desc:   Main logic of picking
*/

// Command line arguments
#include <gflags/gflags.h>

// PickNik
#include <picknik_main/pick_manager.h>

// MoveIt
//#include <moveit/robot_state/conversions.h>
#include <moveit/macros/console_colors.h>

// Boost
//#include <boost/filesystem.hpp>
//#include <boost/foreach.hpp>

namespace picknik_main
{
DEFINE_bool(fake_execution, false, "Fake execution of motions");
DEFINE_bool(fake_perception, true, "Fake perception of things");
DEFINE_bool(use_experience, true, "Plan with an experience database");
DEFINE_bool(show_database, true, "Show experience database");
DEFINE_int32(id, 0, "Identification number for various component modes");

PickManager::PickManager(bool verbose)
  : nh_private_("~")
  , verbose_(verbose)
  , fake_perception_(FLAGS_fake_perception)
  , teleoperation_enabled_(false)
{
  // Warn of fake modes
  if (fake_perception_)
    ROS_WARN_STREAM_NAMED("pick_manager", "In fake perception mode");
  if (FLAGS_fake_execution)
    ROS_WARN_STREAM_NAMED("pick_manager", "In fake execution mode");

  // Load the loader
  robot_model_loader_.reset(new robot_model_loader::RobotModelLoader(ROBOT_DESCRIPTION));

  // Load the robot model
  robot_model_ = robot_model_loader_->getModel();  // Get a shared pointer to the robot

  // Create the planning scene
  planning_scene_.reset(new planning_scene::PlanningScene(robot_model_));

  // Create the planning scene service
  // get_scene_service_ = nh_root_.advertiseService(GET_PLANNING_SCENE_SERVICE_NAME,
  // &PickManager::getPlanningSceneService, this);

  // Load manipulation data for our robot
  config_.reset(new ManipulationData());
  config_->load(robot_model_, FLAGS_fake_execution, package_path_);

  // Create tf transformer
  tf_.reset(new tf::TransformListener(nh_private_));
  // TODO: remove these lines, only an attempt to fix loadPlanningSceneMonitor bug
  ros::spinOnce();

  // Load planning scene monitor
  if (!loadPlanningSceneMonitor())
  {
    ROS_ERROR_STREAM_NAMED("pick_manager", "Unable to load planning scene monitor");
  }

  // Load multiple visual_tools classes
  visuals_.reset(new Visuals(robot_model_, planning_scene_monitor_));

  // Get package path
  package_path_ = ros::package::getPath(PACKAGE_NAME);
  if (package_path_.empty())
    ROS_FATAL_STREAM_NAMED("product", "Unable to get " << PACKAGE_NAME << " package path");

  // Load the remote control for dealing with GUIs
  remote_control_.reset(new RemoteControl(verbose, nh_private_, this));

  // Load line tracker
  tactile_feedback_.reset(new TactileFeedback(config_));

  // Load grasp data specific to our robot
  grasp_datas_[config_->right_arm_].reset(
      new moveit_grasps::GraspData(nh_private_, config_->right_hand_name_, robot_model_));
  // special for jaco
  // grasp_datas_[config_->arm_only_].reset(new moveit_grasps::GraspData(nh_private_,
  // config_->right_hand_name_, robot_model_));

  if (config_->dual_arm_)
    grasp_datas_[config_->left_arm_].reset(
        new moveit_grasps::GraspData(nh_private_, config_->left_hand_name_, robot_model_));

  // Create manipulation manager
  manipulation_.reset(new Manipulation(verbose_, visuals_, planning_scene_monitor_, config_,
                                       grasp_datas_, remote_control_, FLAGS_fake_execution,
                                       tactile_feedback_));

  // Load trajectory IO class
  // trajectory_io_.reset(new TrajectoryIO(remote_control_, visuals_, config_, manipulation_));

  // Load perception layer
  perception_interface_.reset(
      new PerceptionInterface(verbose_, visuals_, config_, tf_, nh_private_));

  // Load planning scene manager
  planning_scene_manager_.reset(new PlanningSceneManager(verbose, visuals_, perception_interface_));

  // Show interactive marker
  setupInteractiveMarker();

  ROS_INFO_STREAM_NAMED("pick_manager", "PickManager Ready.");
}

bool PickManager::checkSystemReady()
{
  std::cout << std::endl;
  std::cout << std::endl;
  std::cout << "-------------------------------------------------------" << std::endl;
  ROS_INFO_STREAM_NAMED("pick_manager", "Starting system ready check:");

  // Check joint model groups, assuming we are the jaco arm
  if (config_->right_arm_->getVariableCount() < 6 || config_->right_arm_->getVariableCount() > 7)
  {
    ROS_FATAL_STREAM_NAMED("pick_manager", "Incorrect number of joints for group "
                                               << config_->right_arm_->getName() << ", joints: "
                                               << config_->right_arm_->getVariableCount());
    return false;
  }
  JointModelGroup* ee_jmg = grasp_datas_[config_->right_arm_]->ee_jmg_;
  if (ee_jmg->getVariableCount() > 6)
  {
    ROS_FATAL_STREAM_NAMED("pick_manager", "Incorrect number of joints for group "
                                               << ee_jmg->getName()
                                               << ", joints: " << ee_jmg->getVariableCount());
    return false;
  }

  // Check trajectory execution manager
  if (!manipulation_->getExecutionInterface()->checkExecutionManager())
  {
    ROS_FATAL_STREAM_NAMED("pick_manager", "Trajectory controllers unable to connect");
    return false;
  }

  // Check Perception
  if (!fake_perception_)
  {
    ROS_INFO_STREAM_NAMED("pick_manager", "Checking perception");
    perception_interface_->isPerceptionReady();
  }

  // Choose which planning group to use
  // JointModelGroup* arm_jmg = config_->dual_arm_ ? config_->both_arms_ : config_->right_arm_;

  // Check robot calibrated
  // TODO

  // Check gantry calibrated
  // TODO

  // Check end effectors calibrated
  // TODO

  ROS_INFO_STREAM_NAMED("pick_manager", "System ready check COMPLETE");
  std::cout << "-------------------------------------------------------" << std::endl;
  return true;
}

// Mode 8
bool PickManager::testEndEffectors()
{
  // Test visualization
  std::size_t i = 0;
  bool open;
  moveit::core::RobotStatePtr current_state = manipulation_->getCurrentState();
  while (ros::ok())
  {
    std::cout << std::endl << std::endl;
    if (i % 2 == 0)
    {
      std::cout << "Showing closed EE of state " << std::endl;

      open = false;
      // manipulation_->setStateWithOpenEE(open, current_state);
      // visuals_->visual_tools_->publishRobotState(current_state);

      // Close all EEs
      manipulation_->openEEs(open);

      ros::Duration(2.0).sleep();
    }
    else
    {
      std::cout << "Showing open EE of state " << std::endl;

      open = true;
      // manipulation_->setStateWithOpenEE(open, current_state);
      // visuals_->visual_tools_->publishRobotState(current_state);

      // Close all EEs
      manipulation_->openEEs(open);

      ros::Duration(2.0).sleep();
    }
    ++i;
  }

  ROS_INFO_STREAM_NAMED("pick_manager", "Done testing end effectors");
  return true;
}

// Mode 5
bool PickManager::testUpAndDown()
{
  double lift_distance_desired = 0.5;

  // Test
  std::size_t i = 0;
  while (ros::ok())
  {
    std::cout << std::endl << std::endl;
    if (i % 2 == 0)
    {
      std::cout << "Moving up --------------------------------------" << std::endl;
      manipulation_->executeVerticlePath(config_->right_arm_, lift_distance_desired,
                                         config_->lift_velocity_scaling_factor_, true);
      if (config_->dual_arm_)
        manipulation_->executeVerticlePath(config_->left_arm_, lift_distance_desired,
                                           config_->lift_velocity_scaling_factor_, true);
      ros::Duration(1.0).sleep();
    }
    else
    {
      std::cout << "Moving down ------------------------------------" << std::endl;
      manipulation_->executeVerticlePath(config_->right_arm_, lift_distance_desired,
                                         config_->lift_velocity_scaling_factor_, false);
      if (config_->dual_arm_)
        manipulation_->executeVerticlePath(config_->left_arm_, lift_distance_desired,
                                           config_->lift_velocity_scaling_factor_, false);
      ros::Duration(1.0).sleep();
    }
    ++i;
  }

  ROS_INFO_STREAM_NAMED("pick_manager", "Done testing up and down");
  return true;
}

// Mode 10
bool PickManager::testInAndOut()
{
  double approach_distance_desired = 1.0;

  // Test
  std::size_t i = 1;
  while (ros::ok())
  {
    visuals_->visual_tools_->deleteAllMarkers();

    std::cout << std::endl << std::endl;
    if (i % 2 == 0)
    {
      std::cout << "Moving in --------------------------------------" << std::endl;
      if (!manipulation_->executeRetreatPath(config_->right_arm_, approach_distance_desired, false))
        return false;
      if (config_->dual_arm_)
        if (!manipulation_->executeRetreatPath(config_->left_arm_, approach_distance_desired,
                                               false))
          return false;
      ros::Duration(1.0).sleep();
    }
    else
    {
      std::cout << "Moving out ------------------------------------" << std::endl;
      if (!manipulation_->executeRetreatPath(config_->right_arm_, approach_distance_desired, true))
        return false;
      if (config_->dual_arm_)
        if (!manipulation_->executeRetreatPath(config_->left_arm_, approach_distance_desired, true))
          return false;
      ros::Duration(1.0).sleep();
    }
    ++i;
  }

  ROS_INFO_STREAM_NAMED("pick_manager", "Done testing in and out");
  return true;
}

// Mode 41
bool PickManager::getSRDFPose()
{
  ROS_DEBUG_STREAM_NAMED("pick_manager", "Get SRDF pose");

  // Choose which planning group to use
  JointModelGroup* arm_jmg = config_->dual_arm_ ? config_->both_arms_ : config_->right_arm_;
  const std::vector<const moveit::core::JointModel*> joints = arm_jmg->getJointModels();

  while (ros::ok())
  {
    ROS_INFO_STREAM("SDF Code for joint values pose:\n");

    // Get current state after grasping
    moveit::core::RobotStatePtr current_state = manipulation_->getCurrentState();

    // Check if current state is valid
    // manipulation_->fixCurrentCollisionAndBounds(arm_jmg);

    // Output XML
    std::cout << "<group_state name=\"\" group=\"" << arm_jmg->getName() << "\">\n";
    for (std::size_t i = 0; i < joints.size(); ++i)
    {
      std::cout << "  <joint name=\"" << joints[i]->getName() << "\" value=\""
                << current_state->getJointPositions(joints[i])[0] << "\" />\n";
    }
    std::cout << "</group_state>\n\n\n\n";

    ros::Duration(4.0).sleep();
  }
  return true;
}

// Mode 42
bool PickManager::testInCollision()
{
  while (ros::ok())
  {
    std::cout << std::endl;

    // For debugging in console
    manipulation_->showJointLimits(config_->right_arm_);

    // manipulation_->fixCurrentCollisionAndBounds(arm_jmg);
    manipulation_->checkCollisionAndBounds(manipulation_->getCurrentState());
    ros::Duration(0.1).sleep();
  }

  ROS_INFO_STREAM_NAMED("pick_manager", "Done checking if in collision");
  return true;
}

// Mode 6
bool PickManager::testRandomValidMotions()
{
  // Allow collision between Jacob and bottom for most links
  {
    planning_scene_monitor::LockedPlanningSceneRW scene(
        planning_scene_monitor_);  // Lock planning scene

    scene->getAllowedCollisionMatrixNonConst().setEntry("base_39", "frame", true);
    scene->getAllowedCollisionMatrixNonConst().setEntry("base_39", "gantry", true);
    scene->getAllowedCollisionMatrixNonConst().setEntry("base_39", "gantry_plate", true);
    scene->getAllowedCollisionMatrixNonConst().setEntry("base_39", "jaco2_link_base", true);
    scene->getAllowedCollisionMatrixNonConst().setEntry("base_39", "jaco2_link_1", true);
  }

  // Plan to random
  while (ros::ok())
  {
    static const std::size_t MAX_ATTEMPTS = 200;
    for (std::size_t i = 0; i < MAX_ATTEMPTS; ++i)
    {
      ROS_DEBUG_STREAM_NAMED("pick_manager", "Attempt " << i << " to plan to a random location");

      // Create start
      moveit::core::RobotStatePtr current_state = manipulation_->getCurrentState();

      // Create goal
      moveit::core::RobotStatePtr goal_state(new moveit::core::RobotState(*current_state));

      // Choose arm
      JointModelGroup* arm_jmg = config_->right_arm_;
      if (config_->dual_arm_)
        if (visuals_->visual_tools_->iRand(0, 1) == 0)
          arm_jmg = config_->left_arm_;

      goal_state->setToRandomPositions(arm_jmg);

      // Check if random goal state is valid
      bool collision_verbose = false;
      if (manipulation_->checkCollisionAndBounds(current_state, goal_state, collision_verbose))
      {
        // Plan to this position
        bool verbose = true;
        bool execute_trajectory = true;
        if (manipulation_->move(current_state, goal_state, arm_jmg,
                                config_->main_velocity_scaling_factor_, verbose,
                                execute_trajectory))
        {
          ROS_INFO_STREAM_NAMED("pick_manager", "Planned to random valid state successfullly");
        }
        else
        {
          ROS_ERROR_STREAM_NAMED("pick_manager", "Failed to plan to random valid state");
          return false;
        }
      }
    }
    ROS_ERROR_STREAM_NAMED("pick_manager", "Unable to find random valid state after "
                                               << MAX_ATTEMPTS << " attempts");

    ros::Duration(1).sleep();
  }  // while

  ROS_INFO_STREAM_NAMED("pick_manager", "Done planning to random valid");
  return true;
}

// Mode 2
bool PickManager::testGoHome()
{
  ROS_DEBUG_STREAM_NAMED("pick_manager", "Going home");

  // Choose which planning group to use
  JointModelGroup* arm_jmg = config_->dual_arm_ ? config_->both_arms_ : config_->right_arm_;
  moveToStartPosition(arm_jmg);
  return true;
}

// Mode 17
bool PickManager::testJointLimits()
{
  ROS_INFO_STREAM_NAMED("pick_manager", "Testing joint limits");
  ROS_WARN_STREAM_NAMED("pick_manager", "DOES NOT CHECK FOR COLLISION");

  moveit::core::RobotStatePtr current_state = manipulation_->getCurrentState();

  // Create goal
  moveit::core::RobotStatePtr goal_state(new moveit::core::RobotState(*current_state));

  // Setup data
  std::vector<double> joint_position;
  joint_position.resize(1);
  const std::vector<const moveit::core::JointModel*>& joints =
      config_->right_arm_->getActiveJointModels();

  // Decide if we are testing 1 joint or all
  int test_joint_limit_joint = 0;
  std::size_t first_joint;
  std::size_t last_joint;
  if (test_joint_limit_joint < 0)
  {
    first_joint = 0;
    last_joint = joints.size();
  }
  else
  {
    first_joint = test_joint_limit_joint;
    last_joint = test_joint_limit_joint + 1;
  }

  // Keep testing
  while (true)
  {
    // Loop through each joint, assuming each joint has only 1 variable
    for (std::size_t i = first_joint; i < last_joint; ++i)
    {
      if (!ros::ok())
        return false;

      const moveit::core::VariableBounds& bound = joints[i]->getVariableBounds()[0];
      double reduce_bound = 0.01;

      // Move to min bound
      std::cout << std::endl;
      std::cout << "-------------------------------------------------------" << std::endl;
      joint_position[0] = bound.min_position_ + reduce_bound;
      ROS_INFO_STREAM_NAMED("pick_manager", "Sending joint " << joints[i]->getName()
                                                             << " to min position of "
                                                             << joint_position[0]);
      goal_state->setJointPositions(joints[i], joint_position);

      if (!manipulation_->executeState(goal_state, config_->right_arm_,
                                       config_->main_velocity_scaling_factor_))
      {
        ROS_ERROR_STREAM_NAMED("pick_manager", "Unable to move to min bound of "
                                                   << joint_position[0] << " on joint "
                                                   << joints[i]->getName());
      }
      ros::Duration(1.0).sleep();

      // Move to max bound
      std::cout << std::endl;
      std::cout << "-------------------------------------------------------" << std::endl;
      joint_position[0] = bound.max_position_ - reduce_bound;
      ROS_INFO_STREAM_NAMED("pick_manager", "Sending joint " << joints[i]->getName()
                                                             << " to max position of "
                                                             << joint_position[0]);
      goal_state->setJointPositions(joints[i], joint_position);

      if (!manipulation_->executeState(goal_state, config_->right_arm_,
                                       config_->main_velocity_scaling_factor_))
      {
        ROS_ERROR_STREAM_NAMED("pick_manager", "Unable to move to max bound of "
                                                   << joint_position[0] << " on joint "
                                                   << joints[i]->getName());
      }
      ros::Duration(1.0).sleep();
    }
  }

  ROS_INFO_STREAM_NAMED("pick_manager", "Done testing joint limits");
  return true;
}

bool PickManager::recordTrajectory()
{
  std::string file_path;
  const std::string file_name = "test_trajectory";
  trajectory_io_->getFilePath(file_path, file_name);

  // Start recording
  trajectory_io_->recordTrajectoryToFile(file_path);

  ROS_INFO_STREAM_NAMED("pick_manager", "Done recording");

  return true;
}

// Mode 34
bool PickManager::playbackTrajectory()
{
  // Choose which planning group to use
  JointModelGroup* arm_jmg = config_->arm_only_;
  if (!arm_jmg)
  {
    ROS_ERROR_STREAM_NAMED("pick_manager", "No joint model group for arm");
    return false;
  }

  // Start playing back file
  std::string file_path;
  const std::string file_name = "calibration_waypoints";
  trajectory_io_->getFilePath(file_path, file_name);

  if (!trajectory_io_->playbackWaypointsFromFile(file_path, arm_jmg,
                                                 config_->calibration_velocity_scaling_factor_))
  {
    ROS_ERROR_STREAM_NAMED("pick_manager", "Unable to playback CSV from file for pose waypoints");
    return false;
  }

  return true;
}

bool PickManager::moveToStartPosition(JointModelGroup* arm_jmg, bool check_validity)
{
  return manipulation_->moveToStartPosition(arm_jmg, check_validity);
}

bool PickManager::loadPlanningSceneMonitor()
{
  // Allows us to sycronize to Rviz and also publish collision objects to ourselves
  ROS_DEBUG_STREAM_NAMED("pick_manager", "Loading Planning Scene Monitor");
  static const std::string PLANNING_SCENE_MONITOR_NAME = "AmazonShelfWorld";
  planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor(
      planning_scene_, robot_model_loader_, tf_, PLANNING_SCENE_MONITOR_NAME));
  ros::spinOnce();

  if (planning_scene_monitor_->getPlanningScene())
  {
    // Optional monitors to start:
    planning_scene_monitor_->startStateMonitor(config_->joint_state_topic_, "");
    planning_scene_monitor_->startPublishingPlanningScene(
        planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE, "picknik_planning_scene");
    planning_scene_monitor_->getPlanningScene()->setName("picknik_planning_scene");
  }
  else
  {
    ROS_ERROR_STREAM_NAMED("pick_manager", "Planning scene not configured");
    return false;
  }
  ros::spinOnce();
  ros::Duration(0.5).sleep();  // when at 0.1, i believe sometimes vjoint not properly loaded

  // Wait for complete state to be recieved
  bool wait_for_complete_state = false;
  // Break early
  if (!wait_for_complete_state)
    return true;

  std::vector<std::string> missing_joints;
  std::size_t counter = 0;
  while (!planning_scene_monitor_->getStateMonitor()->haveCompleteState() && ros::ok())
  {
    ROS_INFO_STREAM_THROTTLE_NAMED(1, "pick_manager", "Waiting for complete state from topic "
                                                          << config_->joint_state_topic_);
    ros::Duration(0.1).sleep();
    ros::spinOnce();

    // Show unpublished joints
    if (counter % 10 == 0)
    {
      planning_scene_monitor_->getStateMonitor()->haveCompleteState(missing_joints);
      for (std::size_t i = 0; i < missing_joints.size(); ++i)
        ROS_WARN_STREAM_NAMED("pick_manager", "Unpublished joints: " << missing_joints[i]);
    }
    counter++;
  }
  ros::spinOnce();

  return true;
}

void PickManager::publishCurrentState()
{
  planning_scene_monitor::LockedPlanningSceneRO scene(
      planning_scene_monitor_);  // Lock planning scene
  visuals_->visual_tools_->publishRobotState(scene->getCurrentState(), rvt::PURPLE);
}

bool PickManager::getPlanningSceneService(moveit_msgs::GetPlanningScene::Request& req,
                                          moveit_msgs::GetPlanningScene::Response& res)
{
  if (req.components.components & moveit_msgs::PlanningSceneComponents::TRANSFORMS)
    planning_scene_monitor_->updateFrameTransforms();
  planning_scene_monitor::LockedPlanningSceneRO ps(planning_scene_monitor_);
  ps->getPlanningSceneMsg(res.scene, req.components);
  return true;
}

RemoteControlPtr PickManager::getRemoteControl() { return remote_control_; }
bool PickManager::allowCollisions(JointModelGroup* arm_jmg)
{
  // Allow collisions between frame of robot and floor
  {
    planning_scene_monitor::LockedPlanningSceneRW scene(planning_scene_monitor_);  // Lock planning
    collision_detection::AllowedCollisionMatrix& collision_matrix =
        scene->getAllowedCollisionMatrixNonConst();

    // Get links of end effector
    const std::vector<std::string>& ee_link_names =
        grasp_datas_[arm_jmg]->ee_jmg_->getLinkModelNames();
    for (std::size_t i = 0; i < ee_link_names.size(); ++i)
    {
      for (std::size_t j = i + 1; j < ee_link_names.size(); ++j)
      {
        // std::cout << "disabling collsion between " << ee_link_names[i] << " and " <<
        // ee_link_names[j] << std::endl;
        collision_matrix.setEntry(ee_link_names[i], ee_link_names[j], true);
      }
    }
  }

  return true;
}

// Mode 9
bool PickManager::gotoPose(const std::string& pose_name)
{
  ROS_INFO_STREAM_NAMED("pick_manager", "Going to pose " << pose_name);
  ros::Duration(1).sleep();
  ros::spinOnce();

  JointModelGroup* arm_jmg = config_->dual_arm_ ? config_->both_arms_ : config_->right_arm_;
  bool check_validity = true;

  if (!manipulation_->moveToSRDFPose(arm_jmg, pose_name, config_->main_velocity_scaling_factor_,
                                     check_validity))
  {
    ROS_ERROR_STREAM_NAMED("pick_manager", "Unable to move to pose");
    return false;
  }
  ROS_INFO_STREAM_NAMED("pick_manager", "Spinning until shutdown requested");
  ros::spin();
  return true;
}

// Mode 25
bool PickManager::testIKSolver()
{
  moveit::core::RobotStatePtr goal_state(
      new moveit::core::RobotState(*manipulation_->getCurrentState()));

  JointModelGroup* arm_jmg = config_->right_arm_;
  Eigen::Affine3d ee_pose = Eigen::Affine3d::Identity();
  ee_pose.translation().x() += 0.3;
  ee_pose.translation().y() += 0.2;
  ee_pose.translation().z() += 1.4;
  ee_pose = ee_pose * Eigen::AngleAxisd(-M_PI / 2.0, Eigen::Vector3d::UnitY());

  visuals_->visual_tools_->publishAxisLabeled(ee_pose, "desired");

  // Transform from world frame to 'gantry' frame
  if (visuals_->isEnabled("generic_bool"))
    ee_pose = goal_state->getGlobalLinkTransform("gantry") * ee_pose;

  for (std::size_t i = 0; i < 100; ++i)
  {
    // Solve IK problem for arm
    std::size_t attempts = 0;  // use default
    double timeout = 0;        // use default
    if (!goal_state->setFromIK(arm_jmg, ee_pose, attempts, timeout))
    {
      ROS_ERROR_STREAM_NAMED("manipulation", "Unable to find arm solution for desired pose");
      return false;
    }

    ROS_INFO_STREAM_NAMED("pick_manager", "SOLVED");

    // Show solution
    visuals_->visual_tools_->publishRobotState(goal_state, rvt::RAND);

    ros::Duration(0.5).sleep();
    goal_state->setToRandomPositions(arm_jmg);
  }

  return true;
}

// Mode 11
bool PickManager::calibrateInCircle()
{
  // Choose which planning group to use
  JointModelGroup* arm_jmg = config_->arm_only_;
  if (!arm_jmg)
  {
    ROS_ERROR_STREAM_NAMED("pick_manager", "No joint model group for arm");
    return false;
  }

  // Get location of camera
  Eigen::Affine3d camera_pose;
  // TODO  manipulation_->getPose(camera_pose, config_->right_camera_frame_);

  // Move camera pose forward away from camera
  Eigen::Affine3d translate_forward = Eigen::Affine3d::Identity();
  translate_forward.translation().x() += 1.0;  // TODO config_->camera_x_translation_from_bin_;
  translate_forward.translation().z() -= 0.15;
  camera_pose = translate_forward * camera_pose;

  // Debug
  visuals_->visual_tools_->publishSphere(camera_pose, rvt::GREEN, rvt::LARGE);
  visuals_->visual_tools_->publishXArrow(camera_pose, rvt::GREEN);

  // Collection of goal positions
  EigenSTL::vector_Affine3d waypoints;

  // Create circle of poses around center
  double radius = 0.05;
  double increment = 2 * M_PI / 4;
  visuals_->visual_tools_->enableBatchPublishing(true);
  for (double angle = 0; angle <= 2 * M_PI; angle += increment)
  {
    // Rotate around circle
    Eigen::Affine3d rotation_transform = Eigen::Affine3d::Identity();
    rotation_transform.translation().z() += radius * cos(angle);
    rotation_transform.translation().y() += radius * sin(angle);

    Eigen::Affine3d new_point = rotation_transform * camera_pose;

    // Convert pose that has x arrow pointing to object, to pose that has z arrow pointing towards
    // object and x out in the grasp dir
    new_point = new_point * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY());
    // new_point = new_point * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ());

    // Debug
    // visuals_->visual_tools_->publishZArrow(new_point, rvt::RED);

    // Translate to custom end effector geometry
    Eigen::Affine3d grasp_pose = new_point * grasp_datas_[arm_jmg]->grasp_pose_to_eef_pose_;
    // visuals_->visual_tools_->publishZArrow(grasp_pose, rvt::PURPLE);
    visuals_->visual_tools_->publishAxis(grasp_pose);

    // Add to trajectory
    waypoints.push_back(grasp_pose);
  }
  visuals_->visual_tools_->triggerBatchPublishAndDisable();

  if (!manipulation_->moveCartesianWaypointPath(arm_jmg, waypoints))
  {
    ROS_ERROR_STREAM_NAMED("pick_manager", "Error executing path");
    return false;
  }

  return true;
}

// Mode 20
bool PickManager::testGraspWidths()
{
  // Test visualization

  const moveit::core::JointModel* joint = robot_model_->getJointModel("jaco2_joint_finger_1");
  double max_finger_joint_limit = manipulation_->getMaxJointLimit(joint);
  double min_finger_joint_limit = manipulation_->getMinJointLimit(joint);

  JointModelGroup* arm_jmg = config_->right_arm_;
  if (false)
  {
    //-----------------------------------------------------------------------------
    //-----------------------------------------------------------------------------
    //-----------------------------------------------------------------------------
    // Send joint position commands

    double joint_position = 0.0;

    while (ros::ok())
    {
      std::cout << std::endl << std::endl;

      ROS_WARN_STREAM_NAMED("apc_manger", "Setting finger joint position " << joint_position);

      // Change fingers
      if (!manipulation_->setEEJointPosition(joint_position, arm_jmg))
      {
        ROS_ERROR_STREAM_NAMED("pick_manager", "Failed to set finger disance");
      }

      // Wait
      ros::Duration(2.0).sleep();
      remote_control_->waitForNextStep("move fingers");

      // Increment the test
      joint_position += (max_finger_joint_limit - min_finger_joint_limit) / 10.0;  //
      if (joint_position > max_finger_joint_limit)
        joint_position = 0.0;
    }
  }
  else
  {
    //-----------------------------------------------------------------------------
    //-----------------------------------------------------------------------------
    //-----------------------------------------------------------------------------
    // Send distance between finger commands

    // Jaco-specific
    double space_between_fingers = grasp_datas_[arm_jmg]->min_finger_width_;

    while (ros::ok())
    {
      std::cout << std::endl << std::endl;

      ROS_WARN_STREAM_NAMED("apc_manger", "Setting finger width distance "
                                              << space_between_fingers);

      // Wait
      ros::Duration(1.0).sleep();
      remote_control_->waitForNextStep("move fingers");

      // Change fingers
      trajectory_msgs::JointTrajectory grasp_posture;
      grasp_datas_[arm_jmg]->fingerWidthToGraspPosture(space_between_fingers, grasp_posture);

      // Send command
      if (!manipulation_->setEEGraspPosture(grasp_posture, arm_jmg))
      {
        ROS_ERROR_STREAM_NAMED("pick_manager", "Failed to set finger width");
      }

      // Increment the test
      space_between_fingers +=
          (grasp_datas_[arm_jmg]->max_finger_width_ - grasp_datas_[arm_jmg]->min_finger_width_) /
          10.0;
      if (space_between_fingers > grasp_datas_[arm_jmg]->max_finger_width_)
      {
        std::cout << std::endl;
        std::cout << "-------------------------------------------------------" << std::endl;
        std::cout << "Wrapping around " << std::endl;
        space_between_fingers = grasp_datas_[arm_jmg]->min_finger_width_;
      }
    }
  }

  ROS_INFO_STREAM_NAMED("pick_manager", "Done testing end effectors");
  return true;
}

void PickManager::processMarkerPose(const geometry_msgs::Pose& pose, bool move)
{
  // NOTE this is in a separate thread, so we should only use visuals_->trajectory_lines_ for
  // debugging!

  // Get pose and visualize
  interactive_marker_pose_ = visuals_->trajectory_lines_->convertPose(pose);

  // Debug
  if (false)
    visuals_->trajectory_lines_->publishZArrow(interactive_marker_pose_, rvt::RED);

  if (!teleoperation_enabled_)
    return;

  move = true;

  // Choose arm
  JointModelGroup* arm_jmg = config_->dual_arm_ ? config_->both_arms_ : config_->right_arm_;

  // Offset ee pose forward, because we are treating interactive marker as a special thing in front
  // of hand
  Eigen::Affine3d ee_pose = interactive_marker_pose_ * config_->teleoperation_offset_;

  // Convert pose to frame of robot base
  const Eigen::Affine3d& world_to_base =
      manipulation_->getCurrentState()->getGlobalLinkTransform("base_link");
  Eigen::Affine3d base_to_desired = world_to_base.inverse() * ee_pose;

  // New Method
  manipulation_->getExecutionInterface()->executePose(base_to_desired, arm_jmg);
}

// Mode 3
void PickManager::insertion()
{
  // Note: The pre-insertion pose is from interactive_marker_pose_
  JointModelGroup* arm_jmg = config_->dual_arm_ ? config_->both_arms_ : config_->right_arm_;

  // Move the the pre-insertion pose
  if (false)
  {
    static const std::string POSE_NAME = "insertion";
    double velocity_scaling_factor = 0.1;
    if (!manipulation_->moveToSRDFPose(arm_jmg, POSE_NAME, velocity_scaling_factor))
    {
      ROS_ERROR_STREAM_NAMED("pick_manager", "Failed to move to starting position");
      return;
    }

    ROS_INFO_STREAM_NAMED("pick_manager", "Preparing to do insertion task");
    ros::Duration(1.0).sleep();
  }

  // Reusable transform from robot base to world. Could be identity. Assumes that it does not change
  Eigen::Affine3d base_to_world =
      manipulation_->getCurrentState()->getGlobalLinkTransform("base_link").inverse();

  // Get current pose - retracted position
  const Eigen::Affine3d desired_world_to_ee =
      manipulation_->getCurrentState()->getGlobalLinkTransform(grasp_datas_[arm_jmg]->parent_link_);

  // Move the desired pose forward from EE base to finger tips
  Eigen::Affine3d desired_world_to_tool =
      desired_world_to_ee * config_->teleoperation_offset_.inverse();

  // pretend that at first we are inserted so that it moves to the correct pre-position
  bool in = false;
  bool achieved_depth = true;  // flag that lets us know if insertion went all the way in

  // Spiral poses
  std::vector<Eigen::Affine3d> poses;
  // allow for alternative insertion poses to be tried if needed
  std::size_t insertion_spiral_pose = 0;
  getSpiralPoses(poses, desired_world_to_tool, config_->insertion_spiral_distance_);
  for (std::size_t i = 0; i < poses.size(); ++i)
  {
    visuals_->visual_tools_->printTransform(poses[i]);
    visuals_->visual_tools_->publishZArrow(poses[i], rvt::RED, rvt::REGULAR);
  }
  ros::Duration(0.5).sleep();
  visuals_->visual_tools_->deleteAllMarkers();

  while (ros::ok())
  {
    tactile_feedback_->recalibrateTactileSensor();
    ros::Duration(0.25).sleep();

    if (in)
    {
      // Move to pre-pose
      std::cout << "-------------------------------------------------------" << std::endl;
      std::cout << "RETRACTING " << std::endl;

      bool direction_in = false;
      bool dummy_achieved_depth;
      if (!manipulation_->executeInsertionClosedLoop(arm_jmg, config_->insertion_distance_ / 2.0,
                                                     desired_world_to_tool, direction_in,
                                                     dummy_achieved_depth))
      {
        ROS_ERROR_STREAM_NAMED("pick_manager", "Unable to execute retract path");
      }

      if (!achieved_depth)
      {
        ROS_INFO_STREAM_NAMED("pick_manager", "Moving to next insertion location "
                                                  << insertion_spiral_pose);
        desired_world_to_tool = poses[insertion_spiral_pose++];

        visuals_->visual_tools_->publishZArrow(desired_world_to_tool, rvt::GREEN, rvt::REGULAR);

        // // Move pose from tips of finger (tool) back to base of EE
        Eigen::Affine3d new_world_to_ee = desired_world_to_tool * config_->teleoperation_offset_;
        // // Convert desired pose from 'world' frame to 'robot base' frame
        Eigen::Affine3d new_base_to_ee = base_to_world * new_world_to_ee;
        manipulation_->getExecutionInterface()->executePose(new_base_to_ee, arm_jmg);
        ros::Duration(0.5).sleep();
      }
    }
    else
    {
      // Move in
      std::cout << std::endl;
      std::cout << "-------------------------------------------------------" << std::endl;
      std::cout << "INSERTING " << std::endl;

      bool direction_in = true;
      if (!manipulation_->executeInsertionClosedLoop(arm_jmg, config_->insertion_distance_,
                                                     desired_world_to_tool, direction_in,
                                                     achieved_depth))
      {
        ROS_ERROR_STREAM_NAMED("pick_manager", "Unable to execute insertion path");
      }

      // Check if insertion didn't go all the way
      if (!achieved_depth)
      {
        ROS_WARN_STREAM_NAMED("pick_manager",
                              "Did not achieve full depth. After retreat will try new location");
      }

      // remote_control_->waitForNextStep("retract");
    }

    ros::Duration(config_->insertion_updown_pause_).sleep();
    // visuals_->visual_tools_->deleteAllMarkers();
    in = !in;
  }

  ROS_INFO_STREAM_NAMED("manipulation", "Finished insertion path");
}

// Mode 1
void PickManager::enableTeleoperation()
{
  ROS_INFO_STREAM_NAMED("pick_manager", "Teleoperation enabled");
  teleoperation_enabled_ = true;
  manipulation_->enableTeleoperation();

  // TEST - measure the offset between blue tool frame and ROS tool frame
  if (false)
  {
    moveit::core::RobotStatePtr before_state(
        new moveit::core::RobotState(*manipulation_->getCurrentState()));

    JointModelGroup* arm_jmg = config_->dual_arm_ ? config_->both_arms_ : config_->right_arm_;

    const Eigen::Affine3d world_to_desired = interactive_marker_pose_;
    const Eigen::Affine3d& world_to_base =
        manipulation_->getCurrentState()->getGlobalLinkTransform("base_link");
    Eigen::Affine3d base_to_desired = world_to_base.inverse() * world_to_desired;

    // New Method
    manipulation_->getExecutionInterface()->executePose(base_to_desired, arm_jmg);

    ros::Duration(1.0).sleep();
    moveit::core::RobotStatePtr after_state(
        new moveit::core::RobotState(*manipulation_->getCurrentState()));

    // Now find the difference between EE poses
    visuals_->visual_tools_->printTransform(
        before_state->getGlobalLinkTransform(grasp_datas_[arm_jmg]->parent_link_));

    visuals_->visual_tools_->printTransform(
        after_state->getGlobalLinkTransform(grasp_datas_[arm_jmg]->parent_link_));
  }
}

// Mode 4
void PickManager::touchControl()
{
  ROS_INFO_STREAM_NAMED("pick_manager", "Responding to touch on finger sensor");
  // manipulation_->enableTeleoperation();

  manipulation_->beginTouchControl();
}

void PickManager::setupInteractiveMarker()
{
  JointModelGroup* arm_jmg = config_->dual_arm_ ? config_->both_arms_ : config_->right_arm_;
  interactive_marker_pose_ =
      manipulation_->getCurrentState()->getGlobalLinkTransform(grasp_datas_[arm_jmg]->parent_link_);

  // Move marker to tip of fingers
  interactive_marker_pose_ = interactive_marker_pose_ * config_->teleoperation_offset_.inverse();

  geometry_msgs::Pose pose_msg = visuals_->visual_tools_->convertPose(interactive_marker_pose_);

  // geometry_msgs::Pose pose =
  // visuals_->visual_tools_->convertPose(config_->grasp_location_transform_);
  remote_control_->initializeInteractiveMarkers(pose_msg);
}

// Mode 7
void PickManager::drawSpiral()
{
  ROS_INFO_STREAM_NAMED("pick_manager", "Drawing spiral");

  JointModelGroup* arm_jmg = config_->dual_arm_ ? config_->both_arms_ : config_->right_arm_;
  const Eigen::Affine3d& world_to_ee =
      manipulation_->getCurrentState()->getGlobalLinkTransform(grasp_datas_[arm_jmg]->parent_link_);

  // Move the pose forward from EE base to finger tips
  Eigen::Affine3d world_to_tool = world_to_ee * config_->teleoperation_offset_.inverse();
  Eigen::Affine3d new_sphere = world_to_tool;

  // Center along x/y axis
  double center_x = world_to_tool.translation().x();
  double center_y = world_to_tool.translation().y();

  double coils = config_->insertion_attempt_radius_;
  double rotation = 3.14;

  // value of theta corresponding to end of last coil
  const double thetaMax = coils * 2 * M_PI;

  // How far to step away from center for each side.
  const double awayStep = config_->insertion_attempt_radius_ / thetaMax;

  // distance between points to plot
  const double chord = config_->insertion_attempt_distance_;

  // For every side, step around and away from center.
  // start at the angle corresponding to a distance of chord
  // away from centre.
  for (double theta = chord / awayStep; theta <= thetaMax;)
  {
    // How far away from center
    double away = awayStep * theta;

    // How far around the center.
    double around = theta + rotation;

    // Convert 'around' and 'away' to X and Y.
    double x = center_x + cos(around) * away;
    double y = center_y + sin(around) * away;

    // Now that you know it, do it.
    new_sphere.translation().x() = x;
    new_sphere.translation().y() = y;
    visuals_->visual_tools_->publishZArrow(new_sphere, rvt::RED, rvt::SMALL);
    visuals_->visual_tools_->printTransform(new_sphere);
    ros::Duration(0.001).sleep();

    // to a first approximation, the points are on a circle
    // so the angle between them is chord/radius
    theta += chord / away;

    if (!ros::ok())
      break;
  }

  /*
    // Draw spiral
    double x, y;
    double distance = config_->insertion_attempt_distance_;
    double radius = config_->insertion_attempt_radius_;
    int rotations = 4;
    while (distance <= rotations * 2.0 * M_PI)
    {
      // Increment
      distance += config_->insertion_attempt_distance_;
      radius += config_->insertion_attempt_radius_;

      // x = rx + (sin(distance) * distance) * radius;
      // y = ry + (sin(distance + (M_PI / 2.0)) * distance + (M_PI / 2)) * radius;

      x = rx + sin(distance) * radius;
      y = ry + sin(distance + M_PI / 2.0) * radius;

      new_sphere.translation().x() = x;
      new_sphere.translation().y() = y;
      visuals_->visual_tools_->publishZArrow(new_sphere, rvt::RED, rvt::SMALL);
      ros::Duration(0.001).sleep();

      // visuals_->visual_tools_->printTransform(new_sphere);

      if (!ros::ok())
        break;
    }
  */
}

void PickManager::getSpiralPoses(std::vector<Eigen::Affine3d>& poses,
                                 const Eigen::Affine3d& center_pose, double distance)
{
  Eigen::Affine3d this_pose = center_pose;
  bool right_down = true;
  std::size_t step = 1;
  std::size_t i;
  const std::size_t iterations = 10;

  for (i = 0; i < iterations; ++i)
  {
    if (right_down)
    {
      // Right
      for (std::size_t j = 0; j < step; ++j)
      {
        this_pose.translation().x() += distance;
        poses.push_back(this_pose);
      }
      // Down
      for (std::size_t j = 0; j < step; ++j)
      {
        this_pose.translation().y() -= distance;
        poses.push_back(this_pose);
      }
    }
    else
    {
      // Left
      for (std::size_t j = 0; j < step; ++j)
      {
        this_pose.translation().x() -= distance;
        poses.push_back(this_pose);
      }
      // Up
      for (std::size_t j = 0; j < step; ++j)
      {
        this_pose.translation().y() += distance;
        poses.push_back(this_pose);
      }
    }
    step++;
    right_down = !right_down;
  }
}

void PickManager::automatedInsertionTest()
{
  JointModelGroup* arm_jmg = config_->right_arm_;

  // Go to pre-grap of knife pose
  static const std::string pose_name = "pickup_location";

  bool check_validity = true;
  if (!manipulation_->moveToSRDFPose(arm_jmg, pose_name, config_->main_velocity_scaling_factor_,
                                     check_validity))
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Unable to go to pregrasp location");
    return;
  }
  manipulation_->getExecutionInterface()->waitForExecution();

  // Get current pose - retracted position
  Eigen::Affine3d desired_world_to_ee =
      manipulation_->getCurrentState()->getGlobalLinkTransform(grasp_datas_[arm_jmg]->parent_link_);

  // Move the desired pose forward from EE base to finger tips
  Eigen::Affine3d desired_world_to_tool =
      desired_world_to_ee * config_->teleoperation_offset_.inverse();

  // Test
  // visuals_->visual_tools_->publishZArrow(desired_world_to_tool, rvt::PURPLE);
  // remote_control_->waitForNextStep("confirm pose");

  // Move in to knife
  bool direction_in = true;
  bool achieved_depth;
  double duration = 10;
  if (!manipulation_->executeInsertionOpenLoopNew(arm_jmg, config_->automated_insertion_distance_,
                                                  duration, desired_world_to_tool, direction_in,
                                                  achieved_depth))
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Unable to go to grasp location");
    return;
  }

  // Close gripper
  remote_control_->waitForNextFullStep("have user manually close gripper");

  // Recalibrate tactile
  tactile_feedback_->recalibrateTactileSensor();

  // Move knife out
  direction_in = false;
  duration = 10;
  if (!manipulation_->executeInsertionOpenLoopNew(arm_jmg, config_->automated_retract_distance_,
                                                  duration, desired_world_to_tool, direction_in,
                                                  achieved_depth))
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Unable to go to pre-grasp location");
    return;
  }

  // Move knife to new location
  duration = 5;
  if (!manipulation_->moveToSRDFPoseNoPlan(arm_jmg, "insertion_location_side", duration))
  {
    ROS_ERROR_STREAM_NAMED("pick_manager", "Unable to move to inseriton location");
    return;
  }
  ros::Duration(6.0).sleep();

  // Get current pose - retracted position
  desired_world_to_ee =
      manipulation_->getCurrentState()->getGlobalLinkTransform(grasp_datas_[arm_jmg]->parent_link_);

  // Move the desired pose forward from EE base to finger tips
  desired_world_to_tool = desired_world_to_ee * config_->teleoperation_offset_.inverse();

  /*
  // Test
  visuals_->visual_tools_->publishZArrow(desired_world_to_tool, rvt::GREEN);
  */

  remote_control_->waitForNextStep("insert");

  // Move knife in
  direction_in = true;
  if (!manipulation_->executeInsertionClosedLoop(arm_jmg, config_->automated_insertion_distance_,
                                                 desired_world_to_tool, direction_in,
                                                 achieved_depth))
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "Unable to insert");
    return;
  }
}

}  // end namespace
