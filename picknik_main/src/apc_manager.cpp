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
   Desc:   Main logic of APC challenge
*/

// Amazon Pick Place Challenge
#include <picknik_main/apc_manager.h>
#include <picknik_main/product_simulator.h>

// Parameter loading
#include <rviz_visual_tools/ros_param_utilities.h>

// MoveIt
#include <moveit/robot_state/conversions.h>
#include <moveit/macros/console_colors.h>

// Boost
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>

namespace picknik_main
{

APCManager::APCManager(bool verbose, std::string order_file_path, bool use_experience, bool autonomous, bool full_autonomous,
                       bool fake_execution, bool fake_perception)
  : nh_private_("~")
  , verbose_(verbose)
  , fake_perception_(fake_perception)
  , skip_homing_step_(true)
  , next_dropoff_location_(0)
  , order_file_path_(order_file_path)
{
  // Warn of fake modes
  if (fake_perception)
    ROS_WARN_STREAM_NAMED("apc_manager","In fake perception mode");
  if (fake_execution)
    ROS_WARN_STREAM_NAMED("apc_manager","In fake execution mode");

  // Load the loader
  robot_model_loader_.reset(new robot_model_loader::RobotModelLoader(ROBOT_DESCRIPTION));

  // Load the robot model
  robot_model_ = robot_model_loader_->getModel(); // Get a shared pointer to the robot

  // Create the planning scene
  planning_scene_.reset(new planning_scene::PlanningScene(robot_model_));

  // Create the planning scene service
  //get_scene_service_ = nh_root_.advertiseService(GET_PLANNING_SCENE_SERVICE_NAME, &APCManager::getPlanningSceneService, this);

  // Create tf transformer
  tf_.reset(new tf::TransformListener(nh_private_));
  // TODO: remove these lines, only an attempt to fix loadPlanningSceneMonitor bug
  ros::spinOnce();
  ros::Duration(0.1).sleep();

  // Load planning scene monitor
  if (!loadPlanningSceneMonitor())
  {
    ROS_ERROR_STREAM_NAMED("apc_manager","Unable to load planning scene monitor");
  }

  // Load multiple visual_tools classes
  visuals_.reset(new Visuals(robot_model_, planning_scene_monitor_));

  // Get package path
  package_path_ = ros::package::getPath(PACKAGE_NAME);
  if( package_path_.empty() )
    ROS_FATAL_STREAM_NAMED("product", "Unable to get " << PACKAGE_NAME << " package path" );

  // Load shelf
  shelf_.reset(new ShelfObject(visuals_, rvt::BROWN, "shelf_0"));
  if (!shelf_->initialize(package_path_, nh_private_))
  {
    ROS_ERROR_STREAM_NAMED("apc_manager","Unable to load shelf");
  }

  // Decide where to publish status text
  status_position_ = shelf_->getBottomRight();
  status_position_.translation().x() = 0.25;
  status_position_.translation().y() += shelf_->getWidth() * 0.5;
  status_position_.translation().z() += shelf_->getHeight() * 1.1;

  // Load the remote control for dealing with GUIs
  remote_control_.reset(new RemoteControl(verbose, nh_private_, this));
  remote_control_->setAutonomous(autonomous);
  remote_control_->setFullAutonomous(full_autonomous);

  // Load manipulation data for our robot
  config_.reset(new ManipulationData());
  config_->load(robot_model_, fake_execution);

  // Load grasp data specific to our robot
  grasp_datas_[config_->right_arm_].reset(new moveit_grasps::GraspData(nh_private_, config_->right_hand_name_, robot_model_));
  if (config_->dual_arm_)
    grasp_datas_[config_->left_arm_].reset(new moveit_grasps::GraspData(nh_private_, config_->left_hand_name_, robot_model_));

  // Create manipulation manager
  manipulation_.reset(new Manipulation(verbose_, visuals_, planning_scene_monitor_, config_, grasp_datas_,
                                       remote_control_, package_path_, shelf_, use_experience, fake_execution));

  // Load perception layer
  perception_interface_.reset(new PerceptionInterface(verbose_, visuals_, shelf_, config_, tf_, nh_private_));

  // Load planning scene manager
  planning_scene_manager_.reset(new PlanningSceneManager(verbose, visuals_, shelf_));
  planning_scene_manager_->displayShelfWithOpenBins();

  // Visualize detailed shelf
  visuals_->visualizeDisplayShelf(shelf_);

  // Allow collisions between frame of robot and floor
  allowCollisions();

  ROS_INFO_STREAM_NAMED("apc_manager","APCManager Ready.");
}

bool APCManager::checkSystemReady(bool remove_from_shelf)
{
  std::cout << std::endl;
  std::cout << std::endl;
  std::cout << "-------------------------------------------------------" << std::endl;
  ROS_INFO_STREAM_NAMED("apc_manager","Starting system ready check:");

  // Check joint model groups, assuming we are the jaco arm
  if (config_->right_arm_->getVariableCount() < 6 || config_->right_arm_->getVariableCount() > 7)
  {
    ROS_FATAL_STREAM_NAMED("apc_manager","Incorrect number of joints for group " << config_->right_arm_->getName()
                           << ", joints: " << config_->right_arm_->getVariableCount());
    return false;
  }
  const robot_model::JointModelGroup* ee_jmg = grasp_datas_[config_->right_arm_]->ee_jmg_;
  if (ee_jmg->getVariableCount() > 6)
  {
    ROS_FATAL_STREAM_NAMED("apc_manager","Incorrect number of joints for group " << ee_jmg->getName() << ", joints: "
                           << ee_jmg->getVariableCount());
    return false;
  }

  // Check trajectory execution manager
  if( !manipulation_->getExecutionInterface()->checkExecutionManager() )
  {
    ROS_FATAL_STREAM_NAMED("apc_manager","Trajectory controllers unable to connect");
    return false;
  }

  // Check Perception
  if (!fake_perception_)
  {
    perception_interface_->isPerceptionReady();
  }

  // Choose which planning group to use
  const robot_model::JointModelGroup* arm_jmg = config_->dual_arm_ ? config_->both_arms_ : config_->right_arm_;

  // Check robot state valid
  if (remove_from_shelf)
  {
    planning_scene_manager_->displayShelfAsWall(); // Reduce collision model to simple wall that prevents Robot from hitting shelf
    while (ros::ok() && !manipulation_->fixCurrentCollisionAndBounds(arm_jmg))
    {
      // Show the current state just for the heck of it
      publishCurrentState();

      ros::Duration(0.5).sleep();
    }
  }

  // Check robot calibrated
  // TODO

  // Check gantry calibrated
  // TODO

  // Check end effectors calibrated
  // TODO

  ROS_INFO_STREAM_NAMED("apc_manager","System ready check COMPLETE");
  std::cout << "-------------------------------------------------------" << std::endl;
  return true;
}

// Mode 1
bool APCManager::mainOrderProcessor(std::size_t order_start, std::size_t jump_to, std::size_t num_orders)
                          
{
  // Load JSON file
  loadShelfContents(order_file_path_);

  // Generate random product poses and visualize the shelf
  createRandomProductPoses();

  return runOrder(order_start, jump_to, num_orders);
}

bool APCManager::runOrder(std::size_t order_start, std::size_t jump_to, std::size_t num_orders)                          
{
  // Decide how many products to pick
  if (num_orders == 0)
    num_orders = orders_.size();

  // Grasps things
  for (std::size_t i = order_start; i < num_orders; ++i)
  {
    if (!ros::ok())
      return false;

    std::cout << std::endl << MOVEIT_CONSOLE_COLOR_BROWN;
    std::cout << "=======================================================" << std::endl;
    std::cout << "Starting order " << i << std::endl;
    std::cout << "=======================================================";
    std::cout << MOVEIT_CONSOLE_COLOR_RESET << std::endl;

    // Check every product if system is still ready
    if (!checkSystemReady())
      return false;

    // Clear old grasp markers
    visuals_->grasp_markers_->deleteAllMarkers();

    WorkOrder &work_order = orders_[i];

    if (!graspObjectPipeline(work_order, verbose_, jump_to))
    {
      ROS_WARN_STREAM_NAMED("apc_manager","An error occured in last product order.");

      if (false)
      {
        // remote_control_->setAutonomous(false);
        // remote_control_->setFullAutonomous(false);
        // remote_control_->waitForNextStep();
      }
      else
      {
        ROS_ERROR_STREAM_NAMED("apc_manager","Shutting down for debug purposes only (it could continue on)");
        return false;
      }
    }

    ROS_INFO_STREAM_NAMED("apc_manager","Cleaning up planning scene");

    // Unattach from EE
    visuals_->visual_tools_->cleanupACO( work_order.product_->getCollisionName() ); // use unique name
    // Delete from planning scene the product
    visuals_->visual_tools_->cleanupCO( work_order.product_->getCollisionName() ); // use unique name

    // Reset markers for next loop
    visuals_->visual_tools_->deleteAllMarkers();

    // Show shelf with remaining products
    visuals_->visualizeDisplayShelf(shelf_);
  }

  statusPublisher("Finished");

  // Show experience database results
  manipulation_->printExperienceLogs();

  return true;
}

bool APCManager::graspObjectPipeline(WorkOrder work_order, bool verbose, std::size_t jump_to)
{
  // Error check
  if (!work_order.product_ || !work_order.bin_)
  {
    ROS_ERROR_STREAM_NAMED("apc_manager","Invalid pointers to product or bin in work_order");
    return false;
  }

  const robot_model::JointModelGroup* arm_jmg;
  bool execute_trajectory = true;

  moveit::core::RobotStatePtr current_state = manipulation_->getCurrentState();

  // Variables
  std::vector<moveit_grasps::GraspCandidatePtr> grasp_candidates;
  moveit::core::RobotStatePtr pre_grasp_state(new moveit::core::RobotState(*current_state)); // Allocate robot states
  moveit::core::RobotStatePtr the_grasp_state(new moveit::core::RobotState(*current_state)); // Allocate robot states
  moveit_msgs::RobotTrajectory approach_trajectory_msg;

  if (!remote_control_->getAutonomous())
  {
    visuals_->start_state_->publishRobotState(current_state, rvt::GREEN);
    ROS_INFO_STREAM_NAMED("apc_manager","Waiting for remote control to be triggered to start");
  }

  // Jump to a particular step in the manipulation pipeline
  std::size_t step = jump_to;

  // Don't move to initial position
  if (skip_homing_step_ && step == 0)
    step = 1;

  while(ros::ok())
  {

    if (!remote_control_->getAutonomous())
    {
      remote_control_->waitForNextStep();
    }
    else
    {
      std::cout << std::endl;
      std::cout << std::endl;
      std::cout << "Running step: " << step << std::endl;
    }

    switch (step)
    {
      // #################################################################################################################
      case 0: statusPublisher("Moving to initial position");

        // Clear the temporary purple robot state image
        visuals_->visual_tools_->hideRobot();

        // Set planning scene
        planning_scene_manager_->displayShelfAsWall(); // Reduce collision model to simple wall that prevents Robot from hitting shelf

        // Move
        if (!moveToStartPosition())
        {
          ROS_ERROR_STREAM_NAMED("apc_manager","Unable to move to initial position");
          return false;
        }

        break;

        // #################################################################################################################
      case 1: statusPublisher("Open end effectors");

        // Set planning scene
        planning_scene_manager_->displayShelfWithOpenBins();

        // Open hand
        if (!manipulation_->openEndEffectors(true))
        {
          ROS_ERROR_STREAM_NAMED("apc_manager","Unable to open end effectors");
          return false;
        }

        //break;
        step++;

        // #################################################################################################################
      case 2: statusPublisher("Finding location of product " + work_order.product_->getName() + " from " + work_order.bin_->getName());

        // Set planning scene
        planning_scene_manager_->displayShelfOnlyBin( work_order.bin_->getName() );

        // Fake perception of product
        if (!fake_perception_)
        {
          // Move camera to desired bin to get pose of product
          if (!perceiveObject(work_order, verbose))
          {
            ROS_ERROR_STREAM_NAMED("apc_manager","Unable to get object pose");
            return false;
          }
        }

        break;

        // #################################################################################################################
      case 3: statusPublisher("Get grasp for product " + work_order.product_->getName() + " from " + work_order.bin_->getName());

        // Set planning scene
        planning_scene_manager_->displayShelfOnlyBin( work_order.bin_->getName() );

        // Choose which arm to use
        arm_jmg = manipulation_->chooseArm(work_order.product_->getWorldPose(shelf_, work_order.bin_));

        // Allow fingers to touch object
        manipulation_->allowFingerTouch(work_order.product_->getCollisionName(), arm_jmg);

        // Generate and chose grasp
        if (!manipulation_->chooseGrasp(work_order, arm_jmg, grasp_candidates, verbose))
        {
          ROS_ERROR_STREAM_NAMED("apc_manager","No grasps found");
          return false;
        }

        // Get the pre and post grasp states
        grasp_candidates.front()->getPreGraspState(pre_grasp_state);
        grasp_candidates.front()->getGraspStateOpen(the_grasp_state);

        // Visualize
        visuals_->start_state_->publishRobotState(pre_grasp_state, rvt::GREEN);
        visuals_->goal_state_->publishRobotState(the_grasp_state, rvt::ORANGE);
        break;

        // #################################################################################################################
      case 4: //statusPublisher("Get pre-grasp by generateApproachPath()");

        // Set planning scene
        // planning_scene_manager_->displayShelfOnlyBin( work_order.bin_->getName() );

        // // Hide the purple robot
        // visuals_->visual_tools_->hideRobot();

        // if (!manipulation_->generateApproachPath(grasp_candidates.front(), approach_trajectory_msg, pre_grasp_state, the_grasp_state, verbose))
        // {
        //   ROS_ERROR_STREAM_NAMED("apc_manager","Unable to generate straight approach path");
        //   return false;
        // }

        // Visualize trajectory in Rviz display
        //visuals_->visual_tools_->publishTrajectoryPath(approach_trajectory_msg, current_state, wait_for_trajetory);
        //break;

        step++;

        // #################################################################################################################
      case 5: // Not implemented

        //break
        step++;

        // #################################################################################################################
      case 6: statusPublisher("Moving to pre-grasp position");

        // Set planning scene
        // TODO: add this back but sometimes do not if the pregrasp is so close that it would be in collision with wall
        //planning_scene_manager_->displayShelfAsWall();

        current_state = manipulation_->getCurrentState();
        manipulation_->setStateWithOpenEE(true, current_state);

        if (!manipulation_->move(current_state, pre_grasp_state, arm_jmg, config_->main_velocity_scaling_factor_,
                                 verbose, execute_trajectory))
        {
          ROS_ERROR_STREAM_NAMED("apc_manager","Unable to plan to pre-grasp position");
          return false;
        }
        break;

        // #################################################################################################################
      case 7: statusPublisher("Cartesian move to the-grasp position");

        // Set planning scene
        planning_scene_manager_->displayShelfOnlyBin( work_order.bin_->getName() );

        // Clear old grasp markers
        visuals_->grasp_markers_->deleteAllMarkers();

        // Visualize trajectory in Rviz display
        //current_state = manipulation_->getCurrentState();
        //visuals_->visual_tools_->publishTrajectoryPath(approach_trajectory_msg, current_state, wait_for_trajetory);

        // Run
        // if( !manipulation_->getExecutionInterface()->executeTrajectory(approach_trajectory_msg) )
        // {
        //   ROS_ERROR_STREAM_NAMED("apc_manager","Failed to move to the-grasp position");
        //   return false;
        // }

        // Execute straight forward
        // if (!manipulation_->executeRetreatPath(arm_jmg, grasp_candidates.front()->grasp_data_->approach_distance_desired_, false))
        // {
        //   ROS_ERROR_STREAM_NAMED("apc_manager","Unable to move through approach path");
        //   return false;
        // }

        // Execute straight forward
        if (!manipulation_->executeSavedCartesianPath(grasp_candidates.front(), moveit_grasps::APPROACH))
        {
          ROS_ERROR_STREAM_NAMED("apc_manager","Unable to move through approach path");
          return false;
        }
        
        // Wait
        ROS_INFO_STREAM_NAMED("apc_manager","Waiting " << config_->wait_before_grasp_ << " seconds before grasping");
        ros::Duration(config_->wait_after_grasp_).sleep();

        break;

        // #################################################################################################################
      case 8: statusPublisher("Grasping");

        // Set planning scene
        planning_scene_manager_->displayShelfOnlyBin( work_order.bin_->getName() );

        // Cleanup grasp generator makers
        visuals_->start_state_->deleteAllMarkers(); // clear all old markers

        // Close EE
        if (!manipulation_->openEndEffectorWithVelocity(false, arm_jmg))
        {
          ROS_WARN_STREAM_NAMED("apc_manager","Unable to close end effector");
          //return false;
        }

        // Attach collision object
        if (!attachProduct(work_order.product_, arm_jmg))
          ROS_ERROR_STREAM_NAMED("apc_manager","Unable to attach collision object");

        ROS_INFO_STREAM_NAMED("apc_manager","Waiting " << config_->wait_after_grasp_ << " seconds after grasping");
        ros::Duration(config_->wait_after_grasp_).sleep();

        break;

        // #################################################################################################################
      case 9: statusPublisher("Lifting product up slightly");

        // Set planning scene
        //planning_scene_manager_->displayShelfOnlyBin( work_order.bin_->getName() );

        // Lift up
        if (!manipulation_->executeSavedCartesianPath(grasp_candidates.front(), moveit_grasps::LIFT))
        {
          ROS_ERROR_STREAM_NAMED("apc_manager","Unable to execute lift path after grasping");
          return false;
        }

        // if (!manipulation_->executeVerticlePath(arm_jmg, grasp_datas_[arm_jmg]->lift_distance_desired_, 
        //                                         config_->lift_velocity_scaling_factor_, true /* up */ ))
        // {
        //ROS_ERROR_STREAM_NAMED("apc_manager","Unable to execute lift path after grasping");
        //   return false;
        // }
        break;

        // #################################################################################################################
      case 10: statusPublisher("Moving back to pre-grasp position (retreat path)");

        // Set planning scene
        //planning_scene_manager_->displayShelfOnlyBin( work_order.bin_->getName() );

        // Retreat backwards using new IK solution
        // if (!manipulation_->executeRetreatPath(arm_jmg, grasp_datas_[arm_jmg]->retreat_distance_desired_, true /*retreat*/))
        // {
        //   ROS_ERROR_STREAM_NAMED("apc_manager","Unable to execute retrieval path");
        //   return false;
        // }

        // Retreat backwards using pre-computed trajectory
        if (!manipulation_->executeSavedCartesianPath(grasp_candidates.front(), moveit_grasps::RETREAT))
        {
          ROS_ERROR_STREAM_NAMED("apc_manager","Unable to execute retreaval path");
          return false;
        }

        break;

        // #################################################################################################################
      case 11: statusPublisher("Placing product in bin");

        // Set planning scene
        //planning_scene_manager_->displayShelfAsWall();

        if (!placeObjectInGoalBin(arm_jmg))
        {
          ROS_ERROR_STREAM_NAMED("apc_manager","Unable to move object to goal bin");
          return false;
        }

        break;

        // #################################################################################################################
      case 12: statusPublisher("Releasing product");

        // Set planning scene
        // planning_scene_manager_->displayShelfAsWall();

        if (!manipulation_->openEndEffectorWithVelocity(true, arm_jmg))
        {
          ROS_ERROR_STREAM_NAMED("apc_manager","Unable to close end effector");
          return false;
        }

        if (!liftFromGoalBin(arm_jmg))
        {
          ROS_ERROR_STREAM_NAMED("apc_manager","Unable to lift up from goal bin");
          return false;
        }

        break;

        // #################################################################################################################
      default:
        ROS_INFO_STREAM_NAMED("apc_manager","Manipulation pipeline finished, pat yourself on the back!");

        // Remove product from shelf
        shelf_->deleteProduct(work_order.bin_, work_order.product_);

        return true;

    } // end switch
    step++;
  } // end for

  return true;
}

// Mode 19
bool APCManager::trainExperienceDatabase()
{
  ROS_ERROR_STREAM_NAMED("apc_manager","disabled");
  /*
  // Create learning pipeline for training the experience database
  bool use_experience = false;
  learning_.reset(new LearningPipeline(verbose_, visuals_,
  planning_scene_monitor_,
  shelf_, use_experience));

  ROS_INFO_STREAM_NAMED("apc_manager","Training experience database");
  learning_->generateTrainingGoals(shelf_);
  */

  return true;
}

// Mode 8
bool APCManager::testEndEffectors()
{
  // Test visualization
  statusPublisher("Testing open close visualization of EE");
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
      manipulation_->setStateWithOpenEE(open, current_state);
      visuals_->visual_tools_->publishRobotState(current_state);

      // Close right and optionally right EE
      manipulation_->openEndEffectorWithVelocity(open, config_->right_arm_);
      if (config_->dual_arm_)
        manipulation_->openEndEffectorWithVelocity(open, config_->left_arm_);

      ros::Duration(4.0).sleep();
    }
    else
    {
      std::cout << "Showing open EE of state " << std::endl;
      open = true;
      manipulation_->setStateWithOpenEE(open, current_state);
      visuals_->visual_tools_->publishRobotState(current_state);

      // Open right and optionally right EE
      manipulation_->openEndEffectorWithVelocity(open, config_->right_arm_);
      if (config_->dual_arm_)
        manipulation_->openEndEffectorWithVelocity(open, config_->left_arm_);
      ros::Duration(4.0).sleep();
    }
    ++i;
  }

  ROS_INFO_STREAM_NAMED("apc_manager","Done testing end effectors");
  return true;
}

// Mode 13
bool APCManager::testVisualizeShelf()
{
  ROS_INFO_STREAM_NAMED("apc_manager","Testing all planning scene manager modes");

  // Load JSON file
  loadShelfContents(order_file_path_);

  // Generate random product poses and visualize the shelf
  createRandomProductPoses();

  //planning_scene_manager_->testAllModes();
  ros::spin();
  return true;
}

// Mode 5
bool APCManager::testUpAndDown()
{
  double lift_distance_desired = 0.5;

  // Setup planning scene
  planning_scene_manager_->displayEmptyShelf();

  // Test
  statusPublisher("Testing up and down calculations");
  std::size_t i = 0;
  while (ros::ok())
  {
    std::cout << std::endl << std::endl;
    if (i % 2 == 0)
    {
      std::cout << "Moving up --------------------------------------" << std::endl;
      manipulation_->executeVerticlePath(config_->right_arm_, lift_distance_desired, config_->lift_velocity_scaling_factor_, true);
      if (config_->dual_arm_)
        manipulation_->executeVerticlePath(config_->left_arm_, lift_distance_desired, config_->lift_velocity_scaling_factor_, true);
      ros::Duration(1.0).sleep();
    }
    else
    {
      std::cout << "Moving down ------------------------------------" << std::endl;
      manipulation_->executeVerticlePath(config_->right_arm_, lift_distance_desired, config_->lift_velocity_scaling_factor_, false);
      if (config_->dual_arm_)
        manipulation_->executeVerticlePath(config_->left_arm_, lift_distance_desired, config_->lift_velocity_scaling_factor_, false);
      ros::Duration(1.0).sleep();
    }
    ++i;
  }

  ROS_INFO_STREAM_NAMED("apc_manager","Done testing up and down");
  return true;
}

// Mode 20
bool APCManager::testInAndOut()
{
  // Set planning scene
  planning_scene_manager_->displayEmptyShelf();

  double approach_distance_desired = 1.0;

  // Test
  statusPublisher("Testing in and out calculations");
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
        if (!manipulation_->executeRetreatPath(config_->left_arm_, approach_distance_desired, false))
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

  ROS_INFO_STREAM_NAMED("apc_manager","Done testing in and out");
  return true;
}

// Mode 7
bool APCManager::testShelfLocation()
{
  static const double SAFETY_PADDING = -0.23; // Amount to prevent collision with shelf edge
  Eigen::Affine3d ee_pose;

  // Set EE as closed so that we can touch the tip easier
  manipulation_->openEndEffectorWithVelocity(false, config_->right_arm_);
  if (config_->dual_arm_)
    manipulation_->openEndEffectorWithVelocity(false, config_->left_arm_);

  // Reduce collision world to simple
  planning_scene_manager_->displayShelfAsWall();

  // Loop through each bin
  for (BinObjectMap::const_iterator bin_it = shelf_->getBins().begin(); bin_it != shelf_->getBins().end(); bin_it++)
  {
    if (!ros::ok())
      return false;

    ROS_INFO_STREAM_NAMED("apc_manager","Testing bin location of " << bin_it->first);

    // Move to far left front corner of bin
    ee_pose = shelf_->getBottomRight() * bin_it->second->getBottomRight(); // convert to world frame
    ee_pose.translation().y() += bin_it->second->getWidth();

    const robot_model::JointModelGroup* arm_jmg = manipulation_->chooseArm(ee_pose);

    ee_pose.translation().x() += SAFETY_PADDING - grasp_datas_[arm_jmg]->finger_to_palm_depth_;

    // Convert pose that has x arrow pointing to object, to pose that has z arrow pointing towards object and x out in the grasp dir
    ee_pose = ee_pose * Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY());
    ee_pose = ee_pose * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ());

    // Translate to custom end effector geometry
    ee_pose = ee_pose * grasp_datas_[arm_jmg]->grasp_pose_to_eef_pose_;

    // Visual debug
    visuals_->visual_tools_->publishSphere(ee_pose);

    if (!manipulation_->moveEEToPose(ee_pose, config_->main_velocity_scaling_factor_, arm_jmg))
    {
      ROS_ERROR_STREAM_NAMED("apc_manager","Failed to move arm to desired shelf location");
      continue;
    }

    // std::cout << std::endl;
    // std::cout << std::endl;
    // std::cout << "Waiting before moving forward" << std::endl;
    // remote_control_->waitForNextStep();


    // ROS_INFO_STREAM_NAMED("apc_manager","Moving forward");
    // bool retreat = false;
    // double desired_approach_distance = 0.02;
    // if (!manipulation_->executeRetreatPath(arm_jmg, desired_approach_distance, retreat))
    // {
    //   ROS_ERROR_STREAM_NAMED("apc_manager","Failed to move forward " << desired_approach_distance);
    // }

    // std::cout << std::endl;
    // std::cout << std::endl;
    // std::cout << "Waiting before retreating" << std::endl;
    // remote_control_->waitForNextStep();


    // retreat = true;
    // if (!manipulation_->executeRetreatPath(arm_jmg, desired_approach_distance, retreat))
    // {
    //   ROS_ERROR_STREAM_NAMED("apc_manager","Failed to move backwards " << desired_approach_distance);
    // }

    remote_control_->waitForNextStep();
  } // end for

  ROS_INFO_STREAM_NAMED("apc_manager","Done testing shelf location");
  return true;
}

// Mode 22
bool APCManager::testApproachLiftRetreat()
{
  // Load JSON file
  loadShelfContents(order_file_path_);

  // Generate random product poses and visualize the shelf
  createRandomProductPoses();

  Eigen::Affine3d ee_pose;
  bool verbose = true;

  // Grasps things in the work order
  for (std::size_t i = 0; i < orders_.size(); ++i)
  {
    if (!ros::ok())
      return false;

    ROS_INFO_STREAM_NAMED("apc_manager","Starting order " << i);
    WorkOrder& work_order = orders_[i];

    // Set a fake pose of object
    //perceiveObjectFake(work_order);

    // Choose which arm to use
    const robot_model::JointModelGroup* arm_jmg = manipulation_->chooseArm(work_order.product_->getWorldPose(shelf_, work_order.bin_));

    // Allow fingers to touch object
    manipulation_->allowFingerTouch(work_order.product_->getCollisionName(), arm_jmg);

    std::size_t repeat_loops = 1; // a debug mode
    for (std::size_t i = 0; i < repeat_loops; ++i)
    {
      if (!ros::ok())
        break;

      std::vector<moveit_grasps::GraspCandidatePtr> grasp_candidates;

      // Generate and chose grasp
      if (!manipulation_->chooseGrasp(work_order, arm_jmg, grasp_candidates, verbose))
      {
        ROS_ERROR_STREAM_NAMED("apc_manager","No grasps found for " << work_order.product_->getName());
      }
    }

    //ROS_INFO_STREAM_NAMED("temp","ending after first order");
    //return true; // TEMP ender
  } // end for

  ROS_INFO_STREAM_NAMED("apc_manager","Done testing cartesian path");
  return true;
}

// Mode 14
bool APCManager::getSRDFPose()
{
  ROS_DEBUG_STREAM_NAMED("apc_manager","Get SRDF pose");

  // Choose which planning group to use
  const robot_model::JointModelGroup* arm_jmg = config_->dual_arm_ ? config_->both_arms_ : config_->right_arm_;
  const std::vector<const moveit::core::JointModel*> joints = arm_jmg->getJointModels();

  while(ros::ok())
  {
    ROS_INFO_STREAM("SDF Code for joint values pose:\n");

    // Get current state after grasping
    moveit::core::RobotStatePtr current_state = manipulation_->getCurrentState();

    // Check if current state is valid
    //manipulation_->fixCurrentCollisionAndBounds(arm_jmg);

    // Output XML
    std::cout << "<group_state name=\"\" group=\"" << arm_jmg->getName() << "\">\n";
    for (std::size_t i = 0; i < joints.size(); ++i)
    {
      std::cout << "  <joint name=\"" << joints[i]->getName() <<"\" value=\""
                << current_state->getJointPositions(joints[i])[0] << "\" />\n";
    }
    std::cout << "</group_state>\n\n\n\n";

    ros::Duration(4.0).sleep();
  }
  return true;
}

// Mode 3
bool APCManager::testGoalBinPose()
{
  const robot_model::JointModelGroup* arm_jmg = config_->dual_arm_ ? config_->both_arms_ : config_->right_arm_;

  // Set planning scene
  planning_scene_manager_->displayShelfWithOpenBins();

  // Create locations if necessary
  generateGoalBinLocations();

  // Test every goal dropoff location
  for (std::size_t i = 0; i < dropoff_locations_.size(); ++i)
  {
    // Close end effector
    if (!manipulation_->openEndEffectorWithVelocity(false, arm_jmg))
    {
      ROS_ERROR_STREAM_NAMED("apc_manager","Unable to close end effector");
      return false;
    }

    // Go to dropoff position
    if (!placeObjectInGoalBin(config_->right_arm_))
    {
      ROS_ERROR_STREAM_NAMED("apc_manager","Failed to place object in goal bin");
      return false;
    }

    // Open end effector
    if (!manipulation_->openEndEffectorWithVelocity(true, arm_jmg))
    {
      ROS_ERROR_STREAM_NAMED("apc_manager","Unable to open end effector");
      return false;
    }

    // Lift
    if (!liftFromGoalBin(arm_jmg))
    {
      ROS_ERROR_STREAM_NAMED("apc_manager","Unable to lift up from goal bin");
      return false;
    }

    // Go home
    moveToStartPosition(arm_jmg);
  }

  ROS_INFO_STREAM_NAMED("apc_manager","Done going to goal bin pose");
  return true;
}

// Mode 15
bool APCManager::testInCollision()
{
  // Choose which planning group to use
  const robot_model::JointModelGroup* arm_jmg = config_->dual_arm_ ? config_->both_arms_ : config_->right_arm_;

  while (ros::ok())
  {
    manipulation_->fixCurrentCollisionAndBounds(arm_jmg);
    ros::Duration(1).sleep();
  }

  ROS_INFO_STREAM_NAMED("apc_manager","Done checking if in collision");
  return true;
}

// Mode 6
bool APCManager::testRandomValidMotions()
{
  planning_scene_manager_->displayShelfWithOpenBins();

  // Allow collision between Jacob and bottom for most links
  {
    planning_scene_monitor::LockedPlanningSceneRW scene(planning_scene_monitor_); // Lock planning scene

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
      ROS_DEBUG_STREAM_NAMED("apc_manager","Attempt " << i << " to plan to a random location");

      // Create start
      moveit::core::RobotStatePtr current_state = manipulation_->getCurrentState();

      // Create goal
      moveit::core::RobotStatePtr goal_state(new moveit::core::RobotState(*current_state));

      // Choose arm
      const robot_model::JointModelGroup* arm_jmg = config_->right_arm_;
      if (config_->dual_arm_)
        if (visuals_->visual_tools_->iRand(0,1) == 0)
          arm_jmg = config_->left_arm_;

      goal_state->setToRandomPositions(arm_jmg);

      // Check if random goal state is valid
      bool collision_verbose = false;
      if (manipulation_->checkCollisionAndBounds(current_state, goal_state, collision_verbose))
      {
        // Plan to this position
        bool verbose = true;
        bool execute_trajectory = true;
        if (manipulation_->move(current_state, goal_state, arm_jmg, config_->main_velocity_scaling_factor_, verbose,
                                execute_trajectory))
        {
          ROS_INFO_STREAM_NAMED("apc_manager","Planned to random valid state successfullly");
        }
        else
        {
          ROS_ERROR_STREAM_NAMED("apc_manager","Failed to plan to random valid state");
          return false;
        }
      }
    }
    ROS_ERROR_STREAM_NAMED("apc_manager","Unable to find random valid state after " << MAX_ATTEMPTS << " attempts");

    ros::Duration(1).sleep();
  } // while

  ROS_INFO_STREAM_NAMED("apc_manager","Done planning to random valid");
  return true;
}

bool APCManager::createRandomProductPoses()
{
  // Generate random product poses and visualize the shelf
  bool product_simulator_verbose = false;
  ProductSimulator product_simulator(product_simulator_verbose, visuals_, planning_scene_monitor_);
  return product_simulator.generateRandomProductPoses(shelf_);
}

// Mode 4
bool APCManager::testCameraPositions()
{
  // Load JSON file
  loadShelfContents(order_file_path_);

  // Generate random product poses and visualize the shelf
  createRandomProductPoses();

  std::size_t bin_skipper = 0;
  for (BinObjectMap::const_iterator bin_it = shelf_->getBins().begin(); bin_it != shelf_->getBins().end(); bin_it++)
  {
    bool process_all_bins = false;
    if (process_all_bins)
    {
      remote_control_->waitForNextStep();
    }
    else
    {
      bin_skipper++;
      if (bin_skipper != 4) // only do bin 4 (bin_D)
        continue;
    }

    if (!ros::ok())
      return true;

    // Get bin and product
    const BinObjectPtr bin = bin_it->second;
    if (bin->getProducts().size() == 0) // Check if there are any objects to get
    {
      ROS_ERROR_STREAM_NAMED("apc_manager","No products in bin");
      return false;
    }
    const ProductObjectPtr product = bin->getProducts()[0]; // Choose first object
    WorkOrder work_order(bin, product);

    // Get pose
    Eigen::Affine3d object_pose;
    bool verbose = true;
    if (!perceiveObject(work_order, verbose))
    {
      ROS_ERROR_STREAM_NAMED("apc_manager","Failed to get product");
      return false;
    }

    // Wait before going to next bin
    ros::Duration(1.0).sleep();
  }

  ROS_INFO_STREAM_NAMED("apc_manager","Done moving to each bin");
  return true;
}

// Mode 10
bool APCManager::calibrateCamera()
{
  ROS_DEBUG_STREAM_NAMED("apc_manager","Calibrating camera");

  // Display planning scene
  planning_scene_manager_->displayShelfWithOpenBins();

  // Close fingers
  if (!manipulation_->openEndEffectors(false))
  {
    ROS_ERROR_STREAM_NAMED("apc_manager","Unable to close end effectors");
    return false;
  }

  // Choose which planning group to use
  const robot_model::JointModelGroup* arm_jmg = config_->dual_arm_ ? config_->both_arms_ : config_->right_arm_;

  // Start playing back file
  std::string file_path;
  const std::string file_name = "calibration_trajectory";
  manipulation_->getFilePath(file_path, file_name);

  //  if (!manipulation_->playbackTrajectoryFromFileInteractive(file_path, arm_jmg, config_->calibration_velocity_scaling_factor_))
  if (!manipulation_->playbackTrajectoryFromFile(file_path, arm_jmg, config_->calibration_velocity_scaling_factor_))
  {
    ROS_ERROR_STREAM_NAMED("apc_manager","Unable to playback " << file_name);
    return false;
  }

  ROS_INFO_STREAM_NAMED("apc_manager","Done calibrating camera");
  return true;
}

// Mode 9
bool APCManager::recordCalibrationTrajectory()
{
  ROS_INFO_STREAM_NAMED("apc_manager","Recoding calibration trajectory");

  std::string file_path;
  const std::string file_name = "calibration_trajectory";
  manipulation_->getFilePath(file_path, file_name);

  // Wait
  remote_control_->waitForNextStep("start recording calibration trajectory");

  // Start recording
  manipulation_->recordTrajectoryToFile(file_path);

  ROS_INFO_STREAM_NAMED("apc_manager","Done recording calibration trajectory");

  return true;
}

// Mode 2
bool APCManager::testGoHome()
{
  ROS_DEBUG_STREAM_NAMED("apc_manager","Going home");

  // Choose which planning group to use
  const robot_model::JointModelGroup* arm_jmg = config_->dual_arm_ ? config_->both_arms_ : config_->right_arm_;
  moveToStartPosition(arm_jmg);
  return true;
}

// Mode 16
bool APCManager::testGraspGenerator()
{
  // Load JSON file
  loadShelfContents(order_file_path_);

  // Generate random product poses and visualize the shelf
  createRandomProductPoses();

  // Benchmark runtime
  ros::Time start_time;
  start_time = ros::Time::now();

  // Variables
  moveit::core::RobotStatePtr current_state = manipulation_->getCurrentState();
  moveit::core::RobotStatePtr the_grasp_state(new moveit::core::RobotState(*current_state)); // Allocate robot states
  Eigen::Affine3d global_object_pose;
  const robot_model::JointModelGroup* arm_jmg;
  std::vector<moveit_grasps::GraspCandidatePtr> grasp_candidates;

  // Scoring
  std::size_t overall_attempts = 0;
  std::size_t overall_successes = 0;
  std::size_t product_attempts;
  std::size_t product_successes;

  std::stringstream csv_log_stream;

  // Create header of product names and save
  namespace fs = boost::filesystem;
  fs::path target_dir(package_path_ + "/meshes/products/");
  fs::directory_iterator it(target_dir), eod;
  ROS_INFO_STREAM_NAMED("apc_manager","Loading meshes from directory: " << target_dir.string());

  std::vector<std::string> product_names;
  BOOST_FOREACH(fs::path const &p, std::make_pair(it, eod))
  {
    product_names.push_back(p.stem().string());
    csv_log_stream << product_names.back() << "\t";
  }
  csv_log_stream << "total_time, average" << std::endl;

  // For each shelf setup (of a single product in each bin)
  for (std::size_t i = 0; i < product_names.size(); ++i)
  {
    if (!ros::ok())
      return false;

    const std::string& product_name = product_names[i];
    product_attempts = 0;
    product_successes = 0;

    // Create shelf
    if (!loadShelfWithOnlyOneProduct(product_name))
    {
      ROS_ERROR_STREAM_NAMED("apc_manager","Failed to load shelf with product " << product_name);
      return false;
    }

    // Test grasping in each bin
    std::size_t bin_skipper = 0;
    for (BinObjectMap::const_iterator bin_it = shelf_->getBins().begin(); bin_it != shelf_->getBins().end(); bin_it++)
    {
      if (!ros::ok())
        return false;

      if (false)
      {
        bin_skipper++;
        if (bin_skipper != 3 && bin_skipper != 6 && bin_skipper != 9)
          continue;
      }

      // Keep score of performance
      overall_attempts++;
      product_attempts++;

      const BinObjectPtr bin = bin_it->second;
      ProductObjectPtr product = bin->getProducts()[0]; // Choose first object
      WorkOrder work_order(bin, product);

      // Get the pose of the product
      //perceiveObjectFake(global_object_pose, product);

      // Choose which arm to use
      arm_jmg = manipulation_->chooseArm(global_object_pose);

      // Allow fingers to touch object
      manipulation_->allowFingerTouch(product->getCollisionName(), arm_jmg);

      // Generate and chose grasp
      bool success = true;
      if (!manipulation_->chooseGrasp(work_order, arm_jmg, grasp_candidates, verbose_))
      {
        ROS_WARN_STREAM_NAMED("apc_manager","No grasps found for product " << product->getName() << " in bin " << bin->getName() );
        success = false;
      }
      else
      {
        overall_successes++;
        product_successes++;
      }

      // Scoring
      ROS_INFO_STREAM_NAMED("apc_manager","Overall success rate: " << std::setprecision(3) << (double(overall_successes)/double(overall_attempts)*100.0));
      ROS_INFO_STREAM_NAMED("apc_manager","Product success rate: " << std::setprecision(3) << (double(product_successes)/double(product_attempts)*100.0));

      std::cout << std::endl << std::endl;
      std::cout << "-------------------------------------------------------" << std::endl;

      // Show robot
      if (success && verbose_)
      {
        if (config_->dual_arm_)
          the_grasp_state->setToDefaultValues(config_->both_arms_, config_->start_pose_); // hide the other arm
        the_grasp_state->setJointGroupPositions(arm_jmg, grasp_candidates.front()->grasp_ik_solution_);
        manipulation_->setStateWithOpenEE(true, the_grasp_state);
        visuals_->visual_tools_->publishRobotState(the_grasp_state, rvt::PURPLE);

        if (verbose_)
          ros::Duration(5.0).sleep();
      }

      visuals_->visual_tools_->deleteAllMarkers();
    } // for each bin

      // Save the stats on the product
    csv_log_stream << (double(product_successes)/double(product_attempts)*100.0) << "\t";

  } // for each product

    // Benchmark runtime
  double duration = (ros::Time::now() - start_time).toSec();
  double average = double(overall_successes)/double(overall_attempts)*100.0;
  ROS_INFO_STREAM_NAMED("","Total time: " << duration << " seconds averaging " << duration/overall_successes << " seconds per grasp");
  csv_log_stream << duration << "\t" << average << std::endl;

  // Save the logging file
  std::string file_path;
  manipulation_->getFilePath(file_path, "grasping_test");
  ROS_INFO_STREAM_NAMED("apc_manager","Saving grasping data to " << file_path);

  std::ofstream logging_file; // open to append
  logging_file.open(file_path.c_str(), std::ios::out | std::ios::app);
  logging_file << csv_log_stream.str();
  logging_file.flush(); // save
  return true;
}

// Mode 17
bool APCManager::testJointLimits()
{
  ROS_INFO_STREAM_NAMED("apc_manager","Testing joint limits");
  ROS_WARN_STREAM_NAMED("apc_manager","DOES NOT CHECK FOR COLLISION");

  moveit::core::RobotStatePtr current_state = manipulation_->getCurrentState();

  // Create goal
  moveit::core::RobotStatePtr goal_state(new moveit::core::RobotState(*current_state));

  // Setup data
  std::vector<double> joint_position;
  joint_position.resize(1);
  const std::vector<const moveit::core::JointModel*> &joints = config_->right_arm_->getActiveJointModels();

  // Decide if we are testing 1 joint or all
  int test_joint_limit_joint;
  std::size_t first_joint;
  std::size_t last_joint;
  rvt::getIntParameter("apc_manager", nh_private_, "test/test_joint_limit_joint", test_joint_limit_joint);
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
      double reduce_bound = 0.5;

      // Move to min bound
      std::cout << std::endl;
      std::cout << "-------------------------------------------------------" << std::endl;
      joint_position[0] = bound.min_position_ + reduce_bound;
      ROS_INFO_STREAM_NAMED("apc_manager","Sending joint " << joints[i]->getName() << " to min position of " << joint_position[0]);
      goal_state->setJointPositions(joints[i], joint_position);

      if (!manipulation_->executeState(goal_state, config_->right_arm_, config_->main_velocity_scaling_factor_))
      {
        ROS_ERROR_STREAM_NAMED("apc_manager","Unable to move to min bound of " << joint_position[0] << " on joint "
                               << joints[i]->getName());
      }
      ros::Duration(1.0).sleep();

      // Move to max bound
      std::cout << std::endl;
      std::cout << "-------------------------------------------------------" << std::endl;
      joint_position[0] = bound.max_position_ - reduce_bound;
      ROS_INFO_STREAM_NAMED("apc_manager","Sending joint " << joints[i]->getName() << " to max position of " << joint_position[0]);
      goal_state->setJointPositions(joints[i], joint_position);

      if (!manipulation_->executeState(goal_state, config_->right_arm_, config_->main_velocity_scaling_factor_))
      {
        ROS_ERROR_STREAM_NAMED("apc_manager","Unable to move to max bound of " << joint_position[0] << " on joint "
                               << joints[i]->getName());
      }
      ros::Duration(1.0).sleep();
    }
  }

  ROS_INFO_STREAM_NAMED("apc_manager","Done testing joint limits");
  return true;
}

// Mode 18
bool APCManager::testPerceptionComm()
{
  // Load JSON file
  loadShelfContents(order_file_path_);

  // Generate random product poses and visualize the shelf
  createRandomProductPoses();

  // Display planning scene
  planning_scene_manager_->displayShelfWithOpenBins();

  BinObjectPtr& bin = shelf_->getBins()["bin_I"];
  if (bin->getProducts().size() == 0)
  {
    ROS_ERROR_STREAM_NAMED("apc_manager","No products in bin "<< bin->getName());
    return false;
  }
  ProductObjectPtr& product = bin->getProducts().front();

  // Communicate with perception pipeline
  perception_interface_->startPerception(product, bin);

  // Dummy wait
  ROS_WARN_STREAM_NAMED("apc_manager","dummy wait");
  ros::Duration(10).sleep();

  // Get result from perception pipeline
  if (!perception_interface_->endPerception(product, bin))
  {
    ROS_ERROR_STREAM_NAMED("apc_manager","End perception failed");
    return false;
  }

  return true;
}

// Mode 11
bool APCManager::recordBinWithCamera(std::size_t bin_id)
{
  return recordBinWithCamera(shelf_->getBin(bin_id));
}

// Mode 12
bool APCManager::perceiveBinWithCamera(std::size_t bin_id)
{
  // Load JSON file
  loadShelfContents(order_file_path_);

  // Generate random product poses and visualize the shelf
  createRandomProductPoses();

  return perceiveBinWithCamera(shelf_->getBin(bin_id));
}

bool APCManager::recordBinWithCamera(BinObjectPtr bin)
{
  ROS_DEBUG_STREAM_NAMED("apc_manager","Recoding bin observation trajectory around " << bin->getName());

  std::string file_path;
  const std::string file_name = "observe_bin_" + bin->getName() + "_trajectory";
  manipulation_->getFilePath(file_path, file_name);

  // Start recording
  manipulation_->recordTrajectoryToFile(file_path);

  ROS_INFO_STREAM_NAMED("apc_manager","Done recording bin with camera");

  return true;
}

bool APCManager::perceiveBinWithCamera(BinObjectPtr bin)
{
  ROS_DEBUG_STREAM_NAMED("apc_manager","Moving camera around " << bin->getName());

  // Choose which planning group to use
  const robot_model::JointModelGroup* arm_jmg = config_->dual_arm_ ? config_->left_arm_ : config_->right_arm_;

  // Start playing back file
  const std::string file_name = "observe_bin_" + bin->getName() + "_trajectory";
  std::string file_path;
  manipulation_->getFilePath(file_path, file_name);

  if (bin->getProducts().empty())
  {
    ROS_ERROR_STREAM_NAMED("apc_manager","No products in bin " << bin->getName());
    return false;
  }
  ProductObjectPtr product = bin->getProducts()[0];

  // Communicate with perception pipeline
  perception_interface_->startPerception(product, bin);

  if (!manipulation_->playbackTrajectoryFromFile(file_path, arm_jmg, config_->calibration_velocity_scaling_factor_))
  {
    ROS_ERROR_STREAM_NAMED("apc_manager","Unable to playback " << file_name);
    return false;
  }

  // Set planning scene
  planning_scene_manager_->displayShelfWithOpenBins();

  // Get result from perception pipeline
  if (!perception_interface_->endPerception(product, bin))
  {
    ROS_ERROR_STREAM_NAMED("apc_manager","End perception failed");
    return false;
  }

  ROS_INFO_STREAM_NAMED("apc_manager","Done observing bin");
  return true;
}


bool APCManager::perceiveObject(WorkOrder work_order, bool verbose)
{
  BinObjectPtr& bin = work_order.bin_;
  ProductObjectPtr& product = work_order.product_;

  // Move camera to the bin
  ROS_INFO_STREAM_NAMED("apc_manager","Moving camera to bin '" << bin->getName() << "'");

  // if (!manipulation_->moveCameraToBin(bin))
  // {
  //   ROS_ERROR_STREAM_NAMED("apc_manager","Unable to move camera to bin " << bin->getName());
  //   return false;
  // }

  // Move camera to bin using pre-recorded trajectory
  if (!perceiveBinWithCamera(bin))
  {
    ROS_ERROR_STREAM_NAMED("apc_manager","Unable to move camera to bin " << bin->getName());
    return false;
  }

  // Communicate with perception pipeline
  std::cout << std::endl;
  std::cout << "-------------------------------------------------------" << std::endl;
  perception_interface_->startPerception(product, bin);

  // Perturb camera
  // if (!manipulation_->perturbCamera(bin))
  // {
  //   ROS_ERROR_STREAM_NAMED("apc_manager","Failed to perturb camera around product");
  // }

  // Run pre-recorded camera trajectory
  if (false)
    if (!perceiveBinWithCamera(bin))
      ROS_ERROR_STREAM_NAMED("apc_manager","Unable to move camera around bin");

  // Let arm come to rest
  double timeout = 20;
  manipulation_->waitForRobotToStop(timeout);

  // Get result from perception pipeline
  if (!perception_interface_->endPerception(product, bin))
  {
    return false;
  }

  return true;
}

bool APCManager::perceiveObjectFake(WorkOrder work_order)
{
  BinObjectPtr& bin = work_order.bin_;
  ProductObjectPtr& product = work_order.product_;

  const Eigen::Affine3d& world_to_bin = picknik_main::transform(bin->getBottomRight(), shelf_->getBottomRight());

  Eigen::Affine3d fake_centroid = Eigen::Affine3d::Identity();
  fake_centroid.translation().y() = 0.12;
  fake_centroid.translation().x() = 0.08;
  fake_centroid.translation().z() = 0.08;
  fake_centroid = fake_centroid
    * Eigen::AngleAxisd(1.57, Eigen::Vector3d::UnitX())
    * Eigen::AngleAxisd(1.57, Eigen::Vector3d::UnitY());
  product->setCentroid(fake_centroid);
  product->setMeshCentroid(fake_centroid);

  // Show in collision and display Rvizs
  product->visualize(world_to_bin);
  product->createCollisionBodies(world_to_bin);

  return true;
}

bool APCManager::placeObjectInGoalBin(const robot_model::JointModelGroup* arm_jmg)
{
  // Move to position
  if (!moveToDropOffPosition(arm_jmg))
  {
    ROS_ERROR_STREAM_NAMED("apc_manager","Unable to plan to goal bin");
    return false;
  }

  // Drop down
  bool up = false;
  if (!manipulation_->executeVerticlePath(arm_jmg, config_->place_goal_down_distance_desired_, 
                                          config_->main_velocity_scaling_factor_, up))
  {
    ROS_ERROR_STREAM_NAMED("apc_manager","Failed to lower product into goal bin, using distance "
                           << config_->place_goal_down_distance_desired_);
    return false;
  }

  return true;
}

bool APCManager::liftFromGoalBin(const robot_model::JointModelGroup* arm_jmg)
{
  bool up = true;
  if (!manipulation_->executeVerticlePath(arm_jmg, config_->place_goal_down_distance_desired_, 
                                          config_->main_velocity_scaling_factor_, up))
  {
    ROS_ERROR_STREAM_NAMED("apc_manager","Failed to raise arm back up from goal, using distance "
                           << config_->place_goal_down_distance_desired_);
    return false;
  }

  return true;
}

bool APCManager::moveToStartPosition(const robot_model::JointModelGroup* arm_jmg, bool check_validity)
{
  return manipulation_->moveToStartPosition(arm_jmg, check_validity);
}

bool APCManager::moveToDropOffPosition(const robot_model::JointModelGroup* arm_jmg)
{
  // Create locations if necessary
  generateGoalBinLocations();

  // Error check
  if (next_dropoff_location_ >= dropoff_locations_.size())
  {
    ROS_ERROR_STREAM_NAMED("apc_manager","This should never happen");
    next_dropoff_location_ = 0; // reset
  }

  // Translate to custom end effector geometry
  Eigen::Affine3d dropoff_location = dropoff_locations_[next_dropoff_location_];
  dropoff_location = dropoff_location * grasp_datas_[arm_jmg]->grasp_pose_to_eef_pose_;

  // Move
  if (!manipulation_->moveEEToPose(dropoff_location, config_->main_velocity_scaling_factor_, arm_jmg))
  {
    ROS_ERROR_STREAM_NAMED("apc_manager","Failed to move arm to dropoff location");
    return false;
  }

  // Set next dropoff location id
  ++next_dropoff_location_;
  if (next_dropoff_location_ >= dropoff_locations_.size())
    next_dropoff_location_ = 0; // reset

  return true;
}

bool APCManager::loadShelfWithOnlyOneProduct(const std::string& product_name)
{
  ROS_INFO_STREAM_NAMED("apc_manager","Loading shelf with product " << product_name);

  // Create a product that we can use over and over
  ProductObjectPtr product_seed(new ProductObject(visuals_, rvt::RAND, product_name, package_path_));
  product_seed->loadCollisionBodies(); // do this once for all objects

  // For each bin
  for (BinObjectMap::const_iterator bin_it = shelf_->getBins().begin(); bin_it != shelf_->getBins().end(); bin_it++)
  {
    // Clear products in this bin
    const BinObjectPtr bin = bin_it->second;
    bin->getProducts().clear();

    // Clone the product
    ProductObjectPtr product(new ProductObject(*product_seed));

    // Add this product
    bin->getProducts().push_back(product);
  }

  // Randomize product locations
  bool product_simulator_verbose = false;
  ProductSimulator product_simulator(product_simulator_verbose, visuals_, planning_scene_monitor_);
  product_simulator.generateRandomProductPoses(shelf_);

  return true;
}

bool APCManager::loadShelfContents(std::string work_order_file_path)
{
  // Make sure shelf is empty
  shelf_->clearProducts();
  orders_.clear();

  // Choose file
  AmazonJSONParser parser(verbose_, visuals_);

  // Parse json
  return parser.parse(work_order_file_path, package_path_, shelf_, orders_);
}

bool APCManager::loadPlanningSceneMonitor()
{
  // Allows us to sycronize to Rviz and also publish collision objects to ourselves
  ROS_DEBUG_STREAM_NAMED("apc_manager","Loading Planning Scene Monitor");
  static const std::string PLANNING_SCENE_MONITOR_NAME = "AmazonShelfWorld";
  planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor(planning_scene_, robot_model_loader_,
                                                                                 tf_, PLANNING_SCENE_MONITOR_NAME));
  ros::spinOnce();

  // Get the joint state topic
  std::string joint_state_topic;
  rvt::getStringParameter("apc_manager", nh_private_, "joint_state_topic", joint_state_topic);
  if (planning_scene_monitor_->getPlanningScene())
  {
    // Optional monitors to start:
    planning_scene_monitor_->startStateMonitor(joint_state_topic, ""); ///attached_collision_object");
    planning_scene_monitor_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
                                                          "picknik_planning_scene");
    planning_scene_monitor_->getPlanningScene()->setName("picknik_planning_scene");
  }
  else
  {
    ROS_ERROR_STREAM_NAMED("apc_manager","Planning scene not configured");
    return false;
  }
  ros::spinOnce();
  ros::Duration(0.5).sleep(); // when at 0.1, i believe sometimes vjoint not properly loaded

  // Wait for complete state to be recieved
  std::vector<std::string> missing_joints;
  std::size_t counter = 0;
  while( !planning_scene_monitor_->getStateMonitor()->haveCompleteState() && ros::ok() )
  {
    ROS_INFO_STREAM_THROTTLE_NAMED(1, "apc_manager","Waiting for complete state from topic " << joint_state_topic);
    ros::Duration(0.1).sleep();
    ros::spinOnce();

    // Show unpublished joints
    if( counter % 10 == 0)
    {
      planning_scene_monitor_->getStateMonitor()->haveCompleteState( missing_joints );
      for(std::size_t i = 0; i < missing_joints.size(); ++i)
        ROS_WARN_STREAM_NAMED("apc_manager","Unpublished joints: " << missing_joints[i]);
    }
    counter++;
  }
  ros::spinOnce();

  return true;
}

void APCManager::publishCurrentState()
{
  planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_); // Lock planning scene
  visuals_->visual_tools_->publishRobotState(scene->getCurrentState(), rvt::PURPLE);
}

bool APCManager::getPlanningSceneService(moveit_msgs::GetPlanningScene::Request &req, moveit_msgs::GetPlanningScene::Response &res)
{
  if (req.components.components & moveit_msgs::PlanningSceneComponents::TRANSFORMS)
    planning_scene_monitor_->updateFrameTransforms();
  planning_scene_monitor::LockedPlanningSceneRO ps(planning_scene_monitor_);
  ps->getPlanningSceneMsg(res.scene, req.components);
  return true;
}

RemoteControlPtr APCManager::getRemoteControl()
{
  return remote_control_;
}

bool APCManager::allowCollisions()
{
  // Allow collisions between frame of robot and floor
  {
    planning_scene_monitor::LockedPlanningSceneRW scene(planning_scene_monitor_); // Lock planning
    scene->getAllowedCollisionMatrixNonConst().setEntry(shelf_->getEnvironmentCollisionObject("floor_wall")->getCollisionName(),
                                                        "frame", true);
  }

  return true;
}

bool APCManager::attachProduct(ProductObjectPtr product, const robot_model::JointModelGroup* arm_jmg)
{
  visuals_->visual_tools_->attachCO(product->getCollisionName(), grasp_datas_[arm_jmg]->parent_link_->getName());
  visuals_->visual_tools_->triggerPlanningSceneUpdate();

  // Debug
  if (false)
  {
    ROS_WARN_STREAM_NAMED("apc_manager","Attached to link " << grasp_datas_[arm_jmg]->parent_link_->getName() << " product "
                          << product->getCollisionName());

    std::vector<const moveit::core::AttachedBody*> attached_bodies;
    manipulation_->getCurrentState()->getAttachedBodies(attached_bodies);

    for (std::size_t i = 0; i < attached_bodies.size(); ++i)
    {
      std::cout << "attached body: " << attached_bodies[i]->getName() << std::endl;
    }
  }

  return true;
}

// Mode 21
bool APCManager::displayExperienceDatabase()
{
  // Choose which planning group to use
  const robot_model::JointModelGroup* arm_jmg = config_->dual_arm_ ? config_->both_arms_ : config_->right_arm_;

  return manipulation_->displayLightningPlansStandAlone(arm_jmg);
}

bool APCManager::generateGoalBinLocations()
{
  // Check if we need to generate these
  if (dropoff_locations_.size())
    return true;

  static std::size_t NUM_DROPOFF_LOCATIONS = 8;

  // Calculate dimensions of goal bin
  bool verbose = false;
  shelf_->getGoalBin()->calculateBoundingBox();

  // Visualize
  shelf_->getGoalBin()->visualizeWireframe(shelf_->getBottomRight());

  // Find starting location of dropoff
  Eigen::Affine3d overhead_goal_bin = shelf_->getBottomRight() * shelf_->getGoalBin()->getCentroid();
  overhead_goal_bin.translation().z() += shelf_->getGoalBin()->getHeight() + config_->goal_bin_clearance_;

  // Convert pose that has z arrow pointing to object, to pose that has z arrow pointing towards object and x out in the grasp dir
  overhead_goal_bin = overhead_goal_bin * Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitX());
  visuals_->visual_tools_->publishAxis(overhead_goal_bin);

  bool visualize_dropoff_locations = visuals_->isEnabled("show_goal_bin_markers");

  // Calculations
  const std::size_t num_cols = 2;
  const std::size_t num_rows = NUM_DROPOFF_LOCATIONS / num_cols;
  const double total_x_depth = shelf_->getGoalBin()->getDepth() / 3.0; // this is the longer dimension
  const double delta_x = total_x_depth / (num_rows - 1);
  const double total_y_width = shelf_->getGoalBin()->getWidth() / 3.0;
  const double delta_y = total_y_width / (num_cols - 1);

  // Create first location
  Eigen::Affine3d first_location = overhead_goal_bin;
  first_location.translation().x() -= total_x_depth / 2.0;
  first_location.translation().y() -= total_y_width / 2.0;
  if (visualize_dropoff_locations)
    visuals_->visual_tools_->publishZArrow(first_location, rvt::BLUE);

  // Generate row and column locaitons
  for (std::size_t y = 0; y < num_cols; ++y)
  {
    for (std::size_t x = 0; x < num_rows; ++x)
    {
      Eigen::Affine3d new_location = first_location;
      new_location.translation().x() += delta_x * x;
      new_location.translation().y() += delta_y * y;

      // Visualize
      if (visualize_dropoff_locations)
        if (!(y ==0 && x==0)) // we already showed this arrow
          visuals_->visual_tools_->publishZArrow(new_location, rvt::GREEN);

      // Save
      dropoff_locations_.push_back(new_location);
    }
  }

  return true;
}

bool APCManager::statusPublisher(const std::string &status)
{
  std::cout << MOVEIT_CONSOLE_COLOR_BLUE << "apc_manager.status: " << status << MOVEIT_CONSOLE_COLOR_RESET << std::endl;

  visuals_->visual_tools_->publishText(status_position_, status, rvt::WHITE, rvt::LARGE);
  return true;
}

// Mode 23
bool APCManager::unitTests()
{
  std::string test_name;

  // Test
  test_name = "SuperSimple";
  if (visuals_->isEnabled("unit_test/" + test_name))
  {
    const std::string json_file = "crayola.json";
    Eigen::Affine3d product_pose = Eigen::Affine3d::Identity();
    product_pose.translation() = Eigen::Vector3d(0.12, 0.1, 0.08);
    product_pose *= Eigen::AngleAxisd(1.57, Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(-1.57, Eigen::Vector3d::UnitY());
    if (!startUnitTest(json_file, test_name, product_pose))
      return false;
  }

  // Test
  test_name = "SimpleRotated";
  if (visuals_->isEnabled("unit_test/" + test_name))
  {
    const std::string json_file = "crayola.json";
    Eigen::Affine3d product_pose = Eigen::Affine3d::Identity();
    product_pose.translation() = Eigen::Vector3d(0.12, 0.1, 0.08);
    product_pose *= Eigen::AngleAxisd(1.57, Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(-1.87, Eigen::Vector3d::UnitY());  // slighlty rotated sideways
    if (!startUnitTest(json_file, test_name, product_pose))
      return false;
  }

  // Test
  test_name = "SimpleVeryRotated";
  if (visuals_->isEnabled("unit_test/" + test_name))
  {
    const std::string json_file = "crayola.json";
    Eigen::Affine3d product_pose = Eigen::Affine3d::Identity();
    product_pose.translation() = Eigen::Vector3d(0.12, 0.1, 0.08);
    product_pose *= Eigen::AngleAxisd(1.57, Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(-2.0, Eigen::Vector3d::UnitY());  // rotated sideways
    if (!startUnitTest(json_file, test_name, product_pose))
      return false;
  }

  // Test
  test_name = "SimpleFarBack";
  if (visuals_->isEnabled("unit_test/" + test_name))
  {  
    const std::string json_file = "crayola.json";
    Eigen::Affine3d product_pose = Eigen::Affine3d::Identity();
    product_pose.translation() = Eigen::Vector3d(0.12, 0.1, 0.14);
    product_pose *= Eigen::AngleAxisd(1.57, Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(-1.57, Eigen::Vector3d::UnitY());  // rotated sideways
    if (!startUnitTest(json_file, test_name, product_pose))
      return false;
  }

  return true;
}

bool APCManager::startUnitTest(const std::string &json_file, const std::string &test_name, const Eigen::Affine3d &product_pose)
{
  std::cout << std::endl << MOVEIT_CONSOLE_COLOR_BROWN;
  std::cout << "------------------------------------------------------------------------------" << std::endl;
  std::cout << "------------------------------------------------------------------------------" << std::endl;
  std::cout << "STARTING UNIT TEST " << test_name << std::endl;
  std::cout << "------------------------------------------------------------------------------" << std::endl;
  std::cout << "------------------------------------------------------------------------------" << std::endl;
  std::cout << MOVEIT_CONSOLE_COLOR_RESET << std::endl;

  // Load json file
  std::string json_file_path = package_path_ + "/orders/" + json_file;
  loadShelfContents(json_file_path);

  // Get all the products in the shelf
  std::vector<ProductObjectPtr> products;
  shelf_->getAllProducts(products);

  // Set all products to same exact pose
  for (std::vector<ProductObjectPtr>::iterator product_it = products.begin(); product_it != products.end(); product_it++)
  {
    ProductObjectPtr &p = *product_it;
    p->setCentroid(product_pose);
    p->setMeshCentroid(product_pose);
  }  

  // Display new shelf
  visuals_->visualizeDisplayShelf(shelf_);

  // Update planning scene
  bool force = true;
  planning_scene_manager_->displayShelfWithOpenBins(force);

  ROS_INFO_STREAM_NAMED("apc_manager","Finished updating json file and product location for unit test");
  ros::Duration(2.0).sleep();
  ros::spinOnce();

  // Disable actual execution
  if (!visuals_->isEnabled("show_simulated_paths_moving"))
    manipulation_->getExecutionInterface()->enableUnitTesting();

  // Start processing
  if (!runOrder(0, 0, 0)) // do all the orders
  {
    ROS_ERROR_STREAM_NAMED("apc_manager","Test '" << test_name << "' failed to run fully");
    return false;
  }
  return true;
}

// Mode 23
bool APCManager::gotoPose(const std::string& pose_name)
{
  ROS_INFO_STREAM_NAMED("apc_manager","Going to pose " << pose_name);
  planning_scene_manager_->displayShelfWithOpenBins();
  ros::Duration(1).sleep();
  ros::spinOnce();

  const robot_model::JointModelGroup* arm_jmg = config_->dual_arm_ ? config_->both_arms_ : config_->right_arm_;
  bool check_validity = true;

  if (!manipulation_->moveToPose(arm_jmg, pose_name, config_->main_velocity_scaling_factor_, check_validity))
  {
    ROS_ERROR_STREAM_NAMED("apc_manager","Unable to move to pose");
    return false;
  }
  ROS_INFO_STREAM_NAMED("apc_manager","Spinning until shutdown requested");
  ros::spin();
  return true;
}

// Mode 25
bool APCManager::testIKSolver()
{
  moveit::core::RobotStatePtr goal_state(new moveit::core::RobotState(*manipulation_->getCurrentState()));

  const robot_model::JointModelGroup* arm_jmg = config_->right_arm_;
  Eigen::Affine3d ee_pose = Eigen::Affine3d::Identity();
  ee_pose.translation().x() += 0.3;
  ee_pose.translation().y() += 0.2;
  ee_pose.translation().z() += 1.4;
  ee_pose = ee_pose * Eigen::AngleAxisd(-M_PI/2.0, Eigen::Vector3d::UnitY());

  visuals_->visual_tools_->publishAxisLabeled(ee_pose, "desired");

  // Transform from world frame to 'gantry' frame  
  if (visuals_->isEnabled("generic_bool"))
    ee_pose = goal_state->getGlobalLinkTransform("gantry") * ee_pose;

  for (std::size_t i = 0; i < 100; ++i)
  {
    // Solve IK problem for arm
    std::size_t attempts = 0; // use default
    double timeout = 0; // use default
    if (!goal_state->setFromIK(arm_jmg, ee_pose, attempts, timeout))
    {
      ROS_ERROR_STREAM_NAMED("manipulation","Unable to find arm solution for desired pose");
      return false;
    }

    ROS_INFO_STREAM_NAMED("apc_manager","SOLVED");

    // Show solution
    visuals_->visual_tools_->publishRobotState(goal_state, rvt::RAND);

    ros::Duration(0.5).sleep();
    goal_state->setToRandomPositions(arm_jmg);
  }

  return true;
}

// Mode 26
bool APCManager::unitTestPerceptionComm()
{
  std::cout << "-------------------------------------------------------" << std::endl;
  std::cout << "-------------------------------------------------------" << std::endl;
  ROS_INFO_STREAM_NAMED("apc_manager","FIRST ENSURE THAT SERVER IS OFF");
  std::cout << "-------------------------------------------------------" << std::endl;
  std::cout << "-------------------------------------------------------" << std::endl;
  remote_control_->waitForNextStep("start with perception server off");

  // Test if connected
  if (perception_interface_->isPerceptionReady())
  {
    ROS_ERROR_STREAM_NAMED("apc_manager","Reports perception is ready when it should not!");
  }

  remote_control_->waitForNextStep("Now start perception server");

  // Test if connected
  if (!perception_interface_->isPerceptionReady())
  {
    ROS_ERROR_STREAM_NAMED("apc_manager","Reports perception is not ready when it should be!");
  }

  return true;
}


} // end namespace
