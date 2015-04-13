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

// Boost
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>

namespace picknik_main
{

APCManager::APCManager(bool verbose, std::string order_file_path, bool use_experience, bool show_database, bool autonomous)
  : nh_private_("~")
  , verbose_(verbose)
  , fake_perception_(false)
  , skip_homing_step_(true)
{
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
  loadShelfContents(order_file_path);

  // Load the remote control for dealing with GUIs
  remote_control_.reset(new RemoteControl(verbose, nh_private_, this));
  remote_control_->setAutonomous(autonomous);

  // Load manipulation data for our robot
  config_.reset(new ManipulationData());
  config_->load(robot_model_);

  // Load grasp data specific to our robot
  grasp_datas_[config_->right_arm_].reset(new moveit_grasps::GraspData(nh_private_, config_->right_hand_name_, robot_model_));
  if (config_->dual_arm_)
    grasp_datas_[config_->left_arm_].reset(new moveit_grasps::GraspData(nh_private_, config_->left_hand_name_, robot_model_));

  // Create manipulation manager
  manipulation_.reset(new Manipulation(verbose_, visuals_, planning_scene_monitor_, config_, grasp_datas_,
                                       remote_control_, package_path_, shelf_, use_experience, show_database));

  // Load perception layer
  perception_interface_.reset(new PerceptionInterface(verbose_, visuals_, shelf_, config_, tf_, nh_private_));

  // Generate random product poses and visualize the shelf
  bool product_simulator_verbose = false;
  ProductSimulator product_simulator(product_simulator_verbose, visuals_, planning_scene_monitor_);
  product_simulator.generateRandomProductPoses(shelf_);

  // Allow collisions between frame of robot and floor
  allowCollisions();

  ROS_INFO_STREAM_NAMED("apc_manager","APC Manager Ready.");
}

bool APCManager::checkSystemReady()
{
  ROS_INFO_STREAM_NAMED("apc_manager","Starting system ready check:");

  // Check joint model groups, assuming we are the jaco arm
  if (config_->right_arm_->getVariableCount() < 6 || config_->right_arm_->getVariableCount() > 7)
  {
    ROS_FATAL_STREAM_NAMED("apc_manager","Incorrect number of joints for group " << config_->right_arm_->getName()
                           << ", joints: " << config_->right_arm_->getVariableCount());
    return false;
  }
  const robot_model::JointModelGroup* ee_jmg = grasp_datas_[config_->right_arm_]->ee_jmg_;
  if (ee_jmg->getVariableCount() > 3)
  {
    ROS_FATAL_STREAM_NAMED("apc_manager","Incorrect number of joints for group " << ee_jmg->getName() << ", joints: "
                           << ee_jmg->getVariableCount());
    return false;
  }

  // Check trajectory execution manager
  if( !manipulation_->checkExecutionManager() )
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
  manipulation_->createCollisionWall(); // Reduce collision model to simple wall that prevents Robot from hitting shelf
  while (ros::ok() && !manipulation_->fixCurrentCollisionAndBounds(arm_jmg))
  {
    // Show the current state just for the heck of it
    publishCurrentState();

    ros::Duration(0.5).sleep();
  }

  // Check robot calibrated
  // TODO

  // Check gantry calibrated
  // TODO

  // Check end effectors calibrated
  // TODO

  ROS_INFO_STREAM_NAMED("apc_manager","checkSystemReady() COMPLETE");
  std::cout << std::endl;
  std::cout << "-------------------------------------------------------" << std::endl;
  return true;
}

bool APCManager::focusSceneOnBin( const std::string& bin_name )
{
  // Disable all bins except desired one
  visuals_->visual_tools_->deleteAllMarkers(); // clear all old markers
  visuals_->visual_tools_->removeAllCollisionObjects(); // clear all old collision objects

  // Visualize
  bool only_show_shelf_frame = false;
  ROS_INFO_STREAM_NAMED("apc_manager","Showing planning scene shelf with focus on bin " << bin_name);

  shelf_->createCollisionBodies(bin_name, only_show_shelf_frame);
  visuals_->visual_tools_->triggerPlanningSceneUpdate();
  shelf_->visualizeAxis(visuals_);
  ros::Duration(0.5).sleep();

  return true;
}

// Mode 1
bool APCManager::runOrder(std::size_t order_start, std::size_t jump_to,
                          std::size_t num_orders)
{
  ROS_INFO_STREAM_NAMED("apc_manager","Starting order ----------------------------");

  // Decide how many products to pick
  if (num_orders == 0)
    num_orders = orders_.size();

  // Grasps things
  for (std::size_t i = order_start; i < num_orders; ++i)
  {
    manipulation_->orderPublisher(orders_[i]); // feedback

    if (!graspObjectPipeline(orders_[i], verbose_, jump_to))
    {
      ROS_ERROR_STREAM_NAMED("apc_manager","Shutting down for debug purposes only (it could continue on)");
      return false;
    }
  }

  manipulation_->statusPublisher("Finished");
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
  moveit_grasps::GraspCandidatePtr chosen; // the grasp to use
  moveit::core::RobotStatePtr pre_grasp_state(new moveit::core::RobotState(*current_state)); // Allocate robot states
  moveit::core::RobotStatePtr the_grasp_state(new moveit::core::RobotState(*current_state)); // Allocate robot states
  moveit_msgs::RobotTrajectory approach_trajectory_msg;
  bool wait_for_trajetory = false;

  // Prevent jump-to errors
  if (jump_to == 3)
  {
    ROS_ERROR_STREAM_NAMED("apc_manager","Cannot jump to step 3 - must start on step 2 to choose grasp");
    jump_to = 0;
  }

  if (!remote_control_->getAutonomous())
  {
    visuals_->start_state_->publishRobotState(current_state, rvt::GREEN);
    ROS_INFO_STREAM_NAMED("apc_manager","Waiting for remote control to be triggered to start");
  }

  // Jump to a particular step in the manipulation pipeline
  std::size_t step = jump_to;
  while(ros::ok())
  {

    if (!remote_control_->getAutonomous())
    {
      std::cout << std::endl;
      std::cout << std::endl;
      std::cout << "Waiting for step: " << step << std::endl;
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
      case 0: manipulation_->statusPublisher("Moving to initial position");

        // Clear the temporary purple robot state image
        visuals_->visual_tools_->hideRobot();
        manipulation_->createCollisionWall(); // Reduce collision model to simple wall that prevents Robot from hitting shelf

        if (!skip_homing_step_)
        {
          // Move
          if (!moveToStartPosition())
          {
            ROS_ERROR_STREAM_NAMED("apc_manager","Unable to move to initial position");
            return false;
          }
        }

        //break;
        step++;

        // #################################################################################################################
      case 1: manipulation_->statusPublisher("Open end effectors");

        if (!manipulation_->openEndEffectors(true))
        {
          ROS_ERROR_STREAM_NAMED("apc_manager","Unable to open end effectors");
          return false;
        }
        //break;
        step++;

        // #################################################################################################################
      case 2: manipulation_->statusPublisher("Finding location of product " + work_order.product_->getName() + " from " + work_order.bin_->getName());

        // Move camera to desired bin
        if (!focusSceneOnBin( work_order.bin_->getName() ))
        {
          ROS_ERROR_STREAM_NAMED("apc_manager","Unable to setup planning scene");
          return false;
        }

        // Get pose of product
        if (!perceiveObject(work_order, verbose))
        {
          ROS_ERROR_STREAM_NAMED("apc_manager","Unable to get object pose");
          return false;
        }
        break;

        // #################################################################################################################
      case 3: manipulation_->statusPublisher("Get grasp for product " + work_order.product_->getName() + " from " + work_order.bin_->getName());

        // Choose which arm to use
        arm_jmg = manipulation_->chooseArm(work_order.product_->getWorldPose(shelf_, work_order.bin_));

        // Allow fingers to touch object
        manipulation_->allowFingerTouch(work_order.product_->getCollisionName(), arm_jmg);

        // Generate and chose grasp
        if (!manipulation_->chooseGrasp(work_order, arm_jmg, chosen, verbose))
        {
          ROS_ERROR_STREAM_NAMED("apc_manager","No grasps found");
          return false;
        }

        // Get the pre and post grasp states
        chosen->getPreGraspState(pre_grasp_state);
        chosen->getGraspState(the_grasp_state);

        // Visualize
        if (!remote_control_->getAutonomous() || true)
        {
          visuals_->start_state_->publishRobotState(pre_grasp_state, rvt::GREEN);
          visuals_->goal_state_->publishRobotState(the_grasp_state, rvt::ORANGE);
        }

        break;

        // #################################################################################################################
      case 4: manipulation_->statusPublisher("Get pre-grasp by generateApproachPath()");

        // Hide the purple robot
        visuals_->visual_tools_->hideRobot();

        if (!manipulation_->generateApproachPath(chosen, approach_trajectory_msg, pre_grasp_state, the_grasp_state, verbose))
        {
          ROS_ERROR_STREAM_NAMED("apc_manager","Unable to generate straight approach path");
          return false;
        }

        // Visualize trajectory in Rviz display
        visuals_->visual_tools_->publishTrajectoryPath(approach_trajectory_msg, current_state, wait_for_trajetory);

        break;

        // #################################################################################################################
      case 5: // Not implemented

        //break
        step++;

        // #################################################################################################################
      case 6: manipulation_->statusPublisher("Moving to pre-grasp position");

        current_state = manipulation_->getCurrentState();
        manipulation_->setStateWithOpenEE(true, current_state);

        if (!manipulation_->move(current_state, pre_grasp_state, arm_jmg, config_->main_velocity_scaling_factor_, verbose, execute_trajectory))
        {
          ROS_ERROR_STREAM_NAMED("apc_manager","Unable to plan");
          return false;
        }
        break;

        // #################################################################################################################
      case 7: manipulation_->statusPublisher("Cartesian move to the-grasp position");

        // Create the collision objects
        if (!focusSceneOnBin( work_order.bin_->getName() ))
        {
          ROS_ERROR_STREAM_NAMED("apc_manager","Unable to setup planning scene");
          return false;
        }

        // Visualize trajectory in Rviz display
        current_state = manipulation_->getCurrentState();
        visuals_->visual_tools_->publishTrajectoryPath(approach_trajectory_msg, current_state, wait_for_trajetory);

        // Run
        if( !manipulation_->executeTrajectory(approach_trajectory_msg) )
        {
          ROS_ERROR_STREAM_NAMED("apc_manager","Failed to move to the-grasp position");
          return false;
        }
        ROS_INFO_STREAM_NAMED("apc_manager","Waiting " << config_->wait_before_grasp_ << " seconds before grasping");
        ros::Duration(config_->wait_after_grasp_).sleep();
        break;

        // #################################################################################################################
      case 8: manipulation_->statusPublisher("Grasping");

        if (!manipulation_->openEndEffectorWithVelocity(false, arm_jmg))
        {
          ROS_ERROR_STREAM_NAMED("apc_manager","Unable to close end effector");
          return false;
        }

        // Attach collision object
        visuals_->visual_tools_->attachCO(work_order.product_->getCollisionName(), grasp_datas_[arm_jmg]->parent_link_->getName());

        ROS_INFO_STREAM_NAMED("apc_manager","Waiting " << config_->wait_after_grasp_ << " seconds after grasping");

        ros::Duration(config_->wait_after_grasp_).sleep();
        break;

        // #################################################################################################################
      case 9: manipulation_->statusPublisher("Lifting product UP slightly");

        // Clear all collision objects
        visuals_->visual_tools_->removeAllCollisionObjects(); // clear all old collision objects

        if (!manipulation_->executeVerticlePath(arm_jmg, config_->lift_distance_desired_))
        {
          ROS_ERROR_STREAM_NAMED("apc_manager","Unable to execute retrieval path after grasping");
          return false;
        }
        break;

        // #################################################################################################################
      case 10: manipulation_->statusPublisher("Moving BACK to pre-grasp position (retreat path)");

        if (!manipulation_->executeRetreatPath(arm_jmg))
        {
          ROS_ERROR_STREAM_NAMED("apc_manager","Unable to execute retrieval path after grasping");
          return false;
        }
        break;

        // #################################################################################################################
      case 11: manipulation_->statusPublisher("Placing product in bin");

        manipulation_->createCollisionWall(); // Reduce collision model to simple wall that prevents Robot from hitting shelf

        if (!placeObjectInGoalBin(arm_jmg))
        {
          ROS_ERROR_STREAM_NAMED("apc_manager","Unable to move object to goal bin");
          return false;
        }

        break;

        // #################################################################################################################
      case 12: manipulation_->statusPublisher("Releasing product");

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

        // Delete from planning scene the product
        shelf_->deleteProduct(work_order.bin_->getName(), work_order.product_->getName());

        // Unattach from EE
        visuals_->visual_tools_->cleanupACO( work_order.product_->getCollisionName() ); // use unique name
        visuals_->visual_tools_->cleanupCO( work_order.product_->getCollisionName() ); // use unique name
        break;

        // #################################################################################################################
        // case 13: manipulation_->statusPublisher("Moving to initial position");

        //   if (!moveToStartPosition(arm_jmg))
        //   {
        //     ROS_ERROR_STREAM_NAMED("apc_manager","Unable to move to initial position");
        //     return false;
        //   }

        // #################################################################################################################
      default:
        ROS_INFO_STREAM_NAMED("apc_manager","Manipulation pipeline end reached, ending");
        return true;

    } // end switch
    step++;
  } // end for

  return true;
}

// Mode 2
bool APCManager::trainExperienceDatabase()
{
  ROS_ERROR_STREAM_NAMED("apc_manager","disabled");
  /*
  // Create learning pipeline for training the experience database
  bool use_experience = false;
  learning_.reset(new LearningPipeline(verbose_, visuals_,
  planning_scene_monitor_, plan_execution_,
  shelf_, use_experience));

  ROS_INFO_STREAM_NAMED("apc_manager","Training experience database");
  learning_->generateTrainingGoals(shelf_);
  */

  return true;
}

// Mode 3
bool APCManager::testEndEffectors()
{
  // Test visualization
  manipulation_->statusPublisher("Testing open close visualization of EE");
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

// Mode 4
// Do nothing

// Mode 5
bool APCManager::testUpAndDown()
{
  double lift_distance_desired = 0.5;

  // Test
  manipulation_->statusPublisher("Testing up and down calculations");
  std::size_t i = 0;
  while (ros::ok())
  {
    std::cout << std::endl << std::endl;
    if (i % 2 == 0)
    {
      std::cout << "Moving up --------------------------------------" << std::endl;
      manipulation_->executeVerticlePath(config_->right_arm_, lift_distance_desired, true);
      if (config_->dual_arm_)
        manipulation_->executeVerticlePath(config_->left_arm_, lift_distance_desired, true);
      ros::Duration(1.0).sleep();
    }
    else
    {
      std::cout << "Moving down ------------------------------------" << std::endl;
      manipulation_->executeVerticlePath(config_->right_arm_, lift_distance_desired, false);
      if (config_->dual_arm_)
        manipulation_->executeVerticlePath(config_->left_arm_, lift_distance_desired, false);
      ros::Duration(1.0).sleep();
    }
    ++i;
  }

  ROS_INFO_STREAM_NAMED("apc_manager","Done testing up and down");
  return true;
}

// Mode 6
bool APCManager::testShelfLocation()
{
  static const double SAFETY_PADDING = -0.23; // Amount to prevent collision with shelf edge
  Eigen::Affine3d ee_pose;

  // Set EE as closed so that we can touch the tip easier
  manipulation_->openEndEffectorWithVelocity(false, config_->right_arm_);
  if (config_->dual_arm_)
    manipulation_->openEndEffectorWithVelocity(false, config_->left_arm_);

  // Reduce collision world to simple
  manipulation_->createCollisionWall();

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

    std::cout << std::endl;
    std::cout << std::endl;
    std::cout << "Waiting before going to next bin" << std::endl;
    remote_control_->waitForNextStep();
  } // end for

  ROS_INFO_STREAM_NAMED("apc_manager","Done testing shelf location");
  return true;
}

// Mode 7
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
    manipulation_->fixCurrentCollisionAndBounds(arm_jmg);

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

}

// Mode 8
bool APCManager::testGoalBinPose()
{
  const robot_model::JointModelGroup* arm_jmg = config_->dual_arm_ ? config_->both_arms_ : config_->right_arm_;

  // Close end effector
  if (!manipulation_->openEndEffectorWithVelocity(false, arm_jmg))
  {
    ROS_ERROR_STREAM_NAMED("apc_manager","Unable to close end effector");
    return false;
  }

  // Go to dropoff position
  if (!placeObjectInGoalBin(config_->right_arm_))
    return false;

  // Lower down into bin
  if (config_->dual_arm_)
    if (!placeObjectInGoalBin(config_->left_arm_))
      return false;

  // Open end effector
  if (!manipulation_->openEndEffectorWithVelocity(true, arm_jmg))
  {
    ROS_ERROR_STREAM_NAMED("apc_manager","Unable to open end effector");
    return false;
  }

  ROS_INFO_STREAM_NAMED("apc_manager","Done going to goal bin pose");
  return true;
}

// Mode 9
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

// Mode 10
bool APCManager::testRandomValidMotions()
{
  visualizeShelf();

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
          arm_jmg == config_->left_arm_;

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

// Mode 11
bool APCManager::testCameraPositions()
{
  std::size_t bin_skipper = 0;
  for (BinObjectMap::const_iterator bin_it = shelf_->getBins().begin(); bin_it != shelf_->getBins().end(); bin_it++)
  {
    bool process_all_bins = false;
    if (process_all_bins)
    {
      std::cout << std::endl;
      std::cout << std::endl;
      std::cout << "Waiting before moving to next bin" << std::endl;
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

// Mode 12
bool APCManager::calibrateCamera()
{
  ROS_DEBUG_STREAM_NAMED("apc_manager","Calibrating camera");

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

// Mode 13
bool APCManager::recordCalibrationTrajectory()
{
  ROS_INFO_STREAM_NAMED("apc_manager","Recoding calibration trajectory");

  std::string file_path;
  const std::string file_name = "calibration_trajectory";
  manipulation_->getFilePath(file_path, file_name);

  // Start recording
  manipulation_->recordTrajectoryToFile(file_path);

  ROS_INFO_STREAM_NAMED("apc_manager","Done recording calibration trajectory");

  return true;
}

// Mode 14
bool APCManager::testGoHome()
{
  ROS_DEBUG_STREAM_NAMED("apc_manager","Going home");

  // Choose which planning group to use
  const robot_model::JointModelGroup* arm_jmg = config_->dual_arm_ ? config_->both_arms_ : config_->right_arm_;
  moveToStartPosition(arm_jmg);
}

// Mode 15
bool APCManager::testGraspGenerator()
{
  // Benchmark runtime
  ros::Time start_time;
  start_time = ros::Time::now();

  // Variables
  moveit::core::RobotStatePtr current_state = manipulation_->getCurrentState();
  moveit::core::RobotStatePtr the_grasp_state(new moveit::core::RobotState(*current_state)); // Allocate robot states
  Eigen::Affine3d global_object_pose;
  const robot_model::JointModelGroup* arm_jmg;
  moveit_grasps::GraspCandidatePtr chosen; // the grasp to use

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
      if (!manipulation_->chooseGrasp(work_order, arm_jmg, chosen, verbose_))
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
        the_grasp_state->setJointGroupPositions(arm_jmg, chosen->grasp_ik_solution_);
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
}

// Mode 16
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
  int first_joint;
  int last_joint;
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

// Mode 17
bool APCManager::testPerceptionComm()
{
  BinObjectPtr& bin = shelf_->getBins()["bin_D"];
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
  ros::Duration(15).sleep();

  // Get result from perception pipeline
  if (!perception_interface_->endPerception(product, bin))
  {
    ROS_ERROR_STREAM_NAMED("apc_manager","End perception failed");
    return false;
  }

  return true;
}

// Mode 18
bool APCManager::recordBinWithCamera(std::size_t bin_id)
{
  return recordBinWithCamera(shelf_->getBin(bin_id));
}

// Mode 19
bool APCManager::perceiveBinWithCamera(std::size_t bin_id)
{
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

  if (!manipulation_->playbackTrajectoryFromFile(file_path, arm_jmg, config_->calibration_velocity_scaling_factor_))
  {
    ROS_ERROR_STREAM_NAMED("apc_manager","Unable to playback " << file_name);
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
  if (!manipulation_->moveCameraToBin(bin))
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

bool APCManager::placeObjectInGoalBin(const robot_model::JointModelGroup* arm_jmg)
{
  if (!moveToDropOffPosition(arm_jmg))
  {
    ROS_ERROR_STREAM_NAMED("apc_manager","Unable to plan to goal bin");
    return false;
  }

  bool lift = false;
  if (!manipulation_->executeVerticlePath(arm_jmg, config_->place_goal_down_distance_desired_, lift))
  {
    ROS_ERROR_STREAM_NAMED("apc_manager","Failed to lower product into goal bin, using distance "
                           << config_->place_goal_down_distance_desired_);
    return false;
  }

  return true;
}

bool APCManager::liftFromGoalBin(const robot_model::JointModelGroup* arm_jmg)
{
  bool lift = true;
  if (!manipulation_->executeVerticlePath(arm_jmg, config_->place_goal_down_distance_desired_, lift))
  {
    ROS_ERROR_STREAM_NAMED("apc_manager","Failed to raise arm back up from goal, using distance "
                           << config_->place_goal_down_distance_desired_);
    return false;
  }

  return true;
}

bool APCManager::moveToStartPosition(const robot_model::JointModelGroup* arm_jmg, bool check_validity)
{
  // Choose which planning group to use
  if (arm_jmg == NULL)
    arm_jmg = config_->dual_arm_ ? config_->both_arms_ : config_->right_arm_;
  return manipulation_->moveToPose(arm_jmg, config_->start_pose_, config_->main_velocity_scaling_factor_, check_validity);
}

bool APCManager::moveToDropOffPosition(const robot_model::JointModelGroup* arm_jmg)
{
  if (arm_jmg == NULL)
    arm_jmg = config_->right_arm_;

  // Choose which pose based on arm
  std::string dropoff_pose;
  if (arm_jmg == config_->right_arm_)
    dropoff_pose = config_->right_arm_dropoff_pose_;
  else
    dropoff_pose = config_->left_arm_dropoff_pose_;

  // Move
  return manipulation_->moveToPose(arm_jmg, dropoff_pose, config_->main_velocity_scaling_factor_);
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
  // Choose file
  AmazonJSONParser parser(verbose_, visuals_);
  // Parse json
  return parser.parse(work_order_file_path, package_path_, shelf_, orders_);
}

bool APCManager::visualizeShelf()
{
  // Show the mesh visualization
  visuals_->visual_tools_display_->deleteAllMarkers(); // clear all old markers
  visuals_->visual_tools_display_->enableBatchPublishing(true);
  shelf_->visualize();
  shelf_->visualizeAxis(visuals_);
  visuals_->visual_tools_display_->triggerBatchPublishAndDisable();

  // Show empty shelf to help in reversing robot arms to initial position
  visuals_->visual_tools_->removeAllCollisionObjects();
  bool just_frame = false;
  bool show_all_products = false;
  shelf_->createCollisionBodies("bin_A", just_frame, show_all_products); // only show the frame
  shelf_->visualizeAxis(visuals_);
  visuals_->visual_tools_->triggerPlanningSceneUpdate();

  // Show the current state just for the heck of it
  publishCurrentState();

  return true;
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
  int counter = 0;
  while( !planning_scene_monitor_->getStateMonitor()->haveCompleteState() && ros::ok() )
  {
    ROS_INFO_STREAM_NAMED("apc_manager","Waiting for complete state...");
    ros::Duration(0.1).sleep();
    ros::spinOnce();

    // Show unpublished joints
    if( counter > 10 )
    {
      planning_scene_monitor_->getStateMonitor()->haveCompleteState( missing_joints );
      for(int i = 0; i < missing_joints.size(); ++i)
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

} // end namespace
