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

namespace picknik_main
{

APCManager::APCManager(bool verbose, std::string order_file_path, bool use_experience, bool show_database)
  : nh_private_("~")
  , verbose_(verbose)
  , autonomous_(false)
  , next_step_ready_(false)
  , is_waiting_(false)
  , fake_perception_(true)
  , find_objects_action_("perception/recognize_objects")
{
  // Load the loader
  robot_model_loader_.reset(new robot_model_loader::RobotModelLoader(ROBOT_DESCRIPTION));

  // Load the robot model
  robot_model_ = robot_model_loader_->getModel(); // Get a shared pointer to the robot

  // Create the planning scene
  planning_scene_.reset(new planning_scene::PlanningScene(robot_model_));

  // Create the planning scene service
  get_scene_service_ = nh_root_.advertiseService(GET_PLANNING_SCENE_SERVICE_NAME, &APCManager::getPlanningSceneService, this);

  // Create tf transformer
  tf_.reset(new tf::TransformListener(nh_private_)); //ros::Duration(10.0)));

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

  // Subscribe to remote control topic
  ROS_DEBUG_STREAM_NAMED("apc_manager","Subscribing to button topics");
  std::size_t queue_size = 10;
  remote_next_control_ = nh_private_.subscribe("/picknik_main/next", queue_size, &APCManager::remoteNextCallback, this);
  remote_run_control_ = nh_private_.subscribe("/picknik_main/run", queue_size, &APCManager::remoteRunCallback, this);

  // Visualize
  ROS_DEBUG_STREAM_NAMED("apc_manager","Visualizing shelf");
  visualizeShelf();

  config_.reset(new ManipulationData(robot_model_));

  // Load grasp data specific to our robot
  if (!grasp_datas_[config_->right_arm_].loadRobotGraspData(nh_private_, config_->right_hand_name_, robot_model_))
    ROS_ERROR_STREAM_NAMED("apc_manager","Unable to load right arm grasp data in namespace " << config_->right_hand_name_);

  if (config_->dual_arm_ && !grasp_datas_[config_->left_arm_].loadRobotGraspData(nh_private_, config_->left_hand_name_, robot_model_))
    ROS_ERROR_STREAM_NAMED("apc_manager","Unable to load left arm grasp data in namespace " << config_->left_hand_name_);

  // Create the pick place pipeline
  manipulation_.reset(new Manipulation(verbose_, visuals_, planning_scene_monitor_, config_, grasp_datas_,
                                       this, shelf_, use_experience, show_database));

  // Do system checks
  checkSystemReady();

  ROS_INFO_STREAM_NAMED("apc_manager","APC Manager Ready.");
}

bool APCManager::checkSystemReady()
{
  ROS_INFO_STREAM_NAMED("apc_manager","Starting system ready check:");

  // Check joint model groups, assuming we are the jaco arm
  if (config_->right_arm_->getVariableCount() != 6)
  {
    ROS_FATAL_STREAM_NAMED("apc_manager","Incorrect number of joints for group " << config_->right_arm_->getName());
    exit(-1);
  }
  const robot_model::JointModelGroup* ee_jmg = grasp_datas_[config_->right_arm_].ee_jmg_;
  if (ee_jmg->getVariableCount() != 3)
  {
    ROS_FATAL_STREAM_NAMED("apc_manager","Incorrect number of joints for group " << ee_jmg->getName());
    exit(-1);
  }

  // Check Perception
  if (!fake_perception_)
  {
    ROS_INFO_STREAM_NAMED("apc_manager","Waiting for find block perception server.");
    find_objects_action_.waitForServer();
  }

  // Check robot state valid
  while (ros::ok() && !manipulation_->checkCurrentCollisionAndBounds(config_->right_arm_))
  {
    ros::Duration(1.0).sleep();
  }

  // Check robot calibrated
  // TODO

  // Check gantry calibrated
  // TODO

  // Check end effectors calibrated
  // TODO

  return true;
}

bool APCManager::setupPlanningScene( const std::string& bin_name )
{
  //TESTING

  // Disable all bins except desired one
  visuals_->visual_tools_->deleteAllMarkers(); // clear all old markers
  visuals_->visual_tools_->removeAllCollisionObjects(); // clear all old collision objects

  // Visualize
  shelf_->createCollisionBodies(bin_name, false);
  visuals_->visual_tools_->triggerPlanningSceneUpdate();
  shelf_->visualizeAxis(visuals_);
  ros::Duration(0.5).sleep();

  return true;
}

void APCManager::remoteNextCallback(const std_msgs::Bool::ConstPtr& msg)
{
  setReadyForNextStep();
}

void APCManager::remoteRunCallback(const std_msgs::Bool::ConstPtr& msg)
{
  setAutonomous();
}

bool APCManager::runOrder(std::size_t order_start, std::size_t jump_to,
                          std::size_t num_orders, bool autonomous)
{
  // Set autonomy
  if (autonomous)
    setAutonomous();

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

bool APCManager::graspObjectPipeline(WorkOrder order, bool verbose, std::size_t jump_to)
{
  // Error check
  if (!order.product_ || !order.bin_)
  {
    ROS_ERROR_STREAM_NAMED("apc_manager","Invalid pointers to product or bin in order");
    return false;
  }

  const robot_model::JointModelGroup* arm_jmg;
  bool execute_trajectory = true;

  moveit::core::RobotStatePtr current_state = manipulation_->getCurrentState();

  // Variables
  moveit_grasps::GraspSolution chosen; // the grasp to use
  moveit::core::RobotStatePtr pre_grasp_state(new moveit::core::RobotState(*current_state)); // Allocate robot states
  moveit::core::RobotStatePtr the_grasp_state(new moveit::core::RobotState(*current_state)); // Allocate robot states
  moveit_msgs::RobotTrajectory approach_trajectory_msg;
  bool wait_for_trajetory = false;
  double desired_lift_distance = 0.05;
  Eigen::Affine3d object_pose;

  // Prevent jump-to errors
  if (jump_to == 3)
  {
    ROS_ERROR_STREAM_NAMED("apc_manager","Cannot jump to step 3 - must start on step 2 to choose grasp");
    jump_to = 0;
  }

  if (!autonomous_)
  {
    visuals_->start_state_->publishRobotState(current_state, rvt::GREEN);
    ROS_INFO_STREAM_NAMED("apc_manager","Waiting for remote control to be triggered to start");
  }

  // Jump to a particular step in the manipulation pipeline
  std::size_t step = jump_to;
  while(ros::ok())
  {

    if (!autonomous_)
    {
      std::cout << std::endl;
      std::cout << std::endl;
      std::cout << "Waiting for step: " << step << std::endl;
      waitForNextStep();
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

        manipulation_->createCollisionWall(); // Reduce collision model to simple wall that prevents Robot from hitting shelf

        // Clear the temporary purple robot state image
        visuals_->visual_tools_->hideRobot();

        // Move
        if (!moveToStartPosition())
        {
          ROS_ERROR_STREAM_NAMED("apc_manager","Unable to move to initial position");
          return false;
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
        break;

        // #################################################################################################################
      case 2: manipulation_->statusPublisher("Finding location of product " + order.product_->getName() + " from " + order.bin_->getName());

        // Create the collision objects
        if (!setupPlanningScene( order.bin_->getName() ))
        {
          ROS_ERROR_STREAM_NAMED("apc_manager","Unable to setup planning scene");
          return false;
        }

        // Move camera to correct shelf
        if (!moveCameraToBin( order.bin_ ))
        {
          ROS_ERROR_STREAM_NAMED("apc_manager","Unable to position camera in front of bin");
          return false;
        }

        // Get object pose
        if (!getObjectPose(object_pose, order, verbose))
        {
          ROS_ERROR_STREAM_NAMED("apc_manager","Unable to get object pose");
          return false;
        }

        break;

        // #################################################################################################################
      case 3: manipulation_->statusPublisher("Get grasp for product " + order.product_->getName() + " from " + order.bin_->getName());

        // Choose which arm to use
        arm_jmg = chooseArm(object_pose);

        // Allow fingers to touch object
        manipulation_->allowFingerTouch(order.product_->getCollisionName(), arm_jmg);

        // Generate and chose grasp
        if (!manipulation_->chooseGrasp(object_pose, arm_jmg, chosen, verbose))
        {
          ROS_ERROR_STREAM_NAMED("apc_manager","No grasps found");
          return false;
        }

        the_grasp_state->setJointGroupPositions(arm_jmg, chosen.grasp_ik_solution_);
        manipulation_->setStateWithOpenEE(true, the_grasp_state);

        if (!autonomous_)
        {
          visuals_->visual_tools_->publishRobotState(the_grasp_state, rvt::PURPLE);
        }

        break;

        // #################################################################################################################
      case 4: manipulation_->statusPublisher("Get pre-grasp by generateApproachPath()");

        // Hide the purple robot
        visuals_->visual_tools_->hideRobot();

        if (!manipulation_->generateApproachPath(arm_jmg, approach_trajectory_msg, pre_grasp_state, the_grasp_state, verbose))
        {
          ROS_ERROR_STREAM_NAMED("apc_manager","Unable to generate straight approach path");
          return false;
        }

        // Visualize trajectory in Rviz display
        visuals_->visual_tools_->publishTrajectoryPath(approach_trajectory_msg, current_state, wait_for_trajetory);

        break;
        //step++;

        // #################################################################################################################
      case 5: // Not implemented

        //break
        step++;

        // #################################################################################################################
      case 6: manipulation_->statusPublisher("Moving to pre-grasp position");

        current_state = manipulation_->getCurrentState();
        manipulation_->setStateWithOpenEE(true, current_state);

        manipulation_->createCollisionWall(); // Reduce collision model to simple wall that prevents Robot from hitting shelf

        if (!manipulation_->move(current_state, pre_grasp_state, arm_jmg, config_->main_velocity_scaling_factor_, verbose, execute_trajectory))
        {
          ROS_ERROR_STREAM_NAMED("apc_manager","Unable to plan");
          return false;
        }
        break;

        // #################################################################################################################
      case 7: manipulation_->statusPublisher("Cartesian move to the-grasp position");

        // Create the collision objects
        if (!setupPlanningScene( order.bin_->getName() ))
        {
          ROS_ERROR_STREAM_NAMED("apc_manager","Unable to setup planning scene");
          return false;
        }

        // Visualize trajectory in Rviz display
        current_state = manipulation_->getCurrentState();
        visuals_->visual_tools_->publishTrajectoryPath(approach_trajectory_msg, current_state, wait_for_trajetory);
        std::cout << "Waiting for next step - DEBUG " << std::endl;
        waitForNextStep();

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

        if (!manipulation_->openEndEffector(false, arm_jmg))
        {
          ROS_ERROR_STREAM_NAMED("apc_manager","Unable to close end effector");
          return false;
        }

        // Attach collision object
        visuals_->visual_tools_->attachCO(order.product_->getCollisionName(), grasp_datas_[arm_jmg].parent_link_name_);

        ROS_INFO_STREAM_NAMED("apc_manager","Waiting " << config_->wait_after_grasp_ << " seconds after grasping");
        ros::Duration(config_->wait_after_grasp_).sleep();
        break;

        // #################################################################################################################
      case 9: manipulation_->statusPublisher("Lifting product UP slightly");

        if (!manipulation_->executeLiftPath(arm_jmg, desired_lift_distance))
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

        if (!moveToDropOffPosition(arm_jmg))
        {
          ROS_ERROR_STREAM_NAMED("apc_manager","Unable to plan");
          return false;
        }
        break;

        // #################################################################################################################
      case 12: manipulation_->statusPublisher("Releasing product");

        if (!manipulation_->openEndEffector(true, arm_jmg))
        {
          ROS_ERROR_STREAM_NAMED("apc_manager","Unable to close end effector");
          return false;
        }

        // Delete from planning scene the product
        shelf_->deleteProduct(order.bin_->getName(), order.product_->getName());

        // Unattach from EE
        visuals_->visual_tools_->cleanupACO( order.product_->getCollisionName() ); // use unique name
        visuals_->visual_tools_->cleanupCO( order.product_->getCollisionName() ); // use unique name
        break;

        // #################################################################################################################
      case 13: manipulation_->statusPublisher("Moving to initial position");

        if (!moveToStartPosition())
        {
          ROS_ERROR_STREAM_NAMED("apc_manager","Unable to move to initial position");
          return false;
        }

        // #################################################################################################################
      default:
        ROS_INFO_STREAM_NAMED("apc_manager","Manipulation pipeline end reached, ending");
        return true;

    } // end switch
    step++;
  } // end for

  return true;
}

bool APCManager::moveCameraToBin(BinObjectPtr bin)
{
  // Create pose to find IK solver
  Eigen::Affine3d ee_pose = bin->getCentroid();
  ee_pose = transform(ee_pose, shelf_->getBottomRight());

  // Move centroid backwards
  ee_pose.translation().x() += config_->camera_x_translation_from_bin_;
  ee_pose.translation().y() += config_->camera_y_translation_from_bin_;
  ee_pose.translation().z() += config_->camera_z_translation_from_bin_;

  // Convert pose that has x arrow pointing to object, to pose that has z arrow pointing towards object and x out in the grasp dir
  ee_pose = ee_pose * Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY());
  ee_pose = ee_pose * Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitZ());

  // Translate to custom end effector geometry
  ee_pose = ee_pose * grasp_datas_[config_->right_arm_].grasp_pose_to_eef_pose_;

  // Customize the direction it is pointing
  // Roll Angle
  ee_pose = ee_pose * Eigen::AngleAxisd(config_->camera_z_rotation_from_standard_grasp_, Eigen::Vector3d::UnitZ());
  // Pitch Angle
  ee_pose = ee_pose * Eigen::AngleAxisd(config_->camera_x_rotation_from_standard_grasp_, Eigen::Vector3d::UnitX());
  // Yaw Angle
  ee_pose = ee_pose * Eigen::AngleAxisd(config_->camera_y_rotation_from_standard_grasp_, Eigen::Vector3d::UnitY());

  return manipulation_->moveEEToPose(ee_pose, config_->main_velocity_scaling_factor_, config_->right_arm_);
}

bool APCManager::calibrateCamera()
{
  ROS_INFO_STREAM_NAMED("apc_manager","Calibrating camera");

  bool result = false; // we want to achieve at least one camera pose

  std::vector<std::string> poses;
  poses.push_back("shelf_calibration1");
  poses.push_back("shelf_calibration2");
  poses.push_back("shelf_calibration3");
  poses.push_back("shelf_calibration4");
  poses.push_back("shelf_calibration5");
  poses.push_back("shelf_calibration6");

  // Move to each pose. The first move is full speed, the others are slow
  double velcity_scaling_factor = config_->main_velocity_scaling_factor_;
  for (std::size_t i = 0; i < poses.size(); ++i)
  {
    ROS_INFO_STREAM_NAMED("apc_manager","Moving to camera pose " << poses[i] << " with scaling factor " << config_->calibration_velocity_scaling_factor_);

    // First one goes fast
    if (i > 0)
      velcity_scaling_factor = config_->calibration_velocity_scaling_factor_;

    if (!manipulation_->moveToPose(config_->right_arm_, poses[i], velcity_scaling_factor))
    {
      ROS_ERROR_STREAM_NAMED("apc_manager","Unable to move to pose " << poses[i]);
      return false;
    }
    else
    {
      result = true;
    }
  }

  return true;
}

bool APCManager::getObjectPose(Eigen::Affine3d& object_pose, WorkOrder order, bool verbose)
{

  // Move camera to the bin
  ROS_INFO_STREAM_NAMED("apc_manager","Moving to bin " << order.bin_->getName());
  if (!moveCameraToBin(order.bin_))
  {
    ROS_ERROR_STREAM_NAMED("apc_manager","Unable to move camera to bin " << order.bin_->getName());
    return false;
  }

  //Move camera left
  ROS_INFO_STREAM_NAMED("apc_manager","Moving camera left distance " << config_->camera_left_distance_);
  bool left = true;
  if (!manipulation_->executeLeftPath(config_->right_arm_, config_->camera_left_distance_, left))
  {
    ROS_ERROR_STREAM_NAMED("apc_manager","Unable to move left");
  }

  // Move camera right
  ROS_INFO_STREAM_NAMED("apc_manager","Moving camera right distance " << config_->camera_left_distance_ * 2.0);
  left = false;
  if (!manipulation_->executeLeftPath(config_->right_arm_, config_->camera_left_distance_ * 2.0, left))
  {
    ROS_ERROR_STREAM_NAMED("apc_manager","Unable to move right");
  }

  // Move back to center
  if (!moveCameraToBin(order.bin_))
  {
    ROS_ERROR_STREAM_NAMED("apc_manager","Unable to move camera to bin " << order.bin_->getName());
    return false;
  }

  // Move camera up
  ROS_INFO_STREAM_NAMED("apc_manager","Lifting camera distance " << config_->camera_lift_distance_);
  if (!manipulation_->executeLiftPath(config_->right_arm_, config_->camera_lift_distance_))
  {
    ROS_ERROR_STREAM_NAMED("apc_manager","Unable to move up");
  }

  // Move camera down
  ROS_INFO_STREAM_NAMED("apc_manager","Lowering camera distance " << config_->camera_lift_distance_ * 2.0);
  bool up = false;
  if (!manipulation_->executeLiftPath(config_->right_arm_, config_->camera_lift_distance_ * 2.0, up))
  {
    ROS_ERROR_STREAM_NAMED("apc_manager","Unable to move down");
  }

  // Tell perception pipeline to detect
  ROS_INFO_STREAM_NAMED("apc_manager","Getting object pose");


  // if (!getObjectPose(object_pose, order, verbose))
  // {
  //   ROS_ERROR_STREAM_NAMED("apc_manager","Failed to get product");
  //   return false;
  // }

  /*
    if (!fake_perception_)
    {
    // Communicate with perception pipeline

    // Setup goal
    picknik_msgs::FindObjectsGoal find_object_goal;
    find_object_goal.desired_object_name = order.product_->getName();

    // Get all of the products in the bin
    order.bin_->getProducts(find_object_goal.expected_objects_names);

    // Get the camera pose
    manipulation_->getCurrentState();
    /// TODO - add a better link
    find_object_goal.camera_pose = visuals_->visual_tools_->convertPose(current_state->getGlobalLinkTransform("jaco2_end_effector"));

    find_objects_action_.sendGoal(find_object_goal);

    // Move camera up
    // ROS_INFO_STREAM_NAMED("apc_manager","Lifting camera distance " << config_->camera_lift_distance_);
    // if (!manipulation_->executeLiftPath(config_->right_arm_, config_->camera_lift_distance_))
    // {
    //   ROS_ERROR_STREAM_NAMED("apc_manager","Unable to move up");
    // }

    // Move camera down
    ROS_INFO_STREAM_NAMED("apc_manager","Lowering camera distance " << config_->camera_lift_distance_ * 2.0);
    bool up = false;
    if (!manipulation_->executeLiftPath(config_->right_arm_, config_->camera_lift_distance_ * 2.0, up))
    {
    ROS_ERROR_STREAM_NAMED("apc_manager","Unable to move down");
    }

    //wait for the action to return
    if (!find_objects_action_.waitForResult(ros::Duration(30.0)))
    {
    ROS_ERROR_STREAM_NAMED("apc_manager","Percetion action did not finish before the time out.");
    return false;
    }

    // Get goal state
    actionlib::SimpleClientGoalState goal_state = find_objects_action_.getState();
    ROS_INFO_STREAM_NAMED("apc_manager","Percetion action finished: " << goal_state.toString());

    // Get result
    picknik_msgs::FindObjectsResultConstPtr perception_result = find_objects_action_.getResult();
    std::cout << "Perception_Result:\n " << *perception_result << std::endl;

    // Set new pose
    order.product_->setBottomRight(visuals_->visual_tools_->convertPose(perception_result->desired_object_pose));

    // Update location visually
    order.product_->visualize(shelf_->getBottomRight());

    }
    else // old method
    {
    ROS_WARN_STREAM_NAMED("apc_manager","Using fake perception system");

    const std::string& coll_obj_name = order.product_->getCollisionName();
    {
    planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_); // Lock planning scene

    collision_detection::World::ObjectConstPtr world_obj;
    world_obj = scene->getWorld()->getObject(coll_obj_name);
    if (!world_obj)
    {
    ROS_ERROR_STREAM_NAMED("apc_manager","Unable to find object " << coll_obj_name << " in planning scene world");
    return false;
    }
    if (!world_obj->shape_poses_.size())
    {
    ROS_ERROR_STREAM_NAMED("apc_manager","Object " << coll_obj_name << " has no shapes!");
    return false;
    }
    if (!world_obj->shape_poses_.size() > 1)
    {
    ROS_WARN_STREAM_NAMED("apc_manager","Unknown situation - object " << coll_obj_name << " has more than one shape");
    }
    object_pose = world_obj->shape_poses_[0];
    }
    }
  */

  return true;
}

bool APCManager::waitForNextStep()
{
  is_waiting_ = true;
  // Wait until next step is ready
  while (!next_step_ready_ && !autonomous_ && ros::ok())
  {
    ros::Duration(0.25).sleep();
  }
  if (!ros::ok())
    return false;
  next_step_ready_ = false;
  is_waiting_ = false;
  return true;
}

const robot_model::JointModelGroup* APCManager::chooseArm(const Eigen::Affine3d& object_pose)
{
  if (!config_->dual_arm_) // jacob
  {
    return config_->right_arm_;
  }
  // Baxter: choose which arm
  else if (object_pose.translation().y() <= 0)
  {
    ROS_INFO_STREAM_NAMED("apc_manager","Using right arm to grasp product");
    return config_->right_arm_;
  }
  else
  {
    ROS_INFO_STREAM_NAMED("apc_manager","Using left arm to grasp product");
    return config_->left_arm_;
  }
}

bool APCManager::moveToStartPosition(const robot_model::JointModelGroup* arm_jmg)
{
  return manipulation_->moveToPose(arm_jmg, config_->start_pose_, config_->main_velocity_scaling_factor_);
}

bool APCManager::moveToDropOffPosition(const robot_model::JointModelGroup* arm_jmg)
{
  return manipulation_->moveToPose(arm_jmg, config_->dropoff_pose_, config_->main_velocity_scaling_factor_);
}

bool APCManager::trainExperienceDatabase()
{
  ROS_ERROR_STREAM_NAMED("temp","disabled");
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
      manipulation_->openEndEffector(open, config_->right_arm_);
      ros::Duration(4.0).sleep();
    }
    else
    {
      std::cout << "Showing open EE of state " << std::endl;
      open = true;
      manipulation_->setStateWithOpenEE(open, current_state);
      visuals_->visual_tools_->publishRobotState(current_state);
      manipulation_->openEndEffector(open, config_->right_arm_);
      ros::Duration(4.0).sleep();
    }
    ++i;
  }

  // EE min approach distance
  //manipulation_->manipulation_->statusPublisher("Testing EE min approach distance");

  ROS_INFO_STREAM_NAMED("apc_manager","Done testing end effectors");
  return true;
}

bool APCManager::testUpAndDown()
{
  // Configure
  const robot_model::JointModelGroup* arm_jmg = config_->right_arm_; // TODO single/dual logic
  double desired_lift_distance = 0.4;

  // Test
  manipulation_->statusPublisher("Testing up and down calculations");
  std::size_t i = 0;
  while (ros::ok())
  {
    std::cout << std::endl << std::endl;
    if (i % 2 == 0)
    {
      std::cout << "Moving up --------------------------------------" << std::endl;
      manipulation_->executeLiftPath(arm_jmg, desired_lift_distance, true);
      ros::Duration(5.0).sleep();
    }
    else
    {
      std::cout << "Moving down ------------------------------------" << std::endl;
      manipulation_->executeLiftPath(arm_jmg, desired_lift_distance, false);
      ros::Duration(5.0).sleep();
    }
    ++i;
  }

  ROS_INFO_STREAM_NAMED("apc_manager","Done testing up and down");
  return true;
}

bool APCManager::testShelfLocation()
{
  double safety_padding = -0.25; // Amount to prevent collision with shelf edge
  double velocity_scaling_factor = 0.5;
  Eigen::Affine3d ee_pose;

  // Set EE as closed so that we can touch the tip easier
  manipulation_->openEndEffector(false, config_->right_arm_);

  // Loop through each bin
  for (BinObjectMap::const_iterator bin_it = shelf_->getBins().begin(); bin_it != shelf_->getBins().end(); bin_it++)
  {
    if (!ros::ok())
      return false;

    ROS_INFO_STREAM_NAMED("apc_manager","Testing bin location of " << bin_it->first);

    // Move to far left front corner of bin
    ee_pose = shelf_->getBottomRight() * bin_it->second->getBottomRight(); // convert to world frame
    ee_pose.translation().y() += bin_it->second->getWidth();
    ee_pose.translation().x() += safety_padding - grasp_datas_[config_->right_arm_].finger_to_palm_depth_;
    std::cout << "Finger to palm depth: " << grasp_datas_[config_->right_arm_].finger_to_palm_depth_ << std::endl;

    // Convert pose that has x arrow pointing to object, to pose that has z arrow pointing towards object and x out in the grasp dir
    ee_pose = ee_pose * Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY());
    ee_pose = ee_pose * Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitZ());

    // Translate to custom end effector geometry
    ee_pose = ee_pose * grasp_datas_[config_->right_arm_].grasp_pose_to_eef_pose_;

    // Visual debug
    visuals_->visual_tools_->publishSphere(ee_pose);

    if (!manipulation_->moveEEToPose(ee_pose, velocity_scaling_factor, config_->right_arm_))
    {
      ROS_ERROR_STREAM_NAMED("apc_manager","Failed to move arm to desired shelf location");
    }

    ros::Duration(1.0).sleep();
  }

  ROS_INFO_STREAM_NAMED("apc_manager","Done testing shelf location");
  return true;
}

bool APCManager::testGoalBinPose()
{

  moveToDropOffPosition();

  ROS_INFO_STREAM_NAMED("apc_manager","Done going to goal bin pose");
  return true;
}

bool APCManager::testInCollision()
{
  while (ros::ok())
  {
    manipulation_->checkCurrentCollisionAndBounds(config_->right_arm_);
    ros::Duration(1).sleep();
  }

  ROS_INFO_STREAM_NAMED("apc_manager","Done checking if in collision");
  return true;
}

bool APCManager::testRandomValidMotions()
{
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
      goal_state->setToRandomPositions(config_->right_arm_);

      // Check if random goal state is valid
      if (manipulation_->checkCollisionAndBounds(current_state, goal_state))
      {
        // Plan to this position
        bool verbose = true;
        bool execute_trajectory = true;
        if (manipulation_->move(current_state, goal_state, config_->right_arm_, config_->main_velocity_scaling_factor_, verbose, execute_trajectory))
        {
          ROS_INFO_STREAM_NAMED("apc_manager","Planned to random valid state successfullly");
          return true;
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
  }

  ROS_INFO_STREAM_NAMED("apc_manager","Done planning to random valid");
  return true;
}

bool APCManager::testCameraPositions()
{
  std::size_t bin_skipper = 0;
  for (BinObjectMap::const_iterator bin_it = shelf_->getBins().begin(); bin_it != shelf_->getBins().end(); bin_it++)
  {
    if (!ros::ok())
      return true;

    // Skip first row
    if (bin_skipper < 3)
    {
      bin_skipper++;
      continue;
    }

    // Get bin and product
    const BinObjectPtr bin = bin_it->second;
    if (bin->getProducts().size() == 0) // Check if there are any objects to get
    {
      ROS_ERROR_STREAM_NAMED("apc_manager","No products in bin");
      return false;
    }
    const ProductObjectPtr product = bin->getProducts()[0]; // Choose first object
    WorkOrder order(bin, product);

    // DEBUG
    if (false)
    {
      Eigen::Affine3d rand_pose;
      geometry_msgs::Pose temp = visuals_->visual_tools_->convertPose(rand_pose);
      visuals_->visual_tools_->generateRandomPose(temp);
      product->setCentroid(rand_pose);
      // Set new pose
      //order.product_->getBottomRight() = visuals_->visual_tools_->convertPose(perception_result->desired_object_pose);

      printTransform(bin->getBottomRight());

      // Update location visually
      product->visualize(bin->getBottomRight());
      break;
    }

    Eigen::Affine3d object_pose;
    bool verbose = true;
    if (!getObjectPose(object_pose, order, verbose))
    {
      ROS_ERROR_STREAM_NAMED("apc_manager","Failed to get product");
      return false;
    }
    return true;
  }

  ROS_INFO_STREAM_NAMED("apc_manager","Done planning to random valid");
  return true;
}


bool APCManager::testCalibration()
{
  calibrateCamera();

  ROS_INFO_STREAM_NAMED("apc_manager","Done calibrating camera");
  return true;
}

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
  getIntParameter(nh_private_, "test/test_joint_limit_joint", test_joint_limit_joint);
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

bool APCManager::getPose()
{
  manipulation_->getSRDFPose(config_->right_arm_);
}

bool APCManager::loadShelfContents(std::string order_file_path)
{
  // Choose file
  AmazonJSONParser parser(verbose_, visuals_);
  // Parse json
  return parser.parse(order_file_path, package_path_, shelf_, orders_);
}

bool APCManager::visualizeShelf()
{
  // Show the mesh visualization
  visuals_->visual_tools_display_->enableBatchPublishing(true);
  shelf_->visualize();
  shelf_->visualizeAxis(visuals_);
  visuals_->visual_tools_display_->triggerBatchPublishAndDisable();

  // Now show empty shelf to help in reversing robot arms to initial position
  visuals_->visual_tools_->removeAllCollisionObjects();
  bool just_frame = false;
  bool show_all_products = true;
  shelf_->createCollisionBodies("", just_frame, show_all_products); // only show the frame
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
  planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor(planning_scene_,
                                                                                 robot_model_loader_,
                                                                                 tf_,
                                                                                 PLANNING_SCENE_MONITOR_NAME));
  ros::spinOnce();

  // Get the joint state topic
  std::string joint_state_topic;
  getStringParameter(nh_private_, "joint_state_topic", joint_state_topic);

  if (planning_scene_monitor_->getPlanningScene())
  {
    // Optional monitors to start:
    bool use_octomap_monitor = false; // this prevents a /tf warning
    //planning_scene_monitor_->startWorldGeometryMonitor(planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
    //                                                   "",
    //planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
    //                                                   use_octomap_monitor);
    //planning_scene_monitor_->startSceneMonitor("/move_group/monitored_planning_scene");
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

bool APCManager::setReadyForNextStep()
{
  if (is_waiting_)
    next_step_ready_ = true;
}

void APCManager::setAutonomous()
{
  autonomous_ = true;
}

bool APCManager::getAutonomous()
{
  return autonomous_;
}

bool APCManager::getPlanningSceneService(moveit_msgs::GetPlanningScene::Request &req, moveit_msgs::GetPlanningScene::Response &res)
{
  if (req.components.components & moveit_msgs::PlanningSceneComponents::TRANSFORMS)
    planning_scene_monitor_->updateFrameTransforms();
  planning_scene_monitor::LockedPlanningSceneRO ps(planning_scene_monitor_);
  ps->getPlanningSceneMsg(res.scene, req.components);
  return true;
}

} // end namespace
