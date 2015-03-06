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

#include <picknik_main/manipulation_pipeline.h>

// MoveIt
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/ompl/model_based_planning_context.h>

#include <ompl/tools/lightning/Lightning.h>

#include <algorithm>

namespace picknik_main
{

ManipulationPipeline::ManipulationPipeline(bool verbose, VisualsPtr visuals,
                                           planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor,
                                           boost::shared_ptr<plan_execution::PlanExecution> plan_execution,
                                           ShelfObjectPtr shelf, bool use_experience, bool show_database)
  : nh_("~")
  , verbose_(verbose)
  , visuals_(visuals)
  , planning_scene_monitor_(planning_scene_monitor)
  , plan_execution_(plan_execution)
  , shelf_(shelf)
  , use_experience_(use_experience)
  , show_database_(show_database)
  , use_logging_(true)
  , use_remote_control_(true)
  , next_step_ready_(false)
{
  // Load performance variables
  getDoubleParameter(nh_, "main_velocity_scaling_factor", main_velocity_scaling_factor_);
  getDoubleParameter(nh_, "approach_velocity_scaling_factor", approach_velocity_scaling_factor_);
  getDoubleParameter(nh_, "lift_velocity_scaling_factor", lift_velocity_scaling_factor_);
  getDoubleParameter(nh_, "retreat_velocity_scaling_factor", retreat_velocity_scaling_factor_);
  getDoubleParameter(nh_, "wait_before_grasp", wait_before_grasp_);
  getDoubleParameter(nh_, "wait_after_grasp", wait_after_grasp_);

  // Load semantics
  getStringParameter(nh_, "start_pose", start_pose_);
  getStringParameter(nh_, "dropoff_pose", dropoff_pose_);
  getStringParameter(nh_, "right_hand_name", right_hand_name_);
  getStringParameter(nh_, "left_hand_name", left_hand_name_);
  getStringParameter(nh_, "right_arm_name", right_arm_name_);
  getStringParameter(nh_, "left_arm_name", left_arm_name_);
  getStringParameter(nh_, "both_arms_name", both_arms_name_);

  // Create initial robot state
  loadRobotStates();

  // Decide where to publish text
  status_position_ = shelf_->bottom_right_;
  bool show_text_for_video = false;
  if (show_text_for_video)
  {
    status_position_.translation().x() = 0.25;
    status_position_.translation().y() += 1.4;
    status_position_.translation().z() += shelf_->getHeight() * 0.75;
  }
  else
  {
    status_position_.translation().x() = 0.25;
    status_position_.translation().y() += shelf_->getWidth() * 0.5;
    status_position_.translation().z() += shelf_->getHeight() * 1.1;
  }
  order_position_ = status_position_;
  order_position_.translation().z() += 0.2;

  // Load grasp data specific to our robot
  if (!grasp_datas_[right_arm_].loadRobotGraspData(nh_, right_hand_name_, robot_model_))
    ROS_ERROR_STREAM_NAMED("pipeline","Unable to load right arm grasp data in namespace " << right_hand_name_);

  if (dual_arm_ && !grasp_datas_[left_arm_].loadRobotGraspData(nh_, left_hand_name_, robot_model_))
    ROS_ERROR_STREAM_NAMED("pipeline","Unable to load left arm grasp data in namespace " << left_hand_name_);

  // Load grasp generator
  grasps_.reset( new moveit_grasps::Grasps(visuals_->visual_tools_) );
  setStateWithOpenEE(true, current_state_); // so that grasp filter is started up with EE open
  grasp_filter_.reset(new moveit_grasps::GraspFilter(current_state_, visuals_->visual_tools_) );


  // Load logging capability
  if (use_logging_ && use_experience)
  {
    /*    if (use_thunder_ && use_experience)
          logging_file_.open("/home/dave/ompl_storage/thunder_whole_body_logging.csv", std::ios::out | std::ios::app);
          else if (use_thunder_ && !use_experience)
          logging_file_.open("/home/dave/ompl_storage/scratch_whole_body_logging.csv", std::ios::out | std::ios::app);
          else*/
    logging_file_.open("/home/dave/ompl_storage/lightning_whole_body_logging.csv", std::ios::out | std::ios::app);
  }

  // Done
  ROS_INFO_STREAM_NAMED("pipeline","Pipeline Ready.");
}

bool ManipulationPipeline::loadRobotStates()
{
  {
    planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_); // Lock planning scene
    current_state_.reset(new moveit::core::RobotState(scene->getCurrentState()));
  }

  visuals_->visual_tools_->getSharedRobotState() = current_state_; // allow visual_tools to have the correct virtual joint
  robot_model_ = current_state_->getRobotModel();

  // Decide what robot we are working with
  if (robot_model_->getName() == "baxter")
  {
    dual_arm_ = true;

    // Load arm groups
    left_arm_ = robot_model_->getJointModelGroup(left_arm_name_);
    right_arm_ = robot_model_->getJointModelGroup(right_arm_name_);
    both_arms_ = robot_model_->getJointModelGroup(both_arms_name_);
  }
  else if (robot_model_->getName() == "jacob")
  {
    dual_arm_ = false;

    // Load arm groups
    right_arm_ = robot_model_->getJointModelGroup(right_arm_name_);
  }
  else
  {
    ROS_WARN_STREAM_NAMED("temp","Unknown type of robot '" << robot_model_->getName() << "'");
    return false;
  }

  return true;
}

bool ManipulationPipeline::setupPlanningScene( const std::string& bin_name )
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

bool ManipulationPipeline::createCollisionWall()
{
  // Disable all bins except desired one
  visuals_->visual_tools_->removeAllCollisionObjects(); // clear all old collision objects

  // Visualize
  shelf_->visualizeAxis(visuals_);
  visuals_->visual_tools_->publishCollisionWall( shelf_->shelf_distance_from_robot_, 0, 0, shelf_->shelf_width_ * 2.0, "SimpleCollisionWall",
                                                 rvt::BROWN );
  shelf_->getGoalBin()->createCollisionBodies(shelf_->bottom_right_);

  visuals_->visual_tools_->triggerPlanningSceneUpdate();
  ros::Duration(0.25).sleep();

  return true;
}

bool ManipulationPipeline::getObjectPose(Eigen::Affine3d& object_pose, WorkOrder order, bool verbose)
{
  // TODO: communicate with perception pipeline
  const std::string& coll_obj_name = order.product_->getCollisionName();
  {
    planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_); // Lock planning scene

    collision_detection::World::ObjectConstPtr world_obj;
    world_obj = scene->getWorld()->getObject(coll_obj_name);
    if (!world_obj)
    {
      ROS_ERROR_STREAM_NAMED("pipeline","Unable to find object " << coll_obj_name << " in planning scene world");
      return false;
    }
    if (!world_obj->shape_poses_.size())
    {
      ROS_ERROR_STREAM_NAMED("pipeline","Object " << coll_obj_name << " has no shapes!");
      return false;
    }
    if (!world_obj->shape_poses_.size() > 1)
    {
      ROS_WARN_STREAM_NAMED("pipeline","Unknown situation - object " << coll_obj_name << " has more than one shape");
    }
    object_pose = world_obj->shape_poses_[0];
  }

  return true;
}

bool ManipulationPipeline::graspObjectPipeline(WorkOrder order, bool verbose, std::size_t jump_to)
{
  // Error check
  if (!order.product_ || !order.bin_)
  {
    ROS_ERROR_STREAM_NAMED("pipeline","Invalid pointers to product or bin in order");
    return false;
  }

  const robot_model::JointModelGroup* arm_jmg;
  bool execute_trajectory = true;

  // Variables
  moveit_grasps::GraspSolution chosen; // the grasp to use
  moveit::core::RobotStatePtr pre_grasp_state(new moveit::core::RobotState(*current_state_)); // Allocate robot states
  moveit::core::RobotStatePtr the_grasp_state(new moveit::core::RobotState(*current_state_)); // Allocate robot states
  moveit_msgs::RobotTrajectory approach_trajectory_msg;
  bool wait_for_trajetory = false;
  double desired_lift_distance = 0.05;
  Eigen::Affine3d object_pose;

  // Prevent jump-to errors
  if (jump_to == 3)
  {
    ROS_ERROR_STREAM_NAMED("pipeline","Cannot jump to step 3 - must start on step 2 to choose grasp");
    jump_to = 0;
  }

  if (use_remote_control_)
  {
    visuals_->start_state_->publishRobotState(current_state_, rvt::GREEN);    
    ROS_INFO_STREAM_NAMED("pipeline","Waiting for remote control to be triggered to start");
  }

  // Jump to a particular step in the manipulation pipeline
  std::size_t step = jump_to;
  while(ros::ok())
  {
    if (use_remote_control_)
    {
      std::cout << std::endl;
      std::cout << std::endl;
      std::cout << "Ready for step: " << step << std::endl;
      // Wait until next step is ready
      while (!next_step_ready_ && use_remote_control_ && ros::ok())
      {
        ros::Duration(0.25).sleep();
      }
      if (!ros::ok())
        return false;
      next_step_ready_ = false;
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

        moveToStartPosition();
        break;

        // #################################################################################################################
      case 1:  statusPublisher("Open end effectors");

        openEndEffectors(true);
        break;

        // #################################################################################################################
      case 2: statusPublisher("Generate and choose grasp for product " + order.product_->getName() +
                              " from " + order.bin_->getName());

        // Create the collision objects
        if (!setupPlanningScene( order.bin_->getName() ))
        {
          ROS_ERROR_STREAM_NAMED("pipeline","Unable to setup planning scene");
          return false;
        }

        // Get object pose
        if (!getObjectPose(object_pose, order, verbose))
        {
          ROS_ERROR_STREAM_NAMED("pipeline","Unable to get object pose");
          return false;
        }

        // Choose which arm to use
        arm_jmg = chooseArm(object_pose);

        // Allow fingers to touch object
        allowFingerTouch(order.product_->getCollisionName(), arm_jmg);

        // Generate and chose grasp
        if (!chooseGrasp(object_pose, arm_jmg, chosen, verbose))
        {
          ROS_ERROR_STREAM_NAMED("pipeline","No grasps found");
          return false;
        }

        the_grasp_state->setJointGroupPositions(arm_jmg, chosen.grasp_ik_solution_);
        setStateWithOpenEE(true, the_grasp_state);

        if (use_remote_control_)
        {
          visuals_->visual_tools_->publishRobotState(the_grasp_state, rvt::PURPLE);
        }

        break;

        // #################################################################################################################
      case 3: // Not implemented

        step++;

        // #################################################################################################################
      case 4: statusPublisher("Get pre-grasp by generateApproachPath()");

        if (use_remote_control_)
        {
          visuals_->visual_tools_->hideRobot();
        }

        if (!generateApproachPath(arm_jmg, approach_trajectory_msg, pre_grasp_state, the_grasp_state, verbose))
        {
          ROS_ERROR_STREAM_NAMED("pipeline","Unable to generate straight approach path");
          return false;
        }
        break;

        // #################################################################################################################
      case 5: // Not implemented

        step++;

        // #################################################################################################################
      case 6: statusPublisher("Moving to pre-grasp position");

        getCurrentState();
        setStateWithOpenEE(true, current_state_);

        if (!move(current_state_, pre_grasp_state, arm_jmg, verbose, execute_trajectory, show_database_))
        {
          ROS_ERROR_STREAM_NAMED("pipeline","Unable to plan");
          return false;
        }
        break;

        // #################################################################################################################
      case 7: statusPublisher("Cartesian move to the-grasp position");

        // Visualize trajectory in Rviz display
        visuals_->visual_tools_->publishTrajectoryPath(approach_trajectory_msg, wait_for_trajetory);

        // Run
        if( !executeTrajectory(approach_trajectory_msg) )
        {
          ROS_ERROR_STREAM_NAMED("pipeline","Failed to execute trajectory");
        }

        ROS_INFO_STREAM_NAMED("pipeline","Waiting " << wait_before_grasp_ << " seconds before grasping");
        ros::Duration(wait_after_grasp_).sleep();
        break;

        // #################################################################################################################
      case 8: statusPublisher("Grasping");

        if (!openEndEffector(false, arm_jmg))
        {
          ROS_ERROR_STREAM_NAMED("pipeline","Unable to close end effector");
          //return false;
        }

        // Attach collision object
        visuals_->visual_tools_->attachCO(order.product_->getCollisionName(), grasp_datas_[arm_jmg].parent_link_name_);

        ROS_INFO_STREAM_NAMED("pipeline","Waiting " << wait_after_grasp_ << " seconds after grasping");
        ros::Duration(wait_after_grasp_).sleep();
        break;

        // #################################################################################################################
      case 9: statusPublisher("Lifting product UP slightly");

        if (!executeLiftPath(arm_jmg, desired_lift_distance))
        {
          ROS_ERROR_STREAM_NAMED("pipeline","Unable to execute retrieval path after grasping");
          return false;
        }
        break;

        // #################################################################################################################
      case 10: statusPublisher("Moving BACK to pre-grasp position (retreat path)");

        if (!executeRetreatPath(arm_jmg))
        {
          ROS_ERROR_STREAM_NAMED("pipeline","Unable to execute retrieval path after grasping");
          return false;
        }
        break;

        // #################################################################################################################
      case 11: statusPublisher("Placing product in bin");

        createCollisionWall(); // Reduce collision model to simple wall that prevents Robot from hitting shelf

        if (!moveToDropOffPosition(arm_jmg))
        {
          ROS_ERROR_STREAM_NAMED("pipeline","Unable to plan");
          return false;
        }
        break;

        // #################################################################################################################
      case 12: statusPublisher("Releasing product");

        if (!openEndEffector(true, arm_jmg))
        {
          ROS_ERROR_STREAM_NAMED("pipeline","Unable to close end effector");
          //return false;
        }

        // Delete from planning scene the product
        shelf_->deleteProduct(order.bin_->getName(), order.product_->getName());

        // Unattach from EE
        visuals_->visual_tools_->cleanupACO( order.product_->getCollisionName() ); // use unique name
        visuals_->visual_tools_->cleanupCO( order.product_->getCollisionName() ); // use unique name
        break;

      // #################################################################################################################
      case 13: statusPublisher("Moving to initial position");

        moveToStartPosition();

        // #################################################################################################################
      default:
        ROS_INFO_STREAM_NAMED("pipeline","Manipulation pipeline end reached, ending");
        return true;

    } // end switch
    step++;
  } // end for

  return true;
}

bool ManipulationPipeline::chooseGrasp(const Eigen::Affine3d& object_pose, const robot_model::JointModelGroup* arm_jmg,
                                       moveit_grasps::GraspSolution& chosen, bool verbose)
{
  const moveit::core::JointModelGroup* ee_jmg = robot_model_->getJointModelGroup(grasp_datas_[arm_jmg].ee_group_name_);

  visuals_->visual_tools_->publishAxis(object_pose);

  // Generate all possible grasps
  std::vector<moveit_msgs::Grasp> possible_grasps;
  grasps_->setVerbose(true);
  double hand_roll = 0;
  grasps_->generateAxisGrasps( object_pose, moveit_grasps::Y_AXIS, moveit_grasps::DOWN, moveit_grasps::HALF, hand_roll,
                               grasp_datas_[arm_jmg], possible_grasps);

  // Visualize
  if (verbose)
  {
    // Visualize animated grasps
    double animation_speed = 0.005;
    ROS_DEBUG_STREAM_NAMED("manipulation.animated_grasps","Showing animated grasps");
    ROS_DEBUG_STREAM_NAMED("manipulation.animated_grasps",
                           (visuals_->visual_tools_->publishAnimatedGrasps(possible_grasps, ee_jmg, animation_speed) ? "Done" : "Failed"));
  }

  // Filter grasps based on IK

  // Filter the grasp for only the ones that are reachable
  bool filter_pregrasps = true;
  std::vector<moveit_grasps::GraspSolution> filtered_grasps;
  if (!grasp_filter_->filterGrasps(possible_grasps, filtered_grasps, filter_pregrasps, arm_jmg))
  {
    ROS_ERROR_STREAM_NAMED("temp","Unable to filter grasps by IK");
    return false;
  }

  // Visualize them
  if (verbose)
  {
    // Visualize valid grasps as arrows with cartesian path as well
    bool show_cartesian_path = false;
    ROS_DEBUG_STREAM_NAMED("manipulation.ik_filtered_grasps", "Showing ik filtered grasps");
    ROS_DEBUG_STREAM_NAMED("manipulation.ik_filtered_grasps",
                           (visualizeGrasps(filtered_grasps, arm_jmg, show_cartesian_path) ? "Done" : "Failed"));
  }

  ROS_INFO_STREAM_NAMED("pipeline","Filtering grasps by collision");

  // Filter grasps based on collision
  getCurrentState();
  setStateWithOpenEE(true, current_state_); // to be passed to the grasp filter
  if (!grasp_filter_->filterGraspsInCollision(filtered_grasps, planning_scene_monitor_, arm_jmg, current_state_, verbose && false))
  {
    ROS_ERROR_STREAM_NAMED("temp","Unable to filter grasps by collision");
    return false;
  }

  // Visualize IK solutions
  double display_time = 0.5;
  ROS_DEBUG_STREAM_NAMED("manipulation.collision_filtered_solutions","Publishing collision filtered solutions");
  ROS_DEBUG_STREAM_NAMED("manipulation.collision_filtered_solutions",
                         visualizeIKSolutions(filtered_grasps, arm_jmg, display_time) ? "Done" : "Failed");

  // Choose grasp
  if (!grasp_filter_->chooseBestGrasp(filtered_grasps, chosen))
  {
    ROS_ERROR_STREAM_NAMED("pipeline","No best grasp found");
    return false;
  }

  return true;
}

const robot_model::JointModelGroup* ManipulationPipeline::chooseArm(const Eigen::Affine3d& object_pose)
{
  if (!dual_arm_) // jacob
  {
    return right_arm_;
  }
  // Baxter: choose which arm
  else if (object_pose.translation().y() <= 0)
  {
    ROS_INFO_STREAM_NAMED("pipeline","Using right arm to grasp product");
    return right_arm_;
  }
  else
  {
    ROS_INFO_STREAM_NAMED("pipeline","Using left arm to grasp product");
    return left_arm_;
  }
}

bool ManipulationPipeline::moveToStartPosition(const robot_model::JointModelGroup* arm_jmg)
{
  return moveToPose(arm_jmg, start_pose_);
}

bool ManipulationPipeline::moveToDropOffPosition(const robot_model::JointModelGroup* arm_jmg)
{
  return moveToPose(arm_jmg, dropoff_pose_);
}

bool ManipulationPipeline::moveToPose(const robot_model::JointModelGroup* arm_jmg, const std::string &pose_name)
{
  // Get default arm(s) to move
  if (!arm_jmg)
    if (dual_arm_)
      arm_jmg = both_arms_;
    else
      arm_jmg = right_arm_;

  // Set new state to current state
  getCurrentState();

  // Set goal state to initial pose
  moveit::core::RobotStatePtr new_state(new moveit::core::RobotState(*current_state_)); // Allocate robot states
  if (!new_state->setToDefaultValues(arm_jmg, pose_name))
  {
    ROS_ERROR_STREAM_NAMED("pipeline","Failed to set pose '" << pose_name << "' for planning group '" << arm_jmg->getName() << "'");
    return false;
  }

  // Plan
  bool execute_trajectory = true;
  if (!move(current_state_, new_state, arm_jmg, verbose_, execute_trajectory, show_database_))
  {
    ROS_ERROR_STREAM_NAMED("pipeline","Unable to move to new position");
    return false;
  }

  // Open gripper
  // TODO jmg might be both arms openEndEffector(true, jmg);

  return true;
}

bool ManipulationPipeline::getSRDFPose(const robot_model::JointModelGroup* jmg)
{
  // Get default arm(s) to move
  if (!jmg)
    if (dual_arm_)
      jmg = both_arms_;
    else
      jmg = right_arm_;

  const std::vector<const moveit::core::JointModel*> joints = jmg->getJointModels();

  while(ros::ok())
  {
    ROS_INFO_STREAM("SDF Code for joint values pose:\n");

    // Get current state after grasping
    getCurrentState();

    // Output XML
    std::cout << "<group_state name=\"\" group=\"" << jmg->getName() << "\">\n";
    for (std::size_t i = 0; i < joints.size(); ++i)
    {
      std::cout << "  <joint name=\"" << joints[i]->getName() <<"\" value=\"" 
                << current_state_->getJointPositions(joints[i])[0] << "\" />\n";
    }
    std::cout << "</group_state>\n\n\n\n";

    ros::Duration(4.0).sleep();
  }
}

bool ManipulationPipeline::move(const moveit::core::RobotStatePtr& start, const moveit::core::RobotStatePtr& goal,
                                const robot_model::JointModelGroup* arm_jmg, bool verbose, bool execute_trajectory,
                                bool show_database)
{
  if (verbose)
  {
    visuals_->start_state_->publishRobotState(start, rvt::GREEN);
    visuals_->goal_state_->publishRobotState(goal, rvt::ORANGE);
  }

  // Check if already in new position
  if (statesEqual(*start, *goal, arm_jmg))
  {
    ROS_INFO_STREAM_NAMED("pipeline","Not planning motion because current state and goal state are close enough.");
    return true;
  }

  // Create motion planning request
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;

  moveit::core::robotStateToRobotStateMsg(*start, req.start_state);

  // Create Goal constraint
  double tolerance_pose = 0.0001;
  moveit_msgs::Constraints goal_constraint = kinematic_constraints::constructGoalConstraints(*goal, arm_jmg,
                                                                                             tolerance_pose, tolerance_pose);
  req.goal_constraints.push_back(goal_constraint);

  // Other settings e.g. OMPL
  req.planner_id = "RRTConnectkConfigDefault";
  //req.planner_id = "RRTstarkConfigDefault";
  req.group_name = arm_jmg->getName();
  req.num_planning_attempts = 1; // this must be one else it threads and doesn't use lightning/thunder correctly
  req.allowed_planning_time = 30; // seconds
  req.use_experience = use_experience_;
  req.experience_method = "lightning";
  req.max_velocity_scaling_factor = main_velocity_scaling_factor_;

  // Parameters for the workspace that the planner should work inside relative to center of robot
  double workspace_size = 1;
  req.workspace_parameters.header.frame_id = robot_model_->getModelFrame();
  req.workspace_parameters.min_corner.x = start->getVariablePosition("virtual_joint/trans_x") - workspace_size;
  req.workspace_parameters.min_corner.y = start->getVariablePosition("virtual_joint/trans_y") - workspace_size;
  req.workspace_parameters.min_corner.z = 0; //floor start->getVariablePosition("virtual_joint/trans_z") - workspace_size;
  req.workspace_parameters.max_corner.x = start->getVariablePosition("virtual_joint/trans_x") + workspace_size;
  req.workspace_parameters.max_corner.y = start->getVariablePosition("virtual_joint/trans_y") + workspace_size;
  req.workspace_parameters.max_corner.z = start->getVariablePosition("virtual_joint/trans_z") + workspace_size;

  //visuals_->visual_tools_->publishWorkspaceParameters(req.workspace_parameters);

  // Call pipeline
  std::vector<std::size_t> dummy;
  planning_interface::PlanningContextPtr planning_context_handle;

  // SOLVE
  loadPlanningPipeline(); // always call before using generatePlan()
  planning_scene::PlanningScenePtr cloned_scene;
  {
    planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_); // Lock planning scene
    cloned_scene = planning_scene::PlanningScene::clone(scene);
  }
  planning_pipeline_->generatePlan(cloned_scene, req, res, dummy, planning_context_handle);

  // Check that the planning was successful
  bool error = false;
  if(res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_ERROR_STREAM_NAMED("pipeline","Could not compute plan successfully");
    error = true;
    if (verbose)
      ROS_INFO_STREAM_NAMED("pipeline","Attempting to visualize trajectory anyway...");
    else
      return false;
  }

  // Get the trajectory
  moveit_msgs::MotionPlanResponse response;
  response.trajectory = moveit_msgs::RobotTrajectory();
  res.getMessage(response);

  // Visualize trajectory in Rviz display
  bool wait_for_trajetory = false;
  visuals_->visual_tools_->publishTrajectoryPath(response.trajectory, wait_for_trajetory);

  // Focus on execution (unless we are in debug mode)
  if (!error)
    visuals_->visual_tools_->hideRobot();

  // Execute trajectory
  if (execute_trajectory)
  {
    if( !executeTrajectory(response.trajectory) )
    {
      ROS_ERROR_STREAM_NAMED("pipeline","Failed to execute trajectory");
      return false;
    }
  }
  else
  {
    ROS_WARN_STREAM_NAMED("pipeline","Execute trajectory currently disabled by user");
  }

  // Save Experience Database
  if (use_experience_)
  {
    moveit_ompl::ModelBasedPlanningContextPtr mbpc
      = boost::dynamic_pointer_cast<moveit_ompl::ModelBasedPlanningContext>(planning_context_handle);
    ompl::tools::ExperienceSetupPtr experience_setup
      = boost::dynamic_pointer_cast<ompl::tools::ExperienceSetup>(mbpc->getOMPLSimpleSetup());


    // Display logs
    experience_setup->printLogs();

    // Logging
    if (use_logging_)
    {
      experience_setup->saveDataLog(logging_file_);
      logging_file_.flush();
    }

    // Save database
    ROS_INFO_STREAM_NAMED("pipeline","Saving experience db...");
    experience_setup->saveIfChanged();

    // Show the database
    if (show_database)
    {
      ROS_ERROR_STREAM_NAMED("pipeline","Showing database...");
      displayLightningPlans(experience_setup, arm_jmg);
    }
  }

  if (error)
  {
    return false;
  }
  return true;
}

bool ManipulationPipeline::testEndEffectors(bool open)
{
  setStateWithOpenEE(open, current_state_);
  visuals_->visual_tools_->publishRobotState(current_state_);
  openEndEffector(open, right_arm_);
  if (dual_arm_)
    openEndEffector(open, left_arm_);
}

bool ManipulationPipeline::testUpAndDown()
{
  // Configure
  const robot_model::JointModelGroup* arm_jmg = right_arm_; // TODO single/dual logic
  double desired_lift_distance = 0.4;

  // Test
  statusPublisher("Testing up and down calculations");
  std::size_t i = 0;
  while (ros::ok())
  {
    std::cout << std::endl << std::endl;
    if (i % 2 == 0)
    {
      std::cout << "Moving up --------------------------------------" << std::endl;
      executeLiftPath(arm_jmg, desired_lift_distance, true);
      ros::Duration(0.0).sleep();
    }
    else
    {
      std::cout << "Moving down ------------------------------------" << std::endl;
      executeLiftPath(arm_jmg, desired_lift_distance, false);
      ros::Duration(0.0).sleep();
    }
    ++i;
  }
}

bool ManipulationPipeline::executeState(const moveit::core::RobotStatePtr robot_state, const moveit::core::JointModelGroup *arm_jmg)
{
  // Convert state to trajecotry
  robot_trajectory::RobotTrajectoryPtr robot_trajectory(new robot_trajectory::RobotTrajectory(robot_model_, arm_jmg));
  double duration_from_previous = 1;
  robot_trajectory->addSuffixWayPoint(robot_state, duration_from_previous);

  // Convert trajectory to a message
  moveit_msgs::RobotTrajectory trajectory_msg;
  robot_trajectory->getRobotTrajectoryMsg(trajectory_msg);

  // Execute
  if( !executeTrajectory(trajectory_msg) )
  {
    ROS_ERROR_STREAM_NAMED("pipeline","Failed to execute trajectory");
  }
}

bool ManipulationPipeline::generateApproachPath(const moveit::core::JointModelGroup *arm_jmg,
                                                moveit_msgs::RobotTrajectory &approach_trajectory_msg,
                                                moveit::core::RobotStatePtr pre_grasp_state,
                                                moveit::core::RobotStatePtr the_grasp_state,
                                                bool verbose)
{
  // Configurations
  Eigen::Vector3d approach_direction;
  approach_direction << -1, 0, 0; // backwards towards robot body
  double desired_approach_distance = grasp_datas_[arm_jmg].finger_to_palm_depth_ + APPROACH_DISTANCE_DESIRED;

  // Show desired distance
  std::vector<robot_state::RobotStatePtr> robot_state_trajectory;
  double path_length;
  bool reverse_path = true;
  if (!computeStraightLinePath( approach_direction, desired_approach_distance, robot_state_trajectory,
                                the_grasp_state, arm_jmg, reverse_path, path_length))
  {
    ROS_ERROR_STREAM_NAMED("pipeline","Error occured while computing straight line path");
    return false;
  }

  // Get approach trajectory message
  if (!convertRobotStatesToTrajectory(robot_state_trajectory, approach_trajectory_msg, arm_jmg, approach_velocity_scaling_factor_))
  {
    ROS_ERROR_STREAM_NAMED("pipeline","Failed to convert to parameterized trajectory");
    return false;
  }

  // Set the pregrasp to be the first state in the trajectory. Copy value, not pointer
  *pre_grasp_state = *robot_state_trajectory.front();

  /*
    if (verbose)
    {
    ROS_INFO_STREAM_NAMED("pipeline","Visualizing pre-grasp");
    visuals_->visual_tools_->publishRobotState(pre_grasp_state, rvt::YELLOW);
    ros::Duration(0.1).sleep();
    }*/

  return true;
}

bool ManipulationPipeline::executeLiftPath(const moveit::core::JointModelGroup *arm_jmg, const double &desired_lift_distance, bool up)
{
  getCurrentState();

  // Clear all collision objects
  visuals_->visual_tools_->removeAllCollisionObjects(); // clear all old collision objects

  // Compute straight line up above grasp
  Eigen::Vector3d approach_direction;
  approach_direction << 0, 0, (up ? 1 : -1); // 1 is up, -1 is down
  std::vector<robot_state::RobotStatePtr> robot_state_trajectory;
  double path_length;
  bool reverse_path = false;
  if (!computeStraightLinePath( approach_direction, desired_lift_distance,
                                robot_state_trajectory, current_state_, arm_jmg, reverse_path, path_length))
  {
    ROS_ERROR_STREAM_NAMED("pipeline","Error occured while computing straight line path");
    return false;
  }

  // Get approach trajectory message
  moveit_msgs::RobotTrajectory cartesian_trajectory_msg;
  if (!convertRobotStatesToTrajectory(robot_state_trajectory, cartesian_trajectory_msg, arm_jmg, lift_velocity_scaling_factor_))
  {
    ROS_ERROR_STREAM_NAMED("pipeline","Failed to convert to parameterized trajectory");
    return false;
  }

  // Visualize trajectory in Rviz display
  bool wait_for_trajetory = false;
  visuals_->visual_tools_->publishTrajectoryPath(cartesian_trajectory_msg, wait_for_trajetory);

  // Execute
  if( !executeTrajectory(cartesian_trajectory_msg) )
  {
    ROS_ERROR_STREAM_NAMED("pipeline","Failed to execute trajectory");
    return false;
  }

  return true;
}

bool ManipulationPipeline::executeRetreatPath(const moveit::core::JointModelGroup *arm_jmg)
{
  // Get current state after grasping
  getCurrentState();

  // Compute straight line in reverse from grasp
  Eigen::Vector3d approach_direction;
  approach_direction << -1, 0, 0; // backwards towards robot body
  double desired_approach_distance = 0.25; //0.15;
  double path_length;
  std::vector<robot_state::RobotStatePtr> robot_state_trajectory;
  bool reverse_path = false;
  if (!computeStraightLinePath( approach_direction, desired_approach_distance,
                                robot_state_trajectory, current_state_, arm_jmg, reverse_path, path_length))
  {
    ROS_ERROR_STREAM_NAMED("pipeline","Error occured while computing straight line path");
    return false;
  }

  // Get approach trajectory message
  moveit_msgs::RobotTrajectory cartesian_trajectory_msg;
  if (!convertRobotStatesToTrajectory(robot_state_trajectory, cartesian_trajectory_msg, arm_jmg, retreat_velocity_scaling_factor_))
  {
    ROS_ERROR_STREAM_NAMED("pipeline","Failed to convert to parameterized trajectory");
    return false;
  }

  // Visualize trajectory in Rviz display
  bool wait_for_trajetory = false;
  visuals_->visual_tools_->publishTrajectoryPath(cartesian_trajectory_msg, wait_for_trajetory);

  // Execute
  if( !executeTrajectory(cartesian_trajectory_msg) )
  {
    ROS_ERROR_STREAM_NAMED("pipeline","Failed to execute trajectory");
    return false;
  }

  return true;
}

bool ManipulationPipeline::computeStraightLinePath( Eigen::Vector3d approach_direction,
                                                    double desired_approach_distance,
                                                    std::vector<robot_state::RobotStatePtr>& robot_state_trajectory,
                                                    robot_state::RobotStatePtr robot_state,
                                                    const moveit::core::JointModelGroup *arm_jmg,
                                                    bool reverse_trajectory,
                                                    double& path_length)
{
  ROS_DEBUG_STREAM_NAMED("pipeline","Computing cartesian path");

  // End effector parent link (arm tip for ik solving)
  const moveit::core::LinkModel *ik_tip_link_model = grasp_datas_[arm_jmg].parent_link_;

  // ---------------------------------------------------------------------------------------------
  // Show desired trajectory in BLACK
  Eigen::Affine3d tip_pose_start = robot_state->getGlobalLinkTransform(ik_tip_link_model);

  // Debug
  if (false)
  {
    std::cout << "Tip Pose Start \n" << tip_pose_start.translation().x() << "\t"
              << tip_pose_start.translation().y()
              << "\t" << tip_pose_start.translation().z() << std::endl;
  }

  if (verbose_)
  {
    visuals_->visual_tools_->publishSphere(tip_pose_start, rvt::RED, rvt::LARGE);

    Eigen::Affine3d tip_pose_end; // = tip_pose_start;
    //tip_pose_end.translation().x() -= desired_approach_distance;

    straightProjectPose( tip_pose_start, tip_pose_end, approach_direction, desired_approach_distance);

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
  double max_step = 0.01; // 0.01 // The maximum distance in Cartesian space between consecutive points on the resulting path

  // Error check
  if (desired_approach_distance < max_step)
  {
    ROS_ERROR_STREAM_NAMED("pipeline","desired_approach_distance (" << desired_approach_distance << ")  < max_step (" << max_step << ")");
    return false;
  }

  // Jump threshold for preventing consequtive joint values from 'jumping' by a large amount in joint space
  double jump_threshold = 0.0; // disabled

  bool collision_checking_verbose = false;

  // Check for kinematic solver
  if( !arm_jmg->canSetStateFromIK( ik_tip_link_model->getName() ) )
    ROS_ERROR_STREAM_NAMED("pipeline","No IK Solver loaded - make sure moveit_config/kinamatics.yaml is loaded in this namespace");

  {
    // Collision check
    boost::scoped_ptr<planning_scene_monitor::LockedPlanningSceneRO> ls;
    ls.reset(new planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor_));
    robot_state::GroupStateValidityCallbackFn constraint_fn
      = boost::bind(&isStateValid, static_cast<const planning_scene::PlanningSceneConstPtr&>(*ls).get(),
                    collision_checking_verbose, visuals_, _1, _2, _3);

    // -----------------------------------------------------------------------------------------------
    // Compute Cartesian Path
    path_length = robot_state->computeCartesianPath(arm_jmg,
                                                    robot_state_trajectory,
                                                    ik_tip_link_model,
                                                    approach_direction,
                                                    true,           // direction is in global reference frame
                                                    desired_approach_distance,
                                                    max_step,
                                                    jump_threshold,
                                                    constraint_fn // collision check
                                                    );

    ROS_DEBUG_STREAM_NAMED("pipeline","Cartesian resulting distance: " << path_length << " desired: " << desired_approach_distance
                           << " number of states in trajectory: " << robot_state_trajectory.size());

    if( path_length == 0 )
    {
      ROS_ERROR_STREAM_NAMED("pipeline","Failed to computer cartesian path: distance is 0. Displaying collision debug information:");

      // Recreate collision checker callback
      collision_checking_verbose = true;
      constraint_fn = boost::bind(&isStateValid, static_cast<const planning_scene::PlanningSceneConstPtr&>(*ls).get(),
                                  collision_checking_verbose, visuals_, _1, _2, _3);

      // Re-compute Cartesian Path
      path_length = robot_state->computeCartesianPath(arm_jmg,
                                                      robot_state_trajectory,
                                                      ik_tip_link_model,
                                                      approach_direction,
                                                      true,           // direction is in global reference frame
                                                      desired_approach_distance,
                                                      max_step,
                                                      jump_threshold,
                                                      constraint_fn // collision check
                                                      );
      return false;
    }
    else if ( path_length < desired_approach_distance * 0.5 )
    {
      ROS_WARN_STREAM_NAMED("pipeline","Resuling cartesian path distance is less than half the desired distance");
    }
  } // end scoped pointer of locked planning scene

  // Reverse the trajectory if neeeded
  if (reverse_trajectory)
    std::reverse(robot_state_trajectory.begin(), robot_state_trajectory.end());

  // Debug
  if (verbose_)
  {

    //TEMP
    //const Eigen::Affine3d tip_pose_start2 = robot_state_trajectory.front()->getGlobalLinkTransform(ik_tip_link_model);

    // Super debug
    if (false)
    {
      std::cout << "Tip Pose Result: \n";
      for (std::size_t i = 0; i < robot_state_trajectory.size(); ++i)
      {
        const Eigen::Affine3d tip_pose_start = robot_state_trajectory[i]->getGlobalLinkTransform(ik_tip_link_model);
        std::cout << tip_pose_start.translation().x() << "\t" << tip_pose_start.translation().y() <<
          "\t" << tip_pose_start.translation().z() << std::endl;
      }
    }

    // Show actual trajectory in GREEN
    ROS_INFO_STREAM_NAMED("pipeline","Displaying cartesian trajectory in green");
    const Eigen::Affine3d& tip_pose_end =
      robot_state_trajectory.back()->getGlobalLinkTransform(ik_tip_link_model);
    visuals_->visual_tools_->publishLine(tip_pose_start, tip_pose_end, rvt::LIME_GREEN, rvt::LARGE);
    visuals_->visual_tools_->publishSphere(tip_pose_end, rvt::ORANGE, rvt::LARGE);

    // Visualize end effector position of cartesian path
    ROS_INFO_STREAM_NAMED("pipeline","Visualize end effector position of cartesian path");
    visuals_->visual_tools_->publishTrajectoryPoints(robot_state_trajectory, ik_tip_link_model);


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

bool ManipulationPipeline::straightProjectPose( const Eigen::Affine3d& original_pose, Eigen::Affine3d& new_pose,
                                                const Eigen::Vector3d direction, double distance)
{
  // Assume everything is in world coordinates

  new_pose = original_pose;

  Eigen::Vector3d longer_direction = direction * distance;

  new_pose.translation() += longer_direction;

  return true;
}

bool ManipulationPipeline::convertRobotStatesToTrajectory(const std::vector<robot_state::RobotStatePtr>& robot_state_trajectory,
                                                          moveit_msgs::RobotTrajectory& trajectory_msg,
                                                          const robot_model::JointModelGroup* jmg,
                                                          const double &velocity_scaling_factor)
{
  ROS_DEBUG_STREAM_NAMED("pipeline","Converting robot states to trajectory using scaling factor " << velocity_scaling_factor);

  // Copy the vector of RobotStates to a RobotTrajectory
  robot_trajectory::RobotTrajectoryPtr robot_trajectory(new robot_trajectory::RobotTrajectory(robot_model_, jmg));

  // -----------------------------------------------------------------------------------------------
  // Smooth the path and add velocities/accelerations
  //const std::vector<moveit_msgs::JointLimits> &joint_limits = jmg->getVariableLimits();

  for (std::size_t k = 0 ; k < robot_state_trajectory.size() ; ++k)
  {
    double duration_from_previous = 10; //TODO: is this overwritten?
    robot_trajectory->addSuffixWayPoint(robot_state_trajectory[k], duration_from_previous);
  }

  // Perform iterative parabolic smoothing
  trajectory_processing::IterativeParabolicTimeParameterization iterative_smoother;
  iterative_smoother.computeTimeStamps( *robot_trajectory, velocity_scaling_factor );

  // Convert trajectory to a message
  robot_trajectory->getRobotTrajectoryMsg(trajectory_msg);

  return true;
}

bool ManipulationPipeline::openEndEffectors(bool open)
{
  openEndEffector(true, right_arm_);
  if (dual_arm_)
    openEndEffector(true, left_arm_);
  return true;
}

bool ManipulationPipeline::openEndEffector(bool open, const robot_model::JointModelGroup* arm_jmg)
{
  getCurrentState();
  const robot_model::JointModelGroup* ee_jmg = grasp_datas_[arm_jmg].ee_jmg_;

  robot_trajectory::RobotTrajectoryPtr ee_traj(new robot_trajectory::RobotTrajectory(robot_model_, ee_jmg));

  if (open)
  {
    ROS_INFO_STREAM_NAMED("pipeline","Opening end effector for " << grasp_datas_[arm_jmg].ee_group_name_);
    ee_traj->setRobotTrajectoryMsg(*current_state_, grasp_datas_[arm_jmg].pre_grasp_posture_); // open
  }
  else
  {
    ROS_INFO_STREAM_NAMED("pipeline","Closing end effector for " << grasp_datas_[arm_jmg].ee_group_name_);
    ee_traj->setRobotTrajectoryMsg(*current_state_, grasp_datas_[arm_jmg].grasp_posture_); // closed
  }

  // Show the change in end effector
  if (verbose_)
  {
    visuals_->start_state_->publishRobotState(current_state_, rvt::GREEN);
    visuals_->goal_state_->publishRobotState(ee_traj->getLastWayPoint(), rvt::ORANGE);
  }

  // Check if already in new position
  if (statesEqual(*current_state_, ee_traj->getLastWayPoint(), ee_jmg))
  {
    ROS_INFO_STREAM_NAMED("pipeline","Not executing motion because current state and goal state are close enough.");
    return true;
  }

  // Convert trajectory to a message
  moveit_msgs::RobotTrajectory trajectory_msg;
  ee_traj->getRobotTrajectoryMsg(trajectory_msg);

  // Hack to speed up gripping
  // TODO sleep less
  trajectory_msg.joint_trajectory.points.push_back(trajectory_msg.joint_trajectory.points[0]);
  trajectory_msg.joint_trajectory.points.back().time_from_start = ros::Duration(2);

  // Execute trajectory
  if( !executeTrajectory(trajectory_msg) )
  {
    ROS_ERROR_STREAM_NAMED("pipeline","Failed to execute grasp trajectory");
    return false;
  }

  return true;
}

bool ManipulationPipeline::setStateWithOpenEE(bool open, moveit::core::RobotStatePtr robot_state)
{
  if (!dual_arm_) // jacob mode
  {
    if (open)
      grasp_datas_[right_arm_].setRobotStatePreGrasp( robot_state );
    else
      grasp_datas_[right_arm_].setRobotStateGrasp( robot_state );
    /*
    for (std::size_t i = 0; i < grasp_datas_[right_arm_].grasp_posture_.joint_names.size(); ++i)
    {
      if (open)
        robot_state->setVariablePosition(grasp_datas_[right_arm_].pre_grasp_posture_.joint_names[i],
                                         grasp_datas_[right_arm_].pre_grasp_posture_.points.front().positions[i]);
      else
        robot_state->setVariablePosition(grasp_datas_[right_arm_].grasp_posture_.joint_names[i],
                                         grasp_datas_[right_arm_].grasp_posture_.points.front().positions[i]);
    }
    */
  }
  else // Baxter mode
  {
    // TODO replace with method that moveit_grasps uses

    const double& right_open_position  = grasp_datas_[right_arm_].pre_grasp_posture_.points[0].positions[0];
    const double& right_close_position = grasp_datas_[right_arm_].grasp_posture_.points[0].positions[0];
    double left_open_position;
    double left_close_position;
    left_open_position  = grasp_datas_[left_arm_].pre_grasp_posture_.points[0].positions[0];
    left_close_position = grasp_datas_[left_arm_].grasp_posture_.points[0].positions[0];

    if (open)
    {
      robot_state->setVariablePosition("left_gripper_r_finger_joint", left_open_position);
      robot_state->setVariablePosition("left_gripper_l_finger_joint", -left_open_position);
      robot_state->setVariablePosition("right_gripper_r_finger_joint", right_open_position);
      robot_state->setVariablePosition("right_gripper_l_finger_joint", right_open_position);
    }
    else
    {
      robot_state->setVariablePosition("left_gripper_r_finger_joint", left_close_position);
      robot_state->setVariablePosition("left_gripper_l_finger_joint", -left_close_position);

      robot_state->setVariablePosition("right_gripper_r_finger_joint", right_close_position);
      robot_state->setVariablePosition("right_gripper_l_finger_joint", right_close_position);
    }
  } // end baxter mode
}



bool ManipulationPipeline::executeTrajectory(moveit_msgs::RobotTrajectory trajectory_msg)
{
  ROS_INFO_STREAM_NAMED("pipeline","Executing trajectory");

  // Debug
  ROS_DEBUG_STREAM_NAMED("pipeline.trajectory","Executing trajectory:\n" << trajectory_msg);

  // Clear
  plan_execution_->getTrajectoryExecutionManager()->clear();

  if(plan_execution_->getTrajectoryExecutionManager()->push(trajectory_msg))
  {
    plan_execution_->getTrajectoryExecutionManager()->execute();

    bool wait_exection = true;
    if (wait_exection)
    {
      // wait for the trajectory to complete
      moveit_controller_manager::ExecutionStatus es = plan_execution_->getTrajectoryExecutionManager()->waitForExecution();
      if (es == moveit_controller_manager::ExecutionStatus::SUCCEEDED)
        ROS_DEBUG_STREAM_NAMED("pipeline","Trajectory execution succeeded");
      else
      {
        if (es == moveit_controller_manager::ExecutionStatus::PREEMPTED)
          ROS_INFO_STREAM_NAMED("pipeline","Trajectory execution preempted");
        else
          if (es == moveit_controller_manager::ExecutionStatus::TIMED_OUT)
            ROS_INFO_STREAM_NAMED("pipeline","Trajectory execution timed out");
          else
            ROS_INFO_STREAM_NAMED("pipeline","Trajectory execution control failed");
        return false;
      }
    }
  }
  else
  {
    ROS_ERROR_STREAM_NAMED("pipeline","Failed to execute trajectory");
    return false;
  }

  return true;
}

bool ManipulationPipeline::allowFingerTouch(const std::string& object_name, const robot_model::JointModelGroup* arm_jmg)
{
  // TODO does this reset properly i.e. clear the matrix?

  // Error check
  if (arm_jmg != right_arm_ && arm_jmg != left_arm_)
  {
    ROS_ERROR_STREAM_NAMED("pipeline","Unknown joint model group passed to allowFingerTouch");
    if (arm_jmg)
      ROS_ERROR_STREAM_NAMED("pipeline","Joint model group: " << arm_jmg->getName());
    return false;
  }

  // Lock planning scene
  {
    planning_scene_monitor::LockedPlanningSceneRW scene(planning_scene_monitor_); // Lock planning scene

    // Get links of end effector
    const std::vector<std::string> &ee_link_names = grasp_datas_[arm_jmg].ee_jmg_->getLinkModelNames();

    // Prevent fingers from causing collision with object
    for (std::size_t i = 0; i < ee_link_names.size(); ++i)
    {
      ROS_DEBUG_STREAM_NAMED("pipeline.collision_matrix","Prevent collision between " << object_name << " and " << ee_link_names[i]);
      scene->getAllowedCollisionMatrixNonConst().setEntry(object_name, ee_link_names[i], true);
    }

    // Prevent object from causing collision with shelf
    for (std::size_t i = 0; i < shelf_->getShelfParts().size(); ++i)
    {
      ROS_DEBUG_STREAM_NAMED("pipeline.collision_matrix","Prevent collision between " << object_name << " and " << shelf_->getShelfParts()[i].getName());
      scene->getAllowedCollisionMatrixNonConst().setEntry(object_name, shelf_->getShelfParts()[i].getName(), true);
    }
  } // end lock planning scene

  // Debug current matrix
  if (false)
  {
    moveit_msgs::AllowedCollisionMatrix msg;
    planning_scene_monitor_->getPlanningScene()->getAllowedCollisionMatrix().getMessage(msg);
    std::cout << "Current collision matrix: " << msg << std::endl;
  }

  return true;
}

void ManipulationPipeline::loadPlanningPipeline()
{
  if (!planning_pipeline_)
  {
    // Setup planning pipeline
    planning_pipeline_.reset(new planning_pipeline::PlanningPipeline(robot_model_, nh_, "planning_plugin", "request_adapters"));
  }
}

bool ManipulationPipeline::statusPublisher(const std::string &status)
{
  std::cout << std::endl << std::endl;
  ROS_INFO_STREAM_NAMED("pipeline.status", status << " -------------------------------------");
  visuals_->visual_tools_->publishText(status_position_, status, rvt::WHITE, rvt::LARGE);
}

bool ManipulationPipeline::orderPublisher(WorkOrder& order)
{
  const std::string status = order.bin_->getName() + ": " + order.product_->getName();
  visuals_->visual_tools_->publishText(status_position_, status, rvt::WHITE, rvt::LARGE);
}

bool ManipulationPipeline::statesEqual(const moveit::core::RobotState &s1, const moveit::core::RobotState &s2,
                                       const robot_model::JointModelGroup* arm_jmg)
{
  static const double STATES_EQUAL_THRESHOLD = 0.01;

  double s1_vars[arm_jmg->getActiveJointModels().size()];
  double s2_vars[arm_jmg->getActiveJointModels().size()];
  s1.copyJointGroupPositions(arm_jmg, s1_vars);
  s2.copyJointGroupPositions(arm_jmg, s2_vars);

  for (std::size_t i = 0; i < arm_jmg->getActiveJointModels().size(); ++i)
  {
    //std::cout << "Diff of " << i << " - " << fabs(s1_vars[i] - s2_vars[i]) << std::endl;
    if ( fabs(s1_vars[i] - s2_vars[i]) > STATES_EQUAL_THRESHOLD )
    {
      return false;
    }
  }

  return true;
}

void ManipulationPipeline::displayLightningPlans(ompl::tools::ExperienceSetupPtr experience_setup,
                                                 const robot_model::JointModelGroup* arm_jmg)
{
  // Create a state space describing our robot's planning group
  moveit_ompl::ModelBasedStateSpacePtr model_state_space
    = boost::dynamic_pointer_cast<moveit_ompl::ModelBasedStateSpace>(experience_setup->getStateSpace());

  //ROS_DEBUG_STREAM_NAMED("pipeline","Model Based State Space has dimensions: " << model_state_space->getDimension());

  // Load lightning and its database
  ompl::tools::LightningPtr lightning = boost::dynamic_pointer_cast<ompl::tools::Lightning>(experience_setup);
  //7lightning.setFile(arm_jmg->getName());

  // Get all of the paths in the database
  std::vector<ompl::base::PlannerDataPtr> paths;
  lightning->getAllPlannerDatas(paths);

  ROS_INFO_STREAM_NAMED("pipeline","Number of paths to publish: " << paths.size());

  // Load the OMPL visualizer
  if (!ompl_visual_tools_)
  {
    ompl_visual_tools_.reset(new ovt::OmplVisualTools(robot_model_->getModelFrame(),
                                                      "/ompl_experience_database", planning_scene_monitor_));
    ompl_visual_tools_->loadRobotStatePub("/picknik_amazon");
  }
  ompl_visual_tools_->deleteAllMarkers(); // clear all old markers
  ompl_visual_tools_->setStateSpace(model_state_space);

  // Get tip links for this setup
  std::vector<const robot_model::LinkModel*> tips;
  arm_jmg->getEndEffectorTips(tips);
  ROS_INFO_STREAM_NAMED("pipeline","Found " << tips.size() << " tips");

  bool show_trajectory_animated = false;//verbose_;

  // Loop through each path
  for (std::size_t path_id = 0; path_id < paths.size(); ++path_id)
  {
    std::cout << "Processing path " << path_id << std::endl;
    ompl_visual_tools_->publishRobotPath(paths[path_id], arm_jmg, tips, show_trajectory_animated);
  }

}

bool ManipulationPipeline::visualizeGrasps(std::vector<moveit_grasps::GraspSolution> filtered_grasps,
                                           const moveit::core::JointModelGroup *arm_jmg, bool show_cartesian_path)
{
  ROS_INFO_STREAM_NAMED("pipeline","Showing valid filtered grasp poses");

  // Publish in batch
  //visuals_->visual_tools_->enableBatchPublishing(true);

  // Get the-grasp
  moveit::core::RobotStatePtr the_grasp_state(new moveit::core::RobotState(*current_state_));

  Eigen::Vector3d approach_direction;
  approach_direction << -1, 0, 0; // backwards towards robot body
  double desired_approach_distance = 0.45; //0.12; //0.15;
  std::vector<robot_state::RobotStatePtr> robot_state_trajectory;
  double path_length;
  double max_path_length = 0; // statistics
  bool reverse_path = false;
  for (std::size_t i = 0; i < filtered_grasps.size(); ++i)
  {
    if (!ros::ok())
      return false;

    if (show_cartesian_path)
    {
      the_grasp_state->setJointGroupPositions(arm_jmg, filtered_grasps[i].grasp_ik_solution_);

      if (!computeStraightLinePath(approach_direction, desired_approach_distance,
                                   robot_state_trajectory, the_grasp_state, arm_jmg, reverse_path, path_length))
      {
        ROS_WARN_STREAM_NAMED("pipeline","Unable to find straight line path");
      }

      // Statistics
      if (path_length > max_path_length)
        max_path_length = path_length;

      bool blocking = false;
      double speed = 0.01;
      visuals_->visual_tools_->publishTrajectoryPath(robot_state_trajectory, arm_jmg, speed, blocking);
    }
    grasps_->publishGraspArrow(filtered_grasps[i].grasp_.grasp_pose.pose, grasp_datas_[arm_jmg], rvt::BLUE, path_length);
  }
  //visuals_->visual_tools_->triggerBatchPublishAndDisable();

  ROS_INFO_STREAM_NAMED("learning","Maximum path length in approach trajetory was " << max_path_length);

  return true;
}

bool ManipulationPipeline::visualizeIKSolutions(std::vector<moveit_grasps::GraspSolution> filtered_grasps,
                                                const moveit::core::JointModelGroup* arm_jmg, double display_time)
{
  // Convert the filtered_grasps into a format moveit_visual_tools can use
  std::vector<trajectory_msgs::JointTrajectoryPoint> ik_solutions;
  ik_solutions.resize(filtered_grasps.size());
  for (std::size_t i = 0; i < filtered_grasps.size(); ++i)
  {
    ik_solutions[i].positions = filtered_grasps[i].grasp_ik_solution_;
  }

  return visuals_->visual_tools_->publishIKSolutions(ik_solutions, arm_jmg->getName(), display_time);
}

void ManipulationPipeline::getCurrentState()
{
  planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_); // Lock planning scene
  (*current_state_) = scene->getCurrentState();
}

} // end namespace

namespace
{
bool isStateValid(const planning_scene::PlanningScene *planning_scene, bool verbose,
                  picknik_main::VisualsPtr visuals, robot_state::RobotState *robot_state,
                  const robot_state::JointModelGroup *group, const double *ik_solution)
{
  robot_state->setJointGroupPositions(group, ik_solution);
  robot_state->update();

  if (!planning_scene)
  {
    ROS_ERROR_STREAM_NAMED("pipeline","No planning scene provided");
    return false;
  }
  if (!planning_scene->isStateColliding(*robot_state, group->getName()))
    return true; // not in collision

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
}
