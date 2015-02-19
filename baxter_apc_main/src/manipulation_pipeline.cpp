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

#include <baxter_apc_main/manipulation_pipeline.h>

// MoveIt
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/ompl/model_based_planning_context.h>

#include <ompl/tools/lightning/Lightning.h>

#include <algorithm>

namespace baxter_apc_main
{

ManipulationPipeline::ManipulationPipeline(bool verbose, mvt::MoveItVisualToolsPtr visual_tools,
                                           mvt::MoveItVisualToolsPtr visual_tools_display,
                                           planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor,
                                           ShelfObjectPtr shelf, bool use_experience, bool show_database)
  : nh_("~")
  , verbose_(verbose)
  , visual_tools_(visual_tools)
  , visual_tools_display_(visual_tools_display)
  , planning_scene_monitor_(planning_scene_monitor)
  , shelf_(shelf)
  , use_experience_(use_experience)
  , show_database_(show_database)
  , use_logging_(true)
{

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
  if (!grasp_datas_[left_arm_].loadRobotGraspData(nh_, "left_hand", robot_model_) ||
      !grasp_datas_[right_arm_].loadRobotGraspData(nh_, "right_hand", robot_model_))
  {
    ROS_ERROR_STREAM_NAMED("pipeline","Unable to load grasp data");
  }

  // Load grasp generator
  grasps_.reset( new moveit_grasps::Grasps(visual_tools_) );
  setStateWithOpenEE(true, current_state_); // so that grasp filter is started up with EE open
  grasp_filter_.reset(new moveit_grasps::GraspFilter(current_state_, visual_tools_) );

  // Load trajectory execution. Or, it will do it automatically later
  //loadPlanExecution();

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

  ROS_INFO_STREAM_NAMED("pipeline","Pipeline Ready.");
}

bool ManipulationPipeline::loadRobotStates()
{
  {
    planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_); // Lock planning scene
    current_state_.reset(new moveit::core::RobotState(scene->getCurrentState()));
  }
  visual_tools_->getSharedRobotState() = current_state_; // allow visual_tools to have the correct virtual joint
  robot_model_ = current_state_->getRobotModel();

  if (current_state_->getVariablePosition("virtual_joint/trans_z") != 0.9)
  {
    ROS_ERROR_STREAM_NAMED("pipeline","Robot state z translation should be 0.9!");
    current_state_->setVariablePosition("virtual_joint/trans_z", 0.9);
  }

  // Load arm groups
  left_arm_ = robot_model_->getJointModelGroup("left_arm");
  right_arm_ = robot_model_->getJointModelGroup("right_arm");
  both_arms_ = robot_model_->getJointModelGroup("both_arms");

  // Set baxter to starting position
  /*
  if (!current_state_->setToDefaultValues(both_arms_, START_POSE))
  {
    ROS_ERROR_STREAM_NAMED("pipeline","Failed to set pose '" << START_POSE << "' for planning group '" << both_arms_->getName() << "'");
  }
  */

  return true;
}

bool ManipulationPipeline::setupPlanningScene( const std::string& bin_name )
{
  // Disable all bins except desired one
  visual_tools_->removeAllCollisionObjects(); // clear all old collision objects
  visual_tools_->deleteAllMarkers(); // clear all old markers
  visual_tools_->triggerPlanningSceneUpdate();
  ros::Duration(1.0).sleep(); // TODO combine these two parts into one

  // Visualize
  shelf_->visualizeAxis(visual_tools_);
  shelf_->createCollisionBodies(bin_name, false);
  visual_tools_->triggerPlanningSceneUpdate();
  ros::Duration(1.0).sleep();

  return true;
}

bool ManipulationPipeline::graspObject( WorkOrder order, bool verbose, std::size_t jump_to )
{
  // Error check
  if (!order.product_ || !order.bin_)
  {
    ROS_ERROR_STREAM_NAMED("pipeline","Invalid pointers to product or bin in order");
    return false;
  }

  ROS_INFO_STREAM_NAMED("temp","Removing all collision objects. Resetting planning scene");

  // Feedback
  statusPublisher("Picking " + order.product_->getName() + " from " + order.bin_->getName());

  if (!setupPlanningScene( order.bin_->getName() ))
    ROS_ERROR_STREAM_NAMED("temp","Unable to setup planning scene");

  // Get object pose
  Eigen::Affine3d object_pose;
  const std::string& coll_obj_name = order.product_->getCollisionName();
  {
    planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_); // Lock planning scene

    collision_detection::World::ObjectConstPtr world_obj;
    world_obj = scene->getWorld()->getObject(coll_obj_name);
    if (!world_obj)
    {
      ROS_ERROR_STREAM_NAMED("pipeline","Unable to find object " << coll_obj_name
                             << " in planning scene world");
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

  bool result = graspObjectPipeline(object_pose, order, verbose, jump_to);

  // Delete from planning scene the product
  shelf_->deleteProduct(order.bin_->getName(), order.product_->getName());
  visual_tools_->cleanupACO( order.product_->getCollisionName() ); // use unique name

  return result;
}

bool ManipulationPipeline::graspObjectPipeline(const Eigen::Affine3d& object_pose, WorkOrder order, bool verbose, std::size_t jump_to)
{
  std::cout << "graspObjectPipeline() jump_to: " << jump_to << std::endl;

  const robot_model::JointModelGroup* arm_jmg = chooseArm(object_pose);
  bool execute_trajectory = true;

  // Variables
  moveit_grasps::GraspSolution chosen; // the grasp to use
  moveit::core::RobotStatePtr pre_grasp_state(new moveit::core::RobotState(*current_state_)); // Allocate robot states
  moveit::core::RobotStatePtr the_grasp_state(new moveit::core::RobotState(*current_state_)); // Allocate robot states
  moveit_msgs::RobotTrajectory approach_trajectory_msg;
  bool wait_for_trajetory = false;

  // Prevent jump-to errors
  if (jump_to == 3)
  {
    ROS_ERROR_STREAM_NAMED("temp","Cannot jump to step 3 - must start on step 2 to choose grasp");
    jump_to = 0;
  }


  // Jump to a particular step in the manipulation pipeline
  for (std::size_t step = jump_to; step < 999; ++step) // 100 is just some large number, really it should quit when default is hit
  {
    std::cout << std::endl;
    std::cout << std::endl;
    std::cout << "Running step: " << step << std::endl;

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
      case 2: statusPublisher("Generate and choose grasp");

        if (!chooseGrasp(object_pose, arm_jmg, chosen, verbose))
        {
          ROS_ERROR_STREAM_NAMED("pipeline","No grasps found");
          return false;
        }
        break;

        // #################################################################################################################
      case 3: statusPublisher("Setting the-grasp (actual grasp)");

        the_grasp_state->setJointGroupPositions(arm_jmg, chosen.grasp_ik_solution_);
        setStateWithOpenEE(true, the_grasp_state);

        if (verbose)
        {
          ROS_INFO_STREAM_NAMED("pipeline","Publishing grasp state in purple");
          visual_tools_->publishRobotState(the_grasp_state, rvt::PURPLE);
          ros::Duration(5.0).sleep();
        }
        break;

        // #################################################################################################################
      case 4: statusPublisher("Get pre-grasp by generateApproachPath()");

        if (!generateApproachPath(arm_jmg, approach_trajectory_msg, pre_grasp_state, the_grasp_state, verbose))
        {
          ROS_ERROR_STREAM_NAMED("pipeline","Unable to generate straight approach path");
          return false;
        }
        break;

        // #################################################################################################################
      case 5: statusPublisher("Opening End Effector");

        if (!openEndEffector(true, arm_jmg))
        {
          ROS_ERROR_STREAM_NAMED("pipeline","Unable to open end effector");
          return false;
        }
        break;

        // #################################################################################################################
      case 6: statusPublisher("Moving to pre-grasp position");

        // Get current state
        {
          planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_); // Lock planning scene
          (*current_state_) = scene->getCurrentState();
        }
        setStateWithOpenEE(true, current_state_);

        if (!move(current_state_, pre_grasp_state, arm_jmg, verbose, execute_trajectory, show_database_))
        {
          ROS_ERROR_STREAM_NAMED("pipeline","Unable to plan");
          return false;
        }

        ros::Duration(0.1).sleep();
        break;

        // #################################################################################################################
      case 7: statusPublisher("Cartesian move to the-grasp position");

        // Visualize trajectory in Rviz display
        visual_tools_->publishTrajectoryPath(approach_trajectory_msg, wait_for_trajetory);

        // Run
        if( !executeTrajectoryMsg(approach_trajectory_msg) )
        {
          ROS_ERROR_STREAM_NAMED("pipeline","Failed to execute trajectory");
        }
        break;

        // #################################################################################################################
      case 8: statusPublisher("Grasping");

        if (!openEndEffector(false, arm_jmg))
        {
          ROS_ERROR_STREAM_NAMED("pipeline","Unable to close end effector");
          return false;
        }

        // Attach collision object
        visual_tools_->attachCO(order.product_->getCollisionName(), grasp_datas_[arm_jmg].parent_link_name_);

        // Allow fingers to touch object
        allowFingerTouch(order.product_->getCollisionName(), arm_jmg);

        ros::Duration(0.1).sleep();
        break;

        // #################################################################################################################
      case 9: statusPublisher("Lifting product UP slightly");

        if (!executeLiftPath(arm_jmg))
        {
          ROS_ERROR_STREAM_NAMED("pipeline","Unable to execute retrieval path after grasping");
          return false;
        }

        ros::Duration(5).sleep();
        break;

        // #################################################################################################################
      case 10: statusPublisher("Moving BACK to pre-grasp position");

        if (!executeRetrievalPath(arm_jmg))
        {
          ROS_ERROR_STREAM_NAMED("pipeline","Unable to execute retrieval path after grasping");
          return false;
        }
        break;

        // #################################################################################################################
      case 11: statusPublisher("Moving back to INITIAL position");

        if (!moveToStartPosition(arm_jmg))
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
          return false;
        }

        // Unattach
        visual_tools_->cleanupACO(order.product_->getCollisionName());

        // #################################################################################################################
      default:
        ROS_INFO_STREAM_NAMED("pipeline","Manipulation pipeline end reached, ending");
        return true;

    } // end switch
  } // end for

  return true;
}

bool ManipulationPipeline::chooseGrasp(const Eigen::Affine3d& object_pose, const robot_model::JointModelGroup* arm_jmg,
                                       moveit_grasps::GraspSolution& chosen, bool verbose)
{
  const moveit::core::JointModelGroup* ee_jmg = robot_model_->getJointModelGroup(grasp_datas_[arm_jmg].ee_group_);

  visual_tools_->publishAxis(object_pose);

  // Generate all possible grasps
  std::vector<moveit_msgs::Grasp> possible_grasps;
  grasps_->setVerbose(true);
  grasps_->generateAxisGrasps( object_pose, moveit_grasps::Y_AXIS, moveit_grasps::DOWN, moveit_grasps::HALF, 0,
                               grasp_datas_[arm_jmg], possible_grasps);

  // Visualize
  if (verbose)
  {
    // Visualize animated grasps
    double animation_speed = 0.01;
    ROS_DEBUG_STREAM_NAMED("manipulation.animated_grasps","Showing animated grasps");
    ROS_DEBUG_STREAM_NAMED("manipulation.animated_grasps",
                           (visual_tools_->publishAnimatedGrasps(possible_grasps, ee_jmg, animation_speed) ? "Done" : "Failed"));
  }

  // Filter grasps based on IK

  // Filter the grasp for only the ones that are reachable
  bool filter_pregrasps = true;
  std::vector<moveit_grasps::GraspSolution> filtered_grasps;
  grasp_filter_->filterGrasps(possible_grasps, filtered_grasps, filter_pregrasps,
                              grasp_datas_[arm_jmg].parent_link_name_, arm_jmg);

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
  {
    planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_); // Lock planning scene
    (*current_state_) = scene->getCurrentState();
  }
  setStateWithOpenEE(true, current_state_); // to be passed to the grasp filter
  grasp_filter_->filterGraspsInCollision(filtered_grasps, planning_scene_monitor_, arm_jmg, current_state_, verbose && false);

  // Visualize IK solutions
  ROS_DEBUG_STREAM_NAMED("manipulation.collision_filtered_solutions","Publishing collision filtered solutions");
  ROS_DEBUG_STREAM_NAMED("manipulation.collision_filtered_solutions",
                         visualizeIKSolutions(filtered_grasps, arm_jmg) ? "Done" : "Failed");

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
  if (object_pose.translation().y() > 0)
  {
    ROS_INFO_STREAM_NAMED("pipeline","Using LEFT arm");
    return robot_model_->getJointModelGroup("left_arm");
  }
  else
  {
    ROS_INFO_STREAM_NAMED("pipeline","Using RIGHT arm");
    return robot_model_->getJointModelGroup("right_arm");
  }
}

bool ManipulationPipeline::moveToStartPosition(const robot_model::JointModelGroup* jmg)
{
  // Set start state to current state
  {
    planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_); // Lock planning scene
    (*current_state_) = scene->getCurrentState();
  }

  const robot_model::JointModelGroup* jmg_ready = both_arms_;

  // Set to default group if needed
  if (!jmg)
    jmg = jmg_ready;

  moveit::core::RobotStatePtr start_state(new moveit::core::RobotState(*current_state_)); // Allocate robot states

  // Set goal state to initial pose
  if (!start_state->setToDefaultValues(jmg_ready, START_POSE))
  {
    ROS_ERROR_STREAM_NAMED("pipeline","Failed to set pose '" << START_POSE << "' for planning group '" << jmg_ready->getName() << "'");
  }

  // Check if already in start position
  if (statesEqual(*current_state_, *start_state, jmg))
  {
    ROS_WARN_STREAM_NAMED("pipeline","Not planning motion because current state and goal state are close enough.");
    return true;
  }

  // Plan
  bool execute_trajectory = true;
  if (!move(current_state_, start_state, jmg, verbose_, execute_trajectory, show_database_))
  {
    ROS_ERROR_STREAM_NAMED("pipeline","Unable to move to start position");
    return false;
  }

  // Open gripper
  // TODO jmg might be both arms openEndEffector(true, jmg);

  return true;
}

bool ManipulationPipeline::move(const moveit::core::RobotStatePtr& start, const moveit::core::RobotStatePtr& goal,
                                const robot_model::JointModelGroup* jmg, bool verbose, bool execute_trajectory,
                                bool show_database)
{
  if (verbose)
  {
    ROS_INFO_STREAM_NAMED("temp","Showing start state");
    visual_tools_->publishRobotState(start, rvt::GREEN);
    ros::Duration(1).sleep();

    ROS_INFO_STREAM_NAMED("temp","Showing goal state");
    visual_tools_->publishRobotState(goal, rvt::ORANGE);
    ros::Duration(1).sleep();
  }

  // Create motion planning request
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;

  moveit::core::robotStateToRobotStateMsg(*start, req.start_state);

  // Create Goal constraint
  double tolerance_pose = 0.0001;
  moveit_msgs::Constraints goal_constraint = kinematic_constraints::constructGoalConstraints(*goal, jmg,
                                                                                             tolerance_pose, tolerance_pose);
  req.goal_constraints.push_back(goal_constraint);

  // Other settings e.g. OMPL
  req.planner_id = "RRTConnectkConfigDefault";
  //req.planner_id = "RRTstarkConfigDefault";
  req.group_name = jmg->getName();
  req.num_planning_attempts = 1; // this must be one else it threads and doesn't use lightning/thunder correctly
  req.allowed_planning_time = 30; // seconds
  req.use_experience = use_experience_;
  req.experience_method = "lightning";
  req.max_velocity_scaling_factor = 0.1;

  // Parameters for the workspace that the planner should work inside relative to center of robot
  double workspace_size = 1;
  req.workspace_parameters.header.frame_id = robot_model_->getModelFrame();
  req.workspace_parameters.min_corner.x = start->getVariablePosition("virtual_joint/trans_x") - workspace_size;
  req.workspace_parameters.min_corner.y = start->getVariablePosition("virtual_joint/trans_y") - workspace_size;
  req.workspace_parameters.min_corner.z = 0; //floor start->getVariablePosition("virtual_joint/trans_z") - workspace_size;
  req.workspace_parameters.max_corner.x = start->getVariablePosition("virtual_joint/trans_x") + workspace_size;
  req.workspace_parameters.max_corner.y = start->getVariablePosition("virtual_joint/trans_y") + workspace_size;
  req.workspace_parameters.max_corner.z = start->getVariablePosition("virtual_joint/trans_z") + workspace_size;

  //visual_tools_->publishWorkspaceParameters(req.workspace_parameters);

  // Call pipeline
  std::vector<std::size_t> dummy;
  planning_interface::PlanningContextPtr planning_context_handle;

  // SOLVE
  loadPlanningPipeline(); // always call before using generatePlan()
  ROS_WARN_STREAM_NAMED("temp","Untested scene cloning feature here");
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
  //std::cout << "Trajectory debug:\n " << response.trajectory << std::endl;

  // Visualize trajectory in Rviz display
  bool wait_for_trajetory = false;
  visual_tools_->publishTrajectoryPath(response.trajectory, wait_for_trajetory);

  // Focus on execution (unless we are in debug mode)
  if (!error)
    visual_tools_->hideRobot();

  // Execute trajectory
  if (execute_trajectory)
  {
    if( !executeTrajectoryMsg(response.trajectory) )
    {
      ROS_ERROR_STREAM_NAMED("pipeline","Failed to execute trajectory");
      return false;
    }
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
      displayLightningPlans(experience_setup, jmg);
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
  visual_tools_->publishRobotState(current_state_);
  openEndEffector(open, left_arm_);
  openEndEffector(open, right_arm_);
}

bool ManipulationPipeline::executeState(const moveit::core::RobotStatePtr robot_state, const moveit::core::JointModelGroup *jmg)
{
  // Convert state to trajecotry
  robot_trajectory::RobotTrajectoryPtr robot_trajectory(new robot_trajectory::RobotTrajectory(robot_model_, jmg));
  double duration_from_previous = 1;
  robot_trajectory->addSuffixWayPoint(robot_state, duration_from_previous);

  // Convert trajectory to a message
  moveit_msgs::RobotTrajectory trajectory_msg;
  robot_trajectory->getRobotTrajectoryMsg(trajectory_msg);

  // Execute
  if( !executeTrajectoryMsg(trajectory_msg) )
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
  bool super_verbose = true;

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
  if (!convertRobotStatesToTrajectory(robot_state_trajectory, approach_trajectory_msg, arm_jmg))
  {
    ROS_ERROR_STREAM_NAMED("pipeline","Failed to convert to parameterized trajectory");
    return false;
  }

  // Set the pregrasp to be the first state in the trajectory. Copy value, not pointer
  *pre_grasp_state = *robot_state_trajectory.front();

  if (verbose)
  {
    ROS_INFO_STREAM_NAMED("pipeline","Visualizing pre-grasp");
    visual_tools_->publishRobotState(pre_grasp_state, rvt::YELLOW);
    ros::Duration(5.0).sleep();
  }

  return true;
}

bool ManipulationPipeline::executeLiftPath(const moveit::core::JointModelGroup *arm_jmg)
{
  bool super_verbose = true;

  // Get current state after grasping
  {
    planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_); // Lock planning scene
    (*current_state_) = scene->getCurrentState();
  }

  // Clear all collision objects
  visual_tools_->removeAllCollisionObjects(); // clear all old collision objects

  // Compute straight line up above grasp
  Eigen::Vector3d approach_direction;
  approach_direction << 0, 0, 1; // up over object
  double desired_lift_distance = 0.1; //0.01;
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
  if (!convertRobotStatesToTrajectory(robot_state_trajectory, cartesian_trajectory_msg, arm_jmg))
  {
    ROS_ERROR_STREAM_NAMED("pipeline","Failed to convert to parameterized trajectory");
    return false;
  }

  // Visualize trajectory in Rviz display
  bool wait_for_trajetory = false;
  visual_tools_->publishTrajectoryPath(cartesian_trajectory_msg, wait_for_trajetory);

  std::cout << "trajectory: " << cartesian_trajectory_msg << std::endl;

  // Execute
  if( !executeTrajectoryMsg(cartesian_trajectory_msg) )
  {
    ROS_ERROR_STREAM_NAMED("pipeline","Failed to execute trajectory");
    return false;
  }

  return true;
}

bool ManipulationPipeline::executeRetrievalPath(const moveit::core::JointModelGroup *arm_jmg)
{
  // Get current state after grasping
  {
    planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_); // Lock planning scene
    (*current_state_) = scene->getCurrentState();
  }

  // Compute straight line in reverse from grasp
  Eigen::Vector3d approach_direction;
  approach_direction << -1, 0, 0; // backwards towards robot body
  double desired_approach_distance = 0.15;
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
  if (!convertRobotStatesToTrajectory(robot_state_trajectory, cartesian_trajectory_msg, arm_jmg))
  {
    ROS_ERROR_STREAM_NAMED("pipeline","Failed to convert to parameterized trajectory");
    return false;
  }

  // Visualize trajectory in Rviz display
  bool wait_for_trajetory = false;
  visual_tools_->publishTrajectoryPath(cartesian_trajectory_msg, wait_for_trajetory);

  // Execute
  if( !executeTrajectoryMsg(cartesian_trajectory_msg) )
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
  const Eigen::Affine3d tip_pose_start = robot_state->getGlobalLinkTransform(ik_tip_link_model);

  //TEMP
  visual_tools_->deleteAllMarkers();
  visual_tools_->publishSphere(tip_pose_start, rvt::RED, rvt::LARGE);

  if (verbose_)
  {
    Eigen::Affine3d tip_pose_end; // = tip_pose_start;
    //tip_pose_end.translation().x() -= desired_approach_distance;

    straightProjectPose( tip_pose_start, tip_pose_end, approach_direction, desired_approach_distance);

    visual_tools_->publishLine(tip_pose_start, tip_pose_end, rvt::BLACK, rvt::REGULAR);
  }

  // ---------------------------------------------------------------------------------------------
  // Settings for computeCartesianPath

  // Resolution of trajectory
  double max_step = 0.01; // 0.01 // The maximum distance in Cartesian space between consecutive points on the resulting path

  // Error check
  if (desired_approach_distance < max_step)
  {
    ROS_ERROR_STREAM_NAMED("temp","desired_approach_distance (" << desired_approach_distance << ")  < max_step (" << max_step << ")");
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
                    collision_checking_verbose, visual_tools_, _1, _2, _3);

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
                                  collision_checking_verbose, visual_tools_, _1, _2, _3);

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

    for (std::size_t i = 0; i < robot_state_trajectory.size(); ++i)
    {
      robot_state_trajectory[i]->update();
    }    

    // Show actual trajectory in GREEN
    ROS_INFO_STREAM_NAMED("pipeline","Displaying cartesian trajectory in green");
    const Eigen::Affine3d& tip_pose_end =
      robot_state_trajectory.back()->getGlobalLinkTransform(ik_tip_link_model);
    visual_tools_->publishLine(tip_pose_start, tip_pose_end, rvt::LIME_GREEN, rvt::LARGE);
    visual_tools_->publishSphere(tip_pose_end, rvt::ORANGE, rvt::LARGE);

    // Visualize end effector position of cartesian path
    ROS_INFO_STREAM_NAMED("pipeline","Visualize end effector position of cartesian path");
    visual_tools_->publishTrajectoryPoints(robot_state_trajectory, ik_tip_link_model);

    ros::Duration(1.0).sleep();
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
                                                          const robot_model::JointModelGroup* jmg)
{
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
  double velocity_scale = 0.05;
  iterative_smoother.computeTimeStamps( *robot_trajectory, velocity_scale );

  // Convert trajectory to a message
  robot_trajectory->getRobotTrajectoryMsg(trajectory_msg);

  return true;
}

bool ManipulationPipeline::openEndEffectors(bool open)
{
  openEndEffector(true, left_arm_);
  openEndEffector(true, right_arm_);
  return true;
}

bool ManipulationPipeline::openEndEffector(bool open, const robot_model::JointModelGroup* jmg)
{
  moveit_msgs::RobotTrajectory trajectory_msg;

  robot_state::RobotState ee_state = planning_scene_monitor_->getPlanningScene()->getCurrentState();

  robot_trajectory::RobotTrajectoryPtr ee_traj(new robot_trajectory::RobotTrajectory(ee_state.getRobotModel(),
                                                                                     grasp_datas_[jmg].ee_group_));

  // Convert trajectory to a message
  ee_traj->getRobotTrajectoryMsg(trajectory_msg);

  if (open)
  {
    ROS_INFO_STREAM_NAMED("pipeline","Opening end effector for " << grasp_datas_[jmg].ee_group_);
    ee_traj->setRobotTrajectoryMsg(ee_state, grasp_datas_[jmg].pre_grasp_posture_); // open
  }
  else
  {
    ROS_INFO_STREAM_NAMED("pipeline","Closing end effector for " << grasp_datas_[jmg].ee_group_);
    ee_traj->setRobotTrajectoryMsg(ee_state, grasp_datas_[jmg].grasp_posture_); // closed
  }

  // Apply the open gripper state to the waypoint
  //ee_traj->addPrefixWayPoint(ee_state, DEFAULT_GRASP_POSTURE_COMPLETION_DURATION);

  // Convert trajectory to a message
  ee_traj->getRobotTrajectoryMsg(trajectory_msg);

  // Hack to speed up gripping
  trajectory_msg.joint_trajectory.points[0].time_from_start = ros::Duration(0.1);

  // Execute trajectory
  if( !executeTrajectoryMsg(trajectory_msg) )
  {
    ROS_ERROR_STREAM_NAMED("pipeline","Failed to execute grasp trajectory");
    return false;
  }

  return true;
}

bool ManipulationPipeline::setStateWithOpenEE(bool open, moveit::core::RobotStatePtr robot_state)
{
  const double& left_open_position  = grasp_datas_[left_arm_].pre_grasp_posture_.points[0].positions[0];
  const double& right_open_position  = grasp_datas_[right_arm_].pre_grasp_posture_.points[0].positions[0];

  const double& left_close_position = grasp_datas_[left_arm_].grasp_posture_.points[0].positions[0];
  const double& right_close_position = grasp_datas_[right_arm_].grasp_posture_.points[0].positions[0];

  if (verbose_ && false)
  {
    std::cout << "Setting end effector to open: " << open << std::endl;
    std::cout << "  right_open_position: " << right_open_position << std::endl;
    std::cout << "  left_open_position: " << left_open_position << std::endl;
    std::cout << "  right_close_position: " << right_close_position << std::endl;
    std::cout << "  left_close_position: " << left_close_position << std::endl;
  }

  if (open)
  {
    robot_state->setVariablePosition("left_gripper_r_finger_joint", left_open_position);
    robot_state->setVariablePosition("left_gripper_l_finger_joint", -left_open_position);
    // Specific to Yale-Arm:
    robot_state->setVariablePosition("right_gripper_r_finger_joint", right_open_position);
    robot_state->setVariablePosition("right_gripper_l_finger_joint", right_open_position);
  }
  else
  {
    robot_state->setVariablePosition("left_gripper_r_finger_joint", left_close_position);
    robot_state->setVariablePosition("left_gripper_l_finger_joint", -left_close_position);
    // Specific to Yale-Arm:
    robot_state->setVariablePosition("right_gripper_r_finger_joint", right_close_position);
    robot_state->setVariablePosition("right_gripper_l_finger_joint", right_close_position);
  }
}



bool ManipulationPipeline::executeTrajectoryMsg(moveit_msgs::RobotTrajectory trajectory_msg)
{
  ROS_INFO_STREAM_NAMED("pipeline","Executing trajectory");

  // Load if necessary
  if( !trajectory_execution_manager_ )
    loadPlanExecution();

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

bool ManipulationPipeline::allowFingerTouch(const std::string& object_name, const robot_model::JointModelGroup* jmg)
{
  {
    planning_scene_monitor::LockedPlanningSceneRW scene(planning_scene_monitor_); // Lock planning scene

    // Prevent fingers from causing collision with object
    if (jmg == left_arm_)
    {
      scene->getAllowedCollisionMatrixNonConst().setEntry(object_name, "left_gripper_r_finger", true);
      scene->getAllowedCollisionMatrixNonConst().setEntry(object_name, "left_gripper_l_finger", true);
    }
    else if (jmg == right_arm_)
    {
      scene->getAllowedCollisionMatrixNonConst().setEntry(object_name, "right_gripper_r_finger", true);
      scene->getAllowedCollisionMatrixNonConst().setEntry(object_name, "right_gripper_l_finger", true);
    }
    else
    {
      ROS_ERROR_STREAM_NAMED("pipeline","Unknown joint model group passed to allowFingerTouch");
      if (jmg)
        ROS_ERROR_STREAM_NAMED("pipeline","Joint model group: " << jmg->getName());
      return false;
    }

    // Prevent object from causing collision with shelf
    for (std::size_t i = 0; i < shelf_->getShelfParts().size(); ++i)
    {
      ROS_DEBUG_STREAM_NAMED("temp","Prevent collision between " << object_name << " and " << shelf_->getShelfParts()[i].getName());
      scene->getAllowedCollisionMatrixNonConst().setEntry(object_name, shelf_->getShelfParts()[i].getName(), true);
    }
  }

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

bool ManipulationPipeline::loadPlanExecution()
{
  // Create trajectory execution manager
  if( !trajectory_execution_manager_ )
  {
    trajectory_execution_manager_.reset(new trajectory_execution_manager::TrajectoryExecutionManager
                                        (planning_scene_monitor_->getRobotModel()));
    plan_execution_.reset(new plan_execution::PlanExecution(planning_scene_monitor_, trajectory_execution_manager_));
  }
  return true;
}

bool ManipulationPipeline::statusPublisher(const std::string &status)
{
  std::cout << std::endl << std::endl;
  ROS_INFO_STREAM_NAMED("pipeline.status", status << " -------------------------------------");
  visual_tools_->publishText(status_position_, status, rvt::WHITE, rvt::LARGE);
}

bool ManipulationPipeline::orderPublisher(WorkOrder& order)
{
  const std::string status = order.bin_->getName() + ": " + order.product_->getName();
  visual_tools_->publishText(status_position_, status, rvt::WHITE, rvt::LARGE);
}

bool ManipulationPipeline::statesEqual(const moveit::core::RobotState &s1, const moveit::core::RobotState &s2,
                                       const robot_model::JointModelGroup* jmg)
{
  static const double STATES_EQUAL_THRESHOLD = 0.01;

  double s1_vars[jmg->getActiveJointModels().size()];
  double s2_vars[jmg->getActiveJointModels().size()];
  s1.copyJointGroupPositions(jmg, s1_vars);
  s2.copyJointGroupPositions(jmg, s2_vars);

  for (std::size_t i = 0; i < jmg->getActiveJointModels().size(); ++i)
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
                                                 const robot_model::JointModelGroup* jmg)
{
  // Create a state space describing our robot's planning group
  moveit_ompl::ModelBasedStateSpacePtr model_state_space
    = boost::dynamic_pointer_cast<moveit_ompl::ModelBasedStateSpace>(experience_setup->getStateSpace());

  //ROS_DEBUG_STREAM_NAMED("pipeline","Model Based State Space has dimensions: " << model_state_space->getDimension());

  // Load lightning and its database
  ompl::tools::LightningPtr lightning = boost::dynamic_pointer_cast<ompl::tools::Lightning>(experience_setup);
  //7lightning.setFile(jmg->getName());

  // Get all of the paths in the database
  std::vector<ompl::base::PlannerDataPtr> paths;
  lightning->getAllPlannerDatas(paths);

  ROS_INFO_STREAM_NAMED("pipeline","Number of paths to publish: " << paths.size());

  // Load the OMPL visualizer
  if (!ompl_visual_tools_)
  {
    ompl_visual_tools_.reset(new ovt::OmplVisualTools(robot_model_->getModelFrame(),
                                                      "/ompl_experience_database", planning_scene_monitor_));
    ompl_visual_tools_->loadRobotStatePub("/baxter_amazon");
  }
  ompl_visual_tools_->deleteAllMarkers(); // clear all old markers
  ompl_visual_tools_->setStateSpace(model_state_space);

  // Get tip links for this setup
  std::vector<const robot_model::LinkModel*> tips;
  jmg->getEndEffectorTips(tips);
  ROS_INFO_STREAM_NAMED("pipeline","Found " << tips.size() << " tips");

  bool show_trajectory_animated = false;//verbose_;

  // Loop through each path
  for (std::size_t path_id = 0; path_id < paths.size(); ++path_id)
  {
    std::cout << "Processing path " << path_id << std::endl;
    ompl_visual_tools_->publishRobotPath(paths[path_id], jmg, tips, show_trajectory_animated);
  }

}

bool ManipulationPipeline::setToDefaultPosition(moveit::core::RobotStatePtr robot_state)
{
  const robot_model::JointModelGroup* jmg_ready = both_arms_;
  if (!robot_state->setToDefaultValues(jmg_ready, START_POSE))
  {
    ROS_ERROR_STREAM_NAMED("pipeline","Failed to set pose '" << START_POSE << "' for planning group '" << jmg_ready->getName() << "'");
    return false;
  }
  return true;
}

bool ManipulationPipeline::visualizeGrasps(std::vector<moveit_grasps::GraspSolution> filtered_grasps,
                                           const moveit::core::JointModelGroup *arm_jmg, bool show_cartesian_path)
{
  ROS_INFO_STREAM_NAMED("pipeline","Showing valid filtered grasp poses");

  // Publish in batch
  //visual_tools_->enableBatchPublishing(true);

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
      visual_tools_->publishTrajectoryPath(robot_state_trajectory, arm_jmg, speed, blocking);
    }
    grasps_->publishGraspArrow(filtered_grasps[i].grasp_.grasp_pose.pose, grasp_datas_[arm_jmg], rvt::BLUE, path_length);
  }
  //visual_tools_->triggerBatchPublishAndDisable();

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

  return visual_tools_->publishIKSolutions(ik_solutions, arm_jmg->getName(), display_time);
}

} // end namespace

namespace
{
bool isStateValid(const planning_scene::PlanningScene *planning_scene, bool verbose,
                  mvt::MoveItVisualToolsPtr visual_tools, robot_state::RobotState *robot_state,
                  const robot_state::JointModelGroup *group, const double *ik_solution)
{
  robot_state->setJointGroupPositions(group, ik_solution);
  robot_state->update();

  if (!planning_scene)
  {
    ROS_ERROR_STREAM_NAMED("temp","No planning scene provided");
    return false;
  }
  if (!planning_scene->isStateColliding(*robot_state, group->getName()))
    return true; // not in collision

  // Display more info about the collision
  if (verbose)
  {
    visual_tools->publishRobotState(*robot_state, rvt::RED);
    planning_scene->isStateColliding(*robot_state, group->getName(), true);
    visual_tools->publishContactPoints(*robot_state, planning_scene);
    ros::Duration(0.5).sleep();
  }
  return false;
}
}
