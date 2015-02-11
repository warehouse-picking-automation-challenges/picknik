/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Dave Coleman <dave@dav.ee>
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

ManipulationPipeline::ManipulationPipeline(bool verbose, 
                                           mvt::MoveItVisualToolsPtr visual_tools,
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
  {
    planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_); // Lock planning scene
    robot_state_.reset(new moveit::core::RobotState(scene->getCurrentState()));
  }
  visual_tools_->getSharedRobotState() = robot_state_; // allow visual_tools to have the correct virtual joint
  robot_model_ = robot_state_->getRobotModel();

  // Set baxter to starting position
  const robot_model::JointModelGroup* jmg = robot_model_->getJointModelGroup("both_arms");
  if (!robot_state_->setToDefaultValues(jmg, START_POSE))
  {
    ROS_ERROR_STREAM_NAMED("pipeline","Failed to set pose '" << START_POSE << "' for planning group '" << jmg->getName() << "'");
  }

  // Allocate robot states
  start_state_.reset(new moveit::core::RobotState(*robot_state_));

  // Load arm groups
  left_arm_ = robot_model_->getJointModelGroup("left_arm");
  right_arm_ = robot_model_->getJointModelGroup("right_arm");

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
  setEndEffectorOpen(true, robot_state_); // so that grasp filter is started up with EE open
  grasp_filter_.reset(new moveit_grasps::GraspFilter(robot_state_, visual_tools_) );

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

bool ManipulationPipeline::chooseGrasp(const Eigen::Affine3d& object_pose, const robot_model::JointModelGroup* jmg,
                                       moveit_grasps::GraspSolution& chosen, bool verbose)
{
  const moveit::core::JointModelGroup* ee_jmg = robot_model_->getJointModelGroup(grasp_datas_[jmg].ee_group_);

  visual_tools_->publishAxis(object_pose);

  std::vector<moveit_msgs::Grasp> possible_grasps;
  grasps_->setVerbose(true);
  grasps_->generateAxisGrasps( object_pose, moveit_grasps::Y_AXIS, moveit_grasps::DOWN, moveit_grasps::HALF, 0,
                               grasp_datas_[jmg], possible_grasps);

  // Visualize
  if (verbose)
  {
    visual_tools_->loadEEMarker("left_hand");
    visual_tools_->publishAnimatedGrasps(possible_grasps, ee_jmg);
  }

  // Filter grasps based on IK

  // Filter the grasp for only the ones that are reachable
  bool filter_pregrasps = true;
  std::vector<moveit_grasps::GraspSolution> filtered_grasps;
  grasp_filter_->filterGrasps(possible_grasps, filtered_grasps, filter_pregrasps,
                              grasp_datas_[jmg].parent_link_name_, jmg);

  // Visualize them
  if (verbose)
    visual_tools_->publishAnimatedGrasps(possible_grasps, ee_jmg);

  // Convert the filtered_grasps into a format moveit_visual_tools can use
  if (verbose)
  {
    std::vector<trajectory_msgs::JointTrajectoryPoint> ik_solutions;
    ik_solutions.resize(filtered_grasps.size());
    for (std::size_t i = 0; i < filtered_grasps.size(); ++i)
    {
      ik_solutions[i].positions = filtered_grasps[i].grasp_ik_solution_;
    }
    visual_tools_->publishIKSolutions(ik_solutions, jmg->getName(), 0.5);
  }

  ROS_INFO_STREAM_NAMED("pipeline","Filtering grasps by collision");

  // Filter grasps based on collision
  {
    planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_); // Lock planning scene
    (*robot_state_) = scene->getCurrentState();
  }
  setEndEffectorOpen(true, robot_state_); // to be passed to the grasp filter

  grasp_filter_->filterGraspsInCollision(filtered_grasps, planning_scene_monitor_, jmg, robot_state_, verbose);

  // Convert the filtered_grasps into a format moveit_visual_tools can use
  if (verbose)
  {
    std::vector<trajectory_msgs::JointTrajectoryPoint> ik_solutions;
    ik_solutions.resize(filtered_grasps.size());
    for (std::size_t i = 0; i < filtered_grasps.size(); ++i)
    {
      ik_solutions[i].positions = filtered_grasps[i].grasp_ik_solution_;
    }
    visual_tools_->publishIKSolutions(ik_solutions, jmg->getName(), 1);
  }

  // Choose grasp
  if (!grasp_filter_->chooseBestGrasp(filtered_grasps, chosen))
  {
    ROS_ERROR_STREAM_NAMED("pipeline","No best grasp found");
    return false;
  }

  return true;
}

bool ManipulationPipeline::setupPlanningScene( const std::string& bin_name )
{
  ROS_WARN_STREAM_NAMED("temp","setupPlanningScene - sleeping 2 seconds ");

  // Disable all bins except desired one
  visual_tools_->removeAllCollisionObjects(); // clear all old collision objects that might be visible in rviz
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

bool ManipulationPipeline::graspObject( WorkOrder order, bool verbose )
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

  bool result = graspObjectPipeline(object_pose, order, verbose);

  // Delete from planning scene the product
  shelf_->deleteProduct(order.bin_->getName(), order.product_->getName());
  visual_tools_->cleanupACO( order.product_->getCollisionName() ); // use unique name

  return result;
}

bool ManipulationPipeline::graspObjectPipeline(const Eigen::Affine3d& object_pose, WorkOrder order, bool verbose)
{
  const robot_model::JointModelGroup* jmg = chooseArm(object_pose);
  bool execute_trajectory = true;

  // Generate and choose grasp
  moveit_grasps::GraspSolution chosen;
  if (!chooseGrasp(object_pose, jmg, chosen, verbose && false))
  {
    ROS_ERROR_STREAM_NAMED("pipeline","No grasps found");
    return false;
  }

  // Allocate robot states
  moveit::core::RobotStatePtr pre_grasp(new moveit::core::RobotState(*robot_state_));
  moveit::core::RobotStatePtr the_grasp(new moveit::core::RobotState(*robot_state_));

  // Get current state
  {
    planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_); // Lock planning scene
    (*start_state_) = scene->getCurrentState();
  }
  setEndEffectorOpen(true, start_state_);

  // Get the-grasp
  the_grasp->setJointGroupPositions(jmg, chosen.grasp_ik_solution_);
  setEndEffectorOpen(true, the_grasp);

  if (verbose && true)
  {
    statusPublisher("Visualizing the-grasp");
    visual_tools_->publishRobotState(the_grasp, rvt::PURPLE);
    ros::Duration(1.0).sleep();
  }

  // Get pre-grasp - compute straight line in reverse from grasp
  Eigen::Vector3d approach_direction;
  approach_direction << -1, 0, 0; // backwards towards robot body
  double desired_approach_distance = 0.12; //0.15;
  // Show desired distance
  std::vector<robot_state::RobotStatePtr> robot_state_trajectory;
  double path_length;
  if (!computeStraightLinePath( approach_direction, desired_approach_distance,
                                robot_state_trajectory, the_grasp, jmg, path_length))
  {
    ROS_ERROR_STREAM_NAMED("pipeline","Error occured while computing straight line path");
    return false;
  }

  // Reverse the trajectory
  std::reverse(robot_state_trajectory.begin(), robot_state_trajectory.end());

  // Visualize end effector position of cartesian path
  for (std::size_t i = 0; i < robot_state_trajectory.size(); ++i)
  {
    const Eigen::Affine3d& tip_pose = 
      robot_state_trajectory[i]->getGlobalLinkTransform(grasp_datas_[jmg].parent_link_);
    visual_tools_->publishSphere(tip_pose);

    //visual_tools_->publishRobotState(robot_state_trajectory[i], rvt::YELLOW);
    //ros::Duration(0.5).sleep();
  }

  // Get approach trajectory message
  moveit_msgs::RobotTrajectory cartesian_trajectory_msg;
  if (!convertRobotStatesToTrajectory(robot_state_trajectory, cartesian_trajectory_msg, jmg))
  {
    ROS_ERROR_STREAM_NAMED("pipeline","Failed to convert to parameterized trajectory");
    return false;
  }

  // Set the pregrasp to be the first state in the trajectory
  pre_grasp = robot_state_trajectory.front();

  if (verbose && true)
  {
    statusPublisher("Visualizing pre-grasp");
    visual_tools_->publishRobotState(pre_grasp, rvt::PURPLE);
    ros::Duration(1.0).sleep();
  }

  // Grasp --------------------------------------------------------------------------------------
  statusPublisher("Opening End Effector");
  if (!openEndEffector(true, jmg))
  {
    ROS_ERROR_STREAM_NAMED("pipeline","Unable to open end effector");
    return false;
  }

  // Plan to pre-grasp ----------------------------------------------------------------------------------
  statusPublisher("Moving to pre-grasp position");
  if (!move(start_state_, pre_grasp, jmg, verbose, execute_trajectory, show_database_))
  {
    ROS_ERROR_STREAM_NAMED("pipeline","Unable to plan");
    return false;
  }

  ros::Duration(1).sleep();

  // Move to grasp --------------------------------------------------------------------------------------
  statusPublisher("Cartesian move to the-grasp position");
  if( !executeTrajectoryMsg(cartesian_trajectory_msg) )
  {
    ROS_ERROR_STREAM_NAMED("pipeline","Failed to execute trajectory");
  }

  // Grasp --------------------------------------------------------------------------------------
  statusPublisher("Grasping");
  if (!openEndEffector(false, jmg))
  {
    ROS_ERROR_STREAM_NAMED("pipeline","Unable to close end effector");
    return false;
  }

  // Attach collision object
  visual_tools_->attachCO(order.product_->getCollisionName(), grasp_datas_[jmg].parent_link_name_);

  // Allow fingers to touch object
  allowFingerTouch(order.product_->getCollisionName(), jmg);

  ros::Duration(1).sleep();

  // Cartesian lift object up ---------------------------------------------------------------
  if (!executeLiftPath(jmg))
  {
    ROS_ERROR_STREAM_NAMED("pipeline","Unable to execute retrieval path after grasping");
    return false;
  }

  // Cartesian move back to pregrasp ------------------------------------------------------------------------------  
  if (!executeRetrievalPath(jmg))
  {
    ROS_ERROR_STREAM_NAMED("pipeline","Unable to execute retrieval path after grasping");
    return false;
  }

  // Plan back to initial state ------------------------------------------------------------------------------
  statusPublisher("Moving back to INITIAL position");
  if (!moveToStartPosition(jmg))
  {
    ROS_ERROR_STREAM_NAMED("pipeline","Unable to plan");
    return false;
  }

  // Release --------------------------------------------------------------------------------------
  statusPublisher("Releasing");
  if (!openEndEffector(true, jmg))
  {
    ROS_ERROR_STREAM_NAMED("pipeline","Unable to close end effector");
    return false;
  }

  // Unattach
  visual_tools_->cleanupACO(order.product_->getCollisionName());

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
    (*start_state_) = scene->getCurrentState();
  }

  const robot_model::JointModelGroup* jmg_ready = robot_model_->getJointModelGroup("both_arms");

  // Set to default group if needed
  if (!jmg)
    jmg = jmg_ready;

  // Set goal state to initial pose
  if (!robot_state_->setToDefaultValues(jmg_ready, START_POSE))
  {
    ROS_ERROR_STREAM_NAMED("pipeline","Failed to set pose '" << START_POSE << "' for planning group '" << jmg_ready->getName() << "'");
  }
  
  // Check if already in start position
  {
    planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_); // Lock planning scene
    if (statesEqual(scene->getCurrentState(), *robot_state_, jmg))
    {
      ROS_WARN_STREAM_NAMED("pipeline","Not planning motion because current state and goal state are close enough.");
      return true;
    }
  }


  // Plan
  bool execute_trajectory = true;
  if (!move(start_state_, robot_state_, jmg, verbose_, execute_trajectory, show_database_))
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
    visual_tools_->publishRobotState(start, rvt::GREEN);
    ros::Duration(1.0).sleep();

    visual_tools_->publishRobotState(goal, rvt::ORANGE);
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

  // Visualize trajectory
  if (true)
  {
    // Visualize the trajectory
    bool wait_for_trajetory = false;
    visual_tools_->publishTrajectoryPath(response.trajectory, wait_for_trajetory);
  }

  // Focus on execution (unless we are in debug mode)
  if (!error)
    visual_tools_->hideRobot();

  // Execute trajectory
  if (execute_trajectory)
  {
    if( !executeTrajectoryMsg(response.trajectory) )
    {
      ROS_ERROR_STREAM_NAMED("pipeline","Failed to execute trajectory");
    }
  }

  // Save Experience Database
  if (use_experience_)
  {
    moveit_ompl::ModelBasedPlanningContextPtr mbpc = boost::dynamic_pointer_cast<moveit_ompl::ModelBasedPlanningContext>(planning_context_handle);
    ompl::tools::ExperienceSetupPtr experience_setup = boost::dynamic_pointer_cast<ompl::tools::ExperienceSetup>(mbpc->getOMPLSimpleSetup());
    

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
      ros::Duration(10).sleep();
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
  if (open)
  {
    setEndEffectorOpen(true, robot_state_); // to be passed to the grasp filter
    visual_tools_->publishRobotState(robot_state_);
    //openEndEffector(true, left_arm_);
    openEndEffector(true, right_arm_);
  }
  else
  {
    setEndEffectorOpen(false, robot_state_); // to be passed to the grasp filter
    visual_tools_->publishRobotState(robot_state_);
    //openEndEffector(false, left_arm_);
    openEndEffector(false, right_arm_);
  }
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

bool ManipulationPipeline::executeLiftPath(const moveit::core::JointModelGroup *jmg)
{
  // Get current state after grasping
  {
    planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_); // Lock planning scene
    (*start_state_) = scene->getCurrentState();
  }

  // Compute straight line up above grasp
  Eigen::Vector3d approach_direction;
  approach_direction << 0, 0, 1; // up over object
  double desired_approach_distance = 0.01;
  std::vector<robot_state::RobotStatePtr> robot_state_trajectory;
  double path_length;
  if (!computeStraightLinePath( approach_direction, desired_approach_distance,
                                robot_state_trajectory, start_state_, jmg, path_length))
  {
    ROS_ERROR_STREAM_NAMED("pipeline","Error occured while computing straight line path");
    return false;
  }

  // Visualize end effector position of cartesian path
  for (std::size_t i = 0; i < robot_state_trajectory.size(); ++i)
  {
    const Eigen::Affine3d& tip_pose = 
      robot_state_trajectory[i]->getGlobalLinkTransform(grasp_datas_[jmg].parent_link_name_);
    visual_tools_->publishSphere(tip_pose, rvt::YELLOW);

    //visual_tools_->publishRobotState(robot_state_trajectory[i], rvt::YELLOW);
    //ros::Duration(0.5).sleep();
  }

  // Get approach trajectory message
  moveit_msgs::RobotTrajectory cartesian_trajectory_msg;
  if (!convertRobotStatesToTrajectory(robot_state_trajectory, cartesian_trajectory_msg, jmg))
  {
    ROS_ERROR_STREAM_NAMED("pipeline","Failed to convert to parameterized trajectory");
    return false;
  }

  // Execute
  statusPublisher("Lifting product UP slightly");
  if( !executeTrajectoryMsg(cartesian_trajectory_msg) )
  {
    ROS_ERROR_STREAM_NAMED("pipeline","Failed to execute trajectory");
    return false;
  }

  return true;
}

bool ManipulationPipeline::executeRetrievalPath(const moveit::core::JointModelGroup *jmg)
{
  // Get current state after grasping
  {
    planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_); // Lock planning scene
    (*start_state_) = scene->getCurrentState();
  }

  // Compute straight line in reverse from grasp
  Eigen::Vector3d approach_direction;
  approach_direction << -1, 0, 0; // backwards towards robot body
  double desired_approach_distance = 0.15;
  double path_length;
  std::vector<robot_state::RobotStatePtr> robot_state_trajectory;
  if (!computeStraightLinePath( approach_direction, desired_approach_distance,
                                robot_state_trajectory, start_state_, jmg, path_length))
  {
    ROS_ERROR_STREAM_NAMED("pipeline","Error occured while computing straight line path");
    return false;
  }

  // Visualize end effector position of cartesian path
  for (std::size_t i = 0; i < robot_state_trajectory.size(); ++i)
  {
    const Eigen::Affine3d& tip_pose = 
      robot_state_trajectory[i]->getGlobalLinkTransform(grasp_datas_[jmg].parent_link_name_);
    visual_tools_->publishSphere(tip_pose, rvt::RED);

    //visual_tools_->publishRobotState(robot_state_trajectory[i], rvt::YELLOW);
    //ros::Duration(0.5).sleep();
  }

  // Get approach trajectory message
  moveit_msgs::RobotTrajectory cartesian_trajectory_msg;
  if (!convertRobotStatesToTrajectory(robot_state_trajectory, cartesian_trajectory_msg, jmg))
  {
    ROS_ERROR_STREAM_NAMED("pipeline","Failed to convert to parameterized trajectory");
    return false;
  }

  // Execute
  statusPublisher("Moving BACK to pre-grasp position");
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
                                                    const moveit::core::JointModelGroup *jmg,
                                                    double& path_length)
{
  ROS_DEBUG_STREAM_NAMED("pipeline","Computing cartesian path");

  // ---------------------------------------------------------------------------------------------
  // Show desired trajectory in BLACK
  const Eigen::Affine3d tip_pose_start = robot_state->getGlobalLinkTransform(grasp_datas_[jmg].parent_link_);
      
  if (verbose_)
  {
    Eigen::Affine3d tip_pose_end = tip_pose_start;
    tip_pose_end.translation().x() -= desired_approach_distance;
    visual_tools_->publishLine(tip_pose_start, tip_pose_end, rvt::BLACK, rvt::REGULAR);
  }

  // ---------------------------------------------------------------------------------------------
  // Settings for computeCartesianPath

  // End effector parent link (arm tip for ik solving)
  const moveit::core::LinkModel *ik_tip_link_model = grasp_datas_[jmg].parent_link_;

  // Resolution of trajectory
  double max_step = 0.05; // 0.01 // The maximum distance in Cartesian space between consecutive points on the resulting path

  // Jump threshold for preventing consequtive joint values from 'jumping' by a large amount in joint space
  double jump_threshold = 0.0; // disabled

  // Check for kinematic solver
  if( !jmg->canSetStateFromIK( ik_tip_link_model->getName() ) )
  {
    ROS_ERROR_STREAM_NAMED("pipeline","No IK Solver loaded - make sure moveit_config/kinamatics.yaml is loaded in this namespace");
  }

  {
    // Collision check
    boost::scoped_ptr<planning_scene_monitor::LockedPlanningSceneRO> ls;
    ls.reset(new planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor_));
    robot_state::GroupStateValidityCallbackFn constraint_fn 
      = boost::bind(&isStateValid, static_cast<const planning_scene::PlanningSceneConstPtr&>(*ls).get(), _1, _2, _3);

    // -----------------------------------------------------------------------------------------------
    // Compute Cartesian Path
    path_length = robot_state->computeCartesianPath(jmg,
                                                    robot_state_trajectory,
                                                    ik_tip_link_model,
                                                    approach_direction,
                                                    true,           // direction is in global reference frame
                                                    desired_approach_distance,
                                                    max_step,
                                                    jump_threshold,
                                                    constraint_fn // collision check
                                                    );
  }

  ROS_DEBUG_STREAM_NAMED("pipeline","Cartesian resulting distance: " << path_length << " desired: " << desired_approach_distance
                         << " number of states in trajectory: " << robot_state_trajectory.size());

  if( path_length == 0 )
  {
    ROS_ERROR_STREAM_NAMED("pipeline","Failed to computer cartesian path: distance is 0");
    return false;
  }
  else if ( path_length < desired_approach_distance * 0.5 )
  {
    ROS_WARN_STREAM_NAMED("pipeline","Resuling cartesian path distance is less than half the desired distance");
  }

  // ---------------------------------------------------------------------------------------------
  // Show actual trajectory in GREEN
  if (verbose_)
  {
    const Eigen::Affine3d& tip_pose_end = 
      robot_state_trajectory.back()->getGlobalLinkTransform(grasp_datas_[jmg].parent_link_);
    visual_tools_->publishLine(tip_pose_start, tip_pose_end, rvt::LIME_GREEN, rvt::LARGE);
  }

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

bool ManipulationPipeline::setEndEffectorOpen(bool open, moveit::core::RobotStatePtr robot_state)
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
  else
  {
    ROS_ERROR_STREAM_NAMED("pipeline","Failed to execute trajectory");
    return false;
  }

  return true;
}

bool ManipulationPipeline::allowFingerTouch(const std::string& object_name, const robot_model::JointModelGroup* jmg)
{
  // Prevent fingers from causing collision with object
  {
    planning_scene_monitor::LockedPlanningSceneRW scene(planning_scene_monitor_); // Lock planning scene

    if (jmg->getName() == "left_arm")
    {
      scene->getAllowedCollisionMatrixNonConst().setEntry(object_name, "left_gripper_r_finger", true);
      scene->getAllowedCollisionMatrixNonConst().setEntry(object_name, "left_gripper_l_finger", true);
    }
    else if (jmg->getName() == "right_arm")
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
  ROS_INFO_STREAM_NAMED("pipeline","Status: " << status << " --------------------------------");
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
  double s1_vars[jmg->getActiveJointModels().size()];
  double s2_vars[jmg->getActiveJointModels().size()];
  s1.copyJointGroupPositions(jmg, s1_vars);
  s2.copyJointGroupPositions(jmg, s2_vars);
  
  for (std::size_t i = 0; i < jmg->getActiveJointModels().size(); ++i)
  {
    //std::cout << "Diff of " << i << " - " << fabs(s1_vars[i] - s2_vars[i]) << std::endl;
    if ( fabs(s1_vars[i] - s2_vars[i]) > 0.001 )
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
    ompl_visual_tools_.reset(new ompl_visual_tools::OmplVisualTools(robot_model_->getModelFrame(), "/ompl_experience_database", 
                                                                    robot_model_));
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
  const robot_model::JointModelGroup* jmg_ready = robot_model_->getJointModelGroup("both_arms");
  if (!robot_state->setToDefaultValues(jmg_ready, START_POSE))
  {
    ROS_ERROR_STREAM_NAMED("pipeline","Failed to set pose '" << START_POSE << "' for planning group '" << jmg_ready->getName() << "'");
    return false;
  }
  return true;
}

} // end namespace

namespace
{
bool isStateValid(const planning_scene::PlanningScene *planning_scene,
                  robot_state::RobotState *state,
                  const robot_state::JointModelGroup *group, const double *ik_solution)
{
  state->setJointGroupPositions(group, ik_solution);
  state->update();
  return (!planning_scene || !planning_scene->isStateColliding(*state, group->getName()));
}
}
