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
   Desc:   Main logic of APC challenge
*/

// Amazon Pick Place Challenge
#include <baxter_apc_main/apc_manager.h>

namespace baxter_apc_main
{

APCManager::APCManager(bool verbose, std::string order_fp)
  : nh_("~")
  , verbose_(verbose)
{
  // Load the loader
  robot_model_loader_.reset(new robot_model_loader::RobotModelLoader(ROBOT_DESCRIPTION));

  // Load the robot model
  robot_model_ = robot_model_loader_->getModel(); // Get a shared pointer to the robot

  // Create the planning scene
  planning_scene_.reset(new planning_scene::PlanningScene(robot_model_));

  // Create tf transformer
  tf_.reset(new tf::TransformListener(ros::Duration(10.0)));

  // Load planning scene monitor
  if (!loadPlanningSceneMonitor())
  {
    ROS_ERROR_STREAM_NAMED("apc_manager","Unable to load planning scene monitor");
  }

  // Load the Robot Viz Tools for publishing to Rviz
  visual_tools_.reset(new mvt::MoveItVisualTools(robot_model_->getModelFrame(), "/amazon_shelf_markers", planning_scene_monitor_));
  visual_tools_->loadRobotStatePub("/baxter_amazon");
  visual_tools_->loadMarkerPub();
  visual_tools_->setAlpha(0.8);
  visual_tools_->hideRobot(); // show that things have been reset
  visual_tools_->deleteAllMarkers(); // clear all old markers
  visual_tools_->setManualSceneUpdating(true);

  // Load the COLLISION Robot Viz Tools for publishing to Rviz
  visual_tools_display_.reset(new mvt::MoveItVisualTools(robot_model_->getModelFrame(), "/amazon_shelf_markers_display", planning_scene_monitor_));
  //visual_tools_display_->loadRobotStatePub("/baxter_amazon_display");
  //visual_tools_display_->hideRobot(); // show that things have been reset
  visual_tools_display_->deleteAllMarkers(); // clear all old markers

  // Get package path
  package_path_ = ros::package::getPath(PACKAGE_NAME);
  if( package_path_.empty() )
    ROS_FATAL_STREAM_NAMED("product", "Unable to get " << PACKAGE_NAME << " package path" );

  // Load shelf
  shelf_.reset(new ShelfObject(visual_tools_, visual_tools_display_, rvt::BROWN, "shelf_0", package_path_));
  loadShelfContents(order_fp);
  visualizeShelf();

  ROS_INFO_STREAM_NAMED("apc_manager","APC Manager Ready.");
}

bool APCManager::runOrder(bool use_experience, bool show_database)
{
  // Create the pick place pipeline
  pipeline_.reset(new ManipulationPipeline(verbose_, visual_tools_, visual_tools_display_,
                                           planning_scene_monitor_, shelf_, use_experience, show_database));

  // Move robot to start location
  pipeline_->statusPublisher("Moving to initial position");

  if (true)
  {
    pipeline_->moveToStartPosition();
    pipeline_->openEndEffectors(true);

    if (!ros::ok())
      return false;
  }

  std::cout << std::endl;
  std::cout << std::endl;
  std::cout << std::endl;
  ROS_INFO_STREAM_NAMED("apc_manager","Starting order ----------------------------");

  //pipeline_->testEndEffector();
  //return true;

  // Grasps things
  if (true) // run normal
  {
    for (std::size_t i = 0; i < orders_.size(); ++i)
    {
      pipeline_->orderPublisher(orders_[i]); // feedback

      if (!pipeline_->graspObject(orders_[i], verbose_))
      {
        ROS_ERROR_STREAM_NAMED("apc_manager","Shutting down for debug purposes only (it could continue on)");
        return false;
      }
    }
  }
  else // debug mode
  {
    ROS_INFO_STREAM_NAMED("apc_manager","Debug mode - grasping only 1 object");
    pipeline_->graspObject(orders_[1], verbose_);
  }

  pipeline_->statusPublisher("Finished");
}

bool APCManager::trainExperienceDatabase()
{
  // Create learning pipeline for training the experience database
  bool use_experience = false;
  bool show_database = false;
  learning_.reset(new LearningPipeline(verbose_, visual_tools_, visual_tools_display_,
                                       planning_scene_monitor_, shelf_, use_experience, show_database));

  ROS_INFO_STREAM_NAMED("apc_manager","Training experience database");
  learning_->generateTrainingGoals(shelf_);

  return true;
}

bool APCManager::testEndEffectors()
{
  // Create the pick place pipeline
  bool use_experience = false;
  bool show_database = false;
  pipeline_.reset(new ManipulationPipeline(verbose_, visual_tools_, visual_tools_display_,
                                           planning_scene_monitor_, shelf_, use_experience, show_database));

  // Test visualization 10 times
  pipeline_->statusPublisher("Testing open close visualization of EE, 10 times");
  for (std::size_t i = 0; i < 10; ++i)
  {
    if (!ros::ok())
      return false;

    std::cout << std::endl << std::endl << std::endl;
    if (i % 2 == 0)
    {
      std::cout << "Showing closed EE of state " << std::endl;
      pipeline_->openEndEffector(false, pipeline_->getRobotState()); // to be passed to the grasp filter
      visual_tools_->publishRobotState(pipeline_->getRobotState());
      ros::Duration(5.0).sleep();
    }
    else
    {
      std::cout << "Showing open EE of state " << std::endl;
      pipeline_->openEndEffector(true, pipeline_->getRobotState()); // to be passed to the grasp filter
      visual_tools_->publishRobotState(pipeline_->getRobotState());
      ros::Duration(5.0).sleep();
    }
  }

  // EE min approach distance
  //pipeline_->statusPublisher("Testing EE min approach distance");

  ROS_INFO_STREAM_NAMED("apc_manager","Done testing end effectors");
}

bool APCManager::loadShelfContents(std::string order_fp)
{
  // Choose file
  AmazonJSONParser parser(verbose_, visual_tools_, visual_tools_display_);

  // Parse json
  return parser.parse(order_fp, package_path_, shelf_, orders_);
}

bool APCManager::visualizeShelf()
{
  // Show the mesh visualization
  shelf_->visualize();
  shelf_->visualizeAxis(visual_tools_display_);

  // Now show empty shelf to help in reversing robot arms to initial position
  visual_tools_->removeAllCollisionObjects();
  shelf_->createCollisionBodies("", true); // only show the frame
  shelf_->visualizeAxis(visual_tools_);
  visual_tools_->triggerPlanningSceneUpdate();

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
  ros::Duration(0.1).sleep();

  if (planning_scene_monitor_->getPlanningScene())
  {
    // Optional monitors to start:
    bool use_octomap_monitor = false; // this prevents a /tf warning
    planning_scene_monitor_->startWorldGeometryMonitor(planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
                                                       planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
                                                       use_octomap_monitor);
    //planning_scene_monitor_->startSceneMonitor("/move_group/monitored_planning_scene");
    planning_scene_monitor_->startStateMonitor(JOINT_STATE_TOPIC, "/attached_collision_object");
    planning_scene_monitor_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
                                                          "baxter_apc_planning_scene");
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


  return true;
}


} // end namespace
