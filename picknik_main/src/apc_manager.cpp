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
  tf_.reset(new tf::TransformListener(nh_)); //ros::Duration(10.0)));

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
  if (!shelf_->initialize(package_path_, nh_))
  {
    ROS_ERROR_STREAM_NAMED("apc_manager","Unable to load shelf");
  }
  loadShelfContents(order_fp);

  // Subscribe to remote control topic
  std::size_t queue_size = 10;
  remote_next_control_ = nh_.subscribe("/picknik_main/next", queue_size, &APCManager::remoteNextCallback, this);
  remote_run_control_ = nh_.subscribe("/picknik_main/run", queue_size, &APCManager::remoteRunCallback, this);

  // Visualize
  visualizeShelf();

  ROS_INFO_STREAM_NAMED("apc_manager","APC Manager Ready.");
}

void APCManager::remoteNextCallback(const std_msgs::Bool::ConstPtr& msg)
{
  pipeline_->next_step_ready_ = true;
}

void APCManager::remoteRunCallback(const std_msgs::Bool::ConstPtr& msg)
{
  pipeline_->autonomous_ = true;
}

bool APCManager::runOrder(bool use_experience, bool show_database, std::size_t order_start, std::size_t jump_to, 
                          std::size_t num_orders, bool autonomous)
{
  // Create the pick place pipeline
  pipeline_.reset(new ManipulationPipeline(verbose_, visuals_,
                                           planning_scene_monitor_, plan_execution_,
                                           shelf_, use_experience, show_database));

  std::cout << std::endl;

  // Make sure we are connected to perception
  pipeline_->checkSystemReady();

  // Set autonomy
  pipeline_->autonomous_ = autonomous;

  ROS_INFO_STREAM_NAMED("apc_manager","Starting order ----------------------------");

  // Decide how many products to pick
  if (num_orders == 0)
    num_orders = orders_.size();

  // Grasps things
  for (std::size_t i = order_start; i < num_orders; ++i)
  {
    pipeline_->orderPublisher(orders_[i]); // feedback

    if (!pipeline_->graspObjectPipeline(orders_[i], verbose_, jump_to))
    {
      ROS_ERROR_STREAM_NAMED("apc_manager","Shutting down for debug purposes only (it could continue on)");
      return false;
    }
  }

  pipeline_->statusPublisher("Finished");
}

bool APCManager::trainExperienceDatabase()
{
  // Create learning pipeline for training the experience database
  bool use_experience = false;
  bool show_database = false;
  learning_.reset(new LearningPipeline(verbose_, visuals_,
                                       planning_scene_monitor_, plan_execution_,
                                       shelf_, use_experience, show_database));

  ROS_INFO_STREAM_NAMED("apc_manager","Training experience database");
  learning_->generateTrainingGoals(shelf_);

  return true;
}

bool APCManager::testEndEffectors()
{
  // Create the pick place pipeline
  bool use_experience = false;
  bool show_database = false;
  pipeline_.reset(new ManipulationPipeline(verbose_, visuals_,
                                           planning_scene_monitor_, plan_execution_,
                                           shelf_, use_experience, show_database));


  // Test visualization
  pipeline_->statusPublisher("Testing open close visualization of EE");
  std::size_t i = 0;
  while (ros::ok())
  {
    std::cout << std::endl << std::endl;
    if (i % 2 == 0)
    {
      std::cout << "Showing closed EE of state " << std::endl;
      pipeline_->testEndEffectors(false);
      ros::Duration(4.0).sleep();
    }
    else
    {
      std::cout << "Showing open EE of state " << std::endl;
      pipeline_->testEndEffectors(true);
      ros::Duration(4.0).sleep();
    }
    ++i;
  }

  // EE min approach distance
  //pipeline_->statusPublisher("Testing EE min approach distance");

  ROS_INFO_STREAM_NAMED("apc_manager","Done testing end effectors");
}

bool APCManager::testUpAndDown()
{
  // Create the pick place pipeline
  bool use_experience = false;
  bool show_database = false;
  pipeline_.reset(new ManipulationPipeline(verbose_, visuals_,
                                           planning_scene_monitor_, plan_execution_,
                                           shelf_, use_experience, show_database));
  pipeline_->testUpAndDown();

  ROS_INFO_STREAM_NAMED("apc_manager","Done testing up and down");
}

bool APCManager::testShelfLocation()
{
  // Create the pick place pipeline
  bool use_experience = false;
  bool show_database = false;
  pipeline_.reset(new ManipulationPipeline(verbose_, visuals_,
                                           planning_scene_monitor_, plan_execution_,
                                           shelf_, use_experience, show_database));

  // Configure
  ROS_WARN_STREAM_NAMED("temp","TODO");

  // TODO

  ROS_INFO_STREAM_NAMED("apc_manager","Done testing shelf location");
}

bool APCManager::testGoalBinPose()
{
  // Create the pick place pipeline
  bool use_experience = false;
  bool show_database = false;
  pipeline_.reset(new ManipulationPipeline(verbose_, visuals_,
                                           planning_scene_monitor_, plan_execution_,
                                           shelf_, use_experience, show_database));
  pipeline_->moveToDropOffPosition();

  ROS_INFO_STREAM_NAMED("apc_manager","Done going to goal bin pose");  
}

bool APCManager::testInCollision()
{
  // Create the pick place pipeline
  bool use_experience = false;
  bool show_database = false;
  pipeline_.reset(new ManipulationPipeline(verbose_, visuals_,
                                           planning_scene_monitor_, plan_execution_,
                                           shelf_, use_experience, show_database));

  while (ros::ok())
  {
    pipeline_->checkCurrentCollisionAndBounds();
    ros::Duration(1).sleep();
  }

  ROS_INFO_STREAM_NAMED("apc_manager","Done checking if in collision");  
}

bool APCManager::getPose()
{
  // Create the pick place pipeline
  bool use_experience = false;
  bool show_database = false;
  pipeline_.reset(new ManipulationPipeline(verbose_, visuals_,
                                           planning_scene_monitor_, plan_execution_,
                                           shelf_, use_experience, show_database));

  pipeline_->getSRDFPose();

}

bool APCManager::loadShelfContents(std::string order_fp)
{
  // Choose file
  AmazonJSONParser parser(verbose_, visuals_);
  // Parse json
  return parser.parse(order_fp, package_path_, shelf_, orders_);
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

  // Create trajectory execution manager
  if( !trajectory_execution_manager_ )
  {
    trajectory_execution_manager_.reset(new trajectory_execution_manager::TrajectoryExecutionManager(robot_model_));
    plan_execution_.reset(new plan_execution::PlanExecution(planning_scene_monitor_, trajectory_execution_manager_));
  }

  // Get the joint state topic
  std::string joint_state_topic;
  getStringParameter(nh_, "joint_state_topic", joint_state_topic);

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

} // end namespace
