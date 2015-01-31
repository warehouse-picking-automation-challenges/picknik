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
   Desc:   Create collision objects for all the things on the shelf
*/

//#ifndef BAXTER_APC_MAIN__SHELF_MANAGER
//#define BAXTER_APC_MAIN__SHELF_MANAGER


// Amazon Pick Place Challenge
#include <baxter_apc_main/amazon_json_parser.h>
#include <baxter_apc_main/shelf.h>
#include <baxter_apc_main/manipulation_pipeline.h>

// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>

// MoveIt
#include <moveit/collision_detection/world.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/plan_execution/plan_execution.h>

// Grasp generation
#include <moveit_grasps/grasps.h>
#include <moveit_grasps/grasp_data.h>
#include <moveit_grasps/grasp_filter.h>

namespace baxter_apc_main
{

static const std::string ROBOT_DESCRIPTION = "robot_description";
static const std::string JOINT_STATE_TOPIC = "/robot/joint_states";
static const std::string PACKAGE_NAME = "baxter_apc_main";

class APCManager
{
private:

  // A shared node handle
  ros::NodeHandle nh_;
  boost::shared_ptr<tf::TransformListener> tf_;

  // Show more visual and console output, with general slower run time.
  bool verbose_;

  // For visualizing things in rviz
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  // Core MoveIt components
  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  robot_model::RobotModelPtr robot_model_;
  planning_scene::PlanningScenePtr planning_scene_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  // Properties
  ShelfObjectPtr shelf_;
  WorkOrders orders_;
  std::string package_path_;

  // Main worker
  ManipulationPipelinePtr pipeline_;

public:

  /**
   * \brief Constructor
   * \param verbose - run in debug mode
   */
  APCManager(bool verbose)
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
    visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools(robot_model_->getModelFrame(), "/amazon_shelf_markers", planning_scene_monitor_));
    visual_tools_->loadRobotStatePub("/baxter_amazon");
    visual_tools_->loadMarkerPub();
    visual_tools_->setAlpha(0.8);
    visual_tools_->hideRobot(); // show that things have been reset
    visual_tools_->deleteAllMarkers(); // clear all old markers
    visual_tools_->setManualSceneUpdating(true);

    // Get package path
    package_path_ = ros::package::getPath(PACKAGE_NAME);
    if( package_path_.empty() )
      ROS_FATAL_STREAM_NAMED("product", "Unable to get " << PACKAGE_NAME << " package path" );

    // Load shelf
    shelf_.reset(new ShelfObject(visual_tools_, rviz_visual_tools::BROWN, "shelf_0", package_path_));
    loadShelfContents();
    visualizeShelf();

    // Create the pick place pipeline
    pipeline_.reset(new ManipulationPipeline(verbose_, visual_tools_, planning_scene_monitor_, shelf_));


    ROS_INFO_STREAM_NAMED("apc_manager","APC Manager Ready.");
  }

  /**
   * \brief Destructor
   */
  ~APCManager()
  {}

  /**
   * \brief Main program runner
   * \return true on success
   */
  bool runOrder()
  {
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
        pipeline_->graspObject(orders_[i], verbose_);
      }
    }
    else // debug mode
    {
      ROS_INFO_STREAM_NAMED("apc_manager","Debug mode - grasping only 1 object");
      pipeline_->graspObject(orders_[1], verbose_);
    }

    pipeline_->statusPublisher("Finished");
  }

  /**
   * \brief Load shelf contents
   * \return true on success
   */
  bool loadShelfContents(std::string order_fp)
  {
    // Choose file
    AmazonJSONParser parser(verbose_, visual_tools_);
    std::string file_path = package_path_ + "/" + order_fp;

    // Parse json
    return parser.parse(file_path, package_path_, shelf_, orders_);
  }

  /**
   * \brief Show detailed shelf
   */
  bool visualizeShelf()
  {
    shelf_->visualizeAxis();

    // Show the STL files
    if (false)
    {
      visual_tools_->removeAllCollisionObjects();
      shelf_->createCollisionShelfDetailed();

      std::cout << "triggering planning scene update " << std::endl;
      visual_tools_->triggerPlanningSceneUpdate();

      ROS_INFO_STREAM_NAMED("apc_manager","Sleeping while shelf is displayed");
      ros::Duration(2).sleep();
    }

    // Now show empty shelf to help in reversing robot arms to initial position
    visual_tools_->removeAllCollisionObjects();
    shelf_->createCollisionBodies("", true); // only show the frame
    visual_tools_->triggerPlanningSceneUpdate();

    // Show each shelf
    //shelf_->visualize();

    return true;
  }

  /**
   * \brief Connect to the MoveIt! planning scene messages
   */
  bool loadPlanningSceneMonitor()
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
    ros::Duration(0.5).sleep();

    // Wait for complete state to be recieved
    std::vector<std::string> missing_joints;
    int counter = 0;
    while( !planning_scene_monitor_->getStateMonitor()->haveCompleteState() && ros::ok() )
    {
      ROS_DEBUG_STREAM_NAMED("apc_manager","Waiting for complete state...");
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

}; // end class

} // end namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "apc_manager");
  ROS_INFO_STREAM_NAMED("main", "Starting Amazon Picking Challenge Manager...");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Check for verbose flag
  bool verbose = false;
  if (argc > 1)
  {
    for (std::size_t i = 0; i < argc; ++i)
    {
      if (strcmp(argv[i], "--verbose") == 0)
      {
        ROS_INFO_STREAM_NAMED("main","Running in VERBOSE mode (slower)");
        verbose = true;
      }
    }
  }

  baxter_apc_main::APCManager manager(verbose);
  manager.runOrder();

  // Shutdown
  ros::Duration(2.0).sleep();
  ROS_INFO_STREAM_NAMED("main", "Shutting down.");
  ros::shutdown();

  return 0;
}

//#endif
