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

#ifndef PICKNIK_MAIN__SHELF_MANAGER
#define PICKNIK_MAIN__SHELF_MANAGER

// Picknik
#include <picknik_main/namespaces.h>
#include <picknik_main/amazon_json_parser.h>
#include <picknik_main/shelf.h>
#include <picknik_main/manipulation.h>
//#include <picknik_main/learning_pipeline.h>
#include <picknik_main/visuals.h>
#include <picknik_main/planning_scene_manager.h>
#include <picknik_main/manipulation_data.h>
#include <picknik_main/perception_interface.h>
#include <picknik_main/remote_control.h>

// Picknik Msgs
#include <picknik_msgs/FindObjectsAction.h>

// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>

// MoveIt!
#include <moveit_msgs/GetPlanningScene.h>

namespace picknik_main
{

static const std::string ROBOT_DESCRIPTION = "robot_description";
static const std::string JOINT_STATE_TOPIC = "/robot/joint_states";
static const std::string PACKAGE_NAME = "picknik_main";
static const std::string GET_PLANNING_SCENE_SERVICE_NAME = "get_planning_scene"; // name of the service that can be used to query the planning scene

//MOVEIT_CLASS_FORWARD(APCManager);

class APCManager
{
public:

  /**
   * \brief Constructor
   * \param verbose - run in debug mode
   * \param order_file_path
   * \param Use an experience database in planning
   * \param autonomous - whether it should pause for human input, except executing trajectories which is always manual
   * \param full_autonomous - whether it should pause for human input
   * \param fake_execution - when true velocities are full speed
   * \param fake_perception   
   */
  APCManager(bool verbose, std::string order_file_path, bool use_experience, bool autonomous = false, bool full_autonomous = false, 
             bool fake_execution = false, bool fake_perception = false);

  /**
   * \brief Check if all communication is properly active
   * \return true on success
   */
  bool checkSystemReady(bool remove_from_shelf = true);

  /**
   * \brief Load the shelf and products
   * \param shelf to focus on. rest of shelves will be disabled
   * \return true on success
   */
  bool focusSceneOnBin( const std::string& bin_name );

  /**
   * \brief Main program runner
   * \param Which product in the order to skip ahead to
   * \param jump_to - which step in manipulation to start at
   * \param num_orders - how many products to pick from the order, 0 = all
   * \return true on success
   */
  bool mainOrderProcessor(std::size_t order_start = 0, std::size_t jump_to = 0, std::size_t num_orders = 0);

  /**
   * \brief Main program runner
   * \param Which product in the order to skip ahead to
   * \param jump_to - which step in manipulation to start at
   * \param num_orders - how many products to pick from the order, 0 = all
   * \return true on success
   */
  bool runOrder(std::size_t order_start = 0, std::size_t jump_to = 0, std::size_t num_orders = 0);

  /**
   * \brief Grasp object once we know the pose
   * \return true on success
   */
  bool graspObjectPipeline(WorkOrder order, bool verbose, std::size_t jump_to = 0);

  /**
   * \brief Generate a discretized array of possible pre-grasps and save into experience database
   * \return true on success
   */
  bool trainExperienceDatabase();

  /**
   * \brief Test the end effectors
   * \param input - description
   * \return true on success
   */
  bool testEndEffectors();

  /**
   * \brief Test switching planning scene modes
   * \return true on success
   */
  bool testVisualizeShelf();

  /**
   * \brief Simple script to move hand up and down on z axis from whereever it currently is
   * \return true on success
   */
  bool testUpAndDown();

  /**
   * \brief Move hand in and out of bin from whereever it currently is
   * \return true on success
   */
  bool testInAndOut();

  /**
   * \brief Script for moving arms to locations of corner of shelf
   * \return true on success
   */
  bool testShelfLocation();

  /**
   * \brief Send to goal bin location
   * \return true on success
   */
  bool testGoalBinPose();

  /**
   * \brief Check if current state is in collision
   * \return true on success
   */
  bool testInCollision();

  /**
   * \brief Plan to random valid motions
   * \return true on success
   */
  bool testRandomValidMotions();

  /**
   * \brief Show random product locations
   * \param input - description
   * \return true on success
   */
  bool createRandomProductPoses();
  
  /**
   * \brief Send arm to camera positions
   * \return true on success
   */
  bool testCameraPositions();

  /**
   * \brief Test moving the camera around for calibration
   * \return true on success
   */
  bool calibrateCamera();

  /**
   * \brief Test moving joints to extreme limits
   * \return true on success
   */
  bool testJointLimits();

  /**
   * \brief Send arm(s) to home position
   * \return true on success
   */
  bool testGoHome();

  /**
   * \brief Get cartesian path for grasping object
   * \return true on success
   */
  bool testApproachLiftRetreat();

  /**
   * \brief Get the XML of a SDF pose of joints
   * \return true on success
   */
  bool getSRDFPose();

  /**
   * \brief Test grasp generator abilities and score results
   * \return true on success
   */
  bool testGraspGenerator();

  /**
   * \brief Requesting perception test
   * \return true on success
   */
  bool testPerceptionComm();

  /**
   * \brief Given an id of a bin (starting at 0=A) record the trajectory of a camera observing it
   * \return true on success
   */
  bool recordBinWithCamera(std::size_t bin_id);

  /**
   * \brief Given an id of a bin (starting at 0=A) playback the trajectory of a camera observing it
   * \return true on success
   */
  bool perceiveBinWithCamera(std::size_t bin_id);

  /**
   * \brief Move camera around a bin by playing back a file
   * \param bin
   * \return true on success
   */
  bool perceiveBinWithCamera(BinObjectPtr bin);

  /**
   * \brief Record camera moving around a bin
   * \param bin
   * \return true on success
   */
  bool recordBinWithCamera(BinObjectPtr bin);

  /**
   * \brief Record a trajectory for calibration
   * \return true on success
   */
  bool recordCalibrationTrajectory();

  /**
   * \brief Get the pose of a requested object
   * \param global_object_pose
   * \param order - desired object
   * \return true on success
   */
  bool perceiveObject(WorkOrder work_order, bool verbose);
  bool perceiveObjectFake(WorkOrder work_order);

  /**
   * \brief Move object into the goal bin
   * \return true on success
   */
  bool placeObjectInGoalBin(const robot_model::JointModelGroup* arm_jmg);

  /**
   * \brief Lift from goal bin
   * \return true on success
   */
  bool liftFromGoalBin(const robot_model::JointModelGroup* arm_jmg);

  /**
   * \brief Move both arms to their start location
   * \param optionally specify which arm to use
   * \return true on success
   */
  bool moveToStartPosition(const robot_model::JointModelGroup* arm_jmg = NULL, bool check_validity = true);

  /**
   * \brief Move to location to get rid of product
   * \param optionally specify which arm to use
   * \return true on success
   */
  bool moveToDropOffPosition(const robot_model::JointModelGroup* arm_jmg);

  /**
   * \brief Load single product, one per shelf, for testing
   * \param product_name
   * \return true on success
   */
  bool loadShelfWithOnlyOneProduct(const std::string& product_name);

  /**
   * \brief Load shelf contents
   * \return true on success
   */
  bool loadShelfContents(std::string order_file_path);

  /**
   * \brief Connect to the MoveIt! planning scene messages
   */
  bool loadPlanningSceneMonitor();

  /**
   * \brief Publish where the robot currently is
   */
  void publishCurrentState();

  /**
   * \brief Allow other nodes such as rviz to request the entire planning scene
   */
  bool getPlanningSceneService(moveit_msgs::GetPlanningScene::Request &req, moveit_msgs::GetPlanningScene::Response &res);

  /**
   * \brief Get remote control functionality
   */
  RemoteControlPtr getRemoteControl();

  /**
   * \brief Disable collision checking for certain pairs
   */
  bool allowCollisions();

  /**
   * \brief Attach a product to an arm for planning
   * \return true on success
   */
  bool attachProduct(ProductObjectPtr product, const robot_model::JointModelGroup* arm_jmg);

  /**
   * \brief Show experience database
   * \return true on success
   */
  bool displayExperienceDatabase();

  /**
   * \brief Create locations to place products
   * \return true on success
   */
  bool generateGoalBinLocations();

  /**
   * \brief Central Rviz status visualizer
   * \return true on success
   */
  bool statusPublisher(const std::string &status);

  /**
   * \brief Test various product benchmarks
   * \return true on success
   */
  bool unitTests();

  /**
   * \brief Update settings for new unit test
   * \return true on success
   */
  bool startUnitTest(const std::string &json_file, const std::string &test_name, const Eigen::Affine3d &product_pose);

  /**
   * \brief Move to a pose named in the SRDF
   * \param pose_name
   * \return true on success
   */
  bool gotoPose(const std::string& pose_name);

private:

  // A shared node handle
  ros::NodeHandle nh_private_;
  ros::NodeHandle nh_root_;
  boost::shared_ptr<tf::TransformListener> tf_;

  // Show more visual and console output, with general slower run time.
  bool verbose_;

  // For visualizing things in rviz
  VisualsPtr visuals_;
  PlanningSceneManagerPtr planning_scene_manager_;

  // Core MoveIt components
  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  robot_model::RobotModelPtr robot_model_;
  planning_scene::PlanningScenePtr planning_scene_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  // User feedback
  Eigen::Affine3d status_position_; // where to display messages

  // Properties
  ShelfObjectPtr shelf_;
  WorkOrders orders_;
  std::string order_file_path_;

  // File path to ROS package on drive
  std::string package_path_;

  // Remote control for dealing with GUIs
  RemoteControlPtr remote_control_;

  // Main worker
  ManipulationPtr manipulation_;

  // Perception interface
  PerceptionInterfacePtr perception_interface_;

  // Helper classes
  //LearningPipelinePtr learning_;

  // Robot-sepcific data for the APC
  ManipulationDataPtr config_;

  // Robot-specific data for generating grasps
  moveit_grasps::GraspDatas grasp_datas_;

  // Perception
  bool fake_perception_;
  bool skip_homing_step_;

  // Allow Rviz to request the entire scene at startup
  ros::ServiceServer get_scene_service_;

  // Locations to dropoff products
  EigenSTL::vector_Affine3d dropoff_locations_;
  std::size_t next_dropoff_location_;

}; // end class

} // end namespace


#endif
