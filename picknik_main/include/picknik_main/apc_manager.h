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
#include <picknik_main/manipulation_data.h>
#include <picknik_main/perception_interface.h>

// Picknik Msgs
#include <picknik_msgs/FindObjectsAction.h>

// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>

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
   * \param Show the experience database after each plan
   * \param autonomous - whether it should pause for human input
   */
  APCManager(bool verbose, std::string order_file_path, bool use_experience, bool show_database, bool autonomous = false);

  /**
   * \brief Destructor
   */
  ~APCManager()
  {}

  /**
   * \brief Check if all communication is properly active
   * \return true on success
   */
  bool checkSystemReady();

  /**
   * \brief Load the shelf and products
   * \param shelf to focus on. rest of shelves will be disabled
   * \return true on success
   */
  bool focusSceneOnBin( const std::string& bin_name );

  /**
   * \brief Remote control from Rviz
   */
  void remoteNextCallback(const std_msgs::Bool::ConstPtr& msg);

  /**
   * \brief Remote control from Rviz
   */
  void remoteAutoCallback(const std_msgs::Bool::ConstPtr& msg);

  /**
   * \brief Recieves inputs from joystick
   * \param input - description
   * \return true on success
   */
  void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);

  /**
   * \brief Main program runner
   * \param Which product in the order to skip ahead to
   * \param jump_to - which step in manipulation to start at
   * \param num_orders - how many products to pick from the order, 0 = all
   * \return true on success
   */
  bool runOrder(std::size_t order_start = 0, std::size_t jump_to = 0,
                std::size_t num_orders = 0);


  /**
   * \brief Grasp object once we know the pose
   * \return true on success
   */
  bool graspObjectPipeline(WorkOrder order, bool verbose, std::size_t jump_to = 0);

  /**
   * \brief Get the pose of a requested object
   * \param global_object_pose
   * \param order - desired object
   * \return true on success
   */
  bool perceiveObject(Eigen::Affine3d& global_object_pose, WorkOrder order, bool verbose);
  bool perceiveObjectFake(Eigen::Affine3d& global_object_pose, ProductObjectPtr& product);


  /**
   * \brief Wait until user presses a button
   * \return true on success
   */
  bool waitForNextStep();

  /**
   * \brief Move object into the goal bin
   * \return true on success
   */
  bool placeObjectInGoalBin(const robot_model::JointModelGroup* arm_jmg);

  /**
   * \brief Move both arms to their start location
   * \param optionally specify which arm to use
   * \return true on success
   */
  bool moveToStartPosition(const robot_model::JointModelGroup* arm_jmg = NULL);

  /**
   * \brief Move to location to get rid of product
   * \param optionally specify which arm to use
   * \return true on success
   */
  bool moveToDropOffPosition(const robot_model::JointModelGroup* arm_jmg = NULL);

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
   * \brief Simple script to move hand up and down on z axis from whereever it currently is
   * \return true on success
   */
  bool testUpAndDown();

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
   * \brief Record a trajectory for calibration
   * \return true on success
   */
  bool recordCalibrationTrajectory();

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
   * \brief Show detailed shelf
   */
  bool visualizeShelf();

  /**
   * \brief Connect to the MoveIt! planning scene messages
   */
  bool loadPlanningSceneMonitor();

  /**
   * \brief Publish where the robot currently is
   */
  void publishCurrentState();

  /**
   * \brief Step to next step
   * \return true on success
   */
  bool setReadyForNextStep();

  /**
   * \brief Enable autonomous mode
   */
  void setAutonomous(bool autonomous = true);

  /**
   * \brief Get the autonomous mode
   * \return true if is in autonomous mode
   */
  bool getAutonomous();

  /**
   * \brief Allow other nodes such as rviz to request the entire planning scene
   */
  bool getPlanningSceneService(moveit_msgs::GetPlanningScene::Request &req, moveit_msgs::GetPlanningScene::Response &res);


private:

  // A shared node handle
  ros::NodeHandle nh_private_;
  ros::NodeHandle nh_root_;
  boost::shared_ptr<tf::TransformListener> tf_;

  // Show more visual and console output, with general slower run time.
  bool verbose_;

  // For visualizing things in rviz
  VisualsPtr visuals_;

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
  ManipulationPtr manipulation_;

  // Perception interface
  PerceptionInterfacePtr perception_interface_;

  // Helper classes
  //LearningPipelinePtr learning_;

  // Remote control
  ros::Subscriber remote_next_control_;
  ros::Subscriber remote_run_control_;
  ros::Subscriber remote_joy_;

  // Robot-sepcific data for the APC
  ManipulationDataPtr config_;

  // Robot-specific data for generating grasps
  GraspDatas grasp_datas_;

  // Remote control
  bool autonomous_;
  bool next_step_ready_;
  bool is_waiting_;

  // Perception
  bool fake_perception_;
  bool skip_homing_step_;

  // Allow Rviz to request the entire scene at startup
  ros::ServiceServer get_scene_service_;

}; // end class

} // end namespace


#endif
