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

#ifndef PICKNIK_MAIN__PICK_MANAGER
#define PICKNIK_MAIN__PICK_MANAGER

// Picknik
#include <picknik_main/namespaces.h>
#include <picknik_main/manipulation.h>
#include <picknik_main/trajectory_io.h>
#include <picknik_main/visuals.h>
#include <picknik_main/planning_scene_manager.h>
#include <picknik_main/manipulation_data.h>
#include <picknik_main/perception_interface.h>
#include <picknik_main/remote_control.h>
#include <picknik_main/tactile_feedback.h>

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
static const std::string GET_PLANNING_SCENE_SERVICE_NAME =
    "get_planning_scene";  // name of the service that can be used to query the planning scene

class PickManager
{
public:
  /**
   * \brief Constructor
   * \param verbose - run in debug mode
   */
  PickManager(bool verbose);

  /**
   * \brief Check if all communication is properly active
   * \return true on success
   */
  bool checkSystemReady();

  /**
   * \brief Test the end effectors
   * \param input - description
   * \return true on success
   */
  bool testEndEffectors();

  /**
   * \brief Testing ideal attached object
   * \return true on success
   */
  bool testIdealAttachedCollisionObject();

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
   * \brief Record trajectory
   * \return true on success
   */
  bool recordTrajectory();

  /**
   * \return true on success
   */
  bool playbackTrajectory();

  /**
   * \brief
   * \param input - description
   * \return true on success
   */
  bool moveToStartPosition(JointModelGroup* arm_jmg, bool check_validity = true);

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
  bool getPlanningSceneService(moveit_msgs::GetPlanningScene::Request& req,
                               moveit_msgs::GetPlanningScene::Response& res);

  /**
   * \brief Get remote control functionality
   */
  RemoteControlPtr getRemoteControl();

  /**
   * \brief Disable collision checking for certain pairs
   */
  bool allowCollisions(JointModelGroup* arm_jmg);

  /**
   * \brief Move to a pose named in the SRDF
   * \param pose_name
   * \return true on success
   */
  bool gotoPose(const std::string& pose_name);

  /**
   * \brief Test IK solver
   * \return true on success
   */
  bool testIKSolver();

  /**
   * \brief Move arm in circle for calibration
   * \return true on success
   */
  bool calibrateInCircle();

  /**
   * \brief Playback waypoint path specified in a csv
   * \return true on success
   */
  bool playbackWaypointsFromFile();

  /**
   * \brief Loop through incremental grasp widths for testing
   * \return true on success
   */
  bool testGraspWidths();

  void processMarkerPose(const geometry_msgs::Pose& pose, bool move);

  /** \brief Peg in hole demo */
  void insertion();

  /** \brief Allow interactive markers to control robot */
  void enableTeleoperation();

  /** \brief Start responding to touch sensors */
  void touchControl();

  /** \brief Initialize with correct location */
  void setupInteractiveMarker();

  VisualsPtr getVisuals() { return visuals_; }
  planning_scene_monitor::PlanningSceneMonitorPtr getPlanningSceneMonitor() const
  {
    return planning_scene_monitor_;
  }

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
  Eigen::Affine3d status_position_;  // where to display messages

  // Properties
  std::string order_file_path_;

  // File path to ROS package on drive
  std::string package_path_;

  // Remote control for dealing with GUIs
  RemoteControlPtr remote_control_;

  // Main worker
  ManipulationPtr manipulation_;

  // Perception interface
  PerceptionInterfacePtr perception_interface_;

  // Line tracking interface
  TactileFeedbackPtr tactile_feedback_;

  // Helper classes
  // LearningPipelinePtr learning_;

  // Robot-sepcific data for the APC
  ManipulationDataPtr config_;

  // Robot-specific data for generating grasps
  moveit_grasps::GraspDatas grasp_datas_;

  // Perception
  bool fake_perception_;
  // bool skip_homing_step_;

  // Allow Rviz to request the entire scene at startup
  ros::ServiceServer get_scene_service_;

  // Locations to dropoff products
  EigenSTL::vector_Affine3d dropoff_locations_;
  std::size_t next_dropoff_location_;

  // Allow loading and saving trajectories to file
  TrajectoryIOPtr trajectory_io_;

  bool teleoperation_enabled_;
  Eigen::Affine3d interactive_marker_pose_;

};  // end class

}  // end namespace

#endif
