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

#ifndef BAXTER_APC_MAIN__MANIPULATION_PIPELINE
#define BAXTER_APC_MAIN__MANIPULATION_PIPELINE

#include <baxter_apc_main/shelf.h>

// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>

// MoveIt
#include <moveit/collision_detection/world.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ompl_visual_tools/ompl_visual_tools.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/plan_execution/plan_execution.h>

// Grasp generation
#include <moveit_grasps/grasps.h>
#include <moveit_grasps/grasp_data.h>
#include <moveit_grasps/grasp_filter.h>

// OMPL
#include <ompl/tools/experience/ExperienceSetup.h>

namespace baxter_apc_main
{

MOVEIT_CLASS_FORWARD(ManipulationPipeline);

static const std::string START_POSE = "both_ready"; // where to move baxter to initially

class ManipulationPipeline
{
public:

  /**
   * \brief Constructor
   * \param verbose - run in debug mode
   */
  ManipulationPipeline(bool verbose, moveit_visual_tools::MoveItVisualToolsPtr visual_tools,
                       planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor,
                       ShelfObjectPtr shelf, bool use_experience, bool show_database);

  /**
   * \brief Destructor
   */
  ~ManipulationPipeline()
  {}

  /**
   * \brief Choose the grasp for the object
   * \return true on success
   */
  bool chooseGrasp(const Eigen::Affine3d& object_pose, const robot_model::JointModelGroup* jmg,
                   moveit_grasps::GraspSolution& chosen, bool verbose);

  /**
   * \brief Load the shelf and products
   * \param shelf to focus on. rest of shelves will be disabled
   * \return true on success
   */
  bool setupPlanningScene( const std::string& bin_name );

  /**
   * \brief Grasp product
   */
  bool graspObject( WorkOrder order, bool verbose );

  /**
   * \brief Grasp object once we know the pose
   * \return true on success
   */
  bool graspObjectPipeline(const Eigen::Affine3d& object_pose, WorkOrder order, bool verbose);

  /**
   * \brief Choose which arm to use for a particular task
   * \return joint model group of arm to use in manip
   */
  const robot_model::JointModelGroup* chooseArm(const Eigen::Affine3d& object_pose);

  /**
   * \brief Move both arms to their start location
   * \return true on success
   */
  bool moveToStartPosition(const robot_model::JointModelGroup* jmg = NULL);

  /**
   * \brief Send a planning request to moveit and execute
   * \return true on success
   */
  bool move(const moveit::core::RobotStatePtr& start, const moveit::core::RobotStatePtr& goal,
            const robot_model::JointModelGroup* jmg, bool verbose);

  /**
   * \brief After grasping an object, lift object up slightly
   * \return true on success
   */
  bool executeLiftPath(const moveit::core::JointModelGroup *jmg);

  /**
   * \brief After grasping an object, pull object out of shelf in reverse
   * \return true on success
   */
  bool executeRetrievalPath(const moveit::core::JointModelGroup *jmg);

  /**
   * \brief Function for testing multiple directions
   * \param approach_direction - direction to move end effector straight
   * \param desired_approach_distance - distance the origin of a robot link needs to travel
   * \param trajectory_msg - resulting path
   * \param robot_state - used as the base state of the robot when starting to move
   * \return true on success
   */
  bool computeStraightLinePath( Eigen::Vector3d approach_direction, 
                                double desired_approach_distance,
                                std::vector<robot_state::RobotStatePtr>& robot_state_trajectory,
                                robot_state::RobotStatePtr robot_state,
                                const moveit::core::JointModelGroup *jmg);

  /**
   * \brief Convert and parameterize a trajectory with velocity information
   * \return true on success
   */
  bool convertRobotStatesToTrajectory(const std::vector<robot_state::RobotStatePtr>& robot_state_trajectory,
                                      moveit_msgs::RobotTrajectory& trajectory_msg,
                                      const robot_model::JointModelGroup* jmg);

  /**
   * \brief Test it
   * \return true on success
   */
  bool testEndEffector();

  /**
   * \brief Open both end effectors in hardware
   * \return true on success
   */
  bool openEndEffectors(bool open);

  /**
   * \brief open/close grippers
   * \param bool if it should be open or closed
   * \return
   */
  bool openEndEffector(bool open, const robot_model::JointModelGroup* jmg);

  /**
   * \brief Set a robot state to have an open or closed EE. Does not actually affect hardware
   * \return true on success
   */
  bool openEndEffector(bool open, moveit::core::RobotStatePtr robot_state);

  /**
   * \brief Send trajectories to Baxter
   * \return true on success
   */
  bool executeTrajectoryMsg(moveit_msgs::RobotTrajectory trajectory_msg);

  /**
   * \brief Prevent a product from colliding with the fingers
   * \return true on success
   */
  bool allowFingerTouch(const std::string& object_name, const robot_model::JointModelGroup* jmg);

  /**
   * \brief Load it
   * \return true on success
   */
  void loadPlanningPipeline();

  /**
   * \brief Create a trajectory execution manager
   * \return true on success
   */
  bool loadPlanExecution();

  /**
   * \brief Central Rviz status visualizer
   * \return true on success
   */
  bool statusPublisher(const std::string &status);

  /**
   * \brief Central Rviz status visualizer for orders
   * \return true on success
   */
  bool orderPublisher(WorkOrder& order);

  /**
   * \brief Helper function for determining if robot is already in desired state
   * \param robotstate to compare to
   * \param robotstate to compare to
   * \param only compare joints in this joint model group
   * \return true if states are close enough in similarity
   */
  bool statesEqual(const moveit::core::RobotState &s1, const moveit::core::RobotState &s2, const robot_model::JointModelGroup* jmg);

  /**
   * \brief Show the trajectories saved in the experience database
   * \return true on success
   */
  void displayLightningPlans(ompl::tools::ExperienceSetupPtr experience_setup, const robot_model::JointModelGroup* jmg);

protected:

  // A shared node handle
  ros::NodeHandle nh_;

  // Show more visual and console output, with general slower run time.
  bool verbose_;

  // For visualizing things in rviz
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
  ompl_visual_tools::OmplVisualToolsPtr ompl_visual_tools_;

  // Core MoveIt components
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  robot_model::RobotModelConstPtr robot_model_;
  planning_pipeline::PlanningPipelinePtr planning_pipeline_;

  // Allocated memory for robot states
  moveit::core::RobotStatePtr robot_state_;
  moveit::core::RobotStatePtr start_state_;

  // Group for each arm
  const robot_model::JointModelGroup* left_arm_;
  const robot_model::JointModelGroup* right_arm_;

  // Grasp generator
  moveit_grasps::GraspsPtr grasps_;
  moveit_grasps::GraspFilterPtr grasp_filter_;

  // robot-specific data for generating grasps
  std::map<const robot_model::JointModelGroup*,moveit_grasps::GraspData> grasp_datas_;

  // Trajectory execution
  trajectory_execution_manager::TrajectoryExecutionManagerPtr trajectory_execution_manager_;
  boost::shared_ptr<plan_execution::PlanExecution> plan_execution_;

  // Properties
  ShelfObjectPtr shelf_;

  // User feedback
  Eigen::Affine3d status_position_; // where to display messages
  Eigen::Affine3d order_position_; // where to display messages

  // Experience-based planning
  bool use_experience_;
  bool show_database_;
  bool use_logging_;
  std::ofstream logging_file_;
}; // end class

} // end namespace

#endif
