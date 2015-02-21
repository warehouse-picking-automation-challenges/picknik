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

#ifndef BAXTER_APC_MAIN__MANIPULATION_PIPELINE
#define BAXTER_APC_MAIN__MANIPULATION_PIPELINE

#include <baxter_apc_main/shelf.h>
#include <baxter_apc_main/namespaces.h>
#include <baxter_apc_main/visuals.h>

// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>

// MoveIt
#include <moveit/collision_detection/world.h>
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

static const std::string START_POSE = "both_neutral"; // where to move baxter to initially
const double APPROACH_DISTANCE_DESIRED = 0.1; // amount beyond min distance

class ManipulationPipeline
{
public:

  /**
   * \brief Constructor
   * \param verbose - run in debug mode
   */
  ManipulationPipeline(bool verbose,
                       VisualsPtr visuals,
                       planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor,
                       ShelfObjectPtr shelf, bool use_experience, bool show_database);

  /**
   * \brief Destructor
   */
  ~ManipulationPipeline()
  {}

  /**
   * \brief Helper for initilizing the robot states
   */
  bool loadRobotStates();

  /**
   * \brief Load the shelf and products
   * \param shelf to focus on. rest of shelves will be disabled
   * \return true on success
   */
  bool setupPlanningScene( const std::string& bin_name );

  /**
   * \brief Show simple collision wall that protects shelf
   * \return true on success
   */
  bool createCollisionWall();

  /**
   * \brief Grasp product
   */
  bool graspObject( WorkOrder order, bool verbose, std::size_t jump_to = 0);

  /**
   * \brief Grasp object once we know the pose
   * \return true on success
   */
  bool graspObjectPipeline(const Eigen::Affine3d& object_pose, WorkOrder order, bool verbose, std::size_t jump_to = 0);

  /**
   * \brief Choose the grasp for the object
   * \return true on success
   */
  bool chooseGrasp(const Eigen::Affine3d& object_pose, const robot_model::JointModelGroup* jmg,
                   moveit_grasps::GraspSolution& chosen, bool verbose);

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
            const robot_model::JointModelGroup* jmg, bool verbose, bool execute_trajectory = true,
            bool show_database = true);

  /**
   * \brief Simple testing script to open close EEs
   * \return true on success
   */
  bool testEndEffectors(bool open);

  /**
   * \brief Send a single state to the controllers for execution
   * \return true on success
   */
  bool executeState(const moveit::core::RobotStatePtr robot_state, const moveit::core::JointModelGroup *jmg);

  /**
   * \brief Generate the straight line path from pregrasp to grasp
   * \param input - description
   * \return true on success
   */
  bool generateApproachPath(const moveit::core::JointModelGroup *arm_jmg,
                            moveit_msgs::RobotTrajectory &approach_trajectory_msg,
                            moveit::core::RobotStatePtr pre_grasp_state, moveit::core::RobotStatePtr the_grasp,
                            bool verbose);

  /**
   * \brief After grasping an object, lift object up slightly
   * \return true on success
   */
  bool executeLiftPath(const moveit::core::JointModelGroup *arm_jmg, const double &desired_lift_distance, bool up = true);

  /**
   * \brief After grasping an object, pull object out of shelf in reverse
   * \return true on success
   */
  bool executeRetreatPath(const moveit::core::JointModelGroup *jmg);

  /**
   * \brief Function for testing multiple directions
   * \param approach_direction - direction to move end effector straight
   * \param desired_approach_distance - distance the origin of a robot link needs to travel
   * \param trajectory_msg - resulting path
   * \param robot_state - used as the base state of the robot when starting to move
   * \param arm_jmg - the joint model group of the arm to plan for
   * \param reverse_trajectory - whether to reverse the generated trajectory before displaying visualizations and returning
   * \param path_length - the length of the resulting cartesian path
   * \return true on success
   */
  bool computeStraightLinePath( Eigen::Vector3d approach_direction,
                                double desired_approach_distance,
                                std::vector<robot_state::RobotStatePtr>& robot_state_trajectory,
                                robot_state::RobotStatePtr robot_state,
                                const moveit::core::JointModelGroup *arm_jmg,
                                bool reverse_trajectory,
                                double& path_length);

  /**
   * \brief Move a pose in a specified direction and specified length, where all poses are in the world frame
   * \return true on success
   */
  bool straightProjectPose( const Eigen::Affine3d& original_pose, Eigen::Affine3d& new_pose,
                            const Eigen::Vector3d direction, double distance);

  /**
   * \brief Convert and parameterize a trajectory with velocity information
   * \return true on success
   */
  bool convertRobotStatesToTrajectory(const std::vector<robot_state::RobotStatePtr>& robot_state_trajectory,
                                      moveit_msgs::RobotTrajectory& trajectory_msg,
                                      const robot_model::JointModelGroup* jmg,
                                      const double &velocity_scaling_factor);

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
  bool setStateWithOpenEE(bool open, moveit::core::RobotStatePtr robot_state);

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

  /**
   * \brief Set both arms to home position
   * \param input - description
   * \return true on success
   */
  bool setToDefaultPosition(moveit::core::RobotStatePtr robot_state);

  /**
   * \brief Getter for RobotState
   */
  moveit::core::RobotStatePtr getRobotState()
  {
    return current_state_;
  }

  /**
   * \brief Setter for RobotState
   */
  void setRobotState(moveit::core::RobotStatePtr robot_state)
  {
    current_state_ = robot_state;
  }

  /**
   * \brief Visulization function
   * \param input - description
   * \return true on success
   */
  bool visualizeGrasps(std::vector<moveit_grasps::GraspSolution> filtered_grasps, const moveit::core::JointModelGroup *arm_jmg,
                       bool show_cartesian_path = true);

  /**
   * \brief Visalize ik solutions
   * \param input - description
   * \return true on success
   */
  bool visualizeIKSolutions(std::vector<moveit_grasps::GraspSolution> filtered_grasps, const moveit::core::JointModelGroup* arm_jmg,
                            double display_time = 2);

protected:

  // A shared node handle
  ros::NodeHandle nh_;

  // Show more visual and console output, with general slower run time.
  bool verbose_;

  // For visualizing things in rviz
  VisualsPtr visuals_;
  ovt::OmplVisualToolsPtr ompl_visual_tools_;

  // Core MoveIt components
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  robot_model::RobotModelConstPtr robot_model_;
  planning_pipeline::PlanningPipelinePtr planning_pipeline_;

  // Allocated memory for robot state
  moveit::core::RobotStatePtr current_state_;

  // Group for each arm
  const robot_model::JointModelGroup* left_arm_;
  const robot_model::JointModelGroup* right_arm_;
  const robot_model::JointModelGroup* both_arms_;

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

  // Performance variables
  double main_velocity_scaling_factor_;
  double approach_velocity_scaling_factor_;
  double lift_velocity_scaling_factor_;
  double retreat_velocity_scaling_factor_;

}; // end class

} // end namespace

namespace
{
bool isStateValid(const planning_scene::PlanningScene *planning_scene, bool verbose,
                  baxter_apc_main::VisualsPtr visuals, robot_state::RobotState *state,
                  const robot_state::JointModelGroup *group, const double *ik_solution);
}

#endif
