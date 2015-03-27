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

#ifndef PICKNIK_MAIN__MANIPULATION
#define PICKNIK_MAIN__MANIPULATION

// Picknik
#include <picknik_main/shelf.h>
#include <picknik_main/namespaces.h>
#include <picknik_main/visuals.h>
#include <picknik_main/manipulation_data.h>

// ROS
#include <ros/ros.h>

// MoveIt
#include <ompl_visual_tools/ompl_visual_tools.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/plan_execution/plan_execution.h>

// OMPL
#include <ompl/tools/experience/ExperienceSetup.h>

// Grasp generation
#include <moveit_grasps/grasp_generator.h>
#include <moveit_grasps/grasp_data.h>
#include <moveit_grasps/grasp_filter.h>

namespace planning_pipeline
{
MOVEIT_CLASS_FORWARD(PlanningPipeline);
}

namespace picknik_main
{

typedef std::map<const robot_model::JointModelGroup*,moveit_grasps::GraspDataPtr> GraspDatas;

MOVEIT_CLASS_FORWARD(APCManager);
MOVEIT_CLASS_FORWARD(Manipulation);

class Manipulation
{
public:

  /**
   * \brief Constructor
   * \param verbose - run in debug mode
   */
  Manipulation(bool verbose, VisualsPtr visuals,
               planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor,
               ManipulationData config, GraspDatas grasp_datas,
               APCManager* parent,
               ShelfObjectPtr shelf, bool use_experience, bool show_database);

  /**
   * \brief Destructor
   */
  ~Manipulation()
  {}

  /**
   * \brief Choose the grasp for the object
   * \return true on success
   */
  bool chooseGrasp(const Eigen::Affine3d& object_pose, const robot_model::JointModelGroup* arm_jmg,
                   moveit_grasps::GraspCandidatePtr& chosen, bool verbose);

  /**
   * \brief Show simple collision wall that protects shelf
   * \return true on success
   */
  bool createCollisionWall();

  /**
   * \brief Move to any pose as defined in the SRDF
   * \return true on success
   */
  bool moveToPose(const robot_model::JointModelGroup* arm_jmg, const std::string &pose_name, double velocity_scaling_factor);

  /**
   * \brief Move EE to a particular pose by solving with IK
   * \param input - description
   * \return true on success
   */
  bool moveEEToPose(const Eigen::Affine3d& ee_pose, double velocity_scaling_factor, const robot_model::JointModelGroup* arm_jmg);

  /**
   * \brief Send a planning request to moveit and execute
   * \return true on success
   */
  bool move(const moveit::core::RobotStatePtr& start, const moveit::core::RobotStatePtr& goal,
            const robot_model::JointModelGroup* arm_jmg, double velocity_scaling_factor,
            bool verbose, bool execute_trajectory = true);

  /**
   * \brief Get planning debug info
   * \return string describing result
   */
  std::string getActionResultString(const moveit_msgs::MoveItErrorCodes &error_code, bool planned_trajectory_empty);

  /**
   * \brief Send a single state to the controllers for execution
   * \return true on success
   */
  bool executeState(const moveit::core::RobotStatePtr goal_state, const moveit::core::JointModelGroup *jmg,
                    double velocity_scaling_factor);

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
   * \brief Translate arm left and right
   * \return true on success
   */
  bool executeLeftPath(const moveit::core::JointModelGroup *arm_jmg, const double &desired_left_distance, bool left = true);

  /**
   * \brief After grasping an object, pull object out of shelf in reverse
   * \return true on success
   */
  bool executeRetreatPath(const moveit::core::JointModelGroup *arm_jmg, double desired_approach_distance = 0.25, bool retreat = true);

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
   * \brief Choose which arm to use for a particular task
   * \return joint model group of arm to use in manip
   */
  const robot_model::JointModelGroup* chooseArm(const Eigen::Affine3d& ee_pose);

  /**
   * \brief Run calibration routine
   * \return true on success
   */
  bool calibrateCamera();

  /**
   * \brief Move camera around to get good view of bin
   * \return true on success
   */
  bool perturbCamera(BinObjectPtr bin);

  /**
   * \brief Move camera to in front of bin
   * \return true on success
   */
  bool moveCameraToBin(BinObjectPtr bin);

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
                                      const robot_model::JointModelGroup* arm_jmg,
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
  bool openEndEffector(bool open, const robot_model::JointModelGroup* arm_jmg);

  /**
   * \brief Set a robot state to have an open or closed EE. Does not actually affect hardware
   * \return true on success
   */
  bool setStateWithOpenEE(bool open, moveit::core::RobotStatePtr robot_state);

  /**
   * \brief Check that the controllers are connected and ready
   * \return true on success
   */
  bool checkExecutionManager();

  /**
   * \brief Send trajectory message to robot controllers
   * \return true on success
   */
  bool executeTrajectory(moveit_msgs::RobotTrajectory &trajectory_msg);

  /**
   * \brief Save a trajectory as CSV to file
   * \return true on success
   */
  bool saveTrajectory(const moveit_msgs::RobotTrajectory &trajectory_msg, const std::string &file_name);

  /**
   * \brief Prevent a product from colliding with the fingers
   * \return true on success
   */
  bool allowFingerTouch(const std::string& object_name, const robot_model::JointModelGroup* arm_jmg);

  /**
   * \brief Load it
   * \return true on success
   */
  void loadPlanningPipeline();

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
  bool statesEqual(const moveit::core::RobotState &s1, const moveit::core::RobotState &s2, const robot_model::JointModelGroup* arm_jmg);

  /**
   * \brief Show the trajectories saved in the experience database
   * \return true on success
   */
  void displayLightningPlans(ompl::tools::ExperienceSetupPtr experience_setup, const robot_model::JointModelGroup* arm_jmg);

  /**
   * \brief Visulization function
   * \param input - description
   * \return true on success
   */
  bool visualizeGrasps(std::vector<moveit_grasps::GraspCandidatePtr> grasp_candidates, const moveit::core::JointModelGroup *arm_jmg,
                       bool show_cartesian_path = true);

  /**
   * \brief Visalize ik solutions
   * \param input - description
   * \return true on success
   */
  bool visualizeIKSolutions(std::vector<moveit_grasps::GraspCandidatePtr> grasp_candidates, const moveit::core::JointModelGroup* arm_jmg,
                            double display_time = 2);

  /**
   * \brief Update the current_state_ RobotState with latest from planning scene
   */
  moveit::core::RobotStatePtr getCurrentState();

  /**
   * \brief Check if current state is in collision or out of bounds
   * \return true if not in collision and not out of bounds
   */
  bool checkCurrentCollisionAndBounds(const robot_model::JointModelGroup* arm_jmg);

  /**
   * \brief Check if states are in collision or out of bounds
   * \param start_state to check
   * \param goal_state OPTIONAL to check
   * \return true if not in collision and not out of bounds
   */
  bool checkCollisionAndBounds(const robot_state::RobotStatePtr &start_state,
                               const robot_state::RobotStatePtr &goal_state = robot_state::RobotStatePtr());

  /**
   * \brief Get location to save a CSV file
   * \param file_path - the variable to populate with a path
   * \param file_name - the desired name of the file
   * \return true on success
   */
  bool getFilePath(std::string &file_path, const std::string &file_name = "moveit_export.csv") const;

  /**
   * \brief Setup products randomly
   * \return true on success
   */
  bool generateRandomProductPoses();

protected:

  // A shared node handle
  ros::NodeHandle nh_;

  // Show more visual and console output, with general slower run time.
  bool verbose_;

  // Trajectory execution
  trajectory_execution_manager::TrajectoryExecutionManagerPtr trajectory_execution_manager_;
  boost::shared_ptr<plan_execution::PlanExecution> plan_execution_;

  // For visualizing things in rviz
  VisualsPtr visuals_;
  ovt::OmplVisualToolsPtr ompl_visual_tools_;

  // Core MoveIt components
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  robot_model::RobotModelConstPtr robot_model_;
  planning_pipeline::PlanningPipelinePtr planning_pipeline_;

  // Allocated memory for robot state
  moveit::core::RobotStatePtr current_state_;

  // Properties
  ShelfObjectPtr shelf_;

  // Robot-sepcific data for the APC
  ManipulationData config_;

  // Robot-specific data for generating grasps
  GraspDatas grasp_datas_;

  // The parent manager
  APCManager* parent_;

  // Experience-based planning
  bool use_experience_;
  bool show_database_;
  bool use_logging_;
  std::ofstream logging_file_;

  // User feedback
  Eigen::Affine3d status_position_; // where to display messages
  Eigen::Affine3d order_position_; // where to display messages

  // Grasp generator
  moveit_grasps::GraspGeneratorPtr grasp_generator_;
  moveit_grasps::GraspFilterPtr grasp_filter_;

}; // end class

} // end namespace

namespace
{
bool isStateValid(const planning_scene::PlanningScene *planning_scene, bool verbose,
                  picknik_main::VisualsPtr visuals, robot_state::RobotState *state,
                  const robot_state::JointModelGroup *group, const double *ik_solution);
}

#endif
