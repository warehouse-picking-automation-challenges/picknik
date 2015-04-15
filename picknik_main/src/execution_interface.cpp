/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman <dave@dav.ee>
   Desc:   Interface between MoveIt! execution tools and PickNik
*/

// PickNik
#include <picknik_main/execution_interface.h>

// MoveIt
#include <moveit/plan_execution/plan_execution.h>

// ros_control
#include <controller_manager_msgs/ListControllers.h>

// Parameter loading
//#include <rviz_visual_tools/ros_param_utilities.h>

namespace picknik_main
{

ExecutionInterface::ExecutionInterface(bool verbose, RemoteControlPtr remote_control, VisualsPtr visuals,
                                       moveit_grasps::GraspDatas grasp_datas,
                                       planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor,
                                       ManipulationDataPtr config, const std::string& package_path, 
                                       moveit::core::RobotStatePtr current_state)
  : verbose_(verbose)
  , remote_control_(remote_control)
  , visuals_(visuals)
  , grasp_datas_(grasp_datas)
  , planning_scene_monitor_(planning_scene_monitor)
  , config_(config)
  , package_path_(package_path)
  , current_state_(current_state)
  , nh_("~")
{
  // Check that controllers are ready
  zaber_list_controllers_client_ = nh_.serviceClient<controller_manager_msgs::ListControllers>("/jacob/zaber/controller_manager/list_controllers");
  kinova_list_controllers_client_ = nh_.serviceClient<controller_manager_msgs::ListControllers>("/jacob/kinova/controller_manager/list_controllers");

  ROS_INFO_STREAM_NAMED("execution_interface","ExecutionInterface Ready.");
}

bool ExecutionInterface::executeTrajectory(moveit_msgs::RobotTrajectory &trajectory_msg, bool ignore_collision)
{
  trajectory_msgs::JointTrajectory& trajectory = trajectory_msg.joint_trajectory;

  ROS_INFO_STREAM_NAMED("execution_interface","Executing trajectory with " << trajectory.points.size() << " waypoints");

  // Remove acceleration command, not to be used with Baxter
  if (false)
    for (std::size_t i = 0; i < trajectory.points.size(); ++i)
    {
      trajectory.points[i].accelerations.clear();
    }

  // Debug
  ROS_DEBUG_STREAM_NAMED("execution_interface.trajectory","Publishing:\n" << trajectory_msg);

  // Save to file
  if (true)
  {
    // Only save non-finger trajectories
    if (trajectory.joint_names.size() > 3)
    {
      static std::size_t trajectory_count = 0;
      saveTrajectory(trajectory_msg, "trajectory_"+ boost::lexical_cast<std::string>(trajectory_count++)+".csv");
      //saveTrajectory(trajectory_msg, "trajectory.csv");
    }
  }

  // Visualize the hand/wrist path in Rviz
  if (trajectory.points.size() > 1)
  {
    visuals_->goal_state_->deleteAllMarkers();
    ros::spinOnce();
    
    //const moveit::core::LinkModel* robot_link = planning_scene_monitor_->getRobotModel()->getLinkModel("jaco2_link_finger_2_tip");
    visuals_->goal_state_->publishTrajectoryLine(trajectory_msg, grasp_datas_[config_->right_arm_]->parent_link_,
                                                 config_->right_arm_, rvt::LIME_GREEN);
  }
  else
    ROS_WARN_STREAM_NAMED("execution_interface","Not visualizing path because trajectory only has "
                          << trajectory.points.size() << " points");

  // Visualize trajectory in Rviz
  bool wait_for_trajetory = false;
  visuals_->visual_tools_->publishTrajectoryPath(trajectory_msg, getCurrentState(), wait_for_trajetory);

  // Debug: check for errors in trajectory
  static const double MAX_TIME_STEP_SEC = 4.0;
  ros::Duration max_time_step(MAX_TIME_STEP_SEC);
  static const double WARN_TIME_STEP_SEC = 3.0;
  ros::Duration warn_time_step(WARN_TIME_STEP_SEC);
  ros::Duration diff;
  for (std::size_t i = 0; i < trajectory.points.size() - 1; ++i)
  {
    diff = (trajectory.points[i+1].time_from_start - trajectory.points[i].time_from_start);
    if (diff > max_time_step)
    {
      ROS_ERROR_STREAM_NAMED("execution_interface","Max time step between points exceeded, likely because of wrap around/IK bug. Point " << i);
      std::cout << "First time: " << trajectory.points[i].time_from_start.toSec() << std::endl;
      std::cout << "Next time: " << trajectory.points[i+1].time_from_start.toSec() << std::endl;
      std::cout << "Diff time: " << diff.toSec() << std::endl;
      std::cout << "-------------------------------------------------------" << std::endl;
      std::cout << std::endl;
      //return false;
    }
    else if (diff > warn_time_step)
    {
      ROS_WARN_STREAM_NAMED("execution_interface","Warn time step between points exceeded, likely because of wrap around/IK bug. Point " << i);
      std::cout << "First time: " << trajectory.points[i].time_from_start.toSec() << std::endl;
      std::cout << "Next time: " << trajectory.points[i+1].time_from_start.toSec() << std::endl;
      std::cout << "Diff time: " << diff.toSec() << std::endl;
      std::cout << "-------------------------------------------------------" << std::endl;
      std::cout << std::endl;
    }
  }

  // Create trajectory execution manager
  if( !trajectory_execution_manager_ )
  {
    trajectory_execution_manager_.reset(new trajectory_execution_manager::
                                        TrajectoryExecutionManager(planning_scene_monitor_->getRobotModel()));
    plan_execution_.reset(new plan_execution::PlanExecution(planning_scene_monitor_, trajectory_execution_manager_));
  }

  // Confirm trajectory before continuing
  if (!remote_control_->getFullAutonomous() &&
      // Only wait for non-finger trajectories
      trajectory.joint_names.size() > 3)
  {
    std::cout << std::endl;
    std::cout << std::endl;

    ROS_INFO_STREAM_NAMED("execution_interface","Waiting before executing trajectory");
    remote_control_->waitForNextFullStep();
    ROS_INFO_STREAM_NAMED("execution_interface","Executing trajectory....");
  }

  // Clear
  plan_execution_->getTrajectoryExecutionManager()->clear();

  if(plan_execution_->getTrajectoryExecutionManager()->push(trajectory_msg))
  {
    plan_execution_->getTrajectoryExecutionManager()->execute();

    bool wait_exection = true;
    if (wait_exection)
    {
      // wait for the trajectory to complete
      moveit_controller_manager::ExecutionStatus es = plan_execution_->getTrajectoryExecutionManager()->waitForExecution();
      if (es == moveit_controller_manager::ExecutionStatus::SUCCEEDED)
      {
        ROS_DEBUG_STREAM_NAMED("execution_interface","Trajectory execution succeeded");
      }
      else // Failed
      {
        if (es == moveit_controller_manager::ExecutionStatus::PREEMPTED)
          ROS_INFO_STREAM_NAMED("execution_interface","Trajectory execution preempted");
        else
          if (es == moveit_controller_manager::ExecutionStatus::TIMED_OUT)
            ROS_ERROR_STREAM_NAMED("execution_interface","Trajectory execution timed out");
          else
            ROS_ERROR_STREAM_NAMED("execution_interface","Trajectory execution control failed");

        // Disable autonomous mode because something went wrong
        remote_control_->setAutonomous(false);

        return false;
      }
    }
  }
  else
  {
    ROS_ERROR_STREAM_NAMED("execution_interface","Failed to execute trajectory");
    return false;
  }

  return true;
}

bool ExecutionInterface::checkExecutionManager()
{
  ROS_INFO_STREAM_NAMED("execution_interface","Checking that execution manager is loaded.");

  // Load MoveIt! managers
  if (!trajectory_execution_manager_)
  {
    trajectory_execution_manager_.reset(new trajectory_execution_manager::
                                        TrajectoryExecutionManager(planning_scene_monitor_->getRobotModel()));
  }
  if (!plan_execution_)
    plan_execution_.reset(new plan_execution::PlanExecution(planning_scene_monitor_, trajectory_execution_manager_));

  const robot_model::JointModelGroup* arm_jmg = config_->dual_arm_ ? config_->both_arms_ : config_->right_arm_;

  // Check active controllers are running
  if (!trajectory_execution_manager_->ensureActiveControllersForGroup(arm_jmg->getName()))
  {
    ROS_ERROR_STREAM_NAMED("execution_interface","Group " << arm_jmg->getName() << " does not have controllers loaded");
    return false;
  }

  // Get the controllers List
  std::vector<std::string> controller_list;
  trajectory_execution_manager_->getControllerManager()->getControllersList(controller_list);
  //std::copy(controller_list.begin(), controller_list.end(), std::ostream_iterator<std::string>(std::cout, "\n"));

  // Check active controllers are running
  if (!trajectory_execution_manager_->ensureActiveControllers(controller_list))
  {
    ROS_ERROR_STREAM_NAMED("execution_interface","Robot does not have the desired controllers active");
    return false;
  }

  // Check that correct controllers are running
  // bool has_error = true;

  // while (has_error && ros::ok())
  // {
  //   has_error = false;
  //   if (!checkTrajectoryController(zaber_list_controllers_client_, "zaber"))
  //   {
  //     has_error = true;
  //   }

  //   bool has_ee = true;
  //   if (!checkTrajectoryController(kinova_list_controllers_client_, "kinova", has_ee))
  //   {
  //     has_error = true;
  //   }
  //   ros::Duration(0.5).sleep();
  // }

  return true;
}

// DEPRECATED IN FAVOR OF MOVEIT CONTROLLER MANAGER VERSION
bool ExecutionInterface::checkTrajectoryController(ros::ServiceClient& service_client, const std::string& hardware_name, bool has_ee)
{
  // Try to communicate with controller manager
  controller_manager_msgs::ListControllers service;
  if (!service_client.call(service))
  {
    ROS_ERROR_STREAM_NAMED("execution_interface","Unable to check if controllers for " << hardware_name << " are loaded, failing. Using nh namespace " << nh_.getNamespace());
    std::cout << "service: " << service.response << std::endl;
    return false;
  }

  // Check if proper controller is running
  bool found_main_controller = false;
  bool found_ee_controller = false;
  for (std::size_t i = 0; i < service.response.controller.size(); ++i)
  {
    if (service.response.controller[i].name == "velocity_trajectory_controller")
    {
      found_main_controller = true;
      if (service.response.controller[i].state != "running")
      {
        ROS_WARN_STREAM_THROTTLE_NAMED(2, "execution_interface","Controller for " << hardware_name << " is in manual mode");
        return false;
      }
    }
    if (service.response.controller[i].name == "ee_velocity_trajectory_controller")
    {
      found_ee_controller = true;
      if (service.response.controller[i].state != "running")
      {
        ROS_WARN_STREAM_THROTTLE_NAMED(2, "execution_interface","Controller for " << hardware_name << " is in manual mode");
        return false;
      }
    }
  }

  if (has_ee && !found_ee_controller)
  {
    ROS_ERROR_STREAM_THROTTLE_NAMED(2, "execution_interface","No end effector controller found for " << hardware_name);
    return false;
  }
  if (!found_main_controller)
  {
    ROS_ERROR_STREAM_THROTTLE_NAMED(2, "execution_interface","No main controller found for " << hardware_name);
    return false;
  }

  return true;
}

bool ExecutionInterface::saveTrajectory(const moveit_msgs::RobotTrajectory &trajectory_msg, const std::string &file_name)
{
  const trajectory_msgs::JointTrajectory &joint_trajectory = trajectory_msg.joint_trajectory;

  // Error check
  if (!joint_trajectory.points.size() || !joint_trajectory.points[0].positions.size())
  {
    ROS_ERROR_STREAM_NAMED("execution_interface","No trajectory points available to save");
    return false;
  }
  bool has_accelerations = true;
  if (joint_trajectory.points[0].accelerations.size() == 0)
  {
    has_accelerations = false;
  }

  std::string file_path;
  getFilePath(file_path, file_name);
  std::ofstream output_file;
  output_file.open (file_path.c_str());

  // Output header -------------------------------------------------------
  output_file << "time_from_start,";
  for (std::size_t j = 0; j < joint_trajectory.joint_names.size(); ++j)
  {
    output_file << joint_trajectory.joint_names[j] << "_pos,"
                << joint_trajectory.joint_names[j] << "_vel,";
    if (has_accelerations)
      output_file << joint_trajectory.joint_names[j] << "_acc,";
  }
  output_file << std::endl;

  // Output data ------------------------------------------------------

  // Subtract starting time

  for (std::size_t i = 0; i < joint_trajectory.points.size(); ++i)
  {
    // Timestamp
    output_file.precision(20);
    output_file << joint_trajectory.points[i].time_from_start.toSec() << ",";
    output_file.precision(5);
    // Output entire trajectory to single line
    for (std::size_t j = 0; j < joint_trajectory.points[i].positions.size(); ++j)
    {
      // Output State
      output_file << joint_trajectory.points[i].positions[j] << ","
                  << joint_trajectory.points[i].velocities[j] << ",";
      if (has_accelerations)
        output_file << joint_trajectory.points[i].accelerations[j] << ",";

    }

    output_file << std::endl;
  }
  output_file.close();
  ROS_DEBUG_STREAM_NAMED("execution_interface","Saved trajectory to file " << file_name);
  return true;
}

bool ExecutionInterface::getFilePath(std::string &file_path, const std::string &file_name) const
{
  namespace fs = boost::filesystem;

  // Check that the directory exists, if not, create it
  fs::path path;
  path = fs::path(package_path_ + "/trajectories");

  boost::system::error_code returnedError;
  fs::create_directories( path, returnedError );

  // Error check
  if ( returnedError )
  {
    ROS_ERROR_STREAM_NAMED("execution_interface", "Unable to create directory " << path.string());
    return false;
  }

  // Directories successfully created, append the group name as the file name
  path = path / fs::path(file_name);
  file_path = path.string();

  ROS_DEBUG_STREAM_NAMED("execution_interface.file_path","Using full file path" << file_path);
  return true;
}

moveit::core::RobotStatePtr ExecutionInterface::getCurrentState()
{
  planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_); // Lock planning scene
  (*current_state_) = scene->getCurrentState();
  return current_state_;
}

} // end namespace
