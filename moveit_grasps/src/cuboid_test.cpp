/* Author: Andy McEvoy
   Desc:   Tests the cuboid grasp generator
*/

#include <cstdlib>
#include <cmath>
#include <string>

// ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_grasps/grasps.h>

namespace baxter_pick_place
{

  // Size and location for randomly generated cuboids
  static const double CUBOID_MIN_SIZE = 0.02;
  static const double CUBOID_MAX_SIZE = 0.15;
  static const double CUBOID_WORKSPACE_MIN_X = 0.01; 
  static const double CUBOID_WORKSPACE_MAX_X = 0.5;
  static const double CUBOID_WORKSPACE_MIN_Y = -0.5;
  static const double CUBOID_WORKSPACE_MAX_Y = 0.5;
  static const double CUBOID_WORKSPACE_MIN_Z = 0.0;
  static const double CUBOID_WORKSPACE_MAX_Z = 1.0;

class CuboidGraspGeneratorTest
{

private:
  ros::NodeHandle nh_;
  moveit_grasps::GraspData grasp_data_;
  const moveit::core::JointModelGroup* ee_jmg_;
  std::vector<moveit_msgs::Grasp> possible_grasps_;
  moveit_grasps::GraspsPtr grasps_;

  // arm description
  std::string arm_;
  std::string ee_group_name_;
  std::string planning_group_name_;

public:
  // cuboid dimensions
  double depth_;
  double width_;
  double height_;
  geometry_msgs::Pose cuboid_pose_;

  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  // Constructor
  CuboidGraspGeneratorTest() : nh_("~")
  {
    // get arm parameters
    nh_.param("arm", arm_, std::string("right"));
    nh_.param("ee_group_name", ee_group_name_, std::string(arm_ + "_hand"));
    planning_group_name_ = arm_ + "_arm";

    ROS_INFO_STREAM_NAMED("init", "Arm: " << arm_);
    ROS_INFO_STREAM_NAMED("init", "End Effector: " << ee_group_name_);
    ROS_INFO_STREAM_NAMED("init", "Planning Group: " << planning_group_name_);

    // set up rviz
    visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("base","/rviz_visual_tools"));
    visual_tools_->setLifetime(120.0);
    visual_tools_->setMuted(false);
    visual_tools_->loadMarkerPub();

    // load grasp data 
    if (!grasp_data_.loadRobotGraspData(nh_, ee_group_name_, visual_tools_->getRobotModel()))
      {
    	ROS_ERROR_STREAM_NAMED("init", "Failed to load grasp data");
    	ros::shutdown();
      }
    ee_jmg_ = visual_tools_->getRobotModel()->getJointModelGroup(ee_group_name_);

    // load grasp generator
    grasps_.reset( new moveit_grasps::Grasps(visual_tools_) );

    // initialize cuboid size
    depth_ = CUBOID_MIN_SIZE;
    width_ = CUBOID_MIN_SIZE;
    height_ = CUBOID_MIN_SIZE;
    
  }

  void generateCuboidGrasp() {

    ROS_INFO_STREAM_NAMED("grasp", "generating random cuboid");
    generateRandomCuboid(cuboid_pose_,depth_,width_,height_);
    visual_tools_->publishRectangle(cuboid_pose_,depth_,width_,height_);

    ROS_INFO_STREAM_NAMED("grasp","generating grasps for cuboid");
    
    possible_grasps_.clear();

    // TODO: create generateCuboidGrasps in grasps.cpp

    // TODO: display possible grasps

  }

  void animateGripperOpenClose(int number_of_trials) {
    geometry_msgs::Pose pose;
    visual_tools_->generateEmptyPose(pose);

    // TODO: visualization doesn't work... need to investigate
    for (int i = 0; i < number_of_trials; i++) 
      {
	// open position
	ROS_DEBUG_STREAM_NAMED("animate", "animating OPEN gripper");
	grasp_data_.setRobotStatePreGrasp( visual_tools_->getSharedRobotState() );
	visual_tools_->publishEEMarkers(pose, ee_jmg_, rviz_visual_tools::ORANGE, "test_eef");
	ros::Duration(1.0).sleep();
	
	// close position
	ROS_DEBUG_STREAM_NAMED("animate", "animating CLOSE gripper");
	grasp_data_.setRobotStateGrasp( visual_tools_->getSharedRobotState() );
	visual_tools_->publishEEMarkers(pose, ee_jmg_, rviz_visual_tools::GREEN, "test_eef");
	ros::Duration(1.0).sleep();
      }

  }

  void generateRandomCuboid(geometry_msgs::Pose& cuboid_pose, double& l, double& w, double& h)
  {
    // Size
    l = fRand(CUBOID_MIN_SIZE, CUBOID_MAX_SIZE);
    w = fRand(CUBOID_MIN_SIZE, CUBOID_MAX_SIZE);
    h = fRand(CUBOID_MIN_SIZE, CUBOID_MAX_SIZE);
    ROS_DEBUG_STREAM_NAMED("random","Size = " << l << ", "<< w << ", " << h);


    // Position
    // Values chosen to be within shelf boundary for Amazon pick & place challenge
    // TODO: get right values
    cuboid_pose.position.x = fRand(CUBOID_WORKSPACE_MIN_X , CUBOID_WORKSPACE_MAX_X);
    cuboid_pose.position.y = fRand(CUBOID_WORKSPACE_MIN_Y , CUBOID_WORKSPACE_MAX_Y);
    cuboid_pose.position.z = fRand(CUBOID_WORKSPACE_MIN_Z , CUBOID_WORKSPACE_MAX_Z);
    ROS_DEBUG_STREAM_NAMED("random","Position = " << cuboid_pose.position.x << ", " << 
    			   cuboid_pose.position.y << ", " << cuboid_pose.position.z);

    // Orientation 
    // Compute random angle and unit vector
    double x = fRand(-1.0, 1.0);
    double y = fRand(-1.0, 1.0);
    double z = fRand(-1.0, 1.0);
    double norm = sqrt(x * x + y * y + z * z);

    cuboid_pose.orientation.w = M_PI * fRand(0.0, 1.57);
    cuboid_pose.orientation.x = x / norm;
    cuboid_pose.orientation.y = y / norm;
    cuboid_pose.orientation.z = z / norm;
    ROS_DEBUG_STREAM_NAMED("random","Quaternion = " << cuboid_pose.orientation.x << ", " <<
			   cuboid_pose.orientation.y << ", " << cuboid_pose.orientation.z);
  }

  double fRand(double fMin, double fMax) 
  {
    return fMin + ( (double)rand() / RAND_MAX ) * (fMax - fMin);
  }

  void runTest (int number_of_trials) {
    int completed_trials = 0;
    while(ros::ok())
      {
	ROS_INFO_STREAM_NAMED("test","Starting test " << completed_trials + 1 << " of " << number_of_trials);

	generateCuboidGrasp();

	completed_trials++;
	if (completed_trials == number_of_trials)
	  break;
      }
  }

}; // class

} // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cuboid_grasp_generator_test");

  ROS_INFO_STREAM_NAMED("main","Cuboid Grasp Tests");
  
  baxter_pick_place::CuboidGraspGeneratorTest tester;

  // Seed random
  srand(ros::Time::now().toSec());

  tester.animateGripperOpenClose(1);

  tester.runTest(1);
    

}
