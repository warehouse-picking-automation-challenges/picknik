/* Author: Andy McEvoy
   Desc:   Tests the cuboid grasp generator
*/

#include <cstdlib>
#include <cmath>
#include <string>

// ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_grasps/grasps.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

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
  // TODO: verify max object size Open Hand can grasp
  static const double OPEN_HAND_MAX_GRASP_SIZE= 0.10;

class CuboidGraspGeneratorTest
{

private:
  ros::NodeHandle nh_;
  moveit_grasps::GraspData grasp_data_;
  const moveit::core::JointModelGroup* ee_jmg_;
  std::vector<moveit_msgs::Grasp> possible_grasps_;
  moveit_grasps::GraspsPtr grasps_;
  Eigen::Affine3d object_global_transform_;

  enum grasp_axis_t {X_AXIS, Y_AXIS, Z_AXIS};

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
    visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("base_link","/rviz_visual_tools"));
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

    // Orientation 
    visual_tools_->generateRandomPose(cuboid_pose);

    ROS_DEBUG_STREAM_NAMED("random","Position = " << cuboid_pose.position.x << ", " << 
    			   cuboid_pose.position.y << ", " << cuboid_pose.position.z);
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

  void generateCuboidGrasp() {

    ROS_INFO_STREAM_NAMED("grasp", "generating random cuboid");
    generateRandomCuboid(cuboid_pose_,depth_,width_,height_);

    visual_tools_->publishRectangle(cuboid_pose_,depth_,width_,height_);
    visual_tools_->publishAxis(cuboid_pose_);

    ROS_INFO_STREAM_NAMED("grasp","generating grasps for cuboid");
    
    possible_grasps_.clear();

    generateCuboidGrasps( visual_tools_->convertPose(cuboid_pose_), depth_, width_, height_, grasp_data_, possible_grasps_);

    visual_tools_->publishAnimatedGrasps(possible_grasps_, ee_jmg_);
  }


  bool generateCuboidGrasps(const Eigen::Affine3d& cuboid_pose, 
			    float depth, float width,float height,
			    const moveit_grasps::GraspData& grasp_data, 
			    std::vector<moveit_msgs::Grasp>& possible_grasps)
  {
    // generate grasps over axes that aren't too wide to grip with Open Hand
    if (depth <= OPEN_HAND_MAX_GRASP_SIZE ) // depth = size along x-axis
      {
	ROS_INFO_STREAM_NAMED("cube","generating grasps around x-axis of cuboid");
	generateCuboidAxisGrasps(cuboid_pose, depth, width, height, X_AXIS, grasp_data, possible_grasps);
      }

    if (width <= OPEN_HAND_MAX_GRASP_SIZE ) // width = size along y-axis
      {
	ROS_INFO_STREAM_NAMED("cube","generating grasps around y-axis of cuboid");
	generateCuboidAxisGrasps(cuboid_pose, depth, width, height, Y_AXIS, grasp_data, possible_grasps);
      }

    if (height <= OPEN_HAND_MAX_GRASP_SIZE ) // height = size along z-axis
      {
	ROS_INFO_STREAM_NAMED("cube","generating grasps around z-axis of cuboid");
	generateCuboidAxisGrasps(cuboid_pose, depth, width, height, Z_AXIS, grasp_data, possible_grasps);
      }    
    
  }

  bool generateCuboidAxisGrasps(const Eigen::Affine3d& cuboid_pose,
				float depth, float width, float height,
				grasp_axis_t axis,
				const moveit_grasps::GraspData& grasp_data, 
				std::vector<moveit_msgs::Grasp>& possible_grasps)
  {
    // create transform from object to world frame (/base_link)
    object_global_transform_ = cuboid_pose;

    // grasp parameters

    ROS_DEBUG_STREAM_NAMED("grasps","generating reusable motions and msgs");

    moveit_msgs::GripperTranslation pre_grasp_approach;
    pre_grasp_approach.direction.header.stamp = ros::Time::now();
    pre_grasp_approach.desired_distance = grasp_data.finger_to_palm_depth_ +0.1;
    pre_grasp_approach.min_distance = grasp_data.finger_to_palm_depth_;

    moveit_msgs::GripperTranslation post_grasp_retreat;
    post_grasp_retreat.direction.header.stamp = ros::Time::now();
    post_grasp_retreat.desired_distance = grasp_data.finger_to_palm_depth_ +0.1;
    post_grasp_retreat.min_distance = grasp_data.finger_to_palm_depth_;
    
    geometry_msgs::PoseStamped grasp_pose_msg;
    grasp_pose_msg.header.stamp = ros::Time::now();
    grasp_pose_msg.header.frame_id = grasp_data.base_link_;

    // grasp generator loop
    double radius = grasp_data.grasp_depth_; 

    moveit_msgs::Grasp new_grasp;
    static int grasp_id = 0;
    double grasp_score;

    Eigen::Affine3d grasp_pose;
    grasp_pose = cuboid_pose;
    
    double dx = cuboid_pose_.position.x;
    double dy = cuboid_pose_.position.y;
    double dz = cuboid_pose_.position.z;

    Eigen::Vector3d grasp_translation;

    switch(axis)
      {
      case X_AXIS:
	// will rotate around x-axis testing grasps
	grasp_translation = grasp_pose * Eigen::Vector3d(0, -width/2 - radius, 0) - Eigen::Vector3d(dx,dy,dz);
	grasp_pose.translation() += grasp_translation;

	break;

      case Y_AXIS:
	// will rotate around y-axis testing grasps
	grasp_translation = grasp_pose * Eigen::Vector3d(0, 0, -height/2 - radius) - Eigen::Vector3d(dx,dy,dz);
	grasp_pose.translation() += grasp_translation;
	break;

      case Z_AXIS:
	// will rotate around z-axis testing grasps
	grasp_translation = grasp_pose * Eigen::Vector3d(-depth/2 - radius, 0, 0) - Eigen::Vector3d(dx,dy,dz);
	grasp_pose.translation() += grasp_translation;
	break;

      default:
	ROS_WARN_STREAM_NAMED("grasps","grasp axis may not be defined properly");
	break;
      }

    // TODO: score grasp
    new_grasp.id = "Grasp" + boost::lexical_cast<std::string>(grasp_id);
    grasp_id++;

    // pre-grasp and grasp postures
    new_grasp.pre_grasp_posture = grasp_data.pre_grasp_posture_;
    new_grasp.grasp_posture = grasp_data.grasp_posture_;

    // TODO: figure out Dave's 90 deg hack
    // TODO: Why have roll_gripper? why only about unit X?
    //Eigen::Affine3d roll_gripper;
    //roll_gripper = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
    //grasp_pose = grasp_pose * roll_gripper;

    //grasp_pose = grasp_pose * grasp_data.grasp_pose_to_eef_pose_;

    //tf::poseEigenToMsg(object_global_transform_ * grasp_pose, grasp_pose_msg.pose);
    tf::poseEigenToMsg(grasp_pose, grasp_pose_msg.pose);

    new_grasp.grasp_pose = grasp_pose_msg;

    // TODO: contact force

    // Approach and retreat
    // aligned with pose
    pre_grasp_approach.direction.header.frame_id = grasp_data.parent_link_name_;
    pre_grasp_approach.direction.vector.x = 0; 
    pre_grasp_approach.direction.vector.y = 0; 
    pre_grasp_approach.direction.vector.z = 1;
    new_grasp.pre_grasp_approach = pre_grasp_approach;
   
    post_grasp_retreat.direction.header.frame_id = grasp_data.parent_link_name_;
    post_grasp_retreat.direction.vector.x = 0; 
    post_grasp_retreat.direction.vector.y = 0; 
    post_grasp_retreat.direction.vector.z = -1;
    new_grasp.post_grasp_retreat = post_grasp_retreat;
   
    possible_grasps.push_back(new_grasp);

    // publish grasp arrow
    grasps_->publishGraspArrow(new_grasp.grasp_pose.pose, grasp_data_, rviz_visual_tools::YELLOW );
    visual_tools_->publishAxis(new_grasp.grasp_pose.pose);
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
