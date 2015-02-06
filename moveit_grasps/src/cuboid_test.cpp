/* Author: Andy McEvoy
   Desc:   Tests the cuboid grasp generator
*/

#include <cstdlib>
#include <cmath>

// ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

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
    visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("base","/rviz_visual_tools"));
    visual_tools_->setLifetime(120.0);
    visual_tools_->setMuted(false);
    visual_tools_->loadMarkerPub();
    depth_ = 0;
    width_ = 0;
    height_ = 0;
    
  }

  void generateRandomCuboid(geometry_msgs::Pose& cuboid_pose, double& l, double& w, double& h)
  {
    // Size
    l = fRand(CUBOID_MIN_SIZE, CUBOID_MAX_SIZE);
    w = fRand(CUBOID_MIN_SIZE, CUBOID_MAX_SIZE);
    h = fRand(CUBOID_MIN_SIZE, CUBOID_MAX_SIZE);
    ROS_DEBUG_STREAM_NAMED("main","Size = " << l << ", "<< w << ", " << h);


    // Position
    // Values chosen to be within shelf boundary for Amazon pick & place challenge
    // TODO: get right values
    cuboid_pose.position.x = fRand(CUBOID_WORKSPACE_MIN_X , CUBOID_WORKSPACE_MAX_X);
    cuboid_pose.position.y = fRand(CUBOID_WORKSPACE_MIN_Y , CUBOID_WORKSPACE_MAX_Y);
    cuboid_pose.position.z = fRand(CUBOID_WORKSPACE_MIN_Z , CUBOID_WORKSPACE_MAX_Z);
    ROS_DEBUG_STREAM_NAMED("main","Position = " << cuboid_pose.position.x << ", " << 
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
    ROS_DEBUG_STREAM_NAMED("main","Quaternion = " << cuboid_pose.orientation.x << ", " <<
			   cuboid_pose.orientation.y << ", " << cuboid_pose.orientation.z);
  }

  double fRand(double fMin, double fMax) 
  {
    return fMin + ( (double)rand() / RAND_MAX ) * (fMax - fMin);
  }

  void runTest()
  {
    // generate cuboid with random size, position & orientation
    generateRandomCuboid(cuboid_pose_,depth_,width_,height_);
    visual_tools_->publishRectangle(cuboid_pose_,depth_,width_,height_);
    
    // 

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


  for (int i = 0; i < 10; i++)
    {
      tester.runTest();
    }

}
