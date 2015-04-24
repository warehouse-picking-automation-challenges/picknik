/*
  Author : Andy McEvoy
  Desc   : Test setup of multi-kinect 2 system
*/

#include <ros/ros.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <Eigen/Core>


namespace picknik_perception
{

class MultiKinectTester
{
private:
  ros::NodeHandle nh_;
  tf::TransformListener listener_;
  tf::StampedTransform transform_;
  Eigen::Affine3d pose_;

  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

public:
  MultiKinectTester()
    : nh_("~")
  {
    // Load 
    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("base", "/picknik_main/markers"));
    visual_tools_->deleteAllMarkers();

    // show world coordinate system
    visual_tools_->publishAxis(Eigen::Affine3d::Identity());
    

    while (ros::ok())
    {
      
      
      // show camera coordinate systems
      try 
      {
        visual_tools_->resetMarkerCounts();
        visual_tools_->publishAxis(Eigen::Affine3d::Identity());

        listener_.lookupTransform("/world", "/kinect2_right_rgb_optical_frame", ros::Time(0), transform_);
        tf::transformTFToEigen(transform_, pose_);
        visual_tools_->publishAxis(pose_, 0.1, 0.01, "kinect_right");

        listener_.lookupTransform("/world", "/kinect2_left_rgb_optical_frame", ros::Time(0), transform_);
        tf::transformTFToEigen(transform_, pose_);
        visual_tools_->publishAxis(pose_, 0.1, 0.01, "kinect_left");

      }
      catch (tf::TransformException ex)
      {
        ROS_WARN_STREAM_NAMED("multi_kinect.test","lookupTransform error: " << ex.what());
        ros::Duration(1.0).sleep();
      }

    }
  }
  

}; // class MultiKinectTester

} // namesapce picknik_perception

int main(int argc, char** argv)
{
  ros::init(argc, argv, "multi_kinect_test");
  ROS_INFO_STREAM_NAMED("multiKinect.test","Starting multi kinect test");

  ros::AsyncSpinner spinner(2);
  spinner.start();

  picknik_perception::MultiKinectTester tester;

  
}
