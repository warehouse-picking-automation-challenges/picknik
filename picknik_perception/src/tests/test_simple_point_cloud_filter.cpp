/*
  Author: Andy McEvoy (mcevoy.andy@gmail.com)
  Desc  : Test for SimplePointCloudFilter class
*/
#include <iostream>
#include <string>

#include <ros/ros.h>
#include <ros/package.h>
#include <Eigen/Core>

#include <picknik_perception/simple_point_cloud_filter.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <keyboard/Key.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

namespace picknik_perception
{

class SimplePointCloudFilterTest
{
private:
  ros::NodeHandle nh_;
  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
  Eigen::Vector3d camera_translation_, camera_rotation_;
  int mode_;
  double delta_;
  bool processing_;
  tf::TransformListener tf_listener_;

  ros::Publisher aligned_cloud_pub_;

public:

  SimplePointCloudFilterTest()
  {
    ROS_DEBUG_STREAM_NAMED("PC_filter.constructor","setting up rviz...");
    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("base", "/rviz_visual_tools"));
    visual_tools_->deleteAllMarkers();

    // listen to keyboard topic
    ros::Subscriber keyboard_sub = 
      nh_.subscribe("/keyboard/keydown", 100, &SimplePointCloudFilterTest::keyboardCallback, this);

    // listen to point cloud topic
    processing_ = false;
    ros::Subscriber pc_sub = 
      nh_.subscribe("/camera/depth_registered/points", 1, &SimplePointCloudFilterTest::pointCloudCallback, this);

    // publish aligned point cloud
    aligned_cloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("aligned_cloud",1);

    // set initial camera transform
    camera_translation_ = Eigen::Vector3d(-1.533, 0.0, 1.076);
    camera_rotation_ = Eigen::Vector3d(0.0, 0.0, 0.0);

    // display test scene
    displayHomeOfficeScene();

    // Print menu for manual alignment of point cloud
    std::cout << "Manual alignment of camera to world CS:" << std::endl;
    std::cout << "=======================================" << std::endl;
    std::cout << "\nChoose Mode:" << std::endl;
    std::cout << "x\tAdjust X translation" << std::endl;
    std::cout << "y\tAdjust Y translation" << std::endl;
    std::cout << "z\tAdjust Z translation" << std::endl;    
    std::cout << "r\tAdjust Roll (rotation about X)" << std::endl;
    std::cout << "p\tAdjust Pitch (rotation about Y)" << std::endl;
    std::cout << "w\tAdjust Yaw (rotation about Z)" << std::endl;

    ros::Rate rate(40.0);
    while (nh_.ok())
    {
      publishCameraTransform();
      rate.sleep();
    }

  }

  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    if (processing_)
    {
      ROS_ERROR_STREAM_NAMED("PC_filter.pcCallback","skipped point cloud because currently busy");
      return;
    }

    processing_ = true;
    processPointCloud(msg);

    processing_ = false;

  }

  void processPointCloud(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    ROS_DEBUG_STREAM_NAMED("PC_filter.process","processing cloud");

    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromROSMsg(*msg, cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (!pcl_ros::transformPointCloud("world", cloud, *aligned_cloud, tf_listener_))
    {
      ROS_ERROR_STREAM_NAMED("PC_filter.process","Error converting to desired frame");
    }
    
    
    aligned_cloud_pub_.publish(aligned_cloud);
  }


  void keyboardCallback(const keyboard::Key::ConstPtr& msg)
  {
    int entry = msg->code;
    double fine = 0.001;
    double coarse = 0.01;
    delta_ = coarse;

    switch(entry)
    {
      case 99: // c (coarse delta)
        std::cout << "Delta = coarse (0.010)" << std::endl;
        delta_ = coarse;
        break;
      case 102: // f (fine delta)
        std::cout << "Delta = fine (0.001)" << std::endl;
        delta_ = fine;
        break;
      case 120: // x 
        std::cout << "Press +/- to adjust X translation" << std::endl;
        mode_ = 1;
        break;
      case 121: // y
        std::cout << "Press +/- to adjust Y translation" << std::endl;
        mode_ = 2;
        break;
      case 122: // z
        std::cout << "Press +/- to adjust Z translation" << std::endl;
        mode_ = 3;
        break;          
      case 114: // r (roll)
        std::cout << "Press +/- to adjust roll angle" << std::endl;
        mode_ = 4;
        break;
      case 112: // p (pitch)
        std::cout << "Press +/- to adjust pitch angle" << std::endl;
        mode_ = 5;
        break;          
      case 119: // w (yaw)
        std::cout << "Press +/- to adjust yaw angle" << std::endl;
        mode_ = 6;
        break;
      case 270: // + (on numberpad)
        updateCameraTransform(mode_, delta_);
        break;
      case 269: // - (on numberpad)
        updateCameraTransform(mode_, -delta_);
        break;
      default:
        // don't do anything
        break;
      }
    
  }

  void updateCameraTransform(int mode, double delta)
  {
    ROS_DEBUG_STREAM_NAMED("PC_filter.update","mode = " << mode << ", delta = " << delta);

    switch(mode)
    {
      case 1:
        camera_translation_ += Eigen::Vector3d(delta, 0, 0);
        break;
      case 2:
        camera_translation_ += Eigen::Vector3d(0, delta, 0);
        break;
      case 3:
        camera_translation_ += Eigen::Vector3d(0, 0, delta);
        break;
      case 4:
        camera_rotation_ += Eigen::Vector3d(delta, 0, 0);
        break;
      case 5:
        camera_rotation_ += Eigen::Vector3d(0, delta, 0);
        break;
      case 6:
        camera_rotation_ += Eigen::Vector3d(0, 0, delta);
        break;
      default:
        // don't do anything
        break;
    }
    
    ROS_DEBUG_STREAM_NAMED("PC_filter.update","translation = \n" << camera_translation_);
    ROS_DEBUG_STREAM_NAMED("PC_filter.update","rotation = \n" << camera_rotation_);

  }

  void displayHomeOfficeScene()
  {
    // This is my home office where I'm testing the point cloud stuff...
    ROS_DEBUG_STREAM_NAMED("PC_filter.officeScene","Loading home office scene...");
    visual_tools_->enableBatchPublishing(true);
    
    // variables for publishing objects
    Eigen::Affine3d object_pose = Eigen::Affine3d::Identity();
    double depth, width, height, radius;
    visual_tools_->publishAxis(object_pose);

    // convert units (yes... they're in inches :)
    double in2m = 0.0254;

    // Display Backwall
    depth = 0.1;
    width = (1.25 + 1.5 + 36) * in2m + 1;
    height = 76 * in2m;
    object_pose.translation() = Eigen::Vector3d(depth/2.0, 0, height/2.0);
    visual_tools_->publishCuboid(object_pose, depth, width, height, rviz_visual_tools::TRANSLUCENT);

    // Display side walls
    depth = (28.0 + 3.0/8.0) * in2m;
    width = 0.5;
    object_pose.translation() = Eigen::Vector3d(-depth/2.0, width/2.0 + (36.0/2.0 + 1.25) * in2m, height/2.0);
    visual_tools_->publishCuboid(object_pose, depth, width, height, rviz_visual_tools::TRANSLUCENT);
    
    object_pose.translation() = Eigen::Vector3d(-depth/2.0, -width/2.0 - (36.0/2.0 + 1.5) * in2m, height/2.0);
    visual_tools_->publishCuboid(object_pose, depth, width, height, rviz_visual_tools::TRANSLUCENT);

    // Display desk
    depth = 26.25 * in2m;
    width = 36 * in2m;
    height = 30.5 * in2m;
    object_pose.translation() = Eigen::Vector3d(-depth/2.0, 0, height/2.0);
    visual_tools_->publishCuboid(object_pose, depth, width, height, rviz_visual_tools::TRANSLUCENT);

    // Display Tripod
    depth = 54 * in2m;
    width = 0;
    height = 41 * in2m;
    radius = 2 * in2m;
    object_pose.translation() = Eigen::Vector3d(-depth, 0, height/2.0);
    visual_tools_->publishCylinder(object_pose, rviz_visual_tools::TRANSLUCENT, height, radius);

    // publish xtion camera
    std::string mesh_path = ros::package::getPath("jacob_description");
    object_pose *= Eigen::AngleAxisd(1.5708, Eigen::Vector3d::UnitX()) 
      * Eigen::AngleAxisd(1.5708, Eigen::Vector3d::UnitY());
    object_pose.translation() = Eigen::Vector3d(-depth + 0.018, 0.0235, height + 0.018);
    visual_tools_->publishMesh(object_pose, "file://" + mesh_path + "/meshes/xtion.stl", 
                               rviz_visual_tools::TRANSLUCENT);

    // publish everything
    visual_tools_->triggerBatchPublishAndDisable();
    ros::Duration(0.001).sleep();
    ROS_DEBUG_STREAM_NAMED("PC_filter.officeScene","Done loading home office scene");


  }

  void publishCameraTransform()
  {
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;

    // set camera pose translation
    transform.setOrigin( tf::Vector3( camera_translation_[0],
                                      camera_translation_[1],
                                      camera_translation_[2]) );

    // set camera pose rotation
    q.setRPY(camera_rotation_[0], camera_rotation_[1], camera_rotation_[2]);
    transform.setRotation(q);

    // publish
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world" , "/camera_link"));
  }

};

} // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_point_cloud_filter_test");
  ROS_INFO_STREAM_NAMED("PC_filter.test","Simple point cloud filter test");

  ros::AsyncSpinner spinner(2);
  spinner.start();

  picknik_perception::SimplePointCloudFilterTest tester;

  return 0;
}
