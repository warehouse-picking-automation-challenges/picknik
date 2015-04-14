/*
  Author: Andy McEvoy (mcevoy.andy@gmail.com)
  Desc  : Test for SimplePointCloudFilter class
*/
#include <iostream>
#include <fstream>
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
#include <pcl/filters/passthrough.h>


#include <boost/shared_ptr.hpp>

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
  ros::Publisher bin_cloud_pub_;

  double bin_depth_, bin_width_, bin_height_;
  Eigen::Affine3d bin_pose_, camera_pose_;

  
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

    // publish aligned point cloud and bin point cloud
    aligned_cloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("aligned_cloud",1);
    bin_cloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("bin_cloud",1);

    // set initial camera transform
    delta_ = 0.010;
    camera_translation_ = Eigen::Vector3d(-1.459, 0.09, 1.0844);
    camera_rotation_ = Eigen::Vector3d(0.01, -0.009, 0.07);
    camera_pose_ = Eigen::Affine3d::Identity();
    camera_pose_ *= Eigen::AngleAxisd(camera_rotation_[0],Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(camera_rotation_[1],Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(camera_rotation_[2],Eigen::Vector3d::UnitZ());
    camera_pose_.translation() = camera_translation_;
    visual_tools_->publishAxis(camera_pose_);

    // display test scene
    visual_tools_->publishAxis(Eigen::Affine3d::Identity());
    displayHomeOfficeScene();

    // set up test cuboid for filtering (area above my desk)
    bin_pose_ = Eigen::Affine3d::Identity();
    bin_depth_ = 25.25 * 0.0254; // 1 inch padding
    bin_width_ = 35 * 0.0254; // 1 inch padding
    bin_height_ = 1.0;
    bin_pose_.translation() += Eigen::Vector3d(-26.25 / 2.0 * 0.0254, 0 , 30.5 * 0.0254 + 0.5 );
    visual_tools_->publishCuboid(bin_pose_, bin_depth_, bin_width_, bin_height_, 
                                 rviz_visual_tools::TRANSLUCENT);

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
    std::cout << "e\tWrite transform values to file" << std::endl;

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
    // turn message into point cloud
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromROSMsg(*msg, cloud);

    // publish aligned point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (!pcl_ros::transformPointCloud("world", cloud, *aligned_cloud, tf_listener_))
    {
      ROS_ERROR_STREAM_NAMED("PC_filter.process","Error converting to desired frame");
    }  
    aligned_cloud_pub_.publish(aligned_cloud);

    /***** get point cloud that lies in region_of_interest *****/
    // (from Dave's baxter_experimental/flex_perception.cpp)
   
    // create new output cloud and place to save filtered results
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr bin_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Transform point cloud to bin CS
    pcl::transformPointCloud(*aligned_cloud, *bin_cloud, bin_pose_.inverse());

    // Filter based on bounding box
    pcl::PassThrough<pcl::PointXYZRGB> pass_x;
    pass_x.setInputCloud(bin_cloud);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(-bin_depth_ / 2.0, bin_depth_ / 2.0);
    pass_x.filter(*bin_cloud);

    pcl::PassThrough<pcl::PointXYZRGB> pass_y;
    pass_y.setInputCloud(bin_cloud);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(-bin_width_ / 2.0, bin_width_ / 2.0);
    pass_y.filter(*bin_cloud);

    pcl::PassThrough<pcl::PointXYZRGB> pass_z;
    pass_z.setInputCloud(bin_cloud);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(-bin_height_ / 2.0, bin_height_ / 2.0);
    pass_z.filter(*bin_cloud);

    // Transform point cloud back to world CS
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr bin_cloud_final(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud(*bin_cloud, *bin_cloud_final, bin_pose_);

    // publish pointcloud of bin
    if (bin_cloud->points.size() == 0)
    {
      ROS_WARN_STREAM_NAMED("PC_filter.process","0 points left after filtering");
    }
    
    bin_cloud_pub_.publish(bin_cloud_final);
  }

  void keyboardCallback(const keyboard::Key::ConstPtr& msg)
  {
    int entry = msg->code;
    double fine = 0.001;
    double coarse = 0.01;

    switch(entry)
    {
      case 101:
        std::cout << "Writing transformation to file..." << std::endl;
        writeTransformToFile();
        break;
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
  }

  void writeTransformToFile()
  {
    std::ofstream file ("camera_transform.txt", std::ios::app);

    if (!file.is_open())
      ROS_ERROR_STREAM_NAMED("PC_filter.write","output file could not be opened");
    else
    {
      ROS_INFO_STREAM_NAMED("PC_filter.write","Camera translation = \n " << camera_translation_);
      ROS_INFO_STREAM_NAMED("PC_filter.write","Camera rotations = \n" << camera_rotation_);
      for (std::size_t i = 0; i < 3; i++)
        file << camera_translation_[i] << "\t ";
      for (std::size_t i = 0; i < 3; i++)
        file << camera_rotation_[i] << "\t";
      file << std::endl;
    }
    file.close();

  }

  void displayHomeOfficeScene()
  {
    // This is my home office where I'm testing the point cloud stuff...
    ROS_DEBUG_STREAM_NAMED("PC_filter.officeScene","Loading home office scene...");
    visual_tools_->enableBatchPublishing(true);
    
    // variables for publishing objects
    Eigen::Affine3d object_pose = Eigen::Affine3d::Identity();
    double depth, width, height, radius;

    // convert units (yes... they're in inches :)
    double in2m = 0.0254;

    // Display Backwall
    depth = 0.1;
    width = (1.25 + 1.5 + 36) * in2m + 1;
    height = 76 * in2m;
    object_pose.translation() = Eigen::Vector3d(depth/2.0, 0, height/2.0);
    visual_tools_->publishCuboid(object_pose, depth, width, height, rviz_visual_tools::TRANSLUCENT_DARK);

    // Display side walls
    depth = (28.0 + 3.0/8.0) * in2m;
    width = 0.5;
    object_pose.translation() = Eigen::Vector3d(-depth/2.0, width/2.0 + (36.0/2.0 + 1.25) * in2m, height/2.0);
    visual_tools_->publishCuboid(object_pose, depth, width, height, rviz_visual_tools::TRANSLUCENT_DARK);
    
    object_pose.translation() = Eigen::Vector3d(-depth/2.0, -width/2.0 - (36.0/2.0 + 1.5) * in2m, height/2.0);
    visual_tools_->publishCuboid(object_pose, depth, width, height, rviz_visual_tools::TRANSLUCENT_DARK);

    // Display desk
    depth = 26.25 * in2m;
    width = 36 * in2m;
    height = 30.5 * in2m;
    object_pose.translation() = Eigen::Vector3d(-depth/2.0, 0, height/2.0);
    visual_tools_->publishCuboid(object_pose, depth, width, height, rviz_visual_tools::TRANSLUCENT_DARK);

    // Display Tripod
    depth = 54 * in2m;
    width = 0;
    height = 41 * in2m;
    radius = 2 * in2m;
    object_pose.translation() = Eigen::Vector3d(-depth, 0, height/2.0);
    visual_tools_->publishCylinder(object_pose, rviz_visual_tools::TRANSLUCENT_DARK, height, radius);

    // publish xtion camera
    std::string mesh_path = ros::package::getPath("jacob_description");
    object_pose *= Eigen::AngleAxisd(1.5708, Eigen::Vector3d::UnitX()) 
      * Eigen::AngleAxisd(1.5708, Eigen::Vector3d::UnitY());
    object_pose.translation() = Eigen::Vector3d(-depth + 0.018, 0.0235, height + 0.018);
    visual_tools_->publishMesh(object_pose, "file://" + mesh_path + "/meshes/xtion.stl", 
                               rviz_visual_tools::TRANSLUCENT_DARK);

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
