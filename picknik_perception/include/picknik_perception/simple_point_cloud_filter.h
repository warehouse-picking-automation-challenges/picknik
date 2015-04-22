/*
  Author: Andy McEvoy (mcevoy.andy@gmail.com)
  Desc  : A simple filter for a static depth camera
*/

#include <iostream>
#include <fstream>
#include <string>

#include <ros/ros.h>
#include <ros/package.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <keyboard/Key.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include <boost/shared_ptr.hpp>




namespace picknik_perception
{

class SimplePointCloudFilter
{
public:

  SimplePointCloudFilter(bool verbose);
  ~SimplePointCloudFilter();

  /* 
   * \brief 
   */
  bool getBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, Eigen::Affine3d& cuboid_pose,
                      double& depth, double& width, double& height);

  /* 
   * \brief 
   */
  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

  /* 
   * \brief 
   */
  void processPointCloud(const sensor_msgs::PointCloud2ConstPtr& msg);

  /* 
   * \brief 
   */
  void publishCameraTransform();


private:
  bool verbose_;
  bool processing_;
  ros::NodeHandle nh_;
  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
  Eigen::Vector3d camera_translation_, camera_rotation_;
  tf::TransformListener tf_listener_;

  ros::Publisher aligned_cloud_pub_;
  ros::Publisher bin_cloud_pub_;

  double bin_depth_, bin_width_, bin_height_;

  double bbox_depth_, bbox_width_, bbox_height_;
  bool get_bbox_;
  std::size_t bbox_frames_;
  Eigen::Matrix3d bbox_rotation_;
  Eigen::Vector3d bbox_translation_;
  Eigen::Affine3d bbox_pose_;

  Eigen::Affine3d bin_pose_, camera_pose_;

}; // class


} // end namespace
  
