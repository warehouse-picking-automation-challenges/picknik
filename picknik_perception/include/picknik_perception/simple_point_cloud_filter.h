/*
  Author: Andy McEvoy (mcevoy.andy@gmail.com)
  Desc  : A simple filter for a static depth camera
*/

//#include <iostream>
//#include <fstream>
#include <string>

#include <ros/ros.h>
//#include <ros/package.h>
//#include <rviz_visual_tools/rviz_visual_tools.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

//#include <keyboard/Key.h>
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

  SimplePointCloudFilter();
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
  void setRegionOfInterest(Eigen::Affine3d pose, double depth, double width, double height);

  // point clouds
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr aligned_cloud_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr roi_cloud_;

  // region of interest pose
  Eigen::Affine3d roi_pose_;

  // process point cloud to get bounding box of item
  bool get_bbox_;

  // Bounding box pose and dimensions
  Eigen::Affine3d bbox_pose_;
  double bbox_depth_, bbox_width_, bbox_height_;

private:
  bool verbose_;
  bool processing_;
  ros::NodeHandle nh_;

  tf::TransformListener tf_listener_;

  // Region of interest dimensions
  double roi_depth_, roi_width_, roi_height_;

}; // class


} // end namespace
  
