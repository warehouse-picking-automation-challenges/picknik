/*
  Author: Andy McEvoy (mcevoy.andy@gmail.com)
  Desc  : A simple filter for a static depth camera
*/

// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>

// PCL
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

// picknik_percpetion
#include <picknik_perception/bounding_box.h>

// Rviz
#include <rviz_visual_tools/rviz_visual_tools.h>

namespace picknik_perception
{

class SimplePointCloudFilter
{
public:

  SimplePointCloudFilter(rviz_visual_tools::RvizVisualToolsPtr& visual_tools);

  /*
   * \brief
   */
  bool getBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, Eigen::Affine3d& cuboid_pose,
                      double& depth, double& width, double& height);

  /**
   * \brief Visualize region of interest
   * \return true on success
   */
  bool publishRegionOfInterest();

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
  void setRegionOfInterest(Eigen::Affine3d bottom_right_front_corner, Eigen::Affine3d top_left_back_corner, double reduction_padding_x, double reduction_padding_y, double reduction_padding_z);

  /**
   * \brief After percieving a particular bin, switch back to showing entire shelf
   * \return true on success
   */
  void resetRegionOfInterst();

  /**
   * \brief Enable bounding box calculatons
   */
  void enableBoundingBox(bool enable = true);

  /**
   * \brief Get object pose
   */
  void getObjectPose(geometry_msgs::Pose &pose);

  bool processing_;

  // point clouds
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr roi_cloud_;

private:

  // process point cloud to get bounding box of item
  bool get_bbox_;

  // Bounding box pose and dimensions
  Eigen::Affine3d bbox_pose_;
  double bbox_depth_, bbox_width_, bbox_height_;

  bool outlier_removal_;

  bool verbose_;

  ros::NodeHandle nh_;

  tf::TransformListener tf_listener_;

  // Region of interest
  double roi_depth_, roi_width_, roi_height_;
  Eigen::Affine3d roi_pose_;
  bool has_roi_;

  // Class for publishing stuff to rviz
  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

  // Publish bin point cloud
  ros::Publisher roi_cloud_pub_;

  // Calculate bounding box
  BoundingBox bounding_box_;

}; // class

// Create boost pointers for this class
typedef boost::shared_ptr<SimplePointCloudFilter> SimplePointCloudFilterPtr;
typedef boost::shared_ptr<const SimplePointCloudFilter> SimplePointCloudFilterConstPtr;

} // end namespace
