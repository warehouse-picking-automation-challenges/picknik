/*********************************************************************
 * Software License Agreement ("Modified BSD License")
 *
 * Copyright (c) 2014, University of Colorado, Boulder
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * * Neither the name of the Univ of CO, Boulder nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
/**
 * Authors : Andy Mcevoy
 * Desc  : A simple filter for a static depth camera
*/

#ifndef PICKNIK_PERCEPTION_SIMPLE_POINT_CLOUD_FILTER_
#define PICKNIK_PERCEPTION_SIMPLE_POINT_CLOUD_FILTER_

#include <sstream>

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>

// PCL
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>

// Rviz
#include <rviz_visual_tools/rviz_visual_tools.h>

// bounding_box
#include <bounding_box/bounding_box.h>

// boost
#include <boost/filesystem.hpp>

namespace picknik_perception
{

class SimplePointCloudFilter
{
public:

  SimplePointCloudFilter(rviz_visual_tools::RvizVisualToolsPtr& visual_tools);

  /**
   * \brief Visualize region of interest
   * \return true on success
   */
  bool publishRegionOfInterest();

  /**
   * \brief save the region of interest as a pcd file.
   */
  static void saveRegionOfInterest(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

  /*
   * \brief
   */
  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

  /*
   * \brief
   */
  void processPointCloud(const sensor_msgs::PointCloud2ConstPtr& msg);

  /**
   * \brief Processing of filtered point cloud
   * \return true on success
   */
  bool detectObjects(bool remove_outliers);

  /*
   * \brief Set the region of interest for the point cloud as a cuboid
   * \param pose - the pose of the cuboid
   * \param depth - the size of the cuboid along the x-axis of the pose
   * \param width - the size of the cuboid along the y-axis of the pose
   * \param height - the size of the cuboid along the z-axis of the pose
   * \param bottom_right_front_corner - pose describing the bottom front right corner of a cuboid
   * \param top_left_back_corner - pose describing the top left back corner of a cuboid
   * \param reduction_padding_x - padding in the x direction of the poses
   * \param reduction_padding_y - padding in the y direction of the poses
   * \param reduction_padding_z - padding in the z direction of the poses
   *
   * NOTE: pose for the region of interes is taken from bottom_right_front_corner
   */
  void setRegionOfInterest(Eigen::Affine3d pose, double depth, double width, double height);
  void setRegionOfInterest(Eigen::Affine3d bottom_right_front_corner, Eigen::Affine3d top_left_back_corner, 
                           double reduction_padding_x, double reduction_padding_y, double reduction_padding_z);

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
  Eigen::Affine3d& getObjectPose();

  bool processing_;

  // point clouds
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr roi_cloud_;

private:

  // Bounding box pose and dimensions
  Eigen::Affine3d bbox_pose_;
  double bbox_depth_, bbox_width_, bbox_height_;



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

  bounding_box::BoundingBox bounding_box_;

  double radius_of_outlier_removal_;

}; // class

// Create boost pointers for this class
typedef boost::shared_ptr<SimplePointCloudFilter> SimplePointCloudFilterPtr;
typedef boost::shared_ptr<const SimplePointCloudFilter> SimplePointCloudFilterConstPtr;

} // end namespace

#endif
