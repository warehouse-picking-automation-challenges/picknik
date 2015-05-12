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
 * Desc    : Simple perception based on point cloud data
 */

#include <picknik_perception/simple_point_cloud_filter.h>

#include <shape_msgs/Mesh.h>
#include <shape_msgs/MeshTriangle.h>
#include <geometry_msgs/Point.h>

// PCL
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/surface/gp3.h>

// Parameter loading
#include <rviz_visual_tools/ros_param_utilities.h>

namespace picknik_perception
{

SimplePointCloudFilter::SimplePointCloudFilter(rviz_visual_tools::RvizVisualToolsPtr& visual_tools)
  : visual_tools_(visual_tools)
  , nh_("~")
  , has_roi_(false)
{
  processing_ = false;

  // set regoin of interest
  roi_depth_ = 1.0;
  roi_width_ = 1.0;
  roi_height_ = 1.0;
  roi_pose_ = Eigen::Affine3d::Identity();

  // initialize cloud pointers
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr roi_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  roi_cloud_ = roi_cloud;

  // publish bin point cloud
  roi_cloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("roi_cloud",1);

  // Load parameters
  const std::string parent_name = "simple_point_cloud_filter"; // for namespacing logging messages
  rviz_visual_tools::getDoubleParameter(parent_name, nh_, "radius_of_outlier_removal", radius_of_outlier_removal_);


  ROS_DEBUG_STREAM_NAMED("point_cloud_filter","Simple point cloud filter ready.");
}

bool SimplePointCloudFilter::publishRegionOfInterest()
{
  if (!has_roi_)
  {
    ROS_ERROR_STREAM_NAMED("point_cloud_filter","No region of interest specified");
    return false;
  }
  // show region of interest
  //visual_tools_->publishAxisLabeled(roi_pose_, "bin");
  visual_tools_->publishWireframeCuboid(roi_pose_, roi_depth_, roi_width_, roi_height_, rviz_visual_tools::CYAN);
}

void SimplePointCloudFilter::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  if (processing_)
  {
    ROS_INFO_STREAM_THROTTLE_NAMED(2.0, "point_cloud_filter","Skipped point cloud because currently busy");
    return;
  }

  processing_ = true;
  processPointCloud(msg);
  processing_ = false;
}

void SimplePointCloudFilter::processPointCloud(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*msg, *cloud);

  // Wait for first TF transform to arrive
  static const std::string BASE_LINK = "/world";
  //ROS_DEBUG_STREAM_NAMED("perception","Waiting for transform from " << BASE_LINK << " to " << cloud->header.frame_id);
  tf_listener_.waitForTransform(BASE_LINK, cloud->header.frame_id, msg->header.stamp, ros::Duration(2.0));

  if (!pcl_ros::transformPointCloud(BASE_LINK, *cloud, *roi_cloud_, tf_listener_))
  {
    ROS_ERROR_STREAM_NAMED("point_cloud_filter.process","Error converting to desired frame");
  }

  if (!has_roi_)
  {
    ROS_DEBUG_STREAM_THROTTLE_NAMED(2, "point_cloud_filter","No region of interest specified yet, showing all points");
  }
  else
  {

    // Filter based on bin location
    pcl::PassThrough<pcl::PointXYZRGB> pass_x;
    pass_x.setInputCloud(roi_cloud_);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(roi_pose_.translation()[0]-roi_depth_ / 2.0, roi_pose_.translation()[0] + roi_depth_ / 2.0);
    pass_x.filter(*roi_cloud_);

    pcl::PassThrough<pcl::PointXYZRGB> pass_y;
    pass_y.setInputCloud(roi_cloud_);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(roi_pose_.translation()[1] - roi_width_ / 2.0, roi_pose_.translation()[1] + roi_width_ / 2.0);
    pass_y.filter(*roi_cloud_);

    pcl::PassThrough<pcl::PointXYZRGB> pass_z;
    pass_z.setInputCloud(roi_cloud_);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(roi_pose_.translation()[2] - roi_height_ / 2.0, roi_pose_.translation()[2] + roi_height_ / 2.0);
    pass_z.filter(*roi_cloud_);
  }

  // publish point clouds for rviz
  roi_cloud_pub_.publish(roi_cloud_);
  ROS_DEBUG_STREAM_THROTTLE_NAMED(2, "point_cloud_filter","Publishing filtered point cloud");
}

bool SimplePointCloudFilter::detectObjects(bool remove_outliers)
{
  // wait until other loop is done processing, then block that loop
  while (processing_)
  {
    ros::Duration(0.1).sleep();
    ROS_INFO_STREAM_THROTTLE_NAMED(1,"point_cloud_filter","Waiting for main point cloud callback to finish processing");
  }
  processing_ = true;

  if (remove_outliers)
  {
    ROS_INFO_STREAM_NAMED("point_cloud_filter","Performing outlier removal");

    pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> rad;
    rad.setInputCloud(roi_cloud_);
    rad.setRadiusSearch(radius_of_outlier_removal_);
    rad.setMinNeighborsInRadius(200);
    rad.filter(*roi_cloud_);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(roi_cloud_);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*roi_cloud_);
  }

  // publish point clouds for rviz
  roi_cloud_pub_.publish(roi_cloud_);
  ros::Duration(5).sleep();

  if (roi_cloud_->points.size() == 0)
  {
    ROS_WARN_STREAM_THROTTLE_NAMED(2, "point_cloud_filter.process","0 points in region of interest");
    processing_ = false;
    return false;
  }

  // get the bounding box of the point cloud
  bounding_box_.getBodyAlignedBoundingBox(roi_cloud_, Eigen::Affine3d::Identity(), bbox_pose_, bbox_depth_, bbox_width_, bbox_height_);

  ROS_DEBUG_STREAM_NAMED("simple_point_cloud_filter.detectObjects","roi_cloud_->header.frame_id = " << roi_cloud_->header.frame_id);

  // save bounding_box_ point cloud for debugging
  ROS_DEBUG_STREAM_NAMED("simple_point_cloud_filter.detectObjects","saving bbox_.cloud with " << bounding_box_.cloud_->size() << " points");
  bounding_box_.cloud_->width = 1;
  bounding_box_.cloud_->height = bounding_box_.cloud_->size();
  saveRegionOfInterest(bounding_box_.cloud_);
  
  // Allow main loop to work again
  processing_ = false;
  
  return true;
}

void SimplePointCloudFilter::setRegionOfInterest(Eigen::Affine3d pose, double depth, double width, double height)
{
  roi_pose_ = pose;
  roi_depth_ = depth;
  roi_width_ = width;
  roi_height_ = height;
  has_roi_ = true;

  // Visualize
  publishRegionOfInterest();
}

void SimplePointCloudFilter::setRegionOfInterest(Eigen::Affine3d bottom_right_front_corner,
                                                 Eigen::Affine3d top_left_back_corner, double reduction_padding_x, double reduction_padding_y, double reduction_padding_z)
{
  Eigen::Vector3d delta = top_left_back_corner.translation() - bottom_right_front_corner.translation();
  roi_depth_ = std::abs(delta[0]) - reduction_padding_x * 2.0;
  roi_width_ = std::abs(delta[1]) - reduction_padding_y * 2.0;
  roi_height_ = std::abs(delta[2])- reduction_padding_z * 2.0;
  has_roi_ = true;

  roi_pose_ = bottom_right_front_corner;
  roi_pose_.translation() += Eigen::Vector3d(roi_depth_ / 2.0 + reduction_padding_x,
                                             roi_width_ / 2.0 + reduction_padding_y,
                                             roi_height_ / 2.0 + reduction_padding_z);

  // Visualize
  publishRegionOfInterest();
}

void SimplePointCloudFilter::saveRegionOfInterest(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{

  // Save files in picknik_perception/data/roi_pcds
  std::string package_path = ros::package::getPath("picknik_perception");

  // from: http://www.cplusplus.com/forum/beginner/70854/
  namespace fs = boost::filesystem;
  fs::path save_path(package_path + "/data/roi_pcds");
  fs::directory_iterator end_iter;

  // get unique identifier for file (count number of pcd files in folder)
  std::size_t file_count = 0;

  for (fs::directory_iterator iter(save_path); iter != end_iter; ++iter)
  {
    if (iter->path().extension() == ".pcd")
    {
      file_count++;
    }
  }
  
  ROS_DEBUG_STREAM_NAMED("point_cloud_filter.save","number of files in directory: " << file_count);

  std::stringstream file_name;
  file_name << save_path.string() << "/roi_pc_" <<(int)file_count << ".pcd";
  std::string full_path = file_name.str();

  ROS_DEBUG_STREAM_NAMED("point_cloud_filter.save","saving file to: " << full_path);

  // remove nan's
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud, *new_cloud, indices);

  // save to file
  pcl::io::savePCDFileASCII(full_path, *new_cloud);
}

void SimplePointCloudFilter::resetRegionOfInterst()
{
  has_roi_ = false;
}


void SimplePointCloudFilter::getObjectPose(geometry_msgs::Pose &pose)
{
  pose = visual_tools_->convertPose(bbox_pose_);
}

Eigen::Affine3d& SimplePointCloudFilter::getObjectPose()
{
  return bbox_pose_;
}

} // namespace
