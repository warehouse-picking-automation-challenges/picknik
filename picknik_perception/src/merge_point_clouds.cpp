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
 * Authors : Andy McEvoy
 * Desc    : Merge point clouds from two cameras into a single ros message.
 *           This assumes cameras are calibrated and aligned.
*/

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <rviz_visual_tools/rviz_visual_tools.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/icp.h>

namespace picknik_perception
{

class PointCloudMerger
{
public:

  PointCloudMerger()
    : nh_("~")
    , has_started_(false)
  {
    id_ = 0;

    has_left_pc_ = false;
    has_right_pc_ = false;
    processing_ = false;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_left(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_right(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud_left_ = cloud_left;
    cloud_right_ = cloud_right;

    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("base","/picknik_main/markers"));
    visual_tools_->deleteAllMarkers();

    // subscribe to point clouds
    left_pc_sub_ = nh_.subscribe("/xtion_left/depth_registered/points", 1, 
                            &PointCloudMerger::leftPointCloudCallback, this);
    right_pc_sub_ = nh_.subscribe("/xtion_right/depth_registered/points", 1, 
                            &PointCloudMerger::rightPointCloudCallback, this);

    // publish aligned point cloud and bin point cloud
    pc_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ> >("points",1);

    // wait to allow buffers to load
    ros::Duration(1.0).sleep();
    ros::spinOnce();

  }

  void leftPointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    if (has_left_pc_ || processing_)
    {
      return;
    }
    else
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::fromROSMsg(*msg, *cloud_left_);

      // get frame to transfer right point cloud into
      transform_link_ = cloud_left_->header.frame_id;

      has_left_pc_ = true;
    }
  
    if (has_left_pc_ && has_right_pc_)
    {
      processing_ = true;
      processPointClouds();
      processing_ = false;
    }
  }

  void rightPointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    if (has_right_pc_ || processing_)
    {
      return;
    }
    else
    {
      const std::string BASE_LINK = transform_link_;;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::fromROSMsg(*msg, *cloud);

      tf_listener_.waitForTransform(BASE_LINK, cloud->header.frame_id, msg->header.stamp, ros::Duration(2.0));
      
      if (!pcl_ros::transformPointCloud(BASE_LINK, *cloud, *cloud_right_, tf_listener_))
      {
        ROS_ERROR_STREAM_NAMED("merge_point_clouds","Error converting left cloud to desired frame");
      }

      has_right_pc_ = true;
    }
   
    if (has_left_pc_ && has_right_pc_)
    {
      processing_ = true;
      processPointClouds();
      processing_ = false;
    }
  }

  void setLeftCameraTopic(std::string topic)
  {
    ROS_DEBUG_STREAM_NAMED("merge_point_clouds","changing left camera topic: " << topic);
    left_pc_sub_.shutdown();
    left_pc_sub_ = nh_.subscribe(topic, 1, &PointCloudMerger::leftPointCloudCallback, this);
  }

  void setRightCameraTopic(std::string topic)
  {
    ROS_DEBUG_STREAM_NAMED("merge_point_clouds","changing right camera topic: " << topic);
    right_pc_sub_.shutdown();
    right_pc_sub_ = nh_.subscribe(topic, 1, &PointCloudMerger::rightPointCloudCallback, this);
  }

  bool processPointClouds()
  {
    // check if both or either of the clouds has zero size
    if (cloud_left_->size() == 0 && cloud_right_->size() == 0)
    {
      ROS_WARN_STREAM_NAMED("merge_point_clouds","Both clouds have zero size, skipping...");
      has_left_pc_ = false;
      has_right_pc_ = false;
      return false;
    }

    if (cloud_left_->size() == 0)
    {
      ROS_WARN_STREAM_NAMED("merge_point_clouds","Left point cloud has zero size...");

      publishCloud(cloud_right_);

      has_left_pc_ = false;
      has_right_pc_ = false;

      return true;      
    }
    
    if (cloud_right_->size() == 0)
    {
      ROS_WARN_STREAM_NAMED("merge_point_clouds","right point cloud has zero size...");

      publishCloud(cloud_left_);

      has_left_pc_ = false;
      has_right_pc_ = false;

      return true;      
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZRGB>); 
  
    *merged_cloud = *cloud_left_ + *cloud_right_;
      
    publishCloud(merged_cloud);

    // user feedback
    if (!has_started_)
    {
      ROS_INFO_STREAM_NAMED("merge_point_clouds","Point cloud publishing has started!");
      has_started_ = true;
    }
    
    has_left_pc_ = false;
    has_right_pc_ = false;
    
    merged_cloud->points.clear();

    return true;
  }

  void publishCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
  {
    cloud->header.seq = id_++;
    cloud->header.frame_id = transform_link_;
    pc_pub_.publish(cloud);

  }

private:
  ros::NodeHandle nh_;

  ros::Subscriber left_pc_sub_;
  ros::Subscriber right_pc_sub_;

  ros::Publisher pc_pub_;

  tf::TransformListener tf_listener_;

  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

  bool has_left_pc_;
  bool has_right_pc_;
  bool processing_;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_left_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_right_;

  std::size_t id_;

  std::string transform_link_;

}; // end class PointCloudMerger

} // end namespace picknik_perception

int main (int argc, char** argv)
{
  ros::init(argc, argv, "merge_point_clouds");

  ros::AsyncSpinner spinner(2);
  spinner.start();

  picknik_perception::PointCloudMerger aligner;

  ROS_INFO_STREAM_NAMED("point_cloud_merger","starting to merge point clouds...");
  
  ros::waitForShutdown();
  
  ROS_INFO_STREAM_NAMED("point_cloud_merger","stopping merged point cloud publisher...");


}
