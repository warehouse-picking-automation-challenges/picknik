/*
  Authors : Andy McEvoy (mcevoy.andy@gmail.com)
  Desc    : Get camera alignment of multiple cameras using ICP
*/

#include <ros/ros.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

namespace picknik_perception
{

class PointCloudAligner
{
private:
  ros::NodeHandle nh_;

  ros::Subscriber left_pc_sub_;
  ros::Subscriber right_pc_sub_;

  ros::Publisher pc_pub_;

  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

  bool has_left_pc_;
  bool has_right_pc_;
  bool processing_;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_left_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_right_;

public:

  PointCloudAligner()
    : nh_("~")
  {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_left(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_right(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud_left_ = cloud_left;
    cloud_right_ = cloud_right;

    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("base"));
    visual_tools_->deleteAllMarkers();

    // subscribe to point clouds
    left_pc_sub_ = nh_.subscribe("/xtion_left/depth_registered/points", 1, 
                            &PointCloudAligner::leftPointCloudCallback, this);
    right_pc_sub_ = nh_.subscribe("/xtion_right/depth_registered/points", 1, 
                            &PointCloudAligner::rightPointCloudCallback, this);

    // publish aligned point cloud and bin point cloud
    pc_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("roi_cloud",1);

    ROS_DEBUG_STREAM_NAMED("PC_aligner","computing point cloud alignment...");

    while (ros::ok())
    {

    }

  }

  void leftPointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    if (has_left_pc_)
    {
      ROS_DEBUG_STREAM_NAMED("PC_aligner","point cloud skipped. waiting on right...");
      return;
    }
    else
    {
      ROS_DEBUG_STREAM_NAMED("PC_aligner","getting left point cloud...");
      pcl::fromROSMsg(*msg, *cloud_left_);
    }

    if (processing_)
    {
      ROS_DEBUG_STREAM_NAMED("PC_aligner","point cloud skipped. processing...");
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
    if (has_right_pc_)
    {
      ROS_DEBUG_STREAM_NAMED("PC_aligner","point cloud skipped. waiting on left...");
      return;
    }
    else
    {
      ROS_DEBUG_STREAM_NAMED("PC_aligner","getting right point cloud...");
      pcl::fromROSMsg(*msg, *cloud_right_);
    }

    if (processing_)
    {
      ROS_DEBUG_STREAM_NAMED("PC_aligner","point cloud skipped. processing...");
    }
    
    if (has_left_pc_ && has_right_pc_)
    {
      processing_ = true;
      processPointClouds();
      processing_ = false;
    }
  }

  void processPointClouds()
  {

    if (cloud_left_->size() == 0 || cloud_right_->size() == 0)
    {
      ROS_DEBUG_STREAM_NAMED("PC_aligner","one of the point clouds has zero size. skipping...");
      has_left_pc_ = false;
      has_right_pc_ = false;
      return;
    }
    else
    {
      ROS_DEBUG_STREAM_NAMED("PC_aligner","processing point cloud alignment...");
      
      // ICP
      pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
      icp.setInputCloud(cloud_left_);
      icp.setInputTarget(cloud_right_);
      pcl::PointCloud<pcl::PointXYZRGB> Final;
      icp.align(Final);
      ROS_INFO_STREAM_NAMED("PC_aligner","converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() );
      ROS_INFO_STREAM_NAMED("PC_ALIGNER","transform:\n" << icp.getFinalTransformation() );      

      has_left_pc_ = false;
      has_right_pc_ = false;
    }

  }

}; // end class PointCloudAligner

} // end namespace picknik_perception

int main (int argc, char** argv)
{
  ros::init(argc, argv, "pc_aligner");

  ros::AsyncSpinner spinner(2);
  spinner.start();

  picknik_perception::PointCloudAligner aligner;

}
