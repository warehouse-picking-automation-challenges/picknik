/*
  Authors : Andy McEvoy (mcevoy.andy@gmail.com)
  Desc    : Get camera alignment of multiple cameras using ICP
*/

#include <ros/ros.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
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

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_left_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_right_;

public:

  PointCloudAligner()
    : nh_("~")
  {

    has_left_pc_ = false;
    has_right_pc_ = false;
    processing_ = false;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_left(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_right(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_left_ = cloud_left;
    cloud_right_ = cloud_right;

    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("base"));
    visual_tools_->deleteAllMarkers();

    // subscribe to point clouds
    left_pc_sub_ = nh_.subscribe("/left_preprocessor/roi_cloud", 1, 
                            &PointCloudAligner::leftPointCloudCallback, this);
    right_pc_sub_ = nh_.subscribe("/right_preprocessor/roi_cloud", 1, 
                            &PointCloudAligner::rightPointCloudCallback, this);

    // publish aligned point cloud and bin point cloud
    pc_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ> >("roi_cloud",1);

    ROS_DEBUG_STREAM_NAMED("PC_aligner","computing point cloud alignment...");

    while (ros::ok())
    {

    }

  }

  void leftPointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    if (has_left_pc_ || processing_)
    {
      return;
    }
    else
    {
      ROS_DEBUG_STREAM_NAMED("PC_aligner","got left cloud...");
      pcl::fromROSMsg(*msg, *cloud_left_);
      has_left_pc_ = true;
    }
  
    if (has_left_pc_ && has_right_pc_)
    {
      ROS_DEBUG_STREAM_NAMED("PC_aligner","have both...");
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
      ROS_DEBUG_STREAM_NAMED("PC_aligner","got right cloud...");
      pcl::fromROSMsg(*msg, *cloud_right_);
      has_right_pc_ = true;
    }
   
    if (has_left_pc_ && has_right_pc_)
    {
      ROS_DEBUG_STREAM_NAMED("PC_aligner","have both...");
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
      std::vector<int> indices_left;
      std::vector<int> indices_right;
      pcl::PointCloud<pcl::PointXYZ>::Ptr icp_left(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr icp_right(new pcl::PointCloud<pcl::PointXYZ>);

      pcl::removeNaNFromPointCloud(*cloud_left_, *icp_left, indices_left);
      pcl::removeNaNFromPointCloud(*cloud_right_, *icp_right, indices_right);

      ROS_DEBUG_STREAM_NAMED("PC_aligner","sizes = " << icp_left->size() << " : " << icp_right->size());

      pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
      icp.setInputCloud(icp_left); // TODO deprecated
      icp.setInputTarget(icp_right);
      pcl::PointCloud<pcl::PointXYZ> Final;

      ROS_DEBUG_STREAM_NAMED("PC_aligner","calling icp");
      
      icp.align(Final);
      ROS_INFO_STREAM_NAMED("PC_aligner","converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() );
      ROS_INFO_STREAM_NAMED("PC_ALIGNER","transform:\n" << icp.getFinalTransformation() );      

      pc_pub_.publish(Final);

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
