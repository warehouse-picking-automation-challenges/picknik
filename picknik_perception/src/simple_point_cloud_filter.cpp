/*
  Author: Andy McEvoy (mcevoy.andy@gmail.com)
  Desc  : Implementation of SimplePointCloudFilter, see *.h for details
*/

#include <picknik_perception/simple_point_cloud_filter.h>

// PCL
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

namespace picknik_perception
{

SimplePointCloudFilter::SimplePointCloudFilter(rviz_visual_tools::RvizVisualToolsPtr& visual_tools)
  : visual_tools_(visual_tools)
  , nh_("~")
  , has_roi_(false)
{
  processing_ = false;
  get_bbox_ = false;
  outlier_removal_ = false;

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
  visual_tools_->publishAxisLabeled(roi_pose_, "bin");
  visual_tools_->publishWireframeCuboid(roi_pose_, roi_depth_, roi_width_, roi_height_, rviz_visual_tools::CYAN);
}

void SimplePointCloudFilter::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  if (processing_)
  {
    ROS_ERROR_STREAM_NAMED("point_cloud_filter.pcCallback","skipped point cloud because currently busy");

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

    // slowish
    if (outlier_removal_)
    {
      pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> rad;
      rad.setInputCloud(roi_cloud_);
      rad.setRadiusSearch(0.03);
      rad.setMinNeighborsInRadius(200);
      rad.filter(*roi_cloud_);

      pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
      sor.setInputCloud(roi_cloud_);
      sor.setMeanK(50);
      sor.setStddevMulThresh(1.0);
      sor.filter(*roi_cloud_);
    }

    if (roi_cloud_->points.size() == 0)
    {
      ROS_WARN_STREAM_THROTTLE_NAMED(2, "point_cloud_filter.process","0 points left after filtering");
      return;
    }
  }

  // publish point clouds for rviz
  roi_cloud_pub_.publish(roi_cloud_);
  ROS_DEBUG_STREAM_THROTTLE_NAMED(2, "point_cloud_filter","Publishing filtered point cloud");

  // optionally get the bounding box of the point cloud
  if (get_bbox_)
  {
    bounding_box_.getBodyAlignedBoundingBox(roi_cloud_, bbox_pose_, bbox_depth_, bbox_width_, bbox_height_);

    // Visualize
    visual_tools_->publishWireframeCuboid(bbox_pose_, bbox_depth_, bbox_width_, bbox_height_,
                                          rviz_visual_tools::MAGENTA);

    get_bbox_ = false;
  }

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

void SimplePointCloudFilter::resetRegionOfInterst()
{
  has_roi_ = false;
}

void SimplePointCloudFilter::enableBoundingBox(bool enable)
{
  get_bbox_ = enable;
}

void SimplePointCloudFilter::getObjectPose(geometry_msgs::Pose &pose)
{
  pose = visual_tools_->convertPose(bbox_pose_);
}

} // namespace
