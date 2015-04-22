/*
  Author: Andy McEvoy (mcevoy.andy@gmail.com)
  Desc  : Test for SimplePointCloudFilter class
*/
#include <picknik_perception/simple_point_cloud_filter.h>

#include <rviz_visual_tools/rviz_visual_tools.h>

namespace picknik_perception
{

class SimpleFilterTest
{
private:
  ros::NodeHandle nh_;

  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

  SimplePointCloudFilter* pc_filter_ptr_;

  ros::Subscriber pc_sub_;
  ros::Publisher aligned_cloud_pub_;
  ros::Publisher roi_cloud_pub_;

public:

  SimpleFilterTest()
    : nh_("~")
  {
    // Load 
    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("base"));
    visual_tools_->deleteAllMarkers();

    pc_filter_ptr_ = new SimplePointCloudFilter();

    // listen to point cloud topic
    // TODO: camera topic should be set in launch file
    pc_sub_ = nh_.subscribe("/camera/depth_registered/points", 1, 
                            &picknik_perception::SimplePointCloudFilter::processPointCloud, pc_filter_ptr_);

    // publish aligned point cloud and bin point cloud
    aligned_cloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("aligned_cloud",1);
    roi_cloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("roi_cloud",1);

    ROS_DEBUG_STREAM_NAMED("PC_filter.test","starting main filter test");

    ros::Rate rate(40.0);
    int bbox_rate = 10;
    int count = 0;

    double depth, width, height;
    Eigen::Affine3d bbox_pose;

    while ( ros::ok() )
    {
      if (count % bbox_rate == 0)
      {
        visual_tools_->deleteAllMarkers();
        
        // show world CS
        visual_tools_->publishAxis(Eigen::Affine3d::Identity());

        // show region of interest and bounding box
        visual_tools_->publishAxis(pc_filter_ptr_->roi_pose_);
        pc_filter_ptr_->getBoundingBox(pc_filter_ptr_->roi_cloud_, bbox_pose, depth, width, height);
        visual_tools_->publishCuboid(bbox_pose, depth, width, height, rviz_visual_tools::TRANSLUCENT);

        aligned_cloud_pub_.publish(pc_filter_ptr_->aligned_cloud_);
        roi_cloud_pub_.publish(pc_filter_ptr_->roi_cloud_);

        count = 0;
      }

      pc_filter_ptr_->publishCameraTransform();
      rate.sleep();

      count++;
    }

  }

}; // class

} // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_point_cloud_filter_test");
  ROS_INFO_STREAM_NAMED("PC_filter.test","Starting simple point cloud filter test");

  ros::AsyncSpinner spinner(2);
  spinner.start();

  picknik_perception::SimpleFilterTest tester;
}
