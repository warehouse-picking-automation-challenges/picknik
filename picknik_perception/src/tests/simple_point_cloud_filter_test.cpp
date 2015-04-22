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

public:

  SimpleFilterTest()
    : nh_("~")
  {
    // Load 
    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("base"));
    visual_tools_->deleteAllMarkers();

    // Visualize origin of world
    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    visual_tools_->publishAxis(pose);

    pc_filter_ptr_ = new SimplePointCloudFilter(true);

    ROS_DEBUG_STREAM_NAMED("PC_filter.test","starting main filter test");

    ros::Rate rate(40.0);
    int bbox_rate = 10;
    int count = 0;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    double depth, width, height;
    Eigen::Affine3d bbox_pose;

    while ( ros::ok() )
    {
      if (count % bbox_rate == 0)
      {
        visual_tools_->publishAxis(pose);
        pc_filter_ptr_->getBoundingBox(cloud, bbox_pose, depth, width, height);
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
