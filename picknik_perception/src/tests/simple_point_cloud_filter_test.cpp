/*
  Author: Andy McEvoy (mcevoy.andy@gmail.com)
  Desc  : Test for SimplePointCloudFilter class
*/
#include <picknik_perception/simple_point_cloud_filter.h>
#include <picknik_perception/manual_tf_alignment.h>

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
    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("base", "/picknik_main/markers"));
    visual_tools_->deleteAllMarkers();

    pc_filter_ptr_ = new SimplePointCloudFilter();

    // listen to point cloud topic
    // TODO: camera topic should be set in launch file
    pc_sub_ = nh_.subscribe("/left_preprocessor/roi_cloud", 1, 
                            &picknik_perception::SimplePointCloudFilter::pointCloudCallback, pc_filter_ptr_);

    // publish aligned point cloud and bin point cloud
    aligned_cloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("aligned_cloud",1);
    roi_cloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("roi_cloud",1);

    // define region of interest
    Eigen::Affine3d roi_pose = Eigen::Affine3d::Identity();
    double roi_depth = 0.3;
    double roi_width = 0.25;
    double roi_height = 0.2;
    roi_pose.translation() += Eigen::Vector3d( 0.7 + roi_depth / 2.0, 
                                              0.42 - roi_width / 2.0, 
                                               0.805 + roi_height / 2.0);

    double padding = 0.02;
    pc_filter_ptr_->setRegionOfInterest(roi_pose, roi_depth - padding, roi_width - padding, roi_height - padding);
    visual_tools_->publishWireframeCuboid(roi_pose, roi_depth, roi_width, roi_height, rviz_visual_tools::CYAN);

    //pc_filter_ptr_->setRegionOfInterest(bottom_left, top_right);

    ros::Rate rate(40.0);
    int bbox_rate = 30;
    int count = 1; // don't want a bounding box on first frame.

    pc_filter_ptr_->outlier_removal_ = true;

    ROS_DEBUG_STREAM_NAMED("PC_filter.test","starting main filter test");
    std::size_t id = 0;

    while ( ros::ok() )
    {
      // get bounding box at specified interval
      if (count % bbox_rate == 0)
      {
        ROS_DEBUG_STREAM_NAMED("PC_filter.test","getting bbox...");
        visual_tools_->deleteAllMarkers();
        visual_tools_->publishWireframeCuboid(roi_pose, roi_depth, roi_width, roi_height, rviz_visual_tools::CYAN);
        
        // show region of interest and bounding box
        pc_filter_ptr_->get_bbox_ = true;
        visual_tools_->publishWireframeCuboid(pc_filter_ptr_->bbox_pose_, pc_filter_ptr_->bbox_depth_,
                                              pc_filter_ptr_->bbox_width_, pc_filter_ptr_->bbox_height_,
                                              rviz_visual_tools::MAGENTA);
      }
      
      if (pc_filter_ptr_->aligned_cloud_->size() == 0)
        ROS_DEBUG_STREAM_NAMED("PC_filter.test","skipping aligned pub. zero size");
      else 
        aligned_cloud_pub_.publish(pc_filter_ptr_->aligned_cloud_);

      if (pc_filter_ptr_->roi_cloud_->size() == 0)
        ROS_DEBUG_STREAM_NAMED("PC_filter.test","skipping roi pub. zero size");
      else 
        roi_cloud_pub_.publish(pc_filter_ptr_->roi_cloud_);

      count++;
      rate.sleep();
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
