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
  ManualTFAlignment* tf_align_ptr_;
  
  ros::Subscriber pc_sub_;
  ros::Subscriber keyboard_sub_;
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
    tf_align_ptr_ = new ManualTFAlignment();

    // set initial camera transform
    // TODO: Should be read in from file or set in launch file
    //-1.202   -0.23   1.28    0      0.03    0
    tf_align_ptr_->setPose(Eigen::Vector3d(-1.202, -0.23, 1.28), Eigen::Vector3d(0, 0.03, 0));
    tf_align_ptr_->from_ = "/world";
    tf_align_ptr_->to_ = "/camera_link";

    // listen to keyboard topic
    keyboard_sub_ = nh_.subscribe("/keyboard/keydown", 100, 
                                  &picknik_perception::ManualTFAlignment::keyboardCallback, tf_align_ptr_);
    tf_align_ptr_->printMenu();

    // listen to point cloud topic
    // TODO: camera topic should be set in launch file
    pc_sub_ = nh_.subscribe("/camera/depth_registered/points", 1, 
                            &picknik_perception::SimplePointCloudFilter::pointCloudCallback, pc_filter_ptr_);

    // publish aligned point cloud and bin point cloud
    aligned_cloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("aligned_cloud",1);
    roi_cloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("roi_cloud",1);

    // define region of interest
    Eigen::Affine3d roi_pose = Eigen::Affine3d::Identity();
    double roi_depth = 0.3;
    double roi_width = 0.25;
    double roi_height = 0.2;
    roi_pose.translation() += Eigen::Vector3d(roi_depth / 2.0 + 0.02, -0.25, 0.85 + roi_height / 2.0);
    pc_filter_ptr_->setRegionOfInterest(roi_pose, roi_depth, roi_width, roi_height);

    ros::Rate rate(40.0);
    int bbox_rate = 10;
    int count = 1; // don't want a bounding box on first frame.

    ROS_DEBUG_STREAM_NAMED("PC_filter.test","starting main filter test");
    while ( ros::ok() )
    {
      // publish transform to camera
      tf_align_ptr_->publishTF();

      // get bounding box at specified interval
      if (count % bbox_rate == 0)
      {
        ROS_DEBUG_STREAM_NAMED("PC_filter.test","getting bbox...");
        visual_tools_->deleteAllMarkers();
        
        // show world CS
        visual_tools_->publishAxis(Eigen::Affine3d::Identity());

        // show region of interest and bounding box
        visual_tools_->publishAxis(pc_filter_ptr_->roi_pose_);
        visual_tools_->publishWireframeCuboid(roi_pose, roi_depth, roi_width, roi_height, rviz_visual_tools::CYAN);
        pc_filter_ptr_->get_bbox_ = true;
        
        visual_tools_->publishWireframeCuboid(pc_filter_ptr_->bbox_pose_, pc_filter_ptr_->bbox_depth_,
                                              pc_filter_ptr_->bbox_width_, pc_filter_ptr_->bbox_height_,
                                              rviz_visual_tools::MAGENTA);

        count = 0;
      }
      
      // publish point clouds for rviz
      if (pc_filter_ptr_->aligned_cloud_->size() ==0)
      {
        continue;
      }
      aligned_cloud_pub_.publish(pc_filter_ptr_->aligned_cloud_);
      roi_cloud_pub_.publish(pc_filter_ptr_->roi_cloud_);

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
