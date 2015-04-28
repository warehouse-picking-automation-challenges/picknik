/* 
   Authors : Andy McEvoy (mcevoy.andy@gmail.com)
   Desc    : Preprocessing of point cloud
*/

#include <picknik_perception/simple_point_cloud_filter.h>

#include <rviz_visual_tools/rviz_visual_tools.h>
#include <rviz_visual_tools/ros_param_utilities.h>


namespace picknik_perception
{

class PreprocessingFilter
{
private:
  ros::NodeHandle nh_;
  ros::Subscriber pc_sub_;

  ros::Publisher roi_cloud_pub_;
  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

  SimplePointCloudFilter* filter_ptr_;

public:
  PreprocessingFilter()
    : nh_("~")
  {

    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("base"));
    visual_tools_->deleteAllMarkers();

    filter_ptr_ = new SimplePointCloudFilter();

    // TODO: read from config file or pass as arg
    pc_sub_ = nh_.subscribe("/xtion_left/depth_registered/points", 1, 
                            &picknik_perception::SimplePointCloudFilter::pointCloudCallback, filter_ptr_);

    // publish aligned point cloud and bin point cloud
    roi_cloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("roi_cloud",1);

    // define region of interest
    // TODO: read from config file
    Eigen::Affine3d roi_pose = Eigen::Affine3d::Identity();
    double roi_depth = 0.875;
    double roi_width = 0.873;
    double roi_height = 2.37;
    roi_pose.translation() += Eigen::Vector3d( 0.687 + roi_depth / 2.0, 
                                              -0.438 + roi_width / 2.0, 
                                               0.002 + roi_height / 2.0);
    filter_ptr_->setRegionOfInterest(roi_pose, roi_depth, roi_width, roi_height);

    // show region of interest and bounding box
    visual_tools_->publishWireframeCuboid(roi_pose, roi_depth, roi_width, roi_height, rviz_visual_tools::CYAN);

    ROS_DEBUG_STREAM_NAMED("PC_preprocess","trimming point cloud down to region of interest...");

    std::size_t id;
    while (ros::ok())
    {
      if (!filter_ptr_->processing_)
      {
        filter_ptr_->roi_cloud_->header.seq = id++;
        filter_ptr_->roi_cloud_->header.frame_id = "/world";
        //      pc_filter_ptr_->roi_cloud_->header.stamp = ros::Time::now();        
        roi_cloud_pub_.publish(filter_ptr_->roi_cloud_);
      }
    }
  }
}; // end class PreprocessingFilter

} // end namespace picknik_perception

int main (int argc, char** argv)
{
  ros::init(argc, argv, "roi_pc");

  ros::AsyncSpinner spinner(2);
  spinner.start();

  picknik_perception::PreprocessingFilter filter;

}
