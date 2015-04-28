/*
  Author: Andy McEvoy (mcevoy.andy@gmail.com)
  Desc  : Test for SimplePointCloudFilter class
*/

#include <picknik_perception/simple_point_cloud_filter.h>
#include <picknik_perception/manual_tf_alignment.h>
#include <picknik_perception/manipulation_interface.h>

#include <rviz_visual_tools/rviz_visual_tools.h>

namespace picknik_perception
{

class SimplePerceptionServer
{
private:
  ros::NodeHandle nh_;

  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

  SimplePointCloudFilter* pc_filter_ptr_;

  ros::Subscriber pc_sub_;
  ros::Publisher aligned_cloud_pub_;
  ros::Publisher roi_cloud_pub_;

  picknik_perception::ManipulationInterfacePtr manipulation_interface_;

public:

  SimplePerceptionServer()
    : nh_("~")
  {
    // Load
    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("base", "/picknik_main/markers"));
    visual_tools_->deleteAllMarkers();

    // Load communication
    manipulation_interface_.reset(new picknik_perception::ManipulationInterface());

    pc_filter_ptr_ = new SimplePointCloudFilter();

    // listen to point cloud topic
    // TODO: camera topic should be set in launch file
    pc_sub_ = nh_.subscribe("/camera/depth_registered/points", 1,
                            &picknik_perception::SimplePointCloudFilter::pointCloudCallback, pc_filter_ptr_);

    // publish aligned point cloud and bin point cloud
    aligned_cloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("aligned_cloud",1);
    roi_cloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("roi_cloud",1);

    // define region of interest
    // Eigen::Affine3d roi_pose = Eigen::Affine3d::Identity();
    // double roi_depth = 0.3;
    // double roi_width = 0.25;
    // double roi_height = 0.2;
    // roi_pose.translation() += Eigen::Vector3d(roi_depth / 2.0 + 0.02, -0.25, 0.85 + roi_height / 2.0);
    // pc_filter_ptr_->setRegionOfInterest(roi_pose, roi_depth, roi_width, roi_height);

    ros::Rate rate(40.0);

    ROS_DEBUG_STREAM_NAMED("simple_perception_server","Starting point cloud filter");
    while ( ros::ok() )
    {
      rate.sleep();

      // Check if goal is recieved
      picknik_msgs::FindObjectsGoalConstPtr goal;
      if (!manipulation_interface_->isReadyToStartPerception(goal))
      {
        ros::Duration(0.1).sleep();
        continue; // loop again
      }
      ROS_INFO_STREAM_NAMED("simple_perception_server","Starting perception for " << goal->bin_name << ", waiting for stop command");

      // Set regions of interest
      pc_filter_ptr_->setRegionOfInterest(visual_tools_->convertPose(goal->front_bottom_right), 
                                          visual_tools_->convertPose(goal->back_top_left));

      // TODO: this stop command is not really applicable to this method of perception
      while (ros::ok())
      {
        // Do perception processing HERE

        // Wait until camera is done moving
        if (manipulation_interface_->isReadyToStopPerception())
          break;

        // Wait
        rate.sleep();
      }

      // Finish up perception
      ROS_INFO_STREAM_NAMED("simple_perception_server","Finishing up perception");
      visual_tools_->deleteAllMarkers();

      // show world CS
      visual_tools_->publishAxis(Eigen::Affine3d::Identity());

      // show region of interest and bounding box
      visual_tools_->publishAxis(pc_filter_ptr_->roi_pose_);
      //visual_tools_->publishWireframeCuboid(roi_pose, roi_depth, roi_width, roi_height, rviz_visual_tools::CYAN);
      pc_filter_ptr_->get_bbox_ = true;

      visual_tools_->publishWireframeCuboid(pc_filter_ptr_->bbox_pose_, pc_filter_ptr_->bbox_depth_,
                                            pc_filter_ptr_->bbox_width_, pc_filter_ptr_->bbox_height_,
                                            rviz_visual_tools::MAGENTA);

      // publish point clouds for rviz
      if (pc_filter_ptr_->aligned_cloud_->size() ==0)
      {
        continue;
      }
      aligned_cloud_pub_.publish(pc_filter_ptr_->aligned_cloud_);
      roi_cloud_pub_.publish(pc_filter_ptr_->roi_cloud_);

      // Create results
      picknik_msgs::FindObjectsResult result;

      // For each object in the bin
      for (std::size_t i = 0; i < goal->expected_objects_names.size(); ++i)
      {
        picknik_msgs::FoundObject new_product;
        new_product.object_name = goal->expected_objects_names[i];

        // TODO Object pose 
        new_product.object_pose.position.x = 0.45;
        new_product.object_pose.position.y = 0;
        new_product.object_pose.position.z = 0;
        new_product.object_pose.orientation.x = 0;
        new_product.object_pose.orientation.y = 0;
        new_product.object_pose.orientation.z = 0;
        new_product.object_pose.orientation.w = 1;

        // Value between 0 and 1 for each expected object's confidence of its pose
        new_product.expected_object_confidence = 1.0;

        // Add object to result
        result.found_objects.push_back(new_product);
      } // end for each product

      // If the camera angle was bad or some other failure, return false
      result.succeeded = true;
      //ROS_INFO_STREAM_NAMED("simple_perception_server","Sending perception result:\n" << result);
      ROS_INFO_STREAM_NAMED("simple_perception_server","Sending perception result");

      // Check if cancel is desired
      if (manipulation_interface_->resetIsNeeded())
        break;

      manipulation_interface_->sendPerceptionResults(result);

    } // end main while loop

  } // end function

}; // class

} // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_perception_server");
  ROS_INFO_STREAM_NAMED("simple_perception_server","Starting simple point cloud filter test");

  ros::AsyncSpinner spinner(2);
  spinner.start();

  picknik_perception::SimplePerceptionServer server;

  ros::waitForShutdown();

  ROS_INFO_STREAM_NAMED("main", "Shutting down.");
  ros::shutdown();  
}
