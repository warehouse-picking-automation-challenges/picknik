/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Andy McEvoy <mcevoy.andy@gmail.com>, Dave Coleman <dave@dav.ee>
   Desc:   Perception server for finding objects in a shelf
*/

#include <picknik_perception/simple_point_cloud_filter.h>
#include <picknik_perception/manual_tf_alignment.h>
#include <picknik_perception/manipulation_interface.h>

// Bounding Box
#include <bounding_box/mesh_utilities.h>

// Visualization
#include <rviz_visual_tools/rviz_visual_tools.h>

// Parameter loading
#include <rviz_visual_tools/ros_param_utilities.h>

namespace picknik_perception
{

class PCLPerceptionServer
{

public:

  PCLPerceptionServer()
    : nh_("~")
  {
    // Load visualizer
    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("base", "/picknik_main/product_perception"));
    visual_tools_->deleteAllMarkers();

    // Load communication
    manipulation_interface_.reset(new picknik_perception::ManipulationInterface());

    // Load filter
    pointcloud_filter_.reset(new SimplePointCloudFilter(visual_tools_));

    // listen to point cloud topic
    ros::Duration(1.0).sleep();
    ros::spinOnce();
    pointcloud_sub_ = nh_.subscribe("/merge_point_clouds/points", 1,
                                    &picknik_perception::SimplePointCloudFilter::pointCloudCallback, pointcloud_filter_);

    // Load parameters
    const std::string parent_name = "pcl_perception_server"; // for namespacing logging messages
    rviz_visual_tools::getDoubleParameter(parent_name, nh_, "roi_reduction_padding_x", roi_reduction_padding_x_);
    rviz_visual_tools::getDoubleParameter(parent_name, nh_, "roi_reduction_padding_y", roi_reduction_padding_y_);
    rviz_visual_tools::getDoubleParameter(parent_name, nh_, "roi_reduction_padding_z", roi_reduction_padding_z_);

    // Show whole shelf
    loadShelfROI();
  }

  bool changePointCloudTopic(std::string topic)
  {
    ROS_INFO_STREAM_NAMED("pcl_perception_server","Changing point cloud topic to: " << topic);
    pointcloud_sub_.shutdown();
    pointcloud_sub_ = nh_.subscribe(topic, 1,
                                    &picknik_perception::SimplePointCloudFilter::pointCloudCallback,
                                    pointcloud_filter_);
  }

  bool mainPipeline()
  {
    ros::Rate rate(40.0);

    ROS_DEBUG_STREAM_NAMED("pcl_perception_server","Point cloud server main loop started");

    // Main looop
    while ( ros::ok() )
    {
      rate.sleep();

      // Check if goal is recieved
      picknik_msgs::FindObjectsGoalConstPtr request;
      if (!manipulation_interface_->isReadyToStartPerception(request))
      {
        ros::Duration(0.1).sleep();
        continue; // loop again
      }
      ROS_INFO_STREAM_NAMED("pcl_perception_server","Starting perception for " << request->bin_name << ", waiting for stop command");

      // Centroid of bin
      Eigen::Affine3d roi_centroid = visual_tools_->convertPose(request->bin_centroid);

      // Size of the bin
      const double &bin_height = request->bin_dimensions.dimensions[shape_msgs::SolidPrimitive::BOX_Z];
      const double &bin_width  = request->bin_dimensions.dimensions[shape_msgs::SolidPrimitive::BOX_Y];
      const double &bin_depth  = request->bin_dimensions.dimensions[shape_msgs::SolidPrimitive::BOX_X];

      // translate pose to corners
      Eigen::Vector3d translate_to_corners(bin_depth / 2.0, bin_width / 2.0, bin_height / 2.0);

      Eigen::Affine3d front_bottom_right = roi_centroid;
      front_bottom_right.translation() -= translate_to_corners;
      Eigen::Affine3d back_top_left = roi_centroid;
      back_top_left.translation() += translate_to_corners;

      // Set regions of interest
      pointcloud_filter_->setRegionOfInterest(front_bottom_right, back_top_left, roi_reduction_padding_x_, roi_reduction_padding_y_, roi_reduction_padding_z_);



      // TODO: this stop command is not really applicable to this method of perception
      while (ros::ok())
      {
        // Do perception processing HERE

        // Wait until camera is done moving
        if (manipulation_interface_->isReadyToStopPerception())
          break;

        // show bounding box
        //pointcloud_filter_->outlier_removal_ = true;
        pointcloud_filter_->enableBoundingBox();

        // Wait
        rate.sleep();
      }
      pointcloud_filter_->outlier_removal_ = false;

      // Finish up perception
      ROS_INFO_STREAM_NAMED("pcl_perception_server","Finishing up perception");

      // Convert point cloud to have its origin at the point cloud's centroid
      // Transform a sensor_msgs::PointCloud2 dataset using an Eigen 4x4 matrix.
      //const Eigen::Matrix4f transform = bbox_pose_.matrix();
      //tf::Transform transform;
      //pcl::PointCloud<pcl::PointXYZRGB>::Ptr product_point_cloud;
      //pcl_ros::transformPointCloud (*pointcloud_filter_->roi_cloud_, *product_point_cloud, transform);

      // void pcl_ros::transformPointCloud(const pcl::PointCloud< PointT > & cloud_in,
      //                                   pcl::PointCloud< PointT > & cloud_out,
      //                                   const tf::Transform & transform

      // Convert point cloud to mesh
      shape_msgs::Mesh mesh_msg;
      mesh_msg = bounding_box::createMeshMsg(pointcloud_filter_->roi_cloud_, pointcloud_filter_->getObjectPose());
      ROS_INFO_STREAM_NAMED("pcl_perception_server","Finished computing mesh msg");
      ROS_DEBUG_STREAM_NAMED("test","sizes = " << mesh_msg.triangles.size() << ", " << mesh_msg.vertices.size());
      //visual_tools_->publishCollisionMesh(Eigen::Affine3d::Identity(), "object", mesh_msg, rviz_visual_tools::BLUE);

      // Create results
      picknik_msgs::FindObjectsResult result;

      if (request->expected_objects_names.size() > 1)
        ROS_WARN_STREAM_NAMED("pcl_perception_server","Perception can currently only handle one product");

      // For each object in the bin
      for (std::size_t i = 0; i < request->expected_objects_names.size(); ++i)
      {
        picknik_msgs::FoundObject new_product;
        new_product.object_name = request->expected_objects_names[i];

        // Object pose
        pointcloud_filter_->getObjectPose(new_product.object_pose.pose);
        new_product.object_pose.header.frame_id = "world";

        // Value between 0 and 1 for each expected object's confidence of its pose
        new_product.expected_object_confidence = 1.0;

        // Set mesh
        new_product.bounding_mesh = mesh_msg;

        // Add object to result
        result.found_objects.push_back(new_product);

        break; // for now we only handle one prodcut
      } // end for each product

      // If the camera angle was bad or some other failure, return false
      result.succeeded = true;
      //ROS_INFO_STREAM_NAMED("pcl_perception_server","Sending perception result:\n" << result);
      ROS_INFO_STREAM_NAMED("pcl_perception_server","Sending perception result");

      // Check if cancel is desired
      if (manipulation_interface_->resetIsNeeded())
        break;

      manipulation_interface_->sendPerceptionResults(result);

      ROS_INFO_STREAM_NAMED("pcl_perception_server","Ready for next perception request");

      // Show whole shelf
      //loadShelfROI();

    } // end main while loop

  } // end function

  bool loadShelfROI()
  {
    // load default ROI - the whole shelf
    Eigen::Affine3d top_left_back_corner = Eigen::Affine3d::Identity();
    top_left_back_corner.translation() = Eigen::Vector3d(1.531, 0.4365, 2.37);
    Eigen::Affine3d bottom_right_front_corner = Eigen::Affine3d::Identity();
    bottom_right_front_corner.translation() = Eigen::Vector3d(0.656, -0.445, 0.002);
    double reduction_padding = 0;
    pointcloud_filter_->setRegionOfInterest( bottom_right_front_corner, top_left_back_corner, reduction_padding, reduction_padding, reduction_padding);

    return true;
  }

private:
  ros::NodeHandle nh_;

  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

  SimplePointCloudFilterPtr pointcloud_filter_;

  ros::Subscriber pointcloud_sub_;

  picknik_perception::ManipulationInterfacePtr manipulation_interface_;

  // Amount to reduce the shelf region of interest by for error compensation
  double roi_reduction_padding_x_;
  double roi_reduction_padding_y_;
  double roi_reduction_padding_z_;

}; // class

} // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl_perception_server");
  ROS_INFO_STREAM_NAMED("pcl_perception_server","Starting PCL-based perception server");

  ros::AsyncSpinner spinner(2);
  spinner.start();

  picknik_perception::PCLPerceptionServer server;
  server.mainPipeline();

  ros::waitForShutdown();

  ROS_INFO_STREAM_NAMED("main", "Shutting down.");
  ros::shutdown();
}
