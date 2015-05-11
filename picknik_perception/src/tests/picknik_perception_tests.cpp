/*********************************************************************
 * Software License Agreement ("Modified BSD License")
 *
 * Copyright (c) 2014, University of Colorado, Boulder
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * * Neither the name of the Univ of CO, Boulder nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
/**
 * Authors : Andy Mcevoy
 * Desc    : Tests for picknik_perception functions
 */

#include <iostream>
#include <string>

#include <ros/ros.h>

#include <shape_msgs/Mesh.h>

#include <picknik_perception/simple_point_cloud_filter.h>
#include <bounding_box/bounding_box.h>
#include <bounding_box/mesh_utilities.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <Eigen/Core>

namespace picknik_perception
{

class PicknikPerceptionTester
{
private:

  ros::NodeHandle nh_;

  ros::Subscriber merged_sub_;
  ros::Subscriber roi_sub_;

  ros::Publisher cloud_pub_;
  ros::Publisher bbox_pub_;

  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged_cloud_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr roi_cloud_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr bbox_cloud_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;

  bounding_box::BoundingBox bbox_;

  std::size_t id_;
  bool is_done_;

public:
  PicknikPerceptionTester()
    : nh_("~")
  {
    ROS_INFO_STREAM_NAMED("picknik_perception.tester","Starting picknik_perception tester...");

    // Load
    visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("base", "perception_test_markers"));
    visual_tools_->deleteAllMarkers();

    // initialize variables
    is_done_ = false;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    merged_cloud_ = merged_cloud;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr roi_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    roi_cloud_ = roi_cloud;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud_ = cloud;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr bbox_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    bbox_cloud_ = bbox_cloud;

    id_ = 0;

    // subscribe to point clouds
    merged_sub_ = nh_.subscribe("/merge_point_clouds/points", 1, 
                            &PicknikPerceptionTester::mergedPointCloudCallback, this);

    roi_sub_ = nh_.subscribe("/pcl_perception_server/roi_cloud", 1, 
                             &PicknikPerceptionTester::roiPointCloudCallback, this);

    // publish point clouds
    cloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("cloud",1);
    bbox_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("bbox_cloud",1);

    printMenu();
    char menu_selection;

    while (ros::ok() && !is_done_)
    {
      // get test to run
      std::cout << "\nEnter your choice: " << std::endl;
      std::cin >> menu_selection;

      runTest(menu_selection);
    }
  }

  void testSaveRegionOfInterest()
  {
    ROS_INFO_STREAM_NAMED("picknik_perception.tester","Testing point cloud save feature...");
    
    if (merged_cloud_->size() == 0)
    {
      ROS_WARN_STREAM_NAMED("picknik_perception.tester","empty point cloud, not saving to disk");
      return;
    }

    SimplePointCloudFilter::saveRegionOfInterest(roi_cloud_);

  }

  void loadPCDFromFile()
  {
    ROS_INFO_STREAM_NAMED("picknik_perception.tester","Loading point cloud from file...");
    
    cloud_->clear();

    std::string file_name;
    std::cout << "Enter name of file to load: ";
    std::cin >> file_name;

    // Save files in picknik_perception/data/roi_pcds
    std::string package_path = ros::package::getPath("picknik_perception");
    
    std::string full_path = package_path + "/data/roi_pcds/" + file_name;
    std::cout << full_path << std::endl;

    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (full_path, *cloud_) == -1)
    {
      ROS_WARN_STREAM_NAMED("picknik_perception.tester","couldn't load file: " << full_path);
      return;
    }
    
    ROS_DEBUG_STREAM_NAMED("picknik_perception.tester","Loaded file with " << cloud_->size() << " points");

    cloud_->header.frame_id = "/world";
    cloud_->header.seq = ++id_;
    cloud_pub_.publish(cloud_);
  }

  void getPCBoundingBox(bool drop_points)
  {
    ROS_INFO_STREAM_NAMED("picknik_perception.tester","Getting bounding box of point cloud...");

    Eigen::Affine3d bbox_pose = Eigen::Affine3d::Identity();
    double bbox_depth, bbox_width, bbox_height;

    bbox_.drop_pose_ = Eigen::Affine3d::Identity();
    bbox_.drop_plane_ = bounding_box::XY;
    bbox_.drop_points_ = drop_points;

    bbox_.getBodyAlignedBoundingBox(cloud_, Eigen::Affine3d::Identity(), bbox_pose, bbox_depth, bbox_width, bbox_height);

    // display results
    visual_tools_->publishAxis(bbox_pose);
    visual_tools_->publishCuboid(bbox_pose, bbox_depth, bbox_width, bbox_height, rviz_visual_tools::TRANSLUCENT_DARK);

    bbox_cloud_->header.frame_id = "/world";
    bbox_cloud_->header.seq = ++id_;
    ROS_DEBUG_STREAM_NAMED("picknik_perception.tester","Size of bbox cloud = " << bbox_.cloud_->size());
    bbox_pub_.publish(bbox_.cloud_);
  }

  void testCreateMesh()
  {
    shape_msgs::Mesh mesh_msg;
    mesh_msg = bounding_box::createMeshMsg(cloud_, Eigen::Affine3d::Identity());
    ROS_DEBUG_STREAM_NAMED("test","triangle count = " << mesh_msg.triangles.size() << ", vertex count = " << mesh_msg.vertices.size());
    visual_tools_->publishCollisionMesh(Eigen::Affine3d::Identity(), "/world", mesh_msg, rviz_visual_tools::BLUE);
  }


  void mergedPointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    pcl::fromROSMsg(*msg, *merged_cloud_);
  }

  void roiPointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    pcl::fromROSMsg(*msg, *roi_cloud_);
  }

  void runTest(char menu_selection)
  {
    visual_tools_->deleteAllMarkers();
    visual_tools_->publishAxis(Eigen::Affine3d::Identity());

    switch (menu_selection)
    {
      case ('a'):
        testSaveRegionOfInterest();
        break;
      case ('b'):
        loadPCDFromFile();
        break;
      case ('c'):
        loadPCDFromFile();
        getPCBoundingBox(false);
        break;
      case ('d'):
        loadPCDFromFile();
        getPCBoundingBox(true);
        break;
      case ('e'):
        loadPCDFromFile();
        testCreateMesh();
        break;
      case (27):
        is_done_ = true;
        break;
      default:
        std::cout << "Not a valid choice. Please try again." << std::endl;
        break;
    }
  }

  void printMenu()
  {
    std::cout << "Choose test to run: "<< std::endl;
    std::cout << "a.\tSave point cloud to file" << std::endl;
    std::cout << "b.\tLoad point cloud from file" << std::endl;
    std::cout << "c.\tLoad point cloud and get bounding box" << std::endl;
    std::cout << "d.\tLoad point cloud and get bounding box with drop points" << std::endl;
    std::cout << "e.\tCreate mesh message from PCD file" << std::endl;
    std::cout << "Esc\tExit" << std::endl;
  }

}; // end class PicknikPerceptionTester



} // end namespace picknik_perception

int main (int argc, char** argv)
{
  ros::init(argc, argv, "picknik_perception_tester");

  ros::AsyncSpinner spinner(2);
  spinner.start();

  picknik_perception::PicknikPerceptionTester tester;
}
