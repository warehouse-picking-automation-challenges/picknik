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
 * Desc    : A test for the generation of ply formats from a point cloud
 */

#include <ros/ros.h>

#include <picknik_perception/simple_point_cloud_filter.h>

#include <rviz_visual_tools/rviz_visual_tools.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/common/common_headers.h>

#include <Eigen/Core>

namespace picknik_perception
{

class PlyGenerationTest
{

private:

  ros::NodeHandle nh_;

  ros::Publisher pc_pub_;

  sensor_msgs::PointCloud2 pc_msg_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_;
  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

public:

  PlyGenerationTest()
    : nh_("~")
  {
    // Load
    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("base"));
    visual_tools_->deleteAllMarkers();

    // publish point cloud
    pc_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("point_cloud",1);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    point_cloud_ = cloud;

    // show origin
    visual_tools_->publishAxis(Eigen::Affine3d::Identity());

    // generate a point cloud for a cube
    addBoxToPointCloud(1.0, 1.0, 1.0, 10.0);

    // publish generated cloud
    std::size_t id = 0;
    point_cloud_->header.frame_id = "/world";
    point_cloud_->header.seq = id++;

    // generate ply file
    SimplePointCloudFilter::createPlyFile("test.ply", point_cloud_);
    
    ROS_DEBUG_STREAM_NAMED("ply_generation","starting main loop...");


    while (ros::ok())
    {
      pc_pub_.publish(point_cloud_);      
      

      ros::Duration(1.0).sleep();      
    }
  }

  void addBoxToPointCloud(float depth, float width, float height, float size)
  {
    // x faces at +/- depth/2
    // y faces at +/- width/2
    // z faces at +/- height/2
    pcl::PointXYZRGB point;
    float delta_x = depth / size;
    float delta_y = width / size;
    float delta_z = height / size;
    float x, y, z;
    uint8_t r, g, b;

    // x faces
    y = -width / 2.0;
    z = -height / 2.0;
    for (int i = 0; i < size +1; i++) // y direction
    {
      for (int j = 0; j < size +1; j++) // z direction
      {
        point.x = depth / 2.0;
        point.y = y + i * delta_y;
        point.z = z + j * delta_z;
        point_cloud_->points.push_back(point);

        point.x *= -1;
        point_cloud_->points.push_back(point);
      }
    }

    // y faces (already did edges with x faces)
    x = -depth / 2.0 + delta_x;
    z = -height / 2.0;
    for (int i = 0; i < size - 1; i++) // x direction
    {
      for (int j = 0; j < size + 1; j++) // z direction
      {
        point.x = x + i * delta_x;
        point.y = width / 2.0;
        point.z = z + j * delta_z;
        point_cloud_->points.push_back(point);

        point.y *= -1;
        point_cloud_->points.push_back(point);
      }
    }

    // z faces (already did edges with x & y faces)
    x = -depth / 2.0 + delta_x;
    y = -width / 2.0 + delta_y;
    for (int i = 0; i < size - 1; i++) // x direction
    {
      for (int j = 0; j < size - 1; j++) // y direction
      {
        point.x = x + i * delta_x;
        point.y = y + j * delta_y;
        point.z = height / 2.0;
        point_cloud_->points.push_back(point);

        point.z *= -1;
        point_cloud_->points.push_back(point);
      }
    } 
  }

}; // class

} // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ply_test");
  ROS_INFO_STREAM_NAMED("ply_generation.test","Starting ply generation test");

  ros::AsyncSpinner spinner(2);
  spinner.start();

  // seed random
  srand(ros::Time::now().toSec());

  picknik_perception::PlyGenerationTest tester;
}
