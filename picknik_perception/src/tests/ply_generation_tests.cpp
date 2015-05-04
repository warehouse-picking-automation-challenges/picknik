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

#include <Eigen/Core>

namespace picknik_perception
{

class PlyGenerationTest
{

private:

  ros::NodeHandle nh_;

  ros::Publisher pc_pub_;

  sensor_msgs::PointCloud2 pc_msg_;

}

} // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ply_generation_test");
  ROS_INFO_STREAM_NAMED("ply_generation.test","Starting ply generation test");

  ros::AsyncSpinner spinner(2);
  spinner.start();

  // seed random
  srand(ros::Time::now().toSec());

  picknik_perception::PlyGenerationTest tester;
}
