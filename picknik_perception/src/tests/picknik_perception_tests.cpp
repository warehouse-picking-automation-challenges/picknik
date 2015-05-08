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


#include <ros/ros.h>
#include <picknik_perception/simple_point_cloud_filter.h>


namespace picknik_perception
{

class PicknikPerceptionTester
{
private:

  ros::NodeHandle nh_;
  ros::Subscriber merged_sub_;

  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged_cloud_;

  bool is_done_;

public:
  PicknikPerceptionTester()
    : nh_("~")
  {
    ROS_INFO_STREAM_NAMED("picknik_perception.tester","Starting picknik_perception tester...");

    // initialize variables
    is_done_ = false;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    merged_cloud_ = merged_cloud;

    // subscribe to point clouds
    merged_sub_ = nh_.subscribe("/merge_point_clouds/points", 1, 
                            &PicknikPerceptionTester::mergedPointCloudCallback, this);

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

    SimplePointCloudFilter::saveRegionOfInterest(merged_cloud_);

  }

  void mergedPointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    pcl::fromROSMsg(*msg, *merged_cloud_);
  }

  void runTest(char menu_selection)
  {
    switch (menu_selection)
    {
      case ('a'):
        testSaveRegionOfInterest();
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
