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

/* Author: Dave Coleman <dave@dav.ee>
   Desc:   Filters shapes from a point cloud
*/

#ifndef PICKNIK_PERCEPTION__SHAPE_FILTER
#define PICKNIK_PERCEPTION__SHAPE_FILTER

#include <picknik_perception/point_cloud_filter.h>

// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

namespace picknik_perception
{

class ShapeFilter
{
public:

  /**
   * \brief Constructor
   * \param verbose - run in debug mode
   */
  ShapeFilter(bool verbose)
    : verbose_(verbose)
  {
    // Create tf transformer
    tf_.reset(new tf::TransformListener(nh_));

    //std::string map_frame = "world";
    std::string map_frame = "";

    point_cloud_filter_.reset(new PointCloudFilter(tf_, map_frame));
    point_cloud_filter_->initialize();

    // Load shelf cad
    shapes::ShapeConstPtr shape;
    if (!loadShelf(shape))
    {
      ROS_ERROR_STREAM_NAMED("shape_filter","Unable to load shelf stl");
      return;
    }

    // Exclude
    const double scale = 1;
    const double padding = 1;
    point_cloud_filter_->excludeShape(shape, scale, padding);

    // Start filtering
    point_cloud_filter_->start();

    ROS_INFO_STREAM_NAMED("ShapeFilter","ShapeFilter Ready.");
  }

  bool loadShelf(shapes::ShapeConstPtr shape)
  {
    std::string collision_mesh_path = "file:///home/dave/ros/ws_amazon/src/picknik/picknik_main/meshes/kiva_pod/meshes/pod_lowres.stl";
    //shapes::Shape *mesh = shapes::createMeshFromResource(collision_mesh_path); // make sure its prepended by file://
    
    shape.reset(shapes::createMeshFromResource(collision_mesh_path)); // make sure its prepended by file://
    // &shape = shapes::createMeshFromResource(collision_mesh_path); // make sure its prepended by file://
    //shapes::ShapeMsg shape_msg; // this is a boost::variant type from shape_messages.h

    // if (!mesh || !shapes::constructMsgFromShape(mesh, shape_msg))
    // {
    //   ROS_ERROR_STREAM_NAMED("shelf","Unable to create mesh shape message from resource " << collision_mesh_path);
    //   return false;
    // }

    //*shape = mesh;
    //mesh_msg_ = boost::get<shape_msgs::Mesh>(shape_msg);

    return true;
  }


private:

  // A shared node handle
  ros::NodeHandle nh_;

  // Show more visual and console output, with general slower run time.
  bool verbose_;

  PointCloudFilterPtr point_cloud_filter_;

  boost::shared_ptr<tf::TransformListener> tf_;

}; // end class

// Create boost pointers for this class
typedef boost::shared_ptr<ShapeFilter> ShapeFilterPtr;
typedef boost::shared_ptr<const ShapeFilter> ShapeFilterConstPtr;

} // end namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ShapeFilter");
  ROS_INFO_STREAM_NAMED("main", "Starting ShapeFilter...");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Check for verbose flag
  bool verbose = false;
  if (argc > 1)
  {
    for (std::size_t i = 0; i < argc; ++i)
    {
      if (strcmp(argv[i], "--verbose") == 0)
      {
        ROS_INFO_STREAM_NAMED("main","Running in VERBOSE mode (slower)");
        verbose = true;
        continue;
      }
    }
  }

  picknik_perception::ShapeFilter server(verbose);
  ros::spin();

  ROS_INFO_STREAM_NAMED("main", "Shutting down.");
  ros::shutdown();

  return 0;
}

#endif
