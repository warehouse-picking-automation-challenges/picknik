/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, University of Colorado, Boulder
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

/* Author: Dave Coleman
   Desc:   Does not actually update the octomap, only filters point clouds
*/

#ifndef MOVEIT_PERCEPTION_POINTCLOUD_FILTER_
#define MOVEIT_PERCEPTION_POINTCLOUD_FILTER_

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/PointCloud2.h>
#include <moveit/point_containment_filter/shape_mask.h>
#include <geometric_shapes/shapes.h>
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <moveit/occupancy_map_monitor/occupancy_map.h>

namespace picknik_perception
{

typedef unsigned int ShapeHandle;
typedef std::map<ShapeHandle, Eigen::Affine3d, std::less<ShapeHandle>,
                 Eigen::aligned_allocator<std::pair<const ShapeHandle, Eigen::Affine3d> > > ShapeTransformCache;
typedef boost::function<bool(const std::string &target_frame, const ros::Time &target_time, ShapeTransformCache &cache)> TransformCacheProvider;

class PointCloudFilter
{
public:

  PointCloudFilter(const boost::shared_ptr<tf::Transformer> &tf, const std::string& map_frame);

  ~PointCloudFilter();

  bool initialize();
  void start();
  void stop();
  ShapeHandle excludeShape(const shapes::ShapeConstPtr &shape, const double &scale, const double &padding);
  void forgetShape(ShapeHandle handle);

  void setTransformCacheCallback(const TransformCacheProvider &transform_callback)
  {
    transform_provider_callback_ = transform_callback;
  }

  void publishDebugInformation(bool flag)
  {
    debug_info_ = flag;
  }

protected:

  TransformCacheProvider transform_provider_callback_;
  ShapeTransformCache transform_cache_;
  bool debug_info_;

  bool updateTransformCache(const std::string &target_frame, const ros::Time &target_time);

private:

  bool getShapeTransform(ShapeHandle h, Eigen::Affine3d &transform) const;
  void cloudMsgCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg);
  void stopHelper();

  ros::NodeHandle root_nh_;
  ros::NodeHandle private_nh_;
  boost::shared_ptr<tf::Transformer> tf_;

  /* params */
  std::string point_cloud_topic_;
  double max_range_;
  unsigned int point_subsample_;
  std::string filtered_cloud_topic_;
  ros::Publisher filtered_cloud_publisher_;

  message_filters::Subscriber<sensor_msgs::PointCloud2> *point_cloud_subscriber_;
  tf::MessageFilter<sensor_msgs::PointCloud2> *point_cloud_filter_;

  /* used to store all cells in the map which a given ray passes through during raycasting.
     we cache this here because it dynamically pre-allocates a lot of memory in its contsructor */
  //octomap::KeyRay key_ray_;

  boost::scoped_ptr<point_containment_filter::ShapeMask> shape_mask_;
  std::vector<int> mask_;

  std::string map_frame_;
}; // class

typedef boost::shared_ptr<PointCloudFilter> PointCloudFilterPtr;
typedef boost::shared_ptr<const PointCloudFilter> PointCloudFilterConstPtr;

} // namespace

#endif
