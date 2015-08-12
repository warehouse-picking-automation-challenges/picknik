/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Dave Coleman <dave@dav.ee>
 *  All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *********************************************************************/
/*
  Author: Dave Coleman <dave@dav.ee>
  Desc:   Holder for multiple visuals tools
*/

// PickNik
#include <picknik_main/line_tracking.h>

namespace picknik_main
{

LineTracking::LineTracking(VisualsPtr visuals) : visuals_(visuals) {

  std::size_t queue_size = 1;
  end_effector_data_sub_ = nh_.subscribe("/end_effector_data", queue_size, &LineTracking::dataCallback, this);
}

void LineTracking::dataCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
  bool verbose = false;

  // Error check
  if (msg->data.size() < 6) {
    ROS_ERROR_STREAM_NAMED("line_tracking","Invalid number of end effector data points recieved: "
                           << msg->data.size());
    return;
  }
  
  // Unpack vector to variable names
  double pt1_x = msg->data[1];
  double pt1_y = msg->data[2];
  const double& eigen_vec_x =msg->data[3];
  const double& eigen_vec_y = msg->data[4];
  const double& eigen_vec_val = msg->data[4];
  const double& sheer_displacment_x = msg->data[5];
  const double& sheer_displacment_y = msg->data[6];
  
  // Calculate point 2
  const double distance_between_points = 2000;
  double pt2_x = pt1_x + distance_between_points * (eigen_vec_x * eigen_vec_val);
  double pt2_y = pt1_y + distance_between_points * (eigen_vec_y * eigen_vec_val);

  if (verbose)
    std::cout << "Pixels: (" << pt1_x << ", " << pt1_y << ") (" << pt2_x << ", " << pt2_y << ")";

  // Convert to meters
  convertPixelToMeters(pt1_x, pt1_y);
  convertPixelToMeters(pt2_x, pt2_y);
  
  // Convert to ROS msg
  geometry_msgs::PoseStamped pt1;  
  pt1.pose.position.x = pt1_x;
  pt1.pose.position.y = pt1_y;
  pt1.pose.position.z = 0;
  pt1.header.frame_id = ATTACH_FRAME;
  
  geometry_msgs::PoseStamped pt2;  
  pt2.pose.position.x = pt2_x;
  pt2.pose.position.y = pt2_y;
  pt2.pose.position.z = 0;
  pt2.header.frame_id = ATTACH_FRAME;

  if (verbose)
    std::cout << "Meters: (" << pt1.pose.position.x << ", " << pt1.pose.position.y << ") (" << pt2.pose.position.x << ", " << pt2.pose.position.y << ")" << std::endl;
    
  // Publish visuals
  visuals_->visual_tools_->publishSphere(pt1, rvt::GREEN,  visuals_->visual_tools_->getScale(rvt::XXSMALL), "Sphere1", 1);
  visuals_->visual_tools_->publishSphere(pt2, rvt::BLUE, visuals_->visual_tools_->getScale(rvt::XXSMALL), "Sphere2", 2);
  publishUpdatedLine(pt1.pose.position, pt2.pose.position);

  // Get tool direction
  double theta = atan2(eigen_vec_y, eigen_vec_x);
  getToolDirection(pt1, theta);
}

void LineTracking::convertPixelToMeters(double &x, double &y) {
  // Input points are in the range of x: (0-480) y: (0-640)
  // Transform these to the width of the gelsight 25mm x 25mm (0.025m x 0.025m)
  //  in     out
  //  ----  -----
  //  640   0.025
  //
  // out = 0.025 * in / 640
  
  x = 0.025 * x / 640; // TODO the range is actually 480, do i need to change this?
  y = 0.025 * y / 640; 
}

void LineTracking::publishUpdatedLine(geometry_msgs::Point& pt1, geometry_msgs::Point& pt2) {

  // Make custom marker because we want it in from of reference of finger
  visualization_msgs::Marker line_marker;
  line_marker.header.frame_id = ATTACH_FRAME;
  line_marker.header.stamp = ros::Time::now();
  line_marker.ns = "LineObject";
  line_marker.type = visualization_msgs::Marker::LINE_STRIP;
  line_marker.action = visualization_msgs::Marker::ADD;
  line_marker.id = 0;
  line_marker.color = visuals_->visual_tools_->getColor(rvt::RED);

  // Base pose is identity
  line_marker.pose.orientation.w = 1.0;

  // Calculate scale
  line_marker.scale.x = 0.001; // Scale y and z not used

  // Add two points
  line_marker.points.push_back(pt1);  
  line_marker.points.push_back(pt2);

 
  visuals_->visual_tools_->publishMarker(line_marker);
  
}

void LineTracking::getToolDirection(const geometry_msgs::PoseStamped& center_point,  double theta) {
  // Visualize tool always pointing down away from gripper

  Eigen::Affine3d eigen_pose = visuals_->visual_tools_->convertPose(center_point.pose);

  if (theta > M_PI / 2.0) {
    theta -= M_PI;
    ROS_DEBUG_STREAM_NAMED("line_tracking","angle of line: " << theta << " (substracted 3.14)");    
  } else {
    ROS_DEBUG_STREAM_NAMED("line_tracking","angle of line: " << theta);
  }
  
  eigen_pose = eigen_pose * Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ());
  
  
  // Do some orientation magic

  geometry_msgs::PoseStamped pose_msg = center_point;
  pose_msg.pose = visuals_->visual_tools_->convertPose(eigen_pose);

  // Visualize arrow
  const double length = 0.1;
  const int id =1;
  visuals_->visual_tools_->publishArrow(pose_msg, rvt::GREY, rvt::SMALL, length, id);
}


} // end namespace
