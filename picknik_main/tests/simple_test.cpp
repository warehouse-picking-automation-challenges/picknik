#include <ros/ros.h>

#include <rviz_visual_tools/rviz_visual_tools.h>

void getSpiralPoses(std::vector<Eigen::Affine3d>& poses, Eigen::Affine3d& center_pose,
                    double distance)
{
  bool right_down = true;
  std::size_t step = 1;
  std::size_t i;
  const std::size_t iterations = 10;

  for (i = 0; i < iterations; ++i)
  {
    if (right_down)
    {
      // Right
      for (std::size_t j = 0; j < step; ++j)
      {
        center_pose.translation().x() += distance;
        poses.push_back(center_pose);
      }
      // Down
      for (std::size_t j = 0; j < step; ++j)
      {
        center_pose.translation().y() -= distance;
        poses.push_back(center_pose);
      }
    }
    else
    {
      // Left
      for (std::size_t j = 0; j < step; ++j)
      {
        center_pose.translation().x() -= distance;
        poses.push_back(center_pose);
      }
      // Up
      for (std::size_t j = 0; j < step; ++j)
      {
        center_pose.translation().y() += distance;
        poses.push_back(center_pose);
      }
    }
    step++;
    right_down = !right_down;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_test");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(4);
  spinner.start();

  // Load visual tools
  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
  visual_tools_.reset(
      new rviz_visual_tools::RvizVisualTools("world", "/picknik_main/tactile_feedback"));
  visual_tools_->deleteAllMarkers();

  // Create
  Eigen::Affine3d center_pose = Eigen::Affine3d::Identity();
  // center_pose.translation().x() = center_x;
  // center_pose.translation().y() = center_y;
  // center_pose.translation().z() = 0;
  double distance = 0.01;

  std::vector<Eigen::Affine3d> poses;
  getSpiralPoses(poses, center_pose, distance);

  // Publish
  for (std::size_t i = 0; i < poses.size(); ++i)
  {
    visual_tools_->publishSphere(poses[i], rviz_visual_tools::BLUE, rviz_visual_tools::LARGE);
    ros::Duration(0.01).sleep();
  }

  return 0;
}
