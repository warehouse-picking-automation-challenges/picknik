/*********************************************************************
 * Software License Agreement
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
   Desc:   Tests publishing meshes
*/

//#ifndef PICKNIK_MAIN__MESH_PUBLISHER
//#define PICKNIK_MAIN__MESH_PUBLISHER

// ROS
#include <ros/ros.h>

// MoveIt
#include <picknik_main/namespaces.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

// Boost
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>

namespace fs = boost::filesystem;

namespace picknik_main
{
class MeshPublisher
{
  // For visualizing things in rviz
  mvt::MoveItVisualToolsPtr visual_tools_;

public:
  /**
   * \brief Constructor
   * \param verbose - run in debug mode
   */
  MeshPublisher(bool verbose)
    : verbose_(verbose)
    , test_rviz_visual_tools_(true)
    , test_planning_scene_(true)
  {
    // Visualizer
    visual_tools_.reset(new mvt::MoveItVisualTools("/world", "/amazon_shelf_markers"));
    visual_tools_->setPlanningSceneTopic("picknik_planning_scene");
    visual_tools_->loadPlanningSceneMonitor();
    ros::spinOnce();
    ros::Duration(1.0).sleep();

    visual_tools_->deleteAllMarkers();
    visual_tools_->removeAllCollisionObjects();
    // visual_tools_->triggerPlanningSceneUpdate();
    // visual_tools_->setManualSceneUpdating(true);
    // visual_tools_->enableBatchPublishing(true);

    ROS_INFO_STREAM_NAMED("mesh_publisher", "MeshPublisher Ready.");
  }

  bool publishAll(double y)
  {
    // TEST COLLISION MESHES ---------------------------

    std::string package_path = ros::package::getPath("picknik_main");
    fs::path target_dir(package_path + "/meshes/products/");

    fs::directory_iterator it(target_dir), eod;
    std::cout << "Directory: " << target_dir.string() << std::endl;

    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    pose.translation().y() = y;
    static std::size_t counter = 0;

    BOOST_FOREACH (fs::path const &p, std::make_pair(it, eod))
    {
      if (!ros::ok())
        return true;

      // ROS_DEBUG_STREAM_NAMED("temp","File: " << p.string());
      ROS_INFO_STREAM_NAMED("temp", "Processing mesh " << p.stem().string());

      processProductMeshes(p, pose);
      pose.translation().x() += 0.25;

      ros::Duration(0.1).sleep();

      std::cout << std::endl;
      std::cout << "Completed " << counter++ << " meshes" << std::endl;
    }

    return true;
  }

  bool processProductMeshes(fs::path const &p, Eigen::Affine3d pose)
  {
    // Collision Name
    static std::size_t counter = 0;
    std::string collision_name = "object" + boost::lexical_cast<std::string>(counter++);

    // Show Label
    Eigen::Affine3d text_pose = pose;
    text_pose.translation().z() += 0.1;
    text_pose.translation().y() += 0.1;
    // visual_tools_->publishText(text_pose, p.filename().string(), rvt::WHITE, rvt::SMALL, false);
    visual_tools_->publishText(text_pose, collision_name, rvt::WHITE, rvt::SMALL, false);

    // Show Product Axis
    // visual_tools_->publishAxis(pose);

    // Files
    fs::path display_file_name("recommended.dae");
    fs::path display_mesh_path = p / display_file_name;
    fs::path collision_file_name("collision.stl");
    fs::path collision_mesh_path = p / collision_file_name;

    // Show Product Display
    if (test_rviz_visual_tools_)
    {
      visual_tools_->publishMesh(pose, "file://" + display_mesh_path.string());
      visual_tools_->triggerBatchPublish();
    }

    // Show Product Collision
    if (test_planning_scene_)
    {
      Eigen::Affine3d collision_pose = pose;
      collision_pose.translation().y() += 0.2;
      visual_tools_->publishCollisionMesh(collision_pose, collision_name,
                                          "file://" + collision_mesh_path.string(), rvt::RAND);
      // visual_tools_->publishCollisionMesh(collision_pose, collision_name, "file://" +
      // display_mesh_path.string(), rvt::RAND);
      // visual_tools_->triggerPlanningSceneUpdate();
    }
    return true;
  }

private:
  // A shared node handle
  ros::NodeHandle nh_;

  // Show more visual and console output, with general slower run time.
  bool verbose_;

  bool test_rviz_visual_tools_;
  bool test_planning_scene_;

};  // end class

// Create boost pointers for this class
typedef boost::shared_ptr<MeshPublisher> MeshPublisherPtr;
typedef boost::shared_ptr<const MeshPublisher> MeshPublisherConstPtr;

}  // end namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mesh_publisher");
  ROS_INFO_STREAM_NAMED("main", "Starting MeshPublisher...");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Check for verbose flag
  bool verbose = false;
  if (argc > 1)
  {
    for (std::size_t i = 0; i < std::size_t(argc); ++i)
    {
      if (strcmp(argv[i], "--verbose") == 0)
      {
        ROS_INFO_STREAM_NAMED("main", "Running in VERBOSE mode (slower)");
        verbose = true;
      }
    }
  }

  picknik_main::MeshPublisher server(verbose);
  for (double y = 0; y < 5; y += 0.5)
  {
    if (!ros::ok())
      return 0;

    std::cout << std::endl;
    std::cout << "-------------------------------------------------------" << std::endl;
    std::cout << "Publishing y: " << y << std::endl;
    server.publishAll(y);
  }

  ROS_INFO_STREAM_NAMED("main", "Shutting down.");
  ros::shutdown();

  return 0;
}

//#endif
