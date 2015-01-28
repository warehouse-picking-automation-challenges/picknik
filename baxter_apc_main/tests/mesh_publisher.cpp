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
   Desc:   Tests publishing meshes
*/

//#ifndef BAXTER_APC_MAIN__MESH_PUBLISHER
//#define BAXTER_APC_MAIN__MESH_PUBLISHER

// ROS
#include <ros/ros.h>

// MoveIt
#include <moveit_visual_tools/moveit_visual_tools.h>

// Boost
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>


namespace baxter_apc_main
{

class MeshPublisher
{
  // For visualizing things in rviz
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

public:

  /**
   * \brief Constructor
   * \param verbose - run in debug mode
   */
  MeshPublisher(bool verbose)
    : verbose_(verbose)
  {
    // Viusalizer
    visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("/world", "/amazon_shelf_markers"));
    visual_tools_->setPlanningSceneTopic("baxter_apc_planning_scene");
    visual_tools_->loadPlanningSceneMonitor();
    ros::spinOnce();
    ros::Duration(1.0).sleep();
    visual_tools_->removeAllCollisionObjects();
    visual_tools_->triggerPlanningSceneUpdate();
    visual_tools_->setManualSceneUpdating(true);

    //static const std::string file_name = "file://home/dave/ros/ws_baxter/src/cu_amazon/mesh_models/amazon_picking_challenge/oreo_mega_stuf/meshes/tsdf.stl";
    static const std::string file_name = "file://home/dave/ros/ws_baxter/src/cu_amazon/mesh_models/amazon_picking_challenge/";
    //static const std::string file_name = "file://home/dave/ros/ws_moveit/src/moveit_visual_tools/resources/demo_mesh.stl";
    //static const std::string file_name = "file://home/dave/ros/ws_moveit/src/moveit_visual_tools/resources/tsdf.stl";

    namespace fs = boost::filesystem;

    fs::path targetDir("/home/dave/ros/ws_baxter/src/cu_amazon/mesh_models/amazon_picking_challenge/");

    fs::recursive_directory_iterator it(targetDir), eod;
    std::cout << "Dir: " << targetDir.string() << std::endl;

    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    std::size_t counter = 1;

    BOOST_FOREACH(fs::path const &p, std::make_pair(it, eod))
    {
      //std::cout << "File: " << p.string() << std::endl;
      if(p.extension().string() == ".stl" && is_regular_file(p))
      {
        if (!ros::ok())
          break;

        // Show product
        pose.translation().x() += 0.2;
        //std::cout << "Pose: " << visual_tools_->convertPose(pose) << std::endl;
        if (visual_tools_->publishCollisionMesh(pose, "object"+boost::lexical_cast<std::string>(counter++), 
                                                "file://" + p.string(), rviz_visual_tools::RAND))
        {
          visual_tools_->publishText(pose, p.filename().string(), rviz_visual_tools::WHITE, rviz_visual_tools::SMALL, false);
          visual_tools_->publishAxis(pose);

          ROS_DEBUG_STREAM_NAMED("apc_manager","File: " << p.filename().string());
          std::cout << std::endl;std::cout << std::endl;
        }
      }
      if (counter % 10 == 0)
      {
        visual_tools_->triggerPlanningSceneUpdate();
        ros::Duration(1).sleep();
      }
    }




    ROS_INFO_STREAM_NAMED("mesh_publisher","MeshPublisher Ready.");
  }

  /**
   * \brief Destructor
   */
  ~MeshPublisher()
  {

  }

private:

  // A shared node handle
  ros::NodeHandle nh_;

  // Show more visual and console output, with general slower run time.
  bool verbose_;

}; // end class

// Create boost pointers for this class
typedef boost::shared_ptr<MeshPublisher> MeshPublisherPtr;
typedef boost::shared_ptr<const MeshPublisher> MeshPublisherConstPtr;

} // end namespace

int main(int argc, char** argv)
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
    for (std::size_t i = 0; i < argc; ++i)
    {
      if (strcmp(argv[i], "--verbose") == 0)
      {
        ROS_INFO_STREAM_NAMED("main","Running in VERBOSE mode (slower)");
        verbose = true;
      }
    }
  }

  baxter_apc_main::MeshPublisher server(verbose);

  ROS_INFO_STREAM_NAMED("main", "Shutting down.");
  ros::shutdown();

  return 0;
}

//#endif
