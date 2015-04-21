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
   Desc:   Interface between the perception pipeline and the manipulation pipeline
*/

#ifndef PICKNIK_MAIN__PERCEPTION_INTERFACE
#define PICKNIK_MAIN__PERCEPTION_INTERFACE

// ROS
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <tf/transform_listener.h>

// Picknik
#include <picknik_main/namespaces.h>
#include <picknik_main/shelf.h>
#include <picknik_main/visuals.h>

// Picknik Msgs
#include <picknik_msgs/FindObjectsAction.h>

namespace picknik_main
{

static const double PRODUCT_POSE_WITHIN_BIN_TOLERANCE = 0.2; // throw an error if pose is beyond this amount
static const std::string PERCEPTION_TOPIC = "perception/recognize_objects";

class PerceptionInterface
{
public:

  /**
   * \brief Constructor
   * \param verbose - run in debug mode
   */
  PerceptionInterface(bool verbose, VisualsPtr visuals, ShelfObjectPtr shelf, ManipulationDataPtr config, 
                  boost::shared_ptr<tf::TransformListener> tf, ros::NodeHandle nh);

  /**
   * \brief Check if perception is ready

   * \return true if ready
   */
  bool isPerceptionReady();
  
  /**
   * \brief Call perception pipeline to start looking around
   * \return true on success
   */
  bool startPerception(ProductObjectPtr& product, BinObjectPtr& bin);

  /**
   * \brief Get result from actionserver and process
   * \return true on success
   */
  bool endPerception(ProductObjectPtr& product, BinObjectPtr& bin);

  /**
   * \brief Update the poses, and optionally the mesh, of the products in a bin
   * \return false if outside the error tolerance bounds of a pose within a bin
   */
  bool processPerceptionResults(picknik_msgs::FindObjectsResultConstPtr result,
                                ProductObjectPtr& product, BinObjectPtr& bin);
  
  /**
   * \brief Get the latest location of the camera on the robot from ROS
   * \param world_to_camera 4x4 matrix to fill in with transpose
   * \param time_stamp - the time that the arm was in this location
   * \return true on success
   */
  bool getCameraPose(Eigen::Affine3d& world_to_camera, ros::Time& time_stamp);

  /**
   * \brief Display a visualization of a camera view frame
   * \return true on success
   */
  bool publishCameraFrame(Eigen::Affine3d camera_to_world);

  bool convertFrameCVToROS(const Eigen::Affine3d& cv_frame, Eigen::Affine3d& ros_frame);

private:

  // Show more visual and console output, with general slower run time.
  bool verbose_;

  // Visualization classes
  VisualsPtr visuals_;

  // Contents
  ShelfObjectPtr shelf_;

  // Robot-sepcific data for the APC
  ManipulationDataPtr config_;

  // Perception pipeline communication
  actionlib::SimpleActionClient<picknik_msgs::FindObjectsAction> find_objects_action_;

  // TF Listener
  boost::shared_ptr<tf::TransformListener> tf_;

  // ROS Comm
  ros::NodeHandle nh_;

  // Tell the perception pipeline we are done moving the camera
  ros::Publisher stop_perception_pub_;

  // Perception processing has started
  bool is_processing_perception_;

  // Camera intrinsics
  double camera_fx_; 
  double camera_fy_; 
  double camera_cx_; 
  double camera_cy_; 
  double camera_min_depth_; 
  
  
}; // end class

// Create boost pointers for this class
typedef boost::shared_ptr<PerceptionInterface> PerceptionInterfacePtr;
typedef boost::shared_ptr<const PerceptionInterface> PerceptionInterfaceConstPtr;

} // end namespace

#endif
