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

// PickNik
#include <picknik_main/perception_interface.h>
#include <picknik_main/namespaces.h>

// ROS
#include <tf_conversions/tf_eigen.h>

// PickNik Msgs
#include <picknik_msgs/StopPerception.h>

// Parameter loading
#include <rviz_visual_tools/ros_param_utilities.h>

namespace picknik_main
{

PerceptionInterface::PerceptionInterface(bool verbose, VisualsPtr visuals, ShelfObjectPtr shelf, ManipulationDataPtr config,
                                         boost::shared_ptr<tf::TransformListener> tf, ros::NodeHandle nh)
  : verbose_(verbose)
  , visuals_(visuals)
  , shelf_(shelf)
  , config_(config)
  , tf_(tf)
  , nh_(nh)
  , find_objects_action_(PERCEPTION_TOPIC)
  , is_processing_perception_(false)
{
  // Load ROS publisher
  stop_perception_client_ = nh_.serviceClient<picknik_msgs::StopPerception>( "/perception/stop_perception" );
  reset_perception_client_ = nh_.serviceClient<picknik_msgs::StopPerception>( "/perception/reset_perception" );

  // Load caamera intrinsics
  const std::string parent_name = "perception_interface"; // for namespacing logging messages
  rvt::getDoubleParameter(parent_name, nh, "camera_intrinsics/fx", camera_fx_);
  rvt::getDoubleParameter(parent_name, nh, "camera_intrinsics/fx", camera_fy_);
  rvt::getDoubleParameter(parent_name, nh, "camera_intrinsics/fx", camera_cx_);
  rvt::getDoubleParameter(parent_name, nh, "camera_intrinsics/fx", camera_cy_);
  rvt::getDoubleParameter(parent_name, nh, "camera_intrinsics/min_depth", camera_min_depth_);

  ROS_INFO_STREAM_NAMED("perception_interface","PerceptionInterface Ready.");
}

bool PerceptionInterface::isPerceptionReady()
{
  // Do initial check before showing warning
  if (!find_objects_action_.waitForServer(ros::Duration(0.1)))
  {
    ROS_WARN_STREAM_NAMED("perception_interface","Waiting for object perception server on topic " << PERCEPTION_TOPIC);

    find_objects_action_.waitForServer();
  }

  return true;
}

bool PerceptionInterface::startPerception(ProductObjectPtr& product, BinObjectPtr& bin)
{
  // Setup goal
  ROS_INFO_STREAM_NAMED("perception_interface","Communicating with perception pipeline");
  picknik_msgs::FindObjectsGoal find_object_goal;
  find_object_goal.desired_object_name = product->getName();

  // Send region of interest
  const Eigen::Affine3d bin_centroid = picknik_main::transform(bin->getCentroid(), shelf_->getBottomRight());
  find_object_goal.bin_name = bin->getName();
  find_object_goal.bin_centroid = visuals_->visual_tools_->convertPose(bin_centroid);
  find_object_goal.bin_dimensions.type = shape_msgs::SolidPrimitive::BOX;
  find_object_goal.bin_dimensions.dimensions.resize(3);
  find_object_goal.bin_dimensions.dimensions[shape_msgs::SolidPrimitive::BOX_X] = bin->getDepth();
  find_object_goal.bin_dimensions.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = bin->getWidth();
  find_object_goal.bin_dimensions.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = bin->getHeight();

  // Get all of the products in the bin
  bin->getProducts(find_object_goal.expected_objects_names);

  // Make sure perception is reset
  if (false)
  {
    picknik_msgs::StopPerception srv;
    srv.request.stop = true;
    reset_perception_client_.call(srv);

    if (!srv.response.stopped)
    {
      ROS_ERROR_STREAM_NAMED("perception_interface","Unable to reset perception pipeline!");
      return false;
    }
  }

  // Send request
  find_objects_action_.sendGoal(find_object_goal);

  // Change state of interface
  is_processing_perception_ = true;

  return true;
}


bool PerceptionInterface::endPerception(ProductObjectPtr& product, BinObjectPtr& bin, bool fake_perception)
{
  if (!is_processing_perception_)
  {
    ROS_ERROR_STREAM_NAMED("perception_interfac","Attempt was made to end perception when it has not been started");
  }

  // Tell the perception pipeline we are done moving the camera
  bool use_stop_command = true;
  if (use_stop_command)
  {
    ROS_INFO_STREAM_NAMED("perception_interface","Sending stop command to perception server");
    picknik_msgs::StopPerception srv;
    srv.request.stop = true;
    stop_perception_client_.call(srv);

    if (!srv.response.stopped)
    {
      ROS_ERROR_STREAM_NAMED("perception_interface","Unable to stop perception pipeline!");
      return false;
    }
  }
  else
    ROS_WARN_STREAM_NAMED("pereption_interface","Not using stop command!");

  ROS_INFO_STREAM_NAMED("perception_interface","Waiting for response from perception server");

  // Wait for the action to return with product pose
  double timeout = 25;
  if (!find_objects_action_.waitForResult(ros::Duration(timeout)))
  {
    ROS_ERROR_STREAM_NAMED("perception_interface","Percetion action did not finish before the time out.");
    is_processing_perception_ = false;
    return false;
  }

  // Get goal state
  actionlib::SimpleClientGoalState goal_state = find_objects_action_.getState();
  ROS_INFO_STREAM_NAMED("perception_interface","Perception action finished: " << goal_state.toString());

  // Get result
  picknik_msgs::FindObjectsResultConstPtr perception_result = find_objects_action_.getResult();
  //ROS_DEBUG_STREAM_NAMED("perception_interface","Perception result:\n" << *perception_result);

  if (!perception_result->succeeded)
  {
    ROS_ERROR_STREAM_NAMED("perception_interface","Perception action server result messages says succeeded = false, aka it found no products");
    is_processing_perception_ = false;
    return false;
  }

  // -----------------------------------------------------------------------------------------------
  // Check bounds and update planning scene
  if (!processPerceptionResults(perception_result, product, bin))
  {
    ROS_ERROR_STREAM_NAMED("perception_interface","Failed to process object poses");
    is_processing_perception_ = false;
    return false;
  }

  ROS_INFO_STREAM_NAMED("perception_interface","Successfully processed perception results");
  is_processing_perception_ = false;
  return true;
}

bool PerceptionInterface::processPerceptionResults(picknik_msgs::FindObjectsResultConstPtr result,
                                                   ProductObjectPtr& product, BinObjectPtr& bin)
{
  std::cout << std::endl;
  std::cout << "-------------------------------------------------------" << std::endl;
  ROS_INFO_STREAM_NAMED("perception_interface","Processing perception results");

  // Error check
  if (result->found_objects.size() != bin->getProducts().size())
  {
    ROS_WARN_STREAM_NAMED("perception_interface","The list of found objects does not have the same size as the expected bin contents");
  }
  if (result->found_objects.empty())
  {
    ROS_ERROR_STREAM_NAMED("percetion","No objects found");
    return false;
  }

  // Check that each product in our bin was found by perception
  // Also check for our desired product
  std::vector<std::string> missing_products;
  bin->getProducts(missing_products);
  bool found_desired_product = false;
  for (std::size_t i = 0; i < result->found_objects.size(); ++i)
  {
    ROS_INFO_STREAM_NAMED("perception_interface","Perception found " << result->found_objects[i].object_name);
    bool was_requested = false;

    // Remove this product from the missing products vector
    for (std::vector<std::string>::iterator product_it = missing_products.begin(); product_it != missing_products.end(); )
    {
      if (*product_it == result->found_objects[i].object_name)
      {
        //std::cout << i << " erasing " << result->found_objects[i].object_name << std::endl;
        product_it = missing_products.erase(product_it);
        was_requested = true;
      }
      else
      {
        ++product_it;
      }
    }

    // Check if this is the desired product
    if (result->found_objects[i].object_name == product->getName())
    {
      found_desired_product = true;
    }

    // Check that we requested this product
    if (!was_requested)
    {
      ROS_WARN_STREAM_NAMED("perception_interface","Found product " << result->found_objects[i].object_name
                            << " that was not requested or suppose to be in " << bin->getName());
    }
  }

  // Show missing objects
  if (missing_products.size())
  {
    ROS_WARN_STREAM_NAMED("perception_interface","Not all products were found from desired set");
    ROS_WARN_STREAM_NAMED("perception_interface","  Missing:");
    for (std::size_t i = 0; i < missing_products.size(); ++i)
    {
      ROS_WARN_STREAM_NAMED("perception_interface","    - " << missing_products[i]);
    }
  }

  // Error check
  if (!found_desired_product)
  {
    ROS_ERROR_STREAM_NAMED("perception_interface","The desired product was not returned in the list of found objects");
    return false;
  }

  if (config_->isEnabled("ddtr_mode"))
  {
    // LuMa Mode
    return processPerceptionResultsDDTR(result, product, bin);
  }
  else
  {
    // Andy Mode
    return processPerceptionResultsPCL(result, product, bin);
  }
}

bool PerceptionInterface::processPerceptionResultsDDTR(picknik_msgs::FindObjectsResultConstPtr result,
                                                       ProductObjectPtr& product, BinObjectPtr& bin)
{
  // Get camera position ------------------------------------------------------------------------
  std::string frame_id = result->found_objects.front().object_pose.header.frame_id;
  Eigen::Affine3d world_to_camera = Eigen::Affine3d::Identity();

  // Set camera pose to that from lu's code
  geometry_msgs::PoseStamped recieved_world_to_camera = result->found_objects.front().camera_pose;
  if (recieved_world_to_camera.pose.position.x == 0 &&
      recieved_world_to_camera.pose.position.y == 0 &&
      recieved_world_to_camera.pose.position.z == 0)
  {
    ROS_INFO_STREAM_NAMED("perception_interface","No camera pose returned from perception server, getting one from tf");
    ros::Time time_stamp;
    getTFTransform(world_to_camera, time_stamp, frame_id);
  }
  else
  {
    world_to_camera = visuals_->visual_tools_->convertPose(recieved_world_to_camera.pose);
  }

  // Debug
  if (visuals_->isEnabled("show_world_to_camera_pose"))
    visuals_->visual_tools_->publishAxisLabeled(world_to_camera, "world_to_camera");

  // Tweak transform
  Eigen::Affine3d object_pose_offset = Eigen::Affine3d::Identity();
  //getHackOffsetPose(object_pose_offset, time_stamp);
  bool use_camera_hack_offset = config_->isEnabled("use_camera_hack_offset");
  if (use_camera_hack_offset)
  {
    ros::Time time_stamp;
    getTFTransform(object_pose_offset, time_stamp, "object_offset_hack");
  }

  // Get bin location
  const Eigen::Affine3d& world_to_bin = picknik_main::transform(bin->getBottomRight(), shelf_->getBottomRight());

  // Debug
  if (visuals_->isEnabled("show_world_to_bin_pose"))
    visuals_->visual_tools_->publishAxisLabeled(world_to_bin, "world_to_bin");

  // Process each product
  for (std::size_t i = 0; i < result->found_objects.size(); ++i)
  {
    const picknik_msgs::FoundObject& found_object = result->found_objects[i];

    // Error check
    if (frame_id != found_object.object_pose.header.frame_id)
    {
      ROS_ERROR_STREAM_NAMED("perception_interface","frame_id has changed between found objects in same perception results message");
    }

    // Get object's transform
    Eigen::Affine3d camera_to_object = visuals_->visual_tools_->convertPose(found_object.object_pose.pose);

    // Debug
    if (visuals_->isEnabled("show_raw_object_pose"))
      visuals_->visual_tools_->publishArrow(found_object.object_pose);

    // Convert to world frame
    const Eigen::Affine3d world_to_object = world_to_camera * camera_to_object;

    // Debug
    if (visuals_->isEnabled("show_world_to_object_pose"))
      visuals_->visual_tools_->publishAxisLabeled(world_to_object, found_object.object_name);

    // Convert to pose of bin
    Eigen::Affine3d bin_to_object = world_to_bin.inverse() * world_to_object;

    // Apply small hack offset
    bin_to_object = object_pose_offset * bin_to_object;

    std::cout << std::endl;
    std::cout << "=============== Found Object =============== " << std::endl;
    std::cout << "object_name:        " << found_object.object_name << std::endl;
    std::cout << "has mesh:           " << ((found_object.bounding_mesh.triangles.empty() || found_object.bounding_mesh.vertices.empty()) ? "NO" : "YES") << std::endl;
    std::cout << "original:           "; printTransform(camera_to_object);
    std::cout << "world_to_object:    "; printTransform(world_to_object);
    std::cout << "bin_to_object:      "; printTransform(bin_to_object);
    if (use_camera_hack_offset)
      std::cout << "object_pose_offset: "; printTransform(object_pose_offset);
    std::cout << std::endl;

    // Check bounds
    if (!checkBounds(bin_to_object, bin, product))
    {
      ROS_ERROR_STREAM_NAMED("perception_interface","Bounds failed");
      return false;
    }

    // Save to the product's property
    product->setCentroid(bin_to_object);
    product->setMeshCentroid(bin_to_object);

    // Show new mesh if possible
    const shape_msgs::Mesh& mesh = found_object.bounding_mesh;
    bool has_new_mesh = false;

    ROS_DEBUG_STREAM_NAMED("perception_interface","Recieved mesh with " << mesh.triangles.size() << " triangles and "
                           << mesh.vertices.size() << " vertices");
    if (! (mesh.triangles.empty() || mesh.vertices.empty()))
    {
      ROS_INFO_STREAM_NAMED("perception_interface","Setting new bounding mesh");
      product->setCollisionMesh(mesh);

      static std::size_t mesh_id = 0;
      const std::string& high_res_mesh_path = config_->package_path_ + "/meshes/detected/current" +
        boost::lexical_cast<std::string>(mesh_id++) + ".stl";

      product->writeCollisionBody(high_res_mesh_path);
      std::cout << "previous high res mesh path: " << product->getHighResMeshPath() << std::endl;
      product->setHighResMeshPath("file://" + high_res_mesh_path);
      std::cout << "new high res mesh path: " << product->getHighResMeshPath() << std::endl;
      has_new_mesh = true;
    }
    else
      ROS_ERROR_STREAM_NAMED("perception_interface","No mesh provided");

    // So that bounding boxes don't build up
    //visuals_->visual_tools_->deleteAllMarkers();

    // Update the bounding box
    updateBoundingMesh(product, bin);

    // Visualize bounding box
    // the mesh is saved in the bin coordinate system by pcl_perception_server
    Eigen::Affine3d world_to_bin = shelf_->getBottomRight() * bin->getBottomRight();
    product->visualizeWireframe(world_to_bin, rvt::LIME_GREEN);

    // Visualize bounding box in high res display
    product->visualizeHighResWireframe(world_to_bin, rvt::LIME_GREEN);

    // Show the new mesh
    if (has_new_mesh)
    {
      product->createCollisionBodies(world_to_bin);

      product->visualizeHighRes(world_to_bin);
    }

  } // for each found product

  visuals_->visual_tools_->triggerPlanningSceneUpdate();

  // Allow mesh to display
  ros::spinOnce();
  ros::Duration(0.1).sleep();

  return true;
}

bool PerceptionInterface::processPerceptionResultsPCL(picknik_msgs::FindObjectsResultConstPtr result,
                                                       ProductObjectPtr& product, BinObjectPtr& bin)
{
  // Get camera position ------------------------------------------------------------------------
  std::string frame_id = result->found_objects.front().object_pose.header.frame_id;
  ROS_INFO_STREAM_NAMED("perception_interface","Frame id is " << frame_id);

  // Get bin location
  const Eigen::Affine3d& world_to_bin = picknik_main::transform(bin->getBottomRight(), shelf_->getBottomRight());

  // Debug
  if (visuals_->isEnabled("show_world_to_bin_pose"))
    visuals_->visual_tools_->publishAxisLabeled(world_to_bin, "world_to_bin");

  // Process each product
  for (std::size_t i = 0; i < result->found_objects.size(); ++i)
  {
    const picknik_msgs::FoundObject& found_object = result->found_objects[i];

    // Error check
    if (frame_id != found_object.object_pose.header.frame_id)
    {
      ROS_WARN_STREAM_NAMED("perception_interface","frame_id has changed between found objects in same perception results message");
    }

    // Get object's transform
    Eigen::Affine3d bin_to_object = visuals_->visual_tools_->convertPose(found_object.object_pose.pose);

    // Debug
    if (visuals_->isEnabled("show_raw_object_pose"))
      visuals_->visual_tools_->publishArrow(found_object.object_pose);

    // Convert to world frame
    const Eigen::Affine3d world_to_object = world_to_bin * bin_to_object;

    // Debug
    if (visuals_->isEnabled("show_world_to_object_pose"))
      visuals_->visual_tools_->publishAxisLabeled(world_to_object, found_object.object_name);

    std::cout << std::endl;
    std::cout << "=============== Found Object =============== " << std::endl;
    std::cout << "object_name:        " << found_object.object_name << std::endl;
    std::cout << "has mesh:           " << ((found_object.bounding_mesh.triangles.empty() || found_object.bounding_mesh.vertices.empty()) ? "NO" : "YES") << std::endl;
    std::cout << "original:           "; printTransform(bin_to_object);
    std::cout << "world_to_object:    "; printTransform(world_to_object);
    std::cout << std::endl;

    // Check bounds
    if (!checkBounds(bin_to_object, bin, product))
    {
      ROS_ERROR_STREAM_NAMED("perception_interface","Bounds failed");
      return false;
    }

    // Save to the product's property
    product->setCentroid(bin_to_object);
    product->setMeshCentroid(bin_to_object);

    // Show new mesh if possible
    const shape_msgs::Mesh& mesh = found_object.bounding_mesh;
    bool has_new_mesh = false;

    ROS_DEBUG_STREAM_NAMED("perception_interface","Recieved mesh with " << mesh.triangles.size() << " triangles and "
                           << mesh.vertices.size() << " vertices");
    if (! (mesh.triangles.empty() || mesh.vertices.empty()))
    {
      ROS_INFO_STREAM_NAMED("perception_interface","Setting new bounding mesh");
      product->setCollisionMesh(mesh);

      static std::size_t mesh_id = 0;
      const std::string& high_res_mesh_path = config_->package_path_ + "/meshes/detected/current" +
        boost::lexical_cast<std::string>(mesh_id++) + ".stl";

      // product->writeCollisionBody(high_res_mesh_path);
      // std::cout << "previous high res mesh path: " << product->getHighResMeshPath() << std::endl;
      // product->setHighResMeshPath("file://" + high_res_mesh_path);
      // std::cout << "new high res mesh path: " << product->getHighResMeshPath() << std::endl;
      has_new_mesh = true;
    }
    else
      ROS_ERROR_STREAM_NAMED("perception_interface","No mesh provided");

    // So that bounding boxes don't build up
    //visuals_->visual_tools_->deleteAllMarkers();

    // Update the bounding box
    updateBoundingMesh(product, bin);

    // Visualize bounding box
    // the mesh is saved in the bin coordinate system by pcl_perception_server
    //Eigen::Affine3d world_to_bin = shelf_->getBottomRight() * bin->getBottomRight();
    product->visualizeWireframe(world_to_bin, rvt::LIME_GREEN);

    // Visualize bounding box in high res display
    product->visualizeHighResWireframe(world_to_bin, rvt::LIME_GREEN);

    // Show the new mesh
    if (has_new_mesh)
    {
      product->createCollisionBodies(world_to_bin);

      product->visualizeHighRes(world_to_bin);
    }

  } // for each found product

  visuals_->visual_tools_->triggerPlanningSceneUpdate();

  // Allow mesh to display
  ros::spinOnce();
  ros::Duration(0.1).sleep();

  return true;
}

bool PerceptionInterface::getTFTransform(Eigen::Affine3d& world_to_frame, ros::Time& time_stamp,
                                         const std::string& frame_id)
{
  return getTFTransform(world_to_frame, time_stamp, config_->world_frame_, frame_id);
}

bool PerceptionInterface::getTFTransform(Eigen::Affine3d& world_to_frame, ros::Time& time_stamp,
                                         const std::string& parent_frame_id, const std::string& frame_id)
{
  tf::StampedTransform tf_transform;

  try
  {
    // Wait to make sure a transform message has arrived
    tf_->waitForTransform(parent_frame_id, frame_id, ros::Time(0), ros::Duration(1));
    // Get latest transform available
    tf_->lookupTransform(parent_frame_id, frame_id, ros::Time(0), tf_transform);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR_STREAM_NAMED("perception_interface", "TF error: " << ex.what());
    return false;
  }

  // Copy results
  tf::transformTFToEigen(tf_transform, world_to_frame);
  time_stamp = tf_transform.stamp_;
  return true;
}

bool PerceptionInterface::publishCameraFrame(Eigen::Affine3d world_to_camera)
{
  // get the pose of the top left point which depth is 0.5 meters away
  Eigen::Vector3d top_left, top_right, bottom_left, bottom_right;
  // top left (0,0)
  top_left << -camera_min_depth_ * (camera_cy_ - 0) / camera_fx_ , -camera_min_depth_ * (camera_cx_ - 0) / camera_fy_ , camera_min_depth_;
  // top right (640,0)
  top_right << -camera_min_depth_ * (camera_cy_ - 640) / camera_fx_ , -camera_min_depth_ * (camera_cx_ - 0) / camera_fy_ , camera_min_depth_;
  // bot right (640.480)
  bottom_right << -camera_min_depth_ * (camera_cy_ - 640) / camera_fx_ , -camera_min_depth_ * (camera_cx_ - 480) / camera_fy_ , camera_min_depth_;
  // bot left (0,480)
  bottom_left << -camera_min_depth_ * (camera_cy_ - 0) / camera_fx_ , -camera_min_depth_ * (camera_cx_ - 480) / camera_fy_ , camera_min_depth_;

  // const double distance_from_camera = 0;
  // Eigen::Affine3d camera_view_finder_offset = Eigen::Affine3d::Identity();
  // camera_view_finder_offset.translation().x() = distance_from_camera;
  // Eigen::Affine3d camera_view_finder = camera_view_finder_offset * world_to_camera;

  Eigen::Affine3d camera_view_finder = Eigen::Affine3d::Identity(); //world_to_camera;

  // camera_view_finder = camera_view_finder
  //   * Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY())
  //   * Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitZ());

  visuals_->visual_tools_->publishWireframeRectangle(camera_view_finder,
                                                     top_left, top_right, bottom_right, bottom_left, rvt::PINK, rvt::SMALL);


  // visuals_->visual_tools_->publishWireframeRectangle(camera_view_finder, height, width, rvt::PINK, rvt::SMALL);
  return true;
}

bool PerceptionInterface::updateBoundingMesh(ProductObjectPtr &product, BinObjectPtr &bin)
{
  ROS_DEBUG_STREAM_NAMED("perception_interface","Updating bounding mesh for product " << product->getName());
  bool verbose_bounding_box = visuals_->isEnabled("verbose_bounding_box");

  /***** GET BOUNDING BOX FROM MESH *****/
  // pcl_perception_server saves the mesh in the BIN pose.
  // drop_pose is set to identity to match the BIN pose.
  Eigen::Affine3d world_to_bin = shelf_->getBottomRight() * bin->getBottomRight();
  visuals_->visual_tools_->publishAxisLabeled(world_to_bin, "WORLD_TO_BIN");

  ROS_DEBUG_STREAM_NAMED("perception_interface","world_to_bin_corner = \n" << world_to_bin.translation() 
                         << "\n" << world_to_bin.rotation());

  if (config_->isEnabled("dropping_bounding_box"))
  {
    ROS_DEBUG_STREAM_NAMED("perception_interface","Dropping points to plane for " << product->getName());

    bounding_box_.drop_pose_ = Eigen::Affine3d::Identity();
    bounding_box_.drop_plane_ = bounding_box::XY;
    bounding_box_.drop_points_ = true;

    visuals_->visual_tools_->publishAxisLabeled(world_to_bin * bounding_box_.drop_pose_, "BOUNDING_BOX_DROP_POSE");
  }

  // Transform from the mesh to the drop pose
  // NOTE: drop pose is in the BIN frame
  // NOTE: mesh is in the BIN frame
  Eigen::Affine3d points_to_drop_pose = Eigen::Affine3d::Identity(); // no transform needed

  // Get bounding box output
  Eigen::Affine3d bin_to_bounding_box; // is actually bin_to_bounding in our case since mesh is saved in BIN frame
  double depth, width, height;
  if (!bounding_box_.getBodyAlignedBoundingBox(product->getCollisionMesh(), points_to_drop_pose, bin_to_bounding_box, 
                                               depth, width, height))
  {
    ROS_ERROR_STREAM_NAMED("manipulation","Failed to get bounding box");
    return false;
  }

  product->setDepth(depth);
  product->setWidth(width);
  product->setHeight(height);

  visuals_->visual_tools_->publishAxisLabeled(world_to_bin * bin_to_bounding_box, "WORLD_TO_MESH");
  
  product->setCentroid(bin_to_bounding_box);

  // Debug
  if (verbose_bounding_box)
  {
    std::cout << std::endl;
    std::cout << "After getBoundingingBoxFromMesh(): " << std::endl;
    std::cout << "  Cuboid Pose: "; printTransform(product->getCentroid());
    std::cout << "  Height: " << product->getHeight() << std::endl;
    std::cout << "  Depth: " << product->getDepth() << std::endl;
    std::cout << "  Width: " << product->getWidth() << std::endl;
    std::cout << "-------------------------------------------------------" << std::endl;
  }

  return true;
}

bool PerceptionInterface::checkBounds(const Eigen::Affine3d &bin_to_object, BinObjectPtr& bin,
                                      ProductObjectPtr& product)
{
  if (bin_to_object.translation().x() < 0 || bin_to_object.translation().x() > bin->getDepth() ||
      bin_to_object.translation().y() < 0 || bin_to_object.translation().y() > bin->getWidth() ||
      bin_to_object.translation().z() < 0 || bin_to_object.translation().z() > bin->getHeight())
  {
    if (bin_to_object.translation().x() < -PRODUCT_POSE_WITHIN_BIN_TOLERANCE ||
        bin_to_object.translation().x() > bin->getDepth() + PRODUCT_POSE_WITHIN_BIN_TOLERANCE ||
        bin_to_object.translation().y() < -PRODUCT_POSE_WITHIN_BIN_TOLERANCE ||
        bin_to_object.translation().y() > bin->getWidth() + PRODUCT_POSE_WITHIN_BIN_TOLERANCE||
        bin_to_object.translation().z() < -PRODUCT_POSE_WITHIN_BIN_TOLERANCE ||
        bin_to_object.translation().z() > bin->getHeight() + PRODUCT_POSE_WITHIN_BIN_TOLERANCE)
    {
      // Should it continue?
      ROS_ERROR_STREAM_NAMED("perception_interface","Product " << product->getName()
                             << " has a reported pose from the perception pipline that is outside the tolerance of "
                             << PRODUCT_POSE_WITHIN_BIN_TOLERANCE);
      ROS_ERROR_STREAM_NAMED("perception_interface","Pose:\n" << visuals_->visual_tools_->convertPose(bin_to_object));
    }
    else
    {
      // Its within error tolerance, just warn
      ROS_WARN_STREAM_NAMED("perception_interface","Product " << product->getName()
                            << " has a reported pose from the perception pipline that is outside the bounds of the bin shape");
      ROS_WARN_STREAM_NAMED("perception_interface","Pose:\n" << visuals_->visual_tools_->convertPose(bin_to_object));
    }
  }
  return true;
}

} // namespace
