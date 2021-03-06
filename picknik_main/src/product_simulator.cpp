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
   Desc:   Simulates placing objects in a bin
*/

// Picknik
#include <picknik_main/product_simulator.h>

namespace picknik_main
{
ProductSimulator::ProductSimulator(
    bool verbose, VisualsPtr visuals,
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor)
  : verbose_(verbose)
  , visuals_(visuals)
  , planning_scene_monitor_(planning_scene_monitor)
  , secondary_scene_(planning_scene_monitor->getPlanningScene()->getCurrentState().getRobotModel())
{
  ROS_INFO_STREAM_NAMED("product_simulator", "ProductSimulator Ready.");
}

bool ProductSimulator::generateRandomProductPoses(ShelfObjectPtr shelf,
                                                  PerceptionInterfacePtr percepetion_interface)
{
  ROS_WARN_STREAM_NAMED("product_simulator", "Generating random product poses (slightly slow)");

  // Setup random pose generator
  Eigen::Affine3d pose;
  Eigen::Affine3d world_to_bin;
  rviz_visual_tools::RandomPoseBounds pose_bounds;
  pose_bounds.x_min_ = RAND_PADDING;
  pose_bounds.y_min_ = RAND_PADDING;
  pose_bounds.z_min_ = RAND_PADDING;
  pose_bounds.x_max_ =
      shelf->bin_depth_ * 0.5 - RAND_PADDING;  // TODO - we are restraining bin depth of object
  pose_bounds.y_max_ = shelf->bin_middle_width_ - RAND_PADDING;
  pose_bounds.z_max_ = shelf->bin_tall_height_ - RAND_PADDING;

  // Show empty shelf in primary scene
  visuals_->visual_tools_->removeAllCollisionObjects();
  bool just_frame = true;
  bool show_all_products = false;
  shelf->createCollisionBodies("", just_frame, show_all_products);  // only show the frame
  visuals_->visual_tools_->triggerPlanningSceneUpdate();

  // Show empty shelf in Rviz DISPLAY
  bool show_random_generated_poses = visuals_->isEnabled("show_random_generated_poses");

  // Loop through each bin
  for (BinObjectMap::const_iterator bin_it = shelf->getBins().begin();
       bin_it != shelf->getBins().end(); bin_it++)
  {
    if (!ros::ok())
      return false;

    BinObjectPtr bin = bin_it->second;
    ROS_DEBUG_STREAM_NAMED("product_simulator", "Updating products in " << bin->getName());

    // Calculate once the bin transform
    world_to_bin = transform(bin->getBottomRight(), shelf->getBottomRight());

    // Loop through each product
    for (std::size_t product_id = 0; product_id < bin->getProducts().size(); ++product_id)
    {
      ProductObjectPtr product = bin->getProducts()[product_id];
      ROS_DEBUG_STREAM_NAMED("product_simulator", "Updating product " << product->getName());

      // Loop until non-collision pose found
      bool found = false;
      for (std::size_t i = 0; i < MAX_ATTEMPTS; ++i)
      {
        // Get random pose
        visuals_->visual_tools_->generateRandomPose(pose, pose_bounds);

        // Set pose of product
        product->setCentroid(pose);
        product->setMeshCentroid(pose);

        if (!inCollision(product, world_to_bin))
        {
          found = true;  // this is good enough
          // Lower product and loop until in collision (e.g. crappy gravity simulation)
          for (std::size_t z = 0; z < 1; z += LOWER_SEARCH_DISCRETIZATION)
          {
            pose.translation().z() -= LOWER_SEARCH_DISCRETIZATION;
            product->setCentroid(pose);
            product->setMeshCentroid(pose);

            // if (verbose_)
            //  product->visualizeHighRes(world_to_bin);

            if (inCollision(product, world_to_bin))
            {
              // Use current location, no matter where it ended up
              break;
            }
          }  // for lower z height

          // Debug
          if (show_random_generated_poses && false)
          {
            product->createCollisionBodies(world_to_bin);
          }

          // so that bounding_box works correctly
          convertMeshToBinFrame(product);

          // Calculate bounding mesh
          if (!percepetion_interface->updateBoundingMesh(product, bin))
          {
            ROS_WARN_STREAM_NAMED("product_simulator", "Unable to update bounding mesh");
          }

          // Visualize bounding box
          product->visualizeHighResWireframe(world_to_bin, rvt::YELLOW);
          product->visualizeHighRes(world_to_bin);

          // debug
          if (show_random_generated_poses)
          {
            product->createCollisionBodies(world_to_bin);
            // product->visualizeHighRes(world_to_bin);
          }

          break;
        }

      }  // for non-collision

      if (!found)
        ROS_ERROR_STREAM_NAMED(
            "product_simulator",
            "A product never had a random pose found and was not added to the planning scene");
    }  // for each product
  }    // for each bin

  return true;
}

bool ProductSimulator::addCollisionMesh(ProductObjectPtr& product,
                                        const Eigen::Affine3d& world_to_bin)
{
  // Get product pose
  Eigen::Affine3d world_to_product = transform(product->getCentroid(), world_to_bin);

  // TODO - don't load the mesh over and over, rather store the mesh
  shapes::Shape* mesh = shapes::createMeshFromResource(product->getCollisionMeshPath());
  shapes::ShapeMsg shape_msg;  // this is a boost::variant type from shape_messages.h
  if (!mesh || !shapes::constructMsgFromShape(mesh, shape_msg))
  {
    ROS_ERROR_STREAM_NAMED("visual_tools", "Unable to create mesh shape message");
    return false;
  }

  // Convert to proper msg type
  shape_msgs::Mesh mesh_msg = boost::get<shape_msgs::Mesh>(shape_msg);

  // Create collision message
  moveit_msgs::CollisionObject collision_object_msg;
  collision_object_msg.header.stamp = ros::Time::now();
  collision_object_msg.header.frame_id = visuals_->visual_tools_->getBaseFrame();
  collision_object_msg.id = "singular_collision_object";  // this will overwrite any pre-existing
                                                          // objects from previous calls
  collision_object_msg.operation = moveit_msgs::CollisionObject::ADD;
  collision_object_msg.mesh_poses.resize(1);
  collision_object_msg.mesh_poses[0] = visuals_->visual_tools_->convertPose(world_to_product);
  collision_object_msg.meshes.resize(1);
  collision_object_msg.meshes[0] = mesh_msg;

  // scene->getCurrentStateNonConst().update(); // hack to prevent bad transforms
  secondary_scene_.processCollisionObjectMsg(collision_object_msg);

  return true;
}

bool ProductSimulator::inCollision(ProductObjectPtr& product, const Eigen::Affine3d& world_to_bin)
{
  // Create new planning scene and add product with random location
  addCollisionMesh(product, world_to_bin);

  // Create request
  collision_detection::CollisionRequest req;
  req.verbose = verbose_;
  collision_detection::CollisionResult res;

  // Check if in collision - Get planning scene lock
  {
    planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_);
    scene->getCollisionWorld()->checkWorldCollision(req, res,
                                                    *secondary_scene_.getCollisionWorld());
  }
  return res.collision;
}

bool ProductSimulator::convertMeshToBinFrame(ProductObjectPtr product)
{
  shape_msgs::Mesh& mesh_msg = product->getCollisionMesh();
  Eigen::Vector3d point;
  for (std::size_t i = 0; i < mesh_msg.vertices.size(); ++i)
  {
    point = visuals_->visual_tools_->convertPoint(mesh_msg.vertices[i]);
    point = product->getCentroid() * point;
    mesh_msg.vertices[i] = visuals_->visual_tools_->convertPoint(point);
  }
  product->setMeshCentroid(Eigen::Affine3d::Identity());

  return true;
}

}  // end namespace
