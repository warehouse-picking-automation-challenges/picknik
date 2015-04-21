/*********************************************************************
 * Software License Agreement
 *
 *  Copyright (c) 2015, Dave Coleman <dave@dav.ee>
 *  All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *********************************************************************/

/* Author: Dave Coleman <dave@dav.ee>
   Desc:   Object oriented shelf system - represents shelf, bins, etc
*/

#include <picknik_main/collision_object.h>
#include <moveit_grasps/grasp_generator.h> // get bounding box

namespace picknik_main
{

// -------------------------------------------------------------------------------------------------
// Basic properties of an object in the world
// -------------------------------------------------------------------------------------------------

CollisionObject::CollisionObject(VisualsPtr visuals,
                                 const rvt::colors &color, const std::string &name)
  : visuals_(visuals)
  , color_(color)
{
  if (name.empty())
  {
    // Create dummy name for this rectangle
    static std::size_t rectangle_id = 0;
    rectangle_id++;

    // use this func so that a unique collision ID is also generated
    setName("collision_" + boost::lexical_cast<std::string>(rectangle_id));

    ROS_WARN_STREAM_NAMED("shelf","Creating default rectangle named " << name_);
  }
  else
  {
    setName(name); // use this func so that a unique collision ID is also generated
  }
}

CollisionObject::CollisionObject(const CollisionObject& copy)
{
  visuals_= copy.visuals_;
  color_= copy.color_;

  // Must set collision_object_name_ and name_ manually via setName() function
  setName(copy.name_);
}

std::string CollisionObject::getName() const
{
  return name_;
}

void CollisionObject::setName(std::string name)
{
  name_ = name;

  // Create unique collision name
  static std::size_t collision_id = 0;
  collision_id++;
  collision_object_name_ = name + "_" + boost::lexical_cast<std::string>(collision_id);
}

const std::string& CollisionObject::getCollisionName() const
{
  return collision_object_name_;
}

void CollisionObject::setCollisionName(std::string name)
{
  collision_object_name_ = name;
}

const rvt::colors& CollisionObject::getColor() const
{
  return color_;
}

void CollisionObject::setColor(const rvt::colors& color)
{
  color_ = color;
}

const std::string& CollisionObject::getHighResMeshPath()
{
  return high_res_mesh_path_;
}

void CollisionObject::setHighResMeshPath(const std::string &high_res_mesh_path)
{
  high_res_mesh_path_ = high_res_mesh_path;
}

const std::string& CollisionObject::getCollisionMeshPath()
{
  return collision_mesh_path_;
}

void CollisionObject::setCollisionMeshPath(const std::string &collision_mesh_path)
{
  collision_mesh_path_ = collision_mesh_path;
}

// -------------------------------------------------------------------------------------------------
// Rectangle Object - uses BottomRight and TopLeft coordinate system (not centroid-basd)
// -------------------------------------------------------------------------------------------------

RectangleObject::RectangleObject(VisualsPtr visuals, const rvt::colors &color, const std::string &name)
  : CollisionObject(visuals, color, name)
  , bottom_right_(Eigen::Affine3d::Identity())
  , top_left_(Eigen::Affine3d::Identity())
{
}

RectangleObject::RectangleObject(const RectangleObject& copy)
  : CollisionObject( copy )
{
  bottom_right_= copy.bottom_right_;
  top_left_= copy.top_left_;
}

bool RectangleObject::visualize(const Eigen::Affine3d& trans) const
{
  // Show simple geometric shape
  return visuals_->visual_tools_display_->publishCuboid( transform(bottom_right_, trans).translation(),
                                                         transform(top_left_, trans).translation(), color_);
}

bool RectangleObject::visualizeWireframe(const Eigen::Affine3d& trans) const
{
  ROS_WARN_STREAM_NAMED("temp","viz wireframe todo");
  //return visuals_->visual_tools_display_->publishWireframeCuboid( transform(centroid_, trans), getDepth(), getWidth(), getHeight(), color_);
  return true;
}

bool RectangleObject::visualizeAxis(const Eigen::Affine3d& trans) const
{
  return visuals_->visual_tools_->publishAxisLabeled( transform(bottom_right_, trans), name_ );
}

bool RectangleObject::createCollisionBodies(const Eigen::Affine3d &trans)
{
  ROS_DEBUG_STREAM_NAMED("shelf","Adding/updating collision body '" << collision_object_name_ << "'");

  // Just use basic rectangle
  return visuals_->visual_tools_->publishCollisionCuboid( transform(bottom_right_, trans).translation(),
                                                          transform(top_left_, trans).translation(),
                                                          collision_object_name_, color_ );
}

double RectangleObject::getHeight() const
{
  return top_left_.translation().z() - bottom_right_.translation().z();
}

double RectangleObject::getWidth() const
{
  return top_left_.translation().y() - bottom_right_.translation().y();
}

double RectangleObject::getDepth() const
{
  return top_left_.translation().x() - bottom_right_.translation().x();
}

const Eigen::Affine3d& RectangleObject::getBottomRight() const
{
  return bottom_right_;
}

void RectangleObject::setBottomRight(const Eigen::Affine3d& bottom_right)
{
  bottom_right_ = bottom_right;
}

void RectangleObject::setBottomRight(const double& x, const double& y, const double& z)
{
  bottom_right_.translation().x() = x;
  bottom_right_.translation().y() = y;
  bottom_right_.translation().z() = z;
}

const Eigen::Affine3d& RectangleObject::getTopLeft() const
{
  return top_left_;
}

void RectangleObject::setTopLeft(const Eigen::Affine3d& top_left)
{
  top_left_ = top_left;
}

void RectangleObject::setTopLeft(const double& x, const double& y, const double& z)
{
  top_left_.translation().x() = x;
  top_left_.translation().y() = y;
  top_left_.translation().z() = z;
}

const Eigen::Affine3d RectangleObject::getCentroid() const
{
  Eigen::Affine3d centroid = bottom_right_;
  centroid.translation().x() += getDepth() / 2.0;
  centroid.translation().y() += getWidth() / 2.0;
  centroid.translation().z() += getHeight() / 2.0;
  return centroid;
}

// -------------------------------------------------------------------------------------------------
// Mesh Object - centroid-based
// -------------------------------------------------------------------------------------------------

MeshObject::MeshObject(VisualsPtr visuals, const rvt::colors &color, const std::string &name)
  : CollisionObject(visuals, color, name)
  , centroid_(Eigen::Affine3d::Identity())
  , height_(0.0)
  , width_(0.0)
  , depth_(0.0)
{
}

MeshObject::MeshObject(const MeshObject& copy)
  : CollisionObject( copy )
{
  centroid_= copy.centroid_;
  height_ = copy.height_;
  width_ = copy.width_;
  depth_ = copy.depth_;
  high_res_mesh_path_ = copy.high_res_mesh_path_;
  collision_mesh_path_ = copy.collision_mesh_path_;
  mesh_msg_ = copy.mesh_msg_;
}

bool MeshObject::visualize(const Eigen::Affine3d& trans) const
{
  if (high_res_mesh_path_.empty())
  {
    ROS_ERROR_STREAM_NAMED("temp","no mesh path provided");
  }

  // Show axis
  visuals_->visual_tools_display_->publishAxis(transform(centroid_, trans), 0.1/2, 0.01/2);

  // Show full resolution mesh - scale = 1, id = 1, namespace = collision object name
  return visuals_->visual_tools_display_->publishMesh(transform(centroid_, trans), high_res_mesh_path_, rvt::CLEAR, 1,
                                                      collision_object_name_, 1);
}

bool MeshObject::visualizeWireframe(const Eigen::Affine3d& trans) const
{
  // Show wireframe in both systems
  visuals_->visual_tools_display_->publishWireframeCuboid( transform(centroid_, trans), depth_, width_, height_, rvt::LIME_GREEN);
  visuals_->visual_tools_->publishWireframeCuboid( transform(centroid_, trans), depth_, width_, height_, rvt::LIME_GREEN);
  return true;
}

bool MeshObject::visualizeAxis(const Eigen::Affine3d& trans) const
{
  return visuals_->visual_tools_->publishAxisLabeled( transform(centroid_, trans), name_ );
}

bool MeshObject::loadCollisionBodies()
{
  shapes::Shape *mesh = shapes::createMeshFromResource(collision_mesh_path_); // make sure its prepended by file://
  shapes::ShapeMsg shape_msg; // this is a boost::variant type from shape_messages.h
  if (!mesh || !shapes::constructMsgFromShape(mesh, shape_msg))
  {
    ROS_ERROR_STREAM_NAMED("shelf","Unable to create mesh shape message from resource " << collision_mesh_path_);
    return false;
  }

  mesh_msg_ = boost::get<shape_msgs::Mesh>(shape_msg);

  return true;
}

const shape_msgs::Mesh& MeshObject::getCollisionMesh() const
{
  return mesh_msg_;
}

void MeshObject::setCollisionMesh(const shape_msgs::Mesh& mesh)
{
  mesh_msg_ = mesh;
}

bool MeshObject::createCollisionBodies(const Eigen::Affine3d &trans)
{
  ROS_DEBUG_STREAM_NAMED("collision_object","Adding/updating collision body '" << collision_object_name_ << "'");

  // Check if mesh is provided
  if (collision_mesh_path_.empty())
  {
    ROS_ERROR_STREAM_NAMED("temp","no collision body provided");
  }

  // Check if mesh needs to be loaded
  if (mesh_msg_.triangles.empty()) // load mesh from file
  {
    if (!loadCollisionBodies())
      return false;
  }
  return visuals_->visual_tools_->publishCollisionMesh(transform(centroid_, trans), collision_object_name_, mesh_msg_, color_);
}

double MeshObject::getHeight() const
{
  return height_;
}

double MeshObject::getWidth() const
{
  return width_;
}

double MeshObject::getDepth() const
{
  return depth_;
}

void MeshObject::setHeight(const double& height)
{
  height_ = height;
}

void MeshObject::setWidth(const double& width)
{
  width_ = width;
}

void MeshObject::setDepth(const double& depth)
{
  depth_ = depth;
}

const Eigen::Affine3d& MeshObject::getCentroid() const
{
  return centroid_;
}

void MeshObject::setCentroid(const Eigen::Affine3d& centroid)
{
  centroid_ = centroid;
}

bool MeshObject::calculateBoundingBox(bool verbose)
{
  if (verbose)
  {
    std::cout << std::endl;
    std::cout << "-------------------------------------------------------" << std::endl;

    std::cout << "Before getBoundingingBoxFromMesh(): " << std::endl;
    std::cout << "  Cuboid Pose: "; printTransform(getCentroid());
    std::cout << "  Height: " << getHeight() << std::endl;
    std::cout << "  Depth: " << getDepth() << std::endl;
    std::cout << "  Width: " << getWidth() << std::endl;
  }

  // Get bounding box
  Eigen::Affine3d cuboid_pose; // TODO what to do with this?
  double depth, width, height;
  if (!moveit_grasps::GraspGenerator::getBoundingBoxFromMesh(getCollisionMesh(), cuboid_pose, depth, width, height))
  {
    ROS_ERROR_STREAM_NAMED("manipulation","Failed to get bounding box");
    return false;
  }
  setDepth(depth);
  setWidth(width);
  setHeight(height);

  if (verbose)
  {
    std::cout << "After getBoundingingBoxFromMesh(): " << std::endl;
    std::cout << "  Cuboid Pose: "; printTransform(getCentroid());
    std::cout << "  Height: " << getHeight() << std::endl;
    std::cout << "  Depth: " << getDepth() << std::endl;
    std::cout << "  Width: " << getWidth() << std::endl;
    std::cout << "-------------------------------------------------------" << std::endl;
  }
  return true;
}

} // namespace
