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

#include <iostream>

namespace picknik_main
{
// -------------------------------------------------------------------------------------------------
// Basic properties of an object in the world
// -------------------------------------------------------------------------------------------------

CollisionObject::CollisionObject(VisualsPtr visuals, const rvt::colors& color,
                                 const std::string& name)
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

    ROS_WARN_STREAM_NAMED("collision_object", "Creating default rectangle named " << name_);
  }
  else
  {
    setName(name);  // use this func so that a unique collision ID is also generated
  }
}

CollisionObject::CollisionObject(const CollisionObject& copy)
{
  visuals_ = copy.visuals_;
  color_ = copy.color_;

  // Must set collision_object_name_ and name_ manually via setName() function
  setName(copy.name_);
}

std::string CollisionObject::getName() const { return name_; }
void CollisionObject::setName(std::string name)
{
  name_ = name;

  // Create unique collision name
  static std::size_t collision_id = 0;
  collision_id++;
  collision_object_name_ = name + "_" + boost::lexical_cast<std::string>(collision_id);
}

const std::string& CollisionObject::getCollisionName() const { return collision_object_name_; }
void CollisionObject::setCollisionName(std::string name) { collision_object_name_ = name; }
const rvt::colors& CollisionObject::getColor() const { return color_; }
void CollisionObject::setColor(const rvt::colors& color) { color_ = color; }
const std::string& CollisionObject::getHighResMeshPath() { return high_res_mesh_path_; }
void CollisionObject::setHighResMeshPath(const std::string& high_res_mesh_path)
{
  high_res_mesh_path_ = high_res_mesh_path;
}

const std::string& CollisionObject::getCollisionMeshPath() { return collision_mesh_path_; }
void CollisionObject::setCollisionMeshPath(const std::string& collision_mesh_path)
{
  collision_mesh_path_ = collision_mesh_path;
}

// -------------------------------------------------------------------------------------------------
// Rectangle Object - uses BottomRight and TopLeft coordinate system (not centroid-basd)
// -------------------------------------------------------------------------------------------------

RectangleObject::RectangleObject(VisualsPtr visuals, const rvt::colors& color,
                                 const std::string& name)
  : CollisionObject(visuals, color, name)
  , bottom_right_(Eigen::Affine3d::Identity())
  , top_left_(Eigen::Affine3d::Identity())
{
}

RectangleObject::RectangleObject(const RectangleObject& copy)
  : CollisionObject(copy)
{
  bottom_right_ = copy.bottom_right_;
  top_left_ = copy.top_left_;
}

bool RectangleObject::visualizeHighRes(const Eigen::Affine3d& trans) const
{
  // Show simple geometric shape
  return visuals_->visual_tools_display_->publishCuboid(
      transform(bottom_right_, trans).translation(), transform(top_left_, trans).translation(),
      color_);
}

bool RectangleObject::visualizeWireframe(const Eigen::Affine3d& trans,
                                         const rvt::colors& color) const
{
  ROS_WARN_STREAM_NAMED("collision_object", "viz wireframe todo");
  // return visuals_->visual_tools_->publishWireframeCuboid( transform(centroid_, trans),
  // getDepth(), getWidth(), getHeight(), color_);
  return true;
}

bool RectangleObject::visualizeHighResWireframe(const Eigen::Affine3d& trans,
                                                const rvt::colors& color) const
{
  ROS_WARN_STREAM_NAMED("collision_object", "viz wireframe todo");
  // return visuals_->visual_tools_display_->publishWireframeCuboid( transform(centroid_, trans),
  // getDepth(), getWidth(), getHeight(), color_);
  return true;
}

bool RectangleObject::visualizeAxis(const Eigen::Affine3d& trans) const
{
  return visuals_->visual_tools_->publishAxisLabeled(transform(bottom_right_, trans), name_);
}

bool RectangleObject::createCollisionBodies(const Eigen::Affine3d& trans)
{
  ROS_DEBUG_STREAM_NAMED("collision_object", "Adding/updating collision body '"
                                                 << collision_object_name_ << "'");

  // Just use basic rectangle
  return visuals_->visual_tools_->publishCollisionCuboid(
      transform(bottom_right_, trans).translation(), transform(top_left_, trans).translation(),
      collision_object_name_, color_);
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

const Eigen::Affine3d& RectangleObject::getBottomRight() const { return bottom_right_; }
void RectangleObject::setBottomRight(const Eigen::Affine3d& bottom_right)
{
  bottom_right_ = bottom_right;
}

void RectangleObject::setBottomRightUpdateAll(const Eigen::Affine3d& bottom_right)
{
  bottom_right_ = bottom_right;

  // TODO
}

void RectangleObject::setBottomRight(const double& x, const double& y, const double& z)
{
  bottom_right_.translation().x() = x;
  bottom_right_.translation().y() = y;
  bottom_right_.translation().z() = z;
}

const Eigen::Affine3d& RectangleObject::getTopLeft() const { return top_left_; }
void RectangleObject::setTopLeft(const Eigen::Affine3d& top_left) { top_left_ = top_left; }
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

MeshObject::MeshObject(VisualsPtr visuals, const rvt::colors& color, const std::string& name)
  : CollisionObject(visuals, color, name)
  , height_(0.0)
  , width_(0.0)
  , depth_(0.0)
  , centroid_(Eigen::Affine3d::Identity())
  , mesh_centroid_(Eigen::Affine3d::Identity())
{
}

MeshObject::MeshObject(const MeshObject& copy)
  : CollisionObject(copy)
{
  centroid_ = copy.centroid_;
  mesh_centroid_ = copy.mesh_centroid_;
  height_ = copy.height_;
  width_ = copy.width_;
  depth_ = copy.depth_;
  high_res_mesh_path_ = copy.high_res_mesh_path_;
  collision_mesh_path_ = copy.collision_mesh_path_;
  mesh_msg_ = copy.mesh_msg_;
}

bool MeshObject::visualizeHighRes(const Eigen::Affine3d& trans) const
{
  if (high_res_mesh_path_.empty())
  {
    ROS_ERROR_STREAM_NAMED("collision_object", "no mesh path provided");
  }

  // Show axis
  // visuals_->visual_tools_display_->publishAxis(transform(centroid_, trans), 0.1/2, 0.01/2);

  // Show full resolution mesh - scale = 1,
  const std::size_t id = 1;
  return visuals_->visual_tools_display_->publishMesh(transform(mesh_centroid_, trans),
                                                      high_res_mesh_path_, rvt::CLEAR, 1,
                                                      collision_object_name_, id);
}

bool MeshObject::visualizeWireframe(const Eigen::Affine3d& trans, const rvt::colors& color) const
{
  Eigen::Affine3d pose = transform(centroid_, trans);
  std::size_t id = 1;
  visuals_->visual_tools_->publishWireframeCuboid(pose, depth_, width_, height_, color,
                                                  collision_object_name_ + "_wireframe", id);
  visuals_->visual_tools_->publishAxis(pose);
  return true;
}

bool MeshObject::visualizeHighResWireframe(const Eigen::Affine3d& trans,
                                           const rvt::colors& color) const
{
  Eigen::Affine3d pose = transform(centroid_, trans);
  std::size_t id = 1;
  visuals_->visual_tools_display_->publishWireframeCuboid(
      pose, depth_, width_, height_, color, collision_object_name_ + "_wireframe", id);
  return true;
}

bool MeshObject::visualizeAxis(const Eigen::Affine3d& trans) const
{
  return visuals_->visual_tools_->publishAxisLabeled(transform(centroid_, trans), name_);
}

bool MeshObject::loadCollisionBodies()
{
  shapes::Shape* mesh =
      shapes::createMeshFromResource(collision_mesh_path_);  // make sure its prepended by file://
  shapes::ShapeMsg shape_msg;  // this is a boost::variant type from shape_messages.h
  if (!mesh || !shapes::constructMsgFromShape(mesh, shape_msg))
  {
    ROS_ERROR_STREAM_NAMED("collision_object", "Unable to create mesh shape message from resource "
                                                   << collision_mesh_path_);
    return false;
  }

  mesh_msg_ = boost::get<shape_msgs::Mesh>(shape_msg);

  return true;
}

bool MeshObject::writeCollisionBody(const std::string& file_path)
{
  ROS_DEBUG_STREAM_NAMED("collision_object", "Writing mesh to file");

  shapes::Shape* shape = shapes::constructShapeFromMsg(mesh_msg_);
  shapes::Mesh* mesh = static_cast<shapes::Mesh*>(shape);

  std::vector<char> buffer;
  shapes::writeSTLBinary(mesh, buffer);

  // Write
  std::ofstream file_stream(file_path.c_str(), std::ios::out | std::ofstream::binary);
  std::copy(buffer.begin(), buffer.end(), std::ostreambuf_iterator<char>(file_stream));

  return true;
}

shape_msgs::Mesh& MeshObject::getCollisionMesh()
{
  // Check if mesh needs to be loaded
  if (mesh_msg_.triangles.empty())  // load mesh from file
  {
    if (!loadCollisionBodies())
    {
      ROS_ERROR_STREAM_NAMED("collision_object", "Unable to load collision object");
    }
  }

  return mesh_msg_;
}

void MeshObject::setCollisionMesh(const shape_msgs::Mesh& mesh) { mesh_msg_ = mesh; }
bool MeshObject::createCollisionBodies(const Eigen::Affine3d& trans)
{
  ROS_DEBUG_STREAM_NAMED("collision_object", "Adding/updating collision body '"
                                                 << collision_object_name_ << "'");

  // Check if mesh is provided
  if (collision_mesh_path_.empty())
  {
    ROS_ERROR_STREAM_NAMED("collision_object", "no collision body provided");
  }

  // Check if mesh needs to be loaded
  if (mesh_msg_.triangles.empty())  // load mesh from file
  {
    if (!loadCollisionBodies())
      return false;
  }
  return visuals_->visual_tools_->publishCollisionMesh(transform(mesh_centroid_, trans),
                                                       collision_object_name_, mesh_msg_, color_);
}

double MeshObject::getHeight() const { return height_; }
double MeshObject::getWidth() const { return width_; }
double MeshObject::getDepth() const { return depth_; }
void MeshObject::setHeight(const double& height) { height_ = height; }
void MeshObject::setWidth(const double& width) { width_ = width; }
void MeshObject::setDepth(const double& depth) { depth_ = depth; }
const Eigen::Affine3d& MeshObject::getCentroid() const { return centroid_; }
void MeshObject::setCentroid(const Eigen::Affine3d& centroid) { centroid_ = centroid; }
const Eigen::Affine3d& MeshObject::getMeshCentroid() const { return mesh_centroid_; }
void MeshObject::setMeshCentroid(const Eigen::Affine3d& centroid) { mesh_centroid_ = centroid; }
}  // namespace
