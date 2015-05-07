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
   Desc:   Basic properties to repersent a collision object
*/

#ifndef PICKNIK_MAIN__COLLISION_OBJECT
#define PICKNIK_MAIN__COLLISION_OBJECT

// ROS
#include <ros/ros.h>

// PickNik
#include <picknik_main/namespaces.h>
#include <picknik_main/visuals.h>

namespace picknik_main
{

MOVEIT_CLASS_FORWARD(RectangleObject);
MOVEIT_CLASS_FORWARD(MeshObject);

int iRand(int min, int max)
{
  int n = max - min + 1;
  int remainder = RAND_MAX % n;
  int x;
  do
  {
    x = rand();
  }
  while (x >= RAND_MAX - remainder);
  return min + x % n;
}

void printTransform(const Eigen::Affine3d &transform)
{
  Eigen::Quaterniond q(transform.rotation());
  std::cout << "T.xyz = [" << transform.translation().x() << ", " << transform.translation().y() << ", " << transform.translation().z() << "], Q.xyzw = ["
            << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() << "]" << std::endl;
}

/**
 * \brief Helper for converting frame of references
 */
inline Eigen::Affine3d transform(const Eigen::Affine3d& pose, const Eigen::Affine3d& trans)
{
  // Transform with respect to shelf
  return trans * pose;
}

// -------------------------------------------------------------------------------------------------
// Basic properties of an object in the world
// -------------------------------------------------------------------------------------------------
class CollisionObject
{
public:
  /**
   * \brief Constructor
   */
  CollisionObject(VisualsPtr visuals, const rvt::colors &color = rvt::RAND, const std::string &name = "");
  CollisionObject(const CollisionObject& copy);
  
  /**
   * \brief Show the outline of the object
   * \param trans - transform from parent container to current container
   * \return true on success
   */
  virtual bool visualizeWireframe(const Eigen::Affine3d& trans, const rvt::colors &color) const = 0;
  virtual bool visualizeHighResWireframe(const Eigen::Affine3d& trans, const rvt::colors &color) const = 0;

  /**
   * \brief Getter for rectangle name
   */ 
  std::string getName() const;
  
  /**
   * \brief Setter for rectangle name
   */
  void setName(std::string name);

  /**
   * \brief Getter for collision name - unique in case there are more than 1 product with the same name
   */
  const std::string& getCollisionName() const;
  
  /**
   * \brief Setter for collision name - unique in case there are more than 1 product with the same name
   */
  void setCollisionName(std::string name);

  /**
   * \brief Getter for Color
   */ 
  const rvt::colors& getColor() const;

  /**
   * \brief Setter for Color
   */
  void setColor(const rvt::colors& color);

  /**
   * \brief Getter for HighResMeshPath
   */ 
  const std::string& getHighResMeshPath();
  
  /**
   * \brief Setter for HighResMeshPath
   */
  void setHighResMeshPath(const std::string &high_res_mesh_path);

  /**
   * \brief Getter for CollisionMeshPath
   */ 
  const std::string& getCollisionMeshPath();

  /**
   * \brief Setter for CollisionMeshPath
   */
  void setCollisionMeshPath(const std::string &collision_mesh_path);

protected:

  // Name of object
  std::string name_;

  // A unique name to the world, whereas name_ can be duplicate e.g. oreo and oreo 
  std::string collision_object_name_;

  // Pointer to a pre-loaded visual_tools_ object
  VisualsPtr visuals_;

  // Color of object
  rvt::colors color_;

  // Mesh paths
  std::string high_res_mesh_path_;
  std::string collision_mesh_path_;
};


// -------------------------------------------------------------------------------------------------
// Rectangle Object - uses BottomRight and TopLeft coordinate system (not centroid-basd)
// -------------------------------------------------------------------------------------------------
class RectangleObject : public CollisionObject
{
public:
  /**
   * \brief Constructor
   */
  RectangleObject(VisualsPtr visuals, const rvt::colors &color = rvt::RAND, const std::string &name = "");
  RectangleObject(const RectangleObject& copy);
  
  /**
   * \brief Show bin in Rviz (not collision bodies)
   * \param trans - transform from parent container to current container
   */
  bool visualizeHighRes(const Eigen::Affine3d& trans) const;

  /**
   * \brief Show the outline of the object
   * \param trans - transform from parent container to current container
   * \return true on success
   */
  bool visualizeWireframe(const Eigen::Affine3d& trans, const rvt::colors &color = rvt::LIME_GREEN) const;
  bool visualizeHighResWireframe(const Eigen::Affine3d& trans, const rvt::colors &color = rvt::LIME_GREEN) const;

  /**
   * \brief Show the bottom right of the object
   * \param trans - transform from parent container to current container
   * \return true on success
   */
  bool visualizeAxis(const Eigen::Affine3d& trans) const;

  /**
   * \brief Create collision bodies of rectangle
   * \param trans - transform from parent container to current container
   */
  bool createCollisionBodies(const Eigen::Affine3d &trans);

  /**
   * \brief Get height of rectangle
   */
  double getHeight() const;

  /**
   * \brief Get width of rectangle
   */
  double getWidth() const;

  /**
   * \brief Get depth of rectangle
   */
  double getDepth() const;

  /**
   * \brief Getter for BottomRight
   */ 
  const Eigen::Affine3d& getBottomRight() const;
  
  /**
   * \brief Setter for BottomRight
   */
  void setBottomRight(const Eigen::Affine3d& bottom_right);

  /**
   * \brief Setter for BottomRight
   */
  void setBottomRight(const double& x, const double& y, const double& z);

  /**
   * \brief Getter for TopLeft
   */ 
  const Eigen::Affine3d& getTopLeft() const;

  /**
   * \brief Setter for TopLeft
   */
  void setTopLeft(const Eigen::Affine3d& top_left);

  /**
   * \brief Setter for TopLeft
   */
  void setTopLeft(const double& x, const double& y, const double& z);

  /**
   * \brief Getter for Centroid
   */ 
  const Eigen::Affine3d getCentroid() const;

protected:

  // Poses relative to center bottom of robot
  Eigen::Affine3d bottom_right_;
  Eigen::Affine3d top_left_;
}; // RectangleObject


// -------------------------------------------------------------------------------------------------
// Mesh Object - centroid-based
// -------------------------------------------------------------------------------------------------
class MeshObject : public CollisionObject
{
public:
  /**
   * \brief Constructor
   */
  MeshObject(VisualsPtr visuals, const rvt::colors &color = rvt::RAND, const std::string &name = "");
  MeshObject(const MeshObject& copy);
  
  /**
   * \brief Show bin in Rviz (not collision bodies)
   * \param trans - transform from parent container to current container
   */
  bool visualizeHighRes(const Eigen::Affine3d& trans) const;

  /**
   * \brief Show the outline of the object
   * \param trans - transform from parent container to current container
   * \return true on success
   */
  bool visualizeWireframe(const Eigen::Affine3d& trans, const rvt::colors &color = rvt::LIME_GREEN) const;
  bool visualizeHighResWireframe(const Eigen::Affine3d& trans, const rvt::colors &color = rvt::LIME_GREEN) const;

  /**
   * \brief Show the centroid of the object
   * \param trans - transform from parent container to current container
   * \return true on success
   */
  bool visualizeAxis(const Eigen::Affine3d& trans) const;

  /**
   * \brief Load from file a collision mesh
   * \return true on success
   */
  bool loadCollisionBodies();

  /**
   * \brief Write to file
   * \return true on success
   */
  bool writeCollisionBody(const std::string& file_path);

  /**
   * \brief Getter for CollisionMesh
   */ 
  const shape_msgs::Mesh& getCollisionMesh();

  /**
   * \brief Setter for CollisionMesh
   */
  void setCollisionMesh(const shape_msgs::Mesh& mesh);

  /**
   * \brief Create collision bodies of rectangle
   * \param trans - transform from parent container to current container
   */
  bool createCollisionBodies(const Eigen::Affine3d &trans);

  /**
   * \brief Get height of rectangle
   */
  double getHeight() const;

  /**
   * \brief Get width of rectangle
   */
  double getWidth() const;

  /**
   * \brief Get depth of rectangle
   */
  double getDepth() const;

  /**
   * \brief Setter for height
   */
  void setHeight(const double& height);

  /**
   * \brief Setter for width
   */
  void setWidth(const double& width);

  /**
   * \brief Setter for depth
   */
  void setDepth(const double& depth);

  /**
   * \brief Getter for Centroid
   */ 
  const Eigen::Affine3d& getCentroid() const;
  
  /**
   * \brief Setter for Centroid
   */
  void setCentroid(const Eigen::Affine3d& centroid);

  /**
   * \brief Getter for mesh-specific centroid
   */ 
  const Eigen::Affine3d& getMeshCentroid() const;


  /**
   * \brief Setter for centroid of mesh, which is often different than the calculated bounding box
  */
  void setMeshCentroid(const Eigen::Affine3d& centroid);

protected:

  // Geometry
  double height_;
  double width_;
  double depth_;

  // Loaded mesh
  shape_msgs::Mesh mesh_msg_;

  // Pose relative to parent object
  Eigen::Affine3d centroid_;
  Eigen::Affine3d mesh_centroid_;

}; // MeshObject

} // namespace

#endif

