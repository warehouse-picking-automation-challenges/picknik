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

#ifndef PICKNIK_MAIN__SHELF
#define PICKNIK_MAIN__SHELF

// ROS
#include <ros/ros.h>

// PickNik
#include <picknik_main/namespaces.h>
#include <picknik_main/visuals.h>
#include <picknik_main/manipulation_data.h>
#include <picknik_main/collision_object.h>

namespace picknik_main
{

MOVEIT_CLASS_FORWARD(ShelfObject);
MOVEIT_CLASS_FORWARD(BinObject);
MOVEIT_CLASS_FORWARD(ProductObject);

// Width of non-important collision objects such that the collision detector does not pass through them
static const double COLLISION_OBJECT_WIDTH = 0.1;

// -------------------------------------------------------------------------------------------------
// Bin Object
// -------------------------------------------------------------------------------------------------
class BinObject : public RectangleObject
{
  // Items in this bin
  std::vector<ProductObjectPtr> products_;

public:

  /**
   * \brief Constructor
   */
  BinObject(VisualsPtr visuals, const rvt::colors &color, const std::string &name);

  /**
   * \brief Show bin in Rviz (not collision bodies)
   * \param trans - transform from parent container to current container
   */
  bool visualizeHighRes(const Eigen::Affine3d& trans) const;

  /**
   * \brief Show coordinate system
   * \param trans - transform from parent container to current container
   */
  bool visualizeAxis(const Eigen::Affine3d& trans, VisualsPtr visuals) const;

  /**
   * \brief Add the products to be picked as collision objects
   * \param trans - transform from parent container to current container
   * \return true on success
   */
  bool createCollisionBodiesProducts(const Eigen::Affine3d &trans) const;

  /**
   * \brief Getter for products
   */ 
  std::vector<ProductObjectPtr>& getProducts();

  /**
   * \brief Getter for the products names
   */ 
  void getProducts(std::vector<std::string> &products);

  /**
   * \brief Getter for a product
   */ 
  ProductObjectPtr getProduct(const std::string& name);

  /**
   * \brief Helper to get the parent transform
   */
  Eigen::Affine3d getBinToWorld(ShelfObjectPtr &parent);

}; // class

typedef std::map<std::string, BinObjectPtr> BinObjectMap;

// -------------------------------------------------------------------------------------------------
// Shelf Object
// -------------------------------------------------------------------------------------------------

class ShelfObject : public RectangleObject
{
public:
  /**
   * \brief Constructor
   * \param shelf_id
   */
  ShelfObject(VisualsPtr visuals, const rvt::colors &color, const std::string &name, bool use_computer_vision_shelf);

  /**
   * \brief Load geometry of shelf and bins (coordinate systems, etc)
   */
  bool initialize(const std::string &package_path, ros::NodeHandle &nh);

  /**
   * \brief Other objects in our collision environment
   */
  void addOtherCollisionObjects();

  /**
   * \brief Helper for creating a bin
   */
  bool insertBinHelper(int bin_id, double height, double width, double wall_y, double bin_z);

  /**
   * \brief Show coordinate system
   */
  bool visualizeAxis(VisualsPtr visuals) const;

  /**
   * \brief Show shelf in Rviz (not collision bodies)
   */
  bool visualizeHighRes(bool show_products = true) const;

  /**
   * \brief Show all other collision objects
   */
  bool visualizeEnvironmentObjects() const;

  /**
   * \brief Add all other collision objects to planning scene
   */
  bool createCollisionBodiesEnvironmentObjects() const;

  /**
   * \brief Create collision bodies of shelf
   * \param focus_bin_id - which bin to enable e.g. allow manipulation in
   * \param only_show_shelf_frame - when false, show the contents of the shelf too
   * \param show_all_products - when false, only show the products of the focus bin
   */
  bool createCollisionBodies(const std::string& focus_bin_name = "", bool only_show_shelf_frame = false, bool show_all_products = false);

  /**
   * \brief Represent shelf in MoveIt! planning scene
   */
  bool createCollisionShelfDetailed();

  /**
   * \brief Getter for Bins
   */ 
  BinObjectMap& getBins();

  /**
   * \brief Getter for a bin by its index number
   * \param index, where 1 is bin A counting up
   */
  BinObjectPtr getBin(std::size_t bin_id);

  /**
   * \brief Get product
   * \param bin
   * \param product name
   * \return product info
   */
  ProductObjectPtr getProduct(const std::string &bin_name, const std::string &product_name);

  /**
   * \brief Get all the products existing in all the bins
   * \return true on success
   */
  bool getAllProducts(std::vector<ProductObjectPtr> &products);

  /**
   * \brief Delete product
   * \param bin
   * \param product name
   * \return true on success
   */
  bool deleteProduct(BinObjectPtr bin, ProductObjectPtr product);

  /**
   * \brief Load a shelf from the computer vision system
   * \return true on success
   */
  bool loadComputerVisionShelf(const std::vector<double>& collision_shelf_transform_doubles,
                               double collision_shelf_transform_x_offset,
                               const std::string& package_path);

  /**
   * \brief Get shelf parts for prevent collision with products
   * \return true on success
   */
  const std::vector<RectangleObject>& getShelfParts()
  {
    return shelf_parts_;
  }

  /**
   * \brief Getter for GoalBin
   */ 
  MeshObjectPtr getGoalBin()
  {
    return goal_bin_;
  }
  
  /**
   * \brief Setter for GoalBin
   */
  void setGoalBin(MeshObjectPtr goal_bin)
  {
    goal_bin_ = goal_bin;
  }

  /**
   * \brief Getter for RightWall
   */ 
  // const RectangleObjectPtr& getRightWall() const
  // {
  //   return right_wall_;
  // }
  
  // /**
  //  * \brief Getter for FloorWall
  //  */ 
  // const RectangleObjectPtr& getFloorWall() const
  // {
  //   return floor_wall_;
  // }
  
  // /**
  //  * \brief Getter for CeilingWall
  //  */ 
  // const RectangleObjectPtr& getCeilingWall() const
  // {
  //   return ceiling_wall_;
  // }
  
  // /**
  //  * \brief Setter for RightWall
  //  */
  // void setRightWall(const RectangleObjectPtr& right_wall)
  // {
  //   right_wall_ = right_wall;
  // }

  // /**
  //  * \brief Getter for LeftWall
  //  */ 
  // const RectangleObjectPtr& getLeftWall() const
  // {
  //   return left_wall_;
  // }
  
  // /**
  //  * \brief Setter for LeftWall
  //  */
  // void setLeftWall(const RectangleObjectPtr& left_wall)
  // {
  //   left_wall_ = left_wall;
  // }

  /**
   * \brief Getter for FrontWall
   */ 
  const RectangleObjectPtr& getFrontWall() const
  {
    return front_wall_;
  }
  
  /**
   * \brief Setter for FrontWall
   */
  void setFrontWall(const RectangleObjectPtr& front_wall)
  {
    front_wall_ = front_wall;
  }

  /**
   * \brief Get an object in the environment collision
   */
  RectangleObjectPtr getEnvironmentCollisionObject(const std::string& name)
  {
    return environment_objects_[name];
  }

  /**
   * \brief Reset contents of shelf
   * \return true on success
   */
  void clearProducts()
  {
    for (BinObjectMap::const_iterator bin_it = bins_.begin(); bin_it != bins_.end(); bin_it++)
    {
      bin_it->second->getProducts().clear();
    }
  }
  
  // Loaded shelf parameter values
  //double shelf_distance_from_robot_;
  Eigen::Affine3d world_to_shelf_transform_;
  Eigen::Affine3d collision_shelf_transform_;
  double shelf_width_;
  double shelf_height_;
  double shelf_depth_;
  double shelf_wall_width_;
  double shelf_surface_thickness_;
  double shelf_inner_wall_width_;
  double first_bin_from_bottom_;
  double first_bin_from_right_;

  // Loaded bin parameter values
  double bin_right_width_;
  double bin_middle_width_;
  double bin_left_width_;
  double bin_short_height_;
  double bin_tall_height_;
  double bin_depth_;

  double bin_top_margin_;
  double bin_left_margin_;
  double num_bins_;

  // Goal bin
  double goal_bin_x_;
  double goal_bin_y_;
  double goal_bin_z_;

  // Side limits (walls)
  double left_wall_y_;
  double right_wall_y_;

  // Top limit
  double ceiling_z_;

  // Safety
  double collision_wall_safety_margin_;

private:

  // Walls of shelf
  std::vector<RectangleObject> shelf_parts_;

  // Bins of shelf
  BinObjectMap bins_;

  MeshObjectPtr goal_bin_;
  MeshObjectPtr computer_vision_shelf_;

  RectangleObjectPtr front_wall_;

  std::map<std::string,RectangleObjectPtr> environment_objects_;

  Eigen::Affine3d high_res_mesh_offset_;

  bool use_computer_vision_shelf_;
}; // class

// -------------------------------------------------------------------------------------------------
// Product Object
// -------------------------------------------------------------------------------------------------
class ProductObject : public MeshObject
{
public:
 /**
   * \brief Constructor
   */
  ProductObject(VisualsPtr visuals, const rvt::colors &color, const std::string &name, const std::string &package_path);
  ProductObject(const ProductObject& copy);

  /**
   * \brief Get pose of product in frame of world
   * \param shelf - the shelf holding the product
   * \param bin - the ben holding the product
   * \return pose of product to world   
   */
  Eigen::Affine3d getWorldPose(const ShelfObjectPtr& shelf, const BinObjectPtr& bin);

private:

}; // class

// -------------------------------------------------------------------------------------------------
// Work Order Struct
// -------------------------------------------------------------------------------------------------
/**
 * \brief Contains the desired product to pick
 */
struct WorkOrder
{
  WorkOrder()
  {}

  WorkOrder( BinObjectPtr bin,
             ProductObjectPtr product)
    : bin_(bin)
    , product_(product)
  {}

  BinObjectPtr bin_;
  ProductObjectPtr product_;
};

typedef std::vector<WorkOrder> WorkOrders;

} // namespace

#endif
