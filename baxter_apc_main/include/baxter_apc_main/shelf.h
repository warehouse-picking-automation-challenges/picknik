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

#ifndef BAXTER_APC_MAIN__SHELF
#define BAXTER_APC_MAIN__SHELF

// ROS
#include <ros/ros.h>

// MoveIt
#include <baxter_apc_main/namespaces.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

namespace baxter_apc_main
{

MOVEIT_CLASS_FORWARD(ShelfObject);
MOVEIT_CLASS_FORWARD(BinObject);
MOVEIT_CLASS_FORWARD(ProductObject);

const double SHELF_WIDTH = 0.873;
const double SHELF_HEIGHT= 2.37;
const double SHELF_DEPTH = 0.875;
const double SHELF_DISTANCE_FROM_BAXTER = 1.0; //0.8 // this is the main variable - how far from baxter's face forward is shelf?

const double FIRST_BIN_FROM_BOTTOM = 0.81;
const double FIRST_BIN_FROM_RIGHT = 0.036;
const double BIN_WIDTH = SHELF_WIDTH / 3.2;
const double BIN_HEIGHT = 0.26; //0.24;
const double BIN_DEPTH = 0.42;
const double BIN_TOP_MARGIN = 0.01;
const double BIN_LEFT_MARGIN = 0.01;
const double NUM_BINS = 12;
const double SHELF_WALL_WIDTH = 0.02;

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

/**
 * \brief Helper for converting frame of references
 */
inline Eigen::Affine3d transform(const Eigen::Affine3d& pose, const Eigen::Affine3d& trans)
{
  // Transform with respect to shelf
  return trans * pose;
}

// -------------------------------------------------------------------------------------------------
// Rectangle Object
// -------------------------------------------------------------------------------------------------
class Rectangle
{
protected:
  // Name of object
  std::string name_;

  // Pointer to a pre-loaded visual_tools_ object
  mvt::MoveItVisualToolsPtr visual_tools_;
  mvt::MoveItVisualToolsPtr visual_tools_display_;

public:

  // Poses relative to center bottom of baxter
  Eigen::Affine3d bottom_right_;
  Eigen::Affine3d top_left_;

  // Color of object
  rvt::colors color_;

  /**
   * \brief Constructor
   * \return
   */
  Rectangle(mvt::MoveItVisualToolsPtr visual_tools, mvt::MoveItVisualToolsPtr visual_tools_display,
            const rvt::colors &color = rvt::RAND, const std::string &name = "");
  
  /**
   * \brief Show coordinate system
   * \param trans - transform from parent container to current container
   */
  bool visualizeAxis(const Eigen::Affine3d& trans, mvt::MoveItVisualToolsPtr visual_tools) const;

  /**
   * \brief Show bin in Rviz (not collision bodies)
   * \param trans - transform from parent container to current container
   */
  bool visualize(const Eigen::Affine3d& trans) const;

  /**
   * \brief Create collision bodies of rectangle
   * \param trans - transform from parent container to current container
   */
  bool createCollisionBodies(const Eigen::Affine3d &trans) const;

  /**
   * \brief Get centroid of bounding box
   */
  void getCentroid(Eigen::Affine3d &pose) const;

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
   * \brief Getter for rectangle name
   */ 
  std::string getName() const;
  
  /**
   * \brief Setter for rectangle name
   */
  void setName(std::string name);

};

// -------------------------------------------------------------------------------------------------
// Bin Object
// -------------------------------------------------------------------------------------------------
class BinObject : public Rectangle
{
  // Items in this bin
  std::vector<ProductObjectPtr> products_;

public:

  /**
   * \brief Constructor
   */
  BinObject(mvt::MoveItVisualToolsPtr visual_tools, mvt::MoveItVisualToolsPtr visual_tools_display,
            const rvt::colors &color,
            const std::string &name);

  /**
   * \brief Show bin in Rviz (not collision bodies)
   * \param trans - transform from parent container to current container
   */
  bool visualize(const Eigen::Affine3d& trans) const;

  /**
   * \brief Create collision bodies of bin
   * \param trans - transform from parent container to current container
   */
  bool createCollisionBodies(const Eigen::Affine3d &trans) const;

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
   * \brief Getter for a product
   */ 
  ProductObjectPtr getProduct(const std::string& name);

}; // class

typedef std::map<std::string, BinObjectPtr> BinObjectMap;

// -------------------------------------------------------------------------------------------------
// Shelf Object
// -------------------------------------------------------------------------------------------------

class ShelfObject : public Rectangle
{
  // Walls of shelf
  std::vector<Rectangle> shelf_parts_;

  // Bins of shelf
  BinObjectMap bins_;

  // STL Model
  std::string mesh_path_;

public:

  /**
   * \brief Constructor
   * \param shelf_id
   */
  ShelfObject(mvt::MoveItVisualToolsPtr visual_tools, mvt::MoveItVisualToolsPtr visual_tools_display,
              const rvt::colors &color,
              const std::string &name,
              const std::string &package_path);

  /**
   * \brief Load geometry of shelf and bins (coordinate systems, etc)
   */
  bool initialize(const std::string &package_path);

  /**
   * \brief Helper for creating a bin
   */
  bool insertBinHelper(rvt::colors color, const std::string& name);

  /**
   * \brief Show coordinate system
   */
  bool visualizeAxis(mvt::MoveItVisualToolsPtr visual_tools) const;

  /**
   * \brief Show shelf in Rviz (not collision bodies)
   */
  //bool visualize() const;
  bool visualize() const;

  /**
   * \brief Create collision bodies of shelf
   * \param focus_bin_id - which bin to enable e.g. allow manipulation in
   */
  bool createCollisionBodies(const std::string& focus_bin_name = "", bool just_frame = false, bool show_all_products = false) const;

  /**
   * \brief Represent shelf in MoveIt! planning scene
   */
  bool createCollisionShelfDetailed() const;

  /**
   * \brief Getter for Bins
   */ 
  BinObjectMap& getBins();

  /**
   * \brief Get product
   * \param bin
   * \param product name
   * \return product info
   */
  ProductObjectPtr getProduct(const std::string &bin_name, const std::string &product_name);

  /**
   * \brief Delete product
   * \param bin
   * \param product name
   */
  bool deleteProduct(const std::string &bin_name, const std::string &product_name);

  /**
   * \brief Get shelf parts for prevent collision with products
   * \return true on success
   */
  const std::vector<Rectangle>& getShelfParts()
  {
    return shelf_parts_;
  }

}; // class

// -------------------------------------------------------------------------------------------------
// Product Object
// -------------------------------------------------------------------------------------------------
class ProductObject : public Rectangle
{
  std::string collision_object_name_;
  std::string collision_mesh_path_;
  std::string mesh_path_;

public:
 /**
   * \brief Constructor
   */
  ProductObject(mvt::MoveItVisualToolsPtr visual_tools, mvt::MoveItVisualToolsPtr visual_tools_display,
                const rvt::colors &color,
                const std::string &name,
                const std::string &package_path);

  /**
   * \brief Getter for collision name - unique incase there are more than 1 product with the same name
   */
  std::string getCollisionName() const;
  
  /**
   * \brief Setter for collision name - unique incase there are more than 1 product with the same name
   */
  void setCollisionName(std::string name);

  /**
   * \brief Show product in Rviz (not collision bodies)
   * \param trans - transform from parent container to current container
   */
  bool visualize(const Eigen::Affine3d &trans) const;

  /**
   * \brief Create collision bodies of rectangle
   * \param trans - transform from parent container to current container
   */
  bool createCollisionBodies(const Eigen::Affine3d &trans) const;

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
