/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Dave Coleman <dave@dav.ee>
 *  All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *********************************************************************/

/* Author: Dave Coleman <dave@dav.ee>
   Desc:   Object oriented shelf system - represents shelf, bins, etc
*/

#include <baxter_apc_main/shelf.h>

namespace baxter_apc_main
{

// -------------------------------------------------------------------------------------------------
// Rectangle Object
// -------------------------------------------------------------------------------------------------

Rectangle::Rectangle(moveit_visual_tools::MoveItVisualToolsPtr visual_tools, const rviz_visual_tools::colors &color,
                     const std::string &name)
  : visual_tools_(visual_tools)
  , color_(color)
  , bottom_right_(Eigen::Affine3d::Identity())
  , top_left_(Eigen::Affine3d::Identity())    
{
  if (name.empty())
  {
    // Create dummy name for this rectangle
    static std::size_t rectangle_id = 0;
    rectangle_id++;
    name_ = "rectangle_" + boost::lexical_cast<std::string>(rectangle_id);
    ROS_WARN_STREAM_NAMED("shelf","Creating default rectangle named " << name_);
  }
  else
  {
    name_ = name;
  }
}

bool Rectangle::visualizeAxis(const Eigen::Affine3d& trans) const
{
  // Show coordinate system
  visual_tools_->publishAxis( transform(bottom_right_, trans) );

  // Show label
  Eigen::Affine3d text_location = transform( bottom_right_, trans);
                                             
  text_location.translation() += Eigen::Vector3d(0,BIN_WIDTH/2.0, BIN_HEIGHT*0.9);

  visual_tools_->publishText( text_location, name_, rviz_visual_tools::BLACK, rviz_visual_tools::REGULAR, false);
  
}

bool Rectangle::visualize(const Eigen::Affine3d& trans) const
{
  // Show bin
  visual_tools_->publishRectangle( transform(bottom_right_, trans).translation(),
                                   transform(top_left_, trans).translation(),
                                   color_);
  return true;
}

bool Rectangle::createCollisionBodies(const Eigen::Affine3d &trans) const
{
  ROS_DEBUG_STREAM_NAMED("shelf","Creating collision rectangle with name " << name_);

  // Show bin
  visual_tools_->publishCollisionRectangle( transform(bottom_right_, trans).translation(),
                                            transform(top_left_, trans).translation(),
                                            name_, color_ );
  return true;
}

void Rectangle::getCentroid(Eigen::Affine3d &pose) const
{
  pose = bottom_right_;
  pose.translation().x() += getDepth() / 2.0;
  pose.translation().y() += getWidth() / 2.0;
  pose.translation().z() += getHeight() / 2.0;
}

double Rectangle::getHeight() const
{
  return top_left_.translation().z() - bottom_right_.translation().z();
}

double Rectangle::getWidth() const
{
  return top_left_.translation().y() - bottom_right_.translation().y();
}

double Rectangle::getDepth() const
{
  return top_left_.translation().x() - bottom_right_.translation().x();
}

std::string Rectangle::getName() const
{
  return name_;
}

void Rectangle::setName(std::string name)
{
  name_ = name;
}

// -------------------------------------------------------------------------------------------------
// Bin Object
// -------------------------------------------------------------------------------------------------

BinObject::BinObject(moveit_visual_tools::MoveItVisualToolsPtr visual_tools,
                     const rviz_visual_tools::colors &color,
                     const std::string &name)
  : Rectangle(visual_tools, color, name)
{
}

bool BinObject::visualize(const Eigen::Affine3d& trans) const
{
  // Show bin
  visual_tools_->publishRectangle( transform(bottom_right_, trans).translation(),
                                   transform(top_left_, trans).translation(),
                                   color_);

  // Show products
  for (std::size_t product_id = 0; product_id < products_.size(); ++product_id)
  {
    products_[product_id]->visualize(trans * bottom_right_); // send transform from world to bin
  }

  return true;
}

bool BinObject::createCollisionBodies(const Eigen::Affine3d &trans) const
{
  ROS_DEBUG_STREAM_NAMED("shelf","Creating collision bin " << name_);

  visual_tools_->publishCollisionRectangle( transform(bottom_right_, trans).translation(),
                                            transform(top_left_, trans).translation(),
                                            name_, color_ );

  return true;
}

bool BinObject::createCollisionBodiesProducts(const Eigen::Affine3d &trans) const
{
  // Show products
  for (std::size_t product_id = 0; product_id < products_.size(); ++product_id)
  {
    products_[product_id]->createCollisionBodies(trans * bottom_right_); // send transform from world to bin
  }
  return true;
}

std::vector<ProductObjectPtr>& BinObject::getProducts()
{
  return products_;
}

ProductObjectPtr BinObject::getProduct(const std::string& name)
{
  // Find correct product
  for (std::size_t prod_id = 0; prod_id < products_.size(); ++prod_id)
  {
    if (products_[prod_id]->getName() == name)
    {
      ROS_DEBUG_STREAM_NAMED("shelf","Found product " << name);
      return products_[prod_id];
    }
  }

  return ProductObjectPtr();
}

// -------------------------------------------------------------------------------------------------
// Shelf Object
// -------------------------------------------------------------------------------------------------

ShelfObject::ShelfObject(moveit_visual_tools::MoveItVisualToolsPtr visual_tools,
                         const rviz_visual_tools::colors &color,
                         const std::string &name,
                         const std::string &package_path)
  : Rectangle(visual_tools, color, name)
{
  initialize(package_path);
}

bool ShelfObject::initialize(const std::string &package_path)
{
  // Calculate shelf corners
  bottom_right_.translation().x() = SHELF_DISTANCE_FROM_BAXTER;
  bottom_right_.translation().y() = -SHELF_WIDTH/2.0;
  bottom_right_.translation().z() = 0;
  top_left_.translation().x() = bottom_right_.translation().x() + SHELF_DEPTH;
  top_left_.translation().y() = SHELF_WIDTH/2.0;
  top_left_.translation().z() = SHELF_HEIGHT;

  // Calculate first bin location
  Eigen::Affine3d bin1_bottom_right = Eigen::Affine3d::Identity();
  bin1_bottom_right.translation().y() = 0; //FIRST_BIN_FROM_RIGHT;
  bin1_bottom_right.translation().z() = FIRST_BIN_FROM_BOTTOM;

  // Create each bin
  std::size_t bin_id = 0;
  for (double z = bin1_bottom_right.translation().z();
       z < SHELF_HEIGHT; z = z + BIN_HEIGHT) // + BIN_TOP_MARGIN)
  {
    for (double y = bin1_bottom_right.translation().y();
         y < SHELF_WIDTH; y = y + BIN_WIDTH) // + BIN_LEFT_MARGIN)
    {
      // Create new bin
      std::string bin_name = "bin_" + boost::lexical_cast<std::string>((char)(65 + NUM_BINS - bin_id - 1)); // reverse the lettering
      ROS_DEBUG_STREAM_NAMED("shelf","Creating bin '" << bin_name << "'");

      insertBinHelper( rviz_visual_tools::YELLOW, bin_name );

      // Calculate new bin location
      bins_[bin_name]->bottom_right_.translation().x() = bin1_bottom_right.translation().x();
      bins_[bin_name]->bottom_right_.translation().y() = y;
      bins_[bin_name]->bottom_right_.translation().z() = z;

      bins_[bin_name]->top_left_ = bins_[bin_name]->bottom_right_;
      bins_[bin_name]->top_left_.translation().x() += BIN_DEPTH;
      bins_[bin_name]->top_left_.translation().y() += BIN_WIDTH;
      bins_[bin_name]->top_left_.translation().z() += BIN_HEIGHT;

      bin_id ++;
      if (bin_id >= NUM_BINS)
        break;
    }
    if (bin_id >= NUM_BINS)
      break;
  }

  // Create shelf parts -----------------------------

  // Base
  // Note: bottom right is at 0,0,0
  shelf_parts_.push_back(Rectangle(visual_tools_, color_, "base"));
  Rectangle &base = shelf_parts_[shelf_parts_.size()-1];
  base.top_left_.translation().x() += SHELF_DEPTH;
  base.top_left_.translation().y() += SHELF_WIDTH;
  base.top_left_.translation().z() += FIRST_BIN_FROM_BOTTOM;

  // Walls
  for (std::size_t i = 0; i < 4; ++i)
  {
    // Note: bottom right is at 0,0,0
    const std::string wall_name = "wall_" + boost::lexical_cast<std::string>(i);
    shelf_parts_.push_back(Rectangle(visual_tools_, color_, wall_name));
    Rectangle &wall = shelf_parts_[shelf_parts_.size()-1];
    // Geometry
    wall.bottom_right_.translation().x() = 0;
    wall.bottom_right_.translation().y() = BIN_WIDTH * i - SHELF_WALL_WIDTH * 0.5;
    wall.bottom_right_.translation().z() = FIRST_BIN_FROM_BOTTOM;
    wall.top_left_.translation().x() = SHELF_DEPTH;
    wall.top_left_.translation().y() = BIN_WIDTH * i + SHELF_WALL_WIDTH * 0.5;
    wall.top_left_.translation().z() = SHELF_HEIGHT;
  }

  // Shelves
  for (std::size_t i = 0; i < 5; ++i)
  {
    const std::string shelf_name = "shelf_" + boost::lexical_cast<std::string>(i);
    shelf_parts_.push_back(Rectangle(visual_tools_, color_, shelf_name));
    Rectangle &shelf = shelf_parts_[shelf_parts_.size()-1];
    // Geometry
    shelf.top_left_.translation().x() = SHELF_WIDTH;
    shelf.top_left_.translation().y() = SHELF_DEPTH;
    shelf.top_left_.translation().z() = FIRST_BIN_FROM_BOTTOM + BIN_HEIGHT * (i+1); // flush with shelf top
    shelf.bottom_right_.translation().z() = shelf.top_left_.translation().z() - SHELF_WALL_WIDTH; // add thickenss
  }

  // Load mesh file name
  mesh_path_ = "file://" + package_path + "/meshes/kiva_pod/meshes/pod_lowres.stl";


  return true;
}

bool ShelfObject::insertBinHelper(rviz_visual_tools::colors color, const std::string& name)
{
  BinObjectPtr new_bin(new BinObject(visual_tools_, color, name));

  bins_.insert( std::pair<std::string, BinObjectPtr>(name, new_bin));
                                                  
  return true;
}

bool ShelfObject::visualizeAxis() const
{
  // Show coordinate system
  visual_tools_->publishAxis( bottom_right_ );

  // Show each bin
  for (BinObjectMap::const_iterator bin_it = bins_.begin(); bin_it != bins_.end(); bin_it++)
  {
    bin_it->second->visualizeAxis(bottom_right_);    
  }
}

bool ShelfObject::visualize() const
{
  // Visualize shelf location as rectangle
  visual_tools_->publishRectangle(bottom_right_.translation(),
                                  top_left_.translation(),
                                  color_);

  // Show each bin
  for (BinObjectMap::const_iterator bin_it = bins_.begin(); bin_it != bins_.end(); bin_it++)
  {
    bin_it->second->visualize(bottom_right_);
  }

  // Create side walls of shelf
  for (std::size_t i = 0; i < shelf_parts_.size(); ++i)
  {
    shelf_parts_[i].visualize(bottom_right_);
  }
}

bool ShelfObject::createCollisionBodies(const std::string& focus_bin_name, bool just_frame) const
{
  // Publish in batch
  visual_tools_->enableBatchPublishing(true);

  // Create side walls of shelf
  for (std::size_t i = 0; i < shelf_parts_.size(); ++i)
  {
    shelf_parts_[i].createCollisionBodies(bottom_right_);
  }

  // Show each bin except the focus on
  if (!just_frame)
  {
    // Send all the shapes for the shelf first, then secondly the products
    BinObjectPtr focus_bin;
    
    for (BinObjectMap::const_iterator bin_it = bins_.begin(); bin_it != bins_.end(); bin_it++)
    {
      // Check if this is the focused bin 
      if (bin_it->second->getName() == focus_bin_name)
      {
        focus_bin = bin_it->second; // save for later
      }
      else
      {
        // Fill in bin as simple rectangle (disabled mode)
        bin_it->second->createCollisionBodies(bottom_right_);
      }
    }

    if (focus_bin)
    {
      // Add products to shelves
      focus_bin->createCollisionBodiesProducts(bottom_right_);
    }
  }

  return visual_tools_->triggerBatchPublishAndDisable();
}

bool ShelfObject::createCollisionShelfDetailed() const
{
  // Get image path based on package name

  // Calculate offset
  Eigen::Affine3d offset;
  offset = Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitX())
    * Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
  offset.translation().x() = bottom_right_.translation().x() + SHELF_DEPTH / 2.0;
  offset.translation().y() = 0;

  // Publish mesh
  if (!visual_tools_->publishCollisionMesh(offset, name_, mesh_path_, color_))
    return false;

  // Add products to shelves
  for (BinObjectMap::const_iterator bin_it = bins_.begin(); bin_it != bins_.end(); bin_it++)
  {
    bin_it->second->createCollisionBodiesProducts(bottom_right_);
  }
}

BinObjectMap& ShelfObject::getBins()
{
  return bins_;
}

ProductObjectPtr ShelfObject::getProduct(const std::string &bin_name, const std::string &product_name)
{
  // Find correct bin
  BinObjectPtr bin = bins_[bin_name];
  
  if (!bin)
  {
    ROS_WARN_STREAM_NAMED("shelf","Unable to find product " << product_name << " in bin " << bin_name << " in the database");
    return ProductObjectPtr();
  }

  ProductObjectPtr product = bin->getProduct(product_name);

  if (!product)
  {
    ROS_WARN_STREAM_NAMED("shelf","Unable to find product " << product_name << " in bin " << bin_name << " in the database");
    return ProductObjectPtr();
  }

  return product;
}

bool ShelfObject::deleteProduct(const std::string &bin_name, const std::string &product_name)
{
  // Find correct bin
  BinObjectPtr bin = bins_[bin_name];

  std::vector<ProductObjectPtr>& products = bin->getProducts();
  // Find correct product
  for (std::size_t prod_id = 0; prod_id < products.size(); ++prod_id)
  {
    if (products[prod_id]->getName() == product_name)
    {
      ROS_DEBUG_STREAM_NAMED("shelf","Found product to delete: " << product_name);
      products.erase(products.begin() + prod_id);
      return true;
    }
  }

  ROS_WARN_STREAM_NAMED("shelf","Unable to delete product " << product_name << " in bin " << bin_name << " in the database");
  return false;  
}

// -------------------------------------------------------------------------------------------------
// Product Object
// -------------------------------------------------------------------------------------------------
ProductObject::ProductObject(moveit_visual_tools::MoveItVisualToolsPtr visual_tools, 
                             const rviz_visual_tools::colors &color,
                             const std::string &name,
                             const std::string &package_path)
  : Rectangle( visual_tools, color, name )
{
  // Ensure the name is unique
  static std::size_t product_id = 0;
  product_id++;
  collision_object_name_ = name + "_" + boost::lexical_cast<std::string>(product_id);

  // Cache the object's mesh
  mesh_path_ = "file://" + package_path + "/meshes/" + name_ + "/recommended.stl";

  ROS_DEBUG_STREAM_NAMED("shelf","Creating collision product with name " << collision_object_name_ << " from mesh " << mesh_path_);
  
}

std::string ProductObject::getCollisionName() const
{
  return collision_object_name_;
}

void ProductObject::setCollisionName(std::string name)
{
  collision_object_name_ = name;
}

bool ProductObject::createCollisionBodies(const Eigen::Affine3d &trans) const
{ 
  // Show product mesh
  if (!visual_tools_->publishCollisionMesh(transform(bottom_right_, trans),
                                           collision_object_name_, 
                                           mesh_path_, color_ ))
  {
    // Fall back to publishing rectangles
    visual_tools_->publishCollisionRectangle( transform(bottom_right_, trans).translation(),
                                              transform(top_left_, trans).translation(),
                                              collision_object_name_, color_ );
    return false;
  }

  return true;
}

} // namespace
