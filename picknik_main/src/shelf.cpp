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

#include <picknik_main/shelf.h>

// Parameter loading
#include <rviz_visual_tools/ros_param_utilities.h>

namespace picknik_main
{

// -------------------------------------------------------------------------------------------------
// Bin Object
// -------------------------------------------------------------------------------------------------

BinObject::BinObject(VisualsPtr visuals,
                     const rvt::colors &color,
                     const std::string &name)
  : RectangleObject(visuals, color, name)
{
}

bool BinObject::visualize(const Eigen::Affine3d& trans) const
{
  // Show bin
  //visuals_->visual_tools_display_->publishCuboid( transform(bottom_right_, trans).translation(),
  //                                 transform(top_left_, trans).translation(),
  //                                 color_);

  // Show products
  for (std::size_t product_id = 0; product_id < products_.size(); ++product_id)
  {
    products_[product_id]->visualize(trans * bottom_right_); // send transform from world to bin
  }

  return true;
}

bool BinObject::visualizeAxis(const Eigen::Affine3d& trans, VisualsPtr visuals) const
{
  // Show coordinate system
  visuals_->visual_tools_->publishAxis( transform(bottom_right_, trans) );

  // Show label
  Eigen::Affine3d text_location = transform( bottom_right_, trans);

  text_location.translation() += Eigen::Vector3d(0,getWidth()/2.0, getHeight()*0.9);

  visuals->visual_tools_->publishText( text_location, name_, rvt::BLACK, rvt::REGULAR, false);

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

void BinObject::getProducts(std::vector<std::string> &products)
{
  products.clear();
  for (std::size_t i = 0; i < products_.size(); ++i)
  {
    products.push_back(products_[i]->getName());
  }
}

ProductObjectPtr BinObject::getProduct(const std::string& name)
{
  // Find correct product
  for (std::size_t prod_id = 0; prod_id < products_.size(); ++prod_id)
  {
    if (products_[prod_id]->getName() == name)
    {
      return products_[prod_id];
    }
  }

  return ProductObjectPtr();
}

// -------------------------------------------------------------------------------------------------
// Shelf Object
// -------------------------------------------------------------------------------------------------

ShelfObject::ShelfObject(VisualsPtr visuals,
                         const rvt::colors &color, const std::string &name)
  : RectangleObject(visuals, color, name)
{
}

bool ShelfObject::initialize(const std::string &package_path, ros::NodeHandle &nh)
{
  const std::string parent_name = "shelf"; // for namespacing logging messages

  // Loaded shelf parameter values
  std::vector<double> world_to_shelf_transform_doubles;
  if (!rvt::getDoubleParameters(parent_name, nh, "world_to_shelf_transform", world_to_shelf_transform_doubles))
    return false;
  if (!rvt::convertDoublesToEigen(parent_name, world_to_shelf_transform_doubles, world_to_shelf_transform_))
    return false;
  if (!rvt::getDoubleParameter(parent_name, nh, "shelf_width", shelf_width_))
    return false;
  if (!rvt::getDoubleParameter(parent_name, nh, "shelf_height", shelf_height_))
    return false;
  if (!rvt::getDoubleParameter(parent_name, nh, "shelf_depth", shelf_depth_))
    return false;
  if (!rvt::getDoubleParameter(parent_name, nh, "shelf_wall_width", shelf_wall_width_))
    return false;
  if (!rvt::getDoubleParameter(parent_name, nh, "shelf_inner_wall_width", shelf_inner_wall_width_))
    return false;
  if (!rvt::getDoubleParameter(parent_name, nh, "shelf_surface_thickness", shelf_surface_thickness_))
    return false;
  if (!rvt::getDoubleParameter(parent_name, nh, "first_bin_from_bottom", first_bin_from_bottom_))
    return false;
  if (!rvt::getDoubleParameter(parent_name, nh, "first_bin_from_right", first_bin_from_right_))
    return false;

  // Loaded bin parameter values
  if (!rvt::getDoubleParameter(parent_name, nh, "bin_right_width", bin_right_width_))
    return false;
  if (!rvt::getDoubleParameter(parent_name, nh, "bin_middle_width", bin_middle_width_))
    return false;
  if (!rvt::getDoubleParameter(parent_name, nh, "bin_left_width", bin_left_width_))
    return false;
  if (!rvt::getDoubleParameter(parent_name, nh, "bin_short_height", bin_short_height_))
    return false;
  if (!rvt::getDoubleParameter(parent_name, nh, "bin_tall_height", bin_tall_height_))
    return false;
  if (!rvt::getDoubleParameter(parent_name, nh, "bin_depth", bin_depth_))
    return false;
  if (!rvt::getDoubleParameter(parent_name, nh, "bin_top_margin", bin_top_margin_))
    return false;
  if (!rvt::getDoubleParameter(parent_name, nh, "bin_left_margin", bin_left_margin_))
    return false;
  if (!rvt::getDoubleParameter(parent_name, nh, "num_bins", num_bins_))
    return false;

  // Goal bin
  if (!rvt::getDoubleParameter(parent_name, nh, "goal_bin_x", goal_bin_x_))
    return false;
  if (!rvt::getDoubleParameter(parent_name, nh, "goal_bin_y", goal_bin_y_))
    return false;
  if (!rvt::getDoubleParameter(parent_name, nh, "goal_bin_z", goal_bin_z_))
    return false;

  // Side limits (walls)
  if (!rvt::getDoubleParameter(parent_name, nh, "left_wall_y", left_wall_y_))
    return false;
  if (!rvt::getDoubleParameter(parent_name, nh, "right_wall_y", right_wall_y_))
    return false;
  if (!rvt::getDoubleParameter(parent_name, nh, "collision_wall_safety_margin", collision_wall_safety_margin_))
    return false;

  // Calculate shelf corners for *this ShelfObject
  bottom_right_ = world_to_shelf_transform_;
  // bottom_right_.translation().x() = world_to_shelf_transform_.translation().x();
  // bottom_right_.translation().y() = -shelf_width_/2.0;
  // bottom_right_.translation().z() = 0;
  top_left_.translation().x() = bottom_right_.translation().x() + shelf_depth_;
  top_left_.translation().y() = shelf_width_/2.0;
  top_left_.translation().z() = shelf_height_;


  // Create shelf parts -----------------------------
  // Note: bottom right is at 0,0,0

  Eigen::Affine3d bottom_right;
  Eigen::Affine3d top_left;
  // TODO delete first_bin_from_right_

  // Base
  shelf_parts_.push_back(RectangleObject(visuals_, color_, "base"));
  RectangleObject &base = shelf_parts_.back();
  top_left = base.getTopLeft();
  top_left.translation().x() += shelf_depth_;
  top_left.translation().y() += shelf_width_;
  top_left.translation().z() += first_bin_from_bottom_;
  base.setTopLeft(top_left);
  
  // Extend base to protect robot from table
  bool immitation_table_mount = false;
  if (immitation_table_mount)
  {
    bottom_right = Eigen::Affine3d::Identity();
    bottom_right.translation().x() -= 1.0;
    base.setBottomRight(bottom_right);
  }

  // Shelf Walls
  double previous_y = shelf_wall_width_ * 0.5;
  double this_shelf_wall_width;
  double this_bin_width;
  double bin_z;
  std::size_t bin_id = 0;
  for (std::size_t wall_id = 0; wall_id < 4; ++wall_id)
  {
    const std::string wall_name = "wall_" + boost::lexical_cast<std::string>(wall_id);
    shelf_parts_.push_back(RectangleObject(visuals_, color_, wall_name));
    RectangleObject &wall = shelf_parts_.back();

    // Choose what wall width the current bin is
    if (wall_id == 1 || wall_id == 2)
      this_shelf_wall_width = shelf_inner_wall_width_;
    else
      this_shelf_wall_width = shelf_wall_width_;

    // Shelf Wall Geometry 
    bottom_right = wall.getBottomRight();
    bottom_right.translation().x() = 0;
    bottom_right.translation().y() = previous_y - this_shelf_wall_width * 0.5;
    bottom_right.translation().z() = first_bin_from_bottom_;
    wall.setBottomRight(bottom_right);

    top_left = wall.getTopLeft();
    top_left.translation().x() = shelf_depth_;
    top_left.translation().y() = previous_y + this_shelf_wall_width * 0.5;
    top_left.translation().z() = shelf_height_;
    wall.setTopLeft(top_left);

    // Choose what width the current bin is
    if (wall_id == 0)
      this_bin_width = bin_right_width_;
    else if (wall_id == 1)
      this_bin_width = bin_middle_width_;
    else if (wall_id == 2)
      this_bin_width = bin_left_width_;
    else
      this_bin_width = 0; // doesn't matter

    // Increment the y location
    previous_y += this_bin_width + this_shelf_wall_width;


    // Create a column of shelf bins from top to bottom
    if (wall_id < 3)
    {
      bin_z = first_bin_from_bottom_;      
      insertBinHelper(wall_id + 0, bin_tall_height_, this_bin_width, top_left.translation().y(), bin_z); // bottom rop
      bin_z += bin_tall_height_;
      insertBinHelper(wall_id + 3, bin_short_height_, this_bin_width, top_left.translation().y(), bin_z); // middle bottom row
      bin_z += bin_short_height_;
      insertBinHelper(wall_id + 6, bin_short_height_, this_bin_width, top_left.translation().y(), bin_z); // middle top row
      bin_z += bin_short_height_;
      insertBinHelper(wall_id + 9, bin_tall_height_, this_bin_width, top_left.translation().y(), bin_z); // top row
    }
  }

  // Shelves
  double previous_z = first_bin_from_bottom_;
  double this_bin_height;
  for (std::size_t i = 0; i < 5; ++i)
  {
    const std::string shelf_name = "surface_" + boost::lexical_cast<std::string>(i);
    shelf_parts_.push_back(RectangleObject(visuals_, color_, shelf_name));
    RectangleObject &shelf = shelf_parts_.back();

    // Geometry
    top_left = shelf.getTopLeft();
    top_left.translation().x() = shelf_width_;
    top_left.translation().y() = shelf_depth_;
    top_left.translation().z() = previous_z;
    shelf.setTopLeft(top_left);

    bottom_right = shelf.getBottomRight();
    bottom_right.translation().z() = shelf.getTopLeft().translation().z() - shelf_surface_thickness_; // add thickenss
    shelf.setBottomRight(bottom_right);

    // Choose what height the current bin is
    if (i == 1 || i == 2)
      this_bin_height = bin_short_height_;
    else
      this_bin_height = bin_tall_height_;

    // Increment the z location
    previous_z += this_bin_height; // flush with top shelf
  }

  // Goal bin
  goal_bin_.reset(new MeshObject(visuals_, rvt::RED, "goal_bin"));
  Eigen::Affine3d goal_bin_centroid = Eigen::Affine3d::Identity();
  goal_bin_centroid.translation().x() = goal_bin_x_;
  goal_bin_centroid.translation().y() = goal_bin_y_;
  goal_bin_centroid.translation().z() = goal_bin_z_;
  goal_bin_->setCentroid(goal_bin_centroid);

  goal_bin_->setHighResMeshPath("file://" + package_path + "/meshes/goal_bin/goal_bin.stl");
  goal_bin_->setCollisionMeshPath("file://" + package_path + "/meshes/goal_bin/goal_bin.stl");


  // Side limit walls
  if (left_wall_y_ > 0.001 || left_wall_y_ < -0.001)
  {
    left_wall_.reset(new RectangleObject(visuals_, rvt::YELLOW, "left_wall"));
    bottom_right = left_wall_->getBottomRight();
    bottom_right.translation().x() = 1;
    bottom_right.translation().y() = left_wall_y_;
    bottom_right.translation().z() = 0;
    left_wall_->setBottomRight(bottom_right);
    top_left = left_wall_->getBottomRight();
    top_left.translation().x() = -1;
    top_left.translation().y() = left_wall_y_ + shelf_wall_width_;
    top_left.translation().z() = shelf_height_;
    left_wall_->setTopLeft(top_left);
  }
  if (right_wall_y_ > 0.001 || right_wall_y_ < -0.001)
  {
    right_wall_.reset(new RectangleObject(visuals_, rvt::YELLOW, "right_wall"));
    bottom_right = right_wall_->getBottomRight();
    bottom_right.translation().x() = 1;
    bottom_right.translation().y() = right_wall_y_;
    bottom_right.translation().z() = 0;
    right_wall_->setBottomRight(bottom_right);
    top_left = right_wall_->getBottomRight();
    top_left.translation().x() = -1;
    top_left.translation().y() = right_wall_y_ + shelf_wall_width_;
    top_left.translation().z() = shelf_height_;
    right_wall_->setTopLeft(top_left);
  }
  // Front wall limit
  static const double INTERNAL_WIDTH = 0.1;
  front_wall_.reset(new RectangleObject(visuals_, rvt::YELLOW, "front_wall"));
  bottom_right = front_wall_->getBottomRight();
  bottom_right.translation().x() = -collision_wall_safety_margin_;
  bottom_right.translation().y() = - shelf_width_ / 2.0;
  bottom_right.translation().z() = 0;
  front_wall_->setBottomRight(bottom_right);
  top_left = front_wall_->getBottomRight();
  top_left.translation().x() = INTERNAL_WIDTH;
  top_left.translation().y() = shelf_width_ * 1.5;
  top_left.translation().z() = shelf_height_;
  front_wall_->setTopLeft(top_left);  

  // Load mesh file name
  high_res_mesh_path_ = "file://" + package_path + "/meshes/kiva_pod/meshes/pod_lowres.stl";
  collision_mesh_path_ = high_res_mesh_path_;

  // Calculate offset for high-res mesh
  //high_res_mesh_offset_ = Eigen::Affine3d::Identity();
  high_res_mesh_offset_ = Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitX())
    * Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
  high_res_mesh_offset_.translation().x() = shelf_depth_ / 2.0;
  high_res_mesh_offset_.translation().y() = shelf_width_ / 2.0;
  high_res_mesh_offset_.translation().z() = 0; //first_bin_from_bottom_ - 0.81; // TODO remove this height - only for temp table setup


  // Calculate offset - FOR COLLISION
  Eigen::Affine3d offset;
  offset = Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitX())
    * Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
  offset.translation().x() = bottom_right_.translation().x() + shelf_depth_ / 2.0;
  offset.translation().y() = 0;


  return true;
}

bool ShelfObject::insertBinHelper(int bin_id, double height, double width, double wall_y, double bin_z)
{
  const std::string bin_name = "bin_" + boost::lexical_cast<std::string>((char)(65 + num_bins_ - bin_id - 1)); // reverse the lettering
  //std::string bin_name = "bin_" + boost::lexical_cast<std::string>(bin_id);
  ROS_DEBUG_STREAM_NAMED("shelf","Creating bin '" << bin_name << "' with id " << bin_id);

  BinObjectPtr new_bin(new BinObject(visuals_, rvt::MAGENTA, bin_name));
  bins_.insert( std::pair<std::string, BinObjectPtr>(bin_name, new_bin));


  // Calculate bottom right
  Eigen::Affine3d bottom_right = Eigen::Affine3d::Identity();
  bottom_right.translation().x() = 0;
  bottom_right.translation().y() = wall_y;
  bottom_right.translation().z() = bin_z;
  new_bin->setBottomRight(bottom_right);

  // Calculate top left
  Eigen::Affine3d top_left = Eigen::Affine3d::Identity();
  top_left.translation().x() += bin_depth_;
  top_left.translation().y() += wall_y + width;
  top_left.translation().z() += bin_z + height - shelf_surface_thickness_;
  new_bin->setTopLeft(top_left);


  return true;
}

bool ShelfObject::visualizeAxis(VisualsPtr visuals) const
{
  // Show coordinate system
  visuals_->visual_tools_->publishAxis( bottom_right_ );

  // Show each bin
  for (BinObjectMap::const_iterator bin_it = bins_.begin(); bin_it != bins_.end(); bin_it++)
  {
    bin_it->second->visualizeAxis(bottom_right_, visuals);
  }
}

bool ShelfObject::visualize(bool show_products) const
{
  Eigen::Affine3d high_res_pose = bottom_right_ * high_res_mesh_offset_;

  // Publish mesh
  if (!visuals_->visual_tools_display_->publishMesh(high_res_pose, high_res_mesh_path_, rvt::BROWN, 1, "Shelf"))
    return false;

  // Show each bin's products
  if (show_products)
  {
    for (BinObjectMap::const_iterator bin_it = bins_.begin(); bin_it != bins_.end(); bin_it++)
    {
      bin_it->second->visualize(bottom_right_);
    }
  }

  // Show goal bin
  goal_bin_->visualize(bottom_right_);

  // Show wall limits
  if (left_wall_)
    left_wall_->visualize(bottom_right_);
  if (right_wall_)
    right_wall_->visualize(bottom_right_);

  // Show workspace
  static const double GAP_TO_SHELF = 0.1;
  const double x1 = world_to_shelf_transform_.translation().x() - GAP_TO_SHELF;
  const double x2 = world_to_shelf_transform_.translation().x() - GAP_TO_SHELF - 2;
  const Eigen::Vector3d point1(x1, 1, 0);
  const Eigen::Vector3d point2(x2, -1, 0.001);
  visuals_->visual_tools_display_->publishCuboid(point1, point2, rvt::DARK_GREY);
}

bool ShelfObject::createCollisionBodies(const std::string& focus_bin_name, bool only_show_shelf_frame, bool show_all_products)
{
  // Publish in batch
  visuals_->visual_tools_->enableBatchPublishing(true);

  // Show full resolution shelf -----------------------------------------------------------------
  //createCollisionShelfDetailed();

  // Show simple version of shelf -----------------------------------------------------------------

  // Create side walls of shelf
  for (std::size_t i = 0; i < shelf_parts_.size(); ++i)
  {
    shelf_parts_[i].createCollisionBodies(bottom_right_);
  }
  
  // Show each bin except the focus on
  if (!only_show_shelf_frame)
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
      else if (!show_all_products)
      {
        // Fill in bin as simple rectangle (disabled mode)
        bin_it->second->createCollisionBodies(bottom_right_);
      }

      // Optionally add all products to shelves
      if (show_all_products)
      {
        bin_it->second->createCollisionBodiesProducts(bottom_right_);
      }
    }

    if (focus_bin && !show_all_products) // don't redisplay products if show_all_products is true
    {
      // Add products to shelves
      focus_bin->createCollisionBodiesProducts(bottom_right_);
    }
  }

  // Show goal bin
  goal_bin_->createCollisionBodies(bottom_right_);
  goal_bin_->visualizeAxis(bottom_right_);

  // Show wall limits
  if (left_wall_)
    left_wall_->createCollisionBodies(bottom_right_);
  if (right_wall_)
    right_wall_->createCollisionBodies(bottom_right_);

  return visuals_->visual_tools_->triggerBatchPublishAndDisable();
}

bool ShelfObject::createCollisionShelfDetailed()
{
  ROS_DEBUG_STREAM_NAMED("shelf","Creating collision body with name " << collision_object_name_);

  // Publish mesh
  if (!visuals_->visual_tools_->publishCollisionMesh(high_res_mesh_offset_, collision_object_name_, high_res_mesh_path_, rvt::BLUE)) //color_))
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

BinObjectPtr ShelfObject::getBin(std::size_t bin_id)
{
  const std::string bin_name = "bin_" + boost::lexical_cast<std::string>((char)(65 + bin_id));
  std::cout << "bin_name: " << bin_name << std::endl;
  return bins_[bin_name];
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
ProductObject::ProductObject(VisualsPtr visuals,
                             const rvt::colors &color,
                             const std::string &name,
                             const std::string &package_path)
  : MeshObject( visuals, color, name )
{
  // Prepend the collision object type to the collision object name
  collision_object_name_ = "product_" + collision_object_name_;

  // Cache the object's mesh
  high_res_mesh_path_ = "file://" + package_path + "/meshes/products/" + name_ + "/recommended.dae";
  collision_mesh_path_ = "file://" + package_path + "/meshes/products/" + name_ + "/collision.stl";

  // Debug
  ROS_DEBUG_STREAM_NAMED("shelf","Creating collision product with name " << collision_object_name_ << " from mesh: " 
                         << high_res_mesh_path_ << "\n Collision mesh: " << collision_mesh_path_);
}

ProductObject::ProductObject(const ProductObject& copy)
  : MeshObject( copy )
{ 
}

Eigen::Affine3d ProductObject::getWorldPose(const ShelfObjectPtr& shelf, const BinObjectPtr& bin)
{
  return shelf->getBottomRight() * bin->getBottomRight() * getCentroid();
}

} // namespace
