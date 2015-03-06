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

namespace picknik_main
{

// -------------------------------------------------------------------------------------------------
// Rectangle Object
// -------------------------------------------------------------------------------------------------

RectangleObject::RectangleObject(VisualsPtr visuals,
                     const rvt::colors &color, const std::string &name)
  : visuals_(visuals)
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

bool RectangleObject::visualize(const Eigen::Affine3d& trans) const
{
  if (!high_res_mesh_path_.empty())
  {
    Eigen::Affine3d centroid;
    getCentroid(centroid);

    // Show axis
    visuals_->visual_tools_display_->publishAxis(transform(centroid, trans), 0.1/2, 0.01/2);

    // Show full resolution mesh
    return visuals_->visual_tools_display_->publishMesh(transform(centroid, trans), high_res_mesh_path_);
  }

  // Show simple geometric shape
  return visuals_->visual_tools_display_->publishRectangle( transform(bottom_right_, trans).translation(),
                                                            transform(top_left_, trans).translation(),
                                                            color_);
}

bool RectangleObject::createCollisionBodies(const Eigen::Affine3d &trans) const
{
  ROS_DEBUG_STREAM_NAMED("shelf","Creating collision rectangle with name " << name_);

  // Check if mesh is provided
  if (!collision_mesh_path_.empty())
  {
    Eigen::Affine3d centroid;
    getCentroid(centroid);

    // Show mesh
    visuals_->visual_tools_->publishCollisionMesh(transform(centroid, trans), collision_object_name_, 
                                                  collision_mesh_path_, color_);
    return true;
  }

  // Just use basic rectangle
  visuals_->visual_tools_->publishCollisionRectangle( transform(bottom_right_, trans).translation(),
                                                      transform(top_left_, trans).translation(),
                                                      name_, color_ );
  return true;
}

void RectangleObject::getCentroid(Eigen::Affine3d &pose) const
{
  pose = bottom_right_;
  pose.translation().x() += getDepth() / 2.0;
  pose.translation().y() += getWidth() / 2.0;
  pose.translation().z() += getHeight() / 2.0;
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

std::string RectangleObject::getName() const
{
  return name_;
}

void RectangleObject::setName(std::string name)
{
  name_ = name;
}

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
  //visuals_->visual_tools_display_->publishRectangle( transform(bottom_right_, trans).translation(),
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

}

bool BinObject::createCollisionBodies(const Eigen::Affine3d &trans) const
{
  ROS_DEBUG_STREAM_NAMED("shelf","Creating collision bin " << name_);

  visuals_->visual_tools_->publishCollisionRectangle( transform(bottom_right_, trans).translation(),
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

ShelfObject::ShelfObject(VisualsPtr visuals,
                         const rvt::colors &color, const std::string &name)
  : RectangleObject(visuals, color, name)
{
}

bool ShelfObject::initialize(const std::string &package_path, ros::NodeHandle &nh)
{
  // Loaded shelf parameter values
  if (!getDoubleParameter(nh,"shelf_distance_from_robot", shelf_distance_from_robot_))
    return false;
  if (!getDoubleParameter(nh,"shelf_width", shelf_width_))
    return false;
  if (!getDoubleParameter(nh,"shelf_height", shelf_height_))
    return false;
  if (!getDoubleParameter(nh,"shelf_depth", shelf_depth_))
    return false;
  if (!getDoubleParameter(nh,"shelf_wall_width", shelf_wall_width_))
    return false;
  if (!getDoubleParameter(nh,"first_bin_from_bottom", first_bin_from_bottom_))
    return false;
  if (!getDoubleParameter(nh,"first_bin_from_right", first_bin_from_right_))
    return false;

  // Loaded bin parameter values
  if (!getDoubleParameter(nh,"bin_width", bin_width_))
    return false;
  if (!getDoubleParameter(nh,"bin_middle_width", bin_middle_width_))
    return false;
  if (!getDoubleParameter(nh,"bin_short_height", bin_short_height_))
    return false;
  if (!getDoubleParameter(nh,"bin_tall_height", bin_tall_height_))
    return false;
  if (!getDoubleParameter(nh,"bin_depth", bin_depth_))
    return false;
  if (!getDoubleParameter(nh,"bin_top_margin", bin_top_margin_))
    return false;
  if (!getDoubleParameter(nh,"bin_left_margin", bin_left_margin_))
    return false;
  if (!getDoubleParameter(nh,"num_bins", num_bins_))
    return false;

  // Goal bin
  if (!getDoubleParameter(nh,"goal_bin_x", goal_bin_x_))
    return false;
  if (!getDoubleParameter(nh,"goal_bin_y", goal_bin_y_))
    return false;
  if (!getDoubleParameter(nh,"goal_bin_z", goal_bin_z_))
    return false;

  // Calculate shelf corners
  bottom_right_.translation().x() = shelf_distance_from_robot_;
  bottom_right_.translation().y() = -shelf_width_/2.0;
  bottom_right_.translation().z() = 0;
  top_left_.translation().x() = bottom_right_.translation().x() + shelf_depth_;
  top_left_.translation().y() = shelf_width_/2.0;
  top_left_.translation().z() = shelf_height_;

  // Calculate first bin location
  Eigen::Affine3d bin1_bottom_right = Eigen::Affine3d::Identity();
  bin1_bottom_right.translation().y() = first_bin_from_right_;
  bin1_bottom_right.translation().z() = first_bin_from_bottom_;

  // Create each bin
  std::size_t bin_id = 0;
  double this_bin_width;
  double this_bin_height;
  for (double z = bin1_bottom_right.translation().z();
       z < shelf_height_; /* Increment */)
  {

    for (double y = bin1_bottom_right.translation().y();
         y < shelf_width_ - bin_width_ * 0.9; /* Increment */)
    {
      // Create new bin
      std::string bin_name = "bin_" + boost::lexical_cast<std::string>((char)(65 + num_bins_ - bin_id - 1)); // reverse the lettering
      ROS_DEBUG_STREAM_NAMED("shelf","Creating bin '" << bin_name << "' with id " << bin_id);

      insertBinHelper( rvt::YELLOW, bin_name );

      // Choose what height the current bin is
      if (bin_id == 3 || bin_id == 4 || bin_id == 5 || bin_id == 6 || bin_id == 7 || bin_id == 8)
        this_bin_height = bin_short_height_;
      else
        this_bin_height = bin_tall_height_;

      // Calculate new bin location
      bins_[bin_name]->bottom_right_.translation().x() = bin1_bottom_right.translation().x();
      bins_[bin_name]->bottom_right_.translation().y() = y;
      bins_[bin_name]->bottom_right_.translation().z() = z;

      bins_[bin_name]->top_left_ = bins_[bin_name]->bottom_right_;
      bins_[bin_name]->top_left_.translation().x() += bin_depth_;
      bins_[bin_name]->top_left_.translation().y() += bin_width_;
      bins_[bin_name]->top_left_.translation().z() += this_bin_height;

      // Choose what width the current bin is
      if (bin_id == 1 || bin_id == 4 || bin_id == 7 || bin_id == 10)
        this_bin_width = bin_middle_width_;
      else
        this_bin_width = bin_width_;

      // Increment
      y = y + this_bin_width + shelf_wall_width_;

      bin_id ++;
      if (bin_id >= num_bins_)
        break;
    }

    // Increment
    z = z + this_bin_height;

    if (bin_id >= num_bins_)
      break;
  }

  // Create shelf parts -----------------------------

  // Base
  // Note: bottom right is at 0,0,0
  shelf_parts_.push_back(RectangleObject(visuals_, color_, "base"));
  RectangleObject &base = shelf_parts_[shelf_parts_.size()-1];
  base.top_left_.translation().x() += shelf_depth_;
  base.top_left_.translation().y() += shelf_width_;
  base.top_left_.translation().z() += first_bin_from_bottom_;
  base.bottom_right_.translation().x() -= 1;

  // Walls
  double previous_y = shelf_wall_width_ * 0.5;
  for (std::size_t i = 0; i < 4; ++i)
  {
    // Note: bottom right is at 0,0,0
    const std::string wall_name = "wall_" + boost::lexical_cast<std::string>(i);
    shelf_parts_.push_back(RectangleObject(visuals_, color_, wall_name));
    RectangleObject &wall = shelf_parts_[shelf_parts_.size()-1];
    // Geometry
    wall.bottom_right_.translation().x() = 0;
    wall.bottom_right_.translation().y() = previous_y - shelf_wall_width_ * 0.5;
    wall.bottom_right_.translation().z() = first_bin_from_bottom_;
    wall.top_left_.translation().x() = shelf_depth_;
    wall.top_left_.translation().y() = previous_y + shelf_wall_width_ * 0.5;
    wall.top_left_.translation().z() = shelf_height_;

    // Choose what width the current bin is
    if (i == 1)
      this_bin_width = bin_middle_width_;
    else
      this_bin_width = bin_width_;

    // Increment the y location
    previous_y += this_bin_width + shelf_wall_width_;
  }

  // Shelves
  double previous_z = first_bin_from_bottom_;
  for (std::size_t i = 0; i < 5; ++i)
  {
    const std::string shelf_name = "shelf_" + boost::lexical_cast<std::string>(i);
    shelf_parts_.push_back(RectangleObject(visuals_, color_, shelf_name));
    RectangleObject &shelf = shelf_parts_[shelf_parts_.size()-1];

    // Geometry
    shelf.top_left_.translation().x() = shelf_width_;
    shelf.top_left_.translation().y() = shelf_depth_;
    shelf.top_left_.translation().z() = previous_z;
    shelf.bottom_right_.translation().z() = shelf.top_left_.translation().z() - shelf_wall_width_; // add thickenss

    // Choose what height the current bin is
    if (i == 1 || i == 2)
      this_bin_height = bin_short_height_;
    else
      this_bin_height = bin_tall_height_;

    // Increment the z location
    previous_z += this_bin_height; // flush with top shelf
  }

  // Goal bin
  goal_bin_.reset(new RectangleObject(visuals_, rvt::RED, "goal_bin"));
  goal_bin_->bottom_right_.translation().x() = goal_bin_x_;
  goal_bin_->bottom_right_.translation().y() = goal_bin_y_;
  goal_bin_->bottom_right_.translation().z() = goal_bin_z_;

  goal_bin_->top_left_ = goal_bin_->bottom_right_;
  goal_bin_->top_left_.translation().x() += 0.61595; // goal bin depth (long side)
  goal_bin_->top_left_.translation().y() += 0.37465; // goal bin width
  goal_bin_->top_left_.translation().z() += 0.2032; // goal bin height

  goal_bin_->setHighResMeshPath("file://" + package_path + "/meshes/goal_bin/goal_bin.stl");
  goal_bin_->setCollisionMeshPath("file://" + package_path + "/meshes/goal_bin/goal_bin.stl");

  // Load mesh file name
  high_res_mesh_path_ = "file://" + package_path + "/meshes/kiva_pod/meshes/pod_lowres.stl";

  return true;
}

bool ShelfObject::insertBinHelper(rvt::colors color, const std::string& name)
{
  BinObjectPtr new_bin(new BinObject(visuals_, color, name));

  bins_.insert( std::pair<std::string, BinObjectPtr>(name, new_bin));

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

bool ShelfObject::visualize() const
{
  // Calculate offset
  Eigen::Affine3d offset;
  offset = Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitX())
    * Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
  offset.translation().x() = bottom_right_.translation().x() + shelf_depth_ / 2.0;
  offset.translation().y() = 0;
  offset.translation().z() = first_bin_from_bottom_ - 0.81; // TODO remove this height - only for temp table setup

  // Publish mesh
  if (!visuals_->visual_tools_display_->publishMesh(offset, high_res_mesh_path_, rvt::BROWN, 1, "Shelf"))
    return false;

  // Show each bin
  for (BinObjectMap::const_iterator bin_it = bins_.begin(); bin_it != bins_.end(); bin_it++)
  {
    bin_it->second->visualize(bottom_right_);
  }

  // Show goal bin
  goal_bin_->visualize(bottom_right_);

  // Show workspace
  static const double GAP_TO_SHELF = 0.1;
  const double x1 = shelf_distance_from_robot_ - GAP_TO_SHELF;
  const double x2 = shelf_distance_from_robot_ - GAP_TO_SHELF - 2;
  const Eigen::Vector3d point1(x1, 1, 0);
  const Eigen::Vector3d point2(x2, -1, 0.001);
  visuals_->visual_tools_display_->publishRectangle(point1, point2, rvt::DARK_GREY);
}

bool ShelfObject::createCollisionBodies(const std::string& focus_bin_name, bool just_frame, bool show_all_products) const
{
  // Publish in batch
  visuals_->visual_tools_->enableBatchPublishing(true);

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
      else if (!show_all_products)
      {
        // Fill in bin as simple rectangle (disabled mode)
        bin_it->second->createCollisionBodies(bottom_right_);
      }

      // Optionall add all products to shelves
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

  return visuals_->visual_tools_->triggerBatchPublishAndDisable();
}

bool ShelfObject::createCollisionShelfDetailed() const
{
  // Get image path based on package name

  // Calculate offset
  Eigen::Affine3d offset;
  offset = Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitX())
    * Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
  offset.translation().x() = bottom_right_.translation().x() + shelf_depth_ / 2.0;
  offset.translation().y() = 0;

  // Publish mesh
  if (!visuals_->visual_tools_->publishCollisionMesh(offset, name_, high_res_mesh_path_, color_))
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
ProductObject::ProductObject(VisualsPtr visuals,
                             const rvt::colors &color,
                             const std::string &name,
                             const std::string &package_path)
  : RectangleObject( visuals, color, name )
{
  // Ensure the name is unique
  static std::size_t product_id = 0;
  product_id++;
  collision_object_name_ = name + "_" + boost::lexical_cast<std::string>(product_id);

  // Cache the object's mesh
  high_res_mesh_path_ = "file://" + package_path + "/meshes/products/" + name_ + "/recommended.dae";
  collision_mesh_path_ = "file://" + package_path + "/meshes/products/" + name_ + "/collision.stl";

  // Debug
  ROS_DEBUG_STREAM_NAMED("shelf","Creating collision product with name " << collision_object_name_ << " from mesh " 
                         << high_res_mesh_path_ << " and collision mesh " << collision_mesh_path_);
}

std::string ProductObject::getCollisionName() const
{
  return collision_object_name_;
}

void ProductObject::setCollisionName(std::string name)
{
  collision_object_name_ = name;
}

bool getDoubleParameter(ros::NodeHandle &nh, const std::string &param_name, double &value)
{
  // Load a param
  if (!nh.hasParam(param_name))
  {
    ROS_ERROR_STREAM_NAMED("shelf","Missing parameter '" << param_name << "'. Searching in namespace: " << nh.getNamespace());
    return false;
  }
  nh.getParam(param_name, value);
  ROS_DEBUG_STREAM_NAMED("shelf","Loaded parameter '" << param_name << "' with value " << value);

  return true;
}

bool getStringParameter(ros::NodeHandle &nh, const std::string &param_name, std::string &value)
{
  // Load a param
  if (!nh.hasParam(param_name))
  {
    ROS_ERROR_STREAM_NAMED("shelf","Missing parameter '" << param_name << "'. Searching in namespace: " << nh.getNamespace());
    return false;
  }
  nh.getParam(param_name, value);
  ROS_DEBUG_STREAM_NAMED("shelf","Loaded parameter '" << param_name << "' with value " << value);

  return true;
}

} // namespace
