/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Dave Coleman <dave@dav.ee>
 *  All rights reserved.
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 *********************************************************************/
/*
  Author: Dave Coleman
  Desc:   Parses a JSON file into inventory and orders for Amazon
*/

// ROS
#include <baxter_apc_main/amazon_json_parser.h>

#include <fstream>

namespace baxter_apc_main
{

AmazonJSONParser::AmazonJSONParser(bool verbose, mvt::MoveItVisualToolsPtr visual_tools,mvt::MoveItVisualToolsPtr visual_tools_display)
  : verbose_(verbose)
  , visual_tools_(visual_tools)
  , visual_tools_display_(visual_tools_display)
{
  ROS_INFO_STREAM_NAMED("parser","AmazonJSONParser Ready.");
}

AmazonJSONParser::~AmazonJSONParser()
{
}

bool AmazonJSONParser::parse(const std::string& file_path, const std::string& package_path, ShelfObjectPtr shelf, WorkOrders& orders)
{
  std::ifstream input_stream(file_path.c_str());

  if (!input_stream.good())
  {
    ROS_ERROR_STREAM_NAMED("parser","Unable to load file " << file_path);
    return false;
  }

  Json::Value root;   // will contains the root value after parsing
  Json::Reader reader;
  bool parsingSuccessful = reader.parse( input_stream, root );
  if ( !parsingSuccessful )
  {
    // report to the user the failure and their locations in the document.
    std::cout  << "Failed to parse configuration\n"
               << reader.getFormattedErrorMessages();
    return false;
  }

  // Parse bin contents
  const Json::Value bin_contents = root["bin_contents"];
  if (!bin_contents)
  {
    ROS_ERROR_STREAM_NAMED("parser","Unable to find json element 'bin_contents'");
    return false;
  }
  if (!parseBins(package_path, bin_contents, shelf))
  {
    return false;
  }

  // Parse work order
  const Json::Value work_orders = root["work_order"];
  if (!work_orders)
  {
    ROS_ERROR_STREAM_NAMED("parser","Unable to find json element 'work_order'");
    return false;
  }
  if (!work_orders.isArray())
  {
    ROS_ERROR_STREAM_NAMED("parser","work_order not an array");
    return false;
  }
  if (!parseWorkOrders(work_orders, shelf, orders))
  {
    return false;
  }

  return true;
}

bool AmazonJSONParser::parseBins(const std::string& package_path, const Json::Value bin_contents, ShelfObjectPtr shelf)
{
  const Json::Value::Members bin_names = bin_contents.getMemberNames();
  for (Json::Value::Members::const_iterator bin_it = bin_names.begin();
       bin_it != bin_names.end(); bin_it++)
  {
    const std::string& bin_name = *bin_it;
    if (verbose_)
      ROS_DEBUG_STREAM_NAMED("parser","Loaded: " << bin_name);

    const Json::Value& bin = bin_contents[bin_name];

    double bin_y_space = 0.08; // where to place objects

    // Get each product
    for ( int index = 0; index < bin.size(); ++index )
    {
      const std::string& product_name = bin[index].asString();
      if (verbose_)
        ROS_DEBUG_STREAM_NAMED("parser","   product: " << product_name);

      // Add object to a bin
      ProductObjectPtr product(new ProductObject(visual_tools_, visual_tools_display_, rvt::RAND, product_name, package_path));

      // Set location of product
      product->bottom_right_.translation() = Eigen::Vector3d(0.05, //visual_tools_->dRand(0.0, 0.05),  // depth
                                                            bin_y_space,  // from right
                                                            0.1);

      // Set size of product
      double width = visual_tools_->dRand(0.01, 0.06);
      product->top_left_.translation() = product->bottom_right_.translation() + 
        Eigen::Vector3d(visual_tools_->dRand(0.01, 0.2), // depth
                        width, //width
                        visual_tools_->dRand(0.05, 0.2)); // height

      bin_y_space += width + 0.07; // spacing between objects

      // Add product to correct location
      shelf->getBins()[bin_name]->getProducts().push_back(product);
    }
  }

  return true;
}

bool AmazonJSONParser::parseWorkOrders(const Json::Value work_orders, ShelfObjectPtr shelf, WorkOrders& orders)
{
  // Get each work order
  for ( int work_id = 0; work_id < 	work_orders.size(); ++work_id )
  {
    const Json::Value& work_order = work_orders[work_id]; // is an object

    if (!work_order.isObject())
    {
      ROS_ERROR_STREAM_NAMED("parser","work_order not an object");
      return false;
    }

    const std::string bin_name = work_order["bin"].asString();
    const std::string product_name = work_order["item"].asString();

    if (verbose_)
      ROS_DEBUG_STREAM_NAMED("parser","Creating work order with product " << product_name
                             << " in bin " << bin_name);

    // Find ptr to the bin
    BinObjectPtr bin = shelf->getBins()[bin_name];

    if (!bin)
    {
      ROS_ERROR_STREAM_NAMED("parser","Unable to find bin named " << bin_name);
      return false;
    }

    // Find ptr to the product
    ProductObjectPtr product = bin->getProduct(product_name);

    if (!product)
    {
      ROS_ERROR_STREAM_NAMED("parser","Unable to find product named " << product_name);
      return false;
    }

    // Set the order
    WorkOrder order(bin, product);

    // Add to the whole order
    orders.push_back(order);
  }

  return true;
}

} //namespace
