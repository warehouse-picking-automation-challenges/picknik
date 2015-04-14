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

#ifndef PICKNIK_MAIN__PRODUCT_SIMULATOR
#define PICKNIK_MAIN__PRODUCT_SIMULATOR

// ROS
#include <ros/ros.h>

// Picknik
#include <picknik_main/shelf.h>
#include <picknik_main/namespaces.h>
#include <picknik_main/visuals.h>

namespace picknik_main
{

static const double LOWER_SEARCH_DISCRETIZATION = 0.001;
const static double RAND_PADDING = 0.01;
const static std::size_t MAX_ATTEMPTS = 1000;

class ProductSimulator
{
public:

  /**
   * \brief Constructor
   * \param verbose - run in debug mode
   */
  ProductSimulator(bool verbose, VisualsPtr visuals,
                   planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor);

  /**
   * \brief Place objects on shelf randomly in simulation
   * \return true on success
   */
  bool generateRandomProductPoses(ShelfObjectPtr shelf);

  /**
   * \brief Add a collision mesh of a product to any scene
   * \param product - object that contains everything needed to know about it
   * \param trans - translation from world from to the frame of refrence of the product's centroid
   * \return true on success
   */
  bool addCollisionMesh(ProductObjectPtr& product, const Eigen::Affine3d& trans);

  /**
   * \brief Checks if new product is in collision with current planning scene world
   * \param product - object that contains everything needed to know about it
   * \param trans - translation from world from to the frame of refrence of the product's centroid
   * \return true if in collision
   */
  bool inCollision(ProductObjectPtr& product, const Eigen::Affine3d& trans);

private:

  // Show more visual and console output, with general slower run time.
  bool verbose_;

  // Visualization classes
  VisualsPtr visuals_;

  // Primary planning scene - monitor
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  // Reusable secondary planning scene for testing new object locations
  planning_scene::PlanningScene secondary_scene_;

}; // end class

// Create boost pointers for this class
typedef boost::shared_ptr<ProductSimulator> ProductSimulatorPtr;
typedef boost::shared_ptr<const ProductSimulator> ProductSimulatorConstPtr;

} // end namespace

#endif
