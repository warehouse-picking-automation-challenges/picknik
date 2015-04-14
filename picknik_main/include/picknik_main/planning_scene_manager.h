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
   Desc:   Shows different versions of a shelf planning scene
*/

#ifndef PICKNIK_MAIN__PLANNING_SCENE_MANAGER
#define PICKNIK_MAIN__PLANNING_SCENE_MANAGER

// PickNik
#include <picknik_main/namespaces.h>

// MoveIt
#include <moveit/macros/class_forward.h>

// ROS
#include <ros/ros.h>

namespace picknik_main
{

MOVEIT_CLASS_FORWARD(ShelfObject);
MOVEIT_CLASS_FORWARD(Visuals);

enum SceneModes {
  NOT_LOADED,
  ALL_OPEN_BINS,
  FOCUSED_ON_BIN,
  ONLY_COLLISION_WALL
};


class PlanningSceneManager
{
public:

  /**
   * \brief Constructor
   * \param verbose - run in debug mode
   */
  PlanningSceneManager(bool verbose, VisualsPtr visuals, ShelfObjectPtr shelf);

  /**
   * \brief Show shelf with all bins enabled
   * \return true on success
   */
  bool displayShelfWithOpenBins();

  /**
   * \brief Show shelf as simple solid wall, not details
   * \return true on success
   */
  bool displayShelfAsWall();

  /**
   * \brief Only show one bin, disable the rest
   * \return true on success
   */
  bool displayShelfOnlyBin( const std::string& bin_name );

  /**
   * \brief Switch between the 3 modes
   */
  bool testAllModes();

private:

  // A shared node handle
  ros::NodeHandle nh_;

  // Show more visual and console output, with general slower run time.
  bool verbose_;

  // For visualizing things in rviz
  VisualsPtr visuals_;

  // Properties
  ShelfObjectPtr shelf_;

  // Mode switching to reduce redudant scene changes
  SceneModes mode_;
  std::string focused_bin_;

}; // end class

// Create boost pointers for this class
typedef boost::shared_ptr<PlanningSceneManager> PlanningSceneManagerPtr;
typedef boost::shared_ptr<const PlanningSceneManager> PlanningSceneManagerConstPtr;

} // end namespace

#endif
