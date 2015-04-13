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
   Desc:   
*/

/** TEMPLATE NOTES
    planning_scene_switcher - verticle_test
    PlanningSceneSwitcher - VerticleApproachTest
    picknik_main - baxter_experimental
    THEN make ifndef all caps with Alt-U  (and Alt-F to skip the #define)
 */

#ifndef picknik_main__planning_scene_switcher
#define picknik_main__planning_scene_switcher

// ROS
#include <ros/ros.h>

namespace picknik_main
{

class PlanningSceneSwitcher
{
public:

  /**
   * \brief Constructor
   * \param verbose - run in debug mode
   */
  PlanningSceneSwitcher(bool verbose)
    : verbose_(verbose)
  {

    ROS_INFO_STREAM_NAMED("planning_scene_switcher","PlanningSceneSwitcher Ready.");
  }

private:

  // A shared node handle
  ros::NodeHandle nh_;

  // Show more visual and console output, with general slower run time.
  bool verbose_;

}; // end class

// Create boost pointers for this class
typedef boost::shared_ptr<PlanningSceneSwitcher> PlanningSceneSwitcherPtr;
typedef boost::shared_ptr<const PlanningSceneSwitcher> PlanningSceneSwitcherConstPtr;

} // end namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "planning_scene_switcher");
  ROS_INFO_STREAM_NAMED("main", "Starting PlanningSceneSwitcher...");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Check for verbose flag
  bool verbose = false;
  if (argc > 1)
  {
    for (std::size_t i = 0; i < argc; ++i)
    {
      if (strcmp(argv[i], "--verbose") == 0)
      {
        ROS_INFO_STREAM_NAMED("main","Running in VERBOSE mode (slower)");
        verbose = true;
        continue;
      }
    }
  }

  picknik_main::PlanningSceneSwitcher server(verbose);

  ROS_INFO_STREAM_NAMED("main", "Shutting down.");
  ros::shutdown();

  return 0;
}

#endif
