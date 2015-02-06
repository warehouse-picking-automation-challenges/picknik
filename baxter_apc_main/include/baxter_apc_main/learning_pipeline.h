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
   Desc:   Generates poses for the shelf to populate the database
*/

#ifndef BAXTER_APC_MAIN__LEARNING_PIPELINE
#define BAXTER_APC_MAIN__LEARNING_PIPELINE

// ROS
#include <ros/ros.h>

// MoveIt
#include <baxter_apc_main/manipulation_pipeline.h>

namespace baxter_apc_main
{

const double POSE_DISCRETIZATION = 0.02; // how spread apart to create training poses
const double POSE_HORIZONTAL_MARGIN = 0.08;
const double POSE_TOP_MARGIN = BIN_HEIGHT - 0.05;
const double POSE_BOTTOM_MARGIN = 0.05;

struct BinExperienceData
{
  EigenSTL::vector_Affine3d poses;
  std::vector<bool> pose_valid;
};

typedef std::map<const std::string, BinExperienceData> BinExperienceDataMap;

class LearningPipeline : private ManipulationPipeline
{
public:

  /**
   * \brief Constructor
   * \param verbose - run in debug mode
   */
  LearningPipeline(bool verbose, moveit_visual_tools::MoveItVisualToolsPtr visual_tools,
                   planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor,
                   ShelfObjectPtr shelf, bool use_experience, bool show_database);

  /**
   * \brief Destructor
   */
  ~LearningPipeline(){};

  /**
   * \brief Generate (and display) all poses to train on
   */
  bool generateTrainingGoals(ShelfObjectPtr shelf);
  bool generateTrainingGoalsBin(Eigen::Affine3d bin_transpose, EigenSTL::vector_Affine3d &poses);

  /**
   * \brief Simple test script for visualizing
   * \param input - description
   * \return true on success
   */
  bool visualizePose(Eigen::Affine3d grasp_pose, const moveit::core::JointModelGroup *arm_jmg);

  /**
   * \brief Determine which grasps have IK solutions
   * \return true on success
   */
  bool testGrasps();

  /**
   * \brief Show all grasps in Rviz
   * \return true on success
   */
  bool displayGrasps(bool valid_only);

private:

  BinExperienceDataMap bin_experience_data_;

}; // end class

// Create boost pointers for this class
typedef boost::shared_ptr<LearningPipeline> LearningPipelinePtr;
typedef boost::shared_ptr<const LearningPipeline> LearningPipelineConstPtr;

} // end namespace

#endif
