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
   Desc:   Main function that processes arguments
*/

#include <string>
#include <iostream>
#include <stdio.h>
#include <picknik_main/apc_manager.h>

// ROS
#include <ros/ros.h>

// Adapted from http://stackoverflow.com/a/478960/1191119
std::string exec(const char* cmd)
{
  FILE* pipe = popen(cmd, "r");
  if (!pipe) return "ERROR";
  char buffer[128];
  std::string result = "";
  while (!feof(pipe))
  {
  if (fgets(buffer, 128, pipe) != NULL)
    result += buffer;
  }
  pclose(pipe);
  return result;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "apc_manager");
  std::cout << std::endl;
  std::cout << std::endl;
  ROS_INFO_STREAM_NAMED("main", "Starting Amazon Picking Challenge Manager");
  std::cout << std::endl;

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(4);
  spinner.start();

  // start timer for run length
  ros::Time begin_time = ros::Time::now();

  // Random
  srand (time(NULL));

  // Command line arguments
  std::size_t mode = 1;
  std::size_t order_start = 0;
  std::size_t jump_to = 1;
  std::size_t num_orders = 0;
  std::size_t id = 0;
  bool verbose = false;
  bool fake_execution = true;
  bool fake_perception = true;
  bool autonomous = false;
  bool full_autonomous = false;
  std::string order_file;
  std::string pose;

  // Parse command line arguments
  for (int i = 0; i < argc; ++i)
  {
    if (strcmp(argv[i], "--verbose") == 0)
    {
      ++i;
      verbose = atoi(argv[i]);
      ROS_DEBUG_STREAM_NAMED("main","Running in verbose mode: " << verbose);
      continue;
    }

    if (strcmp(argv[i], "--order") == 0)
    {
      ++i;
      if (i >= argc) {
        ROS_ERROR_STREAM_NAMED("main", "Remember to tell us where's the json order, aborting");
        return 1;
      }
      order_file = argv[i];
      ROS_DEBUG_STREAM_NAMED("main","Using order file " << order_file);
      continue;
    }

    if (strcmp(argv[i], "--pose") == 0)
    {
      ++i;
      pose = argv[i];
      ROS_DEBUG_STREAM_NAMED("main","Using pose " << pose);
      continue;
    }

    if( std::string(argv[i]).compare("--auto") == 0 )
    {
      ++i;
      autonomous = atoi(argv[i]);
      ROS_DEBUG_STREAM_NAMED("main","Using autonomous: " << autonomous);
      continue;
    }

    if( std::string(argv[i]).compare("--full_auto") == 0 )
    {
      ++i;
      full_autonomous = atoi(argv[i]);
      ROS_DEBUG_STREAM_NAMED("main","Using full_autonomous: " << full_autonomous);
      continue;
    }

    if( std::string(argv[i]).compare("--fake_execution") == 0 )
    {
      ++i;
      fake_execution = atoi(argv[i]);
      ROS_DEBUG_STREAM_NAMED("main","Fake execution: " << fake_execution);
      continue;
    }

    if( std::string(argv[i]).compare("--fake_perception") == 0 )
    {
      ++i;
      fake_perception = atoi(argv[i]);
      ROS_DEBUG_STREAM_NAMED("main","Fake perception: " << fake_perception);
      continue;
    }

    if( std::string(argv[i]).compare("--mode") == 0 )
    {
      ++i;
      mode = atoi(argv[i]);
      //ROS_DEBUG_STREAM_NAMED("main","In mode " << mode);
      continue;
    }
    if( std::string(argv[i]).compare("--order_start") == 0 )
    {
      ++i;
      order_start = atoi(argv[i]);
      ROS_DEBUG_STREAM_NAMED("main","Starting on order item " << order_start);
      continue;
    }
    if( std::string(argv[i]).compare("--jump_to") == 0 )
    {
      ++i;
      jump_to = atoi(argv[i]);
      ROS_DEBUG_STREAM_NAMED("main","Jumping to maniplation step " << jump_to);
      continue;
    }
    if( std::string(argv[i]).compare("--num_orders") == 0 )
    {
      ++i;
      num_orders = atoi(argv[i]);
      ROS_DEBUG_STREAM_NAMED("main","Number of products to process for the order: " << num_orders);
      continue;
    }
    if( std::string(argv[i]).compare("--id") == 0 )
    {
      ++i;
      id = atoi(argv[i]);
      ROS_DEBUG_STREAM_NAMED("main","Using mode with index: " << id);
      continue;
    }
  }

  if (order_file.empty())
  {
    ROS_ERROR_STREAM_NAMED("main","No order json file passed in as argument, aborting.");
    return 1; // error
  }


  // Sort order according to expected scores
  std::stringstream order_sorting_command;
  std::string processed_order_file = order_file + ".processed";
  order_sorting_command << "rosrun picknik_main sort_order.py " << order_file << " " << processed_order_file;
  std::cout << "Order sorting command: " << order_sorting_command.str() << std::endl;
  std::string result;
  result = exec(order_sorting_command.str().c_str());
  ROS_DEBUG_STREAM_NAMED("main", "order sorted by running: \"" << order_sorting_command.str() << "\" returned: " << result);
  if (result.compare("ERROR") == 0) {
    ROS_ERROR_STREAM_NAMED("main", "Running the sorting command failed, aborting.");
    return 1;
  }
  if (!result.empty()) { ROS_DEBUG_STREAM_NAMED("main", "Sorting might have failed, let's try to keep going"); }

  picknik_main::APCManager manager(verbose, processed_order_file, autonomous, full_autonomous, fake_execution, fake_perception);

  std::cout << std::endl;
  std::cout << "-------------------------------------------------------" << std::endl;

  bool remove_from_shelf = true; // option if checkSystemReady() should auto remove arm from shelf

  switch (mode)
  {
    case 1:
      ROS_INFO_STREAM_NAMED("main","Run actual Amazon Picking Challenge mode");
      manager.mainOrderProcessor(order_start, jump_to, num_orders);
      break;
    case 2:
      if (!manager.checkSystemReady(remove_from_shelf)) return 0;;
      ROS_INFO_STREAM_NAMED("main","Go to home position");
      manager.testGoHome();
      break;
    case 3:
      if (!manager.checkSystemReady(remove_from_shelf)) return 0;;
      ROS_INFO_STREAM_NAMED("main","Going to goal_bin place pose");
      manager.testGoalBinPose();
      break;
    case 4:
      if (!manager.checkSystemReady(remove_from_shelf)) return 0;;
      ROS_INFO_STREAM_NAMED("main","Moving camera to each bin location");
      manager.testCameraPositions();
      break;
    case 5:
      //if (!manager.checkSystemReady(remove_from_shelf)) return 0;;
      ROS_INFO_STREAM_NAMED("main","Raise the roof (go up and down)");
      manager.testUpAndDown();
      break;
    case 6:
      if (!manager.checkSystemReady(remove_from_shelf)) return 0;;
      ROS_INFO_STREAM_NAMED("main","Plan to random valid locations");
      manager.testRandomValidMotions();
      break;
    case 7:
      if (!manager.checkSystemReady(remove_from_shelf)) return 0;;
      ROS_INFO_STREAM_NAMED("main","Verify shelf location");
      manager.testShelfLocation();
      break;
    case 8:
      if (!manager.checkSystemReady(remove_from_shelf)) return 0;;
      ROS_INFO_STREAM_NAMED("main","Test end effectors mode");
      manager.testEndEffectors();
      break;
    case 9:
      remove_from_shelf = true;
      if (!manager.checkSystemReady(remove_from_shelf)) return 0;;
      ROS_INFO_STREAM_NAMED("main","Going to pose " << pose);
      manager.gotoPose(pose);
      break;
    case 10:
      remove_from_shelf = false;
      if (!manager.checkSystemReady(remove_from_shelf)) return 0;;
      ROS_INFO_STREAM_NAMED("main","Going in and out of bin");
      manager.testInAndOut();
      break;
    case 11:
      remove_from_shelf = false;
      if (!manager.checkSystemReady(remove_from_shelf)) return 0;;
      ROS_INFO_STREAM_NAMED("main","Going in circle for calibration");
      manager.calibrateInCircle();
      break;
    case 12:
      remove_from_shelf = false;
      if (!manager.checkSystemReady(remove_from_shelf)) return 0;;
      ROS_INFO_STREAM_NAMED("main","Going in square for calibration");
      manager.calibrateInSquare();
      break;




    case 16:
      ROS_INFO_STREAM_NAMED("main","Testing grasp generator abilities and scoring results");
      manager.testGraspGenerator();
      break;
    case 17:
      if (!manager.checkSystemReady(remove_from_shelf)) return 0;;
      ROS_INFO_STREAM_NAMED("main","Test joint limits");
      manager.testJointLimits();
      break;
    case 18:
      if (!manager.checkSystemReady(remove_from_shelf)) return 0;;
      ROS_INFO_STREAM_NAMED("main","Requesting perception test");
      manager.testPerceptionComm(id);
      break;
    case 19:
      if (!manager.checkSystemReady(remove_from_shelf)) return 0;;
      ROS_INFO_STREAM_NAMED("main","Requesting perception test for each bin");
      manager.testPerceptionCommEach();
      break;
    case 20:
      if (!manager.checkSystemReady(remove_from_shelf)) return 0;;
      ROS_INFO_STREAM_NAMED("main","Testing variable grasp widths");
      manager.testGraspWidths();
      break;
    case 22:
      remove_from_shelf = false;
      if (!manager.checkSystemReady(remove_from_shelf)) return 0;;
      ROS_INFO_STREAM_NAMED("main","Testing approach lift retreat cartesian path");
      manager.testApproachLiftRetreat();
      break;
    case 23:
      if (!manager.checkSystemReady(remove_from_shelf)) return 0;;
      ROS_INFO_STREAM_NAMED("main","Unit tests for manipulation");
      manager.unitTests();
      break;
    case 25:
      ROS_INFO_STREAM_NAMED("main","Testing IK solver");
      manager.testIKSolver();
      break;
    case 26:
      ROS_INFO_STREAM_NAMED("main","Unit test for perception communication");
      manager.unitTestPerceptionComm();
      break;
    case 27:
      ROS_INFO_STREAM_NAMED("main","Testing planning ONLY from a shelf bin to the goal bin");
      manager.testPlanningSimple();
      break;

    case 30:
      //if (!manager.checkSystemReady(remove_from_shelf)) return 0;;
      ROS_INFO_STREAM_NAMED("main","Recording a trajectory for calibration");
      manager.recordCalibrationTrajectory(id);
      break;
    case 31:
      if (!manager.checkSystemReady(remove_from_shelf)) return 0;;
      ROS_INFO_STREAM_NAMED("main","Playback trajectory for calibration");
      manager.calibrateCamera(id);
      break;
    case 32:
      //if (!manager.checkSystemReady(remove_from_shelf)) return 0;;
      ROS_INFO_STREAM_NAMED("main","Recording bin observing trajectory");
      manager.recordBinWithCamera(id);
      break;
    case 33:
      if (!manager.checkSystemReady(remove_from_shelf)) return 0;;
      ROS_INFO_STREAM_NAMED("main","Playing back bin observing trajectory");
      manager.perceiveBinWithCamera(id);
      break;
    case 34:
      if (!manager.checkSystemReady(remove_from_shelf)) return 0;;
      ROS_INFO_STREAM_NAMED("main","Playback waypoint path specified in a csv");
      manager.playbackWaypointsFromFile();
      break;

    case 40:
      ROS_INFO_STREAM_NAMED("main","Only visualizing shelf... ready to shutdown.");
      manager.testVisualizeShelf();
      ros::spin();
      break;
    case 41:
      ROS_INFO_STREAM_NAMED("main","Get SRDF pose");
      manager.getSRDFPose();
      break;
    case 42:
      ROS_INFO_STREAM_NAMED("main","Check if current state is in collision");
      manager.testInCollision();
      ros::Duration(5.0).sleep();
      break;
    case 43:
      ROS_INFO_STREAM_NAMED("main","Calibrate shelf");
      manager.calibrateShelf();
      break;
    case 44:
      ROS_INFO_STREAM_NAMED("main","Testing ideal collision object attachment");
      manager.testIdealAttachedCollisionObject();
      ros::spin();
      break;

    case 50:
      ROS_INFO_STREAM_NAMED("main","Train experience database mode");
      manager.trainExperienceDatabase();
      break;
    case 51:
      ROS_INFO_STREAM_NAMED("main","Show experience database");
      manager.displayExperienceDatabase();
      break;

    default:
      ROS_WARN_STREAM_NAMED("main","Unkown mode: " << mode);
  }

  // Shutdown
  std::cout << std::endl << std::endl << std::endl;
  std::cout << "-------------------------------------------------------" << std::endl;
  ROS_INFO_STREAM_NAMED("main", "Shutting down.");
  std::cout << std::endl << std::endl << std::endl;

  ros::Time end_time = ros::Time::now();

  ros::Duration duration = (end_time - begin_time);
  
  ROS_INFO_STREAM_NAMED("main","Test duration = " << duration << " seconds (" << (duration.toSec() / 60.0) << " minutes). "
                        << "Max time allowed = " << (15.0 * 60.0) << " seconds (15 minutes).");
  
  ros::shutdown();

  return 0;
}
