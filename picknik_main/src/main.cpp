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

#include <picknik_main/apc_manager.h>

// ROS
#include <ros/ros.h>

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

  // Random
  srand (time(NULL));

  // Command line arguments
  std::size_t mode = 1;
  std::size_t order_start = 0;
  std::size_t jump_to = 0;
  std::size_t num_orders = 0;
  std::size_t bin_id = 0;
  bool verbose = false;
  bool use_experience = true;
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

    if( std::string(argv[i]).compare("--use_experience") == 0 )
    {
      ++i;
      use_experience = atoi(argv[i]);
      ROS_DEBUG_STREAM_NAMED("main","Using experience: " << use_experience);
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
    if( std::string(argv[i]).compare("--bin_id") == 0 )
    {
      ++i;
      bin_id = atoi(argv[i]);
      ROS_DEBUG_STREAM_NAMED("main","Focusing on bin index: " << bin_id);
      continue;
    }
  }

  if (order_file.empty())
  {
    ROS_ERROR_STREAM_NAMED("main","No order json file passed in as argument, aborting.");
    return 1; // error
  }

  picknik_main::APCManager manager(verbose, order_file, use_experience, autonomous, full_autonomous, fake_execution, fake_perception);

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
      //if (!manager.checkSystemReady(remove_from_shelf)) return 0;;
      ROS_INFO_STREAM_NAMED("main","Recording a trajectory for calibration");
      manager.recordCalibrationTrajectory();
      break;
    case 10:
      if (!manager.checkSystemReady(remove_from_shelf)) return 0;;
      ROS_INFO_STREAM_NAMED("main","Playback trajectory for calibration");
      manager.calibrateCamera();
      break;
    case 11:
      //if (!manager.checkSystemReady(remove_from_shelf)) return 0;;
      ROS_INFO_STREAM_NAMED("main","Recording bin observing trajectory");
      manager.recordBinWithCamera(bin_id);
      break;
    case 12:
      if (!manager.checkSystemReady(remove_from_shelf)) return 0;;
      ROS_INFO_STREAM_NAMED("main","Playing back bin observing trajectory");
      manager.perceiveBinWithCamera(bin_id);
      break;

    case 13:
      ROS_INFO_STREAM_NAMED("main","Only visualizing shelf... ready to shutdown.");
      manager.testVisualizeShelf();
      ros::spin();
      break;
    case 14:
      ROS_INFO_STREAM_NAMED("main","Get SRDF pose");
      manager.getSRDFPose();
      break;
    case 15:
      ROS_INFO_STREAM_NAMED("main","Check if current state is in collision");
      manager.testInCollision();
      ros::Duration(5.0).sleep();
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
      manager.testPerceptionComm();
      break;
    case 19:
      ROS_INFO_STREAM_NAMED("main","Train experience database mode");
      manager.trainExperienceDatabase();
      break;
    case 20:
      remove_from_shelf = false;
      if (!manager.checkSystemReady(remove_from_shelf)) return 0;;
      ROS_INFO_STREAM_NAMED("main","Going in and out of bin");
      manager.testInAndOut();
      break;
    case 21:
      ROS_INFO_STREAM_NAMED("main","Show experience database");
      manager.displayExperienceDatabase();
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
    case 24:
      remove_from_shelf = false;
      if (!manager.checkSystemReady(remove_from_shelf)) return 0;;
      ROS_INFO_STREAM_NAMED("main","Going to pose " << pose);
      manager.gotoPose(pose);
      break;      
    default:
      ROS_WARN_STREAM_NAMED("main","Unkown mode: " << mode);
  }

  // Shutdown
  std::cout << std::endl << std::endl << std::endl;
  std::cout << "-------------------------------------------------------" << std::endl;
  ROS_INFO_STREAM_NAMED("main", "Shutting down.");
  std::cout << std::endl << std::endl << std::endl;
  ros::shutdown();

  return 0;
}
