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
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Command line arguments
  std::size_t mode = 1;
  std::size_t order_start = 0;
  std::size_t jump_to = 0;
  std::size_t num_orders = 0;
  bool verbose = false;
  bool use_experience = true;
  bool show_database = false;
  bool autonomous = false;
  std::string order_fp;

  // Parse command line arguments
  for (std::size_t i = 0; i < argc; ++i)
  {
    if (strcmp(argv[i], "--verbose") == 0)
    {
      ROS_DEBUG_STREAM_NAMED("main","Running in VERBOSE mode (slower)");
      verbose = true;
      continue;
    }

    if (strcmp(argv[i], "--order") == 0)
    {
      ++i;
      if (i >= argc) {
        ROS_ERROR_STREAM_NAMED("main", "Remember to tell us where's the json order, aborting");
        return 1;
      }
      order_fp = argv[i];
      ROS_DEBUG_STREAM_NAMED("main","Using order file " << order_fp);
      continue;
    }

    if( std::string(argv[i]).compare("--auto") == 0 )
    {
      ++i;
      autonomous = atoi(argv[i]);
      ROS_DEBUG_STREAM_NAMED("main","Using autonomous: " << autonomous);
      continue;
    }

    if( std::string(argv[i]).compare("--use_experience") == 0 )
    {
      ++i;
      use_experience = atoi(argv[i]);
      ROS_DEBUG_STREAM_NAMED("main","Using experience: " << use_experience);
      continue;
    }

    if( std::string(argv[i]).compare("--show_database") == 0 )
    {
      ++i;
      show_database = atoi(argv[i]);
      ROS_DEBUG_STREAM_NAMED("main","Showing database: " << show_database);
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
  }

  if (order_fp.empty())
  {
    ROS_ERROR_STREAM_NAMED("main","No order json file passed in as argument, aborting.");
    return 1; // error
  }

  picknik_main::APCManager manager(verbose, order_fp, use_experience, show_database);

  switch (mode)
  {
    case 1:
      ROS_INFO_STREAM_NAMED("main","Run actual Amazon Picking Challenge mode");
      manager.runOrder(order_start, jump_to, num_orders, autonomous);
      break;
    case 2:
      ROS_INFO_STREAM_NAMED("main","Train experience database mode");
      manager.trainExperienceDatabase();
      break;
    case 3:
      ROS_INFO_STREAM_NAMED("main","Test end effectors mode");
      manager.testEndEffectors();
      break;
    case 4:
      ROS_INFO_STREAM_NAMED("main","Only visualizing shelf... ready to shutdown.");
      ros::spin();
      break;
    case 5:
       ROS_INFO_STREAM_NAMED("main","Raise the roof (go up and down)");
       manager.testUpAndDown();
       break;
    case 6:
      ROS_INFO_STREAM_NAMED("main","Verify shelf location");
      manager.testShelfLocation();
      break;
    case 7:
      ROS_INFO_STREAM_NAMED("main","Get SRDF pose");
      manager.getPose();
      break;
    case 8:
      ROS_INFO_STREAM_NAMED("main","Going to goal_bin place pose");
      manager.testGoalBinPose();
      break;
    case 9:
      ROS_INFO_STREAM_NAMED("main","Check if current state is in collision");
      manager.testInCollision();
      ros::Duration(5.0).sleep();
      break;
    case 10:
      ROS_INFO_STREAM_NAMED("main","Plan to random valid locations");
      manager.testRandomValidMotions();
      break;
    case 11:
      ROS_INFO_STREAM_NAMED("main","Moving to camera positions");
      manager.testCameraPositions();
      break;
    case 12:
      ROS_INFO_STREAM_NAMED("main","Test camera calibration");
      manager.testCalibration();
      break;
    case 13:
      ROS_INFO_STREAM_NAMED("main","Test joint limits");
      manager.testJointLimits();
      break;
    // case 12:
    //   ROS_INFO_STREAM_NAMED("main","");
    //   break;
    // case 12:
    //   ROS_INFO_STREAM_NAMED("main","");
    //   break;
    // case 12:
    //   ROS_INFO_STREAM_NAMED("main","");
    //   break;
    // case 12:
    //   ROS_INFO_STREAM_NAMED("main","");
    //   break;
    default:
      ROS_WARN_STREAM_NAMED("main","Unkown mode: " << mode);
  }

  // Shutdown
  ROS_INFO_STREAM_NAMED("main", "Shutting down.");
  ros::shutdown();

  return 0;
}
