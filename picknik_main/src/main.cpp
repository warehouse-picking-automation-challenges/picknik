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
  bool verbose = false;
  bool use_experience = true;
  bool show_database = false;
  std::string order_fp;

  // Parse command line arguments
  for (std::size_t i = 0; i < argc; ++i)
  {
    if (strcmp(argv[i], "--verbose") == 0)
    {
      ROS_INFO_STREAM_NAMED("main","Running in VERBOSE mode (slower)");
      verbose = true;
    }

    if (strcmp(argv[i], "--order") == 0)
    {
      ++i;
      if (i >= argc) {
        ROS_ERROR_STREAM_NAMED("main", "Remember to tell us where's the json order, aborting");
        return 1;
      }
      order_fp = argv[i];
      ROS_INFO_STREAM_NAMED("main","Using order file " << order_fp);
    }

    if( std::string(argv[i]).compare("--use_experience") == 0 )
    {
      ++i;
      use_experience = atoi(argv[i]);
      ROS_INFO_STREAM_NAMED("main","Using experience: " << use_experience);
    }

    if( std::string(argv[i]).compare("--show_database") == 0 )
    {
      ++i;
      show_database = atoi(argv[i]);
      ROS_INFO_STREAM_NAMED("main","Showing database: " << show_database);
    }
    if( std::string(argv[i]).compare("--mode") == 0 )
    {
      ++i;
      mode = atoi(argv[i]);
      //ROS_INFO_STREAM_NAMED("main","In mode " << mode);
    }
    if( std::string(argv[i]).compare("--order_start") == 0 )
    {
      ++i;
      order_start = atoi(argv[i]);
      ROS_INFO_STREAM_NAMED("main","Starting on order item " << order_start);
    }
    if( std::string(argv[i]).compare("--jump_to") == 0 )
    {
      ++i;
      jump_to = atoi(argv[i]);
      ROS_INFO_STREAM_NAMED("main","Jumping to maniplation step " << jump_to);
    }
  }

  if (order_fp.empty()) 
  {
    ROS_ERROR_STREAM_NAMED("main","No order json file passed in as argument, aborting.");
    return 1; // error
  }

  picknik_main::APCManager manager(verbose, order_fp);

  switch (mode)
  {
    case 1:
      ROS_INFO_STREAM_NAMED("main","Run actual Amazon Picking Challenge mode");
      manager.runOrder(use_experience, show_database, order_start, jump_to);
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
    // case 4:
    //   ROS_INFO_STREAM_NAMED("main"," mode");
    //   break;
    default:
      ROS_WARN_STREAM_NAMED("main","Unkown mode: " << mode);
  }

  // Shutdown
  ROS_INFO_STREAM_NAMED("main", "Shutting down.");
  ros::Duration(5.0).sleep();
  ros::shutdown();

  return 0;
}
