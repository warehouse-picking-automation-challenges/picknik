cmake_minimum_required(VERSION 2.8.3)
cmake_policy(SET CMP0054 OLD)

project(moveit_arpg_perception)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  picknik_msgs
  roscpp
  actionlib
  #moveit_visual_tools
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

#ARPG dependencies

find_package(Node  REQUIRED)
find_package(Protobuf REQUIRED)
#find_package(HAL)
#find_package(PbMsgs 0.1 REQUIRED)
#find_package(MINIGLOG QUIET)


#Generate Protobuf messages
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS
				 msgs/ExampleMessage.proto
				 msgs/findObject.proto
				 msgs/ObjectPose.proto)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   picknik_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES moveit_arpg_perception
  CATKIN_DEPENDS 
    picknik_msgs 
    roscpp
    actionlib
    #moveit_visual_tools
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Node_INCLUDE_DIRS}
  ${HAL_INCLUDE_DIRS}
  ${PbMsgs_INCLUDE_DIRS}
  ${MINIGLOG_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
)

## Declare a cpp library
# add_library(moveit_arpg_perception
#   src/${PROJECT_NAME}/moveit_arpg_perception.cpp
# )

## Declare a cpp executable
add_executable(object_recognition_server src/object_recognition_server.cpp ${PROTO_SRCS} ${PROTO_HDRS} )

add_executable(fakeDDTR tests/fakeDDTR.cpp ${PROTO_SRCS} ${PROTO_HDRS})
add_executable(reqDDTR tests/reqDDTR.cpp ${PROTO_SRCS} ${PROTO_HDRS})


## Specify libraries to link a library or executable target against
target_link_libraries(object_recognition_server
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
   ${HAL_LIBRARIES}
   ${Node_LIBRARIES}
   ${PbMsgs_LIBRARIES}
   ${MINIGLOG_LIBRARIES}
   ${PROTOBUF_LIBRARIES}
   dl
)

target_link_libraries(fakeDDTR
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
   ${HAL_LIBRARIES}
   ${Node_LIBRARIES}
   ${PbMsgs_LIBRARIES}
   ${MINIGLOG_LIBRARIES}	
   ${PROTOBUF_LIBRARIES}
   dl
)

target_link_libraries(reqDDTR
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
   ${HAL_LIBRARIES}
   ${Node_LIBRARIES}
   ${PbMsgs_LIBRARIES}
   ${MINIGLOG_LIBRARIES}	
   ${PROTOBUF_LIBRARIES}
   dl
)
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS moveit_arpg_perception object_recognition_server
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_moveit_arpg_perception.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
