cmake_minimum_required(VERSION 2.8.3)
project(picknik_msgs)

set(MSG_DEPS
  actionlib_msgs
  geometry_msgs
  sensor_msgs
  shape_msgs
  std_msgs
)

find_package(catkin REQUIRED COMPONENTS
  genmsg ${MSG_DEPS}
)

set(ACT_FILES
  FindObjects.action
  CalibrateShelf.action
)

## Generate actions in the 'action' folder
add_action_files(DIRECTORY action FILES ${ACT_FILES})

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES ${MSG_DEPS})

catkin_package(DEPENDS ${MSG_DEPS})

