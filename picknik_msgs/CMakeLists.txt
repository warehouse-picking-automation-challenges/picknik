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

add_message_files(DIRECTORY msg FILES 
  FoundObject.msg
)

add_service_files(DIRECTORY srv FILES StopPerception.srv)

## Generate actions in the 'action' folder
set(ACT_FILES
  FindObjects.action
  CalibrateShelf.action
)
add_action_files(DIRECTORY action FILES ${ACT_FILES})

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES ${MSG_DEPS})

catkin_package(DEPENDS ${MSG_DEPS})

