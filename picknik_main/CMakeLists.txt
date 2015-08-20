cmake_minimum_required(VERSION 2.8.3)
project(picknik_main)

# C++ 11
set(CMAKE_CXX_FLAGS "-std=c++0x -Wall ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_grasps
  moveit_ompl
  moveit_visual_tools
  ompl_visual_tools
  cmake_modules
  picknik_msgs
  controller_manager_msgs
  bounding_box  
  ros_param_shortcuts
)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED)
find_package(OMPL REQUIRED)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Gflags REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    moveit_core
    moveit_grasps
    moveit_visual_tools
    moveit_ompl
    picknik_msgs
    controller_manager_msgs
    bounding_box
    ros_param_shortcuts
  DEPENDS
    Eigen
  INCLUDE_DIRS 
    include
    ${OMPL_INCLUDE_DIRS}
  LIBRARIES
    jsoncpp
    fix_state_bounds
    manipulation    
    visuals
    ${OMPL_LIBRARIES}
)

###########
## Build ##
###########
include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  ${Gflags_INCLUDE_DIRS}
)

include_directories(SYSTEM 
  ${Boost_INCLUDE_DIR} 
  ${EIGEN_INCLUDE_DIRS}
)
link_directories(${OMPL_LIBRARY_DIRS})

# Visuals Library
add_library(visuals
  src/visuals.cpp
)
target_link_libraries(visuals
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES}
)

# Manipulation Data Library
add_library(manipulation_data
  src/manipulation_data.cpp
)
target_link_libraries(manipulation_data
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES}
)

# JSON Parser Library
add_library(jsoncpp
  src/json/jsoncpp.cpp
)
target_link_libraries(jsoncpp
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES}
)

# Amazon Parser
# add_library(amazon_json_parser
#   src/amazon_json_parser.cpp
# )
# target_link_libraries(amazon_json_parser
#   jsoncpp
#   visuals
#   ${catkin_LIBRARIES} 
#   ${Boost_LIBRARIES}
# )

# Planning Scene Library
add_library(planning_scene_manager
  src/planning_scene_manager.cpp
)
target_link_libraries(planning_scene_manager
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES}
)

# Remote_control library
add_library(remote_control
  src/remote_control.cpp
)
target_link_libraries(remote_control
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES}
)

# Collision_object library
add_library(collision_object
  src/collision_object.cpp
)
target_link_libraries(collision_object
  visuals  
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES}
)

# Shelf library
# add_library(shelf
#   src/shelf.cpp
# )
# target_link_libraries(shelf
#   visuals
#   collision_object
#   manipulation_data
#   ${catkin_LIBRARIES} 
#   ${Boost_LIBRARIES}
# )

# Fix_state_bounds library
add_library(fix_state_bounds
  src/fix_state_bounds.cpp
)
target_link_libraries(fix_state_bounds
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES}
)

# Product_simulator library
# add_library(product_simulator
#   src/product_simulator.cpp
# )
# target_link_libraries(product_simulator
#   visuals
#   shelf
#   ${catkin_LIBRARIES} 
#   ${Boost_LIBRARIES}
#)

# perception_interface library
add_library(perception_interface
  src/perception_interface.cpp
)
target_link_libraries(perception_interface
  visuals
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES}
)

# execution interface library
add_library(execution_interface
  src/execution_interface.cpp
)
target_link_libraries(execution_interface
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES}
)

# show a sensor line
add_library(tactile_feedback
  src/tactile_feedback.cpp
)
target_link_libraries(tactile_feedback
  visuals
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES}
)

# Manipulation pipeline library
add_library(manipulation
  src/manipulation.cpp
)
add_dependencies(manipulation picknik_main_generate_messages_cpp)
target_link_libraries(manipulation
  visuals
  manipulation_data
  execution_interface
  fix_state_bounds
  remote_control  
  tactile_feedback
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES}
)

# trajectory input/output
add_library(trajectory_io
  src/trajectory_io.cpp
)
target_link_libraries(trajectory_io
  manipulation
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES}
)

# Experience trainer library
# add_library(learning_pipeline
#   src/learning_pipeline.cpp
# )
# target_link_libraries(learning_pipeline
#   ${catkin_LIBRARIES} 
#   ${Boost_LIBRARIES}
#   manipulation
# )

# Main logic of Picking
add_library(pick_manager
  src/pick_manager.cpp
)
target_link_libraries(pick_manager
  trajectory_io
  tactile_feedback
  manipulation
  perception_interface
  planning_scene_manager
  gflags
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES}
)

# Main Executable
add_executable(picknik_main src/picknik_main.cpp)
target_link_libraries(picknik_main
#  amazon_json_parser
  pick_manager
  visuals
  gflags
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES}
)

# TESTS
add_executable(mesh_publisher tests/mesh_publisher.cpp)
target_link_libraries(mesh_publisher 
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES}
)
