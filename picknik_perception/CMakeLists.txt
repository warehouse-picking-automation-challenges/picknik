cmake_minimum_required(VERSION 2.8.3)
project(picknik_perception)

set(CMAKE_CXX_FLAGS "-W -Wall")

find_package(catkin REQUIRED COMPONENTS
  moveit_visual_tools
  cmake_modules
  eigen_conversions
  picknik_msgs
  picknik_main
  moveit_ros_perception
  sensor_msgs
  std_msgs
  pcl_ros
  tf
  tf_conversions
)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED)
find_package(PCL 1.7 REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    moveit_visual_tools
    cmake_modules
    picknik_msgs
    picknik_main
    moveit_ros_perception
  DEPENDS
    Eigen
  INCLUDE_DIRS 
    include
  LIBRARIES
    point_cloud_filter
)

###########
## Build ##
###########
include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

include_directories(SYSTEM 
  ${Boost_INCLUDE_DIR} 
  ${EIGEN_INCLUDE_DIRS}
)

# Library
add_library(simple_point_cloud_filter
  src/simple_point_cloud_filter.cpp
)
target_link_libraries(simple_point_cloud_filter
  ${catkin_LIBRARIES}
)

# Library
add_library(manual_tf_alignment
  src/manual_tf_alignment.cpp
)
target_link_libraries(manual_tf_alignment
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

# Library
add_library(manipulation_interface
  src/manipulation_interface.cpp
)
target_link_libraries(manipulation_interface
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

# Executable
add_executable(simple_point_cloud_filter_test
  src/tests/simple_point_cloud_filter_test.cpp
)
target_link_libraries(simple_point_cloud_filter_test
  simple_point_cloud_filter
  manual_tf_alignment
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
)

# Executable
add_executable(tf_keyboard
  src/tf_keyboard.cpp
)
target_link_libraries(tf_keyboard
  manual_tf_alignment
  ${catkin_LIBRARIES}
)

# Executable
add_executable(cloud_preprocessor
  src/cloud_preprocessor.cpp
)
target_link_libraries(cloud_preprocessor
  simple_point_cloud_filter
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

# Executable
add_executable(align_point_clouds
  src/align_point_clouds.cpp
)
target_link_libraries(align_point_clouds
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(fake_perception_server src/fake_perception_server.cpp)
target_link_libraries(fake_perception_server 
  manipulation_interface
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES}
)
