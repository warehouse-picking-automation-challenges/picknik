cmake_minimum_required(VERSION 2.8.3)
project(picknik_perception)

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
)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED)
find_package(PCL 1.8 REQUIRED)

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

#add_library(point_cloud_filter
#  src/point_cloud_filter.cpp
#)
#target_link_libraries(point_cloud_filter
#  ${catkin_LIBRARIES} 
#  ${Boost_LIBRARIES}
#)

#add_executable(shape_filter
#  src/shape_filter.cpp
#)
#target_link_libraries(shape_filter
#  point_cloud_filter
#  ${catkin_LIBRARIES} 
#  ${Boost_LIBRARIES}

add_library(simple_point_cloud_filter
  src/simple_point_cloud_filter.cpp
)
target_link_libraries(simple_point_cloud_filter
  ${catkin_LIBRARIES}
)

add_executable(simple_point_cloud_filter_test
  src/tests/simple_point_cloud_filter_test.cpp
)
target_link_libraries(simple_point_cloud_filter_test
  simple_point_cloud_filter
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
)
