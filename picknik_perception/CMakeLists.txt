cmake_minimum_required(VERSION 2.8.3)
project(picknik_perception)

find_package(catkin REQUIRED COMPONENTS
  moveit_visual_tools
  cmake_modules
  eigen_conversions
  picknik_msgs
  sensor_msgs
  std_msgs
  pcl_ros
  tf
  tf_conversions
  bounding_box
  ros_param_shortcuts
)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED)
find_package(PCL 1.7 REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    moveit_visual_tools
    cmake_modules
    picknik_msgs
    bounding_box
  DEPENDS
    Eigen
  INCLUDE_DIRS
    include
  LIBRARIES
    manipulation_interface
    simple_point_cloud_filter
    #manual_tf_aligment
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
  ${Boost_LIBRARIES}  
)

# Library
add_library(manual_tf_alignment
  src/tools/manual_tf_alignment.cpp
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
add_executable(tf_keyboard
  src/tools/tf_keyboard.cpp
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
add_executable(merge_point_clouds
  src/merge_point_clouds.cpp
)
target_link_libraries(merge_point_clouds
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(fake_perception_server src/fake_perception_server.cpp)
target_link_libraries(fake_perception_server
  manipulation_interface
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

# Executable
add_executable(pcl_perception_server src/pcl_perception_server.cpp)
target_link_libraries(pcl_perception_server
  simple_point_cloud_filter
  manipulation_interface
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

# Executable
add_executable(picknik_perception_tests src/tests/picknik_perception_tests.cpp)
target_link_libraries(picknik_perception_tests
  simple_point_cloud_filter
  ${catkin_LIBRARIES}
)

## Install

# Install libraries
install(TARGETS manipulation_interface
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

