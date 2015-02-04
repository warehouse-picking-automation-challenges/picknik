cmake_minimum_required(VERSION 2.8.3)
project(baxter_apc_main)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_grasps
  moveit_ompl
  moveit_visual_tools
  ompl_visual_tools
  cmake_modules
)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED)
find_package(OMPL REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    moveit_core
    moveit_grasps
    moveit_visual_tools
    moveit_ompl
  DEPENDS
    Eigen
  INCLUDE_DIRS 
    include
    ${OMPL_INCLUDE_DIRS}
  LIBRARIES
    jsoncpp
    amazon_json_parser
    shelf
    manipulation_pipeline
    learning_pipeline
    ${OMPL_LIBRARIES}
)

###########
## Build ##
###########
include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
)

include_directories(SYSTEM 
  ${Boost_INCLUDE_DIR} 
  ${EIGEN_INCLUDE_DIRS}
)
link_directories(${OMPL_LIBRARY_DIRS})

# JSON Parser Library
add_library(jsoncpp
  src/json/jsoncpp.cpp
)
target_link_libraries(jsoncpp
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES}
)

# Amazon Parser
add_library(amazon_json_parser
  src/amazon_json_parser.cpp
)
target_link_libraries(amazon_json_parser
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES}
  jsoncpp
)

# Visualization Tools Library
add_library(shelf
  src/shelf.cpp
)
target_link_libraries(shelf
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES}
)

# Manipulation pipeline library
add_library(manipulation_pipeline
  src/manipulation_pipeline.cpp
)
target_link_libraries(manipulation_pipeline
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES}
)

# Experience trainer library
add_library(learning_pipeline
  src/learning_pipeline.cpp
)
target_link_libraries(learning_pipeline
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES}
  manipulation_pipeline
)

# Main Executable
add_executable(apc_manager src/apc_manager.cpp)
target_link_libraries(apc_manager 
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES}
  shelf
  amazon_json_parser
  manipulation_pipeline
  learning_pipeline
)

# TESTS
add_executable(mesh_publisher tests/mesh_publisher.cpp)
target_link_libraries(mesh_publisher 
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES}
)

# Install
install(
  TARGETS
    apc_manager
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
