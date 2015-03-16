cmake_minimum_required(VERSION 2.8.3)
project(picknik_main)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_grasps
  moveit_ompl
  moveit_visual_tools
  ompl_visual_tools
  cmake_modules
  picknik_msgs
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
    picknik_msgs
  DEPENDS
    Eigen
  INCLUDE_DIRS 
    include
    ${OMPL_INCLUDE_DIRS}
  LIBRARIES
    jsoncpp
    amazon_json_parser
    shelf
    manipulation
    #learning_pipeline
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
add_library(manipulation
  src/manipulation.cpp
)
add_dependencies(manipulation picknik_main_generate_messages_cpp)
target_link_libraries(manipulation
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

# Main logic of APC
add_library(apc_manager
  src/apc_manager.cpp
)
target_link_libraries(apc_manager
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES}
  shelf
  amazon_json_parser
  manipulation
  # learning_pipeline
)

# Main Executable
add_executable(main src/main.cpp)
target_link_libraries(main 
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES}
  amazon_json_parser
  apc_manager
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
