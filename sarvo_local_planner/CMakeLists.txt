cmake_minimum_required(VERSION 3.0.2)
project(sarvo_local_planner)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  costmap_2d
  nav_msgs
  sarvo_msgs
  spencer_tracking_msgs
)

# find_package(OpenCV 3.2.0 REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


catkin_package( 
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs spencer_tracking_msgs nav_msgs geometry_msgs
)

###########
## Build ##
###########

message(STATUS "catkin_INCLUDE_DIRS = ${catkin_INCLUDE_DIRS}")

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(trajectory_generator include/sarvo_local_planner/trajectory_generator.h
                                 src/trajectory_generator.cpp)
add_library(trajectory_critic include/sarvo_local_planner/trajectory_critic.h
                              src/trajectory_critic.cpp)

# target_link_libraries(sarvo_local_planner ${catkin_LIBRARIES})
# add_dependencies(sarvo_local_planner ${catkin_EXPORTED_TARGETS})

# add_executable(sarvo_local_planner_node src/sarvo_local_planner_node.cpp)
add_executable(${PROJECT_NAME} src/sarvo_local_planner.cpp)

# add_dependencies(predicted_path_viz visual_interface_generate_messages_cpp)
# add_dependencies(${PROJECT_NAME} spencer_tracking_msgs)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} trajectory_generator trajectory_critic)
# target_link_libraries(${PROJECT_NAME} PRIVATE trajectory_generator)

