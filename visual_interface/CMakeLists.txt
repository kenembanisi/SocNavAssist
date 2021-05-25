cmake_minimum_required(VERSION 3.0.2)
project(visual_interface)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  geometry_msgs
  std_msgs
  message_generation
  tf
)

find_package(OpenCV 3.2.0 REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


add_message_files(
 FILES
 trajectory.msg
 trajectoryPair.msg
)


generate_messages(
 DEPENDENCIES
 std_msgs
)


catkin_package(
 INCLUDE_DIRS
 CATKIN_DEPENDS cv_bridge 
                image_transport 
                roscpp 
                sensor_msgs 
                std_msgs
                message_runtime
)

###########
## Build ##
###########

message(STATUS "OpenCV_INCLUDE_DIRS = ${OpenCV_INCLUDE_DIRS}")
message(STATUS "catkin_INCLUDE_DIRS = ${catkin_INCLUDE_DIRS}")

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
  # ${OpenCV_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_node src/visual_interface_node.cpp)
add_executable(proxemics_node src/proxemics_node.cpp)
add_executable(predicted_path_viz src/predicted_path_viz.cpp)

add_dependencies(predicted_path_viz visual_interface_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(proxemics_node ${catkin_LIBRARIES})
target_link_libraries(predicted_path_viz ${catkin_LIBRARIES})

