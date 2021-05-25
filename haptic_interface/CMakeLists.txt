cmake_minimum_required(VERSION 3.0.2)
project(haptic_interface)


## include packages to find
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  joy
  ros_falcon
)

# generate_messages(
#  DEPENDENCIES
#  std_msgs
# )

catkin_package(
 CATKIN_DEPENDS roscpp
)

## include "include" directories
include_directories(include ${catkin_INCLUDE_DIRS})

# include executables
add_executable(haptic_interface_node src/haptic_interface_node.cpp)
# add_executable(main src/main.cpp)

# include dependencies

# include target link libraries
target_link_libraries(haptic_interface_node ${catkin_LIBRARIES})
# target_link_libraries(main ${catkin_LIBRARIES})