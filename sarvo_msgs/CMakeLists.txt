cmake_minimum_required(VERSION 3.0.2)
project(sarvo_msgs)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  std_msgs
)

add_message_files(
  FILES
  Candidate.msg
  CandidateScore.msg
  FeatureScore.msg
  Person.msg
  Point2D.msg
  Pose2D.msg
  Trajectory2D.msg
  Twist2D.msg
  SimulationStates.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs std_msgs
)

catkin_package(
 CATKIN_DEPENDS geometry_msgs message_runtime std_msgs
)

