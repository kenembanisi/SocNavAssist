#!/usr/bin/env bash

# Launch the robot
source /opt/ros/melodic/setup.bash 
source ~/catkin_ws/devel/setup.bash

echo "Starting the user study, kindly wait!"

PARTICIPANT_ID="P04"

roslaunch rvo_ros trial.launch scenario:="approach_human" trial_name:=$PARTICIPANT_ID # approach 1

 
roslaunch rvo_ros trial.launch scenario:="crossing_human" trial_name:=$PARTICIPANT_ID # crossing 1



roslaunch rvo_ros trial.launch scenario:="random_human" trial_name:=$PARTICIPANT_ID # random 1



roslaunch rvo_ros trial.launch scenario:="ahead_human" trial_name:=$PARTICIPANT_ID # ahead 1



roslaunch rvo_ros trial.launch scenario:="crossing_human" trial_name:=$PARTICIPANT_ID # crossing 2



roslaunch rvo_ros trial.launch scenario:="approach_human" trial_name:=$PARTICIPANT_ID # approach 2



roslaunch rvo_ros trial.launch scenario:="ahead_human" trial_name:=$PARTICIPANT_ID # ahead 2



roslaunch rvo_ros trial.launch scenario:="random_human" trial_name:=$PARTICIPANT_ID # random 2