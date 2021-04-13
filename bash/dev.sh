#!/usr/bin/env bash

# Launch the robot
source /opt/ros/melodic/setup.bash 
source ~/catkin_ws/devel/setup.bash

echo "Starting the dev test, kindly wait!"

PARTICIPANT_ID="auto"

roslaunch rvo_ros trial.launch  \
    scenario:="approach_human"  \
    trial_name:=$PARTICIPANT_ID \
    control_mode:="auto"        \
    paused:="false"             \
    rvo_planning_horizon:="3.5"

roslaunch rvo_ros trial.launch  \
    scenario:="approach_human"  \
    trial_name:=$PARTICIPANT_ID \
    control_mode:="auto"        \   
    paused:="false"             \
    rvo_planning_horizon:="3"



 
