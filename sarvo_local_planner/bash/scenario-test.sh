#!/bin/sh

# Launch the robot
# source /opt/ros/melodic/setup.bash
# source ~/workspaces/research_ws/devel/setup.bash

echo "Starting the user study, kindly wait!"

SCENARIO=$1
BEHAVIOR=$2
# FILENAME=$3
# WEIGHTS=$4

# # 1
roslaunch sarvo_local_planner scenario-test.launch scenario:=$SCENARIO \
                                          behavior:=$BEHAVIOR \
                                          paused:="false" \
                                          case1:=1 \
                                          trial_condition:=AUTO \
                                          trial_name:="auto-learning" \
                                          display_viz:=false \
                                          save_trial:=true

sleep 20s

# -----------------------------------------------------------------------------------------------------

echo "-------------------    Complete!    ---------------------"
