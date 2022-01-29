#!/bin/sh

# Launch the robot
# source /opt/ros/melodic/setup.bash
# source ~/workspaces/research_ws/devel/setup.bash

echo "Starting the user study, kindly wait!"

SCENARIO=$1
FILENAME=$2
WEIGHTS=$3
DISP=$4

# 1
roslaunch sarvo_local_planner test.launch scenario:=$SCENARIO \
                                          feature_filename:=$FILENAME \
                                          behavior_weights:=$WEIGHTS \
                                          paused:="false" 
                                          display_viz:=$DISP

# -----------------------------------------------------------------------------------------------------

echo "-------------------    Complete!    ---------------------"
