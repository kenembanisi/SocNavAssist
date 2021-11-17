#!/bin/sh

# Launch the robot
source /opt/ros/melodic/setup.bash
source ~/workspaces/research_ws/devel/setup.bash

echo "Starting the user study, kindly wait!"

PARTICIPANT_ID="S19"

PHASE="learning"

########################################## learning phase #############################################

# 1
# roslaunch socially_aware_rvo trial.launch scenario:="approach-01" trial_name:=$PARTICIPANT_ID phase:=$PHASE trial_condition:="MC" trial_number:="1/12" case6:="1"
# 2
# roslaunch socially_aware_rvo trial.launch scenario:="crossing-01" trial_name:=$PARTICIPANT_ID phase:=$PHASE trial_condition:="H" trial_number:="2/12" case4:="1"
# 3
# roslaunch socially_aware_rvo trial.launch scenario:="random-02" trial_name:=$PARTICIPANT_ID phase:=$PHASE trial_condition:="V-T" trial_number:="3/12" case1:="1"
# 4
# roslaunch socially_aware_rvo trial.launch scenario:="random-01" trial_name:=$PARTICIPANT_ID phase:=$PHASE trial_condition:="HV-T" trial_number:="4/12" case4:="1"
# 5
# roslaunch socially_aware_rvo trial.launch scenario:="crossing-02" trial_name:=$PARTICIPANT_ID phase:=$PHASE trial_condition:="V-B" trial_number:="5/12" case6:="1"
# 6
# roslaunch socially_aware_rvo trial.launch scenario:="approach-02" trial_name:=$PARTICIPANT_ID phase:=$PHASE trial_condition:="HV-B" trial_number:="6/12" case1:="1"
# ------------------------------------------------------------------------------------------------------
# 7
# roslaunch socially_aware_rvo trial.launch scenario:="crossing-03" trial_name:=$PARTICIPANT_ID phase:=$PHASE trial_condition:="HV-T" trial_number:="7/12" case1:="1"
# 8
# roslaunch socially_aware_rvo trial.launch scenario:="approach-01" trial_name:=$PARTICIPANT_ID phase:=$PHASE trial_condition:="H" trial_number:="8/12" case6:="1"
# 9
# roslaunch socially_aware_rvo trial.launch scenario:="random-02" trial_name:=$PARTICIPANT_ID phase:=$PHASE trial_condition:="V-B" trial_number:="9/12" case1:="1"
# 10
# roslaunch socially_aware_rvo trial.launch scenario:="approach-02" trial_name:=$PARTICIPANT_ID phase:=$PHASE trial_condition:="MC" trial_number:="10/12" case4:="1"
# 11
roslaunch socially_aware_rvo trial.launch scenario:="random-01" trial_name:=$PARTICIPANT_ID phase:=$PHASE trial_condition:="HV-B" trial_number:="11/12" case1:="1"
# 12
roslaunch socially_aware_rvo trial.launch scenario:="crossing-01" trial_name:=$PARTICIPANT_ID phase:=$PHASE trial_condition:="V-T" trial_number:="12/12" case4:="1"


# -----------------------------------------------------------------------------------------------------

echo "

 -------------------    Learning Phase Complete!    ---------------------

 "