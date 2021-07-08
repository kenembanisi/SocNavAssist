#!/bin/zsh

# Launch the robot
source /opt/ros/melodic/setup.zsh
source ~/research_ws/devel/setup.zsh

echo "Starting the user study, kindly wait!"

PARTICIPANT_ID="P03"

PHASE="learning"

########################################## learning phase #############################################

# 1
roslaunch socially_aware_rvo trial.launch scenario:="approach-02" trial_name:=$PARTICIPANT_ID phase:=$PHASE trial_condition:="MC"
# 2
roslaunch socially_aware_rvo trial.launch scenario:="crossing-01" trial_name:=$PARTICIPANT_ID phase:=$PHASE trial_condition:="SNA"
# 3
roslaunch socially_aware_rvo trial.launch scenario:="random-03" trial_name:=$PARTICIPANT_ID phase:=$PHASE trial_condition:="SNA-VA"
# 4
roslaunch socially_aware_rvo trial.launch scenario:="random-01" trial_name:=$PARTICIPANT_ID phase:=$PHASE trial_condition:="SNA-VB"
# 5
roslaunch socially_aware_rvo trial.launch scenario:="crossing-02" trial_name:=$PARTICIPANT_ID phase:=$PHASE trial_condition:="SNA-VA"
# 6
roslaunch socially_aware_rvo trial.launch scenario:="approach-03" trial_name:=$PARTICIPANT_ID phase:=$PHASE trial_condition:="MC"
# 7
roslaunch socially_aware_rvo trial.launch scenario:="crossing-03" trial_name:=$PARTICIPANT_ID phase:=$PHASE trial_condition:="SNA"
# 8
roslaunch socially_aware_rvo trial.launch scenario:="approach-01" trial_name:=$PARTICIPANT_ID phase:=$PHASE trial_condition:="SNA-VB"


# -----------------------------------------------------------------------------------------------------

echo "

 -------------------    Learning Phase Complete!    ---------------------

 "