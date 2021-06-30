#!/bin/zsh

# Launch the robot
# source /opt/ros/melodic/setup.bash 
# source ~/socnav_ws/devel/setup.bash
source /opt/ros/melodic/setup.zsh
source ~/research_ws/devel/setup.zsh

echo "Starting the user study, kindly wait!"

PARTICIPANT_ID="P00"

########################################## learning phase #############################################

# block 1 ---------------------------------------------------------------------------------------------
CONDITION="SNA"

# 1
roslaunch socially_aware_rvo trial.launch scenario:="approach-02" trial_name:=$PARTICIPANT_ID trial_condition:=$CONDITION
# 2
roslaunch socially_aware_rvo trial.launch scenario:="crossing-01" trial_name:=$PARTICIPANT_ID trial_condition:=$CONDITION
# 3
roslaunch socially_aware_rvo trial.launch scenario:="random-03" trial_name:=$PARTICIPANT_ID trial_condition:=$CONDITION

# -----------------------------------------------------------------------------------------------------
 
 echo "
 -------------------Block 1 complete---------------------
 "

# block 2 ---------------------------------------------------------------------------------------------
CONDITION="MC"

# 1
roslaunch socially_aware_rvo trial.launch scenario:="random-01" trial_name:=$PARTICIPANT_ID trial_condition:=$CONDITION
# 2
roslaunch socially_aware_rvo trial.launch scenario:="crossing-02" trial_name:=$PARTICIPANT_ID trial_condition:=$CONDITION
# 3
roslaunch socially_aware_rvo trial.launch scenario:="approach-03" trial_name:=$PARTICIPANT_ID trial_condition:=$CONDITION

# -----------------------------------------------------------------------------------------------------

echo "
 -------------------Block 2 complete---------------------
 "

# block 3 ---------------------------------------------------------------------------------------------
CONDITION="SNA-V"

# 1
roslaunch socially_aware_rvo trial.launch scenario:="crossing-03" trial_name:=$PARTICIPANT_ID trial_condition:=$CONDITION
# 2
roslaunch socially_aware_rvo trial.launch scenario:="approach-01" trial_name:=$PARTICIPANT_ID trial_condition:=$CONDITION
# 3
roslaunch socially_aware_rvo trial.launch scenario:="random-02" trial_name:=$PARTICIPANT_ID trial_condition:=$CONDITION

# -----------------------------------------------------------------------------------------------------

echo "
 -------------------Block 3 complete---------------------
 "