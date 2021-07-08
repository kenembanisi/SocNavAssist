#!/bin/zsh

# Launch the robot
# source /opt/ros/melodic/setup.bash 
# source ~/socnav_ws/devel/setup.bash
source /opt/ros/melodic/setup.zsh
source ~/research_ws/devel/setup.zsh

echo "Starting the user study, kindly wait!"

PARTICIPANT_ID="P03"

PHASE="test"

########################################## testing phase #############################################

# block 1 ---------------------------------------------------------------------------------------------
CONDITION="SNA-VA"

# 1
roslaunch socially_aware_rvo trial.launch scenario:="approach-02" trial_name:=$PARTICIPANT_ID phase:=$PHASE trial_condition:=$CONDITION
# 2
roslaunch socially_aware_rvo trial.launch scenario:="crossing-01" trial_name:=$PARTICIPANT_ID phase:=$PHASE trial_condition:=$CONDITION
# 3
roslaunch socially_aware_rvo trial.launch scenario:="random-03" trial_name:=$PARTICIPANT_ID phase:=$PHASE trial_condition:=$CONDITION

# -----------------------------------------------------------------------------------------------------
 
 echo "
 -------------------Block 1 complete---------------------
 "

 sleep 1m # waits for 1 minute

# block 2 ---------------------------------------------------------------------------------------------
CONDITION="MC"

# 1
roslaunch socially_aware_rvo trial.launch scenario:="random-01" trial_name:=$PARTICIPANT_ID phase:=$PHASE trial_condition:=$CONDITION
# 2
roslaunch socially_aware_rvo trial.launch scenario:="crossing-02" trial_name:=$PARTICIPANT_ID phase:=$PHASE trial_condition:=$CONDITION
# 3
roslaunch socially_aware_rvo trial.launch scenario:="approach-03" trial_name:=$PARTICIPANT_ID phase:=$PHASE trial_condition:=$CONDITION

# -----------------------------------------------------------------------------------------------------

echo "
 -------------------Block 2 complete---------------------
 "

 sleep 1m # waits for 1 minute

# block 3 ---------------------------------------------------------------------------------------------
CONDITION="SNA-VB"

1
roslaunch socially_aware_rvo trial.launch scenario:="crossing-03" trial_name:=$PARTICIPANT_ID phase:=$PHASE trial_condition:=$CONDITION
# 2
roslaunch socially_aware_rvo trial.launch scenario:="approach-01" trial_name:=$PARTICIPANT_ID phase:=$PHASE trial_condition:=$CONDITION
# 3
roslaunch socially_aware_rvo trial.launch scenario:="random-02" trial_name:=$PARTICIPANT_ID phase:=$PHASE trial_condition:=$CONDITION

# -----------------------------------------------------------------------------------------------------

echo "
 -------------------Block 3 complete---------------------
 "
sleep 1m # waits for 1 minute


 # block 4 ---------------------------------------------------------------------------------------------
CONDITION="SNA"

# 1
roslaunch socially_aware_rvo trial.launch scenario:="random-01" trial_name:=$PARTICIPANT_ID phase:=$PHASE trial_condition:=$CONDITION
# 2
roslaunch socially_aware_rvo trial.launch scenario:="approach-02" trial_name:=$PARTICIPANT_ID phase:=$PHASE trial_condition:=$CONDITION
# 3
roslaunch socially_aware_rvo trial.launch scenario:="crossing-01" trial_name:=$PARTICIPANT_ID phase:=$PHASE trial_condition:=$CONDITION

# -----------------------------------------------------------------------------------------------------

echo "

 -------------------Block 4 complete---------------------
 
 -------------------User Study is Over!!!---------------------

 "