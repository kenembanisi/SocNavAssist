#!/bin/sh

# Launch the robot
source /opt/ros/melodic/setup.bash
source ~/workspaces/research_ws/devel/setup.bash

echo "Starting the user study, kindly wait!"

PARTICIPANT_ID="S16"

PHASE="test"

########################################## testing phase #############################################

# block 1 ---------------------------------------------------------------------------------------------
# CONDITION="V-T"

# # 1
# roslaunch socially_aware_rvo trial.launch scenario:="approach-01" scenario_layout:="layout-01" \
#                                           trial_name:=$PARTICIPANT_ID \
#                                           phase:=$PHASE trial_condition:=$CONDITION \
#                                           trial_number:="1/18" case4:="1"
# # 2
# roslaunch socially_aware_rvo trial.launch scenario:="crossing-03" scenario_layout:="layout-01" \
#                                           trial_name:=$PARTICIPANT_ID \
#                                           phase:=$PHASE trial_condition:=$CONDITION \
#                                           trial_number:="2/18" case5:="1"
# # 3
# roslaunch socially_aware_rvo trial.launch scenario:="random-02" scenario_layout:="layout-01" \
#                                           trial_name:=$PARTICIPANT_ID \
#                                           phase:=$PHASE trial_condition:=$CONDITION \
#                                           trial_number:="3/18" case1:="1"



# -----------------------------------------------------------------------------------------------------
 
#  echo "
#  -------------------Block 1 complete---------------------
#  "

#  sleep 1m # waits for 1 minute

# block 2 ---------------------------------------------------------------------------------------------
# CONDITION="H"

# 1
# roslaunch socially_aware_rvo trial.launch scenario:="approach-01" scenario_layout:="layout-02" \
#                                           trial_name:=$PARTICIPANT_ID \
#                                           phase:=$PHASE trial_condition:=$CONDITION \
#                                           trial_number:="4/18" case1:="1"
# 2
# roslaunch socially_aware_rvo trial.launch scenario:="random-01" scenario_layout:="layout-02" \
#                                           trial_name:=$PARTICIPANT_ID \
#                                           phase:=$PHASE trial_condition:=$CONDITION \
#                                           trial_number:="5/18" case3:="1"
# # 3
# roslaunch socially_aware_rvo trial.launch scenario:="crossing-03" scenario_layout:="layout-01" \
#                                           trial_name:=$PARTICIPANT_ID \
#                                           phase:=$PHASE trial_condition:=$CONDITION \
#                                           trial_number:="6/18" case1:="1"
# -----------------------------------------------------------------------------------------------------

# echo "
#  -------------------Block 2 complete---------------------
#  "

#  sleep 1m # waits for 1 minute

# block 3 ---------------------------------------------------------------------------------------------
# CONDITION="HV-T"

# 1
# roslaunch socially_aware_rvo trial.launch scenario:="crossing-01" scenario_layout:="layout-01" \
#                                           trial_name:=$PARTICIPANT_ID \
#                                           phase:=$PHASE trial_condition:=$CONDITION \
#                                           trial_number:="7/18" case2:="1"
# 2
# roslaunch socially_aware_rvo trial.launch scenario:="approach-01" scenario_layout:="layout-02" \
#                                           trial_name:=$PARTICIPANT_ID \
#                                           phase:=$PHASE trial_condition:=$CONDITION \
#                                           trial_number:="8/18" case6:="1"
# # 3
# roslaunch socially_aware_rvo trial.launch scenario:="random-01" scenario_layout:="layout-01" \
#                                           trial_name:=$PARTICIPANT_ID \
#                                           phase:=$PHASE trial_condition:=$CONDITION \
#                                           trial_number:="9/18" case5:="1"
# -----------------------------------------------------------------------------------------------------

# echo "
#  -------------------Block 3 complete---------------------
#  "
# sleep 1m # waits for 1 minute


 # block 4 ---------------------------------------------------------------------------------------------
# CONDITION="V-B"

# 1
# roslaunch socially_aware_rvo trial.launch scenario:="crossing-01" scenario_layout:="layout-01" \
#                                           trial_name:=$PARTICIPANT_ID \
#                                           phase:=$PHASE trial_condition:=$CONDITION \
#                                           trial_number:="10/18" case1:="1"
# # 2
# roslaunch socially_aware_rvo trial.launch scenario:="random-02" scenario_layout:="layout-01" \
#                                           trial_name:=$PARTICIPANT_ID \
#                                           phase:=$PHASE trial_condition:=$CONDITION \
#                                           trial_number:="11/18" case3:="1"
# 3
# roslaunch socially_aware_rvo trial.launch scenario:="approach-02" scenario_layout:="layout-01" \
#                                           trial_name:=$PARTICIPANT_ID \
#                                           phase:=$PHASE trial_condition:=$CONDITION \
#                                           trial_number:="12/18" case4:="1"
# -----------------------------------------------------------------------------------------------------

# echo "
#  -------------------Block 4 complete---------------------
#  "

# sleep 1m # waits for 1 minute


 # block 5 ---------------------------------------------------------------------------------------------
CONDITION="MC"

# 1
roslaunch socially_aware_rvo trial.launch scenario:="random-02" scenario_layout:="layout-01" \
                                          trial_name:=$PARTICIPANT_ID \
                                          phase:=$PHASE trial_condition:=$CONDITION \
                                          trial_number:="13/18" case5:="1"
# 2
roslaunch socially_aware_rvo trial.launch scenario:="approach-02" scenario_layout:="layout-02" \
                                          trial_name:=$PARTICIPANT_ID \
                                          phase:=$PHASE trial_condition:=$CONDITION \
                                          trial_number:="14/18" case6:="1"
# 3
roslaunch socially_aware_rvo trial.launch scenario:="crossing-03" scenario_layout:="layout-01" \
                                          trial_name:=$PARTICIPANT_ID \
                                          phase:=$PHASE trial_condition:=$CONDITION \
                                          trial_number:="15/18" case5:="1"
# -----------------------------------------------------------------------------------------------------

echo "
 -------------------Block 5 complete---------------------
 "

 sleep 1m # waits for 1 minute

# block 6 ---------------------------------------------------------------------------------------------

CONDITION="HV-B"

# 1
roslaunch socially_aware_rvo trial.launch scenario:="random-01" scenario_layout:="layout-01" \
                                          trial_name:=$PARTICIPANT_ID \
                                          phase:=$PHASE trial_condition:=$CONDITION \
                                          trial_number:="16/18" case1:="1"
# 2
roslaunch socially_aware_rvo trial.launch scenario:="crossing-03" scenario_layout:="layout-02" \
                                          trial_name:=$PARTICIPANT_ID \
                                          phase:=$PHASE trial_condition:=$CONDITION \
                                          trial_number:="17/18" case2:="1"
# 3
roslaunch socially_aware_rvo trial.launch scenario:="approach-02" scenario_layout:="layout-01" \
                                          trial_name:=$PARTICIPANT_ID \
                                          phase:=$PHASE trial_condition:=$CONDITION \
                                          trial_number:="18/18" case1:="1"
# -----------------------------------------------------------------------------------------------------

echo "

 -------------------Block 6 complete---------------------
 
 -------------------User Study is Over!!!---------------------

 "