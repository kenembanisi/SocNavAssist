#!/bin/sh

# Launch the robot
source /opt/ros/melodic/setup.bash
source ~/workspaces/research_ws/devel/setup.bash

echo "Starting the user study, kindly wait!"

PARTICIPANT_ID="S11"

PHASE="testing"
WORLD="crossing_dynamic-01"
LAYOUT="layout-01"

########################################## validation phase #############################################


# ****************************************************************************************************
# Trial block 1 
# ****************************************************************************************************

TASK="cautious_driving"

BEHAVIOR="none"

# 1
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          scenario_layout:=$LAYOUT \
                                          behavior:=$BEHAVIOR \
                                          trial_name:=$PARTICIPANT_ID \
                                          task_objective:=$TASK \
                                          case1:="1" \
                                          trial_condition:="HV-T" \
                                          display_viz:="true" \
                                          trial_number:="1/3" \
                                          trial_category:="#1" \
                                          trial_mode:="validation"
# 2
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          scenario_layout:=$LAYOUT \
                                          behavior:=$BEHAVIOR \
                                          trial_name:=$PARTICIPANT_ID \
                                          task_objective:=$TASK \
                                          case3:="1" \
                                          trial_condition:="HV-T" \
                                          display_viz:="true" \
                                          trial_number:="2/3" \
                                          trial_category:="#1" \
                                          trial_mode:="validation"
# 3
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          scenario_layout:=$LAYOUT \
                                          behavior:=$BEHAVIOR \
                                          trial_name:=$PARTICIPANT_ID \
                                          task_objective:=$TASK \
                                          case5:="1" \
                                          trial_condition:="HV-T" \
                                          display_viz:="true" \
                                          trial_number:="3/3" \
                                          trial_category:="#1" \
                                          trial_mode:="validation"

# waits for 10 seconds -----------------------------------------------------
echo "**********************************************************************"
echo " "
echo "                 First part of the validation complete!               "
echo " "
echo "**********************************************************************"
sleep 5s
# # --------------------------------------------------------------------------

WORLD="random_dynamic-01"
TASK="assertive_driving"

BEHAVIOR="none"

# 1
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          scenario_layout:=$LAYOUT \
                                          behavior:=$BEHAVIOR \
                                          trial_name:=$PARTICIPANT_ID \
                                          task_objective:=$TASK \
                                          case2:="1" \
                                          trial_condition:="HV-T" \
                                          display_viz:="true" \
                                          trial_number:="1/3" \
                                          trial_category:="#1" \
                                          trial_mode:="validation"
# 2
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          scenario_layout:=$LAYOUT \
                                          behavior:=$BEHAVIOR \
                                          trial_name:=$PARTICIPANT_ID \
                                          task_objective:=$TASK \
                                          case4:="1" \
                                          trial_condition:="HV-T" \
                                          display_viz:="true" \
                                          trial_number:="2/3" \
                                          trial_category:="#1" \
                                          trial_mode:="validation"
# 3
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          scenario_layout:=$LAYOUT \
                                          behavior:=$BEHAVIOR \
                                          trial_name:=$PARTICIPANT_ID \
                                          task_objective:=$TASK \
                                          case1:="1" \
                                          trial_condition:="HV-T" \
                                          display_viz:="true" \
                                          trial_number:="3/3" \
                                          trial_category:="#1" \
                                          trial_mode:="validation"

# waits for 10 seconds -----------------------------------------------------
echo "**********************************************************************"
echo " "
echo "                 Second part of the validation complete!               "
echo " "
echo "**********************************************************************"
sleep 15s
# --------------------------------------------------------------------------


######################################################################################################