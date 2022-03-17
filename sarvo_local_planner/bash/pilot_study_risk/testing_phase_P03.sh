#!/bin/sh

# Launch the robot
source /opt/ros/melodic/setup.bash
source ~/workspaces/research_ws/devel/setup.bash

echo "Starting the user study, kindly wait!"

PARTICIPANT_ID="P03-RISK"

PHASE="testing"
WORLD="crossing_dynamic-02"
LAYOUT="layout-02"
RISK_CONDITION_1="A(H+V)"
RISK_CONDITION_2="NA(H+V)"
RISK_CONDITION_3="NA(V)"

########################################## testing phase #############################################

# ****************************************************************************************************
# Trial block 1 
# ****************************************************************************************************

# 1
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          trial_name:=$PARTICIPANT_ID \
                                          case3:="1" \
                                          trial_condition:="HV-T" \
                                          trial_number:="1/12" \
                                          trial_category:="#3" \
                                          display_viz:="true" \
                                          risk_enabled:= "true" \
                                          risk_condition:=$RISK_CONDITION_1 \
                                          trial_mode:="testing"
# 2
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          trial_name:=$PARTICIPANT_ID \
                                          case5:="1" \
                                          trial_condition:="HV-T" \
                                          trial_number:="2/12" \
                                          trial_category:="#3" \
                                          display_viz:="true" \
                                          risk_enabled:= "true" \
                                          risk_condition:=$RISK_CONDITION_1 \
                                          trial_mode:="testing"

# waits for 10 seconds -----------------------------------------------------
echo "**********************************************************************"
echo " "
echo "                 Part 1/6 completed!                  "
echo "                   Fill out questionnaire...                        "
echo " "
echo "**********************************************************************"
sleep 15s
# --------------------------------------------------------------------------


# 1
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          trial_name:=$PARTICIPANT_ID \
                                          case1:="1" \
                                          trial_condition:="HV-T" \
                                          trial_number:="3/12" \
                                          trial_category:="#3" \
                                          display_viz:="true" \
                                          risk_enabled:= "true" \
                                          risk_condition:=$RISK_CONDITION_2 \
                                          trial_mode:="testing"
# 2
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          trial_name:=$PARTICIPANT_ID \
                                          case2:="1" \
                                          trial_condition:="HV-T" \
                                          trial_number:="4/12" \
                                          trial_category:="#3" \
                                          display_viz:="true" \
                                          risk_enabled:= "true" \
                                          risk_condition:=$RISK_CONDITION_2 \
                                          trial_mode:="testing"

# waits for 10 seconds -----------------------------------------------------
echo "**********************************************************************"
echo " "
echo "                      Part 2/6 completed!                  "
echo "                   Fill out questionnaire...                        "
echo " "
echo "**********************************************************************"
sleep 15s
# --------------------------------------------------------------------------

WORLD="random_dynamic-02"

# 1
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          trial_name:=$PARTICIPANT_ID \
                                          case1:="1" \
                                          trial_condition:="HV-T" \
                                          trial_number:="5/12" \
                                          trial_category:="#3" \
                                          display_viz:="true" \
                                          risk_enabled:= "true" \
                                          risk_condition:=$RISK_CONDITION_3 \
                                          trial_mode:="testing"
# 2
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          trial_name:=$PARTICIPANT_ID \
                                          case4:="1" \
                                          trial_condition:="HV-T" \
                                          trial_number:="6/12" \
                                          trial_category:="#3" \
                                          display_viz:="true" \
                                          risk_enabled:= "true" \
                                          risk_condition:=$RISK_CONDITION_3 \
                                          trial_mode:="testing"

# waits for 10 seconds -----------------------------------------------------
echo "**********************************************************************"
echo " "
echo "                 Part 3/6 completed!                  "
echo "                   Fill out questionnaire...                        "
echo " "
echo "**********************************************************************"
sleep 15s
# --------------------------------------------------------------------------



######################################################################################################

# ****************************************************************************************************
# Trial block 2 
# ****************************************************************************************************

WORLD="crossing_dynamic-02"
LAYOUT="layout-01"


# 1
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          trial_name:=$PARTICIPANT_ID \
                                          case5:="1" \
                                          trial_condition:="HV-T" \
                                          trial_number:="7/12" \
                                          trial_category:="#3" \
                                          display_viz:="true" \
                                          risk_enabled:= "true" \
                                          risk_condition:=$RISK_CONDITION_1 \
                                          trial_mode:="testing" \
                                          distracted_mode:="true"
# 2
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          trial_name:=$PARTICIPANT_ID \
                                          case3:="1" \
                                          trial_condition:="HV-T" \
                                          trial_number:="8/12" \
                                          trial_category:="#3" \
                                          display_viz:="true" \
                                          risk_enabled:= "true" \
                                          risk_condition:=$RISK_CONDITION_1 \
                                          trial_mode:="testing" \
                                          distracted_mode:="true"

# waits for 10 seconds -----------------------------------------------------
echo "**********************************************************************"
echo " "
echo "                 Part 4/6 completed!                  "
echo "                   Fill out questionnaire...                        "
echo " "
echo "**********************************************************************"
sleep 15s
# --------------------------------------------------------------------------


# 1
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          trial_name:=$PARTICIPANT_ID \
                                          case1:="1" \
                                          trial_condition:="HV-T" \
                                          trial_number:="9/12" \
                                          trial_category:="#3" \
                                          display_viz:="true" \
                                          risk_enabled:= "true" \
                                          risk_condition:=$RISK_CONDITION_2 \
                                          trial_mode:="testing" \
                                          distracted_mode:="true"
# 2
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          trial_name:=$PARTICIPANT_ID \
                                          case2:="1" \
                                          trial_condition:="HV-T" \
                                          trial_number:="10/12" \
                                          trial_category:="#3" \
                                          display_viz:="true" \
                                          risk_enabled:= "true" \
                                          risk_condition:=$RISK_CONDITION_2 \
                                          trial_mode:="testing" \
                                          distracted_mode:="true"

# waits for 10 seconds -----------------------------------------------------
echo "**********************************************************************"
echo " "
echo "                      Part 5/6 completed!                  "
echo "                   Fill out questionnaire...                        "
echo " "
echo "**********************************************************************"
sleep 15s
# --------------------------------------------------------------------------

WORLD="random_dynamic-02"

# 1
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          trial_name:=$PARTICIPANT_ID \
                                          case1:="1" \
                                          trial_condition:="HV-T" \
                                          trial_number:="5/12" \
                                          trial_category:="#3" \
                                          display_viz:="true" \
                                          risk_enabled:= "true" \
                                          risk_condition:=$RISK_CONDITION_3 \
                                          trial_mode:="testing" \
                                          distracted_mode:="true"
# 2
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          trial_name:=$PARTICIPANT_ID \
                                          case4:="1" \
                                          trial_condition:="HV-T" \
                                          trial_number:="6/12" \
                                          trial_category:="#3" \
                                          display_viz:="true" \
                                          risk_enabled:= "true" \
                                          risk_condition:=$RISK_CONDITION_3 \
                                          trial_mode:="testing" \
                                          distracted_mode:="true"

# waits for 10 seconds -----------------------------------------------------
echo "**********************************************************************"
echo " "
echo "                 Part 6/6 completed!                  "
echo "                   Fill out questionnaire...                        "
echo " "
echo "**********************************************************************"
sleep 15s
# --------------------------------------------------------------------------

######################################################################################################