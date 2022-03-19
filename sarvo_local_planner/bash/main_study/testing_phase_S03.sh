#!/bin/sh

# Launch the robot
source /opt/ros/melodic/setup.bash
source ~/workspaces/research_ws/devel/setup.bash

echo "Starting the user study, kindly wait!"

PARTICIPANT_ID="S03"

PHASE="testing"
WORLD="crossing_dynamic-02"
LAYOUT="layout-02"

########################################## testing phase #############################################

# ****************************************************************************************************
# Trial block 1 
# ****************************************************************************************************

TASK="assertive_driving"

BEHAVIOR="safety_aligned"

# 1
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          scenario_layout:=$LAYOUT \
                                          behavior:=$BEHAVIOR \
                                          trial_name:=$PARTICIPANT_ID \
                                          task_objective:=$TASK \
                                          case3:="1" \
                                          trial_condition:="HV-T" \
                                          display_viz:="true" \
                                          trial_number:="1/8" \
                                          trial_category:="#1" \
                                          trial_mode:="testing" 
# 2
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          scenario_layout:=$LAYOUT \
                                          behavior:=$BEHAVIOR \
                                          trial_name:=$PARTICIPANT_ID \
                                          task_objective:=$TASK \
                                          case2:="1" \
                                          trial_condition:="HV-T" \
                                          display_viz:="true" \
                                          trial_number:="2/8" \
                                          trial_category:="#1" \
                                          trial_mode:="testing"

# waits for 10 seconds -----------------------------------------------------
echo "**********************************************************************"
echo " "
echo "                 Part 1/4 of Category #1 completed!                  "
echo "                   Fill out questionnaire...                        "
echo " "
echo "**********************************************************************"
sleep 15s
# --------------------------------------------------------------------------

BEHAVIOR="goal_aligned"

# 1
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          scenario_layout:=$LAYOUT \
                                          behavior:=$BEHAVIOR \
                                          trial_name:=$PARTICIPANT_ID \
                                          task_objective:=$TASK \
                                          case5:="1" \
                                          trial_condition:="HV-T" \
                                          display_viz:="true" \
                                          trial_number:="3/8" \
                                          trial_category:="#1" \
                                          trial_mode:="testing"
# 2
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          scenario_layout:=$LAYOUT \
                                          behavior:=$BEHAVIOR \
                                          trial_name:=$PARTICIPANT_ID \
                                          task_objective:=$TASK \
                                          case1:="1" \
                                          trial_condition:="HV-T" \
                                          display_viz:="true" \
                                          trial_number:="4/8" \
                                          trial_category:="#1" \
                                          trial_mode:="testing"

# waits for 10 seconds -----------------------------------------------------
echo "**********************************************************************"
echo " "
echo "                 Part 2/4 of Category #1 completed!                  "
echo "                   Fill out questionnaire...                        "
echo " "
echo "**********************************************************************"
sleep 15s
# --------------------------------------------------------------------------

WORLD="random_dynamic-02"
BEHAVIOR="safety_aligned"

# 1
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          scenario_layout:=$LAYOUT \
                                          behavior:=$BEHAVIOR \
                                          trial_name:=$PARTICIPANT_ID \
                                          task_objective:=$TASK \
                                          case4:="1" \
                                          trial_condition:="HV-T" \
                                          display_viz:="true" \
                                          distracted_mode:="true" \
                                          trial_number:="5/8" \
                                          trial_category:="#1" \
                                          trial_mode:="testing"
# 2
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          scenario_layout:=$LAYOUT \
                                          behavior:=$BEHAVIOR \
                                          trial_name:=$PARTICIPANT_ID \
                                          task_objective:=$TASK \
                                          case2:="1" \
                                          trial_condition:="HV-T" \
                                          display_viz:="true" \
                                          distracted_mode:="true" \
                                          trial_number:="6/8" \
                                          trial_category:="#1" \
                                          trial_mode:="testing"


# waits for 10 seconds -----------------------------------------------------
echo "**********************************************************************"
echo " "
echo "                 Part 3/4 of Category #1 completed!                  "
echo "                   Fill out questionnaire...                        "
echo " "
echo "**********************************************************************"
sleep 15s
# --------------------------------------------------------------------------

BEHAVIOR="goal_aligned"

# 1
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          scenario_layout:=$LAYOUT \
                                          behavior:=$BEHAVIOR \
                                          trial_name:=$PARTICIPANT_ID \
                                          task_objective:=$TASK \
                                          case1:="1" \
                                          trial_condition:="HV-T" \
                                          display_viz:="true" \
                                          distracted_mode:="true" \
                                          trial_number:="7/8" \
                                          trial_category:="#1" \
                                          trial_mode:="testing"
# 2
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          scenario_layout:=$LAYOUT \
                                          behavior:=$BEHAVIOR \
                                          trial_name:=$PARTICIPANT_ID \
                                          task_objective:=$TASK \
                                          case4:="1" \
                                          trial_condition:="HV-T" \
                                          display_viz:="true" \
                                          distracted_mode:="true" \
                                          trial_number:="8/8" \
                                          trial_category:="#1" \
                                          trial_mode:="testing"

# waits for 10 seconds -----------------------------------------------------
echo "**********************************************************************"
echo " "
echo "                 Part 4/4 of Category #1 completed!                  "
echo "                   Fill out questionnaire...                        "
echo " "
echo "**********************************************************************"
sleep 15s
# --------------------------------------------------------------------------

######################################################################################################

# ****************************************************************************************************
# Trial block 2 
# ****************************************************************************************************

WORLD="random_dynamic-02"

TASK="cautious_driving"
LAYOUT="layout-01"

BEHAVIOR="goal_aligned"

# 1
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          scenario_layout:=$LAYOUT \
                                          behavior:=$BEHAVIOR \
                                          trial_name:=$PARTICIPANT_ID \
                                          task_objective:=$TASK \
                                          case2:="1" \
                                          trial_condition:="HV-T" \
                                          display_viz:="true" \
                                          trial_number:="1/8" \
                                          trial_category:="#2" \
                                          trial_mode:="testing"
# 2
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          scenario_layout:=$LAYOUT \
                                          behavior:=$BEHAVIOR \
                                          trial_name:=$PARTICIPANT_ID \
                                          task_objective:=$TASK \
                                          case4:="1" \
                                          trial_condition:="HV-T" \
                                          display_viz:="true" \
                                          trial_number:="2/8" \
                                          trial_category:="#2" \
                                          trial_mode:="testing"

# waits for 10 seconds -----------------------------------------------------
echo "**********************************************************************"
echo " "
echo "                 Part 1/4 of Category #2 completed!                  "
echo "                   Fill out questionnaire...                        "
echo " "
echo "**********************************************************************"
sleep 15s
# --------------------------------------------------------------------------

BEHAVIOR="safety_aligned"

# 1
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          scenario_layout:=$LAYOUT \
                                          behavior:=$BEHAVIOR \
                                          trial_name:=$PARTICIPANT_ID \
                                          task_objective:=$TASK \
                                          case2:="1" \
                                          trial_condition:="HV-T" \
                                          display_viz:="true" \
                                          trial_number:="3/8" \
                                          trial_category:="#2" \
                                          trial_mode:="testing"
# 2
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          scenario_layout:=$LAYOUT \
                                          behavior:=$BEHAVIOR \
                                          trial_name:=$PARTICIPANT_ID \
                                          task_objective:=$TASK \
                                          case1:="1" \
                                          trial_condition:="HV-T" \
                                          display_viz:="true" \
                                          trial_number:="4/8" \
                                          trial_category:="#2" \
                                          trial_mode:="testing"

# waits for 10 seconds -----------------------------------------------------
echo "**********************************************************************"
echo " "
echo "                 Part 2/4 of Category #2 completed!                  "
echo "                   Fill out questionnaire...                        "
echo " "
echo "**********************************************************************"
sleep 15s
# --------------------------------------------------------------------------

WORLD="crossing_dynamic-02"
BEHAVIOR="goal_aligned"
LAYOUT="layout-01"

# 1
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          scenario_layout:=$LAYOUT \
                                          behavior:=$BEHAVIOR \
                                          trial_name:=$PARTICIPANT_ID \
                                          task_objective:=$TASK \
                                          case5:="1" \
                                          trial_condition:="HV-T" \
                                          display_viz:="true" \
                                          distracted_mode:="true" \
                                          trial_number:="5/8" \
                                          trial_category:="#2" \
                                          trial_mode:="testing"
# 2
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          scenario_layout:=$LAYOUT \
                                          behavior:=$BEHAVIOR \
                                          trial_name:=$PARTICIPANT_ID \
                                          task_objective:=$TASK \
                                          case3:="1" \
                                          trial_condition:="HV-T" \
                                          display_viz:="true" \
                                          distracted_mode:="true" \
                                          trial_number:="6/8" \
                                          trial_category:="#2" \
                                          trial_mode:="testing"


# waits for 10 seconds -----------------------------------------------------
echo "**********************************************************************"
echo " "
echo "                 Part 3/4 of Category #2 completed!                  "
echo "                   Fill out questionnaire...                        "
echo " "
echo "**********************************************************************"
sleep 15s
# --------------------------------------------------------------------------

BEHAVIOR="safety_aligned"

# 1
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          scenario_layout:=$LAYOUT \
                                          behavior:=$BEHAVIOR \
                                          trial_name:=$PARTICIPANT_ID \
                                          task_objective:=$TASK \
                                          case1:="1" \
                                          trial_condition:="HV-T" \
                                          display_viz:="true" \
                                          distracted_mode:="true" \
                                          trial_number:="7/8" \
                                          trial_category:="#2" \
                                          trial_mode:="testing"
# 2
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          scenario_layout:=$LAYOUT \
                                          behavior:=$BEHAVIOR \
                                          trial_name:=$PARTICIPANT_ID \
                                          task_objective:=$TASK \
                                          case2:="1" \
                                          trial_condition:="HV-T" \
                                          display_viz:="true" \
                                          distracted_mode:="true" \
                                          trial_number:="8/8" \
                                          trial_category:="#2" \
                                          trial_mode:="testing"

# waits for 10 seconds -----------------------------------------------------
echo "**********************************************************************"
echo " "
echo "                 Part 4/4 of Category #2 completed!                  "
echo "                   Fill out questionnaire...                        "
echo " "
echo "**********************************************************************"
sleep 15s
# --------------------------------------------------------------------------

######################################################################################################