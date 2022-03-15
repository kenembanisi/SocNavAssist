#!/bin/sh

# Launch the robot
source /opt/ros/melodic/setup.bash
source ~/workspaces/research_ws/devel/setup.bash

echo "Starting the user study, kindly wait!"

PARTICIPANT_ID="P05"

PHASE="learning"

########################################## learning phase #############################################


# *****************************************************************************************************
# CATEGORY #4:
# -----------
# World:            dynamic01
# Layout:           layout-01
# Trial condition:  Assisted case (HV-T)
# Behaviors:        Mixed (cautious & assertive)
# Tasks:            Mixed (cautious & assertive)
# Distracted:       Yes
# *****************************************************************************************************

WORLD="crossing_dynamic-01"

TASK_1="cautious_driving"
TASK_2="assertive_driving"

# BEHAVIOR_1="cautious"
# BEHAVIOR_2="assertive"
BEHAVIOR_1="none"
BEHAVIOR_2="none"

# 1
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:="none" \
                                          trial_name:=$PARTICIPANT_ID \
                                          task_objective:="none" \
                                          case3:="1" \
                                          trial_condition:="MC" \
                                          display_viz:="true" \
                                          trial_number:="1/2" \
                                          trial_category:="#4" \
                                          distracted_mode:="true"
# 2
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:="none" \
                                          trial_name:=$PARTICIPANT_ID \
                                          task_objective:="none" \
                                          case3:="1" \
                                          trial_condition:="MC" \
                                          display_viz:="true" \
                                          trial_number:="2/2" \
                                          trial_category:="#4" \
                                          distracted_mode:="true"

# *****************************************************************************************************
# 3
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:=$BEHAVIOR_2 \
                                          trial_name:=$PARTICIPANT_ID \
                                          task_objective:=$TASK_2 \
                                          case4:="1" \
                                          trial_condition:="HV-T" \
                                          display_viz:="true" \
                                          trial_number:="1/4" \
                                          trial_category:="#4" \
                                          distracted_mode:="true"
# 4
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:=$BEHAVIOR_2 \
                                          trial_name:=$PARTICIPANT_ID \
                                          task_objective:=$TASK_2 \
                                          case4:="1" \
                                          trial_condition:="HV-T" \
                                          display_viz:="true" \
                                          trial_number:="2/4" \
                                          trial_category:="#4" \
                                          distracted_mode:="true"

# *****************************************************************************************************

WORLD="random_dynamic-01"

# 5
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:=$BEHAVIOR_1 \
                                          trial_name:=$PARTICIPANT_ID \
                                          task_objective:=$TASK_1 \
                                          case2:="1" \
                                          trial_condition:="HV-T" \
                                          display_viz:="true" \
                                          trial_number:="3/4" \
                                          trial_category:="#4" \
                                          distracted_mode:="true"
# 6
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:=$BEHAVIOR_1 \
                                          trial_name:=$PARTICIPANT_ID \
                                          task_objective:=$TASK_1 \
                                          case2:="1" \
                                          trial_condition:="HV-T" \
                                          display_viz:="true" \
                                          trial_number:="4/4" \
                                          trial_category:="#4" \
                                          distracted_mode:="true"



########################################################################################################

