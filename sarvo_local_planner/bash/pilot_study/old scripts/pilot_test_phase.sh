#!/bin/sh

# Launch the robot
source /opt/ros/melodic/setup.bash
source ~/workspaces/research_ws/devel/setup.bash

echo "Starting the user study, kindly wait!"

PARTICIPANT_ID="P03"

PHASE="learning"


# *****************************************************************************************************
# TEST PHASE A:
# ------------
# World:            Hall w/ many moving pedestrians -> dynamic02
# Trial condition:  Assisted case (HV-T)
# Behaviors:        Mixed (cautious & assertive)
# Tasks:            Mixed (cautious & assertive)
# *****************************************************************************************************

WORLD="dynamic01"
TRIAL_TASK="cautious_driving"

# PRACTICE DRIVE ----------------------------------------------------------------

# 1
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:="assertive" \
                                          trial_name:=$PARTICIPANT_ID \
                                          trial_task:=$TRIAL_TASK \
                                          case2:="1" \
                                          trial_condition:="MC" \
                                          display_viz:="true" \
                                          save_trial:="true" \
                                          trial_mode:="testing"
# 2
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:="cautious" \
                                          trial_name:=$PARTICIPANT_ID \
                                          trial_task:=$TRIAL_TASK \
                                          case3:="1" \
                                          trial_condition:="MC" \
                                          display_viz:="true" \
                                          save_trial:="true" \
                                          trial_mode:="testing"
# 3
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:="assertive" \
                                          trial_name:=$PARTICIPANT_ID \
                                          trial_task:=$TRIAL_TASK \
                                          case2:="1" \
                                          trial_condition:="HV-T" \
                                          display_viz:="true" \
                                          save_trial:="true" \
                                          trial_mode:="testing"
# 4
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:="cautious" \
                                          trial_name:=$PARTICIPANT_ID \
                                          trial_task:=$TRIAL_TASK \
                                          case3:="1" \
                                          trial_condition:="HV-T" \
                                          display_viz:="true" \
                                          save_trial:="true" \
                                          trial_mode:="testing"


# TEST DRIVE --------------------------------------------------------------------

# 1
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:="cautious" \
                                          trial_name:=$PARTICIPANT_ID \
                                          trial_task:=$TRIAL_TASK \
                                          case4:="1" \
                                          trial_condition:="HV-T" \
                                          display_viz:="true" \
                                          save_trial:="true" \
                                          trial_mode:="testing"
# 2
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:="cautious" \
                                          trial_name:=$PARTICIPANT_ID \
                                          trial_task:=$TRIAL_TASK \
                                          case1:="1" \
                                          trial_condition:="HV-T" \
                                          display_viz:="true" \
                                          save_trial:="true" \
                                          trial_mode:="testing"
# 3
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:="assertive" \
                                          trial_name:=$PARTICIPANT_ID \
                                          trial_task:=$TRIAL_TASK \
                                          case1:="1" \
                                          trial_condition:="HV-T" \
                                          display_viz:="true" \
                                          save_trial:="true" \
                                          trial_mode:="testing"
# 4
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:="assertive" \
                                          trial_name:=$PARTICIPANT_ID \
                                          trial_task:=$TRIAL_TASK \
                                          case4:="1" \
                                          trial_condition:="HV-T" \
                                          display_viz:="true" \
                                          save_trial:="true" \
                                          trial_mode:="testing"



# *****************************************************************************************************
# TEST PHASE B:
# ------------
# World:            Hall w/ many moving pedestrians -> dynamic02
# Trial condition:  Assisted case (HV-T)
# Behaviors:        Mixed (cautious & assertive)
# Tasks:            Mixed (cautious & assertive)
# *****************************************************************************************************

WORLD="dynamic01"
TRIAL_TASK="assertive_driving"

# PRACTICE DRIVE ----------------------------------------------------------------

# 1
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:="assertive" \
                                          trial_name:=$PARTICIPANT_ID \
                                          trial_task:=$TRIAL_TASK \
                                          case2:="1" \
                                          trial_condition:="MC" \
                                          display_viz:="true" \
                                          save_trial:="true" \
                                          trial_mode:="testing"
# 2
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:="assertive" \
                                          trial_name:=$PARTICIPANT_ID \
                                          trial_task:=$TRIAL_TASK \
                                          case1:="1" \
                                          trial_condition:="MC" \
                                          display_viz:="true" \
                                          save_trial:="true" \
                                          trial_mode:="testing"
# 1
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:="cautious" \
                                          trial_name:=$PARTICIPANT_ID \
                                          trial_task:=$TRIAL_TASK \
                                          case2:="1" \
                                          trial_condition:="HV-T" \
                                          display_viz:="true" \
                                          save_trial:="true" \
                                          trial_mode:="testing"
# 2
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:="cautious" \
                                          trial_name:=$PARTICIPANT_ID \
                                          trial_task:=$TRIAL_TASK \
                                          case1:="1" \
                                          trial_condition:="HV-T" \
                                          display_viz:="true" \
                                          save_trial:="true" \
                                          trial_mode:="testing"


# TEST DRIVE --------------------------------------------------------------------

# 1
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:="cautious" \
                                          trial_name:=$PARTICIPANT_ID \
                                          trial_task:=$TRIAL_TASK \
                                          case4:="1" \
                                          trial_condition:="HV-T" \
                                          display_viz:="true" \
                                          save_trial:="true" \
                                          trial_mode:="testing"
# 2
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:="cautious" \
                                          trial_name:=$PARTICIPANT_ID \
                                          trial_task:=$TRIAL_TASK \
                                          case3:="1" \
                                          trial_condition:="HV-T" \
                                          display_viz:="true" \
                                          save_trial:="true" \
                                          trial_mode:="testing"
# 3
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:="assertive" \
                                          trial_name:=$PARTICIPANT_ID \
                                          trial_task:=$TRIAL_TASK \
                                          case4:="1" \
                                          trial_condition:="HV-T" \
                                          display_viz:="true" \
                                          save_trial:="true" \
                                          trial_mode:="testing"
# 4
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:="assertive" \
                                          trial_name:=$PARTICIPANT_ID \
                                          trial_task:=$TRIAL_TASK \
                                          case3:="1" \
                                          trial_condition:="HV-T" \
                                          display_viz:="true" \
                                          save_trial:="true" \
                                          trial_mode:="testing"







# # ASSERTIVE DRIVING TASK #############################################################################

# ############ PRACTICE ####################################

# # WORLD="static02"

# # # 3
# # roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
# #                                           behavior:="assertive" \
# #                                           trial_name:=$PARTICIPANT_ID \
# #                                           trial_task:="cautious_driving" \
# #                                           trial_number:="11/16" case3:="1" \
# #                                           trial_condition:="MC" \
# #                                           display_viz:="true"
# # # 1
# # roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
# #                                           behavior:="cautious" \
# #                                           trial_name:=$PARTICIPANT_ID \
# #                                           trial_task:="assertive_driving" \
# #                                           trial_number:="9/16" case2:="1" \
# #                                           trial_condition:="MC" \
# #                                           display_viz:="true"
# # 2
# # roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
# #                                           behavior:="cautious" \
# #                                           trial_name:=$PARTICIPANT_ID \
# #                                           trial_task:="cautious_driving" \
# #                                           trial_number:="10/16" case1:="1" \
# #                                           trial_condition:="MC" \
# #                                           display_viz:="true"
# # # 4
# # roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
# #                                           behavior:="assertive" \
# #                                           trial_name:=$PARTICIPANT_ID \
# #                                           trial_task:="cautious_driving" \
# #                                           trial_number:="12/16" case4:="1" \
# #                                           trial_condition:="MC" \
# #                                           display_viz:="true"
# # 5
# # roslaunch sarvo_local_planner test.launch scenario:="dynamic01" \
# #                                           behavior:="assertive" \
# #                                           trial_name:=$PARTICIPANT_ID \
# #                                           trial_task:="cautious_driving" \
# #                                           trial_number:="10/16" case1:="1" \
# #                                           trial_condition:="MC" \
# #                                           display_viz:="true"
# # # 6
# # roslaunch sarvo_local_planner test.launch scenario:="dynamic01" \
# #                                           behavior:="cautious" \
# #                                           trial_name:=$PARTICIPANT_ID \
# #                                           trial_task:="cautious_driving" \
# #                                           trial_number:="12/16" case3:="1" \
# #                                           trial_condition:="MC" \
# #                                           display_viz:="true"


# ############ TEST ####################################

# WORLD="dynamic01"

# # 1
# roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
#                                           behavior:="assertive" \
#                                           trial_name:=$PARTICIPANT_ID \
#                                           trial_task:="assertive_driving" \
#                                           trial_number:="9/16" case4:="1" \
#                                           trial_condition:="HV-T" \
#                                           display_viz:="true"
# # 2
# roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
#                                           behavior:="assertive" \
#                                           trial_name:=$PARTICIPANT_ID \
#                                           trial_task:="cautious_driving" \
#                                           trial_number:="10/16" case1:="1" \
#                                           trial_condition:="HV-T" \
#                                           display_viz:="true"
# # 3
# roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
#                                           behavior:="cautious" \
#                                           trial_name:=$PARTICIPANT_ID \
#                                           trial_task:="cautious_driving" \
#                                           trial_number:="11/16" case1:="1" \
#                                           trial_condition:="HV-T" \
#                                           display_viz:="true"
# # 4
# roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
#                                           behavior:="cautious" \
#                                           trial_name:=$PARTICIPANT_ID \
#                                           trial_task:="cautious_driving" \
#                                           trial_number:="12/16" case4:="1" \
#                                           trial_condition:="HV-T" \
#                                           display_viz:="true"