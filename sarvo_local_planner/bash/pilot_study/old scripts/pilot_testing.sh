#!/bin/sh

# Launch the robot
source /opt/ros/melodic/setup.bash
source ~/workspaces/research_ws/devel/setup.bash

echo "Starting the user study, kindly wait!"

PARTICIPANT_ID="P01"

PHASE="learning"

########################################## learning phase #############################################

# static00 ---------------------------------------------------------------------------------------------

# WORLD="static00"

# # 1
# roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
#                                           behavior:="assertive" \
#                                           trial_name:=$PARTICIPANT_ID \
#                                           trial_task:="assertive_driving" \
#                                           trial_number:="0/16" case1:="1" \
#                                           trial_condition:="MC" \
#                                           display_viz:="true"
# # 2
# roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
#                                           behavior:="assertive" \
#                                           trial_name:=$PARTICIPANT_ID \
#                                           trial_task:="cautious_driving" \
#                                           trial_number:="0/16" case2:="1" \
#                                           trial_condition:="MC" \
#                                           display_viz:="true"
# # 3
# roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
#                                           behavior:="cautious" \
#                                           trial_name:=$PARTICIPANT_ID \
#                                           trial_task:="cautious_driving" \
#                                           trial_number:="0/16" case3:="1" \
#                                           trial_condition:="MC" \
#                                           display_viz:="true"
# # 4
# roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
#                                           behavior:="cautious" \
#                                           trial_name:=$PARTICIPANT_ID \
#                                           trial_task:="cautious_driving" \
#                                           trial_number:="0/16" case4:="1" \
#                                           trial_condition:="MC" \
#                                           display_viz:="true"


# static01 ---------------------------------------------------------------------------------------------

# WORLD="static01"

# # # 1
# roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
#                                           behavior:="assertive" \
#                                           trial_name:=$PARTICIPANT_ID \
#                                           trial_task:="assertive_driving" \
#                                           trial_number:="1/16" case1:="1" \
#                                           trial_condition:="HV-T" \
#                                           display_viz:="true"
# # 2
# roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
#                                           behavior:="assertive" \
#                                           trial_name:=$PARTICIPANT_ID \
#                                           trial_task:="cautious_driving" \
#                                           trial_number:="2/16" case3:="1" \
#                                           trial_condition:="HV-T" \
#                                           display_viz:="true"
# # 3
# roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
#                                           behavior:="cautious" \
#                                           trial_name:=$PARTICIPANT_ID \
#                                           trial_task:="cautious_driving" \
#                                           trial_number:="3/16" case3:="1" \
#                                           trial_condition:="HV-T" \
#                                           display_viz:="true"
# # 4
# roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
#                                           behavior:="cautious" \
#                                           trial_name:=$PARTICIPANT_ID \
#                                           trial_task:="cautious_driving" \
#                                           trial_number:="4/16" case1:="1" \
#                                           trial_condition:="HV-T" \
#                                           display_viz:="true"


# static02 ---------------------------------------------------------------------------------------------

# WORLD="static02"

# # 1
# roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
#                                           behavior:="cautious" \
#                                           trial_name:=$PARTICIPANT_ID \
#                                           trial_task:="assertive_driving" \
#                                           trial_number:="5/16" case2:="1" \
#                                           trial_condition:="HV-T" \
#                                           display_viz:="true"
# # 2
# roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
#                                           behavior:="cautious" \
#                                           trial_name:=$PARTICIPANT_ID \
#                                           trial_task:="cautious_driving" \
#                                           trial_number:="6/16" case4:="1" \
#                                           trial_condition:="HV-T" \
#                                           display_viz:="true"
# # 3
# roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
#                                           behavior:="assertive" \
#                                           trial_name:=$PARTICIPANT_ID \
#                                           trial_task:="cautious_driving" \
#                                           trial_number:="7/16" case4:="1" \
#                                           trial_condition:="HV-T" \
#                                           display_viz:="true"
# # 4
# roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
#                                           behavior:="assertive" \
#                                           trial_name:=$PARTICIPANT_ID \
#                                           trial_task:="cautious_driving" \
#                                           trial_number:="8/16" case2:="1" \
#                                           trial_condition:="HV-T" \
#                                           display_viz:="true"

# WORLD="static02"

# # 1
# roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
#                                           behavior:="cautious" \
#                                           trial_name:=$PARTICIPANT_ID \
#                                           trial_task:="assertive_driving" \
#                                           trial_number:="5/16" case2:="1" \
#                                           trial_condition:="MC" \
#                                           display_viz:="true"
# # 2
# roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
#                                           behavior:="cautious" \
#                                           trial_name:=$PARTICIPANT_ID \
#                                           trial_task:="cautious_driving" \
#                                           trial_number:="6/16" case4:="1" \
#                                           trial_condition:="MC" \
#                                           display_viz:="true"
# # 3
# roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
#                                           behavior:="assertive" \
#                                           trial_name:=$PARTICIPANT_ID \
#                                           trial_task:="cautious_driving" \
#                                           trial_number:="7/16" case4:="1" \
#                                           trial_condition:="MC" \
#                                           display_viz:="true"
# # 4
# roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
#                                           behavior:="assertive" \
#                                           trial_name:=$PARTICIPANT_ID \
#                                           trial_task:="cautious_driving" \
#                                           trial_number:="8/16" case2:="1" \
#                                           trial_condition:="MC" \
#                                           display_viz:="true"



# # 4
# roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
#                                           behavior:="assertive" \
#                                           trial_name:=$PARTICIPANT_ID \
#                                           trial_task:="cautious_driving" \
#                                           trial_number:="8/16" case2:="1" \
#                                           trial_condition:="HV-T" \
#                                           display_viz:="true" \ 
#                                           use_planner:="true"


# dynamic01 ---------------------------------------------------------------------------------------------

WORLD="dynamic01"

# 3
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:="assertive" \
                                          trial_name:=$PARTICIPANT_ID \
                                          trial_task:="cautious_driving" \
                                          trial_number:="11/16" case1:="1" \
                                          trial_condition:="HV-T" \
                                          display_viz:="true"
# 1
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:="assertive" \
                                          trial_name:=$PARTICIPANT_ID \
                                          trial_task:="assertive_driving" \
                                          trial_number:="9/16" case4:="1" \
                                          trial_condition:="HV-T" \
                                          display_viz:="true"
# 2
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:="cautious" \
                                          trial_name:=$PARTICIPANT_ID \
                                          trial_task:="cautious_driving" \
                                          trial_number:="10/16" case1:="1" \
                                          trial_condition:="HV-T" \
                                          display_viz:="true"
# 4
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:="cautious" \
                                          trial_name:=$PARTICIPANT_ID \
                                          trial_task:="cautious_driving" \
                                          trial_number:="12/16" case4:="1" \
                                          trial_condition:="HV-T" \
                                          display_viz:="true"

# WORLD="dynamic01"

# # 1
# roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
#                                           behavior:="cautious" \
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
#                                           behavior:="assertive" \
#                                           trial_name:=$PARTICIPANT_ID \
#                                           trial_task:="cautious_driving" \
#                                           trial_number:="11/16" case1:="1" \
#                                           trial_condition:="HV-T" \
#                                           display_viz:="true"
# # 4
# roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
#                                           behavior:="assertive" \
#                                           trial_name:=$PARTICIPANT_ID \
#                                           trial_task:="cautious_driving" \
#                                           trial_number:="12/16" case4:="1" \
#                                           trial_condition:="HV-T" \
#                                           display_viz:="true"


# dynamic02 ---------------------------------------------------------------------------------------------

# WORLD="dynamic02"

# # 1
# roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
#                                           behavior:="assertive" \
#                                           trial_name:=$PARTICIPANT_ID \
#                                           trial_task:="assertive_driving" \
#                                           trial_number:="13/16" case1:="1" \
#                                           trial_condition:="HV-T" \
#                                           display_viz:="true"
# # 2
# roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
#                                           behavior:="assertive" \
#                                           trial_name:=$PARTICIPANT_ID \
#                                           trial_task:="cautious_driving" \
#                                           trial_number:="14/16" case3:="1" \
#                                           trial_condition:="HV-T" \
#                                           display_viz:="true"
# # 3
# roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
#                                           behavior:="cautious" \
#                                           trial_name:=$PARTICIPANT_ID \
#                                           trial_task:="cautious_driving" \
#                                           trial_number:="15/16" case3:="1" \
#                                           trial_condition:="HV-T" \
#                                           display_viz:="true"
# # 4
# roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
#                                           behavior:="cautious" \
#                                           trial_name:=$PARTICIPANT_ID \
#                                           trial_task:="cautious_driving" \
#                                           trial_number:="16/16" case1:="1" \
#                                           trial_condition:="HV-T" \
#                                           display_viz:="true"