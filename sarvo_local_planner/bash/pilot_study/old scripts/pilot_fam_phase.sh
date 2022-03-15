#!/bin/sh

# Launch the robot
source /opt/ros/melodic/setup.bash
source ~/workspaces/research_ws/devel/setup.bash

echo "Starting the user study, kindly wait!"

PARTICIPANT_ID="P03"

PHASE="learning"

########################################## learning phase #############################################

# *****************************************************************************************************
# PRACTICE A:
# -----------
# World:            Hall w/o pedestrians -> static00
# Trial condition:  Manual control (MC)
# Behaviors:        None defined
# Tasks:            Node defined
# *****************************************************************************************************

WORLD="static00"

# 1
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:="assertive" \
                                          trial_name:=$PARTICIPANT_ID \
                                          trial_task:="assertive_driving" \
                                          case1:="1" \
                                          trial_condition:="MC" \
                                          display_viz:="true"
# 2
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:="assertive" \
                                          trial_name:=$PARTICIPANT_ID \
                                          trial_task:="cautious_driving" \
                                          case2:="1" \
                                          trial_condition:="MC" \
                                          display_viz:="true"
# 3
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:="cautious" \
                                          trial_name:=$PARTICIPANT_ID \
                                          trial_task:="cautious_driving" \
                                          case3:="1" \
                                          trial_condition:="MC" \
                                          display_viz:="true"
# 4
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:="cautious" \
                                          trial_name:=$PARTICIPANT_ID \
                                          trial_task:="cautious_driving" \
                                          case4:="1" \
                                          trial_condition:="MC" \
                                          display_viz:="true"


# *****************************************************************************************************
# PRACTICE B:
# -----------
# World:            Hall w/ 1 pedestrians -> static01
# Trial condition:  Manual control (MC)
# Behaviors:        None defined
# Tasks:            Node defined
# *****************************************************************************************************

WORLD="static01"

# # 1
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:="assertive" \
                                          trial_name:=$PARTICIPANT_ID \
                                          trial_task:="assertive_driving" \
                                          case1:="1" \
                                          trial_condition:="MC" \
                                          display_viz:="true"
# 2
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:="assertive" \
                                          trial_name:=$PARTICIPANT_ID \
                                          trial_task:="cautious_driving" \
                                          case2:="1" \
                                          trial_condition:="MC" \
                                          display_viz:="true"
# 3
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:="cautious" \
                                          trial_name:=$PARTICIPANT_ID \
                                          trial_task:="cautious_driving" \
                                          case3:="1" \
                                          trial_condition:="MC" \
                                          display_viz:="true"
# 4
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:="cautious" \
                                          trial_name:=$PARTICIPANT_ID \
                                          trial_task:="cautious_driving" \
                                          case4:="1" \
                                          trial_condition:="MC" \
                                          display_viz:="true"


# *****************************************************************************************************
# PRACTICE C:
# -----------
# World:            Hall w/ 1 pedestrians -> static01
# Trial condition:  Assisted case (HV-T)
# Behaviors:        Mixed (cautious & assertive)
# Tasks:            Node defined
# *****************************************************************************************************

WORLD="static01"

# # 1
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:="cautious" \
                                          trial_name:=$PARTICIPANT_ID \
                                          trial_task:="assertive_driving" \
                                          case1:="1" \
                                          trial_condition:="HV-T" \
                                          display_viz:="true"
# 2
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:="cautious" \
                                          trial_name:=$PARTICIPANT_ID \
                                          trial_task:="cautious_driving" \
                                          case3:="1" \
                                          trial_condition:="HV-T" \
                                          display_viz:="true"
# 3
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:="assertive" \
                                          trial_name:=$PARTICIPANT_ID \
                                          trial_task:="cautious_driving" \
                                          case3:="1" \
                                          trial_condition:="HV-T" \
                                          display_viz:="true"
# 4
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:="assertive" \
                                          trial_name:=$PARTICIPANT_ID \
                                          trial_task:="cautious_driving" \
                                          case1:="1" \
                                          trial_condition:="HV-T" \
                                          display_viz:="true"


# *****************************************************************************************************
# PRACTICE D:
# -----------
# World:            Hall w/ many static pedestrians -> static02
# Trial condition:  Assisted case (HV-T)
# Behaviors:        Mixed (cautious & assertive)
# Tasks:            Mixed (cautious & assertive)
# *****************************************************************************************************

WORLD="static02"

# 1
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:="cautious" \
                                          trial_name:=$PARTICIPANT_ID \
                                          trial_task:="cautious_driving" \
                                          case2:="1" \
                                          trial_condition:="HV-T" \
                                          display_viz:="true" \
                                          trial_mode:="testing"
# 2
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:="assertive" \
                                          trial_name:=$PARTICIPANT_ID \
                                          trial_task:="cautious_driving" \
                                          case4:="1" \
                                          trial_condition:="HV-T" \
                                          display_viz:="true" \
                                          trial_mode:="testing"
3
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:="assertive" \
                                          trial_name:=$PARTICIPANT_ID \
                                          trial_task:="assertive_driving" \
                                          case4:="1" \
                                          trial_condition:="HV-T" \
                                          display_viz:="true" \
                                          trial_mode:="testing"
# 4
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:="cautious" \
                                          trial_name:=$PARTICIPANT_ID \
                                          trial_task:="assertive_driving" \
                                          case2:="1" \
                                          trial_condition:="HV-T" \
                                          display_viz:="true" \
                                          trial_mode:="testing"




# *****************************************************************************************************
# PRACTICE E:
# -----------
# World:            Hall w/ few moving pedestrians -> dynamic01
# Trial condition:  Assisted case (HV-T)
# Behaviors:        Mixed (cautious & assertive)
# Tasks:            Mixed (cautious & assertive)
# *****************************************************************************************************

WORLD="dynamic01"

# 3
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:="assertive" \
                                          trial_name:=$PARTICIPANT_ID \
                                          trial_task:="assertive_driving" \
                                          case2:="1" \
                                          trial_condition:="HV-T" \
                                          display_viz:="true" \
                                          trial_mode:="testing"
# 1
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:="assertive" \
                                          trial_name:=$PARTICIPANT_ID \
                                          trial_task:="assertive_driving" \
                                          case3:="1" \
                                          trial_condition:="HV-T" \
                                          display_viz:="true" \
                                          trial_mode:="testing"
# 2
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:="cautious" \
                                          trial_name:=$PARTICIPANT_ID \
                                          trial_task:="cautious_driving" \
                                          case1:="1" \
                                          trial_condition:="HV-T" \
                                          display_viz:="true" \
                                          trial_mode:="testing"

# 4
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:="cautious" \
                                          trial_name:=$PARTICIPANT_ID \
                                          trial_task:="cautious_driving" \
                                          case4:="1" \
                                          trial_condition:="HV-T" \
                                          display_viz:="true" \
                                          trial_mode:="testing"
# 5
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:="cautious" \
                                          trial_name:=$PARTICIPANT_ID \
                                          trial_task:="cautious_driving" \
                                          case1:="1" \
                                          trial_condition:="HV-T" \
                                          display_viz:="true" \
                                          trial_mode:="testing"
# 6
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:="cautious" \
                                          trial_name:=$PARTICIPANT_ID \
                                          trial_task:="assertive_driving" \
                                          case3:="1" \
                                          trial_condition:="HV-T" \
                                          display_viz:="true" \
                                          trial_mode:="testing"


########################################################################################################

