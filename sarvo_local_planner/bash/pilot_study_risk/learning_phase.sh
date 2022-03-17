#!/bin/sh

# Launch the robot
source /opt/ros/melodic/setup.bash
source ~/workspaces/research_ws/devel/setup.bash

echo "Starting the user study, kindly wait!"

PARTICIPANT_ID="P03-RISK"

PHASE="learning"

########################################## learning phase #############################################

# *****************************************************************************************************
# CATEGORY #1:
# -----------
# World:            static01
# Layout:           layout-01
# Trial condition:  Manual control (MC) & HV-T
# Behaviors:        None defined
# Tasks:            Node defined
# *****************************************************************************************************

WORLD="static-01"

# # 1
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          trial_name:=$PARTICIPANT_ID \
                                          case1:="1" \
                                          trial_condition:="MC" \
                                          trial_number:="1/2" \
                                          trial_category:="#1" \
                                          display_viz:="true"
# 2
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          trial_name:=$PARTICIPANT_ID \
                                          case2:="1" \
                                          trial_condition:="MC" \
                                          trial_number:="2/2" \
                                          trial_category:="#1" \
                                          display_viz:="true"

# *****************************************************************************************************
# 3
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          trial_name:=$PARTICIPANT_ID \
                                          case3:="1" \
                                          trial_condition:="HV-T" \
                                          trial_number:="1/3" \
                                          trial_category:="#1" \
                                          display_viz:="true" \
                                          risk_enabled:= "true" \
                                          risk_condition:="NA(V)"
# 4
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          trial_name:=$PARTICIPANT_ID \
                                          case1:="1" \
                                          trial_condition:="HV-T" \
                                          trial_number:="2/3" \
                                          trial_category:="#1" \
                                          display_viz:="true" \
                                          risk_enabled:= "true" \
                                          risk_condition:="NA(H+V)"
# 5
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          trial_name:=$PARTICIPANT_ID \
                                          case2:="1" \
                                          trial_condition:="HV-T" \
                                          trial_number:="3/3" \
                                          trial_category:="#1" \
                                          display_viz:="true" \
                                          risk_enabled:= "true" \
                                          risk_condition:="A(H+V)"


# waits for 10 seconds -----------------------------------------------------
echo "**********************************************************************"
echo " "
echo "                   Category #1 completed!                        "
echo " "
echo "**********************************************************************"
sleep 15s
# --------------------------------------------------------------------------



# *****************************************************************************************************
# CATEGORY #2:
# -----------
# World:            static02
# Layout:           layout-01
# Trial condition:  Assisted case (HV-T)
# Behaviors:        Mixed (cautious & assertive)
# Tasks:            Node defined
# *****************************************************************************************************

# WORLD="crossing_dynamic-01"
WORLD="static-02"
RISK_CONDITION_1="A(H+V)"
RISK_CONDITION_2="NA(H+V)"
RISK_CONDITION_3="NA(V)"

# 1
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          trial_name:=$PARTICIPANT_ID \
                                          case1:="1" \
                                          trial_condition:="HV-T" \
                                          trial_number:="1/6" \
                                          trial_category:="#3" \
                                          display_viz:="true" \
                                          risk_enabled:= "true" \
                                          risk_condition:=$RISK_CONDITION_1
# 2
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          trial_name:=$PARTICIPANT_ID \
                                          case3:="1" \
                                          trial_condition:="HV-T" \
                                          trial_number:="2/6" \
                                          trial_category:="#3" \
                                          display_viz:="true" \
                                          risk_enabled:= "true" \
                                          risk_condition:=$RISK_CONDITION_2
# 3
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          trial_name:=$PARTICIPANT_ID \
                                          case2:="1" \
                                          trial_condition:="HV-T" \
                                          trial_number:="3/6" \
                                          trial_category:="#3" \
                                          display_viz:="true" \
                                          risk_enabled:= "true" \
                                          risk_condition:=$RISK_CONDITION_3

# *****************************************************************************************************

# WORLD="random_dynamic-01"
# WORLD="static-02"

# 4
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          trial_name:=$PARTICIPANT_ID \
                                          case2:="1" \
                                          trial_condition:="HV-T" \
                                          trial_number:="4/6" \
                                          trial_category:="#3" \
                                          display_viz:="true" \
                                          risk_enabled:= "true" \
                                          risk_condition:=$RISK_CONDITION_2
# 5
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          trial_name:=$PARTICIPANT_ID \
                                          case1:="1" \
                                          trial_condition:="HV-T" \
                                          trial_number:="5/6" \
                                          trial_category:="#3" \
                                          display_viz:="true" \
                                          risk_enabled:= "true" \
                                          risk_condition:=$RISK_CONDITION_3
# 6
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          trial_name:=$PARTICIPANT_ID \
                                          case4:="1" \
                                          trial_condition:="HV-T" \
                                          trial_number:="6/6" \
                                          trial_category:="#3" \
                                          display_viz:="true" \
                                          risk_enabled:= "true" \
                                          risk_condition:=$RISK_CONDITION_1




# waits for 10 seconds -----------------------------------------------------
echo "**********************************************************************"
echo " "
echo "                   Category #2 completed!                        "
echo " "
echo "**********************************************************************"
sleep 15s
# --------------------------------------------------------------------------



# *****************************************************************************************************
# CATEGORY #3:
# -----------
# World:            dynamic01
# Layout:           layout-01
# Trial condition:  Manual control (MC) & HV-T
# Behaviors:        None defined
# Tasks:            Node defined
# *****************************************************************************************************

WORLD="crossing_dynamic-01"
# WORLD="static-02"

# 1
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          trial_name:=$PARTICIPANT_ID \
                                          case3:="1" \
                                          trial_condition:="HV-T" \
                                          trial_number:="1/6" \
                                          trial_category:="#3" \
                                          display_viz:="true" \
                                          risk_enabled:= "true" \
                                          risk_condition:=$RISK_CONDITION_2
# 2
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          trial_name:=$PARTICIPANT_ID \
                                          case4:="1" \
                                          trial_condition:="HV-T" \
                                          trial_number:="2/6" \
                                          trial_category:="#3" \
                                          display_viz:="true" \
                                          risk_enabled:= "true" \
                                          risk_condition:=$RISK_CONDITION_1
# 3
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          trial_name:=$PARTICIPANT_ID \
                                          case2:="1" \
                                          trial_condition:="HV-T" \
                                          trial_number:="3/6" \
                                          trial_category:="#3" \
                                          display_viz:="true" \
                                          risk_enabled:= "true" \
                                          risk_condition:=$RISK_CONDITION_3

# *****************************************************************************************************

# 4
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          trial_name:=$PARTICIPANT_ID \
                                          case1:="1" \
                                          trial_condition:="HV-T" \
                                          trial_number:="4/6" \
                                          trial_category:="#3" \
                                          display_viz:="true" \
                                          risk_enabled:= "true" \
                                          risk_condition:=$RISK_CONDITION_2
# 5
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          trial_name:=$PARTICIPANT_ID \
                                          case3:="1" \
                                          trial_condition:="HV-T" \
                                          trial_number:="5/6" \
                                          trial_category:="#3" \
                                          display_viz:="true" \
                                          risk_enabled:= "true" \
                                          risk_condition:=$RISK_CONDITION_3
# 6
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          trial_name:=$PARTICIPANT_ID \
                                          case5:="1" \
                                          trial_condition:="HV-T" \
                                          trial_number:="6/6" \
                                          trial_category:="#3" \
                                          display_viz:="true" \
                                          risk_enabled:= "true" \
                                          risk_condition:=$RISK_CONDITION_1


# waits for 10 seconds -----------------------------------------------------
echo "**********************************************************************"
echo " "
echo "                   Category #3 completed!                        "
echo " "
echo "**********************************************************************"
sleep 15s
# --------------------------------------------------------------------------

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
RISK_CONDITION_1="NA(H+V)"
RISK_CONDITION_2="A(H+V)"
RISK_CONDITION_3="NA(V)"

# 1
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          trial_name:=$PARTICIPANT_ID \
                                          case1:="1" \
                                          trial_condition:="MC" \
                                          display_viz:="true" \
                                          trial_number:="1/2" \
                                          trial_category:="#4" \
                                          distracted_mode:="true"
# 2
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          trial_name:=$PARTICIPANT_ID \
                                          case5:="1" \
                                          trial_condition:="MC" \
                                          display_viz:="true" \
                                          trial_number:="2/2" \
                                          trial_category:="#4" \
                                          distracted_mode:="true"

# *****************************************************************************************************
# 3
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          trial_name:=$PARTICIPANT_ID \
                                          case4:="1" \
                                          trial_condition:="HV-T" \
                                          display_viz:="true" \
                                          trial_number:="1/6" \
                                          trial_category:="#4" \
                                          distracted_mode:="true" \
                                          risk_enabled:= "true" \
                                          risk_condition:=$RISK_CONDITION_1
# 4
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          trial_name:=$PARTICIPANT_ID \
                                          case4:="1" \
                                          trial_condition:="HV-T" \
                                          display_viz:="true" \
                                          trial_number:="2/6" \
                                          trial_category:="#4" \
                                          distracted_mode:="true" \
                                          risk_enabled:= "true" \
                                          risk_condition:=$RISK_CONDITION_1

# *****************************************************************************************************

WORLD="random_dynamic-01"

# 5
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          trial_name:=$PARTICIPANT_ID \
                                          case2:="1" \
                                          trial_condition:="HV-T" \
                                          display_viz:="true" \
                                          trial_number:="3/6" \
                                          trial_category:="#4" \
                                          distracted_mode:="true" \
                                          risk_enabled:= "true" \
                                          risk_condition:=$RISK_CONDITION_2
# 6
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          trial_name:=$PARTICIPANT_ID \
                                          case2:="1" \
                                          trial_condition:="HV-T" \
                                          display_viz:="true" \
                                          trial_number:="4/6" \
                                          trial_category:="#4" \
                                          distracted_mode:="true" \
                                          risk_enabled:= "true" \
                                          risk_condition:=$RISK_CONDITION_2


# *****************************************************************************************************

WORLD="random_dynamic-01"

# 5
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          trial_name:=$PARTICIPANT_ID \
                                          case3:="1" \
                                          trial_condition:="HV-T" \
                                          display_viz:="true" \
                                          trial_number:="5/6" \
                                          trial_category:="#4" \
                                          distracted_mode:="true" \
                                          risk_enabled:= "true" \
                                          risk_condition:=$RISK_CONDITION_3
# 6
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          trial_name:=$PARTICIPANT_ID \
                                          case3:="1" \
                                          trial_condition:="HV-T" \
                                          display_viz:="true" \
                                          trial_number:="6/6" \
                                          trial_category:="#4" \
                                          distracted_mode:="true" \
                                          risk_enabled:= "true" \
                                          risk_condition:=$RISK_CONDITION_3


########################################################################################################


