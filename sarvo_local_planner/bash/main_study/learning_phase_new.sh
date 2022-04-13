#!/bin/sh

# Launch the robot
source /opt/ros/melodic/setup.bash
source ~/workspaces/research_ws/devel/setup.bash

echo "Starting the user study, kindly wait!"

PARTICIPANT_ID="NS01"

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

# 1
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:="none" \
                                          trial_name:=$PARTICIPANT_ID \
                                          task_objective:="none" \
                                          case1:="1" \
                                          trial_condition:="MC" \
                                          trial_number:="1/2" \
                                          trial_category:="#1" \
                                          display_viz:="true"
# 2
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:="none" \
                                          trial_name:=$PARTICIPANT_ID \
                                          task_objective:="none" \
                                          case2:="1" \
                                          trial_condition:="MC" \
                                          trial_number:="2/2" \
                                          trial_category:="#1" \
                                          display_viz:="true"

# *****************************************************************************************************
# 3
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:="none" \
                                          trial_name:=$PARTICIPANT_ID \
                                          task_objective:="none" \
                                          case3:="1" \
                                          trial_condition:="HV-T" \
                                          trial_number:="1/4" \
                                          trial_category:="#1" \
                                          display_viz:="true"
# 4
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:="none" \
                                          trial_name:=$PARTICIPANT_ID \
                                          task_objective:="none" \
                                          case1:="1" \
                                          trial_condition:="HV-T" \
                                          trial_number:="2/4" \
                                          trial_category:="#1" \
                                          display_viz:="true"


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
BEHAVIOR_1="goal_aligned"
BEHAVIOR_2="safety_aligned"

# 1
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:=$BEHAVIOR_1 \
                                          trial_name:=$PARTICIPANT_ID \
                                          task_objective:="none" \
                                          case1:="1" \
                                          trial_condition:="HV-T" \
                                          trial_number:="1/3" \
                                          trial_category:="#3" \
                                          display_viz:="true"
# 2
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:=$BEHAVIOR_2 \
                                          trial_name:=$PARTICIPANT_ID \
                                          task_objective:="none" \
                                          case3:="1" \
                                          trial_condition:="HV-T" \
                                          trial_number:="2/6" \
                                          trial_category:="#3" \
                                          display_viz:="true"
# 3
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:=$BEHAVIOR_1 \
                                          trial_name:=$PARTICIPANT_ID \
                                          task_objective:="none" \
                                          case2:="1" \
                                          trial_condition:="HV-T" \
                                          trial_number:="3/6" \
                                          trial_category:="#3" \
                                          display_viz:="true"

# *****************************************************************************************************

# WORLD="random_dynamic-01"
# WORLD="static-02"

# 4
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:=$BEHAVIOR_2 \
                                          trial_name:=$PARTICIPANT_ID \
                                          task_objective:="none" \
                                          case2:="1" \
                                          trial_condition:="HV-T" \
                                          trial_number:="4/6" \
                                          trial_category:="#3" \
                                          display_viz:="true"
# 5
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:=$BEHAVIOR_1 \
                                          trial_name:=$PARTICIPANT_ID \
                                          task_objective:="none" \
                                          case1:="1" \
                                          trial_condition:="HV-T" \
                                          trial_number:="5/6" \
                                          trial_category:="#3" \
                                          display_viz:="true"
# 6
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:=$BEHAVIOR_2 \
                                          trial_name:=$PARTICIPANT_ID \
                                          task_objective:="none" \
                                          case4:="1" \
                                          trial_condition:="HV-T" \
                                          trial_number:="6/6" \
                                          trial_category:="#3" \
                                          display_viz:="true"




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

WORLD="random_dynamic-02"
# WORLD="static-02"

# 1
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:="none" \
                                          trial_name:=$PARTICIPANT_ID \
                                          task_objective:="assertive_driving" \
                                          case1:="1" \
                                          trial_condition:="MC" \
                                          trial_number:="1/2" \
                                          trial_category:="#2" \
                                          display_viz:="true"
# 2
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:="none" \
                                          trial_name:=$PARTICIPANT_ID \
                                          task_objective:="cautious_driving" \
                                          case1:="1" \
                                          trial_condition:="MC" \
                                          trial_number:="2/2" \
                                          trial_category:="#2" \
                                          display_viz:="true"

# *****************************************************************************************************

WORLD="crossing_dynamic-01"

# 3
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:=$BEHAVIOR_1 \
                                          trial_name:=$PARTICIPANT_ID \
                                          task_objective:="cautious_driving" \
                                          case3:="1" \
                                          trial_condition:="HV-T" \
                                          trial_number:="1/4" \
                                          trial_category:="#2" \
                                          display_viz:="true"
# 4
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:=$BEHAVIOR_2 \
                                          trial_name:=$PARTICIPANT_ID \
                                          task_objective:="cautious_driving" \
                                          case4:="1" \
                                          trial_condition:="HV-T" \
                                          trial_number:="2/4" \
                                          trial_category:="#2" \
                                          display_viz:="true"
# 5
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:=$BEHAVIOR_2 \
                                          trial_name:=$PARTICIPANT_ID \
                                          task_objective:="assertive_driving" \
                                          case3:="1" \
                                          trial_condition:="HV-T" \
                                          trial_number:="3/4" \
                                          trial_category:="#2" \
                                          display_viz:="true"
# 6
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:=$BEHAVIOR_1 \
                                          trial_name:=$PARTICIPANT_ID \
                                          task_objective:="assertive_driving" \
                                          case4:="1" \
                                          trial_condition:="HV-T" \
                                          trial_number:="4/4" \
                                          trial_category:="#2" \
                                          display_viz:="true"


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

WORLD="crossing_dynamic-02"

TASK_1="assertive_driving"
TASK_2="cautious_driving"

# BEHAVIOR_1="safety_aligned"
# BEHAVIOR_2="goal_aligned"
BEHAVIOR_1="none"
BEHAVIOR_2="none"

# 1
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:="none" \
                                          trial_name:=$PARTICIPANT_ID \
                                          task_objective:="none" \
                                          case1:="1" \
                                          trial_condition:="MC" \
                                          display_viz:="true" \
                                          trial_number:="1/2" \
                                          trial_category:="#4" \
                                          distracted_mode:="true"

# *****************************************************************************************************
# 2
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
# 3
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



# waits for 10 seconds -----------------------------------------------------
echo "**********************************************************************"
echo " "
echo "                   Category #4 completed!                        "
echo " "
echo "**********************************************************************"
sleep 15s
# --------------------------------------------------------------------------


# *****************************************************************************************************
# CATEGORY #5:
# -----------
# World:            dynamic01
# Layout:           layout-01
# Trial condition:  MC
# Behaviors:        Mixed (cautious & assertive)
# Tasks:            Mixed (cautious & assertive)
# Distracted:       Yes
# *****************************************************************************************************

WORLD="crossing_dynamic-02"

TASK_1="assertive_driving"
TASK_2="cautious_driving"

# BEHAVIOR_1="safety_aligned"
# BEHAVIOR_2="goal_aligned"
BEHAVIOR_1="none"
BEHAVIOR_2="none"

# *****************************************************************************************************
# 1
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:=$BEHAVIOR_2 \
                                          trial_name:=$PARTICIPANT_ID \
                                          task_objective:=$TASK_2 \
                                          case1:="1" \
                                          trial_condition:="MC" \
                                          display_viz:="true" \
                                          trial_number:="1/4" \
                                          trial_category:="#5"
# 2
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:=$BEHAVIOR_2 \
                                          trial_name:=$PARTICIPANT_ID \
                                          task_objective:=$TASK_2 \
                                          case4:="1" \
                                          trial_condition:="MC" \
                                          display_viz:="true" \
                                          trial_number:="2/4" \
                                          trial_category:="#5" \
                                          distracted_mode:="true"

# *****************************************************************************************************

WORLD="random_dynamic-01"

# 3
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:=$BEHAVIOR_1 \
                                          trial_name:=$PARTICIPANT_ID \
                                          task_objective:=$TASK_1 \
                                          case2:="1" \
                                          trial_condition:="MC" \
                                          display_viz:="true" \
                                          trial_number:="3/4" \
                                          trial_category:="#5" 
# 4
roslaunch sarvo_local_planner test.launch scenario:=$WORLD \
                                          behavior:=$BEHAVIOR_1 \
                                          trial_name:=$PARTICIPANT_ID \
                                          task_objective:=$TASK_1 \
                                          case3:="1" \
                                          trial_condition:="MC" \
                                          display_viz:="true" \
                                          trial_number:="4/4" \
                                          trial_category:="#5" \
                                          distracted_mode:="true"



########################################################################################################

