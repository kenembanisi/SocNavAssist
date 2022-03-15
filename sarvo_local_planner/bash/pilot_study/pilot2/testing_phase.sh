#!/bin/sh

# Launch the robot
source /opt/ros/melodic/setup.bash
source ~/workspaces/research_ws/devel/setup.bash

echo "Starting the user study, kindly wait!"

PARTICIPANT_ID="S01"

PHASE="testing"

########################################## testing phase #############################################

# block 1 ---------------------------------------------------------------------------------------------
TASK="cautious_driving"
BEHAVIOR="cautious"

# 1
roslaunch sarvo_local_planner trial.launch scenario:="random-01" \
                                          behavior:=$BEHAVIOR \
                                          trial_name:=$PARTICIPANT_ID \
                                          trial_task:=$TASK \
                                          trial_number:="1/18" case4:="1"
# 2
roslaunch sarvo_local_planner trial.launch scenario:="random-02" \
                                          behavior:=$BEHAVIOR \
                                          trial_name:=$PARTICIPANT_ID \
                                          trial_task:=$TASK \
                                          trial_number:="2/18" case4:="1"

# waits for 30 seconds -----------------------------------------------------
sleep 30s
# --------------------------------------------------------------------------

BEHAVIOR="neutral"

# 3
roslaunch sarvo_local_planner trial.launch scenario:="random-03" \
                                          behavior:=$BEHAVIOR \
                                          trial_name:=$PARTICIPANT_ID \
                                          trial_task:=$TASK \
                                          trial_number:="3/18" case4:="1"
4
roslaunch sarvo_local_planner trial.launch scenario:="random-02" \
                                          behavior:=$BEHAVIOR \
                                          trial_name:=$PARTICIPANT_ID \
                                          trial_task:=$TASK \
                                          trial_number:="4/18" case4:="1"

# waits for 30 seconds -----------------------------------------------------
sleep 30s
# --------------------------------------------------------------------------

BEHAVIOR="assertive"

# 5
roslaunch sarvo_local_planner trial.launch scenario:="random-03" \
                                          behavior:=$BEHAVIOR \
                                          trial_name:=$PARTICIPANT_ID \
                                          trial_task:=$TASK \
                                          trial_number:="5/18" case4:="1"
# 6
roslaunch sarvo_local_planner trial.launch scenario:="random-01" \
                                          behavior:=$BEHAVIOR \
                                          trial_name:=$PARTICIPANT_ID \
                                          trial_task:=$TASK \
                                          trial_number:="6/18" case4:="1"


# -----------------------------------------------------------------------------------------------------

# waits for 30 seconds -----------------------------------------------------
sleep 30s
# --------------------------------------------------------------------------


# block 2 ---------------------------------------------------------------------------------------------
TASK="assertive_driving"
BEHAVIOR="cautious"

# 1
roslaunch sarvo_local_planner trial.launch scenario:="random-01" \
                                          behavior:=$BEHAVIOR \
                                          trial_name:=$PARTICIPANT_ID \
                                          trial_task:=$TASK \
                                          trial_number:="1/18" case4:="1"
# 2
roslaunch sarvo_local_planner trial.launch scenario:="random-02" \
                                          behavior:=$BEHAVIOR \
                                          trial_name:=$PARTICIPANT_ID \
                                          trial_task:=$TASK \
                                          trial_number:="2/18" case4:="1"

# waits for 30 seconds -----------------------------------------------------
sleep 30s
# --------------------------------------------------------------------------

BEHAVIOR="neutral"

# 3
roslaunch sarvo_local_planner trial.launch scenario:="random-03" \
                                          behavior:=$BEHAVIOR \
                                          trial_name:=$PARTICIPANT_ID \
                                          trial_task:=$TASK \
                                          trial_number:="3/18" case4:="1"
4
roslaunch sarvo_local_planner trial.launch scenario:="random-02" \
                                          behavior:=$BEHAVIOR \
                                          trial_name:=$PARTICIPANT_ID \
                                          trial_task:=$TASK \
                                          trial_number:="4/18" case4:="1"

# waits for 30 seconds -----------------------------------------------------
sleep 30s
# --------------------------------------------------------------------------

BEHAVIOR="assertive"

# 5
roslaunch sarvo_local_planner trial.launch scenario:="random-03" \
                                          behavior:=$BEHAVIOR \
                                          trial_name:=$PARTICIPANT_ID \
                                          trial_task:=$TASK \
                                          trial_number:="5/18" case4:="1"
# 6
roslaunch sarvo_local_planner trial.launch scenario:="random-01" \
                                          behavior:=$BEHAVIOR \
                                          trial_name:=$PARTICIPANT_ID \
                                          trial_task:=$TASK \
                                          trial_number:="6/18" case4:="1"


# -----------------------------------------------------------------------------------------------------

# waits for 30 seconds -----------------------------------------------------
sleep 30s
# --------------------------------------------------------------------------
