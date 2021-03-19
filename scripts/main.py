#!/usr/bin/env python

"""
main_control.py

[...]


"""


#####################################################################################
# Import packages
#####################################################################################

import sys
import rospy
import numpy as np
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import time

from obstacles import ObstacleClass
from agent import AgentClass
from rvo_control import RvoControl
from data_logger import DataLogger

#####################################################################################
# run function
#####################################################################################
def run(args):

    ############################# Instantiate ROS node ##############################
    rospy.init_node('rvo_ros_node')


    ################### Define scenario and pedestrian properties ###################
    if len(args) > 1: # this means something is being passed
        scenario = args[1]
    else:
        scenario = "crossing" # crossing is the default

    if scenario == "crossing":
        # for crossing scenario
        pedestrian_properties = [{'radius': 0.4, 'pref_velocity': [0.8, -0.0]},
                                {'radius': 0.4, 'pref_velocity': [-1.0, -0.0]},
                                {'radius': 0.4, 'pref_velocity': [0.6, -0.0]},
                                {'radius': 0.4, 'pref_velocity': [-0.7, -0.0]}]
    if scenario == "approach":
        # for approach scenario
        pedestrian_properties = [{'radius': 0.4, 'pref_velocity': [0.0, -0.80]},
                                {'radius': 0.4, 'pref_velocity': [0.0, -0.80]},
                                {'radius': 0.4, 'pref_velocity': [0.0, -0.80]},
                                {'radius': 0.4, 'pref_velocity': [0.0, -1.0]}]
    pedestrian_id = ['dynamic_obstacle_1', 'dynamic_obstacle_2', 
                     'dynamic_obstacle_3', 'dynamic_obstacle_4']
    num_pedestrians = len(pedestrian_id)
    
    #################################################################################


    ########################### Define agent and properties #########################
    agent_id = 'trina2'
    agent_properties = {'radius': 1.0}
    agent = AgentClass(agent_id, agent_properties)
    

    ################ Initialize data logger (for active objects only) ###############
    model_ids = ['trina2'] + pedestrian_id
    logger = DataLogger(model_ids, scenario)


    ######################### Define pedestrians as obstacles #######################
    obstacle_list = {}
    for i in range(num_pedestrians):
        obstacle_list[i] = ObstacleClass(pedestrian_id[i], pedestrian_properties[i])

    
    ################ Initiate RVO controller for agent & obstacle set ###############
    D = 0.2 # radius extension for differential drive condition
    tau = 3.0 # planning horizon
    rvo_agent = RvoControl(agent, obstacle_list, D=D, tau=tau)


    ############################ Set agent goal location ############################
    goal = [-6.5, 8.2]
    
    ################### Get control mode from ROS parameter server ##################
    control_mode = rospy.get_param('control_mode')
    AUTO = False
    if control_mode == 'auto':
        AUTO = True

    ################################### Set timer ###################################
    t_start = time.time()

    ################################### Main loop ###################################
    while not rospy.is_shutdown():
        
        # timer to check computing time--------------------------------------------
        # t_start = time.time()
        # -------------------------------------------------------------------------

        # Update simulation -------------------------------------------------------
        alpha = 1 # collision avoidance responsibility, 1 means the agent 
                  # takes full responsibility
        v_opt, v_suitable, v_admissible = rvo_agent.compute_V_opt(goal, alpha=alpha)
        # -------------------------------------------------------------------------

        # get desired/goal agent velocity -----------------------------------------
        v_goal = rvo_agent.get_goal_velocity()

        # store states ------------------------------------------------------------
        logger.store_data(v_opt, v_suitable, v_admissible, v_goal)

        # update agent's state ----------------------------------------------------
        if AUTO:
            agent.update_controls(v_opt[1], v_suitable) # only takes v_opt[1]: the
                                                    # kinematically feasible velocities

        # Move the active obstacles -----------------------------------------------
        for i in range(num_pedestrians):
            obstacle_list[i].move()
            
        # rospy.loginfo("The computed optimal control is: %s", str([round(v_opt[1][0],2), round(v_opt[1][1],2)]))

        # Set stop time -----------------------------------------------------------
        t_stop = time.time()
        dt = t_stop - t_start

        # Save logged data at intervals -------------------------------------------
        save_interval = 20 # seconds
        if round(dt) > 1 and round(dt) % save_interval == 0:
            logger.save_data()
            rospy.loginfo("<<<<< Trial Saving Complete! >>>>>")

#####################################################################################
# main
#####################################################################################

if __name__ == "__main__":

    # get argument passed to node
    args = rospy.myargv(argv=sys.argv)

    try:
       # run the node
       run(args)

    finally:
        pass
        


