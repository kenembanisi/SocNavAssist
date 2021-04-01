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


    ########################### Define agent and properties #########################
    agent_id = 'trina2'
    agent_properties = {'radius': 1.0}
    agent = AgentClass(agent_id, agent_properties)
    

    ######################### Define pedestrians as obstacles #######################
    num_pedestrians = int(rospy.get_param("num_pedestrians")) # convert to int
    pedestrian_id = []
    obstacle_list = {}
    for i in range(num_pedestrians):
        # obstacle_list[i] = ObstacleClass(pedestrian_id[i], pedestrian_properties[i])
        pedestrian_id.append("actor" + str(i+1)) # actor numbering starts from 1
        obstacle_list[i] = ObstacleClass(pedestrian_id[i])


    ################ Initiate RVO controller for agent & obstacle set ###############
    D = 0.2 # radius extension for differential drive condition
    tau = 4.0 # planning horizon
    rvo_agent = RvoControl(agent, obstacle_list, D=D, tau=tau)


    ################ Initialize data logger (for active objects only) ###############
    model_ids = ['trina2'] + pedestrian_id
    logger = DataLogger(model_ids, scenario, obstacle_list)


    ############################ Set agent goal location ############################
    goal = [-6.5, 8.2]
    
    ################### Get control mode from ROS parameter server ##################
    control_mode = rospy.get_param('control_mode')
    AUTO = False
    if control_mode == 'auto':
        AUTO = True

    ################################### Set timer ###################################
    t_start = time.time()
    time_to_goal = 0

    ################################### Main loop ###################################
    while not rospy.is_shutdown():
        
        # timer to check computing time--------------------------------------------
        # t_start = time.time()
        # -------------------------------------------------------------------------

        # Update simulation -------------------------------------------------------
        alpha = 1 # collision avoidance responsibility, 1 means the agent 
                  # takes full responsibility
        v_opt, v_suitable, v_admissible, heading_delta = rvo_agent.compute_V_opt(goal, alpha=alpha)
        # -------------------------------------------------------------------------

        # get desired/goal agent velocity -----------------------------------------
        v_goal = rvo_agent.get_goal_velocity()

        # check goal reached ------------------------------------------------------
        if rvo_agent.reached:
            time_to_goal = (time.time() - t_start)
        
        # store states ------------------------------------------------------------
        logger.store_data(v_opt, v_suitable, v_admissible, v_goal, time_to_goal)

        # update agent's state ----------------------------------------------------
        if AUTO:
            agent.update_controls(v_opt[1], v_suitable) # only takes v_opt[1]: the
                                                    # kinematically feasible velocities

        # publish heading_delta ---------------------------------------------------
        agent.publish_heading_delta(heading_delta) # this is for shared control in manual
                                                   # control mode

        # Move the active obstacles -----------------------------------------------
        for i in range(num_pedestrians):
            obstacle_list[i].update_states()
        #     obstacle_list[i].move()
            
        # rospy.loginfo("The computed optimal control is: %s", str([round(v_opt[1][0],2), round(v_opt[1][1],2)]))

        # Set stop time -----------------------------------------------------------
        t_stop = time.time()
        dt = t_stop - t_start


        # Save logged data at intervals -------------------------------------------
        save_interval = 25 # seconds
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
        


