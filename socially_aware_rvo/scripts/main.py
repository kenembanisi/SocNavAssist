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
import time

from agent import AgentClass
from rvo_control import RvoControl
from data_logger import DataLogger
from pedestrians import PedestriansClass

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
    agent_properties = {'radius': 0.25}
    agent = AgentClass(agent_id, agent_properties)
    

    ######################### Define pedestrians as obstacles #######################
    pedestrians = PedestriansClass()


    ################ Initiate RVO controller for agent & obstacle set ###############
    D = 0.2 # radius extension for differential drive condition
    tau = float(rospy.get_param("rvo_planning_horizon"))
    rvo_agent = RvoControl(agent, pedestrians, D=D, tau=tau)


    ################ Initialize data logger (for active objects only) ###############
    trial_name = rospy.get_param("trial_name")
    logger = DataLogger(scenario, trial_name, pedestrians)


    ############################ Set agent goal location ############################
    goal = [-6.5, 8.2]
    
    ################### Get control mode from ROS parameter server ##################
    control_mode = rospy.get_param('trial_condition')
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
        v_opt, v_suitable, v_admissible, heading_delta, delta_t = rvo_agent.compute_V_opt(goal, alpha=alpha)
        # -------------------------------------------------------------------------

        # get desired/goal agent velocity -----------------------------------------
        # v_goal = rvo_agent.get_goal_velocity()

        # get agent velocity ------------------------------------------------------
        # v_current = agent.get_agent_velocities()

        # check goal reached ------------------------------------------------------
        if rvo_agent.reached:
            time_to_goal = (time.time() - t_start)
        
        # store states ------------------------------------------------------------
        # logger.store_data(v_opt, v_suitable, v_admissible, v_goal, v_current, time_to_goal, delta_t, rvo_agent.sim_states)
        logger.store_data(rvo_agent.sim_states, time_to_goal)

        # update agent's state ----------------------------------------------------
        if AUTO:
            agent.update_controls(v_opt[2], v_suitable) # only takes v_opt[1]: the
                                                    # kinematically feasible velocities

        # publish heading_delta ---------------------------------------------------
        agent.publish_heading_delta(heading_delta) # this is for shared control in manual
                                                   # control mode

        # publish optimal velocity data -------------------------------------------
        agent.publish_optimal_vel_data(v_opt[1])


        # rospy.loginfo("The computed optimal control is: %s", str([round(v_opt[1][0],2), round(v_opt[1][1],2)]))

        # Set stop time -----------------------------------------------------------
        t_stop = time.time()
        dt = t_stop - t_start

        # Save logged data at intervals -------------------------------------------
        # save_interval = 20 # seconds
        # if round(dt) > 1 and round(dt) % save_interval == 0:
        #     logger.save_data()
        #     rospy.loginfo("<<<<< Trial Saving Complete! >>>>>")
        
        # Terminate simulation (all nodes) once goal is reached -------------------
        if rvo_agent.reached:
            logger.save_data()
            rospy.loginfo("<<<<< Goal Reached! >>>>>")
            rospy.signal_shutdown("<<<<< Shutting Down Simulation >>>>>")

        if scenario == "practice":
            rospy.on_shutdown(logger.save_data)

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
        


