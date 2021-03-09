#!/usr/bin/env python

"""
main_control.py

[...]


"""


#------------------------------------------------------------------------------------
# Import packages
#------------------------------------------------------------------------------------

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



#------------------------------------------------------------------------------------
# main
#------------------------------------------------------------------------------------

if __name__ == "__main__":

    try:
        ### define static features. TODO: Detect this directly from a static occupancy grid map
        static_features_id = ['static_obstacle_1', 'static_obstacle_2']
        static_features_properties = [{'radius': 1.7, 'pref_velocity': [0, 0]},
                                    {'radius': 1.7, 'pref_velocity': [0, 0]}]

        ### define active obstacles (pedestrians)
        pedestrian_id = ['dynamic_obstacle_1', 'dynamic_obstacle_2', 'dynamic_obstacle_3', 'dynamic_obstacle_4']
        # pedestrian_id = ['dynamic_obstacle_2', 'dynamic_obstacle_3', 'dynamic_obstacle_4']
        # pedestrian_properties = [{'radius': 0.4, 'pref_velocity': [0.75, -0.1]},
        #                          {'radius': 0.4, 'pref_velocity': [0.0, -0.7]},
        #                          {'radius': 0.4, 'pref_velocity': [-0.2, -0.7]}]
        # pedestrian_properties = [{'radius': 0.4, 'pref_velocity': [0.95, 0.0]},
        #                          {'radius': 0.4, 'pref_velocity': [0.75, -0.1]},
        #                          {'radius': 0.4, 'pref_velocity': [0.0, -0.7]},
        #                          {'radius': 0.4, 'pref_velocity': [-0.2, -0.7]}]
        pedestrian_properties = [{'radius': 0.4, 'pref_velocity': [0.0, 0.0]},
                                 {'radius': 0.4, 'pref_velocity': [0.0, 0.0]},
                                 {'radius': 0.4, 'pref_velocity': [0.0, 0.0]},
                                 {'radius': 0.4, 'pref_velocity': [0.0, 0.0]}]

        ### define agent
        agent_id = 'trina2'
        # agent_id = 'dynamic_obstacle_1'
        agent_properties = {'radius': 1.0}
        

        ### Instantiate the ros node:
        rospy.init_node('test_object')

        ### initialize data logger (for active objects only)
        model_ids = ['trina2'] + pedestrian_id
        # model_ids = ['dynamic_obstacle_1'] + pedestrian_id
        logger = DataLogger(model_ids)

         ### Instantiate object class
        num_static_obstacles = len(static_features_id)
        obstacles = {}
        for i in range(num_static_obstacles):
            obstacles[i] = ObstacleClass(static_features_id[i], static_features_properties[i])

        num_pedestrians = len(pedestrian_id)
        for i in range(num_pedestrians):
            obstacles[num_static_obstacles+i] = ObstacleClass(pedestrian_id[i], pedestrian_properties[i])

        ### Instantiate vehicle class
        agent = AgentClass(agent_id, agent_properties)

        ### Instantiate rvo_control class
        D = 0.2
        rvo_agent = RvoControl(agent, obstacles, D=D, tau=1.5)

        ### set goal position. TODO: This should be set from launch file
        # goal = [4.63, 8.05]
        goal = [8.1, -10.05]
        
        ### Move the active obstacles
        for i in range(num_pedestrians):
            obstacles[i+num_static_obstacles].move()

        t_start = time.time()

        while not rospy.is_shutdown():
            
            # timer:-------------------------------------------------------------------
            # t_start = time.time()
            # -------------------------------------------------------------------------

            # Update simulation:
            v_opt, v_suitable = rvo_agent.compute_V_opt(goal, alpha=1)

            # get desired/goal agent velocity
            v_goal = rvo_agent.get_goal_velocity()

            # store states
            logger.store_data(v_opt, v_suitable, v_goal)

            # update agent's state
            agent.update_controls(v_opt[1], v_suitable)

            # Move the active obstacles
            for i in range(num_pedestrians):
                obstacles[i+2].move()
                
            # rospy.loginfo("The computed optimal control is: %s", str([round(v_opt[1][0],2), round(v_opt[1][1],2)]))

            t_stop = time.time()
            dt = t_stop - t_start
            # rospy.loginfo("Time interval: %s", str(dt))

            if round(dt) > 1 and round(dt) % 10 == 0:
                logger.save_data()
                rospy.loginfo("<<<<< Trial Saving Complete! >>>>>")


    finally:
        pass
        


