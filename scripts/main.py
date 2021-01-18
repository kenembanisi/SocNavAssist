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
        object_id = ['static_obstacle_1', 'static_obstacle_2']

        agent_id = 'trina2'
        agent_properties = {'radius': 1.0}
        object_properties = [{'radius': 1.7, 'pref_velocity': [0, 0]},
                             {'radius': 1.7, 'pref_velocity': [0, 0]}]

        # Instantiate the ros node:
        rospy.init_node('test_object')

        # initialize data logger
        logger = DataLogger()

        # Instantiate object class
        num_obstacles = len(object_id)
        obstacles = {}
        for i in range(num_obstacles):
            obstacles[i] = ObstacleClass(object_id[i], object_properties[i])

        # Include pedestrians:
        pedestrian_id = ['dynamic_obstacle_1', 'dynamic_obstacle_2', 'dynamic_obstacle_3', 'dynamic_obstacle_4']
        # pedestrian_properties = [{'radius': 0.4, 'pref_velocity': [0.7, 0.0]},
        #                          {'radius': 0.4, 'pref_velocity': [0.5, -0.1]},
        #                          {'radius': 0.4, 'pref_velocity': [0.0, -0.7]},
        #                          {'radius': 0.4, 'pref_velocity': [-0.2, -0.5]}]
        pedestrian_properties = [{'radius': 0.4, 'pref_velocity': [0.0, -0.8]},
                                 {'radius': 0.4, 'pref_velocity': [0.0, 0.0]},
                                 {'radius': 0.4, 'pref_velocity': [0.0, 0.0]},
                                 {'radius': 0.4, 'pref_velocity': [0.0, 0.0]}]
        num_pedestrians = len(pedestrian_id)
        
        for i in range(num_pedestrians):
            obstacles[num_obstacles+i] = ObstacleClass(pedestrian_id[i], pedestrian_properties[i])

        # Instantiate vehicle class
        agent = AgentClass(agent_id, agent_properties)

        # Instantiate rvo_control class
        D = 0.2
        rvo_agent = RvoControl(agent, obstacles, D=D, tau=1.5)


        goal = [4.63, 8.05]
        
        num_obstacles = len(obstacles)

        # Move the active obstacles
        for i in range(num_pedestrians):
            obstacles[i+2].move()

        t_start = time.time()

        while not rospy.is_shutdown():
            
            # timer:-------------------------------------------------------------------
            # t_start = time.time()
            # -------------------------------------------------------------------------

            # Update simulation:
            v_opt, v_suitable = rvo_agent.compute_V_opt(goal, alpha=1)

            # store states
            logger.store_data(v_opt[1], v_suitable)

            # update agent's state
            agent.update_controls(v_opt[1], v_suitable)

            # Move the active obstacles
            for i in range(num_pedestrians):
                obstacles[i+2].move()
                
            # rospy.loginfo("The computed optimal control is: %s", str([round(v_opt[1][0],2), round(v_opt[1][1],2)]))

            t_stop = time.time()
            dt = t_stop - t_start


            if round(dt) > 1 and round(dt) % 30 == 0:
                logger.save_data()
                rospy.loginfo("<<<<< Trial Saving Complete! >>>>>")


    finally:
        pass
        


