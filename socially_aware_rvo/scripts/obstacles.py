#!/usr/bin/env python

"""
obstacles.py

Describes the ObstacleClass...


"""


#------------------------------------------------------------------------------------
# Import packages
#------------------------------------------------------------------------------------

import rospy
import numpy as np
from gazebo_msgs.msg import ModelStates, ModelState
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import time


#------------------------------------------------------------------------------------
# Define obstacle class
#------------------------------------------------------------------------------------
class ObstacleClass():

    # Constructor:
    def __init__(self, obstacle_id):

        # Assign attributes:
        self.obstacle_id = obstacle_id
        self.bounding_radius = 0.9          # 0.45 for intimate region, 0.9 for personal region
        self.max_linear_velocity = 2.0      # TODO: Get this from rosparam?
        self.max_angular_velocity = 1.7     # TODO: Get this from rosparam?

        # get initial model position
        model_state = rospy.wait_for_message('gazebo/model_states', ModelStates)
        idx = model_state.name.index(self.obstacle_id)
        self.x = model_state.pose[idx].position.x
        self.y = model_state.pose[idx].position.y
        # self.z = 0
        self.theta = 0
        self.prev_x = 0
        self.prev_y = 0
        self.prev_theta = 0

        self.v = [0,0]   
        self.omega = 0 
        # self.v_pref = obstacle_properties['pref_velocity']

        self.curr_time = time.time()
        self.prev_time = time.time()

        # Instantiate topic services
            # model_state subscriber
        # self.model_subscriber = rospy.Subscriber('/gazebo/model_states', 
        #     ModelStates, self.update_states_callback)

        # publisher to base_controller
        # vel_topic_name = '/' + self.obstacle_id + '/cmd_vel'
        # self.velocity_publisher = rospy.Publisher(vel_topic_name,
        #     Twist, queue_size=1)


    def update_states(self):
        """
        Updates the states by pulling single topic at each call instead of using a subscriber
        """

        # get one instance of message
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('/gazebo/model_states', 
                                ModelStates, timeout=1)
                # scan_data = rospy.wait_for_message('base_scan', LaserScan, timeout=1)
            except:
                pass
        
        # Find the index of this obstacle_id in the name list:
        idx = data.name.index(self.obstacle_id)

        # Retrieve states from data
        self.x = data.pose[idx].position.x
        self.y = data.pose[idx].position.y
            # convert from quaternion to euler
        orientation_quaterion = data.pose[idx].orientation
        orientation_euler = euler_from_quaternion([orientation_quaterion.x,
                                            orientation_quaterion.y, 
                                            orientation_quaterion.z, 
                                            orientation_quaterion.w])
        self.theta = np.degrees(orientation_euler[2])
        
        # compute dt
        self.curr_time = time.time()
        dt = self.curr_time - self.prev_time

        # compute velocities
        self.v = [(self.x - self.prev_x)/dt,
                  (self.y - self.prev_y)/dt]
        self.omega = (self.theta - self.prev_theta)/dt

        # set the previous values
        self.prev_x = self.x
        self.prev_y = self.y
        self.prev_theta = self.theta
        self.prev_time = self.curr_time




