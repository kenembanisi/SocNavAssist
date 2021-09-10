#!/usr/bin/env python

"""
localisation.py


"""


#------------------------------------------------------------------------------------
# Import packages
#------------------------------------------------------------------------------------

import rospy
import numpy as np
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import tf


#------------------------------------------------------------------------------------
# Define localisation class
#------------------------------------------------------------------------------------
class Localisation():
    
    # Constructor:
    def __init__(self):

        self.agent_id = "trina2"

        # pose of the base_link from gazebo world frame (world is gazebo frame)
        self.world_to_base_link_X = 0.0
        self.world_to_base_link_Y = 0.0
        self.world_to_base_link_Z = 0.0
        self.world_to_base_link_Yaw = 0.0

        # pose of the base_link from odom frame
        self.odom_to_base_link_X = 0.0
        self.odom_to_base_link_Y = 0.0
        self.odom_to_base_link_Z = 0.0
        self.odom_to_base_link_Yaw = 0.0

        # Instantiate topic services
            # model_state subscriber
        self.model_subscriber = rospy.Subscriber('/gazebo/model_states', 
            ModelStates, self.model_states_callback)

            # odom subscriber
        self.odom_subscriber = rospy.Subscriber('/base_controller/odom', 
            Odometry, self.odom_callback)

            # map_to_odom transform broadcaster
        self.map_to_odom_broadcaster = tf.TransformBroadcaster()
        

    def model_states_callback(self, data):
        """
        Callback function which updates the position of the agent in 
        world (gazebo) frame
        
        Arguments:
            - data [message struct]

        Returns:
            - None
        """

        idx = data.name.index(self.agent_id)

        # Retrieve states from data
        self.world_to_base_link_X = data.pose[idx].position.x
        self.world_to_base_link_Y = data.pose[idx].position.y
            # convert from quaternion to euler
        orientation_quaterion = data.pose[idx].orientation
        orientation_euler = euler_from_quaternion([orientation_quaterion.x,
                                            orientation_quaterion.y, 
                                            orientation_quaterion.z, 
                                            orientation_quaterion.w])
        # self.theta = np.degrees(orientation_euler[2])
        self.world_to_base_link_Yaw = orientation_euler[2]


    def odom_callback(self, data):
        """
        Callback function which updates the position of the agent in 
        odom frame
        
        Arguments:
            - data [message struct]

        Returns:
            - None
        """

        # Retrieve states from data
        self.odom_to_base_link_X = data.pose.pose.position.x
        self.odom_to_base_link_Y = data.pose.pose.position.y
            # convert from quaternion to euler
        orientation_quaterion = data.pose.pose.orientation
        orientation_euler = euler_from_quaternion([orientation_quaterion.x,
                                            orientation_quaterion.y, 
                                            orientation_quaterion.z, 
                                            orientation_quaterion.w])
        # self.theta = np.degrees(orientation_euler[2])
        self.odom_to_base_link_Yaw = orientation_euler[2]

    def localize(self):
        # construct map->base_link transform
        angle = -1.732
        R_map_to_gazebo = np.array([[math.cos(angle), -math.sin(angle)],
                                    [math.sin(angle), math.cos(angle)]])
        # p_map_to_gazebo = np.array([])

    #TODO: Complete this work
