#!/usr/bin/env python

"""
agent.py

Describes the AgentClass...


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
import time
import math


#------------------------------------------------------------------------------------
# Define Agent class
#------------------------------------------------------------------------------------
class AgentClass():

    # Constructor:
    def __init__(self, agent_id, agent_properties):

        # Assign attributes:
        self.agent_id = agent_id
        self.bounding_radius = agent_properties['radius']
        self.max_linear_velocity = 2.0      # TODO: Get this from rosparam?
        self.max_angular_velocity = 2.0     # TODO: Get this from rosparam?
        self.max_linear_acceleration = 2.0 
        self.max_angular_acceleration = 4.5 # formerly 4.5
        self.x = 0
        self.y = 0
        self.z = 0
        self.theta = 0
        self.v_actual = 0   
        self.omega_actual = 0 
        self.v_cmd = 10.1   
        self.omega_cmd = 10.1 
        self.spawn_complete = False

        # Instantiate topic services
            # model_state subscriber
        self.model_subscriber = rospy.Subscriber('/gazebo/model_states', 
            ModelStates, self.update_pos_states_callback)

            # odom subscriber
        self.odom_subscriber = rospy.Subscriber('/base_controller/odom', 
            Odometry, self.update_vel_states_callback)

            # publisher to base_controller
        self.velocity_publisher = rospy.Publisher('/base_controller/cmd_vel',
            Twist, queue_size=1)

            # base_controller subscriber
        self.cmd_vel_subscriber = rospy.Subscriber('/base_controller/cmd_vel',
            Twist, self.update_cmd_vel_callback)

            # publisher for velocity data
        self.data_publisher = rospy.Publisher('/velocity_data', 
                            Float64MultiArray, queue_size=1)

            # publisher for heading_delta data
        self.heading_delta_publisher = rospy.Publisher('/heading_delta', 
                            Float32, queue_size=1)

            # publisher for control_delta data
        self.control_delta_publisher = rospy.Publisher('/control_delta', 
                            Float64MultiArray, queue_size=1)


    def update_pos_states_callback(self, data):
        """
        Callback function which updates the position of the agent.
        
        Arguments:
            - data [message struct]

        Returns:
            - None
        """

        idx = data.name.index(self.agent_id)

        # Retrieve states from data
        self.x = data.pose[idx].position.x
        self.y = data.pose[idx].position.y
            # convert from quaternion to euler
        orientation_quaterion = data.pose[idx].orientation
        orientation_euler = euler_from_quaternion([orientation_quaterion.x,
                                            orientation_quaterion.y, 
                                            orientation_quaterion.z, 
                                            orientation_quaterion.w])
        # self.theta = np.degrees(orientation_euler[2])
        self.theta = orientation_euler[2]


    def update_vel_states_callback(self, data):
        """
        Callback function which updates the velocity of the agent.
        
        Arguments:
            - data [message struct]

        Returns:
            - None
        """

        self.v_actual = data.twist.twist.linear.x # this is correct! Confirmed.
        self.omega_actual = data.twist.twist.angular.z

    
    def update_cmd_vel_callback(self, data):
        """
        Callback function which obtains the commanded velocity of the agent.
        
        Arguments:
            - data [message struct]

        Returns:
            - None
        """

        self.v_cmd = data.linear.x
        self.omega_cmd = data.angular.z

    
    def update_controls(self, v_opt, v_list):
        """
        Publishes the given velocity to the cmd_vel topic.
        
        Arguments:
            - v_opt [v, omega]

        Returns:
            - None
        """

        # Initialize cmd_vel variable:
        cmd_vel = Twist()

        # Set the variables into cmd_vel:
        cmd_vel.linear.x = v_opt[0]
        cmd_vel.angular.z = v_opt[1]

        # Publish the cmd_vel message:
        self.velocity_publisher.publish(cmd_vel)

        # call the velocity data publisher:
        # self.publish_data(v_opt)


    def get_current_agent_velocities(self):
        """
        Returns the current linear and angular velocities of the agent
        
        Arguments:
            - None
        Returns:
            - [v, omega]
        """
        return [self.v_actual, self.omega_actual]


    def get_commanded_agent_velocities(self):
        """
        Returns the commanded linear and angular velocities of the agent
        
        Arguments:
            - None
        Returns:
            - [v_cmd, omega_cmd]
        """
        return [self.v_cmd, self.omega_cmd]
        

    def compute_heading(self, theta):
        """
        Computes the heading of the robot from the reference position based on a
        rotation of self.theta.

        Arguments: 
            - theta (float)
        Returns: 
            - new_heading (list)

        """
        init_heading = np.array([0, -1])
        theta = math.radians(theta)
        rotation_matrix = np.array([[math.cos(theta), math.sin(theta)],
                                   [-math.sin(theta), math.cos(theta)]])
        new_heading = rotation_matrix.dot(init_heading)
        return [new_heading[0], new_heading[1]]


    def publish_optimal_vel_data(self, v_opt):

        # Instantiate Float64MultiArray
        self.array = Float64MultiArray()
        self.array.data = [v_opt[0], v_opt[1]]

        # Publish array data
        self.data_publisher.publish(self.array)

    
    def publish_heading_delta(self, delta):

        # Instantiate Float32
        heading_delta = Float32()
        heading_delta.data = delta

        # Publish array data
        self.heading_delta_publisher.publish(heading_delta)

    def publish_control_delta(self, control_delta):

        # Instantiate Float64MultiArray
        self.array = Float64MultiArray()
        self.array.data = [control_delta[0], control_delta[1]]

        # Publish array data
        self.control_delta_publisher.publish(self.array)
