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
from tf.transformations import euler_from_quaternion
import time
import math
from obstacles import ObstacleClass


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
        self.max_angular_acceleration = 4.5
        self.x = 0
        self.y = 0
        self.z = 0
        self.theta = 0
        self.v = 0   
        self.omega = 0 
        self.spawn_complete = False

        # Instantiate topic services
            # model_state subscriber
        self.model_subscriber = rospy.Subscriber('/gazebo/model_states', 
            ModelStates, self.update_states_callback)

            # publisher to base_controller
        self.velocity_publisher = rospy.Publisher('/base_controller/cmd_vel',
            Twist, queue_size=1)

            # publisher for velocity data
        self.data_publisher = rospy.Publisher('/velocity_data', 
                            Float64MultiArray, queue_size=1)

            # publisher for heading_delta data
        self.heading_delta_publisher = rospy.Publisher('/heading_delta', 
                            Float32, queue_size=1)


    def update_states_callback(self, data):
        """
        Callback function which updates the states (position and velocity)
        of the agent.
        
        Arguments:
            - data [message struct]

        Returns:
            - None
        """

        # Find the index of this agent_id in the name list:
        # if not self.spawn_complete:     # check if agent id is in list
        #     rospy.loginfo("#########################step in!###########################")
        #     while not self.spawn_complete:  # this should only run once at start up
        #         rospy.loginfo("#########################step in further!###########################")
        #         # if self.agent_id in data.name: 
        #         time.sleep(2)
        #         self.spawn_complete = True
        #             # break
        #         # time.sleep(1)
        #         rospy.loginfo("In the spawn checker loop")
        # time.sleep(5)

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
        self.theta = np.degrees(orientation_euler[2])
        # ---
        self.v = data.twist[idx].linear.x
        self.omega = data.twist[idx].angular.z


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
        # self.publish_data(v_opt, v_list)


    def get_agent_velocities(self):
        """
        Returns the linear and angular velocities of the agent
        
        Arguments:
            - None
        Returns:
            - [v, omega]
        """
        return [self.v, self.omega]
        

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


    def publish_data(self, v_opt, v_list):

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
