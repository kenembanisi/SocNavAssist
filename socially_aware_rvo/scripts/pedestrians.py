#!/usr/bin/env python

"""
pedestrians.py

Describes the PedestriansClass

1. Provide an updated list of pedestrians and pedestrian groups to RVO  
    The list should include pose and twist information
2. At every cycle, retreive msgs from /tracked_persons and /tracked_groups topics
"""


#------------------------------------------------------------------------------------
# Import packages
#------------------------------------------------------------------------------------

import rospy
import numpy as np
from spencer_tracking_msgs.msg import TrackedPerson, TrackedPersons, TrackedGroup, TrackedGroups
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import time
import math

#------------------------------------------------------------------------------------
# Define pedestrian class as a data type
#------------------------------------------------------------------------------------

class PedestrianType():
    def __init__(self, pedtype="single"):
        self.type = pedtype
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v = 0.0
        self.w = 0.0
        self.radius = 0.0
        self.pedestrian_id = 0
        self.group_pedestrian_ids = []
        self.name = "default"

#------------------------------------------------------------------------------------
# Define pedestrians class
#------------------------------------------------------------------------------------
class PedestriansClass():

    # Constructor:
    def __init__(self):

        # Assign attributes:
        # self.obstacle_id = obstacle_id
        # self.bounding_radius = 0.9          # 0.45 for intimate region, 0.9 for personal region
        # self.max_linear_velocity = 2.0      # TODO: Get this from rosparam?
        # self.max_angular_velocity = 1.7     # TODO: Get this from rosparam?

        # get initial model position
        # model_state = rospy.wait_for_message('gazebo/model_states', ModelStates)
        # idx = model_state.name.index(self.obstacle_id)
        # self.x = model_state.pose[idx].position.x
        # self.y = model_state.pose[idx].position.y
        # self.z = 0
        # self.theta = 0
        # self.prev_x = 0
        # self.prev_y = 0
        # self.prev_theta = 0

        # self.v = [0,0]   
        # self.omega = 0 
        # self.v_pref = obstacle_properties['pref_velocity']

        # self.curr_time = time.time()
        # self.prev_time = time.time()

        # initialize subscribers
        self.ped_data_sub = rospy.Subscriber('/spencer/perception/tracked_persons', 
                                TrackedPersons, self.set_ped_data)
        self.group_data_sub = rospy.Subscriber('/spencer/perception/tracked_groups', 
                                TrackedGroups, self.set_group_data)

        # initialize message data
        self.ped_data = TrackedPersons()
        self.ped_group_data = TrackedGroups()

        # Pedestrian and pedestrian_group list
        self.pedestrian_list = []
        self.pedestrian_group_list = []
        self.total_pedestrian_list = []

        # update pedestrian states
        self.update_states()


    def set_ped_data(self, ped_data):
        self.ped_data = ped_data


    def set_group_data(self, ped_group_data):
        self.ped_group_data = ped_group_data


    def update_states(self, use_groups=True):
        """
        Updates the states by pulling single topic at each call instead of using a subscriber
        """

        # # get one instance of message
        # ped_data = None
        # while ped_data is None:
        #     try:
        #         ped_data = rospy.wait_for_message('/spencer/perception/tracked_persons', 
        #                         TrackedPersons, timeout=1)
        #     except:
        #         pass
        # ped_group_data = None
        # while ped_group_data is None:
        #     try:
        #         ped_group_data = rospy.wait_for_message('/spencer/perception/tracked_groups', 
        #                         TrackedGroups, timeout=1)
        #     except:
        #         pass
        
        # clear the pedestrian and pedestrian_group lists
        # self.pedestrian_list.clear()
        # self.pedestrian_group_list.clear()
        del self.pedestrian_list[:]
        del self.pedestrian_group_list[:]

        # update pedestrian_list
        num_pedestrians = len(self.ped_data.tracks)
        for i in range(num_pedestrians):
            pedestrian = PedestrianType(pedtype="single")
            pedestrian.pedestrian_id = i
            pedestrian.x = self.ped_data.tracks[i].pose.pose.position.x
            pedestrian.y = self.ped_data.tracks[i].pose.pose.position.y
            # pedestrian.theta = getOrientation(ped_data.tracks[i].pose.orientation)
            # pedestrian.v = self.compute_linear_velocity(ped_data.tracks[i].twist.twist.linear)
            pedestrian.v = [self.ped_data.tracks[i].twist.twist.linear.x,
                            self.ped_data.tracks[i].twist.twist.linear.y]
            pedestrian.radius = 0.9  # 0.45 for intimate region, 0.9 for personal region

            self.pedestrian_list.append(pedestrian)
        
        # update pedestrian_group_list
        num_groups = len(self.ped_group_data.groups)
        for i in range(num_groups):
            group = PedestrianType(pedtype="group")
            group.group_pedestrian_ids = self.ped_group_data.groups[i].track_ids
            group.x = self.ped_group_data.groups[i].centerOfGravity.pose.position.x
            group.y = self.ped_group_data.groups[i].centerOfGravity.pose.position.y
            group.radius = self.compute_group_radius(group.group_pedestrian_ids, 
                                                    self.ped_group_data.groups[i].centerOfGravity)
            # group.theta = getOrientation(ped_group_data.groups[i].pose.orientation)

            group.v = self.compute_group_velocity(group.group_pedestrian_ids)
            

            self.pedestrian_group_list.append(group)

        if use_groups:
            self.total_pedestrian_list = self.pedestrian_list + self.pedestrian_group_list
        else:
            self.total_pedestrian_list = self.pedestrian_list
        # rospy.loginfo("Updating pedestrian states")


    def compute_linear_velocity(self, xy_data):
        return math.sqrt((xy_data.x)**2+(xy_data.y)**2)


    def compute_group_velocity(self, pedestrian_ids):
        """
        Compute group velocity by taking mean of pedestrian velocities
        """
        velocity = [0, 0]
        for i in range(len(pedestrian_ids)):
            velocity[0] += self.pedestrian_list[pedestrian_ids[i]].v[0]
            velocity[1] += self.pedestrian_list[pedestrian_ids[i]].v[1]
        
        group_velocity = [ v/len(pedestrian_ids) for v in velocity ]

        return group_velocity
        

    def compute_group_radius(self, pedestrian_ids, COG_pose):
        radius = 0.0
        num_pedestrians = len(pedestrian_ids)
        for i in range(num_pedestrians):
            center_position = [COG_pose.pose.position.x, COG_pose.pose.position.y]
            pedestrian_position = [self.pedestrian_list[pedestrian_ids[i]].x, 
                                    self.pedestrian_list[pedestrian_ids[i]].y]
            ped_to_center_dist = self.compute_distance(center_position, pedestrian_position)

            if ped_to_center_dist > radius:
                radius = ped_to_center_dist
        return radius


    def compute_distance(self, pose1, pose2):
        """
        Computes the euclidean distance between two poses in 2D
        
        Arguments:
            - pose1 & pose2 (list, [px, py])
        Returns:
            - vector magnitude or norm
        Credit:
            Adapted from Meng's code - https://github.com/MengGuo/RVO_Py_MAS
        """
        return math.sqrt((pose1[0]-pose2[0])**2+(pose1[1]-pose2[1])**2)
