#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import LaserScan
import numpy as np
import math
import time
import argparse
import sys
import os
from datetime import datetime
from tf.transformations import euler_from_quaternion


"""
data_logger.py
"""

class DataLogger():

    def __init__(self, scenario, trial_name, pedestrians):

        self.pedestrians = pedestrians
        self.pedestrians_list = self.pedestrians.total_pedestrian_list
        self.pedestrian_ids = []
        self.num_active_obstacles = len(self.pedestrians_list)

        temp_pedestrian_ids = []
        for i in range(self.num_active_obstacles):
            if self.pedestrians_list[i].type == "single":
                temp_pedestrian_ids.append("actor")
            if self.pedestrians_list[i].type == "group":
                temp_pedestrian_ids.append("group")
        self.pedestrian_ids.append(temp_pedestrian_ids)
        
        self.n_models = self.num_active_obstacles + 1 # plus one is for the agent
        self.scenario = scenario
        self.trial_name = trial_name

        self.time_to_goal = 0
            
        # variables
        self.x = [[] for i in range(self.n_models)]
        self.y = [[] for i in range(self.n_models)]
        self.theta = [[] for i in range(self.n_models)]
        self.v = [[] for i in range(self.n_models)]
        self.omega = [[] for i in range(self.n_models)]
        self.v_opt = []
        self.v_suitable = []
        self.v_admissible = []
        self.v_goal = []

        # define path
        self.directory = os.path.dirname(os.path.abspath(__file__))+'/logs/'

    def store_data(self, v_opt_, v_suitable_, v_admissible_, v_goal_, time_to_goal):
        
        # update pedestrian data ------------------------------------------------------------------------
        self.pedestrians_list = self.pedestrians.total_pedestrian_list
        temp_pedestrian_ids = []
        self.num_active_obstacles = len(self.pedestrians_list)

        for i in range(self.num_active_obstacles):
            if self.pedestrians_list[i].type == "single":
                temp_pedestrian_ids.append("actor")
            if self.pedestrians_list[i].type == "group":
                temp_pedestrian_ids.append("group")

        self.pedestrian_ids.append(temp_pedestrian_ids)

        
        # get one instance of message -------------------------------------------------------------------
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('/gazebo/model_states', 
                                ModelStates, timeout=1)
                # scan_data = rospy.wait_for_message('base_scan', LaserScan, timeout=1)
            except:
                pass
        

        # get data for "trina2" ------------------------------------------------------------------------
        idx = data.name.index('trina2')
            # Retrieve states from data
        self.x[0].append(data.pose[idx].position.x)
        self.y[0].append(data.pose[idx].position.y)
        self.theta[0].append(euler_from_quaternion(
            [data.pose[idx].orientation.x,
                data.pose[idx].orientation.y,
                data.pose[idx].orientation.z,
                data.pose[idx].orientation.w])[2])
        self.v_opt.append(v_opt_)
        self.v_suitable.append(v_suitable_)
        self.v_admissible.append(v_admissible_)
        self.v_goal.append(v_goal_)

            # transform velocity from world frame to robot frame
        v_world = [data.twist[idx].linear.x, data.twist[idx].linear.y]
        v_robot = self.world2robot_transform(v_world, self.theta[0][-1])

            # set velocities
        self.v[0].append(v_robot[0])
        self.omega[0].append(data.twist[idx].angular.z)

        # get data for pedestrians and groups ------------------------------------------------------------
        
            # previous number of pedestrians/groups
        prev_num_active_obstacles = len(self.x) - 1
            # case #1: check if there has been an increase in number of pedestrians/groups
        if prev_num_active_obstacles < self.num_active_obstacles:
            # append states to existing pedestrian trajectories
            for i in range(1, prev_num_active_obstacles+1): # to count from 1 to n+1
                self.x[i].append(self.pedestrians_list[i-1].x)
                self.y[i].append(self.pedestrians_list[i-1].y)
            # add new pedestrian/group trajectories for newly detected pedestrians/groups
            #   set previous position values to zero
            for j in range(self.num_active_obstacles - prev_num_active_obstacles):
                x_data = [0.0] * len(self.x[0]) + [ self.pedestrians_list[j+prev_num_active_obstacles].x ]
                y_data = [0.0] * len(self.x[0]) + [ self.pedestrians_list[j+prev_num_active_obstacles].y ] 
                self.x.append(x_data)
                self.y.append(y_data)

            # case #2: check if there has been a decrease in number of pedestrians/groups
        if prev_num_active_obstacles > self.num_active_obstacles:
            # append states to existing pedestrian trajectories
            for i in range(1, self.num_active_obstacles+1): # to count from 1 to n+1
                self.x[i].append(self.pedestrians_list[i-1].x)
                self.y[i].append(self.pedestrians_list[i-1].y)
            # maintain the pedestrian/group trace but set values which would locate it outside the vicinity
            for j in range(prev_num_active_obstacles - self.num_active_obstacles):
                self.x[j+self.num_active_obstacles+1].append(40.0) # value of 40 is set arbitrarily
                self.y[j+self.num_active_obstacles+1].append(40.0) # +1 is to address indexing issue because trina2 is index '0'
        
        # case #3: check if num of active obstacles remained the same
        if prev_num_active_obstacles == self.num_active_obstacles:
            # append states to existing pedestrian trajectories
            for i in range(1, self.num_active_obstacles + 1): # to count from 1 to n+1
                self.x[i].append(self.pedestrians_list[i-1].x)
                self.y[i].append(self.pedestrians_list[i-1].y)
                # self.v[i].append(self.pedestrians_list[i-1].v[0])


        # set time to goal
        self.time_to_goal = time_to_goal


    def save_data(self):
 
        # data = np.array([self.model_ids, self.x, self.y, self.theta, self.v, self.omega, self.v_opt, self.v_suitable])
        data = np.array([self.pedestrian_ids, self.x, self.y, self.theta, self.v_opt, self.v_suitable, 
                        self.v_admissible, self.v_goal, self.v, self.omega, self.time_to_goal])
           
        time_struct = time.localtime(time.time())
        time_now = '[' + str(time_struct.tm_mon) + str(time_struct.tm_mday) + '_' + \
                    str(time_struct.tm_hour) + str(time_struct.tm_min) + ']'
        # filename = self.model_ids[i]+'_'+time_now
        # filename = 'data_'+self.scenario+'_'+time_now
        filename = self.trial_name+'_'+self.scenario+'_'+time_now

        if os.path.isdir(self.directory):
            np.save(self.directory+filename+".npy", data)
        else:
            os.makedirs(self.directory)
            np.save(self.directory+filename+".npy", data)

    def world2robot_transform(self, v, theta):
        """
        Converts vector from world frame to robot frame
        
        Arguments: v
        Returns: v_transformed
        """
        # 
        # Minv = [cos(theta)  sin(theta)
        #         -sin(theta) cos(theta)]
        # theta_rad = math.radians(self.agent.theta)
        Minv = np.array([[math.cos(theta), math.sin(theta)],
                        [-math.sin(theta), math.cos(theta)]])
        v_transformed = Minv.dot(np.array([v[0], v[1]]))
        
        # for V[1], convert rad/s to degrees/s
        # transformed_v_opt = [trans_v_opt[0], math.degrees(trans_v_opt[1])] # make a list for consistency sake
        # transformed_v_opt = [trans_v_opt[0], trans_v_opt[1]] # make a list for consistency sake
        
        return v_transformed