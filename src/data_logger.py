#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import LaserScan
import numpy as np
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

    def __init__(self):
        # define objects to track
        # self.stage = args[1]
        # self.method = args[2]

        self.object_id = ['trina2']
            
        # variables
        self.x = [[],[],[]]
        self.y = [[],[],[]]
        self.theta = [[],[],[]]
        self.v = [[],[],[]]  
        self.omega = [[],[],[]] 
        self.min_dist = []
        self.v_opt = [[],[],[]]
        self.v_suitable = [[],[],[]]

        # define path
        self.directory = os.path.dirname(os.path.abspath(__file__))+'/logs/'

    def store_data(self, v_opt_, v_suitable_):
        # get one instance of message
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('/gazebo/model_states', 
                                ModelStates, timeout=1)
                # scan_data = rospy.wait_for_message('base_scan', LaserScan, timeout=1)
            except:
                pass
        
        for i in range(len(self.object_id)):
            # Find the index of this object_id in the name list:
            idx = data.name.index(self.object_id[i])

            # Retrieve states from data
            self.x[i].append(data.pose[idx].position.x)
            self.y[i].append(data.pose[idx].position.y)
            self.theta[i].append(euler_from_quaternion(
                [data.pose[idx].orientation.x,
                 data.pose[idx].orientation.y,
                 data.pose[idx].orientation.z,
                 data.pose[idx].orientation.w])[2])
            self.v[i].append(data.twist[idx].linear.x)
            self.omega[i].append(data.twist[idx].angular.z)
            self.v_opt[i].append(v_opt_)
            self.v_suitable[i].append(v_suitable_)

        # # obtain min obstacle distance
        # scan_range = []
        # for i in range(len(scan_data.ranges)):
        #     if scan_data.ranges[i] == float('Inf'):
        #         scan_range.append(3.5)
        #     elif np.isnan(scan_data.ranges[i]):
        #         scan_range.append(0)
        #     else:
        #         scan_range.append(scan_data.ranges[i])

        # self.min_dist.append(round(min(scan_range), 2))

    def clear_data(self):
        self.x = [[],[],[]]
        self.y = [[],[],[]]
        self.theta = [[],[],[]]
        self.v = [[],[],[]]  
        self.omega = [[],[],[]] 
        self.min_dist = []


    def save_data(self):
        # save state data
        for i in range(len(self.object_id)):
            data = np.array([self.x[i], self.y[i], self.theta[i], self.v[i], self.omega[i], self.v_opt, self.v_suitable])
           
            time_struct = time.localtime(time.time())
            time_now = '[' + str(time_struct.tm_mon) + str(time_struct.tm_mday) + '_' + \
                        str(time_struct.tm_hour) + str(time_struct.tm_min) + ']'
            filename = self.object_id[i]+'_'+time_now

            if os.path.isdir(self.directory):
                np.save(self.directory+filename+".npy", data)
            else:
                os.makedirs(self.directory)
                np.save(self.directory+filename+".npy", data)

