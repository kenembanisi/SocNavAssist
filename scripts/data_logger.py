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

    def __init__(self, model_ids):
        # define objects to track
        # self.stage = args[1]
        # self.method = args[2]

        # self.object_id = ['trina2']
        self.model_ids = model_ids
        self.n_models = len(self.model_ids)
            
        # variables
        self.x = [[] for i in range(self.n_models)]
        self.y = [[] for i in range(self.n_models)]
        self.theta = [[] for i in range(self.n_models)]
        self.v = [[] for i in range(self.n_models)]
        self.omega = [[] for i in range(self.n_models)]
        # self.min_dist = []
        self.v_opt = []
        self.v_suitable = []
        self.v_desired = []

        # define path
        self.directory = os.path.dirname(os.path.abspath(__file__))+'/logs/'

    # def store_data(self, v_opt_, v_suitable_, v_desired_):
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
        
        for i in range(len(self.model_ids)):
            # Find the index of this model_ids in the name list:
            idx = data.name.index(self.model_ids[i])

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

            if self.model_ids[i] == 'trina2':
                self.v_opt.append(v_opt_)
                self.v_suitable.append(v_suitable_)
                # self.v_desired.append(v_desired_)


    def save_data(self):
 
        # data = np.array([self.model_ids, self.x, self.y, self.theta, self.v, self.omega, self.v_opt, self.v_suitable])
        data = np.array([self.model_ids, self.x, self.y, self.theta, self.v_opt, self.v_suitable, self.v_desired, self.v, self.omega])
           
        time_struct = time.localtime(time.time())
        time_now = '[' + str(time_struct.tm_mon) + str(time_struct.tm_mday) + '_' + \
                    str(time_struct.tm_hour) + str(time_struct.tm_min) + ']'
        # filename = self.model_ids[i]+'_'+time_now
        filename = 'data_'+time_now

        if os.path.isdir(self.directory):
            np.save(self.directory+filename+".npy", data)
        else:
            os.makedirs(self.directory)
            np.save(self.directory+filename+".npy", data)

