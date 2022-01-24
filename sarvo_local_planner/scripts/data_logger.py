#!/usr/bin/env python

import numpy as np
import os
import time

# Import ROS packages
import rospy
from sarvo_msgs.msg import SimulationStates


class DataLogger():
    
    def __init__(self):
        # initialize ros node
        rospy.init_node('data_logger')

        # initialize subscriber
        self.sim_states_subscriber = rospy.Subscriber("sarvo_simulation_states", 
            SimulationStates, self.dataCallback)

        # define path
        self.directory = os.path.dirname(os.path.abspath(__file__))+'/logs/'

        # get parameters
        self.trial_condition = rospy.get_param('trial_condition')
        self.layout = rospy.get_param('scenario_layout')
        self.scenario = rospy.get_param('scenario')
        self.trial_name = rospy.get_param('trial_name')
        self.study_phase = rospy.get_param('study_phase')

        # get one instance of message 
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('sarvo_simulation_states', 
                                SimulationStates, timeout=1)
            except:
                pass
        self.num_active_pedestrians = len(data.ped_groups)


        # initialize variables
        self.x = [[] for i in range(self.num_active_pedestrians+1)]
        self.y = [[] for i in range(self.num_active_pedestrians+1)]
        self.theta = [[] for i in range(self.num_active_pedestrians+1)]
        self.v = [[] for i in range(self.num_active_pedestrians+1)]
        self.omega = [[] for i in range(self.num_active_pedestrians+1)]
        self.v_opt = []
        self.v_goal = []
        self.v_commanded = []

        self.heading_delta = []
        self.control_delta = []

        self.optimal_traj = []
        self.operator_traj = []

        self.optimal_feature_count = []
        self.operator_feature_count = []

        self.time = []

    
    def dataCallback(self, msg):
        # get data for robot
        self.x[0].append(msg.robot.pose.x)
        self.y[0].append(msg.robot.pose.y)
        self.theta[0].append(msg.robot.pose.theta)
        self.v[0].append(msg.robot.twist.vx)
        self.omega[0].append(msg.robot.twist.w)
        # get velocities
        self.v_opt.append([[msg.optimal_candidate.velocity.x, 
                           msg.optimal_candidate.velocity.y],
                           [msg.optimal_candidate.twist.vx, 
                           msg.optimal_candidate.twist.w]])
        self.v_goal.append([msg.v_goal.x, msg.v_goal.y])
        self.v_commanded.append([msg.operator_candidate.twist.vx,
                                 msg.operator_candidate.twist.w])
        # get deltas
        # self.heading_delta.append()
        # self.control_delta.append()

        # get trajectories
        self.optimal_traj.append(msg.optimal_candidate.traj.poses)
        self.operator_traj.append(msg.operator_candidate.traj.poses)
        # feature counts
        self.optimal_feature_count.append(msg.optimal_candidate.score.raw_scores)
        self.operator_feature_count.append(msg.operator_candidate.score.raw_scores)

        # get time
        self.time.append(msg.current_time)

        # get data for pedestrians and groups ------------------------------------------------------------
        self.pedestrians_list = msg.ped_groups
        self.num_active_pedestrians = len(self.pedestrians_list)
            # previous number of pedestrians/groups
        prev_num_active_pedestrians = len(self.x) - 1
            # case #1: check if there has been an increase in number of pedestrians/groups
        if prev_num_active_pedestrians < self.num_active_pedestrians:
            # append states to existing pedestrian trajectories
            for i in range(1, prev_num_active_pedestrians+1): # to count from 1 to n+1
                self.x[i].append(self.pedestrians_list[i-1].pose.x)
                self.y[i].append(self.pedestrians_list[i-1].pose.y)
                self.v[i].append([self.pedestrians_list[i-1].velocity.x,
                                  self.pedestrians_list[i-1].velocity.y])
            # add new pedestrian/group trajectories for newly detected pedestrians/groups
            #   set previous position values to zero
            for j in range(self.num_active_pedestrians - prev_num_active_pedestrians):
                x_data = [0.0] * len(self.x[0]) + [ self.pedestrians_list[j+prev_num_active_pedestrians].pose.x ]
                y_data = [0.0] * len(self.x[0]) + [ self.pedestrians_list[j+prev_num_active_pedestrians].pose.y ] 
                v_data = [0.0, 0.0] * len(self.x[0]) + [self.pedestrians_list[j+prev_num_active_pedestrians].velocity.x,
                                                        self.pedestrians_list[j+prev_num_active_pedestrians].velocity.y]
                self.x.append(x_data)
                self.y.append(y_data)
                self.v.append(v_data)

            # case #2: check if there has been a decrease in number of pedestrians/groups
        if prev_num_active_pedestrians > self.num_active_pedestrians:
            # append states to existing pedestrian trajectories
            for i in range(1, self.num_active_pedestrians+1): # to count from 1 to n+1
                self.x[i].append(self.pedestrians_list[i-1].pose.x)
                self.y[i].append(self.pedestrians_list[i-1].pose.y)
                self.v[i].append([self.pedestrians_list[i-1].velocity.x,
                                  self.pedestrians_list[i-1].velocity.y])
            # maintain the pedestrian/group trace but set values which would locate it outside the vicinity
            for j in range(prev_num_active_pedestrians - self.num_active_pedestrians):
                self.x[j+self.num_active_pedestrians+1].append(40.0) # value of 40 is set arbitrarily
                self.y[j+self.num_active_pedestrians+1].append(40.0) # +1 is to address indexing issue because trina2 is index '0'
        
        # case #3: check if num of active obstacles remained the same
        if prev_num_active_pedestrians == self.num_active_pedestrians:
            # append states to existing pedestrian trajectories
            for i in range(1, self.num_active_pedestrians + 1): # to count from 1 to n+1
                self.x[i].append(self.pedestrians_list[i-1].pose.x)
                self.y[i].append(self.pedestrians_list[i-1].pose.y)
                self.v[i].append([self.pedestrians_list[i-1].velocity.x,
                                  self.pedestrians_list[i-1].velocity.y])

        rospy.loginfo("Logging data")


    def saveData(self):

        rospy.loginfo("Saving data")

        # store data in array
        data = np.array([self.x,                        # 0
                        self.y,                         # 1
                        self.theta,                     # 2
                        self.v,                         # 3
                        self.omega,                     # 4
                        self.v_opt,                     # 5
                        self.v_goal,                    # 6
                        self.v_commanded,               # 7
                        self.heading_delta,             # 8
                        self.control_delta,             # 9
                        self.optimal_traj,              # 10
                        self.operator_traj,             # 11
                        self.optimal_feature_count,     # 12
                        self.operator_feature_count,    # 13
                        self.time])                     # 14


        time_struct = time.localtime(time.time())
        time_now = '[' + str(time_struct.tm_mon) + str(time_struct.tm_mday) + '_' + \
                    str(time_struct.tm_hour) + str(time_struct.tm_min) + ']'

        filename = self.trial_name+'_'+self.scenario+'_'+self.layout+'_'+self.study_phase \
                        +'_'+self.trial_condition+'_'+time_now


        if os.path.isdir(self.directory):
            np.save(self.directory+filename+".npy", data)
        else:
            os.makedirs(self.directory)
            np.save(self.directory+filename+".npy", data)


# --------------------------------------------------------------
# main function
# --------------------------------------------------------------

if __name__ == "__main__":
    logger = DataLogger()

    rospy.spin()

    rospy.on_shutdown(logger.saveData)
