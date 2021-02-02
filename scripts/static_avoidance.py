#!/usr/bin/env python

"""
static_avoidance.py

Describes the static obstacle avoidance class for filtering velocities for collision-free
motion

"""

#------------------------------------------------------------------------------------
# Import packages
#------------------------------------------------------------------------------------

import math
import numpy as np
import rospy


class StaticAvoidance():
    """
    Class defining the static avoidance. 

    """
    
    def __init__(self, agent, ws_model, dt):
        """
        Constructor

        Arguments:
            - agent (object): the instantiated vehicleClass object
            - ws_model (list): list of lists; each list is (a, b, c) which refer to ax + by = c
                describing the equation of a line representing a wall obstacle
            - dt (float): time horizon for forward simulation

        """
        self.agent = agent
        self.ws_model = ws_model
        self.num_walls = len(self.ws_model)

        self.eff_obs_radius_tol = 0.1


        # extract agent's parameters
        self.agent_pos = [self.agent.x, self.agent.y]
        #self.agent_vel
        self.agent_radius = self.agent.bounding_radius
        self.dt = dt # set the time horizon
        # threshold for reaching goal
        self.goal_threshold = 0.5


    def compute_velocity(self, goal):
        """

        """

        # update agent state
        self.agent_pos = [self.agent.x, self.agent.y]

        # compute agent velocity
        self.agent_vel = self.compute_V_desired(goal)

        norm_v = self.compute_distance(self.agent_vel, [0, 0]) # magnitude of the agent velocity

        V_suitable = []

        # check current status of agent
        self.set_current_status()

        # sample velocities and compute their suitability
        for theta in np.arange(0, 2*math.pi, 0.1): # <---- Search from 1 to 2pi ~ direction of motion
            for rad in np.arange(0.02, norm_v+0.02, norm_v/10.0): # <-- Search from 0.02 (avoid zero) to agent desired velocity
                candidate_v = [rad*math.cos(theta), rad*math.sin(theta)] # <-- candidate velocity
                # suitable = True

                # perform forward simulation
                new_pose = self.forward(candidate_v)

                # check collision with static features in environment
                suitable = self.check_collision(new_pose)

                # append if suitable
                if suitable:
                    V_suitable.append(candidate_v)

        return V_suitable
                

    def forward(self, v):
        """

        """

        new_agent_pos = self.agent_pos + np.asarray(v) * self.dt 

        # rospy.loginfo(new_agent_pos)

        return new_agent_pos

    
    def set_current_status(self):
        """

        """
        # clear variable
        self.current_status = []
        # check current status with all walls
        for i in range(self.num_walls):
            # get the wall's parameters
            a = self.ws_model[i][0]
            b = self.ws_model[i][1]
            c = self.ws_model[i][2]

            if a * self.agent_pos[0] + b * self.agent_pos[1] \
                            > (c - self.agent_radius - self.eff_obs_radius_tol):
                self.current_status.append('greater')
            else:
                self.current_status.append('smaller')


    def check_collision(self, new_agent_pos):
        """

        """
        forward_status = []
        # check future status with all walls
        for i in range(self.num_walls):
            # get the wall's parameters
            a = self.ws_model[i][0]
            b = self.ws_model[i][1]
            c = self.ws_model[i][2]

            if a * new_agent_pos[0] + b * new_agent_pos[1] \
                            > (c - self.agent_radius - self.eff_obs_radius_tol):
                forward_status.append('greater')
            else:
                forward_status.append('smaller')

            if forward_status == self.current_status:
                safe = True
            else:
                safe = False

            return safe
            

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


    def compute_V_desired(self, goal):
        """
        Computes the velocity vector pointing from the agent's current position
        to the goal position
        
        Arguments:
            - goal (list, i.e. [gx, gy])
        Returns:
            - V_desired (list, [vd_x, vd_y])
        Credit:
            Adapted from Meng's code - https://github.com/MengGuo/RVO_Py_MAS
        """

        V_desired = []
        # find the vector pointing from agent pose to goal
        vec = np.array([goal[0] - self.agent_pos[0], goal[1] - self.agent_pos[1]])
        vec_mag = self.compute_distance(vec, [0,0]) # compute magnitude of vector
        vec_dir = vec*(1/vec_mag) # compute the unit vector
        V_desired = vec_dir * self.agent.max_linear_velocity # multiply unit vector with max velocity

        # check if the agent has reached the goal
        if self.reach_goal(goal, self.agent_pos, self.goal_threshold):
            V_desired = [0.0001, 0.0001]
            self.reached = True

        return V_desired

    
    def reach_goal(self, pose1, pose2, bound=1):
        """
        Checks if two positions are very close (i.e. distance below a bound)

        Arguments:
            - pose1 & pose2 (list, [px, py])
            - bound (float): distance tolerance to classify closeness
        Returns (bool)

        Credit:
            Adapted from Meng's code - https://github.com/MengGuo/RVO_Py_MAS
        """
        if self.compute_distance(pose1, pose2) < bound:
            return True
        else:
            return False