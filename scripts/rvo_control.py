#!/usr/bin/env python

"""
rvo_control.py

Describes the reciprocal velocity obstacle implementation class.

"""

#------------------------------------------------------------------------------------
# Import packages
#------------------------------------------------------------------------------------

import math
import numpy as np
import rospy
import time


class RvoControl():
    """
    Class defining the RVO control. 

    """
    
    def __init__(self, agent, active_obstacle_dict, D=0, tau=0):
        """
        Constructor

        Arguments:
            - agent (object): the instantiated vehicleClass object
            - active_obstacle_dict (dict): dict containing instantiated obstacleClass objects for
                all active obstacle in the field
            - D (float): the measure of the effective radius and center for DD robot kinematics
            - tau (float): time horizon for computation of the velocity obstacle
        """
        self.agent = agent
        self.active_obstacle_dict = active_obstacle_dict
        self.num_obstacles = len(self.active_obstacle_dict)
        self.D = D
        self.tau = tau
        self.eff_obs_radius_tol = 0.1

        # extract the position, velocity, size of the obstacles into lists
        self.obstacle_pos = []
        self.obstacle_vel = []
        self.obstacle_radius = []
        for i in range(self.num_obstacles):
            self.obstacle_pos.append([self.active_obstacle_dict[i].x, self.active_obstacle_dict[i].y])
            self.obstacle_vel.append(self.active_obstacle_dict[i].v)
            self.obstacle_radius.append(self.active_obstacle_dict[i].bounding_radius)

        # extract agent's parameters
        self.agent_pos = [self.agent.x, self.agent.y]
        self.agent_theta = self.agent.theta
        #self.agent_vel
        self.agent_radius = self.agent.bounding_radius
        # state of motion
        self.reached = False
        # threshold for reaching goal
        self.goal_threshold = 0.5
        # sp
        # augment agent radius if D > 0
        if self.D > 0:
            self.augment_player_radius()
        # define time variables
        self.curr_time = time.time()
        self.prev_time = time.time()


    def compute_V_opt(self, goal, alpha=0.5):
        """
        Steps:
            1. Compute the agent's desired velocity: computeVdes()
            2. Update environment states: updateEnvStates()
                - update: obstacle -- pos, vel
                          agent -- pos, vel
            3. Compute computeVopt()
                - Compute RVOs for all obstacles
                - Search through sampled velocities for 'suitable velocities'
                  i.e. outside the RVOs
        """

        # first, update environment states:
        for i in range(self.num_obstacles):
            self.obstacle_pos[i] = [self.active_obstacle_dict[i].x, self.active_obstacle_dict[i].y]
            self.obstacle_vel[i] = self.active_obstacle_dict[i].v
        self.agent_pos = [self.agent.x, self.agent.y]
        self.agent_theta = self.agent.theta

        # for DD scenario, compute effective radius and center
        if self.D > 0:
            self.augment_player_position()

        # compute agent velocity
        self.agent_vel = self.compute_V_desired(goal)
        # self.agent_vel = self.compute_operator_goal()

        # compute RVOs for all obstacles:
        RVO_all = []
        pA = self.agent_pos
        vA = self.agent_vel
        for i in range(self.num_obstacles):
        # for each obstacle
            pB = self.obstacle_pos[i]
            vB = self.obstacle_vel[i]

            dist_BA = self.compute_distance(pB, pA) # mag of distance btw agent and obstacle
            eff_obs_radius = self.obstacle_radius[i] + self.agent_radius + self.eff_obs_radius_tol # minkowski sum; +tol is to give to tolerance
            
            # Check that RVO is not computed for agent in collision with obs
            if eff_obs_radius > dist_BA:
                dist_BA = eff_obs_radius
            phi = math.asin(eff_obs_radius/dist_BA) # phi is the angle btw vector connecting agent and obs and vector from 
                                                   # agent which is tangential with the obs effective boundary
            # using RVO method, translate the apex of the RVO
            RVO_apex_pos = [pA[0] + (1-alpha)*vA[0] + alpha*vB[0],
                            pA[1] + (1-alpha)*vA[1] + alpha*vB[1]]
            theta_BA = math.atan2(pB[1]-pA[1], pB[0]-pA[0]) # orientation of vAB
            # find orientations of boundary vectors lambda_right and lambda_left
            theta_lambda_right = theta_BA - phi
            theta_lambda_left = theta_BA + phi
            # compute lambda_left and lambda right
            lambda_right = [math.cos(theta_lambda_right), math.sin(theta_lambda_right)]
            lambda_left = [math.cos(theta_lambda_left), math.sin(theta_lambda_left)]

            RVO_BA = [RVO_apex_pos, lambda_left, lambda_right, dist_BA, eff_obs_radius]
            RVO_all.append(RVO_BA)
    
        # find set of suitable and unsuitable velocities
        V_suitable, V_unsuitable, V_admissible = self.check_intersection(pA, vA, RVO_all)
    
        # find optimal velocity choice
        V_opt_point = self.select_optimalV(V_suitable, V_unsuitable, RVO_all)

        # compute the heading_delta between current heading and optimal heading
        heading_delta = self.compute_heading_delta(V_opt_point)

        V_opt_DD = [0, 0] 

        if self.D > 0:
            # optimal velocity transformed to be kinematically constrained
            # V_opt_DD = self.transform_augmented_V(V_opt_point)
            V_opt_DD = self.transform_augmented_V_new(V_opt_point, heading_delta)

            # rospy.loginfo("linear speed: [%s], angular speed: [%s]", str(V_opt_DD[0]), str(V_opt_DD[1]))

        V_opt = [V_opt_point, V_opt_DD]

        # make sure the robot stops when it has reached target goal
        if self.reached:
            V_opt = [[0.0, 0.0], [0.0, 0.0]]

        return V_opt, V_suitable, V_admissible, heading_delta

    def check_intersection(self, pA, vA, RVO_all):
        """
        For all RVOs in the list, find the set of suitable velocity vectors which are
        not in the RVOs. 
        
        Arguments:
            - pA (list): position of the agent
            - vA (list): desired (or preferred) velocity of the agent
            - RVO_all (list of lists, each entry: [RVO_apex_pos, lambda_left, lambda_right,
                                                     dist_BA, eff_obs_radius]
        Returns:
            - V_suitable (list of lists): vectors which are admissible
            - V_unsuitable (list of lists): vectors which are not admissible
        Credit:
            Adapted from Meng's code - https://github.com/MengGuo/RVO_Py_MAS
        """

        norm_v = self.compute_distance(vA, [0, 0]) # magnitude of the agent velocity
        V_suitable = []
        V_unsuitable = []
        V_admissible = []

        # --------------------------------------------------------------------------------------------------
        # Velocity search: velocity vectors all around the agent 
        #   (assumes a point agent or holonomic robot)
        # --------------------------------------------------------------------------------------------------
            # N.B. We should be interested in searching within admissible velocities, 
            #   not the whole velocity space (though this isn't searching the whole space)

        # v_a, v_b, v_c, v_d = self.compute_velocity_limits()

        # self.v_x_min = min(v_a[0], v_b[0], v_c[0], v_d[0])
        # self.v_x_max = max(v_a[0], v_b[0], v_c[0], v_d[0])

        # self.v_y_min = min(v_a[1], v_b[1], v_c[1], v_d[1])
        # self.v_y_max = max(v_a[1], v_b[1], v_c[1], v_d[1])

        n_not_admissible = 0
        n_total = 0
        # norm_v = 1.5

        # compute admissible angular range
        phi_range = self.admissible_angular_vel_range()

        # rospy.loginfo("phi range: [%s, %s], current heading: [%s]", str(phi_range[0]), str(phi_range[1]), str(math.radians(self.agent.theta)))

        # for phi in np.arange(0, 2*math.pi, 0.1): # <---- Search from 1 to 2pi ~ direction of motion
        for phi in np.arange(phi_range[0], phi_range[1], 0.04):
            for mag in np.arange(0.02, norm_v+0.02, norm_v/10.0): # <-- Search from 0.02 (avoid zero) to agent desired velocity

                n_total += 1

                candidate_v = [mag*math.cos(phi), mag*math.sin(phi)] # <-- candidate velocity

                # admissible = self.admissibility_check(candidate_v) # <-- is the candidate velocity admissible?

                # if not admissible: # if not admissible, skip the candidate velocity
                #     n_not_admissible += 1
                #     continue

                V_admissible.append(candidate_v) # store admissible velocities

                suitable = True
                for RVO in RVO_all: # <---- Check for all the RVOs
                    RVO_apex_pos = RVO[0]
                    lambda_left = RVO[1]
                    lambda_right = RVO[2]
                    dist_BA = RVO[3]
                    eff_obs_radius = RVO[4]
                    # ---
                    vAB = [candidate_v[0] + pA[0] - RVO_apex_pos[0], 
                           candidate_v[1] + pA[1] - RVO_apex_pos[1]]
                    # find the angles the RVO boundaries make with the global X and then check if 
                    # the angle vAB makes with the global X is within that
                    theta_vAB = math.atan2(vAB[1], vAB[0])
                    theta_right = math.atan2(lambda_right[1], lambda_right[0])
                    theta_left = math.atan2(lambda_left[1], lambda_left[0])
                    # check if the velocity vector is suitable by:
                    #   (1) checking if theta_vAB falls between theta_right and theta_left
                    if self.in_between(theta_right, theta_vAB, theta_left):
                    #   (2) checking if |vAB| is greater than the minimum imminent collision velocity
                        if self.imminent_collision(vAB, dist_BA, eff_obs_radius):
                            suitable = False
                            break
                if suitable:
                    V_suitable.append(candidate_v)
                else:
                    V_unsuitable.append(candidate_v)      
        # --------------------------------------------------------------------------------------------------          
        # rospy.loginfo("Suit V: [%s], Unsuit V: [%s], Admis: [%s], Total: [%s]", str(len(V_suitable)), str(len(V_unsuitable)), str(n_total - n_not_admissible), str(n_total))

        return V_suitable, V_unsuitable, V_admissible

    def select_optimalV(self, V_suitable, V_unsuitable, RVO_all):
        """
        Find the velocity vector which minimizes some objective function
            - for suitable velocities: closest to the desired agent velocity
            - for unsuitable velocities: (1/time-to-collision) + distance to desired velocity
        
        Arguments:
            - V_suitable (list of lists):
            - V_unsuitable (list of lists):
            - RVO_all (list of lists, each entry: [RVO_apex_pos, lambda_left, lambda_right,
                                                     dist_BA, eff_obs_radius]
        Returns:
            - V_opt (list)
        Credit:
            Adapted from Meng's code - https://github.com/MengGuo/RVO_Py_MAS
        """
        pA = self.agent_pos
        vA = self.agent_vel

        if V_suitable:
            # get the velocity with minimum distance to the desired
            V_opt = min(V_suitable, key = lambda v: self.compute_distance(v, vA))

        # If no suitable velocity is found, then:
        else:
            tc_V = dict()
            for unsuit_v in V_unsuitable:
                tc_V[tuple(unsuit_v)] = 0
                tc = []
                for RVO in RVO_all:
                    RVO_apex_pos = RVO[0]
                    lambda_left = RVO[1]
                    lambda_right = RVO[2]
                    dist_AB = RVO[3]
                    eff_obs_radius = RVO[4]

                    vAB = [unsuit_v[0] + pA[0] - RVO_apex_pos[0], 
                            unsuit_v[1] + pA[1]- RVO_apex_pos[1]]

                    theta_vAB = math.atan2(vAB[1], vAB[0])
                    theta_right = math.atan2(lambda_right[1], lambda_right[0])
                    theta_left = math.atan2(lambda_left[1], lambda_left[0])

                    if self.in_between(theta_right, theta_vAB, theta_left): # if velocity is within RVO:
                        small_theta = abs(theta_vAB-0.5*(theta_left+theta_right)) # find diff between velocity angle and the midline velocity of RVO
                        if abs(dist_AB*math.sin(small_theta)) >= eff_obs_radius:
                            eff_obs_radius = abs(dist_AB*math.sin(small_theta))

                        big_theta = math.asin(abs(dist_AB*math.sin(small_theta))/eff_obs_radius)

                        dist_tg = abs(dist_AB*math.cos(small_theta))-abs(eff_obs_radius*math.cos(big_theta))
                        if dist_tg < 0:
                            dist_tg = 0                    
                        tc_v = dist_tg/self.compute_distance(vAB, [0,0]) # tc_v is time-to-collision (ttc) for each RVO
                        tc.append(tc_v)
                # finds the minimum ttc across all agents for a given velocity
                tc_V[tuple(unsuit_v)] = min(tc)+0.001

            # define weighting
            WT = 0.2
            # choose the velocity that minimizes the penalty function
            V_opt = min(V_unsuitable, key = lambda v: ((WT/tc_V[tuple(v)])+self.compute_distance(v, vA)))

        return V_opt

    def in_between(self, theta_right, theta_vAB, theta_left):
        """
        Arguments:
            - theta_right (float): angle of right-side (or front) RVO boundary with global X
            - theta_dif (float): angle of relative velocity vector with global X
            - theta_left (float): angle of left-side (or rear) RVO boundary with global X
        Credit:
            Adapted from Meng's code - https://github.com/MengGuo/RVO_Py_MAS
        """
        if abs(theta_right - theta_left) <= math.pi:
            if theta_right <= theta_vAB <= theta_left:
                return True
            else:
                return False
        else:
            if (theta_left <0) and (theta_right >0):
                theta_left += 2*math.pi
                if theta_vAB < 0:
                    theta_vAB += 2*math.pi
                if theta_right <= theta_vAB <= theta_left:
                    return True
                else:
                    return False
            if (theta_left >0) and (theta_right <0):
                theta_right += 2*math.pi
                if theta_vAB < 0:
                    theta_vAB += 2*math.pi
                if theta_left <= theta_vAB <= theta_right:
                    return True
                else:
                    return False

    def imminent_collision(self, vAB, dist_BA, eff_obs_radius):
        """
        Checks if the candidate velocity vector has a magnitude above the minimum value for
        an imminent collision based on the time horizon

        Arguments:
            - vAB (list): relative velocity vector of agent A and obstacle B
            - dist_BA (float): magniude of vector pAB, i.e. pA - pB
            - eff_obs_radius (float): agent radius + obstacle radius + tol
        Returns:
            - True/False (bool)
        """
        norm_vAB = self.compute_distance(vAB, [0,0]) # compute magnitude of vector
        # calculate the min vAB for an imminent collision
        min_V_imminent = (dist_BA - eff_obs_radius)/self.tau

        # check if candidate velocity is greater that the minimum collision-imminent velocity
        if norm_vAB >= min_V_imminent:
            return True
        else:
            return False

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

    def compute_operator_goal(self):
        """
        This function returns a vector based on the current robot heading and
        (1) max_linear_velocity, or (2) the operator's inputted linear velocity

        Arguments: None
        Returns:
            - operator_goal (list)
        """
        
        ### Implementing case (1): using max_linear_velocity
        vec_dir = self.compute_heading(self.agent.theta)

        operator_goal = [vec_dir[0] * self.agent.max_linear_velocity,
                         vec_dir[1] * self.agent.max_linear_velocity]
        
        return operator_goal

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
   
    def augment_player_radius(self):
        """
        For DD robot scenario:
        As the name suggests, this method defines a new agent RADIUS based on
        D (distance from initial robot center in the direction orthogonal to the axle)
        
        Arguments: None
        Returns: None
        """
        self.agent_radius = self.agent_radius + self.D

    def augment_player_position(self):
        """
        For DD robot scenario:
        As the name suggests, this method defines a new agent POSITION based on
        D (distance from initial robot center in the direction orthogonal to the axle)
        
        Arguments: None
        Returns: None
        """

        # compute direction vector
        heading_vec_dir = self.compute_heading(self.agent_theta) # theta was previously set to -ve
        # compute new position
        self.agent_pos = [self.agent.x + heading_vec_dir[0]*self.D,
                         self.agent.y + heading_vec_dir[1]*self.D]

    def transform_augmented_V(self, v_opt):
        """
        For DD robot scenario:
        The V_opt needs to be transformed by M(theta) to obtain the right V_opt
        
        Arguments: v_opt
        Returns: transformed_v_opt
        """
        # 
        # Minv = [cos(theta)    -sin(theta)
        #         sin(theta)/D cos(theta)/D]
        theta_rad = math.radians(self.agent.theta)
        Minv = np.array([[math.cos(theta_rad), math.sin(theta_rad)],
                        [-(math.sin(theta_rad)/self.D),  (math.cos(theta_rad)/self.D)]])
        trans_v_opt = Minv.dot(np.array([v_opt[0], v_opt[1]]))
        
        # for V[1], convert rad/s to degrees/s
        # transformed_v_opt = [trans_v_opt[0], math.degrees(trans_v_opt[1])] # make a list for consistency sake
        transformed_v_opt = [trans_v_opt[0], trans_v_opt[1]] # make a list for consistency sake
        
        return transformed_v_opt

    def transform_augmented_V_new(self, v_opt, heading_delta):
        """
        For DD robot scenario:
        The V_opt needs to be transformed to its linear and angular component and clipped appropriately
        
        Arguments: v_opt, heading_delta
        Returns: new_agent_vel
        """
        new_agent_vel = [0, 0]

        # compute the time delta
        self.curr_time = time.time()
        dt = self.curr_time - self.prev_time

        # obtain current angular velocity
        current_agent_vel = self.agent.get_agent_velocities() # agent_vel = [v, omega]

        ### linear speed
            # compute optimal linear speed
        v_opt_mag = self.compute_distance(v_opt, [0,0]) # 2-norm (magnitude) of the v_opt
            # check for acceleration limit
        linear_speed_diff = v_opt_mag - current_agent_vel[0]
        if abs(linear_speed_diff) < self.agent.max_linear_acceleration * dt:
            new_agent_vel[0] = v_opt_mag
        else:
            if linear_speed_diff < 0:
                new_agent_vel[0] =  current_agent_vel[0] - self.agent.max_linear_acceleration * dt
            else:
                new_agent_vel[0] =  current_agent_vel[0] + self.agent.max_linear_acceleration * dt

        # rospy.loginfo("des_lin_speed: [%s], cmd_lin_speed: [%s], lin_speed_diff: [%s], curr_lin_speed: [%s], dt: [%s]", \
        #                 str(v_opt_mag), str(new_agent_vel[0]), str(linear_speed_diff), str(current_agent_vel[0]), str(dt))

        ### angular speed
        angular_speed = heading_delta/dt
        angular_speed_diff = angular_speed - current_agent_vel[1]
            # check for velocity limit
        if abs(angular_speed) < self.agent.max_angular_velocity:
            # ---------------------------------------------------------------------------------------
            if abs(angular_speed_diff) < self.agent.max_angular_acceleration * dt:
                new_agent_vel[1] = angular_speed
            else:
                if angular_speed_diff < 0:
                    new_agent_vel[1] = current_agent_vel[1] - self.agent.max_angular_acceleration * dt
                else:
                    new_agent_vel[1] = current_agent_vel[1] + self.agent.max_angular_acceleration * dt
            # ---------------------------------------------------------------------------------------
        else:
            if angular_speed < 0:
                # new_agent_vel[1] = -self.agent.max_angular_velocity
                new_agent_vel[1] = current_agent_vel[1] - self.agent.max_angular_acceleration * dt
            else:
                # new_agent_vel[1] = self.agent.max_angular_velocity
                new_agent_vel[1] = current_agent_vel[1] + self.agent.max_angular_acceleration * dt

        # rospy.loginfo("heading_delta: [%s], des_ang_speed: [%s], cmd_ang_speed: [%s], curr_ang_speed: [%s], dt: [%s]", \
        #                 str(heading_delta), str(angular_speed), str(new_agent_vel[1]), str(current_agent_vel[1]), str(dt))

        # set previous time value to current time
        self.prev_time = self.curr_time

        return new_agent_vel
        
    def DD2point_velocity(self, vel):
        """
        Tranforms robot velocity by M(theta) from DD (kinematic constrained) to point velocity space
        
        Arguments: vel
        Returns: vel_point
        """
        # 
        # M = [cos(theta)  -D*sin(theta)
        #      sin(theta)   D*cos(theta)]
        theta_rad = math.radians(self.agent.theta)
        M = np.array([[math.cos(theta_rad), -math.sin(theta_rad)],
                      [math.sin(theta_rad), math.cos(theta_rad)]])
        vel_point = M.dot(np.array([vel[0], vel[1]]))
        
        # for V[1], convert rad/s to degrees/s
        # transformed_v_opt = [trans_v_opt[0], math.degrees(trans_v_opt[1])] # make a list for consistency sake
        # transformed_v_opt = [trans_v_opt[0], trans_v_opt[1]] # make a list for consistency sake
        
        return vel_point

    def get_goal_velocity(self):
        """
        Gets the agent goal/desired velocity, i.e. velocity towards the agent goal.
        Arguments: None
        Returns: self.desired_agent_vel (list)
        """
        # return self.desired_agent_vel
        return self.agent_vel

    def compute_velocity_limits(self):
        """
        Defines the admissible velocity bounds (limits) for the robot at an instance
        Arguments: None
        Returns: vertex coordinates for the admissible velocity bound in x,y space
        """
        # compute the time delta
        self.curr_time = time.time()
        dt = self.curr_time - self.prev_time

        # obtain current angular velocity
        agent_vel = self.agent.get_agent_velocities() # agent_vel = [v, omega]

        # compute range of admissible velocity headings (v_omega)
        omega_min = agent_vel[1] - self.agent.max_angular_acceleration * dt
        omega_max = agent_vel[1] + self.agent.max_angular_acceleration * dt
        if omega_max > self.agent.max_angular_velocity:
            omega_max = self.agent.max_angular_velocity
        if omega_min < -self.agent.max_angular_velocity:
            omega_min = -self.agent.max_angular_velocity
        
        if self.D > 0:
            v_omega_min = self.D * omega_min
            v_omega_max = self.D * omega_max

        # compute range of admissible velocity magnitudes 
        v_min = agent_vel[0] - self.agent.max_linear_acceleration * dt
        v_max = agent_vel[0] + self.agent.max_linear_acceleration * dt
        if v_max > self.agent.max_linear_velocity:
            v_max = self.agent.max_linear_velocity
        if v_min < -self.agent.max_linear_velocity:
            v_min = -self.agent.max_linear_velocity
        v_range = abs(v_max - v_min)

        # define admissible velocity vector vertices (clockwise direction)
        v_a = [v_min, v_omega_min] # lower left vertex
        v_b = [v_min, v_omega_max]
        v_c = [v_max, v_omega_max] # upper right vertex
        v_d = [v_max, v_omega_min]

        # rotate rect vertices by theta
        v_a_rot = self.DD2point_velocity(v_a)
        v_b_rot = self.DD2point_velocity(v_b)
        v_c_rot = self.DD2point_velocity(v_c)
        v_d_rot = self.DD2point_velocity(v_d)

        # set previous time value to current time
        self.prev_time = self.curr_time

        return v_a_rot, v_b_rot, v_c_rot, v_d_rot

    def admissibility_check(self, vel):
        """
        Checks if the velocity sampled is admissible based on kinodynamic constraints on the diff drive robot
        Arguments: vel
        Returns: bool
        """
        # admissible = False
        # if vel[0] >= self.v_x_min and vel[0] <= self.v_x_max:
        #     if vel[1] >= self.v_y_min and vel[1] <= self.v_y_max:
        #         admissible = True

        # return admissible

        admissible = False
        # linear magnitude check
        if self.compute_distance(vel, [0,0]) <= self.agent.max_linear_velocity:
            admissible = True

        return admissible

    def admissible_angular_vel_range(self):
        """
        Computes the admissible range of velocity headings given the current heading and
        the maximum angular velocity

        Arguments: None
        Returns: phi_range = [phi_min, phi_max]
        """
        phi_range = [0, 0]
        dt = 0.1    # average step size
        allowance = 1.0   # enable rvo consider a wider range of options even though its bounded

        current_heading = math.radians(self.agent.theta)

        phi_range[0] = current_heading - (self.agent.max_angular_velocity * dt) - allowance
        phi_range[1] = current_heading + (self.agent.max_angular_velocity * dt) + allowance

        return phi_range

    def compute_heading(self, theta):
        """
        Computes the heading of the robot from the reference position based on a
        rotation of theta.
        Arguments: 
            - theta (float)
        Returns: 
            - new_heading (list)
        """
        # init_heading = np.array([0, -1])
        init_heading = np.array([1, 0]) # initial heading is aligned to the world x position
        theta = math.radians(theta)
        rotation_matrix = np.array([[math.cos(theta), -math.sin(theta)],
                                   [math.sin(theta), math.cos(theta)]])
        new_heading = rotation_matrix.dot(init_heading)
        return [new_heading[0], new_heading[1]]

    def compute_heading_delta(self, v_opt):
        """
        Computes the angle difference between operator goal (i.e. current heading) and the optimal heading
        defined by rvo.
        Arguments:
            - 
        Returns: 
            - heading_delta
        """

        # calculate the optimal heading based on v_opt velocity
        # optimal_theta = math.atan((v_opt[1]/v_opt[0]))
        optimal_theta = math.atan2(v_opt[1], v_opt[0])

        # calculate heading_delta
        heading_delta = optimal_theta - math.radians(self.agent_theta) # clockwise is -ve, anticlockwise is +ve

        return heading_delta
