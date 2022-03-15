#!/usr/bin/env python3

from matplotlib import transforms
import numpy as np
import math
import os
import argparse
import matplotlib.pyplot as plt
# import statistics


def data_limits(count):
    max_linear_velocity = 2.0
    max_angular_velocity = 2.0
    max_linear_acceleration = 2.0
    max_angular_acceleration = 4.5

    # set the limit lines
    max_linear_vel_limit = np.ones((1,count)) * max_linear_velocity
    min_linear_vel_limit = np.ones((1,count)) * -max_linear_velocity
    max_angular_vel_limit = np.ones((1,count)) * max_angular_velocity
    min_angular_vel_limit = np.ones((1,count)) * -max_angular_velocity

    max_linear_acc_limit = np.ones((1,count-1)) * max_linear_acceleration
    min_linear_acc_limit = np.ones((1,count-1)) * -max_linear_acceleration
    max_angular_acc_limit = np.ones((1,count-1)) * max_angular_acceleration
    min_angular_acc_limit = np.ones((1,count-1)) * -max_angular_acceleration

    return [max_linear_vel_limit, min_linear_vel_limit,
            max_angular_vel_limit, min_angular_vel_limit,
            max_linear_acc_limit, min_linear_acc_limit,
            max_angular_acc_limit, min_angular_acc_limit]


def compute_accelerations(vel, time_delta):
    acc_array = []
    for idx in range(1, len(vel)-1):
        # acc_array.append((vel[idx] - vel[idx-1])/time_delta[idx])
        if time_delta[idx] < 0.0001:
            time_delta[idx] = 0.0001
        acc_array.append((vel[idx] - vel[idx-1])/time_delta[idx])
    return acc_array


def DD2point_velocity(vel, theta_rad):
    """
    Tranforms robot velocity by M(theta) from DD (kinematic constrained) to point velocity space
    
    Arguments: vel
    Returns: vel_point
    """
    # 
    # M = [cos(theta)  -D*sin(theta)
    #      sin(theta)   D*cos(theta)]
    # theta_rad = math.radians(theta)
    D = 0.2
    M = np.array([[math.cos(theta_rad), -D*math.sin(theta_rad)],
                    [math.sin(theta_rad), D*math.cos(theta_rad)]])
    vel_point = M.dot(np.array([vel[0], vel[1]]))
        
    return vel_point


def point2DD_velocity(vel, theta_rad):
    """
    Tranforms robot velocity by Minv(theta) from point velocity space to DD (kinematic constrained)
    
    Arguments: vel
    Returns: vel_dd
    """
    # 
    # Minv = [cos(theta)  sin(theta)
    #      -sin(theta)/D   cos(theta)/D]
    # theta_rad = math.radians(theta)
    D = 0.2
    Minv = np.array([[math.cos(theta_rad), math.sin(theta_rad)],
                    [-(math.sin(theta_rad)/D),  (math.cos(theta_rad)/D)]])
    vel_dd = Minv.dot(np.array([vel[0], vel[1]]))
        
    return vel_dd


def pose_transform(x_list, y_list):
    transformed_x_list = [ (-y - 5.52) for y in y_list]
    transformed_y_list = [ (x - 6.80) for x in x_list]
    return transformed_x_list, transformed_y_list


def in_between(theta_right, theta_vAB, theta_left):
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


def dist(pose1, pose2):
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


def check_intrusion(agent_pos, pedestrian_pos, proxemics_level):

    # define proxemics level
    agent_radius = 0.5
    if proxemics_level == "intimate":
        space_radius = 0.45 + agent_radius
    if proxemics_level == "personal":
        space_radius = 1.2 + agent_radius
    if proxemics_level == "social":
        space_radius = 3.6 + agent_radius
    
    clearance = dist(agent_pos, pedestrian_pos)

    if clearance < space_radius:
        return 1
    else:
        return 0


def calc_ttc(agent_pos, agent_v, agent_theta, pedestrian_pos, pedestrian_v, mode):
    # check if there is a collision in the horizon for the pedestrian

    # check the inputs
    pA = agent_pos
    vA = DD2point_velocity(agent_v, agent_theta)
    pB = pedestrian_pos
    vB = pedestrian_v if (type(pedestrian_v) is list) else [0.0, 0.0]


    # define intrusion mode
    agent_radius = 0.5
    if mode == "collision":
        space_radius = 0.45 + agent_radius
    if mode == "personal":
        space_radius = 1.2 + agent_radius
    tol = 0.1

    dist_BA = dist(pB, pA) # mag of distance btw agent and obstacle
    eff_obs_radius = space_radius + tol # minkowski sum; +tol is to give to tolerance

    # Check that RVO is not computed for agent in collision with obs
    if eff_obs_radius > dist_BA:
        dist_BA = eff_obs_radius
    phi = math.asin(eff_obs_radius/dist_BA) # phi is the angle btw vector connecting agent and obs and vector from 
                                            # agent which is tangential with the obs effective boundary
    # using RVO method, translate the apex of the RVO
    alpha = 1
    RVO_apex_pos = [pA[0] + (1-alpha)*vA[0] + alpha*vB[0],
                    pA[1] + (1-alpha)*vA[1] + alpha*vB[1]]
    theta_BA = math.atan2(pB[1]-pA[1], pB[0]-pA[0]) # orientation of vAB
    # find orientations of boundary vectors lambda_right and lambda_left
    theta_lambda_right = theta_BA - phi
    theta_lambda_left = theta_BA + phi
    # compute lambda_left and lambda right
    lambda_right = [math.cos(theta_lambda_right), math.sin(theta_lambda_right)]
    lambda_left = [math.cos(theta_lambda_left), math.sin(theta_lambda_left)]

    vAB = [vA[0] + pA[0] - RVO_apex_pos[0], 
           vA[1] + pA[1] - RVO_apex_pos[1]]
    # find the angles the RVO boundaries make with the global X and then check if 
    # the angle vAB makes with the global X is within that
    theta_vAB = math.atan2(vAB[1], vAB[0])
    theta_right = math.atan2(lambda_right[1], lambda_right[0])
    theta_left = math.atan2(lambda_left[1], lambda_left[0])
    # check if the velocity vector is suitable by:
    #   (1) checking if theta_vAB falls between theta_right and theta_left
    if in_between(theta_right, theta_vAB, theta_left):
        if dist(vAB, [0,0]) == 0.0: # included to account for ZeroDivisionError
            ttc = 0.0
        else:
            ttc = dist_BA/dist(vAB, [0,0])
    else:
        ttc = 8888 # dummy value

    return ttc


def compute_derivative(val):
    val_array = np.array(val)
    der_array = np.diff(val_array)
    return der_array


############################################################################################################
# MAIN FUNCTION
############################################################################################################
def main(args):

    ## store data in array
    # data = np.array([self.x,                        # 0
    #                 self.y,                         # 1
    #                 self.theta,                     # 2
    #                 self.v,                         # 3
    #                 self.omega,                     # 4
    #                 self.v_opt,                     # 5
    #                 self.v_goal,                    # 6
    #                 self.v_commanded,               # 7
    #                 self.heading_delta,             # 8
    #                 self.control_delta,             # 9
    #                 self.optimal_traj,              # 10
    #                 self.operator_traj,             # 11
    #                 self.optimal_feature_count,     # 12
    #                 self.operator_feature_count,    # 13
    #                 self.time])                     # 14


    # load agent data
    data_filename = args.data
    log_directory = os.path.dirname(os.path.abspath(__file__))+'/'
    data = np.load(log_directory+data_filename, allow_pickle=True, encoding='latin1')
    n_frames = len(data[0][0])

    # get agent data
    agent_x, agent_y = pose_transform(data[0][0][:], data[1][0][:])
    agent_theta = data[3][0]
    agent_v = data[3][0]
    agent_omega = data[4][0]

    # get pedestrian data
    pedestrian_x = data[0][1:]
    pedestrian_y = data[1][1:]
    # pedestrian_theta = data[3][1:]
    pedestrian_v = data[3][1:]
    # pedestrian_omega = data[9][1:]
    num_pedestrians = len(data[0])-1

    # get control data
    heading_delta = data[8]

    ######################################### PATH QUALITY #################################################
    # PATH LENGTH & PATH LENGTH RATIO
    path_length_list = [ math.sqrt((agent_x[i+1]-agent_x[i])**2 + (agent_y[i+1]-agent_y[i])**2) for i in range(n_frames-1) ]
    path_length = sum(path_length_list)
    avg_path_length = path_length

    # CUMULATIVE HEADING CHANGES
    # Described by the cumulative heading changes normalized by the trajectory length
    heading_diff = [ abs(agent_omega[i+1] - agent_omega[i]) for i in range(n_frames-1)]
    heading_diff_norm = sum(heading_diff) / (n_frames-1)
    avg_heading_diff_norm = heading_diff_norm


    # TIME TO COMPLETION
    completion_time = data[14][-1]
    avg_time_to_complete = completion_time

    ######################################## SOCIAL AWARENESS ##############################################
    # AVG. CLOSEST DISTANCE TO PEDESTRIANS
        # minimum distance to each pedestrian at each timestep
    min_pair_wise_dist = [ min([ dist([agent_x[j], agent_y[j]], [pedestrian_x[i][j], pedestrian_y[i][j]]) \
                                            for i in range(num_pedestrians) ]) for j in range(n_frames) ]
    avg_min_dist = sum(min_pair_wise_dist)/n_frames
    avg_avg_min_dist = avg_min_dist

    
    # PROXEMICS INTRUSIONS
        # intimate
    intimate = [ [ check_intrusion([agent_x[j], agent_y[j]], [pedestrian_x[i][j], pedestrian_y[i][j]], "intimate") \
                                            for i in range(num_pedestrians) ] for j in range(n_frames) ]
    intimate_int = [ sum([ 1 if (intimate[j+1][i] - intimate[j][i]) == 1 else 0 for i in range(num_pedestrians) ]) \
                                            for j in range(n_frames-1) ]
    num_intimate = sum(intimate_int)
    avg_num_intimate = num_intimate

        # personal
    personal = [ [ check_intrusion([agent_x[j], agent_y[j]], [pedestrian_x[i][j], pedestrian_y[i][j]], "personal") \
                                            for i in range(num_pedestrians) ] for j in range(n_frames) ]
    personal_int = [ sum([ 1 if (personal[j+1][i] - personal[j][i]) == 1 else 0 for i in range(num_pedestrians) ]) \
                                            for j in range(n_frames-1) ]
    num_personal = sum(personal_int)
    avg_num_personal = num_personal

        # social
    social = [ [ check_intrusion([agent_x[j], agent_y[j]], [pedestrian_x[i][j], pedestrian_y[i][j]], "social") \
                                            for i in range(num_pedestrians) ] for j in range(n_frames) ]
    social_int = [ sum([ 1 if (social[j+1][i] - social[j][i]) == 1 else 0 for i in range(num_pedestrians) ]) \
                                            for j in range(n_frames-1) ]
    num_social = sum(social_int)
    avg_num_social = num_social


    # MINIMUM TIME TO INTRUSION (OR COLLISION)
        # at each timestep, check which pedestrian would lead to a collision
            # for each potential collision, calculate the time to collision
    # min_ttc = [ min([ calc_ttc([agent_x[j], agent_y[j]], [agent_v[j], agent_omega[j]], agent_theta[j], [pedestrian_x[i][j], pedestrian_y[i][j]], \
    #                             pedestrian_v[i][j], "collision") for i in range(num_pedestrians) ]) for j in range(n_frames) ]
    
    min_ttc = [ min([ calc_ttc([agent_x[j], agent_y[j]], [agent_v[j], agent_omega[j]], agent_theta[j], [pedestrian_x[i][j], pedestrian_y[i][j]], \
                                pedestrian_v[i][j], "collision") for i in range(9) ]) for j in range(n_frames) ]
    tmp = []
    for i in range(n_frames):
        if min_ttc[i] != 8888:
            tmp.append(min_ttc[i])
    avg_min_ttc = sum(tmp[1:])/len(tmp[1:])
    # median_min_ttc = statistics.median(tmp)
    
    avg_avg_min_ttc = avg_min_ttc
    # avg_median_min_ttc = median_min_ttc


    ######################################## MOTION QUALITY ##############################################
    # AVG. SPEED
    avg_linear_speed = sum(agent_v)/n_frames
    avg_angular_speed = sum(agent_omega)/n_frames

    # AVG. ACCELERATION
    linear_acc = compute_derivative(agent_v)
    avg_linear_acc = sum(linear_acc)/len(linear_acc)

    angular_acc = compute_derivative(agent_omega)
    avg_angular_acc = sum(angular_acc)/len(angular_acc)

    # AVG. JERK
    linear_jerk = compute_derivative(linear_acc)
    avg_linear_jerk = sum(linear_jerk)/len(linear_jerk)

    angular_jerk = compute_derivative(angular_acc)
    avg_angular_jerk = sum(angular_jerk)/len(angular_jerk)


    ######################################## DISAGREEMENT ##############################################
    # MEAN DISAGREEMENT (using heading delta)
    disagreement = [ abs(heading_delta[i]) for i in range(n_frames)]
    avg_disagreement = sum(disagreement)/len(disagreement)
    avg_avg_disagreement = avg_disagreement




if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plotter")
    # parser.add_argument('--data', default='logs/test_learning_static-01_layout-01_case1_cautious_MC_[32_164].npy', help='logged data filename')
    parser.add_argument('--data', default='logs/test_learning_crossing_dynamic-01_layout-01_case1_cautious_MC_[32_1634].npy', help='logged data filename')              
    args = parser.parse_args()

    main(args)