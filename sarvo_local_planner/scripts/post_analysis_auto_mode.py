#!/usr/bin/env python3

import numpy as np
import math
import csv
import os
import argparse
import matplotlib.pyplot as plt
# import statistics


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

def DD2point_velocity(vel, agent_theta):
        """
        Tranforms robot velocity by M(theta) from DD (kinematic constrained) to point velocity space
        
        Arguments: vel
        Returns: vel_point
        """
        # 
        # M = [cos(theta)  -D*sin(theta)
        #      sin(theta)   D*cos(theta)]
        theta_rad = math.radians(agent_theta)
        M = np.array([[math.cos(theta_rad), -math.sin(theta_rad)],
                      [math.sin(theta_rad), math.cos(theta_rad)]])
        vel_point = M.dot(np.array([vel[0], vel[1]]))
        
        # for V[1], convert rad/s to degrees/s
        # transformed_v_opt = [trans_v_opt[0], math.degrees(trans_v_opt[1])] # make a list for consistency sake
        # transformed_v_opt = [trans_v_opt[0], trans_v_opt[1]] # make a list for consistency sake
        
        return vel_point

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

def pose_transform(x_list, y_list):
    transformed_x_list = [ (-y - 5.52) for y in y_list]
    transformed_y_list = [ (x - 6.80) for x in x_list]
    return transformed_x_list, transformed_y_list

############################################################################################################
# MAIN FUNCTION
############################################################################################################
def main(args):

    ##################################### ITERATE OVER TRIALS ##############################################
    # get participant ID
    # participant_ID = args.participant_ID
    # participant_ID = ['S02', 'S03', 'S04', 'S05', 'S06', 'S07', 'S08', 'S09', 'S10', 'S11', \
    #                      'S12', 'S13', 'S14', 'S15', 'S16', 'S17', 'S18', 'S19']
    # participant_ID = ['ALL_DATA']
    folder = 'AUTO_Testing'

    summary_dir = os.path.dirname(os.path.abspath(__file__))+'/logs/results'
    # log_directory = os.path.dirname(os.path.abspath(__file__))+'/logs/'+participant_ID+'/'

    
    ############################################################################################################
    # Write into a CSV file
    ############################################################################################################
    store_file = 'auto_test_data.csv'
    # with open(summary_dir+'/all_data_summary_data.csv', mode='w') as output:

    with open(summary_dir + '/' + store_file, mode='w') as output:
        dw = csv.DictWriter(output, delimiter=',', fieldnames=['Condition', 
                     'Path Length (in meters)', 
                     'CHC', 
                     'Completion Time',
                     'Avg Ped Clearance', 
                     'Min Ped Clearance', 
                     'Max Ped Clearance', 
                     '% Intimate Intrusions', 
                     '% Personal Intrusions', 
                     '% Social Intrusions', 
                     'Avg Disagreement'])
        dw.writeheader()
            
        results_per_condition = {}
        
        csv_writer = csv.writer(output, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)

        conditions = ['safety_aligned', 
                      'goal_aligned']
        
        for condition in conditions:
            
            avg_path_length = []
            avg_CHC = []
            avg_time_to_complete = []
            avg_ped_clearance = []
            min_ped_clearance = []
            max_ped_clearance = []
            avg_percentage_intimate = []
            avg_percentage_personal = []
            avg_percentage_social = []
            # avg_avg_min_ttc = []
            # avg_median_min_ttc = []
            avg_avg_disagreement = []

            # for ID in participant_ID:
                
            log_directory = os.path.dirname(os.path.abspath(__file__))+'/logs/'+folder+'/'

            for filename in os.listdir(log_directory):

                # split up filename
                trial_ID = filename.split('_')
                # if trial_ID[5]+'_'+trial_ID[6] == condition:
                if trial_ID[4]+'_'+trial_ID[5] == condition:
                    ########################################### LOAD DATA ##################################################
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
                    data = np.load(log_directory+filename, allow_pickle=True, encoding='latin1')
                    n_frames = len(data[0][0])

                    # comment:
                    print('Analysing [ ' + filename + ' ]')

                    if n_frames == 0:
                        print('Saving Error: [ ' + filename + ' ]')
                        continue

                    # get agent data
                    agent_x, agent_y = pose_transform(data[0][0][:], data[1][0][:])
                    agent_theta = data[3][0]
                    agent_v = data[3][0]
                    agent_omega = data[4][0]

                    # get pedestrian data
                    num_ped_groups = len(data[0])-1
                    num_pedestrians = 9
                    pedestrian_x = []
                    pedestrian_y = []
                    for i in range(num_pedestrians):
                        ped_x, ped_y = pose_transform(data[0][i+1], data[1][i+1])
                        pedestrian_x.append(ped_x)
                        pedestrian_y.append(ped_y)

                    pedestrian_v = data[3][1:]
                    # pedestrian_omega = data[9][1:]
                    

                    # get control data
                    heading_delta = data[8]
                    control_delta = data[9]

                    ######################################### PATH QUALITY #################################################
                    
                    # TIME TO COMPLETION
                    completion_time = data[14][-1]
                    avg_time_to_complete.append(completion_time)
                    
                    # PATH LENGTH & PATH LENGTH RATIO
                    path_length_list = [ math.sqrt((agent_x[i+1]-agent_x[i])**2 + (agent_y[i+1]-agent_y[i])**2) for i in range(n_frames-1) ]
                    path_length = sum(path_length_list)
                    avg_path_length.append(path_length)

                    # # CUMULATIVE HEADING CHANGES
                    # # Described by the cumulative heading changes normalized by the trajectory length
                    # heading_diff = [ abs(agent_omega[i+1] - agent_omega[i]) for i in range(n_frames-1)]
                    heading_diff = [ abs(agent_theta[i+1] - agent_theta[i]) for i in range(n_frames-1)]
                    CHC = sum(heading_diff) / (n_frames-1)
                    avg_CHC.append(CHC)

                    

                    ######################################## SOCIAL AWARENESS ##############################################
                    
                    pair_wise_dist = [ [ dist([agent_x[j], agent_y[j]], [pedestrian_x[i][j], pedestrian_y[i][j]]) \
                                                            for i in range(num_pedestrians) ] for j in range(n_frames) ]
                    dist_to_closest_person = [ min(pair_wise_dist[j]) for j in range(n_frames) ]
                    
                    # AVG. DISTANCE TO CLOSEST PERSON ----------------------------------------------------------------------
                        # minimum distance to each pedestrian at each timestep
                    avg_dist_to_closest_person = sum(dist_to_closest_person[1:])/n_frames
                    avg_ped_clearance.append(avg_dist_to_closest_person)

                    # print("num_pedestrians is: "+str(num_pedestrians))

                    # MIN & MAX DISTANCE TO CLOSEST PERSON -----------------------------------------------------------------
                    min_dist_to_closest_person = min(dist_to_closest_person[1:])
                    max_dist_to_closest_person = max(dist_to_closest_person[1:])
                    min_ped_clearance.append(min_dist_to_closest_person)
                    max_ped_clearance.append(max_dist_to_closest_person)

                    # PROXEMICS INTRUSIONS ---------------------------------------------------------------------------------
                    intrusions_intimate = 0
                    intrusions_personal = 0
                    intrusions_social = 0

                    for val in dist_to_closest_person[1:]:
                        if val <= 0.45: 
                            intrusions_intimate += 1
                        elif val > 0.45 and val <= 1.2:
                            intrusions_personal += 1
                        else:
                            intrusions_social += 1


                    intrusions = [(intrusions_intimate*100.0)/n_frames, \
                                (intrusions_personal*100.0)/n_frames, \
                                (intrusions_social*100.0)/n_frames]
                    avg_percentage_intimate.append(intrusions[0])
                    avg_percentage_personal.append(intrusions[1])
                    avg_percentage_social.append(intrusions[2])


                    # MINIMUM TIME TO INTRUSION (OR COLLISION) -------------------------------------------------------------
                        # at each timestep, check which pedestrian would lead to a collision
                            # for each potential collision, calculate the time to collision
                    # min_ttc = [ min([ calc_ttc([agent_x[j], agent_y[j]], [agent_v[j], agent_omega[j]], agent_theta[j], [pedestrian_x[i][j], pedestrian_y[i][j]], \
                    #                             pedestrian_v[i][j], "collision") for i in range(num_pedestrians) ]) for j in range(n_frames) ]
                    
                    # min_ttc_per_timestep = [ min([ calc_ttc([agent_x[j], agent_y[j]], [agent_v[j], agent_omega[j]], agent_theta[j], [pedestrian_x[i][j], pedestrian_y[i][j]], \
                    #                             pedestrian_v[i][j], "collision") for i in range(9) ]) for j in range(n_frames-1) ]
                    # # min_ttc = [ min([ calc_ttc([agent_x[j], agent_y[j]], [agent_v[j], agent_omega[j]], agent_theta[j], [pedestrian_x[i][j], pedestrian_y[i][j]], \
                    # #                             pedestrian_v[i][j], "collision") for i in range(9) ]) for j in range(n_frames) ]
                    # min_ttc_per_timestep_ = []
                    # for i in range(n_frames-1):
                    #     if min_ttc_per_timestep[i] != 8888:
                    #         min_ttc_per_timestep_.append(min_ttc_per_timestep[i])
                    # # avg_min_ttc = sum(min_ttc_per_timestep_[1:])/len(min_ttc_per_timestep_[1:])
                    # # median_min_ttc = statistics.median(tmp)
                    
                    # # avg_avg_min_ttc = avg_min_ttc
                    

                    # min_ttc_personal = 0
                    # min_ttc_social = 0

                    # for val in min_ttc_per_timestep_:
                    #     if val <= 3.0: 
                    #         min_ttc_personal += 1
                    #     else:
                    #         min_ttc_social += 1

                    # num_collision_timesteps = len(min_ttc_per_timestep_)

                    # time_to_collisions = [(min_ttc_personal*100.0)/n_frames, \
                    #             (min_ttc_social*100.0)/n_frames, \
                    #             ((n_frames-num_collision_timesteps)*100.0)/n_frames]


                    # ######################################## DISAGREEMENT ##############################################
                    # # MEAN DISAGREEMENT (using heading delta)
                    disagreement = [ abs(heading_delta[i]) for i in range(n_frames)]
                    avg_disagreement = sum(disagreement)/len(disagreement)
                    avg_avg_disagreement.append(avg_disagreement)
                    # avg_avg_disagreement = 0.00


                    print(
                        "----------------- Path Quality ---------------- \n" + \
                        "Path length (m): " + str(round(path_length, 2)) + "\n" + \
                        "Path roughness(smoothness): " + str(round(CHC, 4)) + "\n" + \
                        "Time to complete (secs): " + str(round(completion_time, 2)) + "\n \n" + \
                        "----------------- Social Awareness -------------- \n" + \
                        "Avg. clearance to pedestrians (m): " + str(round(avg_dist_to_closest_person, 2)) + "\n" + \
                        "Min and Max clearance to pedestrians (m): [" + str(round(min_dist_to_closest_person, 2)) + \
                            ", " + str(round(max_dist_to_closest_person, 2)) + "]\n" + \
                        "Percentage of intrusions (int | pers | soc): " + str(round(intrusions[0], 2)) + " | " + \
                                str(round(intrusions[1], 2)) + " | " + \
                                str(round(intrusions[2], 2)) + "\n" )
        
            #####################################################################################################

            results_per_condition['avg_path_length'] = sum(avg_path_length)/len(avg_path_length)
            results_per_condition['avg_CHC'] = sum(avg_CHC)/len(avg_CHC)
            results_per_condition['avg_completion_time'] = sum(avg_time_to_complete)/len(avg_time_to_complete)
            results_per_condition['avg_ped_clearance'] = sum(avg_ped_clearance)/len(avg_ped_clearance)
            results_per_condition['avg_min_ped_clearance'] = sum(min_ped_clearance)/len(min_ped_clearance)
            results_per_condition['avg_max_ped_clearance'] = sum(max_ped_clearance)/len(max_ped_clearance)
            results_per_condition['avg_percentage_intimate'] = sum(avg_percentage_intimate)/len(avg_percentage_intimate)
            results_per_condition['avg_percentage_personal'] = sum(avg_percentage_personal)/len(avg_percentage_personal)
            results_per_condition['avg_percentage_social'] = sum(avg_percentage_social)/len(avg_percentage_social)
            # results_per_condition['avg_min_ttc'] = sum(avg_avg_min_ttc)/len(avg_avg_min_ttc)
            # results_per_condition['median_min_ttc'] = sum(avg_median_min_ttc)/len(avg_median_min_ttc)
            results_per_condition['avg_avg_disagreement'] = sum(avg_avg_disagreement)/len(avg_avg_disagreement)

            #####################################################################################################

        
            ######################################## WRITE IN CSV FILE ##########################################

            # csv_writer.writerow([ condition, round(results_per_condition['avg_path_length'], 2), 
            #                     round(results_per_condition['avg_CHC'], 4),
            #                     round(results_per_condition['avg_completion_time'], 2),
            #                     round(results_per_condition['avg_ped_clearance'], 2), 
            #                     round(results_per_condition['avg_min_ped_clearance'], 2), 
            #                     round(results_per_condition['avg_max_ped_clearance'], 2), 
            #                     round(results_per_condition['avg_percentage_intimate'], 2), 
            #                     round(results_per_condition['avg_percentage_personal'], 2), 
            #                     round(results_per_condition['avg_percentage_social'], 2),
            #                     round(results_per_condition['avg_min_ttc'], 2), 
            #                     round(results_per_condition['median_min_ttc'], 2 ),
            #                     round(results_per_condition['avg_avg_disagreement'], 2) ])
            csv_writer.writerow([ condition, round(results_per_condition['avg_path_length'], 2), 
                                round(results_per_condition['avg_CHC'], 4),
                                round(results_per_condition['avg_completion_time'], 2),
                                round(results_per_condition['avg_ped_clearance'], 2), 
                                round(results_per_condition['avg_min_ped_clearance'], 2), 
                                round(results_per_condition['avg_max_ped_clearance'], 2), 
                                round(results_per_condition['avg_percentage_intimate'], 3), 
                                round(results_per_condition['avg_percentage_personal'], 3), 
                                round(results_per_condition['avg_percentage_social'], 3),
                                round(results_per_condition['avg_avg_disagreement'], 2), 0.0, 0.0 ])
        
            # comment:
            print('Done with [ ' + condition + ' ]')
        

        

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Post Analysis")
    # parser.add_argument('--data', default='data_approach_human_[331_1638].npy', help='logged data filename')
    parser.add_argument('--participant_ID', default='P01', help='logged data filename')
                
    args = parser.parse_args()

    main(args)