#!/usr/bin/env python3

import numpy as np
import math
import csv
import os
import argparse
import matplotlib.pyplot as plt
import statistics


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
    agent_radius = 0.4
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

def calc_ttc(agent_pos, agent_v, agent_theta, pedestrian_pos, pedestrian_v, mode, proximity_threshold):
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
        elif dist_BA >= proximity_threshold:
            ttc = 8888 # dummy value
        else:
            ttc = dist_BA/dist(vAB, [0,0])
    else:
        ttc = 8888 # dummy value

    return ttc

def compute_derivative(val):
    val_array = np.array(val)
    der_array = np.diff(val_array)
    return der_array

def proximity_checker(agent_pos, pedestrian_pos, proximity_threshold):
    clearance = dist(agent_pos, pedestrian_pos)

    if clearance < proximity_threshold:
        return 1
    else:
        return 0

############################################################################################################
# MAIN FUNCTION
############################################################################################################
def main(args):

    # flags on the analysis to run
    intrusions_analysis = False
    time_to_collision_analysis = False
    disagreement_analysis = True
    disagreement_vector_analysis = False
    path_length = False

    ##################################### ITERATE OVER TRIALS ##############################################
    # get participant ID
    # participant_ID = args.participant_ID
    scenarios = ['appro', 'cross', 'rando']
    # scenarios = ['approach-01_layout-01', 'approach-01_layout-02', 'approach-02_layout-01', 'approach-02_layout-02',
    #             'crossing-01_layout-01', 'crossing-03_layout-01', 'crossing-03_layout-02',
    #             'random-01_layout-01', 'random-01_layout-02', 'random-02_layout-01']
    participant_ID = ['S03', 'S04', 'S05', 'S06', 'S07', 'S08', 'S10', 'S11', \
                         'S13', 'S14', 'S15', 'S16', 'S17', 'S18', 'S19']

    summary_dir = os.path.dirname(os.path.abspath(__file__))+'/logs/results'
    # log_directory = os.path.dirname(os.path.abspath(__file__))+'/logs/'+participant_ID+'/'

    if intrusions_analysis:
        #######################################  INTIMATE INTRUSIONS  ##########################################
        with open(summary_dir+'/summary_data_scenarios_intrusions.csv', mode='w') as output:
            
            dw = csv.DictWriter(output, delimiter=',', fieldnames=['Scenario', 'MC', 'H', 'V-T', 'V-B', 'HV-T', 'HV-B'])
            dw.writeheader()
                
            results_per_condition = {}
            
            csv_writer = csv.writer(output, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)

            conditions = ['MC', 'H', 'V-T', 'V-B', 'HV-T', 'HV-B']
            # conditions = ['HV-B']
            
            for scenario in scenarios:
                
                avg_num_intimate = []
                avg_num_personal = []
                avg_num_social = []
                avg_avg_min_ttc = []
                avg_median_min_ttc = []
                avg_avg_disagreement = []

                # for ID in participant_ID:
                for condition in conditions:
                    
                    for ID in participant_ID:
                        log_directory = os.path.dirname(os.path.abspath(__file__))+'/logs/'+ID+'/'

                        for filename in os.listdir(log_directory):
                            # check filename
                            # if not filename.startswith('P0'):
                            #     continue

                            # split up filename
                            trial_ID = filename.split('_')
                        
                            # if trial_ID[4] == condition and trial_ID[3] == 'test' and trial_ID[1]+'_'+trial_ID[2] == scenario:
                            if trial_ID[4] == condition and trial_ID[3] == 'test' and trial_ID[1][0:5] == scenario:
                                ########################################### LOAD DATA ##################################################
                                # load stored data
                                # data_filename = args.data
                                data = np.load(log_directory+filename, allow_pickle=True, encoding='latin1')
                                n_frames = len(data[1][0])

                                # comment:
                                print('Analysing [ ' + filename + ' ]')

                                # get agent data
                                agent_x = data[1][0]
                                agent_y = data[2][0]
                                agent_theta = data[3][0]
                                agent_v = data[8][0]
                                agent_omega = data[9][0]

                                # get pedestrian data
                                pedestrian_x = data[1][1:]
                                pedestrian_y = data[2][1:]
                                pedestrian_theta = data[3][1:]
                                pedestrian_v = data[8][1:]
                                pedestrian_omega = data[9][1:]
                                num_pedestrians = len(pedestrian_x)

                                # get control data
                                heading_delta = data[15]

                                
                                # PROXEMICS INTRUSIONS
                                    # intimate
                                intimate = [ [ check_intrusion([agent_x[j], agent_y[j]], [pedestrian_x[i][j], pedestrian_y[i][j]], "intimate") \
                                                                        for i in range(num_pedestrians) ] for j in range(n_frames) ]
                                intimate_int = [ sum([ 1 if (intimate[j+1][i] - intimate[j][i]) == 1 else 0 for i in range(num_pedestrians) ]) \
                                                                        for j in range(9, n_frames-1) ] # started counting at 10 to cancel out 
                                                                                                        # effects of collision at the beginning
                                num_intimate = sum(intimate_int)
                                avg_num_intimate.append(num_intimate)

                    if len(avg_num_intimate) == 0:
                        results_per_condition[condition] = 0
                    else:
                        results_per_condition[condition] =  sum(avg_num_intimate)/len(avg_num_intimate)

                    # reset the lists
                    avg_num_intimate.clear()
                            

                ######################################## WRITE IN CSV FILE ##########################################


                csv_writer.writerow([ scenario, round(results_per_condition['MC'], 2), round(results_per_condition['H'], 4),
                                    round(results_per_condition['V-T'], 2), round(results_per_condition['V-B'], 2), 
                                    round(results_per_condition['HV-T'], 2), round(results_per_condition['HV-B'], 2)])

                # comment:
                print('Done with [ ' + scenario + ' ]')
        
        # comment:
        print('Done with Intimate Intrusions')
    
    
    if time_to_collision_analysis:
        #######################################  TIME TO INTRUSION  ##########################################
        with open(summary_dir+'/summary_data_ttc.csv', mode='w') as output:
            
            dw = csv.DictWriter(output, delimiter=',', fieldnames=['Participants', 'MC', 'H', 'V-T', 'V-B', 'HV-T', 'HV-B'])
            dw.writeheader()
                
            results_per_condition = {}
            
            csv_writer = csv.writer(output, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)

            conditions = ['MC', 'H', 'V-T', 'V-B', 'HV-T', 'HV-B']
            # conditions = ['HV-B']
            
            for participant in participant_ID:
                
                avg_num_intimate = []
                avg_num_personal = []
                avg_num_social = []
                avg_avg_min_ttc = []
                avg_median_min_ttc = []
                avg_median_min_ttc_proximity = []
                avg_avg_disagreement = []

                # for ID in participant_ID:
                for condition in conditions:
                    
                    log_directory = os.path.dirname(os.path.abspath(__file__))+'/logs/'+participant+'/'

                    for filename in os.listdir(log_directory):
                        # check filename
                        # if not filename.startswith('P0'):
                        #     continue

                        # split up filename
                        trial_ID = filename.split('_')
                    
                        if trial_ID[4] == condition and trial_ID[3] == 'test':
                            ########################################### LOAD DATA ##################################################
                            # load stored data
                            # data_filename = args.data
                            data = np.load(log_directory+filename, allow_pickle=True, encoding='latin1')
                            n_frames = len(data[1][0])

                            # comment:
                            # print('Analysing [ ' + filename + ' ]')

                            # get agent data
                            agent_x = data[1][0]
                            agent_y = data[2][0]
                            agent_theta = data[3][0]
                            agent_v = data[8][0]
                            agent_omega = data[9][0]

                            # get pedestrian data
                            pedestrian_x = data[1][1:]
                            pedestrian_y = data[2][1:]
                            pedestrian_theta = data[3][1:]
                            pedestrian_v = data[8][1:]
                            pedestrian_omega = data[9][1:]
                            num_pedestrians = len(pedestrian_x)

                            # get control data
                            heading_delta = data[15]


                            # MINIMUM TIME TO INTRUSION (OR COLLISION)
                                # at each timestep, check which pedestrian would lead to a collision
                                    # for each potential collision, calculate the time to collision
                            # proximity_threshold = 30
                            
                            min_ttc = [ min([ calc_ttc([agent_x[j], agent_y[j]], [agent_v[j], agent_omega[j]], agent_theta[j], [pedestrian_x[i][j], pedestrian_y[i][j]], \
                                                        pedestrian_v[i][j], "collision", 30) for i in range(9) ]) for j in range(n_frames) ]
                            tmp = []
                            for i in range(n_frames):
                                if min_ttc[i] != 8888:
                                    tmp.append(min_ttc[i])
                            avg_min_ttc = sum(tmp[1:])/len(tmp[1:])
                            median_min_ttc = statistics.median(tmp)
                            
                            avg_avg_min_ttc.append(avg_min_ttc)
                            avg_median_min_ttc.append(median_min_ttc)

                            # proximity_threshold = 3
                            min_ttc_proximity = [ min([ calc_ttc([agent_x[j], agent_y[j]], [agent_v[j], agent_omega[j]], agent_theta[j], [pedestrian_x[i][j], pedestrian_y[i][j]], \
                                                        pedestrian_v[i][j], "collision", 3.0) for i in range(9) ]) for j in range(n_frames) ]
                            tmp = []
                            for i in range(n_frames):
                                if min_ttc_proximity[i] != 8888:
                                    tmp.append(min_ttc_proximity[i])
                            avg_min_ttc_proximity = sum(tmp[1:])/len(tmp[1:])
                            median_min_ttc_proximity = statistics.median(tmp)
                            
                            # avg_avg_min_ttc_proximity.append(avg_min_ttc_proximity)
                            avg_median_min_ttc_proximity.append(median_min_ttc_proximity)

                            ######################################## DISAGREEMENT ##############################################
                            # MEAN DISAGREEMENT (using heading delta)
                            # disagreement = [ abs(heading_delta[i]) for i in range(n_frames)]
                            # avg_disagreement = sum(disagreement)/len(disagreement)
                            # avg_avg_disagreement.append(avg_disagreement)


                    # results_per_condition[condition] =  sum(avg_num_intimate)
                    results_per_condition[condition] = sum(avg_median_min_ttc)/len(avg_median_min_ttc)
                    # results_per_condition['avg_avg_disagreement'] = sum(avg_avg_disagreement)/len(avg_avg_disagreement)

                    # reset the lists
                    avg_median_min_ttc.clear()
                    # avg_avg_min_ttc.clear()
                            

                ######################################## WRITE IN CSV FILE ##########################################


                csv_writer.writerow([ participant, round(results_per_condition['MC'], 2), round(results_per_condition['H'], 4),
                                    round(results_per_condition['V-T'], 2), round(results_per_condition['V-B'], 2), 
                                    round(results_per_condition['HV-T'], 2), round(results_per_condition['HV-B'], 2)])

                # comment:
                print('Done with [ ' + participant + ' ]')
        
        # comment:
        print('Done with Avg Median Min TTC')


    if disagreement_analysis:
        #######################################  MEAN DISAGREEMENT  ##########################################
        with open(summary_dir+'/summary_data_scenarios_disagreement.csv', mode='w') as output:
            
            dw = csv.DictWriter(output, delimiter=',', fieldnames=['Participants', 'MC', 'H', 'V-T', 'V-B', 'HV-T', 'HV-B'])
            dw.writeheader()
                
            results_per_condition_mean = {}
            results_per_condition_std = {}
            
            csv_writer = csv.writer(output, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)

            conditions = ['MC', 'H', 'V-T', 'V-B', 'HV-T', 'HV-B']
            # conditions = ['HV-B']
            
            for scenario in scenarios:
                
                avg_num_intimate = []
                avg_num_personal = []
                avg_num_social = []
                avg_avg_min_ttc = []
                avg_median_min_ttc = []
                avg_avg_disagreement = []

                # for ID in participant_ID:
                for condition in conditions:
                    
                    for ID in participant_ID:
                        log_directory = os.path.dirname(os.path.abspath(__file__))+'/logs/'+ID+'/'

                        for filename in os.listdir(log_directory):
                            # check filename
                            # if not filename.startswith('P0'):
                            #     continue

                            # split up filename
                            trial_ID = filename.split('_')
                    
                            # if trial_ID[4] == condition and trial_ID[3] == 'test':
                            if trial_ID[4] == condition and trial_ID[3] == 'test' and trial_ID[1][0:5] == scenario:
                                ########################################### LOAD DATA ##################################################
                                # load stored data
                                # data_filename = args.data
                                data = np.load(log_directory+filename, allow_pickle=True, encoding='latin1')
                                n_frames = len(data[1][0])

                                # comment:
                                print('Analysing [ ' + filename + ' ]')

                                # get agent data
                                agent_x = data[1][0]
                                agent_y = data[2][0]
                                agent_theta = data[3][0]
                                agent_v = data[8][0]
                                agent_omega = data[9][0]

                                # get pedestrian data
                                pedestrian_x = data[1][1:]
                                pedestrian_y = data[2][1:]
                                pedestrian_theta = data[3][1:]
                                pedestrian_v = data[8][1:]
                                pedestrian_omega = data[9][1:]
                                num_pedestrians = len(pedestrian_x)

                                # get control data
                                heading_delta = data[15]


                                ######################################## DISAGREEMENT ##############################################
                                # MEAN DISAGREEMENT (using heading delta)
                                disagreement = [ abs(heading_delta[i]) for i in range(n_frames)]
                                avg_disagreement = sum(disagreement)/len(disagreement)
                                avg_avg_disagreement.append(avg_disagreement)


                    # results_per_condition[condition] =  sum(avg_num_intimate)
                    # results_per_condition[condition] = sum(avg_median_min_ttc)/len(avg_median_min_ttc)
                    results_per_condition_mean[condition] = sum(avg_avg_disagreement)/len(avg_avg_disagreement)
                    results_per_condition_std[condition] = np.std(avg_avg_disagreement)

                    # reset the lists
                    avg_avg_disagreement.clear()
                            
            ######################################## WRITE IN CSV FILE ##########################################


                csv_writer.writerow([ scenario, round(results_per_condition_mean['MC'], 2), round(results_per_condition_mean['H'], 4),
                                    round(results_per_condition_mean['V-T'], 2), round(results_per_condition_mean['V-B'], 2), 
                                    round(results_per_condition_mean['HV-T'], 2), round(results_per_condition_mean['HV-B'], 2)])
                csv_writer.writerow([ scenario, round(results_per_condition_std['MC'], 2), round(results_per_condition_std['H'], 4),
                                    round(results_per_condition_std['V-T'], 2), round(results_per_condition_std['V-B'], 2), 
                                    round(results_per_condition_std['HV-T'], 2), round(results_per_condition_std['HV-B'], 2)])

                # comment:
                print('Done with [ ' + scenario + ' ]')


    if disagreement_vector_analysis:
         ##################################  MEAN DISAGREEMENT VECTOR  ########################################
        with open(summary_dir+'/summary_data_disagreement_vector.csv', mode='w') as output:
            
            dw = csv.DictWriter(output, delimiter=',', fieldnames=['Participants', 'MC', 'H', 'V-T', 'V-B', 'HV-T', 'HV-B'])
            dw.writeheader()
                
            results_per_condition = {}
            
            csv_writer = csv.writer(output, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)

            conditions = ['MC', 'H', 'V-T', 'V-B', 'HV-T', 'HV-B']
            # conditions = ['HV-B']
            
            for participant in participant_ID:
                
                avg_num_intimate = []
                avg_num_personal = []
                avg_num_social = []
                avg_avg_min_ttc = []
                avg_median_min_ttc = []
                avg_median_min_ttc_proximity = []
                avg_avg_disagreement = []
                avg_proximity_density = []
                avg_avg_disagreement_vector = []

                # for ID in participant_ID:
                for condition in conditions:
                    
                    log_directory = os.path.dirname(os.path.abspath(__file__))+'/logs/'+participant+'/'

                    for filename in os.listdir(log_directory):
                        # check filename
                        # if not filename.startswith('P0'):
                        #     continue

                        # split up filename
                        trial_ID = filename.split('_')
                    
                        if trial_ID[4] == condition and trial_ID[3] == 'test':
                            ########################################### LOAD DATA ##################################################
                            # load stored data
                            # data_filename = args.data
                            data = np.load(log_directory+filename, allow_pickle=True, encoding='latin1')
                            n_frames = len(data[1][0])

                            # comment:
                            # print('Analysing [ ' + filename + ' ]')

                            # get agent data
                            agent_x = data[1][0]
                            agent_y = data[2][0]
                            agent_theta = data[3][0]
                            agent_v = data[8][0]
                            agent_omega = data[9][0]

                            # get pedestrian data
                            pedestrian_x = data[1][1:]
                            pedestrian_y = data[2][1:]
                            pedestrian_theta = data[3][1:]
                            pedestrian_v = data[8][1:]
                            pedestrian_omega = data[9][1:]
                            num_pedestrians = len(pedestrian_x)

                            # get control data
                            heading_delta = data[15]
                            control_delta = data[16]

                            ######################################## DISAGREEMENT ##############################################
                            # MEAN DISAGREEMENT VECTOR (using control delta)
                            disagreement_vector = [ dist(control_delta[i], [0,0]) for i in range(n_frames)]
                            avg_disagreement_vector = sum(disagreement_vector)/len(disagreement_vector)
                            avg_avg_disagreement_vector.append(avg_disagreement_vector)


                    results_per_condition[condition] = sum(avg_avg_disagreement_vector)/len(avg_avg_disagreement_vector)

                    # reset the lists
                    avg_avg_disagreement_vector.clear()
                            

                ######################################## WRITE IN CSV FILE ##########################################


                csv_writer.writerow([ participant, round(results_per_condition['MC'], 2), round(results_per_condition['H'], 4),
                                    round(results_per_condition['V-T'], 2), round(results_per_condition['V-B'], 2), 
                                    round(results_per_condition['HV-T'], 2), round(results_per_condition['HV-B'], 2)])

                # comment:
                print('Done with [ ' + participant + ' ]')
        
        # comment:
        print('Done with Proximity Density')


    if path_length:
        #######################################  PATH LENGTH  ##########################################
        with open(summary_dir+'/summary_data_scenarios_path_length.csv', mode='w') as output:
            
            dw = csv.DictWriter(output, delimiter=',', fieldnames=['Participants', 'MC', 'H', 'V-T', 'V-B', 'HV-T', 'HV-B'])
            dw.writeheader()
                
            results_per_condition = {}
            
            csv_writer = csv.writer(output, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)

            conditions = ['MC', 'H', 'V-T', 'V-B', 'HV-T', 'HV-B']
            
            for scenario in scenarios:
                
                avg_path_length = []

                # for ID in participant_ID:
                for condition in conditions:
                    
                    for ID in participant_ID:
                        log_directory = os.path.dirname(os.path.abspath(__file__))+'/logs/'+ID+'/'

                        for filename in os.listdir(log_directory):
                            # check filename
                            # if not filename.startswith('P0'):
                            #     continue

                            # split up filename
                            trial_ID = filename.split('_')
                        
                            if trial_ID[4] == condition and trial_ID[3] == 'test' and trial_ID[1]+'_'+trial_ID[2] == scenario:
                                ########################################### LOAD DATA ##################################################
                                # load stored data
                                # data_filename = args.data
                                data = np.load(log_directory+filename, allow_pickle=True, encoding='latin1')
                                n_frames = len(data[1][0])

                                # comment:
                                print('Analysing [ ' + filename + ' ]')

                                # get agent data
                                agent_x = data[1][0]
                                agent_y = data[2][0]
                                agent_theta = data[3][0]
                                agent_v = data[8][0]
                                agent_omega = data[9][0]

                                # get pedestrian data
                                pedestrian_x = data[1][1:]
                                pedestrian_y = data[2][1:]
                                pedestrian_theta = data[3][1:]
                                pedestrian_v = data[8][1:]
                                pedestrian_omega = data[9][1:]
                                num_pedestrians = len(pedestrian_x)

                                # get control data
                                heading_delta = data[15]

                                # PATH LENGTH & PATH LENGTH RATIO
                                path_length_list = [ math.sqrt((agent_x[i+1]-agent_x[i])**2 + (agent_y[i+1]-agent_y[i])**2) for i in range(n_frames-1) ]
                                path_length = sum(path_length_list)
                                avg_path_length.append(path_length)


                    if len(avg_path_length) == 0:
                        results_per_condition[condition] = 0
                    else:
                        results_per_condition[condition] =  sum(avg_path_length)/len(avg_path_length)

                    # reset the lists
                    avg_path_length.clear()
                            

                ######################################## WRITE IN CSV FILE ##########################################


                csv_writer.writerow([ scenario, round(results_per_condition['MC'], 2), round(results_per_condition['H'], 4),
                                    round(results_per_condition['V-T'], 2), round(results_per_condition['V-B'], 2), 
                                    round(results_per_condition['HV-T'], 2), round(results_per_condition['HV-B'], 2)])

                # comment:
                print('Path Length: Done with [ ' + scenario + ' ]')


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Post Analysis")
    # parser.add_argument('--data', default='data_approach_human_[331_1638].npy', help='logged data filename')
    parser.add_argument('--participant_ID', default='P01', help='logged data filename')
                
    args = parser.parse_args()

    main(args)