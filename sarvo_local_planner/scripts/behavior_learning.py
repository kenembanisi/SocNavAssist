#!/usr/bin/env python

import subprocess
import pandas as pd
import numpy as np
import time
import csv


####################################################################################
# Step 0. Set parameters
####################################################################################
learning_rate = 0.005
num_iterations = 50
num_repetitions = 5
display_flag = "false"

rounder = lambda lst, decimal_places: str([round(e, decimal_places) for e in lst])

####################################################################################
# Step 1. Load feature count data for human demonstration data from a file
####################################################################################
scenario = 'crossing-01'
behavior = 'cautious'
exec_file = '../bash/test.sh'
filename = '../data/human_demo_features.csv'
df = pd.read_csv(filename)

columns = ['Scenario','Behavior','F1','F2','F3','F4','F5','']
feature_names = ['F1', 'F2', 'F3', 'F4', 'F5']
num_human_demos = len(df[feature_names[0]])

human_feature_counts = df[['F1', 'F2', 'F3', 'F4', 'F5']].to_numpy()
# human_features_exp = [sum(df[f])/num_demos for f in feature_names]
feature_num = len(feature_names)


####################################################################################
# Step 2. Generate trajectories using SA-RVO
####################################################################################
print("*************************************************************")
print("******************* Starting IRL training *******************")
print("*************************************************************\n")

# 2.1. Initialize weights ------------------------------------------------------
weights = np.array([0.05, 0.75, 0.0, 1.0, 0.03])
# weights = np.array([0.1341, 0.8814, 0.0, 1.1021, 0.1681] )
print('---- Initial weights: {} \n'.format(str(weights)))


time_struct = time.localtime(time.time())
time_now = '[' + str(time_struct.tm_mon) + str(time_struct.tm_mday) + '_' + \
            str(time_struct.tm_hour) + str(time_struct.tm_min) + ']'
log_filename = '../data/learning_logs/'+behavior+'_learning_log_'+time_now+'.txt'  

with open(log_filename, 'w') as log_file_obj:
    log_file_obj.write("*********************************************************** \n")
    log_file_obj.write(behavior+'_learning_log_'+time_now+ "txt \n")
    log_file_obj.write("*********************************************************** \n \n")
    log_file_obj.write("Scenario: [" + scenario +"] \n")
    log_file_obj.write("Behavior: [" + behavior +"] \n")
    log_file_obj.write("Learning rate: [" + str(learning_rate) +"] \n")
    log_file_obj.write("Num iterations and repetitions: [" + str(num_iterations) +"/" + str(num_repetitions)+"] \n \n")
    # log_file_obj.write("Human avg feature counts: " + rounder(human_features_exp, 4) +" \n \n")
    log_file_obj.write("Initial weights, w0: " + rounder(weights, 4) +" \n \n \n")


# 2.2. Setup the iteration -----------------------------------------------------
for iteration in range(num_iterations):
    print('iteration: {}/{} \n'.format(iteration+1, num_iterations))
    

# 2.3. Run the ROS simulation and store feature counts to a file ---------------
    #  2.3.1. Create new storage file for current iteration's results
    time_struct = time.localtime(time.time())
    time_now = '[' + str(time_struct.tm_mon) + str(time_struct.tm_mday) + '_' + \
                str(time_struct.tm_hour) + str(time_struct.tm_min) + ']'
    current_filename = 'data/'+behavior+'_demo_features_'+time_now+'.csv'  
    with open('../'+current_filename, 'a')  as file:
        csvwriter = csv.writer(file)
        csvwriter.writerow(columns)
    
    #  2.3.2. Define weight and argument list for ROS
    w = ''
    for weight in weights:
        w += str(round(weight, 2)) + '/'
    w = w[:-1]
    arg_list = [ exec_file, scenario, current_filename, w, display_flag ]

    #  2.3.3. Run the simulation for the defined number of repetitions
    #       This will store the feature counts in the current open file
    for reps in range(num_repetitions):
        subprocess.check_call(arg_list)

    subprocess.check_call('../bash/killgazebo.sh') # To kill gzserver when it hangs

# 2.4. Calculate the average sarvo feature count ------------------------------
    df2 = pd.read_csv('../'+current_filename)

    feature_names = ['F1', 'F2', 'F3', 'F4', 'F5']
    num_sarvo_demos = len(df2[feature_names[0]])

    # sarvo_features_exp = [sum(df2[f])/num_demos for f in feature_names]
    sarvo_feature_counts = df2[['F1', 'F2', 'F3', 'F4', 'F5']].to_numpy()
    # print(sarvo_features_exp)


# 2.5. Normalize the feature counts -------------------------------------------
    # concatenate human and sarvo feature counts
    total_feature_counts = np.concatenate((sarvo_feature_counts, human_feature_counts), axis=0)
    # compute max/min feature values
    max_v = [total_feature_counts[:,i].max() for i in range(feature_num)]
    min_v = [total_feature_counts[:,i].min() for i in range(feature_num)]
    # normalize each feature value
    for i in range(num_human_demos):
        for j in range(feature_num):
            if max_v[j] == 0:
                human_feature_counts[i][j] = 0.0
            else:
                human_feature_counts[i][j] = \
                    (human_feature_counts[i][j]-min_v[j]) / (max_v[j]-min_v[j])
    for i in range(num_sarvo_demos):
        for j in range(feature_num):
            if max_v[j] == 0:
                sarvo_feature_counts[i][j] = 0.0
            else:
                sarvo_feature_counts[i][j] = \
                    (sarvo_feature_counts[i][j]-min_v[j]) / (max_v[j]-min_v[j])

# 2.5. Compute the gradient ---------------------------------------------------
    sarvo_feature_exp = sarvo_feature_counts.mean(axis=0)
    sarvo_feature_exp[2] = 0.0 # set the operator-related feature to zero
    human_feature_exp = human_feature_counts.mean(axis=0)
    grad = np.array(sarvo_feature_exp) - np.array(human_feature_exp)


# 2.6. Perform gradient descent -----------------------------------------------
    weights -= learning_rate * grad

    print('---- Gradient: {}'.format(str(grad)))
    print('---- Weight update: {} \n'.format(str(weights)))


    with open(log_filename, 'a') as log_file_obj:
        log_file_obj.write("*********************************************************** \n")
        log_file_obj.write("Iteration: " + str(iteration+1) + " / " + str(num_iterations)+" \n")
        log_file_obj.write("*********************************************************** \n")
        log_file_obj.write("Feature count file: " + current_filename +" \n")
        log_file_obj.write("Num demos: " + str(num_sarvo_demos) +" \n \n")
        log_file_obj.write("Human avg feature count: " + rounder(human_feature_exp, 4) +" \n")
        log_file_obj.write("Sarvo avg feature count: " + rounder(sarvo_feature_exp, 4) +" \n")
        log_file_obj.write("Gradient: " + rounder(grad, 4) +" \n")
        log_file_obj.write("Updated weights: " + rounder(weights, 4) +" \n \n \n")

    