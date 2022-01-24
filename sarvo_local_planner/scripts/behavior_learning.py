#!/usr/bin/env python

import subprocess
import pandas as pd
import numpy as np
import time
import csv


####################################################################################
# Step 0. Set parameters
####################################################################################
learning_rate = 0.001
num_iterations = 5
num_repetitions = 3


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
num_demos = len(df[feature_names[0]])

human_features_exp = [sum(df[f])/num_demos for f in feature_names]
feature_num = len(feature_names)


####################################################################################
# Step 2. Generate trajectories using SA-RVO
####################################################################################
print("*************************************************************")
print("******************* Starting IRL training *******************")
print("*************************************************************\n")

# 2.1. Initialize weights ------------------------------------------------------
weights = np.array([0.05, 0.75, 0.0, 1.0, 0.03])
print('---- Initial weights: {} \n'.format(str(weights)))


# 2.2. Setup the iteration -----------------------------------------------------
for iteration in range(num_iterations):
    print('iteration: {}/{} \n'.format(iteration+1, num_iterations))
    

# 2.3. Run the ROS simulation and store feature counts to a file ---------------
    #  2.3.1. Create new storage file for current iteration's results
    time_struct = time.localtime(time.time())
    time_now = '[' + str(time_struct.tm_mon) + str(time_struct.tm_mday) + '_' + \
                str(time_struct.tm_hour) + str(time_struct.tm_min) + ']'
    current_filename = 'data/'+behavior+'_demo_features_'+time_now+'.csv'  
    with open('../'+current_filename, 'w')  as file:
        csvwriter = csv.writer(file)
        csvwriter.writerow(columns)
    
    #  2.3.2. Define weight and argument list for ROS
    w = ''
    for weight in weights:
        w += str(round(weight, 2)) + '/'
    w = w[:-1]
    arg_list = [ exec_file, scenario, current_filename, w ]

    #  2.3.3. Run the simulation for the defined number of repetitions
    #       This will store the feature counts in the current open file
    for reps in range(num_repetitions):
        subprocess.check_call(arg_list)


# 2.4. Calculate the average sarvo feature count ------------------------------
    df2 = pd.read_csv('../'+current_filename)

    feature_names = ['F1', 'F2', 'F3', 'F4', 'F5']
    num_demos = len(df2[feature_names[0]])

    sarvo_features_exp = [sum(df2[f])/num_demos for f in feature_names]
    # print(sarvo_features_exp)


# 2.5. Compute the gradient ---------------------------------------------------
    grad = np.array(sarvo_features_exp) - np.array(human_features_exp)


# 2.6. Perform gradient descent -----------------------------------------------
    weights += learning_rate * grad

    print('---- Gradient: {}'.format(str(grad)))
    print('---- Weight update: {} \n'.format(str(weights)))
