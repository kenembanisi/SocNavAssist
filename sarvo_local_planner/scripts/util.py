#!/usr/bin/env python3

import os

participant_ID = ['S01', 'S02', 'S03', 'S04', 'S05', 'S06',
                  'S07', 'S08', 'S09', 'S10', 'S11', 'S12']

for ID in participant_ID:

    log_directory = os.path.dirname(os.path.abspath(__file__))+'/logs/'+ID+'/'
    # log_directory = os.path.dirname(os.path.abspath(__file__))+'/logs/ALL_DATA/'

    for filename in os.listdir(log_directory):

        # split up filename
        trial_ID = filename.split('_')

        if trial_ID[1] != "testing":
            continue
        
        # new_name = trial_ID[0:6]+trial_ID[9]+trial_ID[7:]
        new_name = trial_ID[0]+'_'+trial_ID[1]+'_'+trial_ID[2]+'_'+trial_ID[3]+'_'+ \
            trial_ID[4]+'_'+trial_ID[5]+'_'+trial_ID[6]+'_'+ \
            trial_ID[7]+'_'+trial_ID[8]+'_'+trial_ID[10]+'_'+ \
            trial_ID[9]+'_'+trial_ID[11]+'_'+trial_ID[12]

        os.rename(log_directory+filename, log_directory+new_name)
        # print(log_directory+new_name)