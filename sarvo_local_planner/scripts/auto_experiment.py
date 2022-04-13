#!/usr/bin/env python

import subprocess
# import pandas as pd
import numpy as np
# import time
# import csv
# import copy


####################################################################################
# Step 0. Set parameters
####################################################################################
num_repetitions = 1
display_flag = "false"

scenarios = ['scenario1-approach', 'scenario2-crossing', 'approach-01_layout-01', 'crossing-01_layout-01']
behaviors = ['safety_aligned', 'goal_aligned']
# case = 4
exec_file = '../bash/scenario-test.sh'


# Scenario 1: Basic approach
for scenario in scenarios:
    for behavior in behaviors:
        arg_list = [ exec_file, scenario, behavior ]

        for reps in range(num_repetitions):
            subprocess.check_call(arg_list)
            subprocess.check_call('../bash/killgazebo.sh') # To kill gzserver when it hangs


    