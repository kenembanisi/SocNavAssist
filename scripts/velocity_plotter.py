#!/usr/bin/env python3

import numpy as np
import math
import os
import argparse
import matplotlib.pyplot as plt




############################################################################################################
# MAIN FUNCTION
############################################################################################################

def main(args):

    # load agent data
    data_filename = args.data
    log_directory = os.path.dirname(os.path.abspath(__file__))+'/logs/'
    data = np.load(log_directory+data_filename, allow_pickle=True, encoding='latin1')
    n_frames = len(data[1])

    # plot data
        # instantiate figure and axes object
    fig, (ax1, ax2) = plt.subplots(2, 1)
        # v_opt
    v_opt = data[4]
    v_opt_linear = [v_opt[i][0] for i in range(len(v_opt))]
    v_opt_angular = [v_opt[i][1] for i in range(len(v_opt))]

        # v_actual
    v_actual_linear = data[7][0]
    v_actual_angular = data[8][0]

    ax1.plot(v_opt_linear, label='Optimal')
    ax1.plot(v_actual_linear, label='Current')

    ax2.plot(v_opt_angular, label='Optimal')
    ax2.plot(v_actual_angular, label='Current')
    
    # set labels
    ax1.set_ylabel('Velocity (m/s)')
    ax2.set_ylabel('Velocity (rad/s)')

    # set title
    ax1.title.set_text('Linear Velocity')
    ax2.title.set_text('Angular Velocity')

    # set legends
    ax1.legend(fontsize=8)
    ax2.legend(fontsize=8)

    plt.show()





if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plotter")
    parser.add_argument('--data', default='data_[35_1219].npy', help='logged data filename')
                
    args = parser.parse_args()

    main(args)
