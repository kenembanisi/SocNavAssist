#!/usr/bin/env python3

import numpy as np
import math
import os
import argparse
import matplotlib.pyplot as plt


def data_limits(count):
    max_linear_velocity = 2.0
    max_angular_velocity = 2.0
    max_linear_acceleration = 1.5
    max_angular_acceleration = 2.0

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

def compute_accelerations(vel):
    vel_array = np.array(vel)
    acc_array = np.diff(vel_array)
    return acc_array


############################################################################################################
# MAIN FUNCTION
############################################################################################################
def main(args):

    # load agent data
    data_filename = args.data
    log_directory = os.path.dirname(os.path.abspath(__file__))+'/'
    data = np.load(log_directory+data_filename, allow_pickle=True, encoding='latin1')
    n_frames = len(data[1])

    #------------------------------------------------------------------------------------------------------
    # Velocity Plotter
    #------------------------------------------------------------------------------------------------------
    # instantiate figure and axes object
    fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, 1)
        # v_opt
    v_opt = data[4]
    v_opt_linear = [v_opt[i][1][0] for i in range(len(v_opt))]
    v_opt_angular = [v_opt[i][1][1] for i in range(len(v_opt))]
    v_opt_point_linear = [v_opt[i][0][0] for i in range(len(v_opt))]
    v_opt_point_angular = [v_opt[i][0][1] for i in range(len(v_opt))]

        # v_actual
    v_actual_linear = data[8][0]
    v_actual_angular = data[9][0]

        # agent.theta
    agent_theta = data[3][0]
    
        # compute the limits
    count = len(v_opt_linear)
    limits = data_limits(count)

        # compute accelerations using discrete approach
    acc_actual_linear = compute_accelerations(v_actual_linear)
    acc_opt_linear = compute_accelerations(v_opt_linear)
    acc_actual_angular = compute_accelerations(v_actual_angular)
    acc_opt_angular = compute_accelerations(v_opt_angular)

        # robot linear velocity (actual vs optimal)
    ax1.plot(v_opt_linear, label='Optimal')
    ax1.plot(v_actual_linear, label='Current')
    ax1.plot(limits[0], linestyle='--', color='k')
    ax1.plot(limits[1], linestyle='--', color='k')

        # robot angular velocity (actual vs optimal)
    ax2.plot(v_opt_angular, label='Optimal')
    ax2.plot(v_actual_angular, label='Current')
    ax2.plot(limits[2], linestyle='--', color='k')
    ax2.plot(limits[3], linestyle='--', color='k')


        # robot linear acceleration
    ax3.plot(acc_opt_linear, label='Optimal')
    ax3.plot(acc_actual_linear, label='Current')
    ax3.plot(limits[4], linestyle='--', color='k')
    ax3.plot(limits[5], linestyle='--', color='k')

        # robot angular acceleration
    ax4.plot(acc_opt_angular, label='Optimal')
    ax4.plot(acc_actual_angular, label='Current')
    ax4.plot(limits[6], linestyle='--', color='k')
    ax4.plot(limits[7], linestyle='--', color='k')

    # ax3.plot(v_opt_point_linear, label='x')
    # ax3.plot(v_opt_point_angular, label='y')

    # ax4.plot(agent_theta, label='theta')
    
    # set labels
    ax1.set_ylabel('Velocity (m/s)')
    ax2.set_ylabel('Velocity (rad/s)')
    ax3.set_ylabel('Acceleration (m/s^2)')
    ax4.set_ylabel('Acceleration (rad/s^2)')

    # set title
    ax1.title.set_text('Linear Velocity')
    ax2.title.set_text('Angular Velocity')
    ax3.title.set_text('Linear Acceleration')
    ax4.title.set_text('Angular Acceleration')

    # set legends
    ax1.legend(fontsize=8)
    ax2.legend(fontsize=8)
    ax3.legend(fontsize=8)
    ax4.legend(fontsize=8)

    #------------------------------------------------------------------------------------------------------
    # Trajectory Plotter
    #------------------------------------------------------------------------------------------------------
    plt.figure()

    # get data
    agent_x = data[1][0][:]
    agent_y = data[2][0][:]

    # call plotter
    plt.plot(agent_x, agent_y)
    plt.plot(agent_x[0], agent_y[0],'go', label='Origin')
    plt.plot(agent_x[-1], agent_y[-1],'ro', label='Goal')

    # set limits & axis
    plt.xlim(-12.93, 0.22)
    plt.ylim(-9.62, 8.5)
    plt.axis('equal')

    # set title
    plt.title("Trajectory of Robot Motion")

    # show legend
    plt.legend()

    # plt.show()

    #------------------------------------------------------------------------------------------------------
    # Angle Plotter
    #------------------------------------------------------------------------------------------------------
    plt.figure()

    plt.plot(agent_theta, label='theta')

    # show legend
    plt.legend()

    plt.show()

    

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plotter")
    parser.add_argument('--data', default='data_approach_human_[331_146].npy', help='logged data filename')
                
    args = parser.parse_args()

    main(args)