#!/usr/bin/env python3

from matplotlib import transforms
import numpy as np
import math
import os
import argparse
import matplotlib.pyplot as plt


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
    n_frames = len(data[0])

    #------------------------------------------------------------------------------------------------------
    # Velocity Plotter
    #------------------------------------------------------------------------------------------------------
    # instantiate figure and axes object
    fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, 1)
        # v_opt
    v_opt = data[5]
    v_opt_point_x = [v_opt[i][0][0] for i in range(len(v_opt))]
    v_opt_point_y = [v_opt[i][0][1] for i in range(len(v_opt))]
    # v_opt_linear_constrained = [v_opt[i][1][0] for i in range(len(v_opt))]
    # v_opt_angular_constrained = [v_opt[i][1][1] for i in range(len(v_opt))]
    v_opt_linear = [v_opt[i][1][0] for i in range(len(v_opt))]
    v_opt_angular = [v_opt[i][1][1] for i in range(len(v_opt))]

        # v_actual
    v_actual_linear = data[3][0]
    v_actual_angular = data[4][0]

        # agent.theta
    agent_theta = data[2][0]

        # v_actual_point
    v_actual_point = [DD2point_velocity([v_actual_linear[i], v_actual_angular[i]],
                        agent_theta[i]) for i in range(len(v_opt))]
    
        # v_opt_constrained_dd
    # v_opt_constrained_point = [DD2point_velocity([v_opt_linear_constrained[i], v_opt_angular_constrained[i]],
    #                            agent_theta[i]) for i in range(len(v_opt))]

        # time_delta
    time_delta = np.diff(data[14])

        # heading delta
    heading_delta = data[8]

        # accelerations
    # acc_opt_linear = [data[10][i][0] for i in range(len(data[10]))]
    # acc_opt_angular = [data[10][i][1] for i in range(len(data[10]))]
    # acc_opt_linear_constrained = [data[11][i][0] for i in range(len(data[11]))]
    # acc_opt_angular_constrained = [data[11][i][1] for i in range(len(data[11]))]
    # acc_actual_linear = [data[12][i][0] for i in range(1, len(data[12]))]
    # acc_actual_angular = [data[12][i][1] for i in range(1, len(data[12]))]

        # compute the limits
    count = len(v_opt_linear)
    limits = data_limits(count)

        # compute accelerations using discrete approach
    acc_actual_linear = compute_accelerations(v_actual_linear, time_delta)
    acc_opt_linear = compute_accelerations(v_opt_linear, time_delta)
    # acc_opt_linear_constrained = compute_accelerations(v_opt_linear_constrained, time_delta)
    acc_actual_angular = compute_accelerations(v_actual_angular, time_delta)
    acc_opt_angular = compute_accelerations(v_opt_angular, time_delta)
    # acc_opt_angular_constrained = compute_accelerations(v_opt_angular_constrained, time_delta)
    

        # robot linear velocity (actual vs optimal)
    ax1.plot(v_opt_linear, label='Optimal')
    # ax1.plot(v_opt_linear_constrained, label='Optimal_Constrained')
    ax1.plot(v_actual_linear, label='Current')
    ax1.plot(limits[0][0], linestyle='--', color='k')
    ax1.plot(limits[1][0], linestyle='--', color='k')

        # robot angular velocity (actual vs optimal)
    ax2.plot(v_opt_angular, label='Optimal')
    # ax2.plot(v_opt_angular_constrained, label='Optimal_Constrained')
    ax2.plot(v_actual_angular, label='Current')
    ax2.plot(limits[2][0], linestyle='--', color='k')
    ax2.plot(limits[3][0], linestyle='--', color='k')


        # robot linear acceleration
    ax3.plot(acc_opt_linear, label='Optimal')
    # ax3.plot(acc_opt_linear_constrained[0:count-1], label='Optimal_Constrained')
    ax3.plot(acc_actual_linear, label='Current')
    ax3.plot(limits[4][0], linestyle='--', color='k')
    ax3.plot(limits[5][0], linestyle='--', color='k')

    # values = np.arange(0, count-1)
    # ax3.scatter(values, acc_opt_linear, label='Optimal')
    # ax3.scatter(values, acc_opt_linear_constrained[0:count-1], label='Optimal_Constrained')
    # ax3.scatter(values, acc_actual_linear, label='Current')
    # ax3.plot(limits[4][0], linestyle='--', color='k')
    # ax3.plot(limits[5][0], linestyle='--', color='k')

        # robot angular acceleration
    ax4.plot(acc_opt_angular, label='Optimal')
    # ax4.plot(acc_opt_angular_constrained[0:count-1], label='Optimal_Constrained')
    ax4.plot(acc_actual_angular, label='Current')
    ax4.plot(limits[6][0], linestyle='--', color='k')
    ax4.plot(limits[7][0], linestyle='--', color='k')

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
    agent_x, agent_y = pose_transform(data[0][0][:], data[1][0][:])
    num_pedestrians = len(data[0])-1
    actors_x = data[0][1:]
    actors_y = data[1][1:]

    # call plotter
    plt.plot(agent_x, agent_y)
    plt.plot(agent_x[0], agent_y[0],'bo', label='Origin')
    plt.plot(agent_x[-1], agent_y[-1],'ro', label='End')
    plt.plot(-6.48, 7.22,'go', label='Goal')

    for i in range(num_pedestrians):
        actor_x, actor_y = pose_transform(actors_x[i], actors_y[i])
        plt.plot(actor_x[2:], actor_y[2:], label='actor'+str(i))



    # set limits & axis
    plt.xlim(-12.93, 0.22)
    plt.ylim(-9.62, 8.5)
    plt.axis('equal')

    # set title
    plt.title("Trajectory of Robot Motion")

    # show legend
    plt.legend()

    plt.show()



if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plotter")
    parser.add_argument('--data', default='logs/test_learning_crossing_dynamic-01_layout-01_case1_cautious_MC_[32_1641].npy', help='logged data filename')
                
    args = parser.parse_args()

    main(args)