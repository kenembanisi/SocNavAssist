# !/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
from matplotlib.collections import PatchCollection
from matplotlib.animation import FuncAnimation
from scipy import interpolate, spatial
from random import random
import argparse
import os


def set_static_features(args):
    if args.stage == 1:
        static_features = [['rect', (-1.85, -2), 3.85, 0.15], # bottom horizontal
                           ['rect', (-2, -2), 0.15, 3.85], # left vertical
                           ['rect', (-2, 1.85), 3.85, 0.15], # top horizontal
                           ['rect', (1.85, 2), 0.15, -3.85]] # right vertical

    if args.stage == 2:
        static_features = [['rect', (-1.85, -2), 3.85, 0.15], # bottom horizontal
                           ['rect', (-2, -2), 0.15, 3.85], # left vertical
                           ['rect', (-2, 1.85), 3.85, 0.15], # top horizontal
                           ['rect', (1.85, 2), 0.15, -3.85], # right vertical
                           ['circ', (-0.6, -0.6), 0.15],
                           ['circ', (0.6, -0.6), 0.15],
                           ['circ', (-0.6, 0.6), 0.15],
                           ['circ', (0.6, 0.6), 0.15]]
    
    if args.stage == 3:
        static_features = [['rect', (-1.85, -2), 3.85, 0.15], # bottom horizontal
                           ['rect', (-2, -2), 0.15, 3.85], # left vertical
                           ['rect', (-2, 1.85), 3.85, 0.15], # top horizontal
                           ['rect', (1.85, 2), 0.15, -3.85], # right vertical
                           ['circ', (-0.6, -0.6), 0.15],
                           ['circ', (0.6, -0.6), 0.15],
                           ['circ', (-0.6, 0.6), 0.15],
                           ['circ', (0.6, 0.6), 0.15]]
    if args.stage == 4:
        static_features = [['rect', (-2.35, -2.5), 4.85, 0.15], # bottom horizontal
                           ['rect', (-2.5, -2.5), 0.15, 4.85], # left vertical
                           ['rect', (-2.5, 2.35), 4.85, 0.15], # top horizontal
                           ['rect', (2.35, 2.5), 0.15, -4.85], # right vertical
                           ['rect', (1.2, 2.4), 0.15, -1.0],
                           ['rect', (1.4, 0.4), 1.00, 0.15],
                           ['rect', (1.1, -0.52), 0.15, -1.0],
                           ['rect', (0.13, 0.7), 0.15, -1.0],
                           ['rect', (-0.3, -1.3), 0.15, -1.0],
                           ['rect', (-1.5, 0.6), 0.15, -1.0],
                           ['rect', (-1.55, 1.49), 1.00, 0.15],
                           ['rect', (-2.4, -1.5), 1.00, 0.15]]

    return static_features


def get_trial_goals(stage, trial):
    goals_x = []
    goals_y = []
    if stage == 1 or stage ==2 or stage == 3:
        trial_goals = [[[0.6, 0.0], [0.9, 1.2], [-0.9, 0.0], [-0.1, -0.1], [0.5, 0.1]],
                        [[0.9, 1.2], [-1.1, 0.9], [0.9, 1.2], [1.1, 0.3], [0.1, 1.0]],
                        [[-0.6, -1.2], [1.2, 0.5], [1.2, -0.6], [-0.1, -1.2], [1.2, 1.2]],
                        [[-0.5, -0.1], [-1.2, -1.1], [-0.1, -0.8], [0.8, 1.1], [-0.3, 1.2]],
                        [[1.2, 1.0], [-0.6, 0.0], [1.2, -1.0], [-0.2, -1.2], [0.1, 0.7]],
                        [[-1.1, -0.7], [-0.9, 1.1], [-0.8, 1.2], [-1.2, -0.3], [1.1, -0.1]],
                        [[1.2, 0.7], [-1.2, -0.5], [1.2, -0.8], [-1.2, 0.8], [1.1, 0.7]],
                        [[-1.1, 0.9], [0.0, -1.0], [-1.2, 0.1], [-0.5, -1.2], [0.6, 1.1]],
                        [[-1.1, 0.5], [1.1, 1.0], [1.0, 0.0], [-0.9, 1.2], [0.0, -0.7]],
                        [[-0.1, 1.1], [1.1, 0.3], [-0.8, 1.2], [-1.2, -0.3], [0.0, -0.9]]]
        
        for i in range(5):
            goals_x.append(trial_goals[trial-1][i][0])
            goals_y.append(trial_goals[trial-1][i][1])

        
        # goals = trial_goals[trial]

    if stage == 4:
        goal_x_list = [0.6, 1.2, -1.2, -0.1, 0.5, -0.1 , 1.2, -0.5,  0.0, -0.2, -1.2, 0.1, 0.9]
        goal_y_list = [0,   0.5,  0.3,  1.1, 0.1, -1.2 , 1.2, -0.1, -1.0, -0.3, -1,  -1.6, 1.2]

        trial_index = [[ 3,  1, 10,  8,  6],
                        [ 7, 10,  6,  0, 10],
                        [ 1,  2,  4,  6,  4],
                        [ 0,  5,  6, 11,  4],
                        [ 0,  7,  6, 11,  6],
                        [11,  8,  4,  6, 10],
                        [ 9,  9,  0,  7,  7],
                        [ 2,  0,  9,  5,  7],
                        [ 2,  3,  1,  9,  6],
                        [ 5,  3,  2, 10,  0]]
        for i in range(4):
            goals_x.append(goal_x_list[trial_index[trial-1][i]])
            goals_y.append(goal_y_list[trial_index[trial-1][i]])
            # goals.append([goal_x_list[trial_index[trial][i]], goal_x_list[trial_index[trial][i]]])

    return goals_x, goals_y


def main(args):
    # stage = args.stage
    # trial_no = args.trial
    agent = [1]

    # instantiate figure and axes object
    fig, axes = plt.subplots()
    # mng = plt.get_current_fig_manager()
    # mng.window.showMaximized()

    # set plot parameters
    # if stage != 4:
    #     axes.set_xlim(-2.25, 2.25)
    #     axes.set_ylim(-2.25, 2.25)
    # else:
    #     axes.set_xlim(-2.75, 2.75)
    #     axes.set_ylim(-2.75, 2.75)
    axes.set_xlim(-12, 12)
    axes.set_ylim(-12, 12.75)
    axes.set_aspect('equal', adjustable='box')

    # define patches for obstacles
    # static_features = set_static_features(args)

    # features_list = []
    # for i in range(len(static_features)):
    #     if static_features[i][0] == 'rect':
    #         x, y = static_features[i][1][0], static_features[i][1][1]
    #         width, height = static_features[i][2], static_features[i][3]
    #         obs = Rectangle((x, y), width, height)
    #         features_list.append(obs)
    #     elif static_features[i][0] == 'circ':
    #         x, y = static_features[i][1][0], static_features[i][1][1]
    #         radius = static_features[i][2]
    #         obs = Circle((x, y), radius)
    #         features_list.append(obs)

    # # add obstacle patches to axes
    # pc_obstacle = PatchCollection(features_list, facecolor='grey')
    # axes.add_collection(pc_obstacle)
    axes.tick_params(axis='both', which='major', labelsize=18)


    # get goal points and plot goal points
    # goals_x, goals_y = get_trial_goals(stage, trial_no)
    # axes.scatter(goals_x, goals_y, s=1000, marker="s", c='w', edgecolors='#000000', linewidths=2.4)
    # for i in range(len(goals_y)):
    #     axes.text(goals_x[i], goals_y[i], r'%d' %(i+1,), fontsize=15)

    # load data
    log_directory = os.path.dirname(os.path.abspath(__file__))+'/logs/'
    # log_directory = '/home/kenembanisi/Downloads/'+'logs/'

    # dqn_agent = np.load(log_directory+"trina2_stage" + ".npy", allow_pickle=True)
    # ddpg_agent = np.load(log_directory+"DDPG/stage"+str(stage) + "/turtlebot3_burger" + "_stage_" + str(stage) + "_ddpg_" + str(trial_no) + ".npy", allow_pickle=True)
    # mb_agent = np.load(log_directory+"Move_base/stage"+str(stage) + "/turtlebot3_burger" + "_stage_" + str(stage) + "_move_base_" + str(trial_no) + ".npy", allow_pickle=True)

    n_methods = len(agent)
    color = {"DQN":'#003399', "DDPG":'#00802b', "Move_Base":'#990000'}
    

    agent_data, agent_list, n_frames = [], [], []
    # if "dqn" in agent:
    #     data = np.load(log_directory+"DQN/stage"+str(stage) + "/turtlebot3_burger" + \
    #             "_stage_" + str(stage) + "_dqn_" + str(trial_no) + ".npy", allow_pickle=True)
    #     agent_data.append(data)
    #     agent_list.append("DQN")
    #     n_frames.append(len(data[1]))
    
    # if "ddpg" in agent:
    #     data = np.load(log_directory+"DDPG/stage"+str(stage) + "/turtlebot3_burger" + \
    #             "_stage_" + str(stage) + "_ddpg_" + str(trial_no) + ".npy", allow_pickle=True)
    #     agent_data.append(data)
    #     agent_list.append("DDPG")
    #     n_frames.append(len(data[1]))
    
    # if "move_base" in agent:
    #     data = np.load(log_directory+"Move_base/stage"+str(stage) + "/turtlebot3_burger" + \
    #             "_stage_" + str(stage) + "_move_base_" + str(trial_no) + ".npy", allow_pickle=True)
    #     agent_data.append(data)
    #     agent_list.append("Move_Base")
    #     n_frames.append(len(data[1]))

    data = np.load(log_directory+"trina2_[115_1632]" + ".npy", allow_pickle=True)
    agent_data.append(data)

    n_agent_frames = len(data[1])

    # append to the end of shorter trajectories
    # for i in range(n_methods):
    #     if n_frames[i] < n_agent_frames:
    #         num = n_agent_frames - n_frames[i]
    #         last_x = agent_data[i][1][-1]
    #         last_y = agent_data[i][2][-1]
    #         for n in range(num):
    #             agent_data[i][1].append(last_x)
    #             agent_data[i][2].append(last_y)

    # define actor lines
    agent_body = []
    agent_traj = []
    # for a in agent_list:
    #     body, = axes.plot([], [], color=color[a], marker="o", markersize=13, alpha=1,
    #                     animated=True, label=a)
    #     traj, = axes.plot([], [], 'o', color='#ffffff', markersize=9, alpha=0.4, 
    #                     markeredgecolor=color[a], animated=True)
    #     agent_body.append(body)
    #     agent_traj.append(traj)

    body, = axes.plot([], [], marker="o", markersize=13, alpha=1,
                    animated=True)
    traj, = axes.plot([], [], 'o', color='#ffffff', markersize=9, alpha=0.4, 
                    animated=True)
    agent_body.append(body)
    agent_traj.append(traj)

    agent_x, agent_y = [[] for _ in range(n_methods)], [[] for _ in range(n_methods)]


    # if stage == 1 or stage == 2:
    #     n_actors = 0
    # elif stage == 4:
    #     n_actors = 2
    # elif stage == 3:
    #     n_actors = 0

    # if n_actors > 0:
    #     actor_, actor_data, n_actor_frames = [], [], []
    #     for i in range(n_actors):
    #         data = np.load(log_directory+"DDPG/stage"+str(stage) + "/obstacle_"+str(i+1)+ "_stage_" + str(stage) + "_ddpg_" + str(trial_no) + ".npy", allow_pickle=True)
    #         actor_.append(data)
    #         n_actor_frames.append(len(data[1]))

    #         # convert array to list
    #         actor_data.append([actor_[i][0].tolist(), actor_[i][1].tolist()])
    #         # append the actor trajectory to ensure complete plot
    #         if n_actor_frames[i] < n_agent_frames:
    #             num = n_agent_frames - n_actor_frames[i]

    #             last_x = actor_data[i][0][-1]
    #             last_y = actor_data[i][1][-1]
    #             for n in range(num):
    #                 actor_data[i][0].append(last_x)
    #                 actor_data[i][1].append(last_y)
    #                 # actor_data[i][0] = np.append(actor_data[i][0], last_x)
    #                 # np.append(actor_data[i][1], last_y)

    #     # define actor lines
    #     actor_body = []
    #     actor_traj = []
    #     for i in range(n_actors):
    #         body, = axes.plot([], [], '#000000', marker="8", markersize=20, alpha=1,
    #                         animated=True)
    #         traj, = axes.plot([], [], ':', color='grey', markersize=5, alpha=1,
    #                         animated=True)
    #         actor_body.append(body)
    #         actor_traj.append(traj)

        
    #     actor_x, actor_y = [[] for _ in range(n_actors)], [[] for _ in range(n_actors)]

    # define animation function
    def animate(i):
        # x_pos.append(agent_data[1][i+1])
        # y_pos.append(agent_data[2][i+1])
        # agent_traj.set_data(x_pos, y_pos)
        # agent_body.set_data(x_pos[-1:], y_pos[-1:])
        # if len(x_pos) > 2:
        #     dx = x_pos[-1] - x_pos[-2]
        #     dy = y_pos[-1] - y_pos[-2]
        # else:
        #     dx, dy = 0, 0
        # agent_heading = axes.arrow(x_pos[-1:][0], y_pos[-1:][0], dx, dy, width=0.03)

        # output = [agent_traj, agent_body, agent_heading]
        scaling = 2
        output = []
        for nx in range(n_methods):
            agent_x[nx].append(agent_data[nx][0][i+1])
            agent_y[nx].append(agent_data[nx][1][i+1])
            agent_traj[nx].set_data(agent_x[nx][-30:], agent_y[nx][-30:])
            agent_body[nx].set_data(agent_x[nx][-1:], agent_y[nx][-1:])
            if len(agent_x[nx]) > 2:
                dx = (agent_x[nx][-1] - agent_x[nx][-2])*10
                dy = (agent_y[nx][-1] - agent_y[nx][-2])*10
            else:
                dx, dy = 0, 0
            agent_heading = axes.arrow(agent_x[nx][-1:][0], agent_y[nx][-1:][0], dx, dy, width=0.1)

            

            for q in range(len(data[5][0][i])):
            # for q in range(50):
                v_suitable = axes.plot([agent_x[nx][-1:][0], agent_x[nx][-1:][0] + scaling * data[5][0][i][q][0]], 
                                       [agent_y[nx][-1:][0], agent_y[nx][-1:][0] + scaling * data[5][0][i][q][1]], linewidth=0.2)
                output.append(v_suitable[0])
            
            output.append(agent_traj[nx])
            output.append(agent_body[nx])
            output.append(agent_heading)

        # if n_actors > 0:
        #     for idx in range(n_actors):
        #         actor_x[idx].append(actor_data[idx][0][i+1])
        #         actor_y[idx].append(actor_data[idx][1][i+1])
        #         actor_traj[idx].set_data(actor_x[idx][-2:], actor_y[idx][-2:])
        #         actor_body[idx].set_data(actor_x[idx][-1:], actor_y[idx][-1:])
        #         output.append(actor_traj[idx])
        #         output.append(actor_body[idx])
            

        return output


    animation = FuncAnimation(fig, func=animate, frames=np.arange(0, n_agent_frames-1, 1),
                            interval=1, blit=True, repeat=False)

    # display figure
    plt.legend(fontsize=20)
    plt.show()

    # writer = PillowWriter(fps=20)
    # animation.save(directory+args.file+".gif", writer=writer)


if __name__=="__main__":

    parser = argparse.ArgumentParser(description="Visualizer")
    parser.add_argument('--stage', type=int, default=1, help='stage')
    parser.add_argument('--trial', type=int, default=1, help='trial number')
    parser.add_argument('--method', default=[], nargs='+', help='trial number')
                
    args = parser.parse_args()

    main(args)