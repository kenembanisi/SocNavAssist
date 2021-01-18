#!/usr/bin/env python3

import pygame
import numpy as np
import math
import os

# initialize pygame
pygame.init()

# set the display window dimensions
SCREEN_HEIGHT = 800
SCREEN_WIDTH = 600

# set pygame screen
screen = pygame.display.set_mode((SCREEN_WIDTH,SCREEN_HEIGHT)) # set width and height
pygame.display.set_caption("Simulation") # set the window caption
run = True

# load agent data
log_directory = os.path.dirname(os.path.abspath(__file__))+'/logs/'
data = np.load(log_directory+"trina2_[118_1647]" + ".npy", 
                allow_pickle=True, encoding='latin1') # included encoding to allow loading py2 generated pickled files in py3
n_frames = len(data[1])


# Define circle obstacle class
class obstacleCircle():
    def __init__(self, x=175,y=900, radius=0):
        self.type = 'circ'
        self.x = x
        self.y = y
        self.radius = radius
        self.max_vel = [75, 75]
        self.motion_mode = 'linear'
        self.V_pref = [2, 0]
        self.origin = 0
        self.color = (169, 169, 169)

    def draw(self, screen, x, y):
        # animate the obstacle
        pygame.draw.circle(screen, self.color, (round(x), round(y)), self.radius, 0) # 3 is for width, I think

        # draw origin point
        pygame.draw.circle(screen, (0, 0, 0), (int(round(self.x)), int(round(self.y))), 2, 0)



# transforms from Gazebo coordinates (in meters) to pixels
def transform_x(x):
    x_pixel = ((10 + x)/20) * SCREEN_WIDTH
    return x_pixel

# transforms from Gazebo coordinates (in meters) to pixels
def transform_y(y):
    y_pixel = SCREEN_HEIGHT - ((12 + y)/24) * SCREEN_HEIGHT
    return y_pixel


# define data drawing function
def draw_window(idx):

    # set the background color to white
    screen.fill((249,250,248)) 
    
    # set agent
    agent_x = transform_x(data[0][idx])
    agent_y = transform_y(data[1][idx])

    # set obstacles
        # 1
    obs_radius = 35 * 1.68
    color = (150,150,140)
    obs1_x = transform_x(-0.74); obs1_y = transform_y(-5.3)
    pygame.draw.circle(screen, color, (round(obs1_x), round(obs1_y)), round(obs_radius))

        # 2
    obs_radius = 35 * 1.68
    color = (150,150,140)
    obs2_x = transform_x(4.74); obs2_y = transform_y(4.39)
    pygame.draw.circle(screen, color, (round(obs2_x), round(obs2_y)), round(obs_radius))

    # draw the rays of suitable headings
    show_rays = True
    scaling = 40
    if show_rays:
        # if D > 0: # if so, move the origin position
        #     augmented_pos = self.augment_position(self.D)
        #     for i in range(len(self.V_suitable)):
        #         pygame.draw.line(screen, (100,100,230), (augmented_pos[0], augmented_pos[1]), 
        #                         (augmented_pos[0] + self.V_suitable[i][0]*scaling, augmented_pos[1] + self.V_suitable[i][1]*scaling))
        # else:
        for i in range(len(data[6][0][idx])):
            pygame.draw.line(screen, (100,100,230), (agent_x, agent_y), (agent_x + data[6][0][idx][i][0]*scaling, 
                            agent_y - data[6][0][idx][i][1]*scaling))


    # draw agent
    radius = 10
    color = (20,40,240)
    pygame.draw.circle(screen, color, (round(agent_x), round(agent_y)), radius)


    # draw agent heading
    pygame.draw.line(screen, (255,0,0), (agent_x, agent_y), (agent_x + np.cos(data[2][idx])*scaling, 
                            agent_y - np.sin(data[2][idx])*scaling), 2)

    # win.blit(rot_image, origin)
    pygame.display.update()


# control loop
idx = 0 # initialize counter
while run:
    pygame.time.delay(25) # This will delay the game the given amount of milliseconds.

    for event in pygame.event.get():  # This will loop through a list of any keyboard or mouse events.
        if event.type == pygame.QUIT: # Checks if the red button in the corner of the window is clicked
            run = False  # Ends the game loop

    keys = pygame.key.get_pressed()

    if keys[pygame.K_SPACE]:
        run = False  # Ends the game loop

    if keys[pygame.K_RIGHT]:
        idx = idx + 1

        if idx < n_frames-1:
            # call window drawing function
            draw_window(idx)

    if keys[pygame.K_LEFT]:
        idx = idx - 1

        if idx < n_frames-1:
            # call window drawing function
            draw_window(idx)

    else:
        # call window drawing function
        draw_window(idx)
    
    # # increment counter
    # idx = idx + 1

    # # check run condition
    # if idx < n_frames-1:
    #     # run = False

    #     # call window drawing function
    #     draw_window(idx)

# If we exit the loop this will execute and close our game
pygame.quit()