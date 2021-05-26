import pygame
import numpy as np
import math
import os
import argparse
import matplotlib.pyplot as plt


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
    # x_pixel = ((10 + x)/20) * SCREEN_WIDTH
    x_pixel = ((14 + x)/15) * SCREEN_WIDTH  # the full range of X is 15m
                                            # 14m is the zero point
    return x_pixel

# transforms from Gazebo coordinates (in meters) to pixels
def transform_y(y):
    # y_pixel = SCREEN_HEIGHT - ((12 + y)/24) * SCREEN_HEIGHT
    y_pixel = SCREEN_HEIGHT - ((11 + y)/23) * SCREEN_HEIGHT     # the full range of Y is 23m
                                                                # 11m is the zero point
    return y_pixel

# draws the time/frame
def drawTime(idx, n_frames):
    black = (0,0,0)
    my_font = pygame.font.SysFont("Times New Roman", 16)
    textlabel = my_font.render("Time Frame: ", 1, black)
    vallabel = my_font.render(str(idx)+" / "+str(n_frames), 1, black)

    screen.blit(textlabel, (20,10))
    screen.blit(vallabel, (120,10))

# define data drawing function
def draw_window(idx, data):

    # define the data
    actor_list = data[0]
    actors_x = data[1][1:]; actors_y = data[2][1:]
    actors_theta = data[3][1:]
    v_opt = data[4]
    v_suitable = data[5]
    v_admissible = data[6]
    v_desired = data[7]
    n_frames = len(data[1][0])

    # set the background color to white
    screen.fill((249,250,248)) 
    
    # set agent
    agent_x = transform_x(data[1][0][idx])
    agent_y = transform_y(data[2][0][idx])
    agent_theta = data[3][0]

    ### set walls & boundaries ------------------------------------------------------------------
    black = (0, 0, 0)
    top_left = (transform_x(-13.2), transform_y(8.5)); top_right = (transform_x(0.7), transform_y(8.5))
    bottom_left = (transform_x(-13.2), transform_y(-10.5)); bottom_right = (transform_x(0.7), transform_y(-10.5))
        
        # main hall w/walls    
    pygame.draw.line(screen, black, top_left, top_right) # top wall
    pygame.draw.line(screen, black, top_left, bottom_left) # left wall
    pygame.draw.line(screen, black, top_right, bottom_right) # right wall
    pygame.draw.line(screen, black, bottom_left, bottom_right) # bottom wall

        # corridor wall
    corr_top_left = (transform_x(-13.2), transform_y(11)); corr_top_right = (transform_x(0.7), transform_y(11))
    pygame.draw.line(screen, black, corr_top_left, corr_top_right)

        # goal position
    red = (230, 0, 0)
    goal_position = (round(transform_x(-6.5)), round(transform_y(8.2)))
    pygame.draw.circle(screen, red, goal_position, int(40*0.5))

    # -------------------------------------------------------------------------------------------
    # set active obstacles (pedestrians)
    # -------------------------------------------------------------------------------------------
    num_pedestrians = len(actor_list) - 1 # to remove the agent count
    intimate_radius = 40 * 0.45   # intimate radius
    personal_radius = 40 * 0.9   # personal radius
    color_1 = (119, 166, 131)
    color_2 = (213, 245, 221)
    for i in range(num_pedestrians):
        ped_x = actors_x[i][idx]; ped_y = actors_y[i][idx]
        ped_x_transformed = transform_x(ped_x); ped_y_transformed = transform_y(ped_y)
        pygame.draw.circle(screen, color_2, (round(ped_x_transformed), round(ped_y_transformed)), round(personal_radius))
        pygame.draw.circle(screen, color_1, (round(ped_x_transformed), round(ped_y_transformed)), round(intimate_radius))
        

        # draw velocity vector for pedestrians
        # pygame.draw.line(screen, (100,100,230), (ped_x_transformed, ped_y_transformed), 
        #                 (ped_x_transformed + data[5][0][idx][i][0]*scaling, 
        #                     agent_y - data[5][0][idx][i][1]*scaling))


    
    # -------------------------------------------------------------------------------------------
    # set the agent
    # -------------------------------------------------------------------------------------------
        # draw the rays of suitable headings & v_opt & v_desired heading
    show_rays = True
    scaling = 40
    if show_rays:
        # v_admissible rays
        for i in range(len(v_admissible[idx])):
            pygame.draw.line(screen, (150, 150, 100), (agent_x, agent_y), (agent_x + v_admissible[idx][i][0]*scaling, 
                            agent_y - v_admissible[idx][i][1]*scaling))

        # v_suitable rays
        for i in range(len(v_suitable[idx])):
            pygame.draw.line(screen, (100,100,255), (agent_x, agent_y), (agent_x + v_suitable[idx][i][0]*scaling, 
                            agent_y - v_suitable[idx][i][1]*scaling))
                            
        # v_desired heading
        pygame.draw.line(screen, (180,100,100), (agent_x, agent_y), (agent_x + v_desired[idx][0]*scaling, 
                            agent_y - v_desired[idx][1]*scaling))

        # v_opt heading
        pygame.draw.line(screen, (255,0,0), (agent_x, agent_y), (agent_x + v_opt[idx][0][0]*scaling, 
                            agent_y - v_opt[idx][0][1]*scaling), 2)

        # draw agent
    radius = 40 * 0.25  # agent radius is set to 0.25m
    color = (60,40,240)
    pygame.draw.circle(screen, color, (round(agent_x), round(agent_y)), round(radius))


    # draw agent heading
    # pygame.draw.line(screen, (0,0,0), (agent_x, agent_y), (agent_x + np.cos(data[3][0][idx])*scaling, 
    #                         agent_y - np.sin(data[3][0][idx])*scaling), 2)
    pygame.draw.line(screen, (0,0,0), (agent_x, agent_y), (agent_x + np.cos(agent_theta[idx])*scaling, 
                            agent_y - np.sin(agent_theta[idx])*scaling), 2)

    drawTime(idx, n_frames)
    
    pygame.display.update()



############################################################################################################
# MAIN FUNCTION
############################################################################################################

def main(args):

    # initialize pygame
    pygame.init()

    # set some variables to global for convenience. TODO: May need to write a class for this later
    global SCREEN_HEIGHT, SCREEN_WIDTH, screen

    # set the display window dimensions
    SCREEN_HEIGHT = 920 # previously 800
    SCREEN_WIDTH = 600

    # set pygame screen
    screen = pygame.display.set_mode((SCREEN_WIDTH,SCREEN_HEIGHT)) # set width and height
    pygame.display.set_caption("Simulation") # set the window caption
    run = True

    # load agent data
    data_filename = args.data
    # log_directory = os.path.dirname(os.path.abspath(__file__))+'/logs/'
    log_directory = os.path.dirname(os.path.abspath(__file__))+'/'
    data = np.load(log_directory+data_filename, allow_pickle=True, encoding='latin1')
    n_frames = len(data[1])


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
                draw_window(idx, data)

        if keys[pygame.K_LEFT]:
            idx = idx - 1

            if idx < n_frames-1:
                # call window drawing function
                draw_window(idx, data)

        else:
            # call window drawing function
            draw_window(idx, data)

    # If we exit the loop this will execute and close our game
    pygame.quit()




if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Visualizer")
    parser.add_argument('--data', default='test_approach_human_[413_853].npy', help='logged data filename')
                
    args = parser.parse_args()

    main(args)