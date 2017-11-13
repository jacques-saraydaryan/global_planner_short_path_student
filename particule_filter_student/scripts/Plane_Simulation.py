#!/usr/bin/env python
# license removed for brevity

from random import WichmannHill
from zope.interface.common.interfaces import INameError

#from samba.dcerpc.smb_acl import group
#from samba.dcerpc.winbind import winbind

__author__ = 'jacques saraydaryan'

import pygame
import random
import os
import math
import numpy as np
from common.Particule import Particule
from Particule_Filter import Particule_Filter
from common.ToolBox import distance_to_obstacle,distance_to_obstacle_coord,entropy,std



class PlaneSimulation:
    BLACK = [0, 0, 0]
    WHITE = [255, 255, 255]
    RED = [255, 0, 0]
    GREEN = [0, 255, 255]
    GREEN_light = [0, 255, 72]
    GREY_light = [183, 179, 179]
    GREEN_GREEN = [0, 255, 0]
    RED = [255, 0, 0]
    BLUE=[0, 0, 255]

    OBSTACLE = 100

    height = 720
    width = 1280

    LEFT_CLICK = 1
    RIGHT_CLICK = 3

    # Set the height and width of the screen
    SIZE = [width, height]
    # SCALE=1
    SCALE = 10

    particuleFilter=""
    obs_grid=""
    entropy_min=500
    entropy_max=0
    entropylist=[]
    std_min=500
    std_max=0
    stdlist=[]

    def __init__(self):
        grid_temp = self.load_obs_matrix("/tmp/", "obstacle.npy")
        if len(grid_temp) == 0:
            self.obs_grid = [[0 for x in range(self.width / self.SCALE)] for y in range(self.height / self.SCALE)]
        else:
            self.obs_grid = grid_temp
        self.particuleFilter=Particule_Filter(self.width,self.height,self.obs_grid)

    # --------------------------------------------------------------------------------------------
    # ---------------------------------------- PYGAME LOOP ---------------------------------------
    # --------------------------------------------------------------------------------------------
    def startPyGameLoop(self):
        # Initialize the game engine
        pygame.init()

        screen = pygame.display.set_mode(self.SIZE)
        pygame.display.set_caption("Particule fiter")


        # initialize font; must be called after 'pygame.init()' to avoid 'Font not Initialized' error
        myfont = pygame.font.SysFont("monospace", 15)

        # load image
        # player_image = pygame.image.load("./img/player.png").convert()
        player_image_not_scaled = pygame.image.load("./img/plane_nptrans.png").convert()
        player_image = pygame.transform.scale(player_image_not_scaled, (self.width / 10, self.height / 10))

        clock = pygame.time.Clock()
        clicked_zone = []
        obstacles_list=[]


        plane_pose = {'x': 0, 'y': Particule_Filter.FIXED_PLANE_Y + 50}

        # Loop until the user clicks the close button.
        done = False
        isButtonPressed = False
        is_in_pause = True
        step=0
        while not done:
            # ------------------------------- MOUSE EVT MNG -----------------------------------
            for event in pygame.event.get():  # User did something
                # get event
                if event.type == pygame.QUIT:
                    done = True
                    print 'end'
                # User pressed down on a key
                elif event.type == pygame.KEYDOWN:
                    print "down"
                    if event.key == pygame.K_s:
                        print 'key s pressed'
                        self.save_obs_matrix("/tmp/", "obstacle.npy", self.obs_grid )
                    elif event.key == pygame.K_SPACE:
                        if is_in_pause:
                            is_in_pause = False
                        else:
                            is_in_pause = True
                    elif event.key == pygame.K_r:
                        plane_pose['x'] = 0
                        self.particuleFilter.resetParticule()
                        self.entropy_min=500
                        self.entropy_max=0
                        self.entropylist=[]
                        self.stdlist=[]
                        self.std_min=500
                        self.std_max=0
                    elif event.key == pygame.K_KP_PLUS:
                        self.particuleFilter.increment += 1
                    elif event.key == pygame.K_KP_MINUS:
                        if self.particuleFilter.increment > -1:
                            self.particuleFilter.increment -= 1


                # handle MOUSEBUTTONUP
                elif event.type == pygame.MOUSEBUTTONUP:
                    clicked_pos = pygame.mouse.get_pos()
                    clicked_zone.append(clicked_pos)
                    obstacles_list.append(clicked_pos)
                    isButtonPressed = False
                    current_click = 0

                elif event.type == pygame.MOUSEBUTTONDOWN:
                    isButtonPressed = True
                    current_click = event.button
                    print 'click type' + str(event.button)

                if isButtonPressed:
                    clicked_pos = pygame.mouse.get_pos()

                    if self.width / self.SCALE > clicked_pos[0] / self.SCALE and self.height / self.SCALE > clicked_pos[
                        1] / self.SCALE:
                        if current_click == self.LEFT_CLICK:
                            self.obs_grid[int(round(clicked_pos[1] / self.SCALE))][
                                int(round(clicked_pos[0] / self.SCALE))] = 100
                            print 'obs--:' + str(clicked_pos[1]) + ',' + str(clicked_pos[0])
                        elif current_click == self.RIGHT_CLICK:
                            self.obs_grid[int(round(clicked_pos[1] / self.SCALE))][
                                int(round(clicked_pos[0] / self.SCALE))] = 0
                            # Process each obstacles in the list
            # for i in range(len(obstacles_list)):
            #     pygame.draw.rect(screen, self.BLACK, [obstacles_list[i][0], obstacles_list[i][1], 10, 10])

            # Set the screen background
            screen.fill(self.WHITE)

            # for i in range(len(self.obs_grid)):
            #     for j in range(len(self.obs_grid[0])):
            #         if self.obs_grid[i][j] == self.OBSTACLE:
            #             # print 'DISPLAY OBSTACLE:'+str(i)+','+str(j)
            #             pygame.draw.rect(screen, self.RED, [i * self.SCALE, j * self.SCALE, self.SCALE, self.SCALE])
            #             # pygame.draw.circle(screen, self.RED, [i * self.SCALE, j * self.SCALE], 1)

            if not is_in_pause:




                # Game logic
                # get current mouse position
                pos = pygame.mouse.get_pos()
                x = pos[0]
                y = pos[1]

                # Copy image to screen:
                # screen.blit(player_image, [x, FIXED_PLANE_Y])
                # plane_pose['x']=x
                # plane_pose['y'] = FIXED_PLANE_Y+50

                # render text
                label = myfont.render("(" + str(plane_pose['x']) + "," + str(plane_pose['y']) + ")", 1, (0, 0, 0))
                label1 = myfont.render("(" +str(x) + "," + str(y)+ ")", 1, (0, 0, 0))
                # screen.blit(label1, (x, y))
                screen.blit(label, (0, 0))

                screen.blit(player_image, [plane_pose['x']-50, plane_pose['y'] - 50])
                # plane_pose['x']=x
                # plane_pose['y'] = FIXED_PLANE_Y+50
                pygame.draw.circle(screen, self.GREEN, [plane_pose['x'], plane_pose['y']], 10)
                

                for i in range(len(clicked_zone)):
                    # render text
                    clicklab = myfont.render("(" + str(clicked_zone[i][0]) + "," + str(clicked_zone[i][1]) + ")", 1,
                                             (255, 255, 0))
                    # screen.blit(label, (x, y))
                    # screen.blit(clicklab, (clicked_zone[i][0], clicked_zone[i][1]))

                # Process each particule in the list
                for i in range(len(self.particuleFilter.particule_list)):
                    # FIXME remove *10 if particule point not in grid
                    # pygame.draw.circle(screen, self.GREEN_light, [self.particuleFilter.particule_list[i].x*10, self.particuleFilter.particule_list[i].y*10],
                    #                    int(round(100 * self.particuleFilter.particule_list[i].proba * len(self.particuleFilter.particule_list) / 2)))
                    pygame.draw.circle(screen, self.GREEN_light, [self.particuleFilter.particule_list[i].x,
                                                              self.particuleFilter.particule_list[i].y],
                                   int(round(100 * self.particuleFilter.particule_list[i].proba * len(
                                       self.particuleFilter.particule_list) / 10)))

                # ----------------------------------------------------------------------------------------------------------------
                # ----------------------------------------- COMPUTED PARTICULE FILTER ---------------------------------------------
                # ----------------------------------------------------------------------------------------------------------------

                self.particuleFilter.updateParticule(plane_pose)

                plane_pose['x'] = plane_pose['x'] + 1 + self.particuleFilter.increment
                

                #entropy_value=entropy(self.particuleFilter.particule_list)
                #self.entropylist.append(entropy_value)
                #if self.entropy_min > entropy_value:
                #    self.entropy_min=entropy_value
                #if self.entropy_max < entropy_value:
                #    self.entropy_max=entropy_value
                # Display entropy history
                #entropy_cum=0
                #for i in range (0, len(self.entropylist)):
                #    entropy_cum=self.entropylist[i]+entropy_cum
                #    pygame.draw.circle(screen, self.GREY_light, (i,int(round((self.entropylist[i]-1)*20))), 2)

                std_value=std(self.particuleFilter.particule_list)
                self.stdlist.append(std_value)
                if self.std_min > std_value:
                    self.std_min=std_value
                if self.std_max < std_value:
                    self.std_max=std_value
                # Display entropy history
                std_cum=0
                for i in range (0, len(self.stdlist)):
                    if math.isnan(self.stdlist[i]):
                        break
                    std_cum=self.stdlist[i]+std_cum
                    pygame.draw.circle(screen, self.GREY_light, (i,int(round((self.stdlist[i]-1)*10))+25), 2)

                #Display line min max avg
                #pygame.draw.line(screen, self.GREEN_GREEN, (0,int(round((self.entropy_min-1)*20))) ,(self.width,int(round((self.entropy_min-1)*20))), 1)
                #pygame.draw.line(screen, self.RED, (0,int(round((self.entropy_max-1)*20))) ,(self.width,int(round((self.entropy_max-1)*20))), 1)
                #pygame.draw.line(screen, self.BLUE, (0,int(round((entropy_cum/len(self.entropylist)-1)*20))) ,(self.width,int(round((entropy_cum/len(self.entropylist)-1)*20))), 1)
                pygame.draw.line(screen, self.GREEN_GREEN, (0,int(round((self.std_min-1)*10))+25) ,(self.width,int(round((self.std_min-1)*10))+25), 1)
                pygame.draw.line(screen, self.RED, (0,int(round((self.std_max-1)*10))+25) ,(self.width,int(round((self.std_max-1)*10))+25), 1)
                pygame.draw.line(screen, self.BLUE, (0,int(round((std_cum/len(self.stdlist)-1)*10))+25) ,(self.width,int(round((std_cum/len(self.stdlist)-1)*10))+25), 1)

                try:
                    distance_to_plane,cell_x,cell_y=distance_to_obstacle_coord(plane_pose['x'],plane_pose['y'],self.obs_grid,self.width,self.height,self.SCALE)
                    label_distance = myfont.render("%.2f" % (distance_to_plane), 1, (0, 0, 0))
                    screen.blit(label_distance, (plane_pose['x'], plane_pose['y']+int(round(abs(plane_pose['y']-cell_y)/2))))
                    pygame.draw.line(screen, self.GREEN, (plane_pose['x'], plane_pose['y']),(cell_x,cell_y), 1)
                except TypeError:
                    print("no value available for display")
                
                #label_entropy = myfont.render("ENTROPY:%.2f, MIN: %.2f, MAX: %.2f, AVG: %.2f" % (entropy_value,self.entropy_min,self.entropy_max,entropy_cum/len(self.entropylist)), 1, (0, 0, 0))
                #screen.blit(label_entropy, (100, 0))

                label_std = myfont.render("STD DEVIATION:%.2f, MIN: %.2f, MAX: %.2f, AVG: %.2f" % (std_value,self.std_min,self.std_max,std_cum/len(self.stdlist)), 1, (0, 0, 0))
                screen.blit(label_std, (100, 0))


                # ----------------------------------------------------------------------------------------------------------------
                # ----------------------------------------------------------------------------------------------------------------

                # Process each obstacles in the list
                # for i in range(len(obstacles_list)):
                #     pygame.draw.rect(screen, self.BLACK, [obstacles_list[i][0], obstacles_list[i][1], 10, 10])

                for y_i in range(len(self.obs_grid)):
                    for x_j in range(len(self.obs_grid[0])):
                        if self.obs_grid[y_i][x_j] == self.OBSTACLE:
                            distance_to_line=distance_to_obstacle(x_j * self.SCALE,plane_pose['y'],self.obs_grid,self.width,self.height,self.SCALE)
                            if(distance_to_plane ==distance_to_line):
                                pygame.draw.rect(screen, self.GREEN,
                                                 [x_j * self.SCALE,y_i * self.SCALE , self.SCALE, self.SCALE])
                            else:
                                pygame.draw.rect(screen, self.RED, [x_j * self.SCALE, y_i * self.SCALE, self.SCALE, self.SCALE])
            
            step=step+1
            # Go ahead and update the screen with what we've drawn.
            pygame.display.flip()
            # clock.tick(20)
            clock.tick(40)

        # Be IDLE friendly. If you forget this line, the program will 'hang'
        # on exit.
        pygame.quit()





    def adjustFileName(self, file_path, file_name_prefix, file_name_sufix):
        if (os.path.isfile(file_path + '0-' + file_name_prefix + file_name_sufix)):
            index = 1
            while os.path.isfile(file_path + str(index) + '-' + file_name_prefix + file_name_sufix):
                index = index + 1
            return file_path + str(index) + '-' + file_name_prefix
        else:
            return file_path + '0-' + file_name_prefix

    def save_obs_matrix(self,filePath, fileName, matrix):
        # file=adjustFileName(filePath,fileName,matrix,'obs.data.npy')
        file = filePath + fileName
        np.save(file, matrix)

    def load_obs_matrix(self,filePath, fileName):
        if os.path.isfile(filePath + fileName):
            # return np.load(filePath + fileName)
            return np.asarray(np.load(filePath + fileName))
        else:
            return []


# COMMAND TYPE
# S key save the current obstacle map to /tmp/obstacle.npy
# R key reset the plane position and particule filter
# + increase plane speed
# - decrease plane speed
# SPACE pause/resume
if __name__ == '__main__':
    pl_simulator=PlaneSimulation()
    pl_simulator.startPyGameLoop()






