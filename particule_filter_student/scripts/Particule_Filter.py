import random


from common.Particule import Particule
from common.ToolBox import distance_to_obstacle,update_coord_according_scale
import math

class Particule_Filter:

    NB_PARTICLES=200
    FIXED_PLANE_Y = 100
    increment = 0
    DISTANCE_ERROR = 2

    width=0
    height=0

    MOTION_PLANNER_MIN=-1
    MOTION_PLANNER_MAX=5

    SCALE_FACTOR=10

    obs_grid=[]
    particule_list=[]


    def __init__(self,width,height,obs_grid):
        self.width=width
        self.height=height
        self.obs_grid=obs_grid
        self.particule_list=self.getRandParticule(self.NB_PARTICLES,0,width,self.FIXED_PLANE_Y,self.FIXED_PLANE_Y)

    def resetParticule(self):
        self.particule_list = self.getRandParticule(self.NB_PARTICLES, 0, self.width, self.FIXED_PLANE_Y, self.FIXED_PLANE_Y)

        # ----------------------------------------------------------------------------------------------------------------
        # ----------------------------------------- COMPUTED RANDOM PARTICULES--------------------------------------------
        # ----------------------------------------------------------------------------------------------------------------
    def getRandParticule(self,nbr, start_x, max_x, start_y, max_y):
        particule_list = []
        ###################################
        ##### TODO
        ##   nbr: number fo particules
        ##   start_x: min x possible coordinate
        ##   max_x: max x possible coordinate
        ##   start_y: min y possible coordinate
        ##   max_y: max y possible coordinate
        #####
        ## Use the Particule object to fill the list particule_list
        ##

        return particule_list

        # ----------------------------------------------------------------------------------------------------------------
        # ----------------------------------- UPDATE PARTICULE ACCORDING NEX POSE-----------------------------------------
        # ----------------------------------------------------------------------------------------------------------------
    def updateParticule(self,plane_pose):
        # process particule according motion planning
        self.particule_list = self.motion_prediction()

        current_distance_to_obstacle = distance_to_obstacle(plane_pose['x'], plane_pose['y'], self.obs_grid,self.width,self.height,self.SCALE_FACTOR)

        self.weightingParticule_list( current_distance_to_obstacle)


        # ----------------------------------------------------------------------------------------------------------------
        # -------------------------------------- MOTION PREDICTION AND RESAMPLING   --------------------------------------
        # ----------------------------------------------------------------------------------------------------------------
    def motion_prediction(self):
        new_particule_list = []
        choices = {}
        for i in range(len(self.particule_list)):
            choices[self.particule_list[i].id()] = self.particule_list[i].w

            ###################################
            ##### TODO
            ##   self.particule_list: list des particules disponibles
            ##
            #####
            ## Utiliser la fonction self.weighted_random_choice(choices) return
            #  coordinate from a particule according a
            ##  roulette wheel algorithm
            #  Note weighted_random_choice retourne une string contenant
            #  le x et y de la particule choisie
            #   coord = self.weighted_random_choice(choices)
            #   x_coord = int(coord.split('_')[0])
            #   y_coord = int(coord.split('_')[1])

        return new_particule_list

        # -------------------------------------------------------
        # ----------- SELECT PARTICULE  -----------
        # -------------------------------------------------------
    def weighted_random_choice(self,choices):
        ###################################
        ##### TODO
        ##   choices: dictionary holding particule coordination as key
        ##  and weight as value
        ##  return the selected particule key
        #####
        return ""

    # ----------------------------------------------------------------------------------------------------------------
    # --------------------------------------------- EVALUATE PARTICULE (proba) ---------------------------------------
    # ----------------------------------------------------------------------------------------------------------------
    def weightingParticule_list(self,observed_distance):
        sum_weights = 0
        for i in range(len(self.particule_list)):
            # current_weight = self.weightingParticule(self.particule_list[i].x, self.particule_list[i].y, observed_distance)
            # current_weight = self.weightingParticuleBinary(self.particule_list[i].x, self.FIXED_PLANE_Y+50,
            #                                          observed_distance)
            current_weight = self.weightingParticuleExp(self.particule_list[i].x, self.FIXED_PLANE_Y+50,
                                                     observed_distance)
            self.particule_list[i].w = current_weight
            sum_weights += current_weight
        for i in range(len(self.particule_list)):
            if sum_weights != 0:
                self.particule_list[i].proba = self.particule_list[i].w / float(sum_weights)
            else:
                self.particule_list[i].proba = 0


    # -----------------------------------------------------
    #  ----------- EVALUATE PARTICULE (Weight)  -----------
    # -----------------------------------------------------
    def weightingParticule(self,p_x, p_y, observed_distance):
        ###################################
        ##### TODO
        ##   p_x: x coordinate of the particule p
        ##  p_y: y coordinate of the particule p
        ##  observed_distance: distance to the ground
        ##  measure by the probe
        ##
        ## return weight corresponding to the given particule
        ## according observation
        ##
        ## Note ue the function distance_to_obstacle to get the
        ## estimate particule to the ground distance
        return ""
