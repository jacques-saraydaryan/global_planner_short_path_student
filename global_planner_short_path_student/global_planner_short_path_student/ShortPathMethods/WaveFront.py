__author__ = 'Jacques saraydaryan'

from global_planner_short_path_student.ShortPathMethods.AbstractShortPath import AbstractShortPath
# import sys
# sys.path.append('../')


import time
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point, PoseStamped, PointStamped

import numpy as np
import heapq
from queue import Queue, LifoQueue, PriorityQueue
import rclpy
import time


# http://www.bogotobogo.com/python/python_PriorityQueue_heapq_Data_Structure.php


class WaveFront(AbstractShortPath):
    SLEEP_TIME_BEFORE_NEXT_ITERATION = 0.01

    def __init__(self):
        print('')

    def goto(self, source, target, matrix, pub_marker, marker_array):
        # In wavefront computation start from the target
        start = {'x': target['x'], 'y': target['y']}

        # FIFO that store data to process
        frontier = Queue()
        # Initialized with the first point
        frontier.put(start)
        # Dictionary that holds local precedence of each point
        came_from = {}
        came_from[str(start['x']) + '_' + str(start['y'])] = None

        it = 0

        # While their is no data to process, another condition could be while goal is not reached
        while not frontier.empty():
            # get the point of the FIFO and remove it from the frontier
            current = frontier.get()
            # create visual info
            self.createClosedMarkerPt(current, marker_array)

            # for all neighbors of the current point
            for next in self.getNeighbors(current, matrix):
                # counter of number of iteration, only to see computation size
                it = it + 1

                # check that the current Neighbor has not be processed
                if str(next['x']) + '_' + str(next['y']) not in came_from:
                    # create visual info
                    self.createFontierUnitMarkerPt(next, marker_array)
                    # Add the Neighbor to be processed in th FIFO
                    frontier.put(next)

                    # Add the previous reference of the current Neighbor
                    came_from[str(next['x']) + '_' + str(next['y'])] = str(current['x']) + '_' + str(current['y'])

            # publish the visual markers
            pub_marker.publish(marker_array)
            #marker_array = MarkerArray()
            # wait before next iteration
            time.sleep(self.SLEEP_TIME_BEFORE_NEXT_ITERATION)
        print('end of wave front it:' + str(it))
        pub_marker.publish(marker_array)

        # return the dictionary of precedence
        return came_from

    def getNeighbors(self, currentNode, matrix):
        """ Compute Neighbors of the current point, Return the list of the point neighbors in Cfree"""
        x_c = currentNode['x']
        y_c = currentNode['y']
        neighbors = []
        self.checkAndAdd(neighbors, x_c + 1, y_c, matrix)
        self.checkAndAdd(neighbors, x_c, y_c + 1, matrix)
        self.checkAndAdd(neighbors, x_c - 1, y_c, matrix)
        self.checkAndAdd(neighbors, x_c, y_c - 1, matrix)
        return neighbors

    def checkAndAdd(self, neighbors, x, y, matrix):
        """ Check that the candidate neighbor is valid == not an obstacle, in current bound, add the nieghbor node to the node list"""
        if (x > 0 and x < len(matrix) and y > 0 and y < len(matrix[0])):
            if (matrix[y][x] != self.MAP_OBSTACLE_VALUE):
                neighbors.append({'x': x, 'y': y})
        return neighbors
