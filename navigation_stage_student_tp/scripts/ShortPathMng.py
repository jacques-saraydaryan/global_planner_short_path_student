#!/usr/bin/env python
# license removed for brevity
__author__ = 'jacques saraydaryan'

import rospy
import time
import tf
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point, PoseStamped, PointStamped
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import Path

from ShortPathMethods.WaveFront import WaveFront
from ShortPathMethods.Dijkstra import Dijsktra
from ShortPathMethods.GreedyBestFirstSearch import GreedyBestFirstSearch
from ShortPathMethods.AStar import AStar
# from local_planner_student.srv import Path as Path_Planner
from Queue import Queue, LifoQueue

class ShortPathMng:
    mapArray = ""
    OBSTACLE_VALUE = 100
    MAP_OBSTACLE_VALUE = -100
    RESOLUTION = 8
    shortPathAlgoMap = {'WAVEFRONT': WaveFront(), 'ASTAR': AStar(), 'DIJKSTRA': Dijsktra(),
                        'GREEDY_BEST_FIRST_SEARCH': GreedyBestFirstSearch()}
    shortPathMethodeSelected = 'WAVEFRONT'
    tflistener = ""

    MAX_VALUE = 1000000

    def __init__(self, resolution, shortPathMethod, isLocalPlanner, inflate_radius):
        # init params
        self.shortPathMethodeSelected = shortPathMethod
        self.RESOLUTION = resolution
        self.isLocalPlanner = isLocalPlanner
        self.inflate_radius = inflate_radius

        # init ros node
        rospy.init_node('ShortPathMng', anonymous=True)

        # ------------------#
        # --- Subscriber ----#
        # -------------------#

        # get the current map
        rospy.Subscriber("/map", OccupancyGrid, self.mapCallback)
        # get the current goal for navigation
        rospy.Subscriber("/clicked_point", PointStamped, self.askForGoalCallback)

        # ------------------#
        # --- Publisher ----#
        # ------------------#

        # marker to display algorithm
        self.pub_marker = rospy.Publisher('process_algo', Marker, queue_size=1)
        # Send goal to navigation stack
        self.pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=100)

        self.isMapComputed = False
        self.tflistener = tf.TransformListener()

        # ------------------#
        # ---- Service -----#
        # ------------------#
        self.local_planner_service = ""
        ### TODO
        ### create the link between our navigation ros node and our local_planner
        ### self.local_planner_service: service container to call the local_planner node
        ###
        #
        #
        #
        #
        #                       TODO
        #
        #
        #
        #
        ###

    # ******************************************************************************************
    # ************************************   MAP PROCESSING   **********************************
    # ******************************************************************************************

    def mapCallback(self, data):
        self.map_width = data.info.width
        self.map_height = data.info.height
        self.resolution = data.info.resolution
        self.mapArray = [[0 for x in range(self.map_height)] for x in range(self.map_width)]

        size = self.map_width * self.map_height
        i = 0
        j = 0
        for index in range(0, size):
            current_index_y = 0;
            current_index_x = index % (self.map_width)

            if index / (self.map_width) != 0:
                current_index_y = index / self.map_width

            cellValue = 0

            if data.data[index] == self.OBSTACLE_VALUE:
                cellValue = self.MAP_OBSTACLE_VALUE

            self.mapArray[current_index_y][current_index_x] = cellValue

        # INFLATE the map according the given inflate radius
        inflate_map = self.inflate_map(self.mapArray, data.info.resolution)

        # resize map
        self.resizedMap = self.resizeWithResolution(inflate_map, self.RESOLUTION)
        print('map received and processed')
        self.isMapComputed = True

        for shortPathMetodName in self.shortPathAlgoMap.iterkeys():
            self.shortPathAlgoMap[shortPathMetodName].setMap(self.resizedMap, self.map_width, self.map_height,self.resolution,self.RESOLUTION)
            self.shortPathAlgoMap[shortPathMetodName].RESOLUTION = self.RESOLUTION

    # **************************************************
    # ***************   INFLATE MAP    *****************
    # **************************************************

    def inflate_map(self, map, map_resolution):
        ### TODO
        ### map :original map ( like a grid[][] )
        ### map_resolution :original map resolution (e.g 0.05)
        ###
        ### self.inflate_radius : radius of obstacle inflate (0.3 m)
        ### self.MAP_OBSTACLE_VALUE : value into the map of an obstacle (-100)
        #
        #
        #
        #
        #                       TODO
        #
        #
        #
        #
        ###

        return map
        ## UNCOMMENT LINE BELLOW TO TEST YOUR INFLATED MAP
        #return new_inflated_map

    def resizeWithResolution(self, map, resolution):
        marker_container = Marker()
        marker_container.id = 2
        marker_container.type = Marker.CUBE_LIST
        marker_container.points = []
        marker_container.colors = []
        marker_container.header.frame_id = "map";
        marker_container.ns = "wall";
        marker_container.scale.x = (0.5 / float(10)) * resolution;
        marker_container.scale.y = (0.5 / float(10)) * resolution;
        marker_container.header.stamp = rospy.Time.now()
        marker_container.pose.orientation.w = 1

        resizedMapArray = [[0 for x in range(self.map_height / resolution)] for x in range(self.map_width / resolution)]
        i = 0
        j = 0
        while i < len(map[0]):
            j = 0
            while j < len(map):
                if (i == 0):
                    new_i = 0
                else:
                    new_i = int(round(i / float(resolution)))

                if (j == 0):
                    new_j = 0
                else:
                    new_j = int(round(j / float(resolution)))

                # if(j>=0 and j<self.map_width/resolution and i>=0 and i<self.map_height/resolution):
                if self.isObstacle(map, i, j, resolution):
                    resizedMapArray[new_i][new_j] = self.MAP_OBSTACLE_VALUE
                    current_point = Point()
                    current_color = ColorRGBA()

                    current_color.a = 0.5;
                    current_color.r = 0;
                    current_color.g = 1;
                    current_color.b = 1;

                    current_point.z = 0.20 / float(10)
                    offset=resolution *self.resolution/float(2)

                    current_point.x = ((new_j * resolution *self.resolution))+offset
                    current_point.y = ((new_i * resolution *self.resolution))+offset

                    #current_point.x = ((new_j / float(2)) / (float(10) / resolution))
                    #current_point.y = ((new_i / float(2)) / (float(10) / resolution)) 

                    marker_container.points.append(current_point)
                    marker_container.colors.append(current_color)
                else:
                    # print 'i:'+str(i)+"--> obstacle"
                    # print 'j:'+str(j)
                    resizedMapArray[new_i][new_j] = 0
                j = j + resolution
            i = i + resolution
        self.pub_marker.publish(marker_container)
        return resizedMapArray

    def isObstacle(self, map, i, j, resolution):
        for k in range(0, resolution):
            for l in range(0, resolution):
                if (j + l >= 0 and j + l < self.map_width and i + k >= 0 and i + k < self.map_height):
                    if (map[i + k][j + l] == self.MAP_OBSTACLE_VALUE):
                        return True
        return False

    # ******************************************************************************************
    # ************************************   GOTO   ********************************************
    # ******************************************************************************************
    def askForGoalCallback(self, goal):
        robot_pos = ""

        # Get current robot position
        try:
            # if self.tflistener.waitForTransform("/base_link", "/map", rospy.Time(0), rospy.Duration(2.0)):
            t = self.tflistener.getLatestCommonTime("map", "/base_link")
            position, quaternion = self.tflistener.lookupTransform("map", "/base_link", t)

            # self.tflistener.waitForTransform("map", "base_link", rospy.Time(0),rospy.Duration(2.0))
            # transPose = self.tflistener.transformPose("base_link", "map")
            # # if self.tflistener.waitForTransform(poseStamped.header.frame_id, "/map", rospy.Time(0), rospy.Duration(1.0)) :
            # #     t = self.tflistener.getLatestCommonTime("kinect", "/base_link")
            # #     position, quaternion = self.tflistener.lookupTransform(poseStamped.header.frame_id, "/base_link", t)
            # self.tflistener.waitForTransform("map", "base_link", rospy.Time.now(),rospy.Duration(2.0))
            # transPose = self.tflistener.transformPose("base_link", "map")
            robot_pos = position
        except Exception as err:
            rospy.logwarn("no common frame between [base_link] and [%s], error message [%s]", "map", str(err))

        # find points to matrix
        robot_pos_matrix = {}
        goal_matrix = {}
        if len(str(robot_pos)) > 0:
            rospy.loginfo(" New Navigation From [" + str(robot_pos[0]) + "," + str(robot_pos[1]) + "] To [" + str(
                goal.point.x) + "," + str(goal.point.y) + "], Real positions ")

        # Fix point to matrix coord
        robot_pos_matrix['x'] = int(round(robot_pos[0] / float(self.RESOLUTION * 0.05), 0))
        robot_pos_matrix['y'] = int(round(robot_pos[1] / float(self.RESOLUTION * 0.05), 0))

        goal_matrix['x'] = int(round(goal.point.x / float(self.RESOLUTION * 0.05), 0))
        goal_matrix['y'] = int(round(goal.point.y / float(self.RESOLUTION * 0.05), 0))

        rospy.loginfo(
            " New Navigation From [" + str(robot_pos_matrix['x']) + "," + str(robot_pos_matrix['y']) + "] To [" + str(
                goal_matrix['x']) + "," + str(goal_matrix['y']) + "], Matrix based positions ")

        self.pub_marker.publish((self.createAlgoTxtMarker('Cleaning markers', -1, 5)))
        # Clear all markers FIXME take lots of time !!!
        # self.clearAllMarkers(self.pub_marker)
        self.pub_marker.publish((self.createAlgoTxtMarker(self.shortPathMethodeSelected, -1, 5)))

        # ASk to compute the shortest path from selected algorithm
        #marker_array = MarkerArray()
        marker_container =  self.shortPathAlgoMap[self.shortPathMethodeSelected]._create_marker_container()
        goalMap = self.shortPathAlgoMap[self.shortPathMethodeSelected].goto(robot_pos_matrix, goal_matrix,
                                                                            self.resizedMap, self.pub_marker,
                                                                            marker_container)
        rospy.loginfo(" Path to follow:" + str(goalMap))
        #path = self.shortPathAlgoMap[self.shortPathMethodeSelected].goto(robot_pos_matrix, goal_matrix, self.mapArray)
        if 'WAVEFRONT' == self.shortPathMethodeSelected:
            self.pushGoals(goalMap, robot_pos_matrix, marker_container, False, self.isLocalPlanner)
        else:
            self.pushGoals(goalMap, goal_matrix, marker_container, True, self.isLocalPlanner)
        self.pub_marker.publish(marker_container)

    # ******************************************************************************************
    # ******************************   CLEAR ALL MARKERs   *************************************
    # ******************************************************************************************
    def clearAllMarkers(self, publisher):
        marker_array = MarkerArray()
        for i in range(self.map_height):
            for j in range(self.map_width):
                marker = Marker()
                marker.header.frame_id = "map";
                marker.header.stamp = rospy.Time.now();
                marker.action = 3
                # marker.ns ="wave";
                # marker.id = i + 1000 * j
                marker_array.markers.append(marker)
        publisher.publish(marker_array)

    # ******************************************************************************************
    # **********************************   GOAL MARKER   ***************************************
    # ******************************************************************************************

    def createGoalMarker(self, currentgoal, marker_container, x, y):
        current_point = Point()
        current_color = ColorRGBA()

        current_color.a = 0.5
        current_color.r = 1
        current_color.g = 0
        current_color.b = 0

        current_point.x = currentgoal.pose.position.x
        current_point.y = currentgoal.pose.position.y

        marker_container.points.append(current_point)
        marker_container.colors.append(current_color)


    # ******************************************************************************************
    # **********************************   ALGO TXT MARKER   ***************************************
    # ******************************************************************************************

    def createAlgoTxtMarker(self, text, x, y):
        #marker_array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "map";
        marker.header.stamp = rospy.Time.now();
        # marker.ns ="wave";
        marker.id = 1234567
        # print 'GOAL MARKER-->'+str((currentgoal.pose.position.x+1000*currentgoal.pose.position.y)*10)
        marker.action = 0
        marker.color.a = 1
        marker.color.r = 1
        marker.color.g = 0
        marker.color.b = 0
        marker.scale.z = 1
        marker.type = marker.TEXT_VIEW_FACING
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.orientation.w = 1
        marker.text = text
        #marker_array.markers.append(marker)
        return marker

    # ******************************************************************************************
    # ************************************   GOAL MNG   ****************************************
    # ******************************************************************************************

    def pushGoals(self, mapNode, start, marker_container, isreverted, isPathOnService):
        # x=round(int(target['x'])/float(self.RESOLUTION*0.5),0)
        # y=round(int(target['y'])/float(self.RESOLUTION*0.5),0)
        revert = []
        x = start['x']
        y = start['y']
        # goalQueue=LifoQueue()
        goalQueue = Queue()
        goalLifo = LifoQueue()
        goalQueue.put(self.createGoal(x, y))
        try:
            prev = mapNode[str(int(x)) + '_' + str(int(y))]
            # FIXME TO CHECK NONE VALUE
            while prev != None:
                # x=round(int(prev.split('_')[0])/float(self.RESOLUTION*0.5),0)
                # y=round(int(prev.split('_')[1])/float(self.RESOLUTION*0.5),0)
                print('GOAL -->' + prev)
                x = int(prev.split('_')[0])
                y = int(prev.split('_')[1])
                currentgoal = self.createGoal(x, y)
                self.createGoalMarker(currentgoal, marker_container, x, y)
                # rospy.sleep(0.01)
                goalQueue.put(currentgoal)
                prev = mapNode[str(x) + '_' + str(y)]
        except KeyError as e:
            print('end reverse path')
        self.pub_marker.publish(marker_container)

        if (isreverted):
            while not goalQueue.empty():
                goalLifo.put(goalQueue.get())
            while not goalLifo.empty():
                goalQueue.put(goalLifo.get())

        if isPathOnService:

            ### TODO
            ### call here the local planner service (self.local_planner_service)
            ### goalQueue: queue of goal to acheive (Posestamped ros message)
            ###
            ### self.local_planner_service: service to call the local planner ( TODO need to be created on the ShortPathMng constructor)
            #
            #
            #
            #
            #                       TODO
            #
            #
            #
            #
            ###
            print('')
        else:
            while not goalQueue.empty():
                self.pub_goal.publish(goalQueue.get())
                rospy.sleep(2)

    def createGoal(self, x, y):
        goal = PoseStamped()
        goal.header.frame_id = "map";
        goal.header.stamp = rospy.Time.now();

        offset=self.RESOLUTION *self.resolution/float(2)

        goal.pose.position.x = ((x * self.resolution *self.RESOLUTION))+offset
        goal.pose.position.y = ((y * self.resolution *self.RESOLUTION))+offset

        #goal.pose.position.x = (x / float(2) / (float(10) / self.RESOLUTION)) + 0.2
        #goal.pose.position.y = (y / float(2) / (float(10) / self.RESOLUTION)) + 0.2
        goal.pose.orientation.x = 0
        goal.pose.orientation.y = 0
        goal.pose.orientation.z = 0.0379763283083
        goal.pose.orientation.w = 0.999278639063

        return goal

if __name__ == '__main__':
    try:

        # ------------------#
        # --- ROS PARAM ----#
        # ------------------#
        # FIXME Check private or global ros param
        # RESOLUTION = rospy.get_param('~SHORT_PATH_RESOLUTION', 8)
        RESOLUTION = rospy.get_param('~SHORT_PATH_RESOLUTION', 4)
        shortPathMethodeSelected = rospy.get_param('~SHORT_PATH_METHOD', 'GREEDY_BEST_FIRST_SEARCH')
        #shortPathMethodeSelected = rospy.get_param('~SHORT_PATH_METHOD', 'WAVEFRONT')
        isLocalPlanner = rospy.get_param('~LOCAL_PLANNER_USED', False)
        inflate_radius = rospy.get_param('~INFLATE_RADIUS', 0.3)
        print("------>Used SHORT_PATH_METHOD: " + str(shortPathMethodeSelected))

        nav = ShortPathMng(RESOLUTION, shortPathMethodeSelected, isLocalPlanner, inflate_radius)

        # keep python node alive
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
