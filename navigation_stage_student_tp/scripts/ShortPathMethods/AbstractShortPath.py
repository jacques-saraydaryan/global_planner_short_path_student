__author__ = 'Jacques Saraydaryan'
from abc import ABCMeta, abstractmethod
import random
import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point,PoseStamped,PointStamped


class AbstractShortPath:
    RESOLUTION = 8
    MAP_OBSTACLE_VALUE = -100


    def __init__(self):
        # FIXME need to take value from ShortPathMng
        self.RESOLUTION=8
        self.MAP_OBSTACLE_VALUE = -100


    @abstractmethod
    def goto(self, source, target,matrix,pub_marker,marker_array): pass


    # @abstractmethod
    # def robot_detection_callback(self): pass


    def setMap(self,resizedMap,map_width,map_height):
        self.resizedMap=resizedMap
        self.map_width=map_width
        self.map_height=map_height


    # ******************************************************************************************
    # ********************************   MARKER CREATION   *************************************
    # ******************************************************************************************

    def createCameFromMarker(self, v, current, markerArray):
        marker = Marker()
        marker.header.frame_id = "map";
        marker.header.stamp = rospy.Time.now();
        # marker.ns ="wave";
        marker.id = v['y'] + 1000 * v['x']
        start = Point()
        start.x = v['x']
        start.y = v['y']

        end = Point()
        end.x = current['x']
        end.y = current['y']
        marker.action = 0
        marker.color.a = 0.5;
        marker.color.r = 0;
        marker.color.g = 0;
        marker.color.b = 1;
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.20
        marker.type = marker.CYLINDER
        # marker.pose.position.x = v['x']/float(2)
        # marker.pose.position.y = v['y']/float(2)
        markerArray.markers.append(marker)

    def createFontierMarker(self, currentNeighbors, markerArray):
        for v in currentNeighbors:
            marker = Marker()
            marker.header.frame_id = "map";
            marker.header.stamp = rospy.Time.now();
            # marker.ns ="wave";
            marker.id = v['y'] + 1000 * v['x']
            marker.action = 0
            marker.color.a = 0.5
            marker.color.r = 0
            marker.color.g = 1
            marker.color.b = 0
            marker.scale.x = (0.5 / float(10)) * self.RESOLUTION
            marker.scale.y = (0.5 / float(10)) * self.RESOLUTION
            marker.scale.z = 0.20 / float(10)
            marker.type = marker.CYLINDER
            marker.pose.position.x = (v['x'] / float(2) / (float(10) / self.RESOLUTION)) + 0.2
            marker.pose.position.y = (v['y'] / float(2) / (float(10) / self.RESOLUTION)) + 0.2
            marker.pose.orientation.w = 1

            markerArray.markers.append(marker)

    def createGoalMarker(self, currentgoal, markerArray):
        marker = Marker()
        marker.header.frame_id = "map";
        marker.header.stamp = rospy.Time.now();
        # marker.ns ="wave";
        marker.id = (currentgoal.pose.position.x + 1000 * currentgoal.pose.position.y) * 10
        print 'GOAL MARKER-->' + str((currentgoal.pose.position.x + 1000 * currentgoal.pose.position.y) * 10)
        marker.action = 0
        marker.color.a = 0.5
        marker.color.r = 1
        marker.color.g = 0
        marker.color.b = 0
        marker.scale.x = (0.5 / float(10)) * self.RESOLUTION
        marker.scale.y = (0.5 / float(10)) * self.RESOLUTION
        marker.scale.z = 0.50 / float(10)
        marker.type = marker.CUBE
        marker.pose.position.x = currentgoal.pose.position.x
        marker.pose.position.y = currentgoal.pose.position.y
        marker.pose.orientation.w = 1

        markerArray.markers.append(marker)

    def createClosedMarker(self, current, markerArray):
        marker = Marker()
        marker.header.frame_id = "map";
        marker.header.stamp = rospy.Time.now();
        # marker.ns ="wave";
        marker.id = current['y'] + 1000 * current['x']
        marker.action = 0
        marker.action = 0
        marker.color.a = 0.5;
        marker.color.r = 0;
        marker.color.g = 0;
        marker.color.b = 1;
        marker.scale.x = (0.5 / float(10)) * self.RESOLUTION
        marker.scale.y = (0.5 / float(10)) * self.RESOLUTION
        marker.scale.z = 0.20 / float(10)
        marker.type = marker.CUBE
        marker.pose.position.x = (current['x'] / float(2) / (float(10) / self.RESOLUTION)) + 0.2
        marker.pose.position.y = (current['y'] / float(2) / (float(10) / self.RESOLUTION)) + 0.2
        marker.pose.orientation.w = 1
        markerArray.markers.append(marker)

    def createFontierUnitMarker(self, v, markerArray):
        marker = Marker()
        marker.header.frame_id = "map";
        marker.header.stamp = rospy.Time.now();
        # marker.ns ="wave";
        marker.id = v['y'] + 1000 * v['x']
        marker.action = 0
        marker.color.a = 0.5
        marker.color.r = 0
        marker.color.g = 1
        marker.color.b = 0
        marker.scale.x = (0.5 / float(10)) * self.RESOLUTION
        marker.scale.y = (0.5 / float(10)) * self.RESOLUTION
        marker.scale.z = 0.20 / float(10)
        marker.type = marker.CYLINDER
        marker.pose.position.x = (v['x'] / float(2) / (float(10) / self.RESOLUTION)) + 0.2
        marker.pose.position.y = (v['y'] / float(2) / (float(10) / self.RESOLUTION)) + 0.2
        marker.pose.orientation.w = 1
        markerArray.markers.append(marker)