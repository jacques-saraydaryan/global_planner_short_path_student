__author__ = 'Jacques Saraydaryan'

from abc import ABCMeta, abstractmethod
import rclpy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA


class AbstractShortPath:
    RESOLUTION = 8
    MAP_OBSTACLE_VALUE = -100

    def __init__(self):
        # FIXME need to take value from ShortPathMng
        self.RESOLUTION = 8
        self.MAP_OBSTACLE_VALUE = -100

    @abstractmethod
    def goto(self, source, target, matrix, pub_marker, marker_array): pass

    # @abstractmethod
    # def robot_detection_callback(self): pass

    def setMap(self, resizedMap, map_width, map_height,map_resolution,resolution):
        self.resizedMap = resizedMap
        self.map_width = map_width
        self.map_height = map_height
        self.map_resolution = map_resolution
        self.resolution = resolution

    # ******************************************************************************************
    # ********************************   MARKER CREATION   *************************************
    # ******************************************************************************************

    def createClosedMarkerPt(self, current, marker_container):
        current_point = Point()
        
        offset=self.map_resolution *self.resolution/float(2)

        current_point.x = (((current['x'] * self.map_resolution *self.resolution))+offset)
        current_point.y = (((current['y'] * self.map_resolution *self.resolution))+offset)        
        #current_point.x = (current['x'] / float(2) / (float(10) / self.RESOLUTION)) + 0.2
        #current_point.y = (current['y'] / float(2) / (float(10) / self.RESOLUTION)) + 0.2
        current_point.z = 0.20 / float(10)

        current_color = ColorRGBA()
        current_color.r = 0.0
        current_color.g = 0.0
        current_color.b = 1.0
        current_color.a = 0.5

        marker_container.points.append(current_point)
        marker_container.colors.append(current_color)

    def createFontierUnitMarkerPt(self, v, marker_container):
        current_point = Point()
        
        offset=self.map_resolution *self.resolution/float(2)

        current_point.x = (((v['x'] * self.map_resolution *self.resolution))+offset)
        current_point.y = (((v['y'] * self.map_resolution *self.resolution))+offset)
        
        
        #current_point.x = (v['x'] / float(2) / (float(10) / self.RESOLUTION)) + 0.2
        #current_point.y = (v['y'] / float(2) / (float(10) / self.RESOLUTION)) + 0.2
        current_point.z = 0.20 / float(10)

        current_color = ColorRGBA()
        current_color.r = 0.0
        current_color.g = 1.0
        current_color.b = 0.0
        current_color.a = 0.5

        marker_container.points.append(current_point)
        marker_container.colors.append(current_color)

    def _create_marker_container(self):
        marker_container = Marker()
        marker_container.id = 1
        marker_container.type = Marker.CUBE_LIST
        marker_container.points = []
        marker_container.colors = []
        marker_container.header.frame_id = "map";
        marker_container.header.stamp = rclpy.time.Time().to_msg()
        marker_container.scale.x = (0.5 / float(10)) * self.RESOLUTION
        marker_container.scale.y = (0.5 / float(10)) * self.RESOLUTION
        marker_container.pose.orientation.w = 1.0
        return marker_container
