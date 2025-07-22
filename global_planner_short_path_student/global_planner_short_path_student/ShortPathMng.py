import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point, PoseStamped, PointStamped
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import Path
from queue import Queue, LifoQueue
from nav_msgs.srv import GetMap
import time

# Base class to handle exceptions
from tf2_ros import TransformException 
 
# Stores known frames and offers frame graph requests
from tf2_ros.buffer import Buffer
 
# Easy way to request and receive coordinate frame transform information
from tf2_ros.transform_listener import TransformListener 

from rclpy.duration import Duration
 

from global_planner_short_path_student.ShortPathMethods.WaveFront import WaveFront
from global_planner_short_path_student.ShortPathMethods.Dijkstra import Dijsktra
from global_planner_short_path_student.ShortPathMethods.GreedyBestFirstSearch import GreedyBestFirstSearch
from global_planner_short_path_student.ShortPathMethods.AStar import AStar

from nav2_simple_commander.robot_navigator import BasicNavigator


class ShortPathMng(Node):
    mapArray = ""
    OBSTACLE_VALUE = 100
    MAP_OBSTACLE_VALUE = -100
    sim_resolution_factor = 4 #8
    shortPathAlgoMap = {'WAVEFRONT': WaveFront(), 'ASTAR': AStar(), 'DIJKSTRA': Dijsktra(),
                        'GREEDY_BEST_FIRST_SEARCH': GreedyBestFirstSearch()}
    shortPathMethodeSelected = 'WAVEFRONT'
    tflistener = ""
    MAX_VALUE = 1000000

    def __init__(self,):
        super().__init__('short_path_mng_node') # Call the constructor of the parent with the node name
        # init params
        self.declare_parameter('resolution',4 )
        param_resolution = self.get_parameter('resolution').get_parameter_value().integer_value

        self.declare_parameter('shortPathMethod','WAVEFRONT' )
        param_shortPathMethod = self.get_parameter('shortPathMethod').get_parameter_value().string_value

        self.declare_parameter('isLocalPlanner',False )
        param_isLocalPlanner = self.get_parameter('isLocalPlanner').get_parameter_value().bool_value

        self.declare_parameter('inflate_radius',0.3 )
        param_inflate_radius = self.get_parameter('inflate_radius').get_parameter_value().double_value

        self.get_logger().info(f'-------------------------------------------')
        self.get_logger().info(f'Parameter shortPathMethod: {param_shortPathMethod}')
        self.get_logger().info(f"Parameter resolution: {param_resolution} ")
        self.get_logger().info(f"Parameter isLocalPlanner: {param_isLocalPlanner} ")
        self.get_logger().info(f"Parameter inflate_radius: {param_inflate_radius} ")
        self.get_logger().info(f'-------------------------------------------')
        


        self.shortPathMethodeSelected = param_shortPathMethod
        self.isLocalPlanner = param_isLocalPlanner
        self.inflate_radius = param_inflate_radius
        self.sim_resolution_factor = param_resolution
        
        # ------------------#
        # --- Subscriber ---#
        # ------------------#

        # get the current map
        #self.sub = self.create_subscription( # Create a Topic subscriber
        #        OccupancyGrid,                               # Msg type
        #        '/map',                              # Topic name
        #        self.mapCallback,                   # Callback function
        #        1)                                    # Queue size
        #


        # get the current goal for navigation
        self.subPt = self.create_subscription(PointStamped,"/clicked_point", self.askForGoalCallback,1)

        # ------------------#
        # --- Publisher ----#
        # ------------------#
        self.pub_marker = self.create_publisher(Marker, 'process_algo', 1)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


        # Send goal to navigation stack
        # Nomore exist
        #self.pub_goal = self.create_publisher(PoseStamped, '/move_base_simple/goal',  queue_size=100)

        self.isMapComputed = False




        self.map_service = self.create_client(GetMap, '/map_server/map')
        while not self.map_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service {} not available, waiting again...'.format('/map_server/map'))
        
        future = self.map_service.call_async(GetMap.Request())
        
        while not future.done():
            rclpy.spin_once(self)
            self.get_logger().info('Waiting result of service {0} ...'.format('/map_server/map'))
            time.sleep(1)
        self.get_logger().info('Result of service {0} available !'.format('/map_server/map'))
        #Processing the received map
        self.mapCallback(future.result().map)

        self.navigator = BasicNavigator()
        #self.navigator.waitUntilNav2Active()

        self.get_logger().info('[ShortPathMng] Started......')


    def mapCallback(self, data):
        self.map_width = data.info.width
        self.map_height = data.info.height
        self.map_resolution = data.info.resolution

        self.get_logger().info(f'data.info.height:{data.info.height}, data.info.width:{data.info.width}')
        self.mapArray = [[0 for x in range(self.map_width)] for x in range(self.map_height)]

        size = self.map_width * self.map_height
        i = 0
        j = 0
        for index in range(0, size):
            current_index_y = 0;
            current_index_x = index % (self.map_width)

            if int(index / (self.map_width)) != 0:
                current_index_y = int(index / self.map_width)

            cellValue = 0

            if data.data[index] == self.OBSTACLE_VALUE:
                cellValue = self.MAP_OBSTACLE_VALUE
            
            self.mapArray[current_index_y][current_index_x] = cellValue

        # INFLATE the map according the given inflate radius
        inflate_map = self.inflate_map(self.mapArray, self.map_resolution)

        # resize map
        self.resizedMap = self.resizeWithResolution(inflate_map, self.sim_resolution_factor)
        self.get_logger().info(f'Map received and processed')
        self.isMapComputed = True

        for shortPathMetodName in self.shortPathAlgoMap:
            self.shortPathAlgoMap[shortPathMetodName].setLogger(self.get_logger())
            self.shortPathAlgoMap[shortPathMetodName].setMap(self.resizedMap, self.map_width, self.map_height,self.map_resolution,self.sim_resolution_factor)
            self.shortPathAlgoMap[shortPathMetodName].sim_resolution_factor = self.sim_resolution_factor

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
    def resizeWithResolution(self, map, map_resolution):
        marker_container = Marker()
        marker_container.id = 2
        marker_container.type = Marker.CUBE_LIST
        marker_container.points = []
        marker_container.colors = []
        marker_container.header.frame_id = "map";
        marker_container.ns = "wall";
        marker_container.scale.x = (0.5 / float(10)) * map_resolution;
        marker_container.scale.y = (0.5 / float(10)) * map_resolution;
        marker_container.header.stamp = rclpy.time.Time().to_msg()
        marker_container.pose.orientation.w = 1.0

        resizedMapArray = [[0 for x in range(int(self.map_width / map_resolution))] for x in range(int(self.map_height / map_resolution))]
        
        i = 0
        j = 0
        while i < len(map):
            j = 0
            while j < len(map[0]):
                if (i == 0):
                    new_i = 0
                else:
                    new_i = int(round(i / float(map_resolution)))
                    new_i = new_i if new_i< len(resizedMapArray) else len(resizedMapArray) -1

                if (j == 0):
                    new_j = 0
                else:
        
                    new_j = int(round(j / float(map_resolution)))
                    new_j = new_j if new_j< len(resizedMapArray[0]) else len(resizedMapArray[0]) -1

                # if(j>=0 and j<self.map_width/resolution and i>=0 and i<self.map_height/resolution):
                if self.isObstacle(map, i, j, map_resolution):
        
                    resizedMapArray[new_i][new_j] = self.MAP_OBSTACLE_VALUE
                    current_point = Point()
                    current_color = ColorRGBA()

                    current_color.a = 0.5;
                    current_color.r = 0.0;
                    current_color.g = 1.0;
                    current_color.b = 1.0;

                    current_point.z = 0.20 / float(10)
                    offset=map_resolution *self.map_resolution/float(2)
                    

                    current_point.x = ((new_j * map_resolution *self.map_resolution))+offset
                    current_point.y = ((new_i * map_resolution *self.map_resolution))+offset

                    #current_point.x = ((new_j / float(2)) / (float(10) / resolution))
                    #current_point.y = ((new_i / float(2)) / (float(10) / resolution)) 

                    marker_container.points.append(current_point)
                    marker_container.colors.append(current_color)
                else:
                    # print 'i:'+str(i)+"--> obstacle"
                    # print 'j:'+str(j)
                    resizedMapArray[new_i][new_j] = 0
                    
                j = j + map_resolution
            i = i + map_resolution
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

        try:
            now = rclpy.time.Time()
            now2 = self.get_clock().now()
            #Providing rclpy.time.Time() will just get us the latest available transform. All this is wrapped in a try-except block to handle possible exceptions.
            trans = self.tf_buffer.lookup_transform(
                  "map",
                  "base_link",
                  now)
            robot_pos = {}
            robot_pos[0] = trans.transform.translation.x
            robot_pos[1] = trans.transform.translation.y
            
        except TransformException as ex:
            self.get_logger().info('Could not transform base_link to map: {0}, time {1}'.format(ex,now))
      

        ## Get current robot position
        #try:
        #    # if self.tflistener.waitForTransform("/base_link", "/map", rospy.Time(0), rospy.Duration(2.0)):
        #    t = self.tflistener.getLatestCommonTime("map", "/base_link")
        #    position, quaternion = self.tflistener.lookupTransform("map", "/base_link", t)
#
        #    # self.tflistener.waitForTransform("map", "base_link", rospy.Time(0),rospy.Duration(2.0))
        #    # transPose = self.tflistener.transformPose("base_link", "map")
        #    # # if self.tflistener.waitForTransform(poseStamped.header.frame_id, "/map", rospy.Time(0), rospy.Duration(1.0)) :
        #    # #     t = self.tflistener.getLatestCommonTime("kinect", "/base_link")
        #    # #     position, quaternion = self.tflistener.lookupTransform(poseStamped.header.frame_id, "/base_link", t)
        #    # self.tflistener.waitForTransform("map", "base_link", rospy.Time.now(),rospy.Duration(2.0))
        #    # transPose = self.tflistener.transformPose("base_link", "map")
        #    robot_pos = position
        #except Exception as err:
        #    self.get_logger().warn("[ShortPathMng] no common frame between [base_link] and [%s], error message [%s]", "map", str(err))


        # find points to matrix
        robot_pos_matrix = {}
        goal_matrix = {}
        if len(str(robot_pos)) > 0:
            self.get_logger().debug("[ShortPathMng] New Navigation From [" + str(robot_pos[0]) + "," + str(robot_pos[1]) + "] To [" + str(
                goal.point.x) + "," + str(goal.point.y) + "], Real positions ")

        # Fix point to matrix coord
        robot_pos_matrix['x'] = int(round(robot_pos[0] / float(self.sim_resolution_factor * 0.05), 0))
        robot_pos_matrix['y'] = int(round(robot_pos[1] / float(self.sim_resolution_factor * 0.05), 0))

        goal_matrix['x'] = int(round(goal.point.x / float(self.sim_resolution_factor * 0.05), 0))
        goal_matrix['y'] = int(round(goal.point.y / float(self.sim_resolution_factor * 0.05), 0))

        self.get_logger().debug("[ShortPathMng] New Navigation From [" + str(robot_pos_matrix['x']) + "," + str(robot_pos_matrix['y']) + "] To [" + str(
                goal_matrix['x']) + "," + str(goal_matrix['y']) + "], Matrix based positions ")

        self.pub_marker.publish((self.createAlgoTxtMarker('Cleaning markers', -1.0, 5.0)))
        # Clear all markers FIXME take lots of time !!!
        # self.clearAllMarkers(self.pub_marker)
        self.pub_marker.publish((self.createAlgoTxtMarker(self.shortPathMethodeSelected, -1.0, 5.0)))

        # ASk to compute the shortest path from selected algorithm
        #marker_array = MarkerArray()
        marker_container =  self.shortPathAlgoMap[self.shortPathMethodeSelected]._create_marker_container()
        goalMap = self.shortPathAlgoMap[self.shortPathMethodeSelected].goto(robot_pos_matrix, goal_matrix,
                                                                            self.resizedMap, self.pub_marker,
                                                                            marker_container)
        self.get_logger().debug("[ShortPathMng]  Path to follow:" + str(goalMap))
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
                marker.header.stamp = rclpy.time.Time().to_msg()
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
        current_color.r = 1.0
        current_color.g = 0.0
        current_color.b = 0.0

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
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        # marker.ns ="wave";
        marker.id = 1234567
        # print 'GOAL MARKER-->'+str((currentgoal.pose.position.x+1000*currentgoal.pose.position.y)*10)
        marker.action = 0
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.scale.z = 1.0
        marker.type = marker.TEXT_VIEW_FACING
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.orientation.w = 1.0
        marker.text = text
        #marker_array.markers.append(marker)
        return marker

    # ******************************************************************************************
    # ************************************   GOAL MNG   ****************************************
    # ******************************************************************************************

    def pushGoals(self, mapNode, start, marker_container, isreverted, isPathOnService):
        # x=round(int(target['x'])/float(self.sim_resolution_factor*0.5),0)
        # y=round(int(target['y'])/float(self.sim_resolution_factor*0.5),0)
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
                # x=round(int(prev.split('_')[0])/float(self.sim_resolution_factor*0.5),0)
                # y=round(int(prev.split('_')[1])/float(self.sim_resolution_factor*0.5),0)
                self.get_logger().debug(f'GOAL -->{prev}')
                x = int(prev.split('_')[0])
                y = int(prev.split('_')[1])
                currentgoal = self.createGoal(x, y)
                self.createGoalMarker(currentgoal, marker_container, x, y)
                # rospy.sleep(0.01)
                goalQueue.put(currentgoal)
                prev = mapNode[str(x) + '_' + str(y)]
        except KeyError as e:
            self.get_logger().debug(f'end reverse path')
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
            self.get_logger().info(f'')
        else:
            while not goalQueue.empty():              
                self.navigator.goToPose(goalQueue.get())
                while not self.navigator.isTaskComplete():
                    feedback = self.navigator.getFeedback()
                    if Duration.from_msg(feedback.navigation_time) > Duration(seconds=5.0):
                        self.navigator.cancelTask()


    def createGoal(self, x, y):
        goal = PoseStamped()
        goal.header.frame_id = "map";
        goal.header.stamp = rclpy.time.Time().to_msg()

        offset=self.sim_resolution_factor *self.map_resolution/float(2)

        goal.pose.position.x = ((x * self.map_resolution *self.sim_resolution_factor))+offset
        goal.pose.position.y = ((y * self.map_resolution *self.sim_resolution_factor))+offset

        #goal.pose.position.x = (x / float(2) / (float(10) / self.RESOLUTION)) + 0.2
        #goal.pose.position.y = (y / float(2) / (float(10) / self.RESOLUTION)) + 0.2
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0379763283083
        goal.pose.orientation.w = 0.999278639063

        return goal

def main(args=None):
    rclpy.init(args=args)   # Initialized the rclpy lib
    mng = ShortPathMng() # Create the node
    rclpy.spin(mng)  # Execute and block until the contexte is shutdown, also to keep alive node and use associated callback function
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mng.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
