# global_planner_short_path_student
## Description
Use the turtlebot simulator (state) to learn short path algorithm.

![Global Planner](https://github.com/jacques-saraydaryan/global_planner_short_path_student/blob/master/img/global_path_computing_astar.png "Define set of custom short path planner")

## How to use

First, install turtlebot simulator 
```
sudo apt-get install ros-kinetic-turtlebot-stage
```

Launch the simulator 
```
roslaunch navigation_stage_student_tp navigation_tp.launch
```

Launch the custom short path computation
```
rosrun navigation_stage_student_tp ShortPathMng.py
```

On the rviz panel click on the publish point button to select a goal on the map. Your algorithm begins

## Customization
Parameters can be modified into the ShortPathMng.py file :

- Define the grid resolution (caution large grid lead to long processing...)
```python
RESOLUTION = rospy.get_param('~SHORT_PATH_RESOLUTION', 4)
```

- Define the default short path method
```python
shortPathMethodeSelected = rospy.get_param('~SHORT_PATH_METHOD', 'GREEDY_BEST_FIRST_SEARCH'): 
```

- Activate custom local planner or not
```python
isLocalPlanner = rospy.get_param('~LOCAL_PLANNER_USED', True)
```

- Define the inflate radius of obstacles

```python
inflate_radius= rospy.get_param('~INFLATE_RADIUS', 0.3)
```

## the job to do 

1 Inflate the obstacles of your map according to the robot radius and the grid resolution
```python
    # **************************************************
    # ***************   INFLATE MAP    *****************
    # **************************************************

    def inflate_map(self,map,map_resolution):
        new_inflated_map=""
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
        # return new_inflated_map
```
2 Complete the files Dijkstra.py to add the Dijkstra short path algorithm
```python
class Dijsktra(AbstractShortPath):
    def __init__(self):
        print ''

    def goto(self, source, target, matrix,pub_marker,marker_array):
        prev={}
        ### TODO
        ###########################################################
        ################### Function Paramters ###################
        ###########################################################
        ### source: coordinate of the robot position source['x'] return the x position, source['y'] return the y position
        ###
        ### target: coordinate of the target position target['x'] return the x position, target['y'] return the y position
        ###
        ### matrix: rescaled map (including obstacles) matrix[i][j] return the value of the cell i,j of the matrix
        ###
        ### self.MAP_OBSTACLE_VALUE: value of an obstacle into the matrix (-100)
        ###
        ### pub_marker: marker publisher to visualize information into rviz (usage pub_marker.publish(marker_array) )
        ###
        ### marker_array: marker container where new markers new to be added
        ###
        ###########################################################
        ################### Function Toolboxes ###################
        ###########################################################
        #   # create a visual information
        #   self.createFontierUnitMarker(v, marker_array)
        #
        #    # publish visual information
        #    pub_marker.publish(marker_array)
        #
        #    # create a visual information
        #    self.createClosedMarker(u, marker_array)
        #
        #
        #
        #
        #
        #
        #                       TODO
        #
        #
        #
        #
        #
        #
        ###
        ### prev:  disctionary holding node precedence
        ### CAUTION prev dictionary has to be completed as follow:
        ###
        ### prev[str(v['x']) + '_' + str(v['y'])] = str(u['x']) + '_' + str(u['y'])
        ###
        ### where v['x'] return the x position of the node v in the resized map
        ### where v['y'] return the y position of the node v in the resized map
        return prev

```

3 Complete the files AStar.py to add the AStar short path algorithm
```python
class AStar(AbstractShortPath):
    def __init__(self):
        print ''

    def goto(self, source, target, matrix,pub_marker,marker_array):
        prev={}
        ### TODO
        ###########################################################
        ################### Function Paramters ###################
        ###########################################################
        ### source: coordinate of the robot position source['x'] return the x position, source['y'] return the y position
        ###
        ### target: coordinate of the target position target['x'] return the x position, target['y'] return the y position
        ###
        ### matrix: rescaled map (including obstacles) matrix[i][j] return the value of the cell i,j of the matrix
        ###
        ### elf.MAP_OBSTACLE_VALUE: value of an obstacle into the matrix (-100)
        ###
        ### pub_marker: marker publisher to visualize information into rviz (usage pub_marker.publish(marker_array) )
        ###
        ### marker_array: marker container where new markers new to be added
        ###
        ###########################################################
        ################### Function Toolboxes ###################
        ###########################################################
        #   # create a visual information
        #   self.createFontierUnitMarker(v, marker_array)
        #
        #    # publish visual information
        #    pub_marker.publish(marker_array)
        #
        #    # create a visual information
        #    self.createClosedMarker(u, marker_array)
        #
        #
        #
        #
        #
        #
        #                       TODO
        #
        #
        #
        #
        #
        #
        ###
        ### prev:  disctionary holding node precedence
        ### CAUTION prev dictionary has to be completed as follow:
        ###
        ### prev[str(v['x']) + '_' + str(v['y'])] = str(u['x']) + '_' + str(u['y'])
        ###
        ### where v['x'] return the x position of the node v in the resized map
        ### where v['y'] return the y position of the node v in the resized map
        return prev

```

4 Link your global planner to your local planner
```python
 def __init__(self, resolution, shortPathMethod,isLocalPlanner,inflate_radius):
	 ...
	# ------------------#
        # ---- Service -----#
        # ------------------#
        self.local_planner_service =""
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
	...
  ```
  
  5 Complete in the case of using your local planner
```python
   def pushGoals(self,mapNode,start,markerArray,isreverted,isPathOnService):
	...
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
		    print''
```
## Example of results (videos)    
### GREEDY_BEST_FIRST_SEARCH

| Behavior | Result |
| --- | ----------- |
| [![GREEDY_BEST_FIRST_SEARCH](img/naviGreedy.gif "GREEDY_BEST_FIRST_SEARCH")](https://www.youtube.com/watch?v=lDA77HuNVHk) | [![Alt text](https://img.youtube.com/vi/lDA77HuNVHk/0.jpg)](https://www.youtube.com/watch?v=lDA77HuNVHk) |
### Wave front
| Behavior | Result |
| --- | ----------- |
| [![Wave front](img/naviWave.gif "Wave front")](https://www.youtube.com/watch?v=O9L5zjO_doc) | [![Alt text](https://img.youtube.com/vi/O9L5zjO_doc/0.jpg)](https://www.youtube.com/watch?v=O9L5zjO_doc) |

### Dijsktra
| Behavior | Result |
| --- | ----------- |
| [![Dijsktra](img/naviDijstr.gif "Dijsktra")](https://www.youtube.com/watch?v=nK7DpX4PoL0) | [![Alt text](https://img.youtube.com/vi/nK7DpX4PoL0/0.jpg)](https://www.youtube.com/watch?v=nK7DpX4PoL0) |

 ### A*
 | Behavior | Result |
 | --- | ----------- |
 | [![AStar](img/naviAstar.gif "AStar")](https://www.youtube.com/watch?v=wf7FvOBaquY) |  [![Alt text](https://img.youtube.com/vi/wf7FvOBaquY/0.jpg)](https://www.youtube.com/watch?v=wf7FvOBaquY) |

