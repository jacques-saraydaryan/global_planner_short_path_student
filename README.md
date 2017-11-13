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
```
RESOLUTION = rospy.get_param('~SHORT_PATH_RESOLUTION', 4)
```

- Define the default short path method
```
shortPathMethodeSelected = rospy.get_param('~SHORT_PATH_METHOD', 'GREEDY_BEST_FIRST_SEARCH'): 
```

- Activate custom local planner or not
```
isLocalPlanner = rospy.get_param('~LOCAL_PLANNER_USED', True)
```

- Define the inflate radius of obstacles

```
inflate_radius= rospy.get_param('~INFLATE_RADIUS', 0.3)
```

## the job to do 

Modify the files xxx to realise 
