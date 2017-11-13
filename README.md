# global_planner_short_path_student
## Description
Use the turtlebot simulator (state) to learn short path algorithm.

## How to use

first install turtlebot simulator 
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

On the rviz panel click on the goal selection
Select a goal on the map
Your algorithm began

## Customization
Parameters can be modified into the ShortPathMng.py file :
```
RESOLUTION = rospy.get_param('~SHORT_PATH_RESOLUTION', 4): définit la résolution de la grille pour appliquer le plus court chemin (attention calculer un plus court chemin sur une grille très grande peu prendre beaucoup de temps)
```
```
shortPathMethodeSelected = rospy.get_param('~SHORT_PATH_METHOD', 'GREEDY_BEST_FIRST_SEARCH'): definit la méthode par defaut comme algo de plus court chemin
```
```
isLocalPlanner = rospy.get_param('~LOCAL_PLANNER_USED', True) : active l'utilisation de notre local planner ou laisse la main à la couche de navigation de ROS
```
```
inflate_radius= rospy.get_param('~INFLATE_RADIUS', 0.3): definit le rayon d'inflate des obstacles
```



## the job to do 

Modify the files xxx to realise 
