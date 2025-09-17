# Global planner short path student Gazebo

## How to use

- Install requirments if needed
```
   sudo apt install ros-jazzy-rmw-cyclonedds-cpp

   sudo apt install ros-jazzy-navigation2
   sudo apt install ros-jazzy-nav2-bringup

   sudo apt install ros-jazzy-turtlebot3-gazebo
```

- Create a Ros workspace and clone the training repo:

```
    mkdir -p ~/ros_planner_ws/src
    cd ~/ros_planner_ws/src
    git clone https://github.com/jacques-saraydaryan/global_planner_short_path_student.git
```

- Complie your ros workspace
```
    cd ~/ros_planner_ws
    colcon build --symlink-install
    source ./install/setup.bash
```


- Export your trainingrepo path
```
    export TRAINING_GLOBAL=<your training repo path>
```
- E.g
```
    export TRAINING_GLOBAL=/home/tp/ros_nav_ws/src/global_planner_short_path_student
```

- Start the simulation (set headless:=False fo visualize gazebo gui)

```
    ros2 launch nav2_bringup tb4_simulation_launch.py world:=$TRAINING_GLOBAL/world/baseline.world x_pose:=1.0 y_pose:=1.0 z_pose:=0 map:=$TRAINING_GLOBAL/map/baseline.yaml
```
- Compile the training Repo and start the Node

```
cd ~/ros_planner_ws/src
colcon build --symlink-install --packages-select global_planner_short_path_student
source install/setup.bash
ros2 run global_planner_short_path_student ShortPathMng
```

On the rviz panel click on the publish point button to select a goal on the map. Your algorithm begins


- Tips: you can set the different algorithm by passing parameters to the node:

```
    ros2 run global_planner_short_path_student ShortPathMng --ros-args -p resolution:=1 -p shortPathMethod:='WAVEFRONT' -p inflate_radius:=0.5 -p isLocalPlanner:=True
```
