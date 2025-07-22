# global_planner_short_path_student Stage - Docker Start


## How to use

Every thing work inside the following given docker image

Prepare your terminal (X11 redirection)
```
xhost +
```

Start the container (ros humble)
```
sudo docker run -it -p 2222:22 --name global_planner -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix registry.gitlab.com/js-ros-training/ros-training-docker-public/ros-humble-desktop-stage:v2
```

On the opened container terminal load env:
```
source /opt/ros/humble/setup.bash
cd /home/tp/ros_ws/
source install/setup.bash
export ROS_LOCALHOST_ONLY=1
```

Start stage simulator :
```
 ros2 run stage_ros stageros src/stage_ros2/world/maze.world
```
---- 
> for all next ros2 command , open another terminal on the started container:
> ```
>  xhost +
>  sudo docker exec -it <container ID> /bin/bash
> ```
> load env:
> ```
>  source /opt/ros/humble/setup.bash
>  cd /home/tp/ros_ws/
>  source install/setup.bash
> export ROS_LOCALHOST_ONLY=1
> ```
---- 

(New container terminal) Start navigation for stage simulator :
```
ros2 launch stage_ros robot_launch.py nav:=true
```

(New container terminal) Start map_sever :
```
ros2 run nav2_util lifecycle_bringup map_server
```


(New container terminal) Launch the custom short path computation
```
cd /home/tp/ros_ws/src
git clone https://github.com/jacques-saraydaryan/global_planner_short_path_student.git
cd /home/tp/ros_ws
colcon build --packages-select global_planner_short_path_student
source install/setup.bash
ros2 run global_planner_short_path_student ShortPathMng
```

On the rviz panel click on the publish point button to select a goal on the map. Your algorithm begins
