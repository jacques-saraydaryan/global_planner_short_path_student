# Global planner short path student Gazebo

## How to use

- Set up the environment by following the Docker guide **[SETUP_TP.md](https://gitlab.com/cpelyon/rob_ressources/ros2_cuda_gazebo.docker/-/blob/main/SETUP_TP.md)** (steps 0 to 4): Docker image, workspace, submission repository, container launch.
  - Do **not** `git clone` this repository: add it to your submission repository with `git subtree add` (guide, step 2, `tp_global`).
  - ROS 2 Jazzy, Nav2, Gazebo and the Turtlebot simulation are already installed in the container, and the ROS environment (`RMW_IMPLEMENTATION`, discovery range, `TURTLEBOT3_MODEL`) is already set: no `apt install` or `export` needed.

- Launch the container from the root of your workspace (run it again to open other terminals in the same container)
```
    ./run_tp.sh
```

- In the container, compile your workspace
```
    cd /root/student_ws
    colcon build --symlink-install
    source install/setup.bash
```

- Path of this training repository in the container
```
    export TRAINING_GLOBAL=/root/student_ws/src/global_planner_short_path_student
```

- Start the simulation (set headless:=True to hide the gazebo gui). It also opens rviz.

```
    ros2 launch nav2_bringup tb4_simulation_launch.py headless:=False world:=$TRAINING_GLOBAL/world/baseline.world x_pose:=1.0 y_pose:=1.0 z_pose:=0 map:=$TRAINING_GLOBAL/map/baseline.yaml
```

- In another terminal (`./run_tp.sh`), set the initial pose of the robot on the map (or use the **2D Pose Estimate** tool of rviz)
```
    ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{header: {frame_id: map}, pose: {pose: {position: {x: 1.0, y: 1.0}, orientation: {w: 1.0}}}}"
```

- In another terminal (`./run_tp.sh`), start the node
```
    ros2 run global_planner_short_path_student ShortPathMng
```

On the rviz panel click on the publish point button to select a goal on the map. Your algorithm begins


- Tips: you can set the different algorithm by passing parameters to the node:

```
    ros2 run global_planner_short_path_student ShortPathMng --ros-args -p resolution:=1 -p shortPathMethod:='WAVEFRONT' -p inflate_radius:=0.5 -p isLocalPlanner:=True
```

- Submit your work: commit and push from the host, in the `src` folder of your workspace (guide, step 5).
