# ROS 2 Web App

Small web UI for launching the existing `my_robot_bringup` launch files from a browser.

## Features

- Launch dropdown for the current bringup targets
- Start and stop controls
- Live status panel
- Recent launch log output in the browser

## Launch Options

- `Robot Bringup` -> `ros2 launch my_robot_bringup my_robot.launch.py`
- `Lidar Only` -> `ros2 launch my_robot_bringup lidar.launch.py`
- `RViz Only` -> `ros2 launch my_robot_bringup rviz.launch.py`
- `Gazebo Sim` -> `ros2 launch my_robot_bringup gazebo.launch.py`

## Run

From the workspace root:

```bash
cd /home/tom/ros2_ws/webapp
python3 server.py
```

Then open `http://localhost:8080`.

The backend sources:

- `/opt/ros/$ROS_DISTRO/setup.bash` when available
- `/home/tom/ros2_ws/install/setup.bash` when available

If your workspace has not been built yet, run `colcon build` first.
