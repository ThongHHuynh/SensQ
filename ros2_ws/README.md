# mobile_base
---
# Update 
- Run the workspace in Docker using
```
docker start ros2_dev
docker exec -it ros2_dev bash
```

# Updates 6/20/2026
- The new iteration of the robot is implemented, with new hardware and URDF. 
- Switch from deploying in Docker to deploying locally. Took over the ownership with:
```
cd ~
sudo chown -R $USER:$USER ros2_ws
```

# Updates 6/27/2026
- Navigation is running using nav2
- Error: global costmap is off from local and lidar -> Solution: match the location start mapping to location start navigating

# Coverage planner archive
- Production planner uses safe-map cell decomposition, RViz cell labels, lawnmower sweeps, checked transitions, and bounded A* fallback.
- It auto-activates lifecycle nodes, logs planning time, rejects unsafe paths, skips unreachable cells, and reports real swept-area progress.
- Package: `coverage_planner_archive`.
- Run: `ros2 launch coverage_planner_archive production_path.launch.py use_sim_time:=false`.

# Gazebo odometry investigation

`gazebo_navigation.launch.py` uses this path:

```text
/cmd_vel -> ros_gz_bridge -> /model/my_robot/cmd_vel
-> Gazebo DiffDrive -> /model/my_robot/odometry
-> ros_gz_bridge -> /diff_drive_controller/odom
-> odom -> base_footprint TF
```

AMCL separately publishes `map -> odom`; `robot_state_publisher` publishes
`base_footprint -> base_link -> sensors/wheels`. The ROS 2 control
`diff_drive_controller` is not launched in this simulation.

Live test results:

- Stopped odometry stayed fixed at `x=0.598900 m`; there was no isolated
  stationary drift.
- A forward test reported `0.598900 m` in odometry while Gazebo moved
  `0.616484 m`: odometry under-reported distance by `2.93%`.
- Cause: wheel collision radius is `0.035 m`, but Gazebo DiffDrive uses
  `0.034 m`. The controller file has a third value, `0.0335 m`, for hardware.
- The wheel collisions are spheres, which can also permit unrealistic lateral
  contact/slip during turns.
- Another Gazebo instance was running on the default transport partition.
  Launching a second instance produced competing `/clock` data and continuous
  `TF_OLD_DATA` warnings for `base_footprint`.

Troubleshoot:

```bash
# Expect one raw odometry publisher: ros_gz_bridge
ros2 topic info /diff_drive_controller/odom --verbose

# Frames must be map -> odom -> base_footprint -> base_link
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_tools view_frames

# Check for duplicate simulators and Gazebo clock publishers
ps -ef | grep -E 'ign gazebo|gz sim|parameter_bridge'
ign topic -i -t /clock

# Check stopped drift and commanded velocity
ros2 topic echo /diff_drive_controller/odom
ros2 topic echo /cmd_vel
```

Run only one simulator/bridge, rebuild after changing Xacro, and use one
measured wheel radius consistently in collision geometry and the Gazebo
DiffDrive plugin. Use cylindrical wheel collisions before tuning friction.

# Nav2 tuning
- DWB is tuned for faster coverage travel with straight tracking, no waypoint wait, and quick in-place turns.
- Entry navigation uses DWB; complete coverage-cell paths use bounded-search
  Regulated Pure Pursuit.
- NavigateToPose counts rotational progress, permits faster smoothed turns, and waits 1 s during BT recovery.

# Simulation IMU
- `navigation.launch.py` fuses `/imu` and wheel odometry using `simulation-ekf.yaml`.
- Use `headless:=true use_rviz:=false` for GUI-free simulation.

# Test coverage planner
- `test_coverage` plans optimized Boustrophedon sweeps with smooth headlands and safe, corner-segmented inter-cell transits.
- Headland curves blend 5 cm into adjoining rows for smooth transitions.
- Inter-cell transitions use collision-checked, tangent-continuous curves.
- Each cell stays one path; retries resume from confirmed in-cell progress.
- RViz shows the planned path, turn types, and labeled decomposition cells.
- Run `ros2 launch test_coverage coverage.launch.py use_sim_time:=true`.
- Headland tuning record: `src/test-coverage/README_HEADLAND_TUNING.md`.
