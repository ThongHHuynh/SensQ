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
- The maze SDF resolves `maze.dae` from its installed `worlds` directory.
- The description package and navigation launch export the package `share` parent so Gazebo can resolve robot mesh URIs independently of shell state.
- Run `bws` in Zsh to rebuild and source the workspace.

# Simulation RealSense
- The D435-style RGB-D sensor uses the `0.15 0 0.10` mount and publishes color, depth, camera info, and points under `/camera`.
- All simulation navigation launches isolate Gazebo transport and delay spawning to prevent stale servers from stealing the robot.
- One Ctrl-C shuts down the isolated Gazebo and ROS launch tree.

## RViz Camera jitter
- Symptoms: `camera_optical_frame ... queue is full` or `timestamp ... earlier than all the data in the transform cache`; the Camera display jumps while Image stays smooth.
- Cause: Camera renders timestamped 3D overlays through `map -> odom -> base_footprint -> camera`. Delayed images expose AMCL corrections to `map -> odom`; odometry drift is smooth, while AMCL correction and stale frames produce jumps.
- For plain video, use an Image display on `/camera/color/image_raw` with Best Effort, Keep Last, and depth 1; RViz may remain fixed to `map`.
- For 3D Camera overlays, use depth 1, Best Effort, minimal Visibility, and fixed frame `odom` for smooth local motion. Disable unused point-cloud displays or lower the camera rate if queues still fill.
- Diagnose with `tf2_echo map odom`, `tf2_echo odom base_footprint`, `ros2 topic hz /camera/color/image_raw`, and `ros2 topic info /clock --verbose`.

# Simulation AprilTag
- `apriltag_36h11_0` is an installable, upright 200 mm marker on a 5 mm box with an RGB PNG texture.
- Run `bws --packages-select my_robot_description`, then insert it with Gazebo's Resource Spawner.
- `models/apriltag_assets/convert_svg_tags.py` converts all resized tag SVGs to crisp 1000 px RGB PNGs.
- `maze_apriltags.sdf` includes only persistent tags; the launch file spawns the robot separately.

# CSI camera AprilTag
- `csi_camera` publishes synchronized `/camera/image_raw` and `/camera/camera_info` with sensor-data QoS.
- Build with `bws --packages-select csi_camera`, then run `ros2 run csi_camera camera_node --ros-args -p camera_info_url:=file:///absolute/path/camera.yaml`.
- The calibration YAML must match the selected resolution; the default frame is `camera_optical_frame`.
- Run AprilTag with `qos_profile:=sensor_data`, remapping `image_rect` to `/camera/image_raw` and `camera_info` to `/camera/camera_info`.

# Staging and docking
- `my_robot_docking` sends a coarse Nav2 staging goal, searches for the configured tag, then visually aligns `base_footprint` to the tag pose.
- Dock definitions and final offsets are in `my_robot_docking/config/dock_database.yaml`; controller and safety limits are in `my_robot_docking/config/docking_config.yaml`.
- `/cmd_vel_dock` has priority over Nav2 through `velocity_arbiter`; stale commands stop at `/cmd_vel_out`.
- AprilTag uses Reliable camera QoS in Gazebo and Sensor Data QoS with the physical D435.
- Simulation starts docking from `my_robot_navigation/navigation.launch.py`. Hardware bringup also starts the D435; use `start_camera:=false` if its driver is already running.
- Trigger: `ros2 action send_goal /dock my_robot_docking_msgs/action/Dock "{dock_id: home_dock, navigate_to_staging_pose: true, use_offset_override: false}" --feedback`.
- Tag-not-found/lost retries back up with rear LiDAR safety, rotate, then reacquire the tag locally.
- Set a dock's `reverse_docking: true` to capture its tag, rotate the final heading by 180 degrees, and back into the same tag-relative position with rear-sector LiDAR safety.
- Keep `staging_pose` facing the tag; reverse mode changes only the final approach and heading.
- The front D435 cannot see behind the robot, so reverse mode freezes the tag goal in `odom`; `max_reverse_distance` bounds that non-visual final approach.

# Test coverage planner
- `test_coverage` plans optimized Boustrophedon sweeps with smooth headlands and safe, corner-segmented inter-cell transits.
- Headland curves blend 5 cm into adjoining rows for smooth transitions.
- Inter-cell transitions use collision-checked, tangent-continuous curves.
- Each cell stays one path; retries resume from confirmed in-cell progress.
- RViz shows the planned path, turn types, and labeled decomposition cells.
- Run `ros2 launch test_coverage coverage.launch.py use_sim_time:=true`.
- Headland tuning record: `src/test-coverage/README_HEADLAND_TUNING.md`.
