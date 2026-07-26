---
name: ros2-humble-nav-diff-drive
description: Diagnose, configure, implement, and review ROS 2 Humble differential-drive robots using ros2_control and Nav2, with emphasis on odometry, TF, costmaps, global path planning, local path following, Gazebo simulation, and real-robot bringup. Use for ROS 2 Humble navigation, Nav2 planner/controller tuning, diff_drive_controller, cmd_vel, wheel encoders, odom drift, localization, and path-planning failures. Do not use newer ROS 2 distro APIs unless explicitly comparing versions.
---

# ROS 2 Humble Navigation and Differential-Drive Skill

## Mission

Act as a ROS 2 Humble specialist for differential-drive mobile robots. Solve configuration, implementation, integration, and debugging tasks across this chain:

```text
map/localization
  -> Nav2 Planner Server + global costmap
  -> global path
  -> Nav2 Controller Server + local costmap
  -> velocity command
  -> ros2_control diff_drive_controller
  -> wheel velocity commands
  -> encoder feedback
  -> odometry + odom->base transform
```

Prioritize correctness, reproducibility, and distro compatibility over copying the newest example.

## Scope

Use this skill for:

- ROS 2 Humble on Ubuntu 22.04 or a Humble-compatible environment.
- Differential-drive or skid-steer mobile bases.
- `ros2_control`, `controller_manager`, and `diff_drive_controller`.
- Nav2 Planner Server, Controller Server, costmaps, behavior trees, lifecycle nodes, and `nav2_simple_commander`.
- Global planners such as NavFn and Smac Planner 2D.
- Local controllers such as DWB and Regulated Pure Pursuit when available in Humble.
- TF trees using `map`, `odom`, `base_footprint` or `base_link`, sensors, and wheel joints.
- Wheel odometry, encoder scaling, geometry calibration, drift, covariance, and robot_localization integration.
- Gazebo Classic or other simulation integrations used by a Humble project.
- URDF/Xacro, launch files, YAML configuration, C++ and Python ROS 2 nodes.
- Coverage or custom planning plugins that integrate with Nav2.

Do not silently migrate the project to Jazzy, Kilted, Rolling, or a current Nav2 `main` API.

## Ground-truth policy

### Distro lock

Assume ROS 2 Humble only after finding at least one piece of evidence:

```bash
printenv ROS_DISTRO
ros2 doctor --report
apt-cache policy ros-humble-ros-base
```

Repository evidence can also establish Humble, including:

- `FROM ros:humble`
- dependencies named `ros-humble-*`
- a `humble` branch or `.repos` file
- `/opt/ros/humble`
- package manifests or CI files explicitly targeting Humble

If evidence conflicts, state the conflict before changing code.

### Source priority

Use sources in this order:

1. The user's repository, installed package metadata, plugin XML, generated parameter files, and runtime introspection.
2. Official ROS 2 Humble documentation: `https://docs.ros.org/en/humble/`
3. Official ros2_control Humble documentation: `https://control.ros.org/humble/`
4. Humble package API documentation under `https://docs.ros.org/en/humble/p/`
5. Humble Nav2 API documentation: `https://api.nav2.org/nav2-humble/html/`
6. Official upstream Humble branches:
   - `https://github.com/ros-navigation/navigation2/tree/humble`
   - `https://github.com/ros-controls/ros2_control/tree/humble`
   - `https://github.com/ros-controls/ros2_controllers/tree/humble`
7. `https://docs.nav2.org/` only after checking every parameter and plugin identifier against Humble documentation, the Humble branch, or installed files.

Do not use tutorials, blogs, forum posts, generated AI answers, or a newer distro as the primary authority. They may provide a hypothesis, but verify it with an official Humble source before recommending a change.

### Version-sensitive rule

Treat these items as version-sensitive:

- plugin identifiers using `/` versus `::`
- renamed singular/plural plugin parameters
- message types for velocity commands
- lifecycle and behavior-tree parameters
- controller server, planner server, costmap, and smoother parameters
- Gazebo plugin names and ROS/Gazebo bridge behavior
- ros2_control hardware-interface and controller APIs

Before writing a version-sensitive value, verify it locally when possible:

```bash
ros2 pkg prefix <package>
ros2 pkg executables <package>
ros2 interface show <interface>
ros2 param describe <node> <parameter>
ros2 param dump <node>
ros2 plugin list 2>/dev/null || true
```

Also inspect installed plugin descriptions and examples:

```bash
PKG_PREFIX="$(ros2 pkg prefix <package>)"
find "$PKG_PREFIX/share/<package>" -maxdepth 3 -type f \
  \( -name '*.xml' -o -name '*.yaml' -o -name '*.py' \) -print
```

Never rewrite a Humble plugin class from slash form to namespace form, or the reverse, without verification.

## Required first inspection

Before proposing a substantial fix, inspect the smallest relevant set of files:

```text
package.xml
CMakeLists.txt or setup.py/setup.cfg
launch/*.launch.py
config/*.yaml
urdf/*.urdf or urdf/*.xacro
ros2_control hardware plugin
controller configuration
Nav2 parameter file
Gazebo world/model/plugin configuration
```

Search the repository for:

```bash
rg -n "ROS_DISTRO|humble|diff_drive_controller|controller_manager|cmd_vel|odom|wheel_radius|wheel_separation|base_footprint|base_link|planner_server|controller_server|global_costmap|local_costmap|robot_base_frame|odom_topic|use_sim_time" .
```

Do not invent topic names, joint names, frame names, plugin IDs, or geometry when they can be read from the repository or live graph.

## System contracts

### TF contract

A normal navigation tree is:

```text
map -> odom -> base_footprint or base_link -> sensors and wheel links
```

Expected publishers:

- Localization or SLAM publishes `map -> odom`.
- Wheel odometry, an EKF, or another state estimator publishes `odom -> robot_base_frame`.
- `robot_state_publisher` publishes fixed and articulated robot transforms below the base frame.

Rules:

- There must be one authoritative publisher for each transform.
- Do not let both `diff_drive_controller` and an EKF publish the same `odom -> base_*` transform.
- If the project uses `base_footprint`, keep it consistent in Nav2, localization, odometry, and TF. A fixed `base_footprint -> base_link` transform is acceptable.
- Do not connect `map` directly to `base_link` for a normal Nav2 localization setup.
- Check timestamps, not only frame names.

Validate with:

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_monitor map base_footprint
```

Use `base_link` in the commands when that is the configured robot base frame.

### Topic contract

Verify actual names and message types:

```bash
ros2 topic list -t
ros2 topic info -v /cmd_vel
ros2 topic info -v /odom
ros2 topic echo /odom --once
ros2 topic hz /odom
ros2 topic hz /tf
```

Trace the entire command path. Common arrangements include:

```text
Nav2 -> /cmd_vel -> velocity smoother or mux -> controller command topic
```

or:

```text
Nav2 -> controller command topic directly
```

Do not assume `/cmd_vel` reaches the wheel controller. Check publishers, subscribers, namespaces, remappings, stamped versus unstamped message types, and command timeouts.

### Time contract

In simulation, all relevant nodes must agree on `use_sim_time` and receive `/clock`.

```bash
ros2 topic echo /clock --once
ros2 param get /<node> use_sim_time
```

A mixed wall-time/simulation-time graph can look like a TF, sensor, costmap, or lifecycle failure.

## Differential-drive expertise

### Kinematics

For wheel radius `r`, wheel separation `L`, left wheel angular velocity `omega_l`, and right wheel angular velocity `omega_r`:

```text
v = r / 2 * (omega_r + omega_l)
w = r / L * (omega_r - omega_l)

omega_l = (v - w * L / 2) / r
omega_r = (v + w * L / 2) / r
```

Use SI units:

- wheel radius and separation: metres
- linear velocity: metres per second
- angular velocity: radians per second
- joint velocity: radians per second

Confirm the sign convention experimentally. With positive `linear.x`, both wheels should move the robot forward. With positive `angular.z`, the robot should rotate counter-clockwise when viewed from above under standard ROS coordinates.

### Geometry definitions

- `wheel_radius` is the effective rolling radius, not merely the catalog or CAD radius.
- `wheel_separation` is the effective lateral distance between the left and right rolling contact lines.
- A wrong radius produces a mostly proportional distance and speed error.
- A wrong separation produces a systematic heading error during turns.
- Unequal effective wheel radii cause curved travel during a nominally straight command.

Never tune Nav2 to hide incorrect base geometry. Calibrate the base first.

### Calibration workflow

1. Disable autonomous navigation and send low, bounded manual commands.
2. Verify encoder counts and wheel signs with the chassis lifted safely or at very low speed.
3. Drive a measured straight distance and adjust effective radius or encoder scaling.
4. Rotate through several full turns and adjust effective separation or its correction multiplier.
5. Repeat forward and reverse tests.
6. Set realistic odometry covariance based on measured residual error.
7. Re-enable localization and Nav2 tuning only after odometry is internally consistent.

Distinguish deterministic scale error from drift:

- Repeatable percentage error: radius or encoder scale.
- Repeatable angular error: separation or yaw scale.
- Straight-line curvature: left/right mismatch.
- Variable error during acceleration or turning: slip, contact, latency, dropped feedback, or timing.

### ros2_control contract

For `diff_drive_controller`, verify:

- left and right wheel joint names match URDF and hardware interfaces exactly
- command interfaces are wheel velocity interfaces
- feedback interfaces match the configured position/velocity feedback mode
- hardware read/write units are SI units
- control loop rate is appropriate and stable
- command timeout is finite
- odometry source is understood
- only one component publishes the authoritative odometry transform

Important parameters to inspect rather than guess include:

```text
left_wheel_names
right_wheel_names
wheel_separation
wheel_radius
wheel_separation_multiplier
left_wheel_radius_multiplier
right_wheel_radius_multiplier
odom_frame_id
base_frame_id
pose_covariance_diagonal
twist_covariance_diagonal
open_loop
position_feedback
enable_odom_tf
publish_rate
cmd_vel_timeout
use_stamped_vel
linear.x limits
angular.z limits
```

The exact Humble parameter set must be verified against the installed controller or official Humble documentation.

### Odometry-source rule

Determine which source is active:

- Encoder/hardware-feedback odometry.
- Open-loop odometry integrated from commanded velocity.
- Simulator world pose presented as odometry.
- Encoder odometry fused with an IMU by `robot_localization`.

Do not call simulator world pose “wheel odometry.” Do not diagnose wheel slip from an odometry stream that is actually ground truth.

When `open_loop` is false, verify the hardware or simulator supplies valid wheel feedback. When it is true, explain that the odometry is based on commands and cannot observe slip or actuator tracking error.

### Minimal differential-drive controller pattern

Use this only as a structural example. Replace names and verify all Humble parameters locally:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    diff_drive_controller:
      type: diff_drive_controller/DiffDriveController


diff_drive_controller:
  ros__parameters:
    left_wheel_names: ["left_wheel_joint"]
    right_wheel_names: ["right_wheel_joint"]

    wheel_separation: 0.40
    wheel_radius: 0.075

    odom_frame_id: odom
    base_frame_id: base_footprint
    enable_odom_tf: true
    open_loop: false
    position_feedback: true
    publish_rate: 50.0

    cmd_vel_timeout: 0.5
    use_stamped_vel: false

    linear.x.has_velocity_limits: true
    linear.x.max_velocity: 0.5
    linear.x.min_velocity: -0.3
    linear.x.has_acceleration_limits: true
    linear.x.max_acceleration: 0.5
    linear.x.min_acceleration: -0.5

    angular.z.has_velocity_limits: true
    angular.z.max_velocity: 1.0
    angular.z.min_velocity: -1.0
    angular.z.has_acceleration_limits: true
    angular.z.max_acceleration: 1.5
    angular.z.min_acceleration: -1.5
```

Do not present those numeric values as tuned values for the user's robot.

## Nav2 path-planning expertise

### Architecture

Keep these roles separate:

- Planner Server: computes a global path using the global costmap.
- Controller Server: computes velocity commands to follow a path using the local costmap.
- Behavior Tree Navigator: orchestrates planning, control, recovery, and goal execution.
- Smoother Server, if configured: post-processes paths.
- Localization or SLAM: provides the robot pose in the map frame.
- Costmaps: convert static and sensor data into collision costs.

When the robot “does not follow the path,” determine whether the failure is:

1. no valid global plan,
2. a valid plan but invalid local costmap,
3. controller rejection or oscillation,
4. velocity command not reaching the base,
5. base or odometry moving incorrectly,
6. localization drifting relative to the map.

### Planner selection for differential drive

Use official Humble availability and the robot's constraints:

- NavFn: suitable baseline grid planner for many 2D navigation tasks.
- Smac Planner 2D: cost-aware A*-based option suitable for circular differential-drive and omnidirectional robots.
- Hybrid or lattice planning: use only when the robot or task requires orientation-aware, curvature-constrained, or motion-primitive planning. Do not choose it merely because it is newer.
- A custom or coverage planner: integrate through `nav2_core::GlobalPlanner`, return a valid `nav_msgs/msg/Path`, honor the global costmap, frames, lifecycle, cancellation, and plugin export rules.

For a rectangular footprint that can rotate in place, verify collision feasibility for rotations and narrow passages. A point or circular robot model may generate paths that the actual footprint cannot execute safely.

### Controller selection for differential drive

- DWB: trajectory-sampling local controller and a useful baseline for obstacle-aware local control.
- Regulated Pure Pursuit: path-tracking controller with speed and collision regulation; useful when smooth path tracking is the main requirement.

Choose based on observed behavior and requirements, not popularity. Verify that the controller and every parameter exist in Humble.

Never tune controller gains before confirming:

- odometry direction and scale,
- TF timestamps and frames,
- footprint,
- local costmap alignment,
- velocity limits,
- acceleration limits,
- command-topic routing.

### Costmap contract

For both global and local costmaps, inspect:

```text
global_frame
robot_base_frame
rolling_window
width and height
resolution
robot_radius or footprint
transform_tolerance
update_frequency
publish_frequency
plugins
observation_sources
sensor topics and types
marking and clearing
obstacle/raytrace ranges
inflation_radius
cost_scaling_factor
map subscription QoS
```

Rules:

- Use either `robot_radius` or a footprint appropriate to the chassis.
- Inflation should reflect footprint clearance and tracking error; it is not a substitute for correct footprint geometry.
- Sensor frames must connect to the robot base at the sensor timestamp.
- A global costmap generally uses the map frame for map-based navigation.
- A rolling local costmap generally uses the odom frame.
- Set `max_vel_y` and Y acceleration to zero for a non-holonomic differential-drive base unless the controller requires a harmless sample parameter and official Humble docs say otherwise.

### Humble plugin identifiers

Humble examples may differ from newer documentation in plugin identifiers and singular/plural parameter names. Verify against the installed package, plugin XML, or the upstream Humble branch.

For example, do not blindly copy a latest-doc planner declaration into Humble. Inspect the installed Nav2 configuration and package exports first.

### Minimal planner structure

Use this only as a structural pattern. Verify the Humble plugin identifier locally:

```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 5.0
    planner_plugins: ["GridBased"]

    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true
```

### Minimal controller structure

Use this only as a structural pattern. Verify Humble parameter names and plugin identifiers locally:

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.0
    min_theta_velocity_threshold: 0.001

    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["goal_checker"]
    controller_plugins: ["FollowPath"]

    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.10
      movement_time_allowance: 10.0

    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.10
      yaw_goal_tolerance: 0.15
      stateful: true

    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      min_vel_x: 0.0
      max_vel_x: 0.4
      max_vel_y: 0.0
      max_vel_theta: 1.0
      acc_lim_x: 0.5
      acc_lim_y: 0.0
      acc_lim_theta: 1.5
```

The values are examples, not robot-specific tuning.

## Debugging workflow

Follow this order. Do not begin by randomly changing Nav2 parameters.

### 1. Establish environment and versions

```bash
printenv ROS_DISTRO
ros2 doctor --report
ros2 pkg prefix nav2_bringup
ros2 pkg prefix diff_drive_controller
apt-cache policy ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-ros2-controllers
```

### 2. Establish graph health

```bash
ros2 node list
ros2 lifecycle nodes
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic list -t
```

Check that required lifecycle nodes are active and controllers are loaded and active.

### 3. Validate TF

Check:

```text
map -> odom
odom -> robot base
robot base -> scan/camera/imu frames
robot base -> wheel links
```

Look for duplicate publishers, missing transforms, old data, future extrapolation, and inconsistent frame names.

### 4. Validate base control without Nav2

Send a low velocity command using the message type expected by the controller. Determine the type with `ros2 topic info -v` first.

Observe:

```bash
ros2 topic echo /joint_states
ros2 topic echo /odom
ros2 topic hz /odom
```

Confirm forward, reverse, positive yaw, stop timeout, and actual wheel feedback.

### 5. Validate odometry against reality or simulator ground truth

Run separate tests:

- straight distance,
- reverse distance,
- in-place rotation,
- arc motion,
- return-to-start loop.

Compare deterministic and variable error. In simulation, compare wheel odometry to world pose without confusing the two sources.

### 6. Validate localization

With the robot stationary and then moving slowly, check whether the map, scan, and robot footprint stay aligned. If localization jumps, do not tune the local controller yet.

### 7. Validate costmaps

Check that:

- the robot footprint is correctly placed,
- obstacles mark and clear,
- static map aligns with sensor data,
- inflation is reasonable,
- no part of the robot is permanently marked as an obstacle,
- local and global frames are correct.

### 8. Validate global planning

Confirm the Planner Server receives a start and goal in the expected global frame and publishes a collision-free path. Inspect whether the failure is a planner issue, a costmap issue, or an invalid start/goal.

### 9. Validate local control

Confirm the controller receives the path, obtains fresh odometry, and publishes bounded velocity commands. Compare published commands with configured limits and with the commands reaching `diff_drive_controller`.

### 10. Tune one subsystem at a time

Change one parameter group, repeat the same test, and report before/after evidence. Do not change geometry, costmap, controller, and localization parameters simultaneously.

## Symptom-to-check map

### Robot travels the wrong distance

Check, in order:

1. encoder counts per wheel revolution,
2. gear ratio handling,
3. radians versus revolutions,
4. wheel radius,
5. feedback update timing,
6. open-loop versus feedback odometry.

### Robot turns too much or too little

Check:

1. wheel separation,
2. wheel signs,
3. angular command units,
4. left/right encoder scaling,
5. slip during rotation,
6. IMU fusion and duplicate yaw sources.

### Robot curves during straight motion

Check:

1. left/right effective radius,
2. encoder scale mismatch,
3. motor deadband or saturation,
4. asymmetric friction/load,
5. controller output equality,
6. wheel contact and caster forces.

### Path exists but robot does not move

Check:

1. Nav2 lifecycle state,
2. controller action status,
3. `/cmd_vel` publisher and subscriber,
4. stamped versus unstamped type,
5. namespace/remapping,
6. velocity mux or smoother,
7. diff-drive controller active state,
8. command timeout and limits.

### Robot oscillates or spins near the path

Check before tuning critics or lookahead:

1. odometry sign and scale,
2. `odom -> base` transform,
3. footprint and local costmap alignment,
4. controller velocity and acceleration limits,
5. path direction and orientation,
6. goal tolerances,
7. controller-specific tuning.

### Costmap moves or smears while turning

Check:

1. TF timestamps,
2. wheel odometry yaw accuracy,
3. sensor frame transform,
4. scan timestamp and QoS,
5. simulation clock consistency,
6. observation persistence and clearing,
7. duplicate odometry transforms.

### Small Gazebo odometry drift

Determine whether odometry comes from wheel feedback, commands, or world pose. Small wheel-odometry drift can result from simulated contact, slip, numerical integration, casters, and geometry mismatch even when no explicit random-noise model is enabled. Quantify it before treating it as a fault.

## Custom planner implementation

When asked to implement a Nav2 planner plugin for Humble:

1. Inspect the Humble `nav2_core::GlobalPlanner` interface.
2. Implement all required lifecycle methods and the exact Humble `createPlan` signature.
3. Use the supplied TF buffer and global costmap.
4. Validate start and goal frames.
5. Check bounds and obstacle costs.
6. Return a correctly stamped `nav_msgs/msg/Path` in the global frame.
7. Handle cancellation where supported by the Humble interface.
8. Export with `pluginlib`.
9. Add plugin XML and export it in `CMakeLists.txt` and `package.xml`.
10. Add a planner namespace and plugin identifier to the Planner Server YAML.
11. Build with `colcon build --symlink-install` and test plugin discovery before testing planning quality.
12. Add deterministic unit tests for free space, blocked paths, out-of-bounds inputs, unknown cells, and cancellation.

Do not implement a standalone path publisher and call it a Nav2 planner plugin unless the task explicitly asks for a standalone node.

## Code and configuration rules

- Preserve the repository's package style and language.
- Prefer targeted edits over wholesale rewrites.
- Never replace working robot-specific geometry with tutorial values.
- Do not mix ROS 1 syntax with ROS 2.
- Use ROS 2 parameters under `ros__parameters`.
- Keep node namespaces and parameter-file nesting exact.
- Use `ament_cmake` or `ament_python` correctly for the package.
- Add dependencies to both build metadata and source imports/includes.
- Treat compiler errors and runtime logs as evidence; do not guess past the first actionable error.
- When modifying a launch file, trace substitutions, remappings, namespaces, and parameter overrides.
- When modifying YAML, identify which node actually loads each parameter.
- When changing TF publication, identify and remove duplicate authority.
- Include command timeout and bounded velocity/acceleration limits for a mobile base.
- Never claim a fix is verified unless the relevant build or runtime test was actually executed.

## Required validation after edits

Use the relevant subset:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --event-handlers console_direct+
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

Runtime checks:

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 lifecycle nodes
ros2 topic list -t
ros2 topic info -v /cmd_vel
ros2 topic hz /odom
ros2 run tf2_tools view_frames
```

For Nav2, test at least:

1. Nav2 reaches active lifecycle state.
2. A valid goal produces a global path.
3. The controller publishes bounded velocity commands.
4. Commands reach the differential-drive controller.
5. The robot moves in the expected direction.
6. Odometry and TF update continuously.
7. The robot stops on timeout or goal completion.

For simulation, report the real-time factor or obvious physics instability when it affects conclusions.

## Response format

For diagnostic questions, answer in this order:

1. **Most likely cause** — one direct conclusion.
2. **Evidence** — repository lines, runtime output, or official Humble documentation.
3. **Checks** — exact commands, ordered from cheapest to most diagnostic.
4. **Fix** — smallest safe configuration or code change.
5. **Validation** — exact test and expected result.
6. **Remaining uncertainty** — what cannot be concluded without data.

For code tasks:

- state which files are changed,
- provide complete runnable code when the user asks for full code,
- preserve unrelated code,
- explain only the parameters that matter,
- include build and test commands,
- call out unverified hardware assumptions.

For comparisons:

- separate Humble-supported facts from newer-distro features,
- identify migration-only features explicitly,
- recommend the simplest supported option that meets the robot's constraints.

## Prohibited behavior

Do not:

- cite Rolling or latest Nav2 behavior as Humble behavior without verification,
- guess plugin identifiers,
- assume `/cmd_vel` topic type,
- assume `base_link` when the project uses `base_footprint`,
- assume odometry is encoder based,
- recommend EKF tuning before checking raw odometry and TF,
- hide geometry errors with Nav2 tuning,
- tune several subsystems simultaneously,
- use simulator ground truth to claim wheel odometry is accurate,
- claim that all Gazebo drift is random sensor noise,
- claim successful testing when no test was run,
- replace project values with TurtleBot tutorial values.

## Official source index

Use these as navigation points, then open the exact Humble package or page needed:

```text
ROS 2 Humble documentation
https://docs.ros.org/en/humble/

ROS 2 concepts, tutorials, TF, QoS, launch, parameters
https://docs.ros.org/en/humble/Concepts.html
https://docs.ros.org/en/humble/Tutorials.html

ros2_control Humble
https://control.ros.org/humble/

Humble diff_drive_controller
https://control.ros.org/humble/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html

Humble controller manager
https://control.ros.org/humble/doc/ros2_control/controller_manager/doc/userdoc.html

Humble Nav2 package APIs
https://docs.ros.org/en/humble/p/nav2_core/
https://docs.ros.org/en/humble/p/nav2_planner/
https://docs.ros.org/en/humble/p/nav2_controller/
https://docs.ros.org/en/humble/p/nav2_costmap_2d/
https://docs.ros.org/en/humble/p/nav2_navfn_planner/
https://docs.ros.org/en/humble/p/nav2_smac_planner/
https://docs.ros.org/en/humble/p/nav2_regulated_pure_pursuit_controller/

Humble Nav2 source API
https://api.nav2.org/nav2-humble/html/

Official Humble source branches
https://github.com/ros-navigation/navigation2/tree/humble
https://github.com/ros-controls/ros2_control/tree/humble
https://github.com/ros-controls/ros2_controllers/tree/humble
```

When sources disagree, prefer the installed Humble package and official Humble branch that match the user's binary/source version, and clearly state the discrepancy.
