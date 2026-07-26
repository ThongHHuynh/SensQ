# Coverage Planner Archive

Archived ROS 2 safe-map coverage planner for mobile robots.

## Production Pipeline

`production_path.launch.py` starts:

- `map_processor_node`: inflates obstacles and publishes `/coverage/safe_map`.
- `production_path_gen`: lifecycle planner that generates `/coverage/path` on `~/generate_path`.
- `coverage_visualizer_node`: publishes RViz waypoint markers and numbered cell labels.
- `coverage_executor_node`: lifecycle Nav2 `FollowPath` executor with distance-based pause/resume and cancel services.

The production launch auto-configures and activates lifecycle nodes.

## Planning

The planner decomposes safe free space into boustrophedon cells, orders cells from the robot TF pose, generates lawnmower sweeps, validates transitions, and falls back to bounded A* when direct transitions are blocked.

The executor uses `NavigateToPose` for a distant path entry, then densifies validated edges and executes them with `FollowPath` without changing the published coverage geometry.

Production hardening includes:

- bounded A* via `max_astar_iterations`;
- unsafe-edge rejection before publishing;
- simplification rollback when RDP creates unsafe edges;
- planning-time logging;
- unreachable-cell skipping;
- executor cancellation through public action APIs;
- continuous `nav_msgs/Path` execution through Nav2 `FollowPath`;
- real swept-area progress from the safe map and robot TF;
- progress watchdog and optional mission timeout.

## Run

```bash
colcon build --packages-select coverage_planner_archive
source install/setup.bash
ros2 launch coverage_planner_archive production_path.launch.py use_sim_time:=false
ros2 service call /production_path_gen/generate_path std_srvs/srv/Trigger
ros2 service call /coverage_executor_node/start std_srvs/srv/Trigger
```

Tune production parameters in `config/production_coverage.yaml`.

## Test

```bash
python3 -m pytest src/coverage_planner_archive/unit_test
```
