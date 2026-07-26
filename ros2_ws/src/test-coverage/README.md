# test_coverage

ROS 2 Humble Boustrophedon coverage planner with obstacle-aware cell routing,
curve/square headland turns, and Nav2 execution.

```bash
colcon build --packages-select test_coverage
source install/setup.zsh
ros2 launch test_coverage coverage.launch.py use_sim_time:=true
ros2 service call /coverage_planner/generate std_srvs/srv/Trigger
ros2 service call /coverage_executor/start std_srvs/srv/Trigger
```

The normal navigation RViz view shows the full path, colored path types,
translucent decomposition cells, cell labels, and start/end markers. For a
standalone view, launch with `use_rviz:=true`.

The planner uses bounded 2-opt cell ordering plus obstacle-aware endpoint
selection. Inter-cell transitions use collision-checked tangent curves when
space permits. Headland turns use a configurable lane overlap for smooth
lead-in and lead-out; the simulation profile uses a 16 cm minimum turning
radius. The executor calls `NavigateToPose`, then sends each complete cell path
to Nav2 `FollowPath`; inter-cell transits and their abrupt corners remain
isolated. Retries resume from monotonic in-cell progress instead of replaying
the cell from its first pose. Entry navigation uses DWB; coverage uses a
bounded-search Regulated Pure Pursuit controller.

Run tests with `colcon test --packages-select test_coverage`.

See [README_HEADLAND_TUNING.md](README_HEADLAND_TUNING.md) for the headland and
transition run that reached segment 48 of 198.
