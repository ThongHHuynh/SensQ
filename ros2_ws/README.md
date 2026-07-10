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

# Coverage planner
- Production planner uses safe-map cell decomposition, RViz cell labels, lawnmower sweeps, checked transitions, and bounded A* fallback.
- It auto-activates lifecycle nodes, logs planning time, rejects unsafe paths, skips unreachable cells, and reports real swept-area progress.
- Config: `src/coverage_planner/config/production_coverage.yaml`.
- Run: `ros2 launch coverage_planner production_path.launch.py use_sim_time:=false`

# Nav2 tuning
- DWB is tuned for faster coverage travel with straight tracking, no waypoint wait, and quick in-place turns.
