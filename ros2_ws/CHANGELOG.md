# Changelog

This file contains concise notes for changes made to the codebase in this workspace.

## [2026-06-28]
* **Problem**: Topic mismatch and missing QoS latching prevented zigzag paths and markers from displaying in RViz.
* **Resolution**: Updated `map_processor_node` to publish `/coverage/safe_map` and updated path/marker nodes to use `TRANSIENT_LOCAL` QoS.
## [2026-06-28]
* **Problem**: Path visualizer dropped markers due to large array size, and path generator only generated endpoints instead of dense grid waypoints.
* **Resolution**: Switched visualizer to use `SPHERE_LIST` and updated `path_generator_node` to add dense waypoints along line segments and handle obstacle splits properly.
## [2026-06-28]
* **Subject**: configuration & visualization
* **Problem**: Nodes crashed due to missing config file installation, `ros_parameters` typo, missing `use_sim_time`, and RViz threw TF past extrapolation errors for latched markers.
* **Resolution**: Fixed `setup.py` to install config, fixed YAML double-underscore typo, added `use_sim_time` to launch nodes, and set static message timestamps to 0 to bypass TF history limits.
## [2026-06-28]
* **Subject**: navigation / coverage path planning
* **Problem**: Naive global lawnmower sweep blindly iterated row-by-row across the full map width, causing the robot to jump across walls to reach disconnected segments on the other side of an obstacle.
* **Resolution**: Replaced with a localized greedy cellular decomposition algorithm. Each row of free space is decomposed into horizontal `Segment` objects (y, x1, x2). The algorithm picks a starting segment, sweeps it, then looks for an overlapping (connected) segment on the adjacent row via `is_connected()` (checks horizontal overlap ± spacing). It continues zigzagging within the same connected zone. If no connected neighbor exists, it reverses vertical direction (U-turn recovery). Only when the entire zone is exhausted does it jump to the nearest unvisited segment (Euclidean distance), starting a new localized zigzag.
## [2026-06-28]
* **Subject**: navigation / path refinement
* **Problem**: Row-to-row transitions created diagonal jumps that cut through walls. Waypoint traversal order was not visible in RViz.
* **Resolution**: Added L-shaped transition waypoints between rows: first a vertical step at the current X to the next row's Y, then a horizontal step to the sweep start. Added `TEXT_VIEW_FACING` numbered labels at every Nth waypoint (~50 labels max) in both `coverage_visualizer_node` and `coverage_node` `publish_markers()`.
