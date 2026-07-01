coverage_manager_node
- Starts/stops coverage mission
- Coordinates all other nodes
- Later: expose /start_coverage, /pause_coverage, /cancel_coverage

map_processor_node
- Subscribes to /map
- Converts OccupancyGrid to safe free-space map
- Applies robot clearance/inflation
- Publishes /coverage/safe_map

path_generator_node
- Subscribes to /coverage/safe_map
- Generates lawnmower/coverage waypoints
- Publishes /coverage/path

coverage_visualizer_node
- Subscribes to /coverage/path
- Publishes RViz MarkerArray
- Helps debug waypoints

nav2_executor_node
- Subscribes to /coverage/path
- Sends poses to Nav2 NavigateThroughPoses

coverage_database_node
- Subscribes to mission/path/status
- Saves map metadata, path, job result