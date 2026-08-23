# Build Prompt — SensQ Full Stack: Fix Everything + End-to-End UI

Copy the prompt below and use it with `/goal` for a thorough autonomous build.

---

````text
Build the complete SensQ autonomy stack and operator UI end-to-end. Fix all known bugs first, then build every missing feature. The workspace is at /home/tom/SensQ. It runs ROS 2 Humble on Ubuntu 22.04.

Work in phases. Build and verify each phase before moving on. Follow all conventions already established in the codebase.

## Workspace context

### Architecture
```
Frontend (React/Vite/Tailwind) → HTTP/WebSocket → Backend (FastAPI) → ROS Gateway (rclpy RosMonitor thread) → ROS 2 → Robot
```
The frontend NEVER talks directly to ROS. All robot interaction goes through the backend API.

### Existing file layout
```
/home/tom/SensQ/
├── app/
│   ├── backend/           # FastAPI backend (app/backend/app/)
│   │   ├── app/main.py    # REST endpoints + WebSocket
│   │   ├── app/state.py   # RobotState in-memory snapshot
│   │   ├── app/ros_monitor.py    # rclpy node in background thread
│   │   ├── app/launch_manager.py # subprocess manager for ROS launches
│   │   ├── app/coverage_manager.py
│   │   ├── app/mapping_manager.py
│   │   ├── app/teleop_manager.py
│   │   ├── app/docking.py        # DockingCatalog (dock_database.yaml)
│   │   ├── app/models.py         # SQLAlchemy models (RobotEvent, RobotSnapshot, SavedMap)
│   │   ├── app/database.py       # async DB helpers
│   │   ├── app/websocket_manager.py
│   │   ├── app/config.py         # all config constants
│   │   └── requirements.txt
│   ├── ui/                # React frontend (app/ui/src/)
│   │   ├── src/App.jsx
│   │   ├── src/layout/AppLayout.jsx      # sidebar nav + status
│   │   ├── src/pages/HomePage.jsx        # dashboard, 3D model, quick actions
│   │   ├── src/pages/DeviceStatusPage.jsx # device table
│   │   ├── src/pages/MapsPage.jsx        # map canvas, nav, coverage, SLAM
│   │   ├── src/pages/DockingPage.jsx     # dock/create station
│   │   ├── src/pages/VisualizationPage.jsx # STUBBED camera page
│   │   ├── src/pages/SettingsPage.jsx    # STUBBED settings
│   │   ├── src/hooks/useRobotSnapshot.js # WebSocket + polling + mock fallback
│   │   ├── src/services/robotApi.js      # REST client
│   │   ├── src/data/mockRobot.js         # mock state for offline dev
│   │   ├── src/components/RobotModelViewer.jsx  # Three.js 3D robot
│   │   ├── src/components/MetricCard.jsx
│   │   ├── src/components/StatusBadge.jsx
│   │   └── public/robot_meshes/*.STL     # 3D mesh assets
│   └── dev.sh             # orchestrator script
├── data/sensq.db          # SQLite database
└── ros2_ws/               # ROS 2 workspace
    └── src/
        ├── my_robot_navigation/   # canonical bringup (launch/bringup.launch.py, launch/simulation.launch.py)
        ├── my_robot_bringup/      # standalone launches
        ├── my_robot_description/  # URDF/Xacro, Gazebo worlds
        ├── my_robot_hardware/     # C++ ros2_control SystemInterface (serial: V <wr> <wl>, C <rc> <lc>)
        ├── my_robot_docking/      # production docking action server (/dock)
        ├── my_robot_docking_msgs/ # Dock.action definition
        ├── coverage_planner/      # package name "test_coverage", Boustrophedon planner + Nav2 executor
        ├── csi_camera/            # Jetson CSI camera driver
        ├── sllidar_ros2/          # LiDAR driver
        └── tm_imu/                # IMU driver
```

### Existing backend API endpoints (already working)
- `GET /api/health`, `GET /api/robot/snapshot`
- `POST /api/robot/launch/mobile-base`, `POST /api/robot/stop`
- `POST /api/teleop/start`, `POST /api/teleop/stop`, `POST /api/teleop/cmd_vel`
- `GET /api/docking/config`, `POST /api/docking/start`, `POST /api/docking/cancel`, `POST /api/docking/stations`
- `GET /api/maps`, `POST /api/maps/{map_id}/select`, `PATCH /api/maps/{map_id}`
- `POST /api/mapping/start`, `POST /api/mapping/stop`, `POST /api/mapping/reset`, `POST /api/mapping/save`
- `POST /api/navigation/initial-pose`, `POST /api/navigation/goals`, `POST /api/navigation/cancel`
- `POST /api/coverage/prepare`, `POST /api/coverage/generate`, `POST /api/coverage/execute`, `POST /api/coverage/cancel`
- `WS /ws/robot-state`

### Existing state shape (state.py → WebSocket → useRobotSnapshot)
The frontend receives the full robot state via WebSocket. Key fields:
- `connection.launch_state` (stopped/starting/running/error)
- `hardwareStatus` (are_motors_ready, temperature, debug_message)
- `battery` (percent, voltage, state) — currently STUBBED as null
- `pose` (frame, x, y, yaw)
- `navigation` (state, activeMap, localization, initialPose, goal)
- `docking` (active, state, request, distanceRemaining, lateralError, yawError, retryCount, result)
- `coverage` (plannerReady, state, detail, metrics, path)
- `liveMap` (frame, width, height, resolution, origin, data)
- `devices` (dict of device names → {status, detail})
- `maps` (list of saved + workspace maps)
- `tagDetections` (list of visible AprilTag observations)

### Conventions
- Backend uses async SQLAlchemy with `aiosqlite` (default) or `asyncpg` (PostgreSQL)
- ROS interactions happen ONLY in `ros_monitor.py` (rclpy background thread)
- Subprocess launches use dedicated manager classes (LaunchManager pattern)
- Frontend uses functional React components with hooks
- State flows: ROS → ros_monitor → state.py → WebSocket → useRobotSnapshot → components
- All velocities clamped: linear [-0.5, 0.5], angular [-1.5, 1.5]
- Robot base frame: `base_footprint` everywhere
- `velocity_arbiter` multiplexes `/cmd_vel_nav`, `/cmd_vel_dock` → `/cmd_vel_out`
- Coverage planner topics: `/test_coverage/path`, `/test_coverage/planning_status`, `/test_coverage/execution_status`

---

## Phase 1 — Fix all known bugs

### ROS 2 workspace fixes
1. **Camera launch**: In `ros2_ws/src/my_robot_bringup/launch/peripherals/camera.launch.py`, change `package='my_robot_bringup'` to `package='csi_camera'` for the `camera_node` executable.
2. **RViz install**: In `ros2_ws/src/my_robot_bringup/CMakeLists.txt`, add `rviz` to the `install(DIRECTORY ... DESTINATION share/${PROJECT_NAME})` call.
3. **Hardcoded map path**: In `ros2_ws/src/my_robot_navigation/launch/simulation.launch.py`, replace the hardcoded `/home/tom/maps/simple_maze.yaml` with a `LaunchConfiguration('map')` defaulting to the package share maps directory. Install maps in CMakeLists.txt if needed.
4. **Coverage package name**: Rename the directory `ros2_ws/src/coverage_planner` to `ros2_ws/src/test_coverage` to match the package name in package.xml and setup.py. Update any cross-references.

### Backend fixes
5. **AppLayout status**: In `app/ui/src/layout/AppLayout.jsx`, the footer shows hardcoded `"ROS bridge: Mocked data source"`. Replace with dynamic text reflecting the actual `robot.connection` source from the snapshot (e.g., "Connected to backend" vs "Mock data").
6. **Config portability**: In `app/backend/app/config.py`, ensure `DOCK_DATABASE_FILE` and `DOCKING_CONFIG_FILE` paths are resolved relative to `ROS_WORKSPACE` rather than hardcoded absolute paths.

### Build and verify
```bash
cd /home/tom/SensQ/ros2_ws && colcon build --symlink-install
cd /home/tom/SensQ/app/ui && npm run build
```

---

## Phase 2 — Live camera streaming

Implement MJPEG camera streaming from ROS to the frontend through the backend.

### Backend
1. In `ros_monitor.py`, add a subscriber to `/camera/image_raw` (`sensor_msgs/Image`) or `/camera/color/image_raw`. Convert each frame to JPEG using OpenCV (`cv2.imencode`). Store the latest JPEG bytes in a thread-safe buffer (e.g., `threading.Lock` + `bytes` attribute on the monitor).
2. Add `opencv-python-headless` to `requirements.txt`.
3. In `main.py`, add a streaming endpoint:
   ```python
   @app.get("/api/camera/stream")
   async def camera_stream():
       # Return StreamingResponse with media_type="multipart/x-mixed-replace; boundary=frame"
       # Yield MJPEG frames from the ros_monitor buffer at ~15 fps
   ```
4. Add a `GET /api/camera/snapshot` endpoint that returns a single JPEG frame.
5. Update `state.py` to include `camera.available` (bool) and `camera.topic` (str) fields. Set `camera.available = True` when frames are being received.

### Frontend
6. Replace the stubbed `VisualizationPage.jsx` with a working camera page:
   - Display the MJPEG stream using an `<img>` tag with `src` pointed at `/api/camera/stream`.
   - Show a "No camera feed" placeholder when `robot.camera.available` is false.
   - Display camera metadata: topic name, resolution (from state), frame rate.
   - Add a "Snapshot" button that fetches `/api/camera/snapshot` and allows download.
   - Keep the existing overlay toggle checkboxes for future use (e.g., AprilTag overlay, grid overlay).
7. Update `mockRobot.js` to include `camera: { available: false, topic: '/camera/image_raw' }`.

---


## Phase 3 — Settings persistence

### Backend
1. Add a `Setting` model to `models.py`: `id` (PK), `key` (Str, unique, indexed), `value` (JSON), `updated_at` (DateTime).
2. Add endpoints:
   - `GET /api/settings` — returns all settings as a key-value dict.
   - `PUT /api/settings` — accepts a partial dict and upserts each key.
3. Define settings keys: `backend_url`, `ros_domain_id`, `default_map`, `command_safety_mode`, `use_sim_time`.
4. On backend startup, load persisted settings and apply relevant ones (e.g., set `ROS_DOMAIN_ID` environment variable).

### Frontend
5. Rewrite `SettingsPage.jsx` to:
   - Fetch current settings from `GET /api/settings` on mount.
   - Provide form inputs for each setting (keep the existing form layout and fields).
   - Save changes via `PUT /api/settings` with a "Save" button.
   - Show success/error toast on save.
   - Add a "Reset to defaults" button.
6. Add settings API calls to `robotApi.js`.

---

## Phase 4 — Undock action (ROS 2 + backend + frontend)

### ROS 2: Create Undock.action
1. In `ros2_ws/src/my_robot_docking_msgs/action/`, create `Undock.action`:
   - Goal: `string dock_id`
   - Result: `bool success`, `uint16 error_code`, `string message`
   - Error constants: `NONE=0`, `UNKNOWN_DOCK=1`, `SAFETY_STOP=2`, `CONTROL_FAILED=3`, `CANCELLED=4`
   - Feedback: `uint8 state`, `float64 distance_cleared`
   - State constants: `IDLE=0`, `REVERSING=1`, `ROTATING=2`, `CLEARING=3`, `COMPLETE=4`
2. Rebuild `my_robot_docking_msgs`.

### ROS 2: Implement undock server
3. Create `ros2_ws/src/my_robot_docking/my_robot_docking/undock_server.py`:
   - Action server at `/undock` using `Undock.action`.
   - Looks up dock in `dock_database.yaml` for approach heading.
   - Publishes to `/cmd_vel_dock` (velocity_arbiter priority).
   - Sequence: reverse at 0.1 m/s for configured distance (default 0.3m) with rear LiDAR safety check (reuse the safety monitoring pattern from `docking_server.py` — subscribe to `/scan`, check rear sector), then rotate 180° to face away from dock, then signal complete.
   - Publish RViz markers to `/docking/markers`.
4. Add `undock_server` entry point in `setup.py`.
5. Add `undock_server` node to `my_robot_docking/launch/docking.launch.py`.
6. Add the node to `my_robot_navigation/launch/bringup.launch.py` and `simulation.launch.py`.
7. Build and verify the action interface generates and node starts.

### Backend: Wire undock
8. In `ros_monitor.py`, add an action client for `/undock` (`Undock.action`). Add methods `send_undock_goal(dock_id)` and `cancel_undock()`. Process feedback and update state.
9. In `state.py`, add undock fields to `docking` (or add a separate `undocking` section): `undocking_active`, `undocking_state`, `undocking_distance_cleared`.
10. In `main.py`, add endpoints:
    - `POST /api/docking/undock` — sends undock goal with `dock_id`.
    - `POST /api/docking/undock/cancel` — cancels active undock.

### Frontend: Wire undock UI
11. In `DockingPage.jsx`, add an "Undock" button (visible when `robot.docking.state` indicates docked or idle). Wire it to `POST /api/docking/undock`. Show undock progress (state, distance cleared) in the telemetry panel. Add a cancel button for active undock.
12. In `HomePage.jsx`, add an "Undock Robot" quick action button alongside the existing "Dock Robot" button.
13. Add undock API calls to `robotApi.js`.
14. Update `mockRobot.js` with undock state fields.

---

## Phase 5 — Coverage path obstacle avoidance with DWB

**IMPORTANT: Do NOT modify `nav2_config.yaml` or `sim_nav2_config.yaml`. The existing navigation configuration is working and must not be touched.**

### Create a separate coverage navigation config
1. Create a NEW file `ros2_ws/src/my_robot_navigation/config/coverage_nav2_config.yaml`. This file contains ONLY the controller_server override that adds a second `CoveragePath` controller alongside the existing `FollowPath`. Copy the existing controller_server block from `nav2_config.yaml` as a base, keep `FollowPath` identical, and add `CoveragePath`:
   ```yaml
   controller_server:
     ros__parameters:
       controller_plugins: ["FollowPath", "CoveragePath"]

       FollowPath:
         # Copy the EXACT existing FollowPath config from nav2_config.yaml here
         # Do not change any values — this preserves normal navigation behavior
         plugin: "dwb_core::DWBLocalPlanner"
         # ... (copy all existing FollowPath params unchanged)

       CoveragePath:
         plugin: "dwb_core::DWBLocalPlanner"
         # DWB tuned for tight coverage path tracking + obstacle avoidance
         PathAlign.scale: 32.0
         PathDist.scale: 32.0
         GoalDist.scale: 24.0
         BaseObstacle.scale: 0.02
         ObstacleFootprint.scale: 0.5
         max_vel_x: 0.4
         min_vel_x: 0.0
         max_vel_y: 0.0
         max_vel_theta: 1.0
         acc_lim_x: 0.5
         acc_lim_y: 0.0
         acc_lim_theta: 1.5
         min_speed_xy: 0.0
         max_speed_xy: 0.4
         min_speed_theta: 0.0
   ```
   Also create `coverage_sim_nav2_config.yaml` following the same pattern using `sim_nav2_config.yaml` as the base.
2. Install both new config files in CMakeLists.txt (they should be picked up by the existing `config` directory install rule).
3. Update the coverage launch path: in `ros2_ws/src/coverage_planner/launch/coverage.launch.py` (or wherever the coverage executor is launched), pass the coverage nav2 config as a parameter override to the controller_server so it loads the dual-controller config. If the coverage executor runs within an already-running Nav2 stack, use `ros2 param load` or a launch argument to reload the controller_server params from the coverage config before starting coverage, and restore the original config after coverage completes.
4. In `ros2_ws/src/coverage_planner/test_coverage/executor_node.py`, update the FollowPath action goal to specify `controller_id: "CoveragePath"` when sending coverage cell paths. Entry navigation goals should continue using `controller_id: "FollowPath"`.
5. Build and verify both controllers load when using the coverage config.

### Live costmap validation in planner
4. In `ros2_ws/src/coverage_planner/test_coverage/planner_node.py`, subscribe to the global costmap topic (`/global_costmap/costmap` — `nav_msgs/OccupancyGrid`). After generating the coverage path, validate each waypoint against the live costmap. Log warnings for waypoints in lethal/inscribed cells. Optionally mark blocked segments in the published markers with a distinct color (red).

---

## Phase 6 — Mission sequencer (ROS 2 + backend + full UI page)

### ROS 2: Mission action interface
1. Add `Mission.action` to `my_robot_docking_msgs/action/` (reuse the existing msgs package to avoid a new package):
   - Goal: `string mission_type` ("coverage" or "patrol"), `string dock_id` (dock to return to), `bool auto_dock_on_complete`
   - Result: `bool success`, `float64 area_covered_m2`, `float64 duration_seconds`, `string message`
   - Feedback: `uint8 phase`, `string current_zone`, `float64 progress_percent`, `string detail`
   - Phase constants: `UNDOCKING=0`, `NAVIGATING=1`, `COVERING=2`, `RETURNING=3`, `DOCKING=4`, `COMPLETE=5`, `PAUSED=6`, `ERROR=7`
2. Rebuild `my_robot_docking_msgs`.

### ROS 2: Mission sequencer node
3. Create a new package `ros2_ws/src/my_robot_mission` (ament_python):
   - `my_robot_mission/mission_sequencer.py`: Action server at `/execute_mission`.
   - Orchestration loop:
     a. Undock (call `/undock` action, wait for completion)
     b. Navigate to coverage start (call `/navigate_to_pose`)
     c. Execute coverage (call `/coverage_executor/start` service or FollowPath action)
     d. Navigate to dock staging area (call `/navigate_to_pose`)
     e. Dock (call `/dock` action)
     f. Report complete with metrics
   - Handle preemption: new mission cancels current, robot stops safely.
   - Publish mission events to `/mission/events` (`std_msgs/String` JSON).
   - Log mission start/end times and coverage metrics.
4. Add launch file `my_robot_mission/launch/mission.launch.py`.
5. Wire into `my_robot_navigation/launch/bringup.launch.py` and `simulation.launch.py` as an optional node gated by `enable_mission:=true`.
6. Build and verify.

### Backend: Wire mission
7. In `ros_monitor.py`, add an action client for `/execute_mission` (`Mission.action`). Process feedback and update state.
8. In `state.py`, add a `mission` section:
   ```python
   "mission": {
       "active": False,
       "phase": "IDLE",
       "mission_type": None,
       "current_zone": None,
       "progress_percent": 0.0,
       "detail": "",
       "elapsed_seconds": 0,
       "result": None
   }
   ```
9. In `main.py`, add endpoints:
   - `POST /api/mission/start` — accepts `{ mission_type, dock_id, auto_dock_on_complete }`, sends mission goal.
   - `POST /api/mission/cancel` — cancels active mission.
   - `GET /api/mission/history` — returns past mission results from the database.
10. Add a `MissionRecord` model to `models.py`: `id`, `mission_type`, `dock_id`, `started_at`, `completed_at`, `success`, `area_covered_m2`, `duration_seconds`, `message`.
11. When a mission completes, save the result to the database.

### Frontend: Mission page
12. Create a new page `src/pages/MissionPage.jsx` and add it to the sidebar navigation in `AppLayout.jsx` (icon: `Play` or `Route` from lucide-react):
   - **Mission Control Panel**:
     - Mission type selector: "Coverage Clean" / "Patrol" dropdown.
     - Dock selector: dropdown from `robot.docking` catalog (reuse the pattern from DockingPage).
     - "Auto-dock on complete" toggle.
     - "Start Mission" button → `POST /api/mission/start`.
     - "Cancel Mission" button (visible when active) → `POST /api/mission/cancel`.
   - **Live Mission Status**:
     - Phase indicator: visual stepper showing UNDOCKING → NAVIGATING → COVERING → RETURNING → DOCKING → COMPLETE, highlighting the current phase.
     - Progress bar showing `progress_percent`.
     - Elapsed time counter.
     - Detail text showing `mission.detail`.
   - **Mission History Table**:
     - Fetches from `GET /api/mission/history`.
     - Columns: Date, Type, Duration, Area Covered, Status (success/failed), Message.
     - Most recent missions first.
13. Add mission API calls to `robotApi.js`:
    ```javascript
    startMission: (missionType, dockId, autoDock) => post('/api/mission/start', { mission_type: missionType, dock_id: dockId, auto_dock_on_complete: autoDock }),
    cancelMission: () => post('/api/mission/cancel'),
    getMissionHistory: () => get('/api/mission/history'),
    ```
14. Update `mockRobot.js` with mission state fields.
15. In `HomePage.jsx`, add a "Start Mission" quick action card that links to the Mission page. Add a mission status MetricCard showing the current phase when a mission is active.

---

## Phase 7 — Polish and consistency

1. **Package metadata**: Fill all `TODO` placeholders in `package.xml` and `setup.py` across `csi_camera`, `my_py_pkg`, `coverage_planner`/`test_coverage` with real descriptions, license (e.g., MIT or Apache-2.0), and maintainer email.
2. **Launch file naming**: Rename `my_robot_bringup` launch files to use `.launch.py` suffix: `robot-navigation.py` → `robot_navigation.launch.py`, `robot-navigation-no-ekf.py` → `robot_navigation_no_ekf.launch.py`, `robot-slam.py` → `robot_slam.launch.py`.
3. **Mock data**: Ensure `mockRobot.js` includes ALL new state fields (camera, mission, undocking) so the UI works in offline/mock mode.
4. **Error handling**: Ensure all new frontend API calls have proper error handling with user-visible error messages (follow the existing pattern in `robotApi.js`).

---

## General rules

- Follow existing code style and patterns in each package and file.
- Preserve all existing comments and docstrings unrelated to changes.
- Use `base_footprint` as the robot base frame everywhere.
- Use the `velocity_arbiter` pattern for any new cmd_vel sources.
- Add proper dependencies to `package.xml` and `CMakeLists.txt` or `setup.py`.
- Add proper dependencies to `requirements.txt` for new Python backend packages.
- Do not use ROS 2 APIs newer than Humble.
- Frontend talks ONLY to the backend API, never directly to ROS.
- State flows through: ROS → ros_monitor.py → state.py → WebSocket → useRobotSnapshot → components.
- New REST endpoints follow the existing pattern in `main.py` (async handlers, try/except, JSON responses with status codes).
- New frontend pages follow the existing pattern: functional component, `useRobotSnapshot` hook for state, `robotApi` for commands, Tailwind for styling, lucide-react for icons.
- Build ROS packages with `colcon build --symlink-install` after ROS changes.
- Build frontend with `npm run build` in `app/ui/` after UI changes.
- Fix any build errors before proceeding to the next phase.
````
