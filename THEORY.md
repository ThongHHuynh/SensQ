# SensQ Theory And Takeover Guide

This file explains how the SensQ codebase works and where to start when you want to modify it yourself. The goal is to help you take over the project without needing to understand every file at once.

## Mental Model

SensQ is split into four layers:

```text
React UI
  |
  | REST + WebSocket
  v
FastAPI backend
  |
  | rclpy subscribers/publishers + shell launch commands
  v
ROS 2 system
  |
  v
Robot hardware, sensors, SLAM, controllers
```

The browser never talks to ROS directly. The backend is the bridge. This is important because browser code should not publish raw ROS commands, manage launch files, or make hardware safety decisions.

The current project intentionally keeps `ros2_ws` as a reference workspace. Do not edit `ros2_ws` from the UI/backend work unless you specifically mean to change the robot firmware/ROS packages.

## Where To Start

Start with these files in this order:

1. `ui/src/main.jsx`

   This is the React entry point. It defines the main tabs and decides which page component is shown.

2. `ui/src/hooks/useRobotSnapshot.js`

   This is how the UI gets robot state. It first tries `GET /api/robot/snapshot`, then listens to `/ws/robot-state`. If the backend is unavailable, it keeps mock data.

3. `ui/src/services/robotApi.js`

   This is the frontend API boundary. UI pages call functions here instead of hardcoding `fetch()` everywhere.

4. `backend/app/main.py`

   This is the backend API router. When the frontend clicks a button, most requests land here first.

5. `backend/app/ros_monitor.py`

   This is the live ROS bridge. It subscribes to ROS topics, updates robot state, broadcasts WebSocket updates, and publishes `/cmd_vel`.

6. `backend/app/state.py`

   This defines the shape of the robot snapshot that both backend and frontend expect.

## Frontend Structure

The UI lives in `ui/`.

Important folders:

```text
ui/src/components/   Reusable UI pieces
ui/src/layout/       Main app shell and sidebar navigation
ui/src/pages/        Full tabs: Home, Device Status, Maps, Settings
ui/src/hooks/        React hooks, especially robot state loading
ui/src/services/     Frontend API calls to backend
ui/src/data/         Mock robot data for frontend-only viewing
ui/public/           Static assets, including robot STL meshes
```

### Main React Flow

`ui/src/main.jsx` creates the app:

```text
main.jsx
  -> useRobotSnapshot()
  -> AppLayout
  -> HomePage / DeviceStatusPage / MapsPage / SettingsPage
```

The tabs are defined in `main.jsx`:

```js
const tabs = [
  { id: "home", label: "Home", icon: Home },
  { id: "status", label: "Device Status", icon: Activity },
  { id: "maps", label: "Maps", icon: Map },
  { id: "settings", label: "Settings", icon: Settings }
];
```

To add a new tab:

1. Create a new page file in `ui/src/pages/`.
2. Import it in `main.jsx`.
3. Add a tab entry.
4. Add it to the `pages` object.

### Robot State In The UI

Most pages receive a `robot` object. That object comes from `useRobotSnapshot()`.

Example:

```jsx
function MapsPage({ robot }) {
  const pose = robot.pose ?? {};
  const liveMap = robot.liveMap ?? {};
}
```

Do not make every component fetch its own robot state. Keep the single state stream in `useRobotSnapshot()` and pass data down through props.

### API Calls From UI

All frontend backend calls should go through:

```text
ui/src/services/robotApi.js
```

Examples:

```js
sendTeleopCommand({ linear_x: 0.2, angular_z: 0 });
startMapping();
saveMap("Lab map");
selectMap(map.id);
```

If you add a new backend endpoint, add a matching function in `robotApi.js`, then call that function from the page.

### Home Page

File:

```text
ui/src/pages/HomePage.jsx
```

The Home page shows:

- connection status
- motors status
- battery status
- navigation status
- live robot context
- 3D robot model

The 3D robot model is in:

```text
ui/src/components/RobotModelViewer.jsx
```

The STL mesh files are served from:

```text
ui/public/robot_meshes/
```

If the robot model orientation looks wrong, edit the mesh transforms in `RobotModelViewer.jsx`. If the live orientation is wrong, check how `pose.yaw` is calculated in `backend/app/ros_monitor.py`.

### Maps Page

File:

```text
ui/src/pages/MapsPage.jsx
```

This is currently the busiest page. It handles:

- live map canvas
- robot marker on map
- saved map list
- map rename/select
- start/stop mapping
- save map
- navigation goal UI placeholder
- teleop keyboard mode
- teleop joystick mode
- speed sliders

The map is drawn by `LiveMapCanvas()` inside `MapsPage.jsx`.

The robot marker is also drawn there. The important lines are:

```js
ctx.translate(canvasX, canvasY);
ctx.rotate(-DEG_TO_RAD * yaw + Math.PI / 2);
```

If the arrow points the wrong way, adjust the rotation offset. `Math.PI / 2` means 90 degrees.

### Teleop

Frontend file:

```text
ui/src/pages/MapsPage.jsx
```

Backend route:

```text
POST /api/teleop/cmd_vel
```

Backend publisher:

```text
backend/app/ros_monitor.py
```

Current command path:

```text
Teleop button or joystick
  -> sendTeleopCommand()
  -> POST /api/teleop/cmd_vel
  -> ros_monitor.publish_cmd_vel()
  -> ROS topic /cmd_vel
```

Speed limits are enforced in two places:

Frontend sliders:

```text
linear:  0.05 to 0.5 m/s
angular: 0.1 to 1.5 rad/s
```

Backend validation:

```py
linear_x: ge=-0.5, le=0.5
angular_z: ge=-1.5, le=1.5
```

Backend clamp:

```py
twist.linear.x = max(-0.5, min(0.5, float(linear_x)))
twist.angular.z = max(-1.5, min(1.5, float(angular_z)))
```

If you increase speed, update both frontend and backend. Do not only change the slider.

## Backend Structure

The backend lives in `backend/app/`.

Important files:

```text
main.py              FastAPI app, routes, startup
state.py             Shared robot snapshot shape
ros_monitor.py       ROS subscribers, TF lookup, /cmd_vel publisher
launch_manager.py    Starts/stops robot launch file
mapping_manager.py   Start/stop/save/select/rename maps
teleop_manager.py    Teleop process lifecycle
database.py          SQLAlchemy setup and DB helpers
models.py            PostgreSQL table definitions
websocket_manager.py WebSocket connection manager
config.py            Environment variable config
```

### Backend Startup

When the backend starts, `main.py` runs:

```text
init_db()
save_event("backend", "Backend started")
mapping_manager.load_saved_maps_into_state()
create_monitor(loop)
ros_monitor.start()
save_snapshot(robot_state.get_snapshot())
```

This means the backend always tries to:

- connect to PostgreSQL
- load saved maps
- start the ROS monitor
- publish the initial robot snapshot

If ROS Python imports are unavailable, the backend can still run, but ROS-dependent features will show offline/error status.

### Robot Snapshot

The shared state shape is in:

```text
backend/app/state.py
```

The initial snapshot contains:

```text
connection
hardwareStatus
battery
pose
navigation
liveMap
devices
maps
updatedAt
```

When adding new robot data, add it to `initial_snapshot()` first. Then update the frontend to read that new field.

Example workflow for a new sensor:

1. Add a default value to `state.py`.
2. Subscribe to the ROS topic in `ros_monitor.py`.
3. Update `robot_state` inside the callback.
4. Broadcast the updated snapshot.
5. Display it in a React page.

### ROS Monitor

File:

```text
backend/app/ros_monitor.py
```

This is the core bridge between ROS and the web app.

It currently handles:

- `/diff_drive_controller/odom`
- `/joint_states`
- `/scan`
- `/imu`
- `/map`
- `map -> base_footprint` TF lookup
- `/cmd_vel` publishing

Topic names come from:

```text
backend/app/config.py
```

Defaults:

```py
ODOM_TOPIC = "/diff_drive_controller/odom"
JOINT_STATES_TOPIC = "/joint_states"
CMD_VEL_TOPIC = "/cmd_vel"
```

Override them when launching if needed:

```bash
SENSQ_ODOM_TOPIC=/odom SENSQ_CMD_VEL_TOPIC=/cmd_vel ./scripts/dev.sh
```

### Live Map

ROS publishes `/map` as a full `nav_msgs/OccupancyGrid`. That can be large, so the backend downsamples it before sending to the browser.

The downsampling function is:

```text
occupancy_grid_to_payload()
```

in:

```text
backend/app/ros_monitor.py
```

The frontend draws the downsampled map in:

```text
LiveMapCanvas()
```

inside:

```text
ui/src/pages/MapsPage.jsx
```

If the map is slow, reduce `max_cells` in `occupancy_grid_to_payload()`.

If the map is too blurry/blocky, increase `max_cells`.

### Robot Location On Map

The best robot pose for map display is `map -> base_footprint`, not raw odometry.

The backend tries this in `ros_monitor.py`:

```text
tf_buffer.lookup_transform("map", "base_footprint", ...)
```

If TF is available, `robot.pose.frame` becomes `map`.

If TF is not available, the system falls back to odometry.

If the marker position is wrong:

1. Check that RViz can see `map -> base_footprint`.
2. Check `/tf` is being published.
3. Check `pose.frame` shown in the UI.
4. Check `liveMap.origin` and `liveMap.resolution`.

### Launch Manager

File:

```text
backend/app/launch_manager.py
```

This starts and stops:

```bash
ros2 launch my_robot_bringup my_robot.launch.py
```

It also tries to stop an already-running launch process, even if that launch was started before the website.

If Stop Mobile Base does not stop all robot processes, inspect this file first.

### Mapping Manager

File:

```text
backend/app/mapping_manager.py
```

This manages:

- mapping state
- saving maps
- loading saved map metadata from PostgreSQL
- selecting active map
- renaming saved maps

Saving a map runs:

```bash
ros2 run nav2_map_server map_saver_cli -f <map_prefix>
```

Saved map files go to:

```text
data/maps/
```

Saved map metadata goes to PostgreSQL.

Current limitation: selecting a saved map updates the UI active map state, but does not yet load that map into Nav2/map_server. That is a future backend feature.

## Database

The database is PostgreSQL.

Default URL:

```text
postgresql+asyncpg://sensq:sensq@localhost:5432/sensq
```

Configured in:

```text
backend/app/config.py
```

The SQLAlchemy setup is in:

```text
backend/app/database.py
```

Models are in:

```text
backend/app/models.py
```

The database currently stores:

- backend events
- robot snapshots
- saved map metadata

For a new table:

1. Add a model in `models.py`.
2. Add helper functions in `database.py`.
3. Call those helpers from a manager or route.
4. Add API endpoints if the frontend needs access.

There is no Alembic migration system yet. The backend currently creates tables at startup with SQLAlchemy metadata.

## WebSocket Flow

The live update path is:

```text
ROS callback
  -> robot_state.update(...)
  -> ws_manager.broadcast(snapshot)
  -> browser WebSocket receives JSON
  -> useRobotSnapshot sets React state
  -> UI rerenders
```

WebSocket endpoint:

```text
/ws/robot-state
```

Frontend socket creation:

```text
ui/src/services/robotApi.js
```

Frontend socket consumption:

```text
ui/src/hooks/useRobotSnapshot.js
```

Backend WebSocket manager:

```text
backend/app/websocket_manager.py
```

## Running The Project

Frontend only, no Jetson or hardware:

```bash
cd /home/tom/SensQ/ui
npm install
npm run dev
```

This is best for layout and UI work.

Full stack on the robot/Jetson:

```bash
cd /home/tom/SensQ
ROS_DISTRO=humble PUBLIC_HOST=<JETSON_IP> ./scripts/dev.sh
```

Example:

```bash
ROS_DISTRO=humble PUBLIC_HOST=192.168.51.216 ./scripts/dev.sh
```

The script starts:

- FastAPI backend on port `8000`
- Vite frontend on port `5200`
- PostgreSQL connection check
- ROS environment sourcing

## Environment Variables

Common variables:

```bash
ROS_DISTRO=humble
PUBLIC_HOST=192.168.51.216
BACKEND_PORT=8000
FRONTEND_PORT=5200
DATABASE_URL=postgresql+asyncpg://sensq:sensq@localhost:5432/sensq
ROS_WORKSPACE=/home/tom/SensQ/ros2_ws
SENSQ_ODOM_TOPIC=/diff_drive_controller/odom
SENSQ_JOINT_STATES_TOPIC=/joint_states
SENSQ_CMD_VEL_TOPIC=/cmd_vel
SENSQ_SERIAL_PORT=/dev/ttyACM0
```

Frontend API overrides:

```bash
VITE_API_BASE_URL=http://localhost:8000
VITE_WS_BASE_URL=ws://localhost:8000
```

## Common Modification Recipes

### Change A UI Label

Search for the label:

```bash
rg "Label text" ui/src
```

Edit the matching page/component.

Run:

```bash
cd ui
npm run lint
npm run build
```

### Add A New Device Status Row

Backend:

1. Add a default row in `backend/app/state.py`.
2. Update that device from `backend/app/ros_monitor.py`.

Frontend:

1. Usually no change is needed because `DeviceStatusPage.jsx` renders `robot.devices`.

### Add A New ROS Topic Display

Backend:

1. Import the message type in `ros_monitor.py`.
2. Create a subscription.
3. Convert the ROS message to plain JSON-safe values.
4. Update `robot_state`.
5. Broadcast the snapshot.

Frontend:

1. Read the new value from `robot`.
2. Display it in the correct page.

### Add A New Button That Commands ROS

Frontend:

1. Add a function to `ui/src/services/robotApi.js`.
2. Add a button to the page.
3. Call the service function on click.

Backend:

1. Add a route in `backend/app/main.py`.
2. Add command logic in the correct manager or `ros_monitor.py`.
3. Return JSON like `{ "ok": true, "message": "..." }`.

### Change Teleop Speed Limits

Update frontend sliders in:

```text
ui/src/pages/MapsPage.jsx
```

Update backend validation in:

```text
backend/app/main.py
```

Update backend clamp in:

```text
backend/app/ros_monitor.py
```

Then test carefully with the robot lifted or in a safe open area.

### Fix Map Marker Direction

Edit:

```text
ui/src/pages/MapsPage.jsx
```

Inside `LiveMapCanvas()`, adjust:

```js
ctx.rotate(-DEG_TO_RAD * yaw + Math.PI / 2);
```

Use these offsets:

```text
+ Math.PI / 2  = rotate 90 degrees
- Math.PI / 2  = rotate -90 degrees
+ Math.PI      = rotate 180 degrees
```

### Add Real Navigation Goal Sending

The UI has goal inputs, but the backend does not yet send a Nav2 goal.

Recommended backend path:

1. Add request model in `backend/app/main.py`.
2. Add route like `POST /api/navigation/goal`.
3. Create a navigation manager file, for example `backend/app/navigation_manager.py`.
4. Use a ROS 2 action client for Nav2 `NavigateToPose`.
5. Broadcast goal status through `robot_state`.

Frontend path:

1. Add `sendNavigationGoal()` to `robotApi.js`.
2. Wire the Maps page Send Goal button to that function.
3. Show goal status in the Navigation panel.

## Debugging Checklist

### Blank UI

Run:

```bash
cd ui
npm install
npm run dev
```

Open the Vite URL, not `index.html` directly.

Check the browser console for import/runtime errors.

### Backend Not Reachable

Check:

```bash
curl -s http://localhost:8000/api/health
```

Check port:

```bash
sudo ss -ltnp | grep ':8000'
```

Kill stale process:

```bash
sudo fuser -k 8000/tcp
```

### Frontend Port Busy

Check:

```bash
sudo ss -ltnp | grep ':5200'
```

Kill stale process:

```bash
sudo fuser -k 5200/tcp
```

Or run another port:

```bash
FRONTEND_PORT=5201 ./scripts/dev.sh
```

### ROS Publisher Is Not Available

Usually means the backend could not import or initialize ROS Python packages.

Check:

```bash
source /opt/ros/humble/setup.bash
source /home/tom/SensQ/ros2_ws/install/setup.bash
cd /home/tom/SensQ/backend
source .venv/bin/activate
python -c "import rclpy, geometry_msgs, numpy; print('ok')"
```

If imports fail, install/fix dependencies in the backend virtualenv.

### Map Does Not Show

Check ROS:

```bash
ros2 topic list | grep /map
ros2 topic echo /map --once
```

Check backend snapshot:

```bash
curl -s http://localhost:8000/api/robot/snapshot
```

Look for:

```text
liveMap.width
liveMap.height
liveMap.data
```

### Robot Marker Does Not Move

Check:

```bash
ros2 topic echo /diff_drive_controller/odom --once
ros2 run tf2_ros tf2_echo map base_footprint
```

If TF is missing, marker may use odom instead of map.

## Development Rules

Keep these rules while modifying the code:

1. Keep ROS-specific logic in the backend.
2. Keep UI pages dependent on `robotApi.js`, not raw URLs everywhere.
3. Keep robot state JSON-safe: strings, numbers, booleans, arrays, objects, or null.
4. Add defaults in `state.py` before using new fields in React.
5. Do not block ROS callbacks with slow database writes.
6. Do not publish unbounded motion commands.
7. Test frontend with `npm run lint` and `npm run build`.
8. Test backend syntax with `python3 -m py_compile backend/app/*.py`.
9. Avoid editing `ros2_ws` unless the task is truly a ROS package change.

## Recommended Learning Path

1. Change a small label in `HomePage.jsx`.
2. Add one new field to `state.py` and display it in `SettingsPage.jsx`.
3. Add one backend health/detail endpoint in `main.py`.
4. Add one frontend function in `robotApi.js` that calls that endpoint.
5. Add a small status card using `MetricCard.jsx`.
6. Add a ROS topic subscription in `ros_monitor.py`.
7. Add a real command endpoint that safely publishes or calls ROS.

Follow that order and you will understand the project from UI to backend to ROS without getting lost in the full stack at once.

