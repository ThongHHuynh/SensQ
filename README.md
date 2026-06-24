# SensQ Robot System Architecture

## Goal

SensQ uses a web UI, backend API, database, and ROS 2 robot stack to let users monitor devices, view camera frames, run mapping, save maps, and send navigation commands.

The main rule:

> The frontend should never directly control ROS topics, services, actions, hardware, or shell commands.

Instead:

```text
Frontend UI
   ↓ HTTP / WebSocket / Video Stream
Backend API
   ↓ Controlled ROS gateway calls
ROS 2 System
   ↓
Robot hardware
```

---

## Overall Architecture

```text
SensQ/
├── apps/
│   └── operator-ui/          # React frontend
│
├── services/
│   └── api/                  # FastAPI backend
│
├── ros2_ws/
│   └── src/
│       ├── sensq_bringup/    # Launch files
│       ├── sensq_description/# URDF/Xacro
│       ├── sensq_hardware/   # ros2_control hardware interface
│       ├── sensq_navigation/ # Nav2 + SLAM configs
│       └── sensq_gateway/    # ROS gateway node
│
├── config/
│   ├── shared/               # Committed default configs
│   └── local/                # Gitignored machine-specific configs
│
├── data/                     # Gitignored maps, bags, recordings
└── docs/
```

---

## Frontend

Recommended tools:

- React
- TypeScript
- Vite
- Tailwind CSS
- Axios or Fetch API
- WebSocket client

Frontend responsibilities:

- Display dashboard
- Show device status
- Show camera stream
- Show camera dummy frame in Visualization until ROS image topics are wired
- Show robot mode and errors
- Send approved user commands to backend
- Display mapping/navigation state with pan, zoom, and center controls
- Keep temporary UI state only

Frontend should store:

- Current selected page/tab
- Temporary form input
- Current selected robot/map
- Short-lived UI state
- Auth token if login is added later

Frontend should not store:

- Raw ROS data history
- Camera frames
- Maps as source of truth
- Robot configuration source of truth
- Secrets or database credentials

---

## Backend

Recommended tools:

- FastAPI
- PostgreSQL
- SQLAlchemy
- Alembic migrations
- Pydantic schemas
- WebSocket endpoint
- Optional Redis later for caching or queues

Backend responsibilities:

- Authentication and permissions
- Validate user commands
- Store persistent records
- Keep audit logs
- Talk to ROS gateway
- Stream robot status to frontend
- Manage map metadata and mission history

Backend should store in PostgreSQL:

- Users and roles
- Robot records
- Device configuration
- Last known device status
- Saved map metadata
- Navigation mission history
- Command history
- Error and event logs
- Settings

Backend should not store at full rate:

- Raw camera frames
- Raw `/scan`
- Raw `/tf`
- Every `/odom` message
- Every `/cmd_vel` message

For high-rate history, use ROS bags or separate time-series logging.

## Launch Orchestrator

Use the repo-level script to run the backend and frontend together:

```bash
./scripts/dev.sh
```

The script starts:

- FastAPI backend on `http://localhost:8000`
- React/Vite frontend on `http://127.0.0.1:5200`

Before running it the first time, install dependencies:

```bash
cd backend
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt

cd ../ui
npm install
```

Make sure PostgreSQL is running and the `sensq` database exists. If the ROS workspace has not been built yet:

```bash
cd /home/tom/SensQ/ros2_ws
colcon build
```

You can override ports:

```bash
BACKEND_PORT=8001 FRONTEND_PORT=5201 ./scripts/dev.sh
```

---

## ROS 2 Layer

Recommended tools:

- ROS 2 Humble
- `rclpy` or `rclcpp`
- `ros2_control`
- `diff_drive_controller`
- `slam_toolbox`
- Nav2
- Camera driver
- Lidar driver
- `robot_state_publisher`
- TF

ROS responsibilities:

- Hardware control
- Sensor publishing
- SLAM
- Localization
- Navigation
- TF tree
- Robot state
- Low-level safety behavior

Real robot floor Nav2:

```bash
cd /home/tom/SensQ/ros2_ws
ros2 launch my_robot_bringup real_nav2_floor.launch.py \
  serial_port:=/dev/ttyACM0 lidar_serial_port:=/dev/ttyUSB0
```

Use `lidar_serial_baudrate:=115200`, `256000`, or `460800` to match the SLLidar model.
This launch runs Nav2 with real time. `/map` comes from `map_server`, and AMCL publishes `map -> odom` after it has the map, `/scan`, odom TF, and an initial pose.
The floor launch defaults to `ros2_ws/maps/my_map.yaml`. Global and local costmaps use the map plus live lidar and inflation layers, so they will not look identical to the raw YAML map.
For a smaller Nav2 config, pass `params_file:=/home/tom/SensQ/ros2_ws/src/my_robot_bringup/config/nav2_config.yaml`.
That file keeps the Humble BT helper nodes required by this setup.

ROS interfaces:

| Feature | ROS Interface |
|---|---|
| Manual drive | Publish `/cmd_vel` |
| Robot pose | Subscribe `/odom` and TF |
| Lidar | Subscribe `/scan` |
| Camera | Subscribe camera image topic |
| Mapping | `slam_toolbox` |
| Save map | SLAM/map save service |
| Reset map | `POST /api/mapping/reset` clears the live map and asks SLAM to reset |
| Navigation | Nav2 `NavigateToPose` action |
| Cancel navigation | Cancel Nav2 action |
| Controller status | `ros2_control` + diagnostics |

---

## ROS Gateway

The ROS gateway is the safe bridge between backend and ROS.

It should expose only approved robot operations, not arbitrary ROS commands.

Example gateway operations:

```text
get_robot_status()
get_device_status()
start_mapping()
save_map(name)
send_navigation_goal(x, y, yaw)
cancel_navigation()
send_safe_cmd_vel(linear, angular)
stop_robot()
```

Gateway responsibilities:

- Clamp speed commands
- Reject unsafe commands
- Convert backend requests to ROS topics/services/actions
- Convert ROS feedback to backend status events
- Monitor ROS node/topic health

---

## Communication Workflow

### Device Status

```text
Frontend opens Status page
   ↓ GET /api/v1/devices/status
Backend scans configured devices or asks ROS gateway
   ↓
Backend returns ESP32, LIDAR, camera status
   ↓
Frontend displays device cards
```

### Camera Stream

```text
Camera publishes frames
   ↓
ROS/camera service or media gateway reads frames
   ↓
Backend exposes MJPEG/WebRTC stream
   ↓
Frontend displays stream when user clicks camera card
```

For MVP, MJPEG is okay. For production, prefer WebRTC.

### Save Map

```text
User clicks Save Map
   ↓ POST /api/v1/maps/save
Backend validates map name and robot state
   ↓
ROS gateway calls map save service
   ↓
Map files saved to data/maps/
   ↓
Backend stores map metadata in PostgreSQL
   ↓
Frontend shows success/failure
```

### Navigation Goal

```text
User selects goal on map
   ↓ POST /api/v1/navigation/goals
Backend validates coordinates and robot state
   ↓
ROS gateway sends Nav2 NavigateToPose action
   ↓
Nav2 drives robot
   ↓
Gateway sends feedback to backend
   ↓
Backend streams status to frontend over WebSocket
```

### Emergency Stop / Stop Button

```text
User clicks Stop
   ↓ POST /api/v1/robot/stop
Backend validates command
   ↓
ROS gateway cancels navigation and publishes zero /cmd_vel
   ↓
Robot stops
```

A web stop button is not a replacement for a physical hardware E-stop.

---

## API Pattern

Use REST for commands:

```text
GET  /api/v1/robots
GET  /api/v1/devices/status
POST /api/v1/mapping/start
POST /api/v1/maps/save
POST /api/v1/navigation/goals
POST /api/v1/navigation/cancel
POST /api/v1/robot/stop
```

Use WebSocket for live status:

```text
WS /ws/v1/robot-status
```

Use video stream endpoint for camera:

```text
GET /api/v1/camera/stream
```

---

## Data Ownership

| Data | Owner |
|---|---|
| UI tab/page state | Frontend |
| User settings | Backend/PostgreSQL |
| Robot configs | Backend + config files |
| Device ports | Local config + backend status |
| Saved map files | `data/maps/` |
| Map metadata | PostgreSQL |
| Live odom/scan/tf | ROS |
| Camera stream | ROS/media stream |
| Command history | PostgreSQL |
| Navigation feedback | ROS → Backend → WebSocket |
| Hardware control | ROS |

---

## Development Run Order

Terminal 1: ROS bringup

```bash
cd ~/SensQ/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch sensq_bringup bringup.launch.py
```

Terminal 2: ROS gateway

```bash
cd ~/SensQ/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run sensq_gateway gateway_node
```

Terminal 3: Backend

```bash
cd ~/SensQ/services/api
source .venv/bin/activate
uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
```

Terminal 4: Frontend

```bash
cd ~/SensQ/apps/operator-ui
npm run dev -- --host 0.0.0.0
```

---

## Implementation Order

1. Build frontend layout: right navigation, Home, Status, Settings.
2. Build backend `/api/v1/devices/status`.
3. Show ESP32, LIDAR, and camera status.
4. Add camera stream preview.
5. Add ROS gateway node.
6. Connect backend to ROS gateway.
7. Add mapping start/save.
8. Add Nav2 navigation goal.
9. Add PostgreSQL records for maps, commands, and events.
10. Add authentication and permissions.

---

## Design Rules

1. Frontend talks only to backend.
2. Backend validates all user commands.
3. ROS gateway exposes only approved ROS actions.
4. PostgreSQL stores persistent records, not raw sensor streams.
5. ROS owns live robot data and hardware control.
6. Use REST for commands, WebSocket for live status, and video streaming for camera.
7. Keep machine-specific config out of Git.
8. Use a physical hardware E-stop for real robot safety.
