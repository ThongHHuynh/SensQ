# SensQ Backend

FastAPI backend for the SensQ robot UI. The MVP launches `my_robot.launch.py`, listens to ROS 2 topics, stores events/snapshots in PostgreSQL, and streams live robot state to the React UI.

## Architecture

```text
React UI
  -> REST commands + WebSocket state
FastAPI backend
  -> launches my_robot.launch.py
  -> rclpy subscriptions
  -> PostgreSQL event/snapshot storage
ROS 2 workspace
  -> ros2_control, ESP32 serial hardware interface, lidar, SLAM
```

## PostgreSQL Setup

Create a database and user:

```bash
sudo -u postgres psql
```

```sql
CREATE USER sensq WITH PASSWORD 'sensq';
CREATE DATABASE sensq OWNER sensq;
\q
```

The default backend URL is:

```text
postgresql+asyncpg://sensq:sensq@localhost:5432/sensq
```

Override it with:

```bash
export DATABASE_URL="postgresql+asyncpg://sensq:sensq@localhost:5432/sensq"
```

If startup reports a password failure for user `sensq`, reset the local database password or run with the correct `DATABASE_URL`:

```bash
sudo -u postgres psql
```

```sql
ALTER USER sensq WITH PASSWORD 'sensq';
\q
```

## Run

Build the ROS workspace first if needed:

```bash
cd /home/tom/SensQ/ros2_ws
colcon build
```

Start the backend from the repo root:

```bash
cd /home/tom/SensQ/backend
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
source /opt/ros/$ROS_DISTRO/setup.bash
source /home/tom/SensQ/ros2_ws/install/setup.bash
uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
```

Start the UI:

```bash
cd /home/tom/SensQ/ui
npm run dev
```

Or start backend and frontend together from the repo root:

```bash
./scripts/dev.sh
```

## MVP Flow

1. Open the UI.
2. Click `Start Mobile Base` on the Home page.
3. The UI calls `POST /api/robot/launch/mobile-base`.
4. FastAPI starts:

```bash
ros2 launch my_robot_bringup my_robot.launch.py
```

5. The backend listens for:

- `/hardware_status`
- `/joint_states`
- `/diff_drive_controller/odom`
- `/scan`
- `/imu`

6. The backend streams updates to:

```text
ws://localhost:8000/ws/robot-state
```

7. The Device Status page shows `ESP32` as online once real hardware status is received through the serial-connected mobile base.

## Step-by-Step Implementation Summary

1. Added a separate `backend/` FastAPI project.
2. Added PostgreSQL persistence with SQLAlchemy async models:
   - `robot_events`
   - `robot_snapshots`
3. Added `LaunchManager` to start and stop `my_robot.launch.py`.
4. Added `RosMonitor` using `rclpy` subscriptions for live robot updates.
5. Added REST endpoints for health, snapshot, launch, and stop.
6. Added WebSocket streaming at `/ws/robot-state`.
7. Updated the React UI service layer to consume FastAPI when available.
8. Added an `ESP32` device row in the frontend status model.

## API

- `GET /api/health`
- `GET /api/robot/snapshot`
- `POST /api/robot/launch/mobile-base`
- `POST /api/robot/stop`
- `POST /api/teleop/start`
- `POST /api/teleop/stop`
- `POST /api/teleop/cmd_vel`
- `WS /ws/robot-state`

## Teleoperation

The Maps page includes a teleoperation panel. `Start Teleop` launches:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

The web buttons publish bounded `geometry_msgs/Twist` commands to `/cmd_vel` through the backend.

If your local package/executable name is different, override it before running the orchestrator:

```bash
SENSQ_TELEOP_PACKAGE=teleop_twist_keyboard \
SENSQ_TELEOP_EXECUTABLE=teleop_twist_keyboard \
./scripts/dev.sh
```

For example, if your package is actually named `teleop_2_keyboard`, run:

```bash
SENSQ_TELEOP_PACKAGE=teleop_2_keyboard \
SENSQ_TELEOP_EXECUTABLE=teleop_2_keyboard \
./scripts/dev.sh
```

## Notes

- The backend assumes the ESP32 is connected at `/dev/ttyACM0`, matching `my_robot.launch.py`.
- Override the serial port for display/status metadata with `SENSQ_SERIAL_PORT`.
- The ROS launch file still owns the real hardware connection decision.
