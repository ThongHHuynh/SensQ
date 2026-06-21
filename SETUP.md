# SensQ Setup

This guide describes how to run SensQ on a new machine.

## 1. Install System Dependencies

```bash
sudo apt update
sudo apt install -y python3 python3-venv python3-pip postgresql postgresql-contrib
sudo apt install -y python3-colcon-common-extensions
```

Install ROS 2 separately if it is not already installed. The current project has been run with ROS 2 Humble.

The frontend requires Node.js 18 or newer. Ubuntu 22.04 `apt install nodejs` may install Node 12, which is too old for Vite.

Recommended with `nvm`:

```bash
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.1/install.sh | bash
source ~/.bashrc
nvm install 20
nvm use 20
node --version
```

The reported Node version should be `v18` or newer.

## 2. Clone Or Copy The Project

```bash
cd /home/tom
git clone <your-repo-url> SensQ
cd /home/tom/SensQ
```

If the project was copied manually, just enter the project folder:

```bash
cd /home/tom/SensQ
```

## 3. Set Up PostgreSQL

Start PostgreSQL:

```bash
sudo systemctl start postgresql
```

Create the SensQ database user and database:

```bash
sudo -u postgres psql
```

Inside `psql`:

```sql
CREATE USER sensq WITH PASSWORD 'sensq';
CREATE DATABASE sensq OWNER sensq;
\q
```

If the user or database already exists, use:

```sql
ALTER USER sensq WITH PASSWORD 'sensq';
ALTER DATABASE sensq OWNER TO sensq;
\q
```

Only run SQL commands inside the `postgres=#` prompt. After `\q`, you are back in the Linux shell, so SQL commands like `ALTER USER ...` will fail with `bash: ALTER: command not found`.

The backend default database URL is:

```text
postgresql+asyncpg://sensq:sensq@localhost:5432/sensq
```

If you use a different user, password, or database name, run the orchestrator with:

```bash
DATABASE_URL="postgresql+asyncpg://USER:PASSWORD@localhost:5432/DB_NAME" ./scripts/dev.sh
```

## 4. Build The ROS Workspace

For ROS 2 Humble:

```bash
cd /home/tom/SensQ/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
```

For another ROS distro:

```bash
cd /home/tom/SensQ/ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build
```

## 5. Set Up The Backend

```bash
cd /home/tom/SensQ/backend
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## 6. Set Up The Frontend

```bash
cd /home/tom/SensQ/ui
npm install
```

If you previously ran `npm install` with Node 12, remove the installed dependencies and reinstall after switching to Node 18+:

```bash
cd /home/tom/SensQ/ui
rm -rf node_modules package-lock.json
npm install
```

## 7. Run Everything

From the repo root:

```bash
cd /home/tom/SensQ
ROS_DISTRO=humble ./scripts/dev.sh
```

Open:

```text
http://127.0.0.1:5200/
```

If the UI runs on a Jetson and you open it from another computer, use the Jetson IP address:

```bash
hostname -I
```

Then start with:

```bash
ROS_DISTRO=humble PUBLIC_HOST=<JETSON_IP> ./scripts/dev.sh
```

Open:

```text
http://<JETSON_IP>:5200/
```

Without `PUBLIC_HOST`, the frontend uses `localhost` for API calls. From another computer, `localhost` points to that computer, not the Jetson, causing browser `NetworkError when attempting to fetch resource`.

## Web Teleop Troubleshooting

If the teleop process starts but the UI says:

```text
ROS publisher is not available
```

restart the orchestrator after pulling the latest code. The backend creates the `/cmd_vel` publisher independently from optional status subscriptions.

You can also check that the ROS environment is available before running the backend:

```bash
source /opt/ros/humble/setup.bash
source /home/tom/SensQ/ros2_ws/install/setup.bash
ros2 topic list
```

## Live Map Display

The Maps tab renders the ROS occupancy grid from:

```text
/map
```

This matches the normal RViz workflow of setting the fixed frame to `map` and adding the Map display. The backend subscribes to `/map`, downsamples large occupancy grids for browser performance, then streams the map to the frontend over WebSocket.

To verify that SLAM is publishing:

```bash
source /opt/ros/humble/setup.bash
source /home/tom/SensQ/ros2_ws/install/setup.bash
ros2 topic echo /map --once
```

If the Maps tab says it is waiting for `/map`, drive the robot after launching `my_robot.launch.py` and confirm `slam_toolbox` is running.

## Mapping Controls And Saving Maps

The Maps tab has controls for:

- Start Mapping
- Stop Mapping
- Save Map
- custom map name

`my_robot.launch.py` already starts `slam_toolbox`, so Start Mapping marks the backend/UI mapping session active and waits for `/map`.

Save Map calls:

```bash
ros2 run nav2_map_server map_saver_cli -f <map_path>
```

Saved map files are written to:

```text
/home/tom/SensQ/data/maps/
```

The backend also stores saved map metadata in PostgreSQL in the `saved_maps` table. Saved maps then appear in Available Maps.

If saving fails, make sure `nav2_map_server` is installed:

```bash
sudo apt install -y ros-humble-nav2-map-server
```

## PostgreSQL Pool Timeout

If the backend reports:

```text
sqlalchemy.exc.TimeoutError: QueuePool limit of size 5 overflow 10 reached
```

restart the orchestrator after pulling the latest code. The backend streams live robot state over WebSocket but throttles PostgreSQL snapshot history writes so high-rate ROS topics such as `/joint_states`, `/odom`, and `/map` do not exhaust database connections.

The orchestrator starts:

- FastAPI backend on `http://localhost:8000`
- React frontend on `http://127.0.0.1:5200`

## Teleop Package Override

The default teleop command is:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

If your package or executable is different, override it:

```bash
ROS_DISTRO=humble \
SENSQ_TELEOP_PACKAGE=teleop_twist_keyboard \
SENSQ_TELEOP_EXECUTABLE=teleop_twist_keyboard \
./scripts/dev.sh
```

Example for a package named `teleop_2_keyboard`:

```bash
ROS_DISTRO=humble \
SENSQ_TELEOP_PACKAGE=teleop_2_keyboard \
SENSQ_TELEOP_EXECUTABLE=teleop_2_keyboard \
./scripts/dev.sh
```

## ESP32 Serial Access

The launch file expects the ESP32 at:

```text
/dev/ttyACM0
```

Check that it exists:

```bash
ls /dev/ttyACM0
```

If access is denied, add your user to the `dialout` group:

```bash
sudo usermod -aG dialout $USER
```

Then log out and log back in.

## Common Checks

Check PostgreSQL:

```bash
sudo systemctl status postgresql
```

Check the ROS workspace install file:

```bash
ls /home/tom/SensQ/ros2_ws/install/setup.bash
```

Check backend dependencies:

```bash
cd /home/tom/SensQ/backend
source .venv/bin/activate
python3 -m py_compile app/*.py
```

If the backend reports one of these Jetson errors:

```text
ModuleNotFoundError: No module named 'asyncpg.protocol.protocol'
ModuleNotFoundError: No module named 'pydantic_core._pydantic_core'
ValueError: the greenlet library is required to use this function. No module named 'greenlet._greenlet'
ROS command publisher disabled: No module named 'numpy'
```

then a compiled Python dependency is missing or corrupted inside the virtualenv. Repair it with:

```bash
cd /home/tom/SensQ/backend
source .venv/bin/activate
pip install --upgrade pip setuptools wheel
pip install --force-reinstall --no-cache-dir asyncpg greenlet pydantic-core pydantic numpy
pip install -r requirements.txt
```

If reinstalling keeps failing, recreate the virtualenv:

```bash
cd /home/tom/SensQ/backend
deactivate 2>/dev/null || true
rm -rf .venv
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip setuptools wheel
pip install -r requirements.txt
```

If packages build from source and fail, install build tools and retry:

```bash
sudo apt install -y build-essential python3-dev rustc cargo
cd /home/tom/SensQ/backend
source .venv/bin/activate
pip install --force-reinstall --no-cache-dir asyncpg greenlet pydantic-core pydantic numpy
```

Check frontend:

```bash
cd /home/tom/SensQ/ui
npm run lint
npm run build
```

## Stopping An Existing ROS Launch

The UI Stop button can stop a launch started by the backend. It also checks for an already-running command:

```bash
ros2 launch my_robot_bringup my_robot.launch.py
```

and signals that process group. This lets the website stop lidar, SLAM, ros2_control, and related child nodes even if the launch was started before the website.

If a manually started launch still remains, inspect it:

```bash
pgrep -af "ros2 launch my_robot_bringup my_robot.launch.py"
```
