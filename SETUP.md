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

Check frontend:

```bash
cd /home/tom/SensQ/ui
npm run lint
npm run build
```
