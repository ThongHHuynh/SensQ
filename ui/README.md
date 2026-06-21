# SensQ UI

React + Tailwind frontend for the SensQ robot control interface. This folder is separate from `ros2_ws` and currently uses mock data so the UI can evolve before the backend and ROS bridge are connected.

## Implemented Steps

1. Created a top-level `ui/` folder without modifying `ros2_ws`.
2. Added a Vite React project structure.
3. Added Tailwind CSS configuration and shared global styles.
4. Built a responsive left navigation layout with Home, Device Status, Maps, and Settings tabs.
5. Added mock robot data based on current ROS references, including `HardwareStatus.msg`.
6. Added a `services/robotApi.js` boundary so page components do not depend directly on ROS or backend details.

## Project Structure

```text
ui/
  index.html
  package.json
  postcss.config.js
  tailwind.config.js
  src/
    components/       Shared UI components
    data/             Mock robot data used before backend integration
    layout/           App shell and navigation
    pages/            Home, Device Status, Maps, Settings screens
    services/         Future backend API boundary
    styles/           Tailwind entrypoint and global CSS
```

## Architecture

The frontend is intentionally separated from ROS. React pages read robot state through `src/services/robotApi.js`. Today that service returns mock data from `src/data/mockRobot.js`; later it can call a backend endpoint such as `/api/robot/snapshot` or subscribe to a WebSocket stream without changing the page layout.

Recommended future flow:

```text
React UI -> Backend API/WebSocket -> ROS 2 bridge/node -> ROS topics/services/actions
```

Suggested backend responsibilities:

- Expose robot snapshot data for battery, pose, hardware, and navigation.
- Translate UI commands into ROS 2 services/actions.
- Stream topic updates to the browser over WebSocket.
- Keep safety checks server-side before motion commands are sent to ROS.

## Run Locally

Install dependencies:

```bash
cd ui
npm install
```

Start the development server:

```bash
npm run dev
```

Then open the Vite URL shown in the terminal, usually `http://localhost:5173`.

If that page is blank, make sure you opened the `http://localhost:...` URL from the terminal output. Do not open `ui/index.html` directly from the file browser; Vite must serve the React modules.

To run the backend and frontend together from the repo root:

```bash
./scripts/dev.sh
```

## Notes For ROS Integration

- `Device Status` includes fields from `ros2_ws/src/my_robot_interfaces/msg/HardwareStatus.msg`.
- `Device Status` includes an `ESP32` row for the serial-connected controller used by `my_robot.launch.py`.
- `Maps` is prepared for future Nav2, SLAM, saved map, and goal-setting workflows.
- `Settings` includes backend URL, `ROS_DOMAIN_ID`, and realtime transport placeholders.
- No files inside `ros2_ws` were changed.

## Backend Integration

By default the UI tries to connect to:

```text
http://localhost:8000
ws://localhost:8000/ws/robot-state
```

Override these when starting Vite:

```bash
VITE_API_BASE_URL=http://localhost:8000 VITE_WS_BASE_URL=ws://localhost:8000 npm run dev
```

When the backend is unavailable, the UI falls back to mock data.
