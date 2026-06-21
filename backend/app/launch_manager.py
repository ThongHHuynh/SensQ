import asyncio
import os
from pathlib import Path

from .config import ROS_DISTRO, ROS_WORKSPACE, SERIAL_PORT
from .database import save_event
from .state import robot_state
from .websocket_manager import ws_manager


class LaunchManager:
    def __init__(self) -> None:
        self._process: asyncio.subprocess.Process | None = None
        self._log_task: asyncio.Task | None = None

    @property
    def is_running(self) -> bool:
        return self._process is not None and self._process.returncode is None

    async def start_mobile_base(self) -> dict:
        if self.is_running:
            return robot_state.get_snapshot()

        workspace = Path(ROS_WORKSPACE)
        serial_exists = os.path.exists(SERIAL_PORT)
        serial_accessible = serial_exists and os.access(SERIAL_PORT, os.R_OK | os.W_OK)
        command = (
            f"source /opt/ros/{ROS_DISTRO}/setup.bash && "
            f"source {workspace}/install/setup.bash && "
            "ros2 launch my_robot_bringup my_robot.launch.py"
        )

        env = os.environ.copy()
        self._process = await asyncio.create_subprocess_exec(
            "bash",
            "-lc",
            command,
            cwd=str(workspace),
            env=env,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.STDOUT,
        )

        snapshot = robot_state.set_launch_state("starting")
        if serial_accessible:
            robot_state.update_device("ESP32", "warning", f"Serial device found, waiting for hardware activation at {SERIAL_PORT}", SERIAL_PORT)
        elif serial_exists:
            robot_state.update_device("ESP32", "warning", f"Serial device exists but is not readable/writable: {SERIAL_PORT}", SERIAL_PORT)
        else:
            robot_state.update_device("ESP32", "offline", f"{SERIAL_PORT} not found; launch will use mock hardware", SERIAL_PORT)
            robot_state.update_device("Mobile base", "warning", "Mock hardware selected by launch file")
        await save_event("launch", "Started my_robot.launch.py")
        await ws_manager.broadcast(snapshot)

        self._log_task = asyncio.create_task(self._read_logs())
        return robot_state.get_snapshot()

    async def stop(self) -> dict:
        if self._process and self._process.returncode is None:
            self._process.terminate()
            try:
                await asyncio.wait_for(self._process.wait(), timeout=8)
            except asyncio.TimeoutError:
                self._process.kill()
                await self._process.wait()

        if self._log_task:
            self._log_task.cancel()

        snapshot = robot_state.set_launch_state("stopped")
        robot_state.update_device("Mobile base", "offline", "Launch stopped")
        robot_state.update_device("ESP32", "offline", "Launch stopped", SERIAL_PORT)
        await save_event("launch", "Stopped my_robot.launch.py")
        await ws_manager.broadcast(snapshot)
        return robot_state.get_snapshot()

    async def _read_logs(self) -> None:
        assert self._process and self._process.stdout

        async for raw_line in self._process.stdout:
            line = raw_line.decode(errors="replace").strip()
            if not line:
                continue

            await save_event("ros_launch", line)
            lower = line.lower()
            if "opening serial" in lower:
                snapshot = robot_state.set_launch_state("running")
                robot_state.update_device("ESP32", "online", "Hardware interface opened serial port", SERIAL_PORT)
                robot_state.update_device("Mobile base", "warning", "Hardware interface active; waiting for joint feedback")
                await ws_manager.broadcast(snapshot)
            elif "failed to open serial" in lower:
                snapshot = robot_state.set_launch_state("error")
                robot_state.update_device("ESP32", "offline", line, SERIAL_PORT)
                robot_state.update_device("Mobile base", "offline", "Serial hardware activation failed")
                await ws_manager.broadcast(snapshot)
            elif "controller_manager" in lower or "diff_drive_controller" in lower:
                snapshot = robot_state.update_device("ros2_control", "online", "Controller launch output detected")
                await ws_manager.broadcast(snapshot)

        return_code = await self._process.wait()
        if return_code != 0:
            snapshot = robot_state.set_launch_state("error")
            await save_event("launch", f"my_robot.launch.py exited with code {return_code}", "error")
        else:
            snapshot = robot_state.set_launch_state("stopped")
            await save_event("launch", "my_robot.launch.py exited")

        await ws_manager.broadcast(snapshot)


launch_manager = LaunchManager()
