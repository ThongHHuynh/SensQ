import asyncio
import os
import signal
from pathlib import Path

from .config import ROS_DISTRO, ROS_WORKSPACE, USE_SIM_TIME
from .database import save_event


class CoverageManager:
    def __init__(self) -> None:
        self._process: asyncio.subprocess.Process | None = None
        self._log_task: asyncio.Task | None = None

    @property
    def is_running(self) -> bool:
        return self._process is not None and self._process.returncode is None

    async def start(self, use_sim_time: bool | None = None) -> dict:
        if self.is_running:
            return {"ok": True, "running": True, "message": "Coverage services are running"}

        simulation_clock = USE_SIM_TIME if use_sim_time is None else use_sim_time
        workspace = Path(ROS_WORKSPACE)
        command = (
            f"source /opt/ros/{ROS_DISTRO}/setup.bash && "
            f"source {workspace}/install/setup.bash && "
            "ros2 launch test_coverage coverage.launch.py "
            f"use_sim_time:={'true' if simulation_clock else 'false'} "
            "auto_generate:=false auto_start:=false use_rviz:=false"
        )
        self._process = await asyncio.create_subprocess_exec(
            "bash",
            "-lc",
            command,
            cwd=str(workspace),
            env=os.environ.copy(),
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.STDOUT,
            start_new_session=True,
        )
        self._log_task = asyncio.create_task(self._read_logs())
        await save_event("coverage", "Started coverage planner and executor")
        await asyncio.sleep(0.5)
        if self._process.returncode is not None:
            return {
                "ok": False,
                "running": False,
                "message": f"Coverage launch exited with code {self._process.returncode}",
            }
        return {"ok": True, "running": True, "message": "Coverage planner is starting"}

    async def stop(self) -> dict:
        if self._process is not None and self._process.returncode is None:
            try:
                os.killpg(self._process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                await asyncio.wait_for(self._process.wait(), timeout=5)
            except asyncio.TimeoutError:
                try:
                    os.killpg(self._process.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                await self._process.wait()
        if self._log_task:
            self._log_task.cancel()
        await save_event("coverage", "Stopped coverage planner and executor")
        return {"ok": True, "running": False, "message": "Coverage services stopped"}

    async def _read_logs(self) -> None:
        assert self._process is not None and self._process.stdout is not None
        async for raw_line in self._process.stdout:
            line = raw_line.decode(errors="replace").strip()
            if line:
                await save_event("coverage_launch", line)


coverage_manager = CoverageManager()
