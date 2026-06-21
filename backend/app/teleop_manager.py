import asyncio
import os
from pathlib import Path

from .config import ROS_DISTRO, ROS_WORKSPACE, TELEOP_EXECUTABLE, TELEOP_PACKAGE
from .database import save_event


class TeleopManager:
    def __init__(self) -> None:
        self._process: asyncio.subprocess.Process | None = None
        self._log_task: asyncio.Task | None = None

    @property
    def is_running(self) -> bool:
        return self._process is not None and self._process.returncode is None

    async def start(self) -> dict:
        if self.is_running:
            return {"ok": True, "running": True, "message": "Teleop process already running"}

        workspace = Path(ROS_WORKSPACE)
        command = (
            f"source /opt/ros/{ROS_DISTRO}/setup.bash && "
            f"source {workspace}/install/setup.bash && "
            f"ros2 run {TELEOP_PACKAGE} {TELEOP_EXECUTABLE}"
        )

        self._process = await asyncio.create_subprocess_exec(
            "bash",
            "-lc",
            command,
            cwd=str(workspace),
            env=os.environ.copy(),
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.STDOUT,
        )
        await save_event("teleop", f"Started ros2 run {TELEOP_PACKAGE} {TELEOP_EXECUTABLE}")
        self._log_task = asyncio.create_task(self._read_logs())

        await asyncio.sleep(0.4)
        if self._process.returncode is not None:
            message = f"Teleop exited with code {self._process.returncode}"
            await save_event("teleop", message, "error")
            return {"ok": False, "running": False, "message": message}

        return {"ok": True, "running": True, "message": f"Started {TELEOP_PACKAGE}/{TELEOP_EXECUTABLE}"}

    async def stop(self) -> dict:
        if self._process and self._process.returncode is None:
            self._process.terminate()
            try:
                await asyncio.wait_for(self._process.wait(), timeout=3)
            except asyncio.TimeoutError:
                self._process.kill()
                await self._process.wait()

        if self._log_task:
            self._log_task.cancel()

        await save_event("teleop", "Stopped teleop process")
        return {"ok": True, "running": False, "message": "Teleop stopped"}

    async def _read_logs(self) -> None:
        assert self._process and self._process.stdout
        async for raw_line in self._process.stdout:
            line = raw_line.decode(errors="replace").strip()
            if line:
                await save_event("teleop", line)


teleop_manager = TeleopManager()
