import asyncio
import os
import re
from pathlib import Path

from .config import MAP_SAVE_DIR, ROS_DISTRO, ROS_WORKSPACE
from .database import create_saved_map, list_saved_maps, save_event
from .state import robot_state
from .websocket_manager import ws_manager


def slugify_map_name(name: str) -> str:
    slug = re.sub(r"[^a-zA-Z0-9_-]+", "_", name.strip()).strip("_").lower()
    return slug or "sensq_map"


def saved_map_to_payload(saved_map) -> dict:
    return {
        "id": f"saved-{saved_map.id}",
        "name": saved_map.name,
        "resolution": saved_map.resolution,
        "updated": saved_map.created_at.isoformat() if saved_map.created_at else "Saved",
        "yamlPath": saved_map.yaml_path,
        "imagePath": saved_map.image_path,
        "frame": saved_map.frame_id,
    }


class MappingManager:
    def __init__(self) -> None:
        self._active = False

    async def load_saved_maps_into_state(self) -> dict:
        saved_maps = [saved_map_to_payload(saved_map) for saved_map in await list_saved_maps()]
        return robot_state.set_saved_maps(saved_maps)

    async def start(self) -> dict:
        self._active = True
        snapshot = robot_state.update({"navigation": {"mapping": "active", "state": "Mapping", "activeMap": "Live SLAM"}})
        await save_event("mapping", "Mapping session started")
        await ws_manager.broadcast(snapshot)
        return {"ok": True, "mapping": "active", "message": "Mapping session started"}

    async def stop(self) -> dict:
        self._active = False
        snapshot = robot_state.update({"navigation": {"mapping": "idle", "state": "Idle"}})
        await save_event("mapping", "Mapping session stopped")
        await ws_manager.broadcast(snapshot)
        return {"ok": True, "mapping": "idle", "message": "Mapping session stopped"}

    async def save(self, name: str) -> dict:
        map_name = name.strip()
        if not map_name:
            return {"ok": False, "message": "Map name is required"}

        snapshot = robot_state.update({"navigation": {"mapping": "saving"}})
        await ws_manager.broadcast(snapshot)

        MAP_SAVE_DIR.mkdir(parents=True, exist_ok=True)
        slug = slugify_map_name(map_name)
        map_prefix = MAP_SAVE_DIR / slug
        suffix = 1
        while Path(f"{map_prefix}.yaml").exists() or Path(f"{map_prefix}.pgm").exists():
            suffix += 1
            map_prefix = MAP_SAVE_DIR / f"{slug}_{suffix}"

        workspace = Path(ROS_WORKSPACE)
        command = (
            f"source /opt/ros/{ROS_DISTRO}/setup.bash && "
            f"source {workspace}/install/setup.bash && "
            f"ros2 run nav2_map_server map_saver_cli -f {map_prefix}"
        )

        process = await asyncio.create_subprocess_exec(
            "bash",
            "-lc",
            command,
            cwd=str(workspace),
            env=os.environ.copy(),
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.STDOUT,
        )
        stdout, _ = await process.communicate()
        output = stdout.decode(errors="replace").strip()

        if process.returncode != 0:
            snapshot = robot_state.update({"navigation": {"mapping": "error"}})
            await ws_manager.broadcast(snapshot)
            await save_event("mapping", f"Map save failed: {output}", "error")
            return {"ok": False, "message": output or f"map_saver_cli exited with code {process.returncode}"}

        yaml_path = f"{map_prefix}.yaml"
        image_path = f"{map_prefix}.pgm"
        saved_map = await create_saved_map(map_name, yaml_path, image_path, resolution="Saved from /map")
        await self.load_saved_maps_into_state()
        snapshot = robot_state.update({"navigation": {"mapping": "active" if self._active else "idle"}})
        await save_event("mapping", f"Saved map '{map_name}' to {yaml_path}")
        await ws_manager.broadcast(snapshot)
        return {"ok": True, "message": f"Saved map '{map_name}'", "map": saved_map_to_payload(saved_map)}


mapping_manager = MappingManager()
