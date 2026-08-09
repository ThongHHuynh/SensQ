import asyncio
import os
import re
from pathlib import Path

from .config import MAP_SAVE_DIR, ROS_DISTRO, ROS_WORKSPACE
from .database import create_saved_map, get_saved_map, list_saved_maps, rename_saved_map, save_event
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


def workspace_maps() -> list[dict]:
    maps_directory = Path(ROS_WORKSPACE) / "maps"
    return [
        {
            "id": f"workspace-{path.stem}",
            "name": path.stem.replace("_", " ").title(),
            "resolution": "ROS workspace map",
            "updated": "From ros2_ws/maps",
            "yamlPath": str(path),
            "imagePath": None,
            "frame": "map",
        }
        for path in sorted(maps_directory.glob("*.yaml"))
        if path.is_file()
    ]


class MappingManager:
    def __init__(self) -> None:
        self._active = False

    async def _call_slam_reset_service(self) -> str | None:
        workspace = Path(ROS_WORKSPACE)
        candidates = [
            "ros2 service call /slam_toolbox/reset std_srvs/srv/Empty {}",
            "ros2 service call /slam_toolbox/clear std_srvs/srv/Empty {}",
        ]

        for service_call in candidates:
            command = (
                f"source /opt/ros/{ROS_DISTRO}/setup.bash && "
                f"source {workspace}/install/setup.bash && "
                f"{service_call}"
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
            try:
                stdout, _ = await asyncio.wait_for(process.communicate(), timeout=4)
            except asyncio.TimeoutError:
                process.kill()
                await process.communicate()
                continue

            output = stdout.decode(errors="replace").strip()
            if process.returncode == 0:
                return output or service_call

        return None

    async def load_saved_maps_into_state(self) -> dict:
        saved_maps = [saved_map_to_payload(saved_map) for saved_map in await list_saved_maps()]
        saved_maps.extend(workspace_maps())
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

    async def reset(self) -> dict:
        self._active = False
        slam_output = await self._call_slam_reset_service()
        snapshot = robot_state.update(
            {
                "navigation": {
                    "mapping": "idle",
                    "state": "Idle",
                    "activeMap": "Live SLAM",
                    "localization": "Waiting",
                },
                "liveMap": {
                    "frame": "map",
                    "width": 0,
                    "height": 0,
                    "resolution": None,
                    "origin": {"x": 0.0, "y": 0.0, "yaw": 0.0},
                    "data": [],
                    "updatedAt": None,
                    "status": "Map reset",
                },
            }
        )
        await save_event("mapping", "Map reset requested")
        await ws_manager.broadcast(snapshot)

        if slam_output:
            return {"ok": True, "mapping": "idle", "message": "Map reset and SLAM clear requested"}
        return {"ok": True, "mapping": "idle", "message": "Map display reset; no SLAM reset service responded"}

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

    async def select_map(self, map_id: str) -> dict:
        selected = await self.resolve_map(map_id)
        if selected is None:
            return {"ok": False, "message": "Map not found"}
        active_name = selected["name"]

        snapshot = robot_state.update({"navigation": {"activeMap": active_name}})
        await save_event("mapping", f"Selected active map '{active_name}'")
        await ws_manager.broadcast(snapshot)
        return {
            "ok": True,
            "message": f"Selected '{active_name}'",
            "activeMap": active_name,
            "map": selected,
        }

    async def resolve_map(self, map_id: str) -> dict | None:
        if map_id == "live":
            return {"id": "live", "name": "Live SLAM", "yamlPath": None}
        if map_id == "maze":
            return {
                "id": "maze",
                "name": "Maze simulation",
                "yamlPath": str(Path(ROS_WORKSPACE) / "maps/simple_maze.yaml"),
            }
        if map_id.startswith("workspace-"):
            stem = map_id.replace("workspace-", "", 1)
            if not re.fullmatch(r"[A-Za-z0-9_-]+", stem):
                return None
            path = (Path(ROS_WORKSPACE) / "maps" / f"{stem}.yaml").resolve()
            maps_directory = (Path(ROS_WORKSPACE) / "maps").resolve()
            if path.parent != maps_directory or not path.is_file():
                return None
            return {
                "id": map_id,
                "name": stem.replace("_", " ").title(),
                "yamlPath": str(path),
            }
        if not map_id.startswith("saved-"):
            return None
        try:
            database_id = int(map_id.replace("saved-", "", 1))
        except ValueError:
            return None
        saved_map = await get_saved_map(database_id)
        if saved_map is None:
            return None
        return saved_map_to_payload(saved_map)

    async def rename(self, map_id: str, name: str) -> dict:
        new_name = name.strip()
        if not new_name:
            return {"ok": False, "message": "Map name is required"}
        if not map_id.startswith("saved-"):
            return {"ok": False, "message": "Only saved maps can be renamed"}

        saved_map = await rename_saved_map(int(map_id.replace("saved-", "", 1)), new_name)
        if saved_map is None:
            return {"ok": False, "message": "Saved map not found"}

        await self.load_saved_maps_into_state()
        snapshot = robot_state.update({"navigation": {"activeMap": new_name}})
        await save_event("mapping", f"Renamed saved map to '{new_name}'")
        await ws_manager.broadcast(snapshot)
        return {"ok": True, "message": f"Renamed map to '{new_name}'", "map": saved_map_to_payload(saved_map)}


mapping_manager = MappingManager()
