import math
import os
from pathlib import Path
import re
import tempfile
from copy import deepcopy
from threading import RLock

import yaml

from .config import DOCK_DATABASE_FILE, DOCKING_CONFIG_FILE


class DockingConfigError(ValueError):
    pass


def _read_yaml(path: Path) -> dict:
    try:
        with path.open("r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream)
    except (OSError, yaml.YAMLError) as error:
        raise DockingConfigError(f"Could not read {path}: {error}") from error
    if not isinstance(data, dict):
        raise DockingConfigError(f"{path.name} must contain a YAML mapping")
    return data


class DockingCatalog:
    def __init__(
        self,
        database_path: Path = DOCK_DATABASE_FILE,
        config_path: Path = DOCKING_CONFIG_FILE,
    ) -> None:
        self._database_path = database_path
        self._config_path = config_path
        self._lock = RLock()
        self._payload = self._load()

    @property
    def action_name(self) -> str:
        return str(self._payload["serverConfig"].get("dock_action", "/dock"))

    def get_payload(self) -> dict:
        with self._lock:
            return deepcopy(self._payload)

    def reload(self) -> dict:
        with self._lock:
            self._payload = self._load()
            return deepcopy(self._payload)

    def tag_frames(self) -> dict[int, str]:
        with self._lock:
            return {
                int(dock["tag_id"]): str(dock["tag_frame"])
                for dock in self._payload["docks"]
            }

    def save_station(self, station: dict) -> dict:
        dock_id = str(station.get("dock_id", "")).strip()
        if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,63}", dock_id):
            raise DockingConfigError(
                "Station name must use letters, numbers, dashes, or underscores"
            )

        tag_id = station.get("tag_id")
        if isinstance(tag_id, bool) or not isinstance(tag_id, int) or tag_id < 0:
            raise DockingConfigError("tag_id must be a non-negative integer")

        tag_frame = str(station.get("tag_frame") or f"tag_{tag_id}").strip()
        if not tag_frame:
            raise DockingConfigError("tag_frame is required")

        reference_pose = self._pose(station.get("reference_pose"), "reference_pose")
        staging_pose = self._pose(station.get("staging_pose"), "staging_pose")
        final_distance = self._finite(station.get("final_distance"), "final_distance")
        lateral_offset = self._finite(station.get("lateral_offset", 0.0), "lateral_offset")
        yaw_offset = self._finite(station.get("yaw_offset", 0.0), "yaw_offset")
        reverse_docking = bool(station.get("reverse_docking", False))
        minimum_distance = float(
            self._payload["serverConfig"].get("minimum_tag_distance", 0.0)
        )
        if final_distance < minimum_distance:
            raise DockingConfigError(
                f"final_distance must be at least {minimum_distance:g} m"
            )

        with self._lock:
            database = _read_yaml(self._database_path)
            docks = database.setdefault("docks", {})
            if not isinstance(docks, dict):
                raise DockingConfigError("Dock database docks field must be a mapping")

            for existing_id, existing in docks.items():
                if (
                    existing_id != dock_id
                    and isinstance(existing, dict)
                    and existing.get("tag_id") == tag_id
                ):
                    raise DockingConfigError(
                        f"AprilTag {tag_id} is already assigned to {existing_id}"
                    )

            docks[dock_id] = {
                "tag_id": tag_id,
                "tag_frame": tag_frame,
                "global_frame": "map",
                "reference_pose": reference_pose,
                "staging_pose": staging_pose,
                "final_distance": final_distance,
                "lateral_offset": lateral_offset,
                "yaw_offset": yaw_offset,
                "reverse_docking": reverse_docking,
            }
            self._write_database(database)
            self._payload = self._load()
            return next(
                deepcopy(dock)
                for dock in self._payload["docks"]
                if dock["dockId"] == dock_id
            )

    def normalize_goal(self, request: dict) -> dict:
        with self._lock:
            payload = deepcopy(self._payload)
        docks = {dock["dockId"]: dock for dock in payload["docks"]}
        dock_id = request.get("dock_id")
        if dock_id not in docks:
            raise DockingConfigError("Choose a configured dock")

        dock = docks[dock_id]
        use_override = bool(request.get("use_offset_override", False))
        values = {}
        for field in ("final_distance", "lateral_offset", "yaw_offset"):
            requested_value = request.get(field)
            value = float(
                dock[field] if requested_value is None else requested_value
            )
            if not math.isfinite(value):
                raise DockingConfigError(f"{field} must be finite")
            values[field] = value

        minimum_distance = float(
            payload["serverConfig"].get("minimum_tag_distance", 0.0)
        )
        if use_override and values["final_distance"] < minimum_distance:
            raise DockingConfigError(
                f"final_distance must be at least {minimum_distance:g} m"
            )

        return {
            "dock_id": dock_id,
            "navigate_to_staging_pose": bool(
                request.get("navigate_to_staging_pose", True)
            ),
            "use_offset_override": use_override,
            **values,
        }

    @staticmethod
    def _finite(value, field: str) -> float:
        try:
            number = float(value)
        except (TypeError, ValueError) as error:
            raise DockingConfigError(f"{field} must be numeric") from error
        if not math.isfinite(number):
            raise DockingConfigError(f"{field} must be finite")
        return number

    @classmethod
    def _pose(cls, value, field: str) -> list[float]:
        if not isinstance(value, (list, tuple)) or len(value) != 3:
            raise DockingConfigError(f"{field} must be [x, y, yaw]")
        return [cls._finite(item, f"{field}[{index}]") for index, item in enumerate(value)]

    def _write_database(self, database: dict) -> None:
        self._database_path.parent.mkdir(parents=True, exist_ok=True)
        descriptor, temporary_path = tempfile.mkstemp(
            prefix=f".{self._database_path.name}.",
            suffix=".tmp",
            dir=self._database_path.parent,
            text=True,
        )
        try:
            with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
                yaml.safe_dump(database, stream, sort_keys=False)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary_path, self._database_path)
        except Exception:
            try:
                os.unlink(temporary_path)
            except FileNotFoundError:
                pass
            raise

    def _load(self) -> dict:
        database = _read_yaml(self._database_path)
        config = _read_yaml(self._config_path)
        raw_docks = database.get("docks")
        if not isinstance(raw_docks, dict) or not raw_docks:
            raise DockingConfigError("Dock database must contain at least one dock")

        docks = []
        for dock_id, definition in raw_docks.items():
            if not isinstance(definition, dict):
                raise DockingConfigError(f"Dock {dock_id!r} must be a mapping")
            docks.append({"dockId": str(dock_id), **definition})

        server_config = config.get("docking_server", {}).get(
            "ros__parameters", {}
        )
        arbiter_config = config.get("velocity_arbiter", {}).get(
            "ros__parameters", {}
        )
        if not isinstance(server_config, dict) or not isinstance(
            arbiter_config, dict
        ):
            raise DockingConfigError(
                "Docking configuration must contain ROS parameter mappings"
            )

        return {
            "schemaVersion": database.get("schema_version"),
            "docks": docks,
            "serverConfig": server_config,
            "arbiterConfig": arbiter_config,
        }


docking_catalog = DockingCatalog()
