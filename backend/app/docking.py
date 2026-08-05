import math
from pathlib import Path

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
        self._payload = self._load()

    @property
    def action_name(self) -> str:
        return str(self._payload["serverConfig"].get("dock_action", "/dock"))

    def get_payload(self) -> dict:
        return self._payload

    def normalize_goal(self, request: dict) -> dict:
        docks = {dock["dockId"]: dock for dock in self._payload["docks"]}
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
            self._payload["serverConfig"].get("minimum_tag_distance", 0.0)
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
