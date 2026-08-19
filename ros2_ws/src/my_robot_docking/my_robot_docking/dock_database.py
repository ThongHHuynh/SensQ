from dataclasses import dataclass
import math
from pathlib import Path
from typing import Dict, List, Sequence

import yaml


class DockDatabaseError(ValueError):
    """Raised when the dock database is invalid."""


@dataclass(frozen=True)
class DockDefinition:
    """A dataclass to hold the definition of a docking station."""
    dock_id: str
    tag_id: int
    tag_frame: str
    global_frame: str

    reference_x: float
    reference_y: float
    reference_yaw: float
    approach_yaw: float

    predocking_distance: float
    staging_distance: float
    final_distance: float
    lateral_offset: float
    yaw_offset: float

    reverse_docking: bool


class DockDatabase:
    """Loads and validates docking station definitions from a YAML file.

    ``reference_pose`` carries two different meanings depending on how the
    dock was authored, and they differ by about a quarter turn:

    * with ``staging_pose`` -- the raw tag frame pose as written by
      ``record_dock_pose``. Its yaw is the tag frame's own yaw, which lies in
      the tag plane, so the approach direction is derived from the recorded
      robot pose instead and ``reference_pose[2]`` is ignored.
    * with ``staging_distance`` -- a hand-authored pose whose yaw *is* the
      approach direction, because there is no robot pose to derive one from.

    Mixing the two silently aims the dock ninety degrees away from its tag, so
    the mismatch is reported through :meth:`warnings`.
    """

    SUPPORTED_SCHEMA_VERSIONS = (1, 2)
    APPROACH_YAW_MISMATCH_TOLERANCE = 0.35

    def __init__(self, database_path, legacy_predocking_offset=0.5):
        self._path = Path(database_path)
        self._legacy_predocking_offset = float(legacy_predocking_offset)
        if self._legacy_predocking_offset <= 0.0:
            raise DockDatabaseError(
                'legacy_predocking_offset must be positive'
            )
        self._docks: Dict[str, DockDefinition] = {}
        self._warnings: List[str] = []
        self._load()

    def _load(self):
        if not self._path.is_file():
            raise DockDatabaseError(
                f'Dock database does not exist: {self._path}'
            )

        try:
            with self._path.open('r', encoding='utf-8') as file:
                data = yaml.safe_load(file)
        except OSError as error:
            raise DockDatabaseError(
                f'Could not read dock database {self._path}: {error}'
            ) from error
        except yaml.YAMLError as error:
            raise DockDatabaseError(
                f'Invalid YAML in dock database in {self._path}: {error}'
            ) from error

        if not isinstance(data, dict):
            raise DockDatabaseError(
                'Dock database must be a dictionary'
            )

        version = data.get('schema_version')
        if version not in self.SUPPORTED_SCHEMA_VERSIONS:
            raise DockDatabaseError(
                f'Unsupported schema version {version}. '
                f'Supported versions are {self.SUPPORTED_SCHEMA_VERSIONS}'
            )
        self._schema_version = version

        raw_docks = data.get('docks')
        if not isinstance(raw_docks, dict) or not raw_docks:
            raise DockDatabaseError(
                'The docks field must contain at least one dock definition'
            )

        used_tag_ids = set()

        for dock_id, raw_dock in raw_docks.items():
            definition = self._parse_dock(dock_id, raw_dock)

            if definition.tag_id in used_tag_ids:
                raise DockDatabaseError(
                    f'Dock {dock_id} has a duplicate tag_id: {definition.tag_id}'
                )
            used_tag_ids.add(definition.tag_id)
            self._docks[dock_id] = definition

    def _parse_dock(self, dock_id: str, raw_dock: dict) -> DockDefinition:
        if not isinstance(raw_dock, dict):
            raise DockDatabaseError(
                f'Dock {dock_id} must be a dictionary'
            )

        required_fields = [
            'tag_id',
            'tag_frame',
            'global_frame',
            'reference_pose',
            'final_distance',
            'lateral_offset',
            'yaw_offset',
            'reverse_docking',
        ]
        if self._schema_version == 1:
            required_fields.append('staging_pose')
        else:
            required_fields.append('predocking_distance')

        for field in required_fields:
            if field not in raw_dock:
                raise DockDatabaseError(
                    f'Dock {dock_id} is missing required field: {field}'
                )
        tag_id = raw_dock['tag_id']

        if isinstance(tag_id, bool) or not isinstance(tag_id, int):
            raise DockDatabaseError(
                f'Dock {dock_id!r} tag_id must be an integer'
            )

        reference = self._parse_pose(
            dock_id,
            'reference_pose',
            raw_dock['reference_pose'],
        )
        has_staging_pose = 'staging_pose' in raw_dock
        has_staging_distance = 'staging_distance' in raw_dock
        if self._schema_version == 2 and (
            has_staging_pose == has_staging_distance
        ):
            raise DockDatabaseError(
                f'Dock {dock_id} schema 2 must define exactly one of '
                'staging_pose or staging_distance'
            )

        if has_staging_pose:
            staging = self._parse_pose(
                dock_id,
                'staging_pose',
                raw_dock['staging_pose'],
            )
            tag_delta_x = reference[0] - staging[0]
            tag_delta_y = reference[1] - staging[1]
            staging_distance = math.hypot(tag_delta_x, tag_delta_y)
            approach_yaw = math.atan2(tag_delta_y, tag_delta_x)
            self._check_approach_yaw(dock_id, approach_yaw, reference[2])
            if self._schema_version == 1:
                predocking_distance = (
                    staging_distance + self._legacy_predocking_offset
                )
            else:
                predocking_distance = self._number(
                    dock_id,
                    'predocking_distance',
                    raw_dock['predocking_distance'],
                )
        else:
            predocking_distance = self._number(
                dock_id,
                'predocking_distance',
                raw_dock['predocking_distance'],
            )
            staging_distance = self._number(
                dock_id,
                'staging_distance',
                raw_dock['staging_distance'],
            )
            approach_yaw = reference[2]
        final_distance = self._number(
            dock_id,
            'final_distance',
            raw_dock['final_distance'],
        )
        lateral_offset = self._number(
            dock_id,
            'lateral_offset',
            raw_dock['lateral_offset'],
        )
        yaw_offset = self._number(
            dock_id,
            'yaw_offset',
            raw_dock['yaw_offset'],
        )

        if final_distance <= 0:
            raise DockDatabaseError(
                f'Dock {dock_id} final_distance must be positive'
            )
        if not predocking_distance > staging_distance > final_distance:
            raise DockDatabaseError(
                f'Dock {dock_id} must satisfy predocking_distance > '
                'staging_distance > final_distance'
            )
        reverse_docking = raw_dock['reverse_docking']
        if not isinstance(reverse_docking, bool):
            raise DockDatabaseError(
                f'Dock {dock_id} reverse_docking must be a boolean'
            )
        return DockDefinition(
            dock_id=str(dock_id),
            tag_id=tag_id,
            tag_frame=self._string(
                dock_id,
                'tag_frame',
                raw_dock['tag_frame'],
            ),
            global_frame=self._string(
                dock_id,
                'global_frame',
                raw_dock['global_frame'],
            ),
            reference_x=reference[0],
            reference_y=reference[1],
            reference_yaw=reference[2],
            approach_yaw=approach_yaw,
            predocking_distance=predocking_distance,
            staging_distance=staging_distance,
            final_distance=final_distance,
            lateral_offset=lateral_offset,
            yaw_offset=yaw_offset,
            reverse_docking=reverse_docking,
        )

    def _check_approach_yaw(self, dock_id, approach_yaw, reference_yaw):
        """Flag a dock whose recorded tag yaw fights its approach direction."""

        difference = abs(
            math.atan2(
                math.sin(approach_yaw - reference_yaw),
                math.cos(approach_yaw - reference_yaw),
            )
        )
        if difference > self.APPROACH_YAW_MISMATCH_TOLERANCE:
            self._warnings.append(
                f'Dock {dock_id}: approach direction {approach_yaw:.3f} rad '
                f'derived from staging_pose differs from reference_pose yaw '
                f'{reference_yaw:.3f} rad by {difference:.3f} rad. '
                'reference_pose yaw is ignored for this dock; it is only used '
                'when a dock is defined with staging_distance instead.'
            )

    def warnings(self):
        """Non-fatal problems found while loading, in file order."""

        return tuple(self._warnings)

    def get(self, dock_id: str) -> DockDefinition:
        """Return one dock by its configured ID."""

        try:
            return self._docks[dock_id]
        except KeyError as error:
            available = ', '.join(sorted(self._docks))

            raise DockDatabaseError(
                f'Unknown dock {dock_id!r}; available docks: {available}'
            ) from error

    def ids(self):
        """Return all configured dock IDs."""

        return tuple(sorted(self._docks))

    @classmethod
    def _parse_pose(
        cls,
        dock_id: str,
        field: str,
        value: Sequence,
    ):
        if (
            not isinstance(value, (list, tuple))
            or len(value) != 3
        ):
            raise DockDatabaseError(
                f'Dock {dock_id!r} {field} must be [x, y, yaw]'
            )

        return tuple(
            cls._number(dock_id, f'{field}[{index}]', item)
            for index, item in enumerate(value)
        )

    @staticmethod
    def _number(dock_id, field, value):
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise DockDatabaseError(
                f'Dock {dock_id!r} {field} must be numeric'
            )

        number = float(value)
        if not math.isfinite(number):
            raise DockDatabaseError(
                f'Dock {dock_id!r} {field} must be finite'
            )
        return number

    @staticmethod
    def _string(dock_id, field, value):
        if not isinstance(value, str) or not value.strip():
            raise DockDatabaseError(
                f'Dock {dock_id!r} {field} must be a non-empty string'
            )

        return value
