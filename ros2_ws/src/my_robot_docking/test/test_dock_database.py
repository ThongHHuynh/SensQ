import yaml

import pytest

from my_robot_docking.dock_database import DockDatabase, DockDatabaseError


def write_database(path, predocking=1.5, staging=1.0, final=0.3):
    path.write_text(
        yaml.safe_dump({
            'schema_version': 2,
            'docks': {
                'test_dock': {
                    'tag_id': 4,
                    'tag_frame': 'tag_4',
                    'global_frame': 'map',
                    'reference_pose': [2.0, 3.0, 0.0],
                    'predocking_distance': predocking,
                    'staging_distance': staging,
                    'final_distance': final,
                    'lateral_offset': 0.0,
                    'yaw_offset': 0.0,
                    'reverse_docking': False,
                },
            },
        }),
        encoding='utf-8',
    )


def write_legacy_database(path):
    path.write_text(
        yaml.safe_dump({
            'schema_version': 1,
            'docks': {
                'legacy_dock': {
                    'tag_id': 7,
                    'tag_frame': 'tag_7',
                    'global_frame': 'map',
                    'reference_pose': [2.0, 3.0, 1.2],
                    'staging_pose': [1.0, 3.0, 0.0],
                    'final_distance': 0.3,
                    'lateral_offset': 0.0,
                    'yaw_offset': 0.0,
                    'reverse_docking': False,
                },
            },
        }),
        encoding='utf-8',
    )


def write_recorded_schema_2_database(path):
    path.write_text(
        yaml.safe_dump({
            'schema_version': 2,
            'docks': {
                'recorded_dock': {
                    'tag_id': 8,
                    'tag_frame': 'tag_8',
                    'global_frame': 'map',
                    'reference_pose': [-0.185, -0.188, 1.577],
                    'staging_pose': [0.131, -0.196, -3.129],
                    'predocking_distance': 0.816101,
                    'final_distance': 0.2,
                    'lateral_offset': 0.0,
                    'yaw_offset': 0.0,
                    'reverse_docking': False,
                },
            },
        }),
        encoding='utf-8',
    )


def test_loads_ordered_predocking_staging_and_final_distances(tmp_path):
    database_path = tmp_path / 'docks.yaml'
    write_database(database_path)

    dock = DockDatabase(database_path).get('test_dock')

    assert dock.predocking_distance == pytest.approx(1.5)
    assert dock.staging_distance == pytest.approx(1.0)
    assert dock.final_distance == pytest.approx(0.3)
    assert dock.approach_yaw == pytest.approx(0.0)


def test_derives_predocking_geometry_from_legacy_staging_pose(tmp_path):
    database_path = tmp_path / 'legacy_docks.yaml'
    write_legacy_database(database_path)

    dock = DockDatabase(
        database_path,
        legacy_predocking_offset=0.5,
    ).get('legacy_dock')

    assert dock.staging_distance == pytest.approx(1.0)
    assert dock.predocking_distance == pytest.approx(1.5)
    assert dock.approach_yaw == pytest.approx(0.0)


def test_schema_2_preserves_recorded_staging_geometry(tmp_path):
    database_path = tmp_path / 'recorded_docks.yaml'
    write_recorded_schema_2_database(database_path)

    dock = DockDatabase(database_path).get('recorded_dock')

    assert dock.staging_distance == pytest.approx(0.3161012496)
    assert dock.predocking_distance == pytest.approx(0.816101)
    assert dock.approach_yaw == pytest.approx(3.1162816044)


@pytest.mark.parametrize(
    ('predocking', 'staging', 'final'),
    [
        (1.0, 1.0, 0.3),
        (0.8, 1.0, 0.3),
        (1.5, 0.3, 0.3),
    ],
)
def test_rejects_invalid_distance_ordering(
    tmp_path,
    predocking,
    staging,
    final,
):
    database_path = tmp_path / 'docks.yaml'
    write_database(database_path, predocking, staging, final)

    with pytest.raises(DockDatabaseError, match='predocking_distance'):
        DockDatabase(database_path)


def test_reference_yaw_that_fights_the_approach_direction_is_reported(
    tmp_path,
):
    """The two reference_pose conventions differ by about a quarter turn.

    record_dock_pose stores the raw tag frame yaw, which lies in the tag
    plane; a dock authored with staging_distance instead stores the approach
    direction there. Reading one as the other aims the dock sideways, so the
    disagreement has to surface at load time rather than on the floor.
    """

    database_path = tmp_path / 'recorded_docks.yaml'
    write_recorded_schema_2_database(database_path)

    database = DockDatabase(database_path)

    assert len(database.warnings()) == 1
    warning = database.warnings()[0]
    assert 'recorded_dock' in warning
    assert 'reference_pose yaw is ignored' in warning


def test_consistent_reference_yaw_is_not_reported(tmp_path):
    database_path = tmp_path / 'aligned_docks.yaml'
    database_path.write_text(
        yaml.safe_dump({
            'schema_version': 2,
            'docks': {
                'aligned_dock': {
                    'tag_id': 9,
                    'tag_frame': 'tag_9',
                    'global_frame': 'map',
                    # Tag ahead of the robot along +x, so both conventions
                    # agree that the approach direction is zero.
                    'reference_pose': [2.0, 0.0, 0.0],
                    'staging_pose': [1.0, 0.0, 0.0],
                    'predocking_distance': 1.5,
                    'final_distance': 0.3,
                    'lateral_offset': 0.0,
                    'yaw_offset': 0.0,
                    'reverse_docking': False,
                },
            },
        }),
        encoding='utf-8',
    )

    assert DockDatabase(database_path).warnings() == ()


def test_hand_authored_docks_report_nothing(tmp_path):
    """staging_distance docks use reference_pose yaw, so there is no conflict."""

    database_path = tmp_path / 'docks.yaml'
    write_database(database_path)

    assert DockDatabase(database_path).warnings() == ()
