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
