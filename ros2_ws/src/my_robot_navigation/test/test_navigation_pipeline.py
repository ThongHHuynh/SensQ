from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml


PACKAGE_DIRECTORY = Path(__file__).parents[1]
BT_FILE = (
    PACKAGE_DIRECTORY
    / 'behavior_trees'
    / 'navigate_to_pose_fast_recovery.xml'
)


def load_navigation_config(filename):
    with (PACKAGE_DIRECTORY / 'config' / filename).open(
        encoding='utf-8'
    ) as stream:
        return yaml.safe_load(stream)


@pytest.mark.parametrize(
    'filename',
    ('nav2_config.yaml', 'sim_nav2_config.yaml'),
)
def test_navfn_remains_dijkstra(filename):
    config = load_navigation_config(filename)
    planner = config['planner_server']['ros__parameters']['GridBased']

    assert planner['use_astar'] is False


def test_behavior_tree_smooths_navfn_path_at_two_hz():
    root = ET.parse(BT_FILE).getroot()
    rate = root.find('.//RateController')
    compute = root.find('.//ComputePathToPose')
    smooth = root.find('.//SmoothPath')
    follow = root.find('.//FollowPath')

    assert rate is not None
    assert compute is not None
    assert smooth is not None
    assert follow is not None
    assert rate.attrib['hz'] == '2.0'
    assert compute.attrib['path'] == '{raw_path}'
    assert smooth.attrib['unsmoothed_path'] == '{raw_path}'
    assert smooth.attrib['smoothed_path'] == '{path}'
    assert smooth.attrib['smoother_id'] == 'simple_smoother'
    assert smooth.attrib['check_for_collisions'] == 'true'
    assert follow.attrib['path'] == '{path}'
