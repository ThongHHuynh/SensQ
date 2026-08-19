from pathlib import Path

import pytest
import yaml


CONFIG_DIRECTORY = Path(__file__).parents[1] / 'config'


def detector_parameters(filename):
    with (CONFIG_DIRECTORY / filename).open(encoding='utf-8') as stream:
        return yaml.safe_load(stream)['apriltag_node']['ros__parameters']


@pytest.mark.parametrize(
    ('filename', 'expected_size'),
    (
        ('apriltag.yaml', 0.13),
        ('apriltag_sim.yaml', 0.16),
    ),
)
def test_detector_default_and_per_tag_sizes_match(filename, expected_size):
    parameters = detector_parameters(filename)

    assert parameters['size'] == pytest.approx(expected_size)
    assert parameters['tag']['sizes']
    assert all(
        size == pytest.approx(expected_size)
        for size in parameters['tag']['sizes']
    )
