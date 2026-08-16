import math
from types import SimpleNamespace

from builtin_interfaces.msg import Time
import pytest
from visualization_msgs.msg import Marker

from my_robot_docking.docking_geometry import Pose2D, normalize_angle
from my_robot_docking.docking_visualization import (
    DockingStage,
    DockingVisualizer,
    STAGE_COLORS,
    build_docking_reference,
    should_append_trail_point,
)


def make_dock(reverse=False):
    return SimpleNamespace(
        reference_x=3.0,
        reference_y=2.0,
        approach_yaw=math.pi / 2.0,
        predocking_distance=1.5,
        staging_distance=1.0,
        reverse_docking=reverse,
        global_frame='map',
    )


def test_reference_contains_collinear_stage_targets():
    reference = build_docking_reference(
        make_dock(),
        final_distance=0.3,
        lateral_offset=0.0,
        yaw_offset=0.0,
    )

    assert reference.predocking.x == pytest.approx(3.0)
    assert reference.predocking.y == pytest.approx(0.5)
    assert reference.staging.y == pytest.approx(1.0)
    assert reference.final.y == pytest.approx(1.7)
    assert reference.final.yaw == pytest.approx(math.pi / 2.0)


def test_reverse_reference_flips_only_final_heading():
    reference = build_docking_reference(
        make_dock(reverse=True),
        final_distance=0.3,
        lateral_offset=0.0,
        yaw_offset=0.0,
    )

    assert reference.final.x == pytest.approx(3.0)
    assert reference.final.y == pytest.approx(1.7)
    assert abs(normalize_angle(reference.final.yaw + math.pi / 2.0)) < 1e-9


def test_trail_downsampling_keeps_translation_and_rotation():
    origin = Pose2D(0.0, 0.0, 0.0)

    assert not should_append_trail_point(
        origin,
        Pose2D(0.005, 0.0, 0.01),
        minimum_distance=0.01,
        minimum_yaw=0.02,
    )
    assert should_append_trail_point(
        origin,
        Pose2D(0.02, 0.0, 0.0),
        minimum_distance=0.01,
        minimum_yaw=0.02,
    )
    assert should_append_trail_point(
        origin,
        Pose2D(0.0, 0.0, 0.03),
        minimum_distance=0.01,
        minimum_yaw=0.02,
    )


def test_each_motion_stage_has_a_distinct_color():
    assert len(set(STAGE_COLORS.values())) == len(STAGE_COLORS)


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeClock:
    class Now:
        @staticmethod
        def to_msg():
            return Time()

    @staticmethod
    def now():
        return FakeClock.Now()


class FakeNode:
    def __init__(self):
        self.publisher = FakePublisher()

    def create_publisher(self, *_args):
        return self.publisher

    @staticmethod
    def get_clock():
        return FakeClock()


def test_visualizer_publishes_reference_and_actual_stage_markers():
    node = FakeNode()
    visualizer = DockingVisualizer(
        node=node,
        enabled=True,
        topic='/docking/markers',
        line_width=0.03,
        minimum_point_distance=0.01,
        publish_rate=10.0,
        maximum_points=100,
    )
    visualizer.begin_goal(make_dock(), 0.3, 0.0, 0.0)
    visualizer.set_stage(DockingStage.PREDOCK_ALIGNMENT)
    visualizer.append_pose(Pose2D(0.0, 0.0, 0.0), 'odom')
    visualizer.append_pose(Pose2D(0.02, 0.0, 0.0), 'odom')
    visualizer.finish(True, 'Docked')

    markers = node.publisher.messages[-1].markers
    namespaces = {marker.ns for marker in markers}
    assert 'reference/staging' in namespaces
    assert 'reference/final' in namespaces
    assert 'actual/predock_alignment' in namespaces
    actual = next(
        marker
        for marker in markers
        if marker.ns == 'actual/predock_alignment'
    )
    assert actual.type == Marker.LINE_STRIP
    assert len(actual.points) == 2
