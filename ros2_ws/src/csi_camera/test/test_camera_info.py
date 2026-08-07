"""Tests for CSI camera calibration metadata."""

import pytest

from csi_camera.camera_node import load_camera_info
from csi_camera.camera_node import uncalibrated_camera_info


CALIBRATION = """\
image_width: 640
image_height: 480
camera_name: csi_camera
camera_matrix:
  rows: 3
  cols: 3
  data: [500.0, 0.0, 320.0, 0.0, 501.0, 240.0, 0.0, 0.0, 1.0]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [0.1, -0.2, 0.0, 0.0, 0.05]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
projection_matrix:
  rows: 3
  cols: 4
  data: [500.0, 0.0, 320.0, 0.0, 0.0, 501.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]
"""


def test_load_camera_info(tmp_path):
    """A standard ROS calibration YAML populates CameraInfo."""
    calibration_path = tmp_path / "camera.yaml"
    calibration_path.write_text(CALIBRATION, encoding="utf-8")

    info = load_camera_info(
        f"file://{calibration_path}",
        expected_width=640,
        expected_height=480,
    )

    assert info.width == 640
    assert info.height == 480
    assert info.distortion_model == "plumb_bob"
    assert info.k[0] == 500.0
    assert info.p[5] == 501.0
    assert list(info.d) == [0.1, -0.2, 0.0, 0.0, 0.05]


def test_rejects_resolution_mismatch(tmp_path):
    """Calibration for a different resolution must not be used."""
    calibration_path = tmp_path / "camera.yaml"
    calibration_path.write_text(CALIBRATION, encoding="utf-8")

    with pytest.raises(ValueError, match="does not match"):
        load_camera_info(
            str(calibration_path),
            expected_width=1280,
            expected_height=720,
        )


def test_uncalibrated_camera_info_uses_zero_intrinsics():
    """A zero K matrix advertises an uncalibrated camera per ROS."""
    info = uncalibrated_camera_info(640, 480)

    assert info.width == 640
    assert info.height == 480
    assert info.k[0] == 0.0
