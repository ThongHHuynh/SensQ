from pathlib import Path

import pytest
from ament_flake8.main import main_with_errors


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    package_root = Path(__file__).resolve().parents[1]
    return_code, errors = main_with_errors(
        argv=[
            str(package_root / "setup.py"),
            str(package_root / "launch"),
            str(package_root / "my_robot_mission"),
            str(package_root / "test"),
        ]
    )
    assert return_code == 0, "Found code style errors:\n" + "\n".join(errors)
