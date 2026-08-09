import os
from pathlib import Path


APP_DIR = Path(__file__).resolve().parents[2]
ROOT_DIR = Path(os.getenv("SENSQ_ROOT", APP_DIR.parent))
ROS_WORKSPACE = Path(os.getenv("ROS_WORKSPACE", ROOT_DIR / "ros2_ws"))
DOCK_DATABASE_FILE = Path(
    os.getenv(
        "SENSQ_DOCK_DATABASE",
        ROS_WORKSPACE / "src/my_robot_docking/config/dock_database.yaml",
    )
)
DOCKING_CONFIG_FILE = Path(
    os.getenv(
        "SENSQ_DOCKING_CONFIG",
        ROS_WORKSPACE / "src/my_robot_docking/config/docking_config.yaml",
    )
)
DATA_DIR = Path(os.getenv("SENSQ_DATA_DIR", ROOT_DIR / "data"))
MAP_SAVE_DIR = Path(os.getenv("SENSQ_MAP_SAVE_DIR", DATA_DIR / "maps"))
ROS_DISTRO = os.getenv("ROS_DISTRO", "humble")
DATABASE_URL = os.getenv(
    "DATABASE_URL",
    f"sqlite+aiosqlite:///{DATA_DIR / 'sensq.db'}",
)
SERIAL_PORT = os.getenv("SENSQ_SERIAL_PORT", "/dev/ttyACM0")
ODOM_TOPIC = os.getenv("SENSQ_ODOM_TOPIC", "/diff_drive_controller/odom")
JOINT_STATES_TOPIC = os.getenv("SENSQ_JOINT_STATES_TOPIC", "/joint_states")
CMD_VEL_TOPIC = os.getenv("SENSQ_CMD_VEL_TOPIC", "/cmd_vel")
INITIAL_POSE_TOPIC = os.getenv("SENSQ_INITIAL_POSE_TOPIC", "/initialpose")
MAP_TOPIC = os.getenv("SENSQ_MAP_TOPIC", "/map")
MAP_LOAD_SERVICE = os.getenv("SENSQ_MAP_LOAD_SERVICE", "/map_server/load_map")
COVERAGE_GENERATE_SERVICE = os.getenv(
    "SENSQ_COVERAGE_GENERATE_SERVICE", "/coverage_planner/generate"
)
COVERAGE_START_SERVICE = os.getenv(
    "SENSQ_COVERAGE_START_SERVICE", "/coverage_executor/start"
)
COVERAGE_CANCEL_SERVICE = os.getenv(
    "SENSQ_COVERAGE_CANCEL_SERVICE", "/coverage_executor/cancel"
)
COVERAGE_PATH_TOPIC = os.getenv(
    "SENSQ_COVERAGE_PATH_TOPIC", "/test_coverage/path"
)
COVERAGE_PLANNING_STATUS_TOPIC = os.getenv(
    "SENSQ_COVERAGE_PLANNING_STATUS_TOPIC",
    "/test_coverage/planning_status",
)
COVERAGE_EXECUTION_STATUS_TOPIC = os.getenv(
    "SENSQ_COVERAGE_EXECUTION_STATUS_TOPIC",
    "/test_coverage/execution_status",
)
DOCK_RELOAD_SERVICE = os.getenv(
    "SENSQ_DOCK_RELOAD_SERVICE", "/docking_server/reload_database"
)
USE_SIM_TIME = os.getenv("SENSQ_USE_SIM_TIME", "false").lower() in {
    "1",
    "true",
    "yes",
    "on",
}
ROBOT_LAUNCH_PACKAGE = os.getenv(
    "SENSQ_ROBOT_LAUNCH_PACKAGE", "my_robot_navigation"
)
ROBOT_LAUNCH_FILE = os.getenv(
    "SENSQ_ROBOT_LAUNCH_FILE", "bringup.launch.py"
)
TELEOP_PACKAGE = os.getenv("SENSQ_TELEOP_PACKAGE", "teleop_twist_keyboard")
TELEOP_EXECUTABLE = os.getenv("SENSQ_TELEOP_EXECUTABLE", "teleop_twist_keyboard")
