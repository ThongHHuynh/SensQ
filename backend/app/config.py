from pathlib import Path
import os


ROOT_DIR = Path(__file__).resolve().parents[2]
ROS_WORKSPACE = Path(os.getenv("ROS_WORKSPACE", ROOT_DIR / "ros2_ws"))
DATA_DIR = Path(os.getenv("SENSQ_DATA_DIR", ROOT_DIR / "data"))
MAP_SAVE_DIR = Path(os.getenv("SENSQ_MAP_SAVE_DIR", DATA_DIR / "maps"))
ROS_DISTRO = os.getenv("ROS_DISTRO", "jazzy")
DATABASE_URL = os.getenv(
    "DATABASE_URL",
    "postgresql+asyncpg://sensq:sensq@localhost:5432/sensq",
)
SERIAL_PORT = os.getenv("SENSQ_SERIAL_PORT", "/dev/ttyACM0")
ODOM_TOPIC = os.getenv("SENSQ_ODOM_TOPIC", "/diff_drive_controller/odom")
JOINT_STATES_TOPIC = os.getenv("SENSQ_JOINT_STATES_TOPIC", "/joint_states")
CMD_VEL_TOPIC = os.getenv("SENSQ_CMD_VEL_TOPIC", "/cmd_vel")
TELEOP_PACKAGE = os.getenv("SENSQ_TELEOP_PACKAGE", "teleop_twist_keyboard")
TELEOP_EXECUTABLE = os.getenv("SENSQ_TELEOP_EXECUTABLE", "teleop_twist_keyboard")
