#!/usr/bin/env python3
"""
record_dock_pose.py
===================
Curses TUI that lets you teleop the robot to a desired docking position,
then press [R] to snapshot the robot staging pose and the AprilTag
reference pose, and append a new entry to dock_database.yaml.

Usage (after colcon build + source install/setup.bash)
------------------------------------------------------
    ros2 run my_robot_docking record_dock_pose \\
        --tag-id 2 \\
        --dock-id my_new_dock

Or directly:
    python3 src/my_robot_docking/my_robot_docking/record_dock_pose.py \\
        --tag-id 2 --dock-id my_new_dock \\
        --database src/my_robot_docking/config/dock_database.yaml

Requirements
------------
    rclpy, tf2_ros, apriltag_msgs (optional), std_srvs, pyyaml
"""

import argparse
import curses
import math
import os
import subprocess
import sys
import tempfile
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from ament_index_python.packages import get_package_share_directory

import yaml

# apriltag_msgs provides AprilTagDetectionArray on /detections
try:
    from apriltag_msgs.msg import AprilTagDetectionArray
    _HAS_APRILTAG_MSGS = True
except ImportError:
    _HAS_APRILTAG_MSGS = False


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

def _quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Extract yaw from a unit quaternion."""
    return math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )


def _round3(v: float) -> float:
    return round(v, 3)


def _default_database_path() -> str:
    """Resolve dock_database.yaml from the installed package share dir."""
    try:
        share = get_package_share_directory('my_robot_docking')
        return os.path.join(share, 'config', 'dock_database.yaml')
    except Exception:
        # Fallback: relative to this file (in-source run)
        here = os.path.dirname(os.path.abspath(__file__))
        return os.path.join(here, '..', 'config', 'dock_database.yaml')


# ─────────────────────────────────────────────────────────────────────────────
# ROS2 Node
# ─────────────────────────────────────────────────────────────────────────────

class DockRecorderNode(Node):
    """Lightweight node that tracks robot and tag poses via TF2."""

    def __init__(self, args: argparse.Namespace):
        super().__init__('dock_recorder')
        self._args = args
        self._lock = threading.Lock()

        # Latest detection stamp per tag id  {tag_id: Time}
        self._detection_stamps: dict = {}

        # TF2
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # /detections subscription (optional — for tag-age guard)
        if _HAS_APRILTAG_MSGS:
            self.create_subscription(
                AprilTagDetectionArray,
                args.detections_topic,
                self._detections_cb,
                10,
            )
        else:
            self.get_logger().warning(
                'apriltag_msgs not found — tag-age guard disabled. '
                'Tag visibility is inferred from TF only.'
            )

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _detections_cb(self, msg) -> None:
        stamp = Time.from_msg(msg.header.stamp)
        with self._lock:
            for det in msg.detections:
                self._detection_stamps[det.id] = stamp

    # ── Public API ────────────────────────────────────────────────────────

    def get_robot_pose(self):
        """Return (x, y, yaw) of base_footprint in global_frame, or None."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self._args.global_frame,
                self._args.base_frame,
                Time(),
                timeout=Duration(seconds=0.1),
            )
        except TransformException:
            return None
        t = tf.transform.translation
        r = tf.transform.rotation
        return float(t.x), float(t.y), _quat_to_yaw(r.x, r.y, r.z, r.w)

    def get_tag_pose(self):
        """Return (x, y, yaw) of tag_N in global_frame, or None."""
        tag_frame = f'tag_{self._args.tag_id}'
        try:
            tf = self.tf_buffer.lookup_transform(
                self._args.global_frame,
                tag_frame,
                Time(),
                timeout=Duration(seconds=0.1),
            )
        except TransformException:
            return None
        t = tf.transform.translation
        r = tf.transform.rotation
        return float(t.x), float(t.y), _quat_to_yaw(r.x, r.y, r.z, r.w)

    def get_tag_age(self):
        """Return seconds since last detection for our tag_id, or None."""
        with self._lock:
            stamp = self._detection_stamps.get(self._args.tag_id)
        if stamp is None:
            return None
        age = (self.get_clock().now() - stamp).nanoseconds / 1e9
        return max(0.0, age)

    def call_reload_service(self):
        """Call /docking_server/reload_database. Returns (success, message)."""
        try:
            result = subprocess.run(
                [
                    'ros2', 'service', 'call',
                    '/docking_server/reload_database',
                    'std_srvs/srv/Trigger', '{}',
                ],
                capture_output=True,
                text=True,
                timeout=6.0,
            )
            if result.returncode == 0:
                return True, 'Docking server reloaded OK'
            return False, f'Reload failed: {result.stderr.strip()}'
        except subprocess.TimeoutExpired:
            return False, 'Reload service call timed out'
        except FileNotFoundError:
            return False, 'ros2 CLI not found — reload manually'


# ─────────────────────────────────────────────────────────────────────────────
# YAML writer
# ─────────────────────────────────────────────────────────────────────────────

def _load_database(path: str) -> dict:
    with open(path, 'r', encoding='utf-8') as fh:
        data = yaml.safe_load(fh)
    if not isinstance(data, dict):
        raise ValueError('dock_database.yaml must be a YAML mapping')
    if data.get('schema_version') not in (1, 2):
        raise ValueError(
            f"Unsupported schema_version: {data.get('schema_version')} "
            '(expected 1 or 2)'
        )
    if 'docks' not in data or not isinstance(data['docks'], dict):
        data['docks'] = {}
    return data


def save_dock_entry(
    database_path: str,
    dock_id: str,
    tag_id: int,
    global_frame: str,
    reference_pose: tuple,
    staging_pose: tuple,
    predocking_distance: float,
    final_distance: float,
    lateral_offset: float,
    yaw_offset: float,
    reverse_docking: bool,
) -> None:
    """
    Append a new dock to dock_database.yaml (atomic write).

    Raises
    ------
    ValueError  — dock_id / tag_id already exists, or schema mismatch.
    OSError     — file I/O problem.
    """
    data = _load_database(database_path)
    docks = data['docks']

    # Reject duplicate dock_id
    if dock_id in docks:
        raise ValueError(
            f"dock_id '{dock_id}' already exists. Choose a different --dock-id."
        )

    # Reject duplicate tag_id
    for existing_id, existing_dock in docks.items():
        if existing_dock.get('tag_id') == tag_id:
            raise ValueError(
                f"tag_id {tag_id} is already used by dock '{existing_id}'. "
                "Each dock must have a unique tag_id."
            )

    dock_entry = {
        'tag_id': tag_id,
        'tag_frame': f'tag_{tag_id}',
        'global_frame': global_frame,
        'reference_pose': [
            _round3(reference_pose[0]),
            _round3(reference_pose[1]),
            _round3(reference_pose[2]),
        ],
        'final_distance': final_distance,
        'lateral_offset': lateral_offset,
        'yaw_offset': yaw_offset,
        'reverse_docking': reverse_docking,
    }
    if data['schema_version'] == 1:
        dock_entry['staging_pose'] = [
            _round3(staging_pose[0]),
            _round3(staging_pose[1]),
            _round3(staging_pose[2]),
        ]
    else:
        tag_delta_x = reference_pose[0] - staging_pose[0]
        tag_delta_y = reference_pose[1] - staging_pose[1]
        staging_distance = math.hypot(tag_delta_x, tag_delta_y)
        if not predocking_distance > staging_distance > final_distance:
            raise ValueError(
                'Expected predocking distance > recorded staging distance > '
                f'final distance, got {predocking_distance:.3f} > '
                f'{staging_distance:.3f} > {final_distance:.3f}'
            )
        dock_entry['predocking_distance'] = _round3(predocking_distance)
        dock_entry['staging_pose'] = [
            _round3(staging_pose[0]),
            _round3(staging_pose[1]),
            _round3(staging_pose[2]),
        ]

    docks[dock_id] = dock_entry

    # Atomic write: temp file → os.replace
    dir_name = os.path.dirname(os.path.abspath(database_path))
    fd, tmp_path = tempfile.mkstemp(dir=dir_name, suffix='.yaml.tmp')
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as fh:
            yaml.dump(data, fh, default_flow_style=False, sort_keys=False)
        os.replace(tmp_path, database_path)
    except Exception:
        try:
            os.unlink(tmp_path)
        except OSError:
            pass
        raise


# ─────────────────────────────────────────────────────────────────────────────
# Curses TUI
# ─────────────────────────────────────────────────────────────────────────────

# Colour pair indices
_C_TITLE   = 1
_C_OK      = 2
_C_WARN    = 3
_C_ERR     = 4
_C_DIM     = 5
_C_BORDER  = 6
_C_SUCCESS = 7
_C_HINT    = 8


def _init_colours() -> None:
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(_C_TITLE,   curses.COLOR_CYAN,    -1)
    curses.init_pair(_C_OK,      curses.COLOR_GREEN,   -1)
    curses.init_pair(_C_WARN,    curses.COLOR_YELLOW,  -1)
    curses.init_pair(_C_ERR,     curses.COLOR_RED,     -1)
    curses.init_pair(_C_DIM,     curses.COLOR_WHITE,   -1)
    curses.init_pair(_C_BORDER,  curses.COLOR_BLUE,    -1)
    curses.init_pair(_C_SUCCESS, curses.COLOR_GREEN,   -1)
    curses.init_pair(_C_HINT,    curses.COLOR_MAGENTA, -1)


def _safe_add(win, row: int, col: int, text: str, attr: int = 0) -> None:
    try:
        win.addstr(row, col, text, attr)
    except curses.error:
        pass


def _draw_box(win, row: int, col: int, h: int, w: int) -> None:
    attr = curses.color_pair(_C_BORDER)
    try:
        win.addch(row,         col,         curses.ACS_ULCORNER, attr)
        win.addch(row,         col + w - 1, curses.ACS_URCORNER, attr)
        win.addch(row + h - 1, col,         curses.ACS_LLCORNER, attr)
        win.addch(row + h - 1, col + w - 1, curses.ACS_LRCORNER, attr)
        for c in range(col + 1, col + w - 1):
            win.addch(row,         c, curses.ACS_HLINE, attr)
            win.addch(row + h - 1, c, curses.ACS_HLINE, attr)
        for r in range(row + 1, row + h - 1):
            win.addch(r, col,         curses.ACS_VLINE, attr)
            win.addch(r, col + w - 1, curses.ACS_VLINE, attr)
    except curses.error:
        pass


def _hline(win, row: int, col: int, w: int) -> None:
    attr = curses.color_pair(_C_BORDER)
    _safe_add(win, row, col, '├' + '─' * (w - 2) + '┤', attr)


def _pose_str(pose) -> str:
    if pose is None:
        return 'unavailable'
    x, y, yaw = pose
    return f'x={x:+.3f}  y={y:+.3f}  yaw={yaw:+.4f} rad'


def _pose_attr(pose) -> int:
    return curses.color_pair(_C_OK) if pose is not None else curses.color_pair(_C_ERR)


# ─── Save logic ───────────────────────────────────────────────────────────────

def _do_save(node, args, robot_pose, tag_pose, tag_age):
    """
    Validate + write. Returns (status_text, curses_attr, saved_summary).
    saved_summary is non-empty only on full success.
    """
    err  = curses.color_pair(_C_ERR)  | curses.A_BOLD
    ok   = curses.color_pair(_C_SUCCESS) | curses.A_BOLD
    warn = curses.color_pair(_C_WARN) | curses.A_BOLD

    if robot_pose is None:
        return 'ERROR: Robot pose unavailable (map→base_footprint TF missing)', err, ''

    if tag_pose is None:
        return (
            f'ERROR: tag_{args.tag_id} not in TF — is the camera seeing the tag?',
            err, '',
        )

    if _HAS_APRILTAG_MSGS:
        if tag_age is None:
            return (
                f'ERROR: No detection received for tag {args.tag_id} yet.',
                err, '',
            )
        if tag_age > args.tag_max_age:
            return (
                f'ERROR: Tag age {tag_age:.2f}s > {args.tag_max_age}s limit — '
                'aim camera at the tag.',
                err, '',
            )

    try:
        save_dock_entry(
            database_path=args.database,
            dock_id=args.dock_id,
            tag_id=args.tag_id,
            global_frame=args.global_frame,
            reference_pose=tag_pose,
            staging_pose=robot_pose,
            predocking_distance=args.predocking_distance,
            final_distance=args.final_distance,
            lateral_offset=args.lateral_offset,
            yaw_offset=args.yaw_offset,
            reverse_docking=args.reverse,
        )
    except ValueError as exc:
        return f'ERROR: {exc}', err, ''
    except OSError as exc:
        return f'FILE ERROR: {exc}', err, ''

    ok_bool, reload_msg = node.call_reload_service()
    x, y, yaw = robot_pose
    summary = f"{args.dock_id}  staging=({x:.3f}, {y:.3f}, {yaw:.3f})"
    if not ok_bool:
        return f'Saved — but reload failed: {reload_msg}', warn, summary
    return f'Saved & reloaded: {reload_msg}', ok, summary


# ─── Main TUI loop ────────────────────────────────────────────────────────────

BOX_W = 72
BOX_H = 22

_POLL_MS = 50   # 20 Hz refresh rate


def _prompt_input(
    stdscr,
    prompt: str,
    initial: str = '',
    max_len: int = 40,
    validator=None,
) -> 'str | None':
    """
    Show a single-line inline input prompt at the bottom of the screen.

    Returns the entered string (stripped) on Enter, or None on Escape.
    *validator* is an optional callable(str) -> bool; if it returns False the
    field is shown in red and Enter is ignored.
    """
    rows, _ = stdscr.getmaxyx()
    prompt_row = rows - 2

    curses.curs_set(1)
    stdscr.nodelay(False)
    stdscr.timeout(-1)

    buf = list(initial)

    while True:
        stdscr.move(prompt_row, 0)
        stdscr.clrtoeol()
        label = f' {prompt}: '
        _safe_add(stdscr, prompt_row, 0, label,
                  curses.color_pair(_C_TITLE) | curses.A_BOLD)
        value = ''.join(buf)
        valid = validator is None or validator(value)
        val_attr = (curses.color_pair(_C_OK) if valid
                    else curses.color_pair(_C_ERR)) | curses.A_BOLD
        _safe_add(stdscr, prompt_row, len(label), value + '▌', val_attr)
        stdscr.refresh()

        key = stdscr.getch()
        if key == 27:                          # Escape — cancel
            result = None
            break
        elif key in (10, 13, curses.KEY_ENTER):  # Enter — confirm
            if validator is None or validator(''.join(buf)):
                result = ''.join(buf).strip()
                break
        elif key in (curses.KEY_BACKSPACE, 127, 8):
            if buf:
                buf.pop()
        elif 32 <= key < 127 and len(buf) < max_len:
            buf.append(chr(key))

    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(_POLL_MS)
    return result


def _run_tui(stdscr, node, args) -> None:
    _init_colours()
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(_POLL_MS)

    status_text = ''
    status_attr = 0
    last_saved  = ''

    while True:
        # Spin ROS callbacks (non-blocking)
        rclpy.spin_once(node, timeout_sec=0.0)

        # Gather data
        robot_pose = node.get_robot_pose()
        tag_pose   = node.get_tag_pose()
        tag_age    = node.get_tag_age()

        # Tag status text + attr
        if tag_age is not None:
            if tag_age <= args.tag_max_age:
                tag_txt  = f'VISIBLE  (age {tag_age:.2f} s)'
                tag_attr = curses.color_pair(_C_OK) | curses.A_BOLD
            else:
                tag_txt  = f'STALE    (age {tag_age:.2f} s > {args.tag_max_age} s)'
                tag_attr = curses.color_pair(_C_WARN) | curses.A_BOLD
        elif tag_pose is not None:
            tag_txt  = 'TF OK  (age unknown — apriltag_msgs not installed)'
            tag_attr = curses.color_pair(_C_WARN) | curses.A_BOLD
        else:
            tag_txt  = 'NOT VISIBLE'
            tag_attr = curses.color_pair(_C_ERR) | curses.A_BOLD

        # ── Draw ──────────────────────────────────────────────────────
        stdscr.erase()
        rows, cols = stdscr.getmaxyx()
        br = max(0, (rows - BOX_H) // 2)
        bc = max(0, (cols - BOX_W) // 2)

        _draw_box(stdscr, br, bc, BOX_H, BOX_W)

        r = br
        c = bc + 2

        # ── Title row ─────────────────────────────────────────────────
        title = '  Dock Position Recorder  '
        _safe_add(
            stdscr, r, bc + (BOX_W - len(title)) // 2,
            title,
            curses.color_pair(_C_TITLE) | curses.A_BOLD,
        )
        r += 1
        _hline(stdscr, r, bc, BOX_W); r += 1

        # ── Config block (editable fields highlighted) ────────────────
        _safe_add(stdscr, r, c, 'Dock ID  : ', curses.color_pair(_C_DIM))
        _safe_add(stdscr, r, c + 11, args.dock_id,
                  curses.color_pair(_C_HINT) | curses.A_BOLD)
        _safe_add(stdscr, r, c + 11 + len(args.dock_id), '  [N]',
                  curses.color_pair(_C_DIM)); r += 1

        tag_label = f'{args.tag_id}  →  tag_{args.tag_id}'
        _safe_add(stdscr, r, c, 'Tag ID   : ', curses.color_pair(_C_DIM))
        _safe_add(stdscr, r, c + 11, tag_label,
                  curses.color_pair(_C_HINT) | curses.A_BOLD)
        _safe_add(stdscr, r, c + 11 + len(tag_label), '  [T]',
                  curses.color_pair(_C_DIM)); r += 1

        _safe_add(stdscr, r, c,
                  f'Database : {os.path.basename(args.database)}',
                  curses.color_pair(_C_DIM)); r += 1
        _safe_add(stdscr, r, c,
                  f'Params   : predock={args.predocking_distance}m  '
                  f'final={args.final_distance}m  '
                  f'lat={args.lateral_offset}  yaw={args.yaw_offset}  '
                  f'reverse={args.reverse}',
                  curses.color_pair(_C_DIM)); r += 1

        _hline(stdscr, r, bc, BOX_W); r += 1

        # ── Live poses ────────────────────────────────────────────────
        _safe_add(stdscr, r, c, 'Tag      : ', curses.color_pair(_C_DIM))
        _safe_add(stdscr, r, c + 11, tag_txt, tag_attr); r += 1

        _safe_add(stdscr, r, c, 'Robot    : ', curses.color_pair(_C_DIM))
        _safe_add(stdscr, r, c + 11, _pose_str(robot_pose),
                  _pose_attr(robot_pose)); r += 1

        _safe_add(stdscr, r, c, 'Tag map  : ', curses.color_pair(_C_DIM))
        _safe_add(stdscr, r, c + 11, _pose_str(tag_pose),
                  _pose_attr(tag_pose)); r += 1

        _hline(stdscr, r, bc, BOX_W); r += 1

        # ── Readiness hint ────────────────────────────────────────────
        ready = robot_pose is not None and tag_pose is not None and (
            not _HAS_APRILTAG_MSGS
            or (tag_age is not None and tag_age <= args.tag_max_age)
        )
        if ready:
            hint = '✔ Ready to save!'
            hint_attr = curses.color_pair(_C_OK) | curses.A_BOLD
        else:
            hint = '⚠ Waiting for robot pose and fresh tag detection…'
            hint_attr = curses.color_pair(_C_WARN)
        _safe_add(stdscr, r, c, hint, hint_attr); r += 1

        _hline(stdscr, r, bc, BOX_W); r += 1

        # ── Keybindings ───────────────────────────────────────────────
        _safe_add(stdscr, r, c,
                  '[R] Record & Save   [T] Edit Tag ID   [N] Edit Dock ID   [Q] Quit',
                  curses.A_BOLD); r += 1

        # ── Status / last-saved ───────────────────────────────────────
        if status_text:
            _safe_add(stdscr, r, c, status_text[: BOX_W - 4], status_attr)
        r += 1
        if last_saved:
            _safe_add(stdscr, r, c,
                      f'Last saved: {last_saved}'[: BOX_W - 4],
                      curses.color_pair(_C_SUCCESS))

        stdscr.refresh()

        # ── Input ─────────────────────────────────────────────────────
        key = stdscr.getch()

        if key in (ord('q'), ord('Q')):
            break

        if key in (ord('t'), ord('T')):
            new_val = _prompt_input(
                stdscr,
                'New Tag ID (integer)',
                str(args.tag_id),
                validator=lambda s: s.lstrip('-').isdigit(),
            )
            if new_val is not None:
                try:
                    new_id = int(new_val)
                    args.tag_id = new_id
                    node._args.tag_id = new_id
                    status_text = f'Tag ID updated → tag_{new_id}'
                    status_attr = curses.color_pair(_C_OK) | curses.A_BOLD
                except ValueError:
                    status_text = f'ERROR: "{new_val}" is not a valid integer'
                    status_attr = curses.color_pair(_C_ERR) | curses.A_BOLD

        elif key in (ord('n'), ord('N')):
            new_val = _prompt_input(
                stdscr,
                'New Dock ID',
                args.dock_id,
                validator=lambda s: bool(s.strip()),
            )
            if new_val:
                args.dock_id = new_val
                node._args.dock_id = new_val
                status_text = f'Dock ID updated → "{new_val}"'
                status_attr = curses.color_pair(_C_OK) | curses.A_BOLD

        elif key in (ord('r'), ord('R')):
            # Re-snapshot at the moment of keypress
            robot_pose = node.get_robot_pose()
            tag_pose   = node.get_tag_pose()
            tag_age    = node.get_tag_age()
            status_text, status_attr, saved = _do_save(
                node, args, robot_pose, tag_pose, tag_age
            )
            if saved:
                last_saved = saved


# ─────────────────────────────────────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────────────────────────────────────

def _parse_args(argv=None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description=(
            'Dock position recorder — drive the robot, press [R] to save.'
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument('--tag-id', type=int, required=True,
                   help='AprilTag numeric ID (e.g. 2)')
    p.add_argument('--dock-id', type=str, default=None,
                   help='Key written to dock_database.yaml  (default: dock_<tag_id>)')
    p.add_argument('--database', type=str,
                   default=_default_database_path(),
                   help='Path to dock_database.yaml')
    p.add_argument('--global-frame', type=str, default='map',
                   help='Global / planning TF frame')
    p.add_argument('--base-frame', type=str, default='base_footprint',
                   help='Robot base TF frame')
    p.add_argument('--detections-topic', type=str, default='/detections',
                   help='AprilTagDetectionArray topic')
    p.add_argument('--final-distance', type=float, default=0.30,
                   help='Stop distance from tag (metres)')
    p.add_argument('--predocking-distance', type=float, default=1.50,
                   help='Predocking distance from tag (metres)')
    p.add_argument('--lateral-offset', type=float, default=0.0,
                   help='Side offset from tag centre (metres)')
    p.add_argument('--yaw-offset', type=float, default=0.0,
                   help='Angular correction at final pose (radians)')
    p.add_argument('--reverse', action='store_true', default=False,
                   help='Back into the dock (reverse_docking: true)')
    p.add_argument('--tag-max-age', type=float, default=1.0,
                   help='Max allowed tag age before save is refused (seconds)')

    args = p.parse_args(argv)
    if args.dock_id is None:
        args.dock_id = f'dock_{args.tag_id}'
    args.database = os.path.realpath(args.database)
    return args


def main(argv=None) -> None:
    args = _parse_args(argv)

    # Pre-flight checks before entering curses
    if not os.path.isfile(args.database):
        sys.exit(
            f'ERROR: dock_database.yaml not found at:\n  {args.database}\n'
            'Pass the correct path with --database.'
        )
    try:
        _load_database(args.database)
    except Exception as exc:
        sys.exit(f'ERROR reading database: {exc}')

    print(f'Starting dock recorder — tag_id={args.tag_id}  dock_id={args.dock_id}')
    print(f'Database: {args.database}')
    print('Launching TUI…')
    time.sleep(0.5)

    rclpy.init()
    node = DockRecorderNode(args)
    try:
        curses.wrapper(_run_tui, node, args)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print('Dock recorder exited.')


if __name__ == '__main__':
    main()
