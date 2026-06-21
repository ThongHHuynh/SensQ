#!/usr/bin/env python3

import json
import os
import signal
import subprocess
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse


HOST = "0.0.0.0"
PORT = 8080
WORKSPACE_ROOT = Path("/home/tom/ros2_ws")
STATIC_DIR = WORKSPACE_ROOT / "webapp" / "static"
LOG_LIMIT = 400

LAUNCH_OPTIONS = {
    "robot": {
        "label": "Robot Bringup",
        "package": "my_robot_bringup",
        "launch_file": "my_robot.launch.py",
        "description": "Starts robot_state_publisher, ros2_control, lidar, and slam_toolbox.",
    },
    "lidar": {
        "label": "Lidar Only",
        "package": "my_robot_bringup",
        "launch_file": "lidar.launch.py",
        "description": "Starts the sllidar node using your current lidar.launch.py defaults.",
    },
    "rviz": {
        "label": "RViz Only",
        "package": "my_robot_bringup",
        "launch_file": "rviz.launch.py",
        "description": "Starts RViz with my_robot_description/rviz/urdf_config.rviz.",
    },
    "gazebo": {
        "label": "Gazebo Sim",
        "package": "my_robot_bringup",
        "launch_file": "gazebo.launch.py",
        "description": "Starts the Gazebo simulation bringup.",
    },
}


class LaunchManager:
    def __init__(self) -> None:
        self.process = None
        self.process_key = None
        self.started_at = None
        self.lock = threading.Lock()
        self.log_lines = []
        self.reader_thread = None

    def _append_log(self, line: str) -> None:
        self.log_lines.append(line.rstrip())
        if len(self.log_lines) > LOG_LIMIT:
            self.log_lines = self.log_lines[-LOG_LIMIT:]

    def _stream_output(self, process: subprocess.Popen) -> None:
        assert process.stdout is not None
        for raw_line in iter(process.stdout.readline, ""):
            if not raw_line:
                break
            line = raw_line.rstrip("\n")
            with self.lock:
                self._append_log(line)
        process.stdout.close()

    def _build_command(self, option_key: str) -> str:
        option = LAUNCH_OPTIONS[option_key]
        ros_distro = os.environ.get("ROS_DISTRO", "jazzy")
        ros_setup = f"/opt/ros/{ros_distro}/setup.bash"
        workspace_setup = WORKSPACE_ROOT / "install" / "setup.bash"

        setup_parts = []
        if Path(ros_setup).exists():
            setup_parts.append(f"source {ros_setup}")
        if workspace_setup.exists():
            setup_parts.append(f"source {workspace_setup}")

        setup_cmd = " && ".join(setup_parts)
        launch_cmd = f"ros2 launch {option['package']} {option['launch_file']}"
        if setup_cmd:
            return f"{setup_cmd} && {launch_cmd}"
        return launch_cmd

    def start(self, option_key: str) -> dict:
        if option_key not in LAUNCH_OPTIONS:
            raise ValueError("Unknown launch option.")

        with self.lock:
            if self.process and self.process.poll() is None:
                raise RuntimeError("A launch is already running. Stop it before starting another one.")

            command = self._build_command(option_key)
            self.log_lines = [f"$ {command}"]
            self.process = subprocess.Popen(
                ["bash", "-lc", command],
                cwd=str(WORKSPACE_ROOT),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                preexec_fn=os.setsid,
            )
            self.process_key = option_key
            self.started_at = time.time()
            self.reader_thread = threading.Thread(
                target=self._stream_output,
                args=(self.process,),
                daemon=True,
            )
            self.reader_thread.start()
            return self.status()

    def stop(self) -> dict:
        with self.lock:
            if not self.process or self.process.poll() is not None:
                self.process = None
                self.process_key = None
                self.started_at = None
                return self.status()

            os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)

        deadline = time.time() + 5
        while time.time() < deadline:
            if self.process.poll() is not None:
                break
            time.sleep(0.1)

        with self.lock:
            if self.process and self.process.poll() is None:
                os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
            self._append_log("[webapp] Launch stopped.")
            self.process = None
            self.process_key = None
            self.started_at = None
            return self.status()

    def status(self) -> dict:
        running = False
        return_code = None
        option = None
        uptime_seconds = None

        if self.process:
            return_code = self.process.poll()
            running = return_code is None
            if self.process_key:
                option = {
                    "key": self.process_key,
                    **LAUNCH_OPTIONS[self.process_key],
                }
            if running and self.started_at:
                uptime_seconds = round(time.time() - self.started_at, 1)

        return {
            "running": running,
            "return_code": return_code,
            "current_launch": option,
            "uptime_seconds": uptime_seconds,
            "log_lines": self.log_lines[-120:],
            "launch_options": [
                {"key": key, **value} for key, value in LAUNCH_OPTIONS.items()
            ],
        }


MANAGER = LaunchManager()


class RequestHandler(BaseHTTPRequestHandler):
    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path == "/":
            self._serve_file("index.html", "text/html; charset=utf-8")
            return
        if parsed.path == "/app.js":
            self._serve_file("app.js", "application/javascript; charset=utf-8")
            return
        if parsed.path == "/styles.css":
            self._serve_file("styles.css", "text/css; charset=utf-8")
            return
        if parsed.path == "/api/status":
            self._send_json(MANAGER.status())
            return
        self.send_error(HTTPStatus.NOT_FOUND, "Not found")

    def do_POST(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path == "/api/start":
            payload = self._read_json()
            try:
                data = MANAGER.start(payload.get("launch_key", ""))
                self._send_json(data)
            except (ValueError, RuntimeError) as exc:
                self._send_json({"error": str(exc)}, status=HTTPStatus.BAD_REQUEST)
            return
        if parsed.path == "/api/stop":
            self._send_json(MANAGER.stop())
            return
        self.send_error(HTTPStatus.NOT_FOUND, "Not found")

    def log_message(self, format: str, *args) -> None:
        return

    def _read_json(self) -> dict:
        length = int(self.headers.get("Content-Length", "0"))
        if length == 0:
            return {}
        raw = self.rfile.read(length).decode("utf-8")
        return json.loads(raw)

    def _serve_file(self, filename: str, content_type: str) -> None:
        path = STATIC_DIR / filename
        if not path.exists():
            self.send_error(HTTPStatus.NOT_FOUND, "Static file not found")
            return
        body = path.read_bytes()
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_json(self, payload: dict, status: int = HTTPStatus.OK) -> None:
        body = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def main() -> None:
    server = ThreadingHTTPServer((HOST, PORT), RequestHandler)
    print(f"Serving ROS 2 web app at http://localhost:{PORT}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        MANAGER.stop()
        server.server_close()


if __name__ == "__main__":
    main()
