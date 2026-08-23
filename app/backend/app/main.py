import asyncio
import math
import os

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import Response, StreamingResponse
from pydantic import BaseModel, ConfigDict, Field

from .database import (
    get_all_settings,
    init_db,
    list_mission_records,
    save_event,
    save_snapshot,
    upsert_settings,
)
from .coverage_manager import coverage_manager
from .config import DEFAULT_SETTINGS, USE_SIM_TIME
from .docking import DockingConfigError, docking_catalog
from .launch_manager import launch_manager
from .mapping_manager import mapping_manager
from .ros_monitor import create_monitor
from .state import robot_state
from .teleop_manager import teleop_manager
from .websocket_manager import ws_manager


app = FastAPI(title="SensQ Backend", version="0.1.0")
ros_monitor = None


class CmdVelRequest(BaseModel):
    linear_x: float = Field(0.0, ge=-0.5, le=0.5)
    angular_z: float = Field(0.0, ge=-1.5, le=1.5)


class SaveMapRequest(BaseModel):
    name: str = Field(..., min_length=1, max_length=120)


class RenameMapRequest(BaseModel):
    name: str = Field(..., min_length=1, max_length=120)


class DockGoalRequest(BaseModel):
    model_config = ConfigDict(allow_inf_nan=False)

    dock_id: str = Field(..., min_length=1, max_length=120)
    navigate_to_staging_pose: bool = True
    use_offset_override: bool = False
    final_distance: float | None = None
    lateral_offset: float | None = None
    yaw_offset: float | None = None


class UndockRequest(BaseModel):
    dock_id: str = Field(..., min_length=1, max_length=120)


class MissionStartRequest(BaseModel):
    mission_type: str = Field(..., min_length=1, max_length=32)
    dock_id: str = Field(..., min_length=1, max_length=120)
    auto_dock_on_complete: bool = True


class PoseRequest(BaseModel):
    model_config = ConfigDict(allow_inf_nan=False)

    x: float
    y: float
    yaw_degrees: float = Field(0.0, ge=-180.0, le=180.0)


class DockStationRequest(BaseModel):
    model_config = ConfigDict(allow_inf_nan=False)

    dock_id: str = Field(..., min_length=1, max_length=64)
    tag_id: int = Field(..., ge=0)
    staging_x: float
    staging_y: float
    staging_yaw_degrees: float = Field(..., ge=-180.0, le=180.0)
    reverse_docking: bool = False
    final_distance: float = Field(0.30, gt=0.0)
    lateral_offset: float = 0.0
    yaw_offset: float = 0.0


class CoveragePrepareRequest(BaseModel):
    use_sim_time: bool = USE_SIM_TIME


app.add_middleware(
    CORSMiddleware,
    allow_origin_regex=r"http://(localhost|127\.0\.0\.1|[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+):[0-9]+",
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.on_event("startup")
async def startup() -> None:
    global ros_monitor
    await init_db()
    await save_event("backend", "Backend started")
    persisted_settings = await get_all_settings()
    if "ros_domain_id" in persisted_settings:
        os.environ["ROS_DOMAIN_ID"] = str(persisted_settings["ros_domain_id"])
    await mapping_manager.load_saved_maps_into_state()
    loop = asyncio.get_running_loop()
    server_config = docking_catalog.get_payload()["serverConfig"]
    ros_monitor = create_monitor(
        loop,
        docking_catalog.action_name,
        tag_frames=docking_catalog.tag_frames(),
        tag_normal_sign=float(server_config.get("tag_normal_sign", -1.0)),
    )
    ros_monitor.start()
    await save_snapshot(robot_state.get_snapshot())


@app.on_event("shutdown")
async def shutdown() -> None:
    if ros_monitor is not None:
        ros_monitor.stop()
    if coverage_manager.is_running:
        await coverage_manager.stop()


@app.get("/api/health")
async def health() -> dict:
    return {"ok": True}


@app.get("/api/robot/snapshot")
async def robot_snapshot() -> dict:
    return robot_state.get_snapshot()


@app.post("/api/robot/launch/mobile-base")
async def launch_mobile_base() -> dict:
    return await launch_manager.start_mobile_base()


@app.post("/api/robot/stop")
async def stop_robot() -> dict:
    return await launch_manager.stop()


@app.post("/api/teleop/start")
async def start_teleop() -> dict:
    return await teleop_manager.start()


@app.post("/api/teleop/stop")
async def stop_teleop() -> dict:
    if ros_monitor is not None:
        ros_monitor.publish_cmd_vel(0.0, 0.0)
    return await teleop_manager.stop()


@app.post("/api/teleop/cmd_vel")
async def teleop_cmd_vel(command: CmdVelRequest) -> dict:
    if ros_monitor is None:
        return {"ok": False, "message": "ROS monitor is not initialized"}

    published, message = ros_monitor.publish_cmd_vel(command.linear_x, command.angular_z)
    return {
        "ok": published,
        "message": message,
        "linear_x": command.linear_x,
        "angular_z": command.angular_z,
    }


CAMERA_STREAM_INTERVAL_SECONDS = 1 / 15


@app.get("/api/camera/stream")
async def camera_stream() -> StreamingResponse:
    if ros_monitor is None:
        raise HTTPException(status_code=503, detail="ROS monitor is not initialized")

    async def frame_generator():
        boundary = b"--frame"
        while True:
            frame = ros_monitor.get_camera_frame()
            if frame is not None:
                yield (
                    boundary + b"\r\n"
                    b"Content-Type: image/jpeg\r\n"
                    b"Content-Length: " + str(len(frame)).encode("ascii") + b"\r\n\r\n" + frame + b"\r\n"
                )
            await asyncio.sleep(CAMERA_STREAM_INTERVAL_SECONDS)

    return StreamingResponse(
        frame_generator(),
        media_type="multipart/x-mixed-replace; boundary=frame",
    )


@app.get("/api/camera/snapshot")
async def camera_snapshot() -> Response:
    if ros_monitor is None:
        raise HTTPException(status_code=503, detail="ROS monitor is not initialized")
    frame = ros_monitor.get_camera_frame()
    if frame is None:
        raise HTTPException(status_code=404, detail="No camera frame is available yet")
    return Response(content=frame, media_type="image/jpeg")


@app.get("/api/settings")
async def get_settings() -> dict:
    persisted = await get_all_settings()
    merged = dict(DEFAULT_SETTINGS)
    merged.update(persisted)
    return merged


@app.put("/api/settings")
async def put_settings(payload: dict) -> dict:
    unknown = set(payload.keys()) - set(DEFAULT_SETTINGS.keys())
    if unknown:
        raise HTTPException(status_code=400, detail=f"Unknown settings keys: {', '.join(sorted(unknown))}")
    persisted = await upsert_settings(payload)
    if "ros_domain_id" in payload:
        os.environ["ROS_DOMAIN_ID"] = str(payload["ros_domain_id"])
    await save_event("settings", f"Updated settings: {', '.join(sorted(payload.keys()))}")
    merged = dict(DEFAULT_SETTINGS)
    merged.update(persisted)
    return merged


@app.get("/api/docking/config")
async def docking_config() -> dict:
    return docking_catalog.get_payload()


@app.post("/api/docking/start")
async def start_docking(request: DockGoalRequest) -> dict:
    if ros_monitor is None:
        raise HTTPException(status_code=503, detail="ROS monitor is not initialized")
    try:
        goal = docking_catalog.normalize_goal(request.model_dump())
    except DockingConfigError as error:
        raise HTTPException(status_code=400, detail=str(error)) from error

    started, message = ros_monitor.start_docking(goal)
    if not started:
        raise HTTPException(status_code=409, detail=message)
    await save_event("docking", message)
    return {"ok": True, "message": message, "docking": robot_state.get_snapshot()["docking"]}


@app.post("/api/docking/cancel")
async def cancel_docking() -> dict:
    if ros_monitor is None:
        raise HTTPException(status_code=503, detail="ROS monitor is not initialized")
    cancelled, message = ros_monitor.cancel_docking()
    if not cancelled:
        raise HTTPException(status_code=409, detail=message)
    await save_event("docking", message)
    return {"ok": True, "message": message, "docking": robot_state.get_snapshot()["docking"]}


@app.post("/api/docking/undock")
async def undock(request: UndockRequest) -> dict:
    if ros_monitor is None:
        raise HTTPException(status_code=503, detail="ROS monitor is not initialized")
    started, message = ros_monitor.send_undock_goal(request.dock_id)
    if not started:
        raise HTTPException(status_code=409, detail=message)
    await save_event("undocking", message)
    return {"ok": True, "message": message, "undocking": robot_state.get_snapshot()["undocking"]}


@app.post("/api/docking/undock/cancel")
async def cancel_undock() -> dict:
    if ros_monitor is None:
        raise HTTPException(status_code=503, detail="ROS monitor is not initialized")
    cancelled, message = ros_monitor.cancel_undock()
    if not cancelled:
        raise HTTPException(status_code=409, detail=message)
    await save_event("undocking", message)
    return {"ok": True, "message": message, "undocking": robot_state.get_snapshot()["undocking"]}


@app.post("/api/mission/start")
async def start_mission(request: MissionStartRequest) -> dict:
    if ros_monitor is None:
        raise HTTPException(status_code=503, detail="ROS monitor is not initialized")
    started, message = ros_monitor.start_mission(
        request.mission_type, request.dock_id, request.auto_dock_on_complete
    )
    if not started:
        raise HTTPException(status_code=409, detail=message)
    await save_event("mission", message)
    return {"ok": True, "message": message, "mission": robot_state.get_snapshot()["mission"]}


@app.post("/api/mission/cancel")
async def cancel_mission() -> dict:
    if ros_monitor is None:
        raise HTTPException(status_code=503, detail="ROS monitor is not initialized")
    cancelled, message = ros_monitor.cancel_mission()
    if not cancelled:
        raise HTTPException(status_code=409, detail=message)
    await save_event("mission", message)
    return {"ok": True, "message": message, "mission": robot_state.get_snapshot()["mission"]}


@app.get("/api/mission/history")
async def mission_history() -> dict:
    records = await list_mission_records()
    return {
        "missions": [
            {
                "id": record.id,
                "missionType": record.mission_type,
                "dockId": record.dock_id,
                "startedAt": record.started_at.isoformat() if record.started_at else None,
                "completedAt": record.completed_at.isoformat() if record.completed_at else None,
                "success": record.success,
                "areaCoveredM2": record.area_covered_m2,
                "durationSeconds": record.duration_seconds,
                "message": record.message,
            }
            for record in records
        ]
    }


@app.post("/api/docking/stations")
async def save_dock_station(request: DockStationRequest) -> dict:
    if ros_monitor is None:
        raise HTTPException(status_code=503, detail="ROS monitor is not initialized")
    observation = ros_monitor.get_tag_observation(request.tag_id)
    if observation is None or not observation.get("visible"):
        raise HTTPException(
            status_code=409,
            detail=f"AprilTag {request.tag_id} is not currently visible",
        )
    if observation.get("mapX") is None or observation.get("mapY") is None:
        raise HTTPException(
            status_code=409,
            detail="The tag has no map coordinate; localize the robot before saving",
        )
    reference_yaw = observation.get("mapYaw")
    if reference_yaw is None:
        reference_yaw = math.radians(request.staging_yaw_degrees)

    try:
        station = docking_catalog.save_station(
            {
                "dock_id": request.dock_id,
                "tag_id": request.tag_id,
                "tag_frame": observation.get("tagFrame", f"tag_{request.tag_id}"),
                "reference_pose": [
                    observation["mapX"],
                    observation["mapY"],
                    reference_yaw,
                ],
                "staging_pose": [
                    request.staging_x,
                    request.staging_y,
                    math.radians(request.staging_yaw_degrees),
                ],
                "final_distance": request.final_distance,
                "lateral_offset": request.lateral_offset,
                "yaw_offset": request.yaw_offset,
                "reverse_docking": request.reverse_docking,
            }
        )
    except DockingConfigError as error:
        raise HTTPException(status_code=400, detail=str(error)) from error

    ros_monitor.set_tag_frames(docking_catalog.tag_frames())
    reloaded, reload_message = await asyncio.to_thread(ros_monitor.reload_docks)
    message = f"Saved docking station '{request.dock_id}'"
    if not reloaded:
        message += f". Restart docking before use: {reload_message}"
    await save_event("docking", message)
    return {
        "ok": True,
        "message": message,
        "reloaded": reloaded,
        "station": station,
        "catalog": docking_catalog.get_payload(),
    }


@app.get("/api/maps")
async def saved_maps() -> dict:
    snapshot = await mapping_manager.load_saved_maps_into_state()
    return {"maps": snapshot["maps"]}


@app.post("/api/mapping/start")
async def start_mapping() -> dict:
    return await mapping_manager.start()


@app.post("/api/mapping/stop")
async def stop_mapping() -> dict:
    return await mapping_manager.stop()


@app.post("/api/mapping/reset")
async def reset_mapping() -> dict:
    return await mapping_manager.reset()


@app.post("/api/mapping/save")
async def save_map(request: SaveMapRequest) -> dict:
    return await mapping_manager.save(request.name)


@app.post("/api/maps/{map_id}/select")
async def select_map(map_id: str) -> dict:
    selected = await mapping_manager.resolve_map(map_id)
    if selected is None:
        raise HTTPException(status_code=404, detail="Map not found")
    yaml_path = selected.get("yamlPath")
    if yaml_path:
        if ros_monitor is None:
            raise HTTPException(status_code=503, detail="ROS monitor is not initialized")
        loaded, message = await asyncio.to_thread(ros_monitor.load_map, yaml_path)
        if not loaded:
            raise HTTPException(status_code=409, detail=message)
    return await mapping_manager.select_map(map_id)


@app.patch("/api/maps/{map_id}")
async def rename_map(map_id: str, request: RenameMapRequest) -> dict:
    return await mapping_manager.rename(map_id, request.name)


@app.post("/api/navigation/initial-pose")
async def set_initial_pose(request: PoseRequest) -> dict:
    if ros_monitor is None:
        raise HTTPException(status_code=503, detail="ROS monitor is not initialized")
    success, message = ros_monitor.set_initial_pose(
        request.x,
        request.y,
        request.yaw_degrees,
    )
    if not success:
        raise HTTPException(status_code=409, detail=message)
    await save_event("navigation", message)
    return {"ok": True, "message": message}


@app.post("/api/navigation/goals")
async def send_navigation_goal(request: PoseRequest) -> dict:
    if ros_monitor is None:
        raise HTTPException(status_code=503, detail="ROS monitor is not initialized")
    success, message = ros_monitor.start_navigation(
        request.x,
        request.y,
        request.yaw_degrees,
    )
    if not success:
        raise HTTPException(status_code=409, detail=message)
    await save_event("navigation", message)
    return {"ok": True, "message": message}


@app.post("/api/navigation/cancel")
async def cancel_navigation_goal() -> dict:
    if ros_monitor is None:
        raise HTTPException(status_code=503, detail="ROS monitor is not initialized")
    success, message = ros_monitor.cancel_navigation()
    if not success:
        raise HTTPException(status_code=409, detail=message)
    return {"ok": True, "message": message}


@app.post("/api/coverage/prepare")
async def prepare_coverage(request: CoveragePrepareRequest) -> dict:
    if ros_monitor is None:
        raise HTTPException(status_code=503, detail="ROS monitor is not initialized")
    if ros_monitor.coverage_services_ready():
        return {"ok": True, "running": True, "message": "Coverage services are ready"}
    result = await coverage_manager.start(request.use_sim_time)
    if not result["ok"]:
        raise HTTPException(status_code=409, detail=result["message"])
    for _ in range(30):
        if ros_monitor.coverage_services_ready():
            return {"ok": True, "running": True, "message": "Coverage services are ready"}
        await asyncio.sleep(0.25)
    raise HTTPException(status_code=504, detail="Coverage services did not become ready")


@app.post("/api/coverage/generate")
async def generate_coverage() -> dict:
    if ros_monitor is None:
        raise HTTPException(status_code=503, detail="ROS monitor is not initialized")
    success, message = await asyncio.to_thread(ros_monitor.generate_coverage)
    if not success:
        raise HTTPException(status_code=409, detail=message)
    await save_event("coverage", "Generated coverage path")
    return {"ok": True, "message": message}


@app.post("/api/coverage/execute")
async def execute_coverage() -> dict:
    if ros_monitor is None:
        raise HTTPException(status_code=503, detail="ROS monitor is not initialized")
    success, message = await asyncio.to_thread(ros_monitor.execute_coverage)
    if not success:
        raise HTTPException(status_code=409, detail=message)
    await save_event("coverage", message)
    return {"ok": True, "message": message}


@app.post("/api/coverage/cancel")
async def cancel_coverage() -> dict:
    if ros_monitor is None:
        raise HTTPException(status_code=503, detail="ROS monitor is not initialized")
    success, message = await asyncio.to_thread(ros_monitor.cancel_coverage)
    if not success:
        raise HTTPException(status_code=409, detail=message)
    await save_event("coverage", message)
    return {"ok": True, "message": message}


@app.websocket("/ws/robot-state")
async def robot_state_ws(websocket: WebSocket) -> None:
    await ws_manager.connect(websocket)
    await websocket.send_json(robot_state.get_snapshot())
    try:
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        ws_manager.disconnect(websocket)
