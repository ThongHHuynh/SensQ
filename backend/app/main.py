import asyncio

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field

from .database import init_db, save_event, save_snapshot
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
    await mapping_manager.load_saved_maps_into_state()
    loop = asyncio.get_running_loop()
    ros_monitor = create_monitor(loop)
    ros_monitor.start()
    await save_snapshot(robot_state.get_snapshot())


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
    return await mapping_manager.select_map(map_id)


@app.patch("/api/maps/{map_id}")
async def rename_map(map_id: str, request: RenameMapRequest) -> dict:
    return await mapping_manager.rename(map_id, request.name)


@app.websocket("/ws/robot-state")
async def robot_state_ws(websocket: WebSocket) -> None:
    await ws_manager.connect(websocket)
    await websocket.send_json(robot_state.get_snapshot())
    try:
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        ws_manager.disconnect(websocket)
