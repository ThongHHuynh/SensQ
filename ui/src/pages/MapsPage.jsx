import {
  ArrowDown,
  ArrowLeft,
  ArrowRight,
  ArrowUp,
  CircleStop,
  Crosshair,
  Disc3,
  Gamepad2,
  Keyboard,
  LocateFixed,
  Play,
  Pencil,
  Save,
  Send,
  Square
} from "lucide-react";
import { useEffect, useRef, useState } from "react";
import PageHeader from "../components/PageHeader.jsx";
import { renameMap, saveMap, selectMap, sendTeleopCommand, startMapping, startTeleop, stopMapping, stopTeleop } from "../services/robotApi.js";

const DEG_TO_RAD = Math.PI / 180;

function MapsPage({ robot }) {
  const maps = Array.isArray(robot.maps) ? robot.maps : [];
  const pose = robot.pose ?? {};
  const navigation = robot.navigation ?? {};
  const liveMap = robot.liveMap ?? {};
  const mappingState = navigation.mapping ?? "idle";
  const [mapName, setMapName] = useState("Lab map");
  const [mapMessage, setMapMessage] = useState("Select or rename saved maps.");
  const [editingMapId, setEditingMapId] = useState(null);
  const [editingMapName, setEditingMapName] = useState("");
  const [mappingMessage, setMappingMessage] = useState("Start mapping, drive the robot, then save the map.");
  const [linearSpeed, setLinearSpeed] = useState(0.2);
  const [angularSpeed, setAngularSpeed] = useState(0.6);
  const [teleopReady, setTeleopReady] = useState(false);
  const [teleopStatus, setTeleopStatus] = useState("idle");
  const [teleopMessage, setTeleopMessage] = useState("Start teleop before sending web drive commands.");
  const [teleopMode, setTeleopMode] = useState("keyboard");

  async function handleStartTeleop() {
    setTeleopStatus("starting");
    setTeleopMessage("Starting teleop...");
    try {
      const result = await startTeleop();
      setTeleopReady(Boolean(result.running));
      setTeleopStatus(result.ok && result.running ? "online" : "error");
      setTeleopMessage(result.message || "Teleop started");
    } catch (err) {
      setTeleopReady(false);
      setTeleopStatus("error");
      setTeleopMessage(err.message);
    }
  }

  async function handleStopTeleop() {
    try {
      await sendTeleopCommand({ linear_x: 0, angular_z: 0 });
      const result = await stopTeleop();
      setTeleopReady(false);
      setTeleopStatus("idle");
      setTeleopMessage(result.message || "Teleop stopped");
    } catch (err) {
      setTeleopStatus("error");
      setTeleopMessage(err.message);
    }
  }

  async function handleDrive(linear_x, angular_z) {
    try {
      const result = await sendTeleopCommand({ linear_x, angular_z });
      setTeleopStatus(result.ok ? "online" : "error");
      setTeleopMessage(result.message || "Command sent");
    } catch (err) {
      setTeleopStatus("error");
      setTeleopMessage(err.message);
    }
  }

  async function handleStartMapping() {
    try {
      const result = await startMapping();
      setMappingMessage(result.message || "Mapping started");
    } catch (err) {
      setMappingMessage(err.message);
    }
  }

  async function handleStopMapping() {
    try {
      const result = await stopMapping();
      setMappingMessage(result.message || "Mapping stopped");
    } catch (err) {
      setMappingMessage(err.message);
    }
  }

  async function handleSaveMap() {
    try {
      const result = await saveMap(mapName);
      setMappingMessage(result.message || "Map saved");
    } catch (err) {
      setMappingMessage(err.message);
    }
  }

  async function handleSelectMap(map) {
    try {
      const result = await selectMap(map.id);
      setMapMessage(result.message || "Map selected");
    } catch (err) {
      setMapMessage(err.message);
    }
  }

  function handleStartRename(map) {
    setEditingMapId(map.id);
    setEditingMapName(map.name || "");
  }

  async function handleRenameMap(map) {
    try {
      const result = await renameMap(map.id, editingMapName);
      setEditingMapId(null);
      setMapMessage(result.message || "Map renamed");
    } catch (err) {
      setMapMessage(err.message);
    }
  }

  return (
    <>
      <PageHeader
        eyebrow="Maps"
        title="Mapping and localization"
        description="Placeholder map surface for later Nav2, SLAM, saved map selection, and goal-setting workflows."
        actions={
          <>
            <button className="inline-flex h-10 items-center gap-2 rounded-md bg-console-rail px-3 text-sm font-semibold text-white">
              <LocateFixed className="h-4 w-4" aria-hidden="true" />
              Center
            </button>
            <button className="inline-flex h-10 items-center gap-2 rounded-md border border-console-line bg-white px-3 text-sm font-semibold text-console-ink">
              <Crosshair className="h-4 w-4" aria-hidden="true" />
              Set Goal
            </button>
          </>
        }
      />

      <section className="grid gap-4 xl:grid-cols-[1fr_280px]">
        <div className="space-y-4">
          <div className="relative min-h-[72vh] overflow-hidden rounded-md border border-console-line bg-white">
            <LiveMapCanvas liveMap={liveMap} pose={pose} />
            <div className="absolute bottom-4 left-4 rounded-md border border-console-line bg-white px-3 py-2 text-sm">
              Active map: {navigation.activeMap ?? "Waiting"}
            </div>
            <div className="absolute right-4 top-4 rounded-md border border-console-line bg-white px-3 py-2 text-sm">
              <div className="font-semibold text-console-ink">Robot pose</div>
              <div className="mt-1 font-mono text-xs text-slate-600">
                {pose.frame ?? "odom"} x {pose.x ?? 0}, y {pose.y ?? 0}, yaw {pose.yaw ?? 0} deg
              </div>
            </div>
          </div>

          <div className="grid gap-4 xl:grid-cols-2">
            <div className="rounded-md border border-console-line bg-white p-4 shadow-soft">
          <div className="flex flex-col gap-3 lg:flex-row lg:items-center lg:justify-between">
            <div>
              <div className="flex items-center gap-2">
                <Disc3 className="h-5 w-5 text-signal-info" aria-hidden="true" />
                <h2 className="text-lg font-semibold tracking-normal">Mapping</h2>
                <MappingStatusLight status={mappingState} />
              </div>
              <p className="mt-1 text-sm text-slate-600">{mappingMessage}</p>
            </div>
            <div className="flex flex-wrap gap-2">
              <button
                type="button"
                onClick={handleStartMapping}
                className="inline-flex h-9 items-center gap-2 rounded-md bg-console-rail px-3 text-sm font-semibold text-white"
              >
                <Play className="h-4 w-4" aria-hidden="true" />
                Start
              </button>
              <button
                type="button"
                onClick={handleStopMapping}
                className="inline-flex h-9 items-center gap-2 rounded-md border border-console-line bg-white px-3 text-sm font-semibold text-console-ink"
              >
                <Square className="h-4 w-4" aria-hidden="true" />
                Stop
              </button>
            </div>
          </div>

          <div className="mt-4 grid gap-3 lg:grid-cols-[1fr_auto]">
            <label className="block">
              <span className="mb-2 block text-sm font-medium text-slate-600">Map name</span>
              <input
                className="h-10 w-full rounded-md border border-console-line px-3"
                value={mapName}
                onChange={(event) => setMapName(event.target.value)}
                placeholder="Enter map name"
              />
            </label>
            <button
              type="button"
              onClick={handleSaveMap}
              className="inline-flex h-10 items-center gap-2 self-end rounded-md bg-signal-info px-3 text-sm font-semibold text-white"
            >
              <Save className="h-4 w-4" aria-hidden="true" />
              Save Map
            </button>
          </div>
            </div>

            <div className="rounded-md border border-console-line bg-white p-4 shadow-soft">
          <div className="flex flex-col gap-3 lg:flex-row lg:items-center lg:justify-between">
            <div>
              <div className="flex items-center gap-2">
                <Crosshair className="h-5 w-5 text-signal-info" aria-hidden="true" />
                <h2 className="text-lg font-semibold tracking-normal">Navigation</h2>
                <span className="inline-flex items-center gap-2 rounded-md border border-console-line bg-white px-2 py-1 text-xs font-semibold text-slate-600">
                  <span className="h-2.5 w-2.5 rounded-full bg-slate-300" aria-hidden="true" />
                  Goal idle
                </span>
              </div>
              <p className="mt-1 text-sm text-slate-600">Goal selection controls will use this panel.</p>
            </div>
          </div>
          <div className="mt-4 grid gap-3 sm:grid-cols-3">
            <label className="block">
              <span className="mb-2 block text-sm font-medium text-slate-600">Goal X</span>
              <input className="h-10 w-full rounded-md border border-console-line px-3" defaultValue="0.0" />
            </label>
            <label className="block">
              <span className="mb-2 block text-sm font-medium text-slate-600">Goal Y</span>
              <input className="h-10 w-full rounded-md border border-console-line px-3" defaultValue="0.0" />
            </label>
            <label className="block">
              <span className="mb-2 block text-sm font-medium text-slate-600">Yaw</span>
              <input className="h-10 w-full rounded-md border border-console-line px-3" defaultValue="0" />
            </label>
          </div>
          <button
            type="button"
            className="mt-4 inline-flex h-10 items-center gap-2 rounded-md bg-console-rail px-3 text-sm font-semibold text-white"
          >
            <Send className="h-4 w-4" aria-hidden="true" />
            Send Goal
          </button>
            </div>
          </div>
        </div>

        <aside className="space-y-4">
          <div className="rounded-md border border-console-line bg-white p-4">
            <h2 className="text-base font-semibold tracking-normal">Available maps</h2>
            <p className="mt-1 text-xs text-slate-500">{mapMessage}</p>
            <div className="mt-3 max-h-[34vh] space-y-2 overflow-y-auto pr-1">
              {maps.map((map) => {
                const isActive = map.name === navigation.activeMap;
                const isSaved = String(map.id || "").startsWith("saved-");
                const isEditing = editingMapId === map.id;

                return (
                  <div
                    key={map.id || map.name}
                    className={`rounded-md border p-2 transition ${
                      isActive ? "border-emerald-400 bg-emerald-50" : "border-console-line bg-console-panel"
                    }`}
                  >
                    {isEditing ? (
                      <div className="flex gap-2">
                        <input
                          className="h-8 min-w-0 flex-1 rounded-md border border-console-line px-2 text-xs"
                          value={editingMapName}
                          onChange={(event) => setEditingMapName(event.target.value)}
                        />
                        <button
                          type="button"
                          onClick={() => handleRenameMap(map)}
                          className="h-8 rounded-md bg-console-rail px-2 text-xs font-semibold text-white"
                        >
                          Save
                        </button>
                      </div>
                    ) : (
                      <div className="flex items-start justify-between gap-2">
                        <button type="button" onClick={() => handleSelectMap(map)} className="min-w-0 flex-1 text-left">
                          <div className="flex items-center gap-2">
                            <span className="truncate text-sm font-medium text-console-ink">{map.name || "Unnamed map"}</span>
                            {isActive ? <span className="h-2.5 w-2.5 shrink-0 rounded-full bg-emerald-500" aria-label="Active map" /> : null}
                          </div>
                          <div className="mt-1 text-xs text-slate-500">{map.resolution || "Resolution unknown"}</div>
                        </button>
                        {isSaved ? (
                          <button
                            type="button"
                            onClick={() => handleStartRename(map)}
                            className="flex h-7 w-7 shrink-0 items-center justify-center rounded-md border border-console-line bg-white text-slate-600"
                            aria-label={`Rename ${map.name}`}
                            title="Rename map"
                          >
                            <Pencil className="h-3.5 w-3.5" aria-hidden="true" />
                          </button>
                        ) : null}
                      </div>
                    )}
                  </div>
                );
              })}
              {maps.length === 0 ? <div className="text-sm text-slate-500">Waiting for map metadata.</div> : null}
            </div>
          </div>

          <div className="rounded-md border border-console-line bg-white p-4 shadow-soft">
            <div className="flex items-center gap-2">
              <Gamepad2 className="h-5 w-5 text-signal-info" aria-hidden="true" />
              <h2 className="text-base font-semibold tracking-normal">Teleoperation</h2>
              <TeleopStatusLight status={teleopStatus} />
            </div>
            <p className="mt-1 text-xs text-slate-600">{teleopMessage}</p>
            <div className="mt-3 flex flex-wrap gap-2">
              <button
                type="button"
                onClick={handleStartTeleop}
                className="inline-flex h-9 items-center gap-2 rounded-md bg-console-rail px-3 text-sm font-semibold text-white"
              >
                <Gamepad2 className="h-4 w-4" aria-hidden="true" />
                Start
              </button>
              <button
                type="button"
                onClick={handleStopTeleop}
                className="inline-flex h-9 items-center gap-2 rounded-md border border-console-line bg-white px-3 text-sm font-semibold text-console-ink"
              >
                <CircleStop className="h-4 w-4" aria-hidden="true" />
                Stop
              </button>
            </div>

            <div className="mt-4 inline-flex rounded-md border border-console-line bg-console-panel p-1">
              <ModeButton active={teleopMode === "keyboard"} icon={Keyboard} label="Keyboard" onClick={() => setTeleopMode("keyboard")} />
              <ModeButton active={teleopMode === "joystick"} icon={Gamepad2} label="Joystick" onClick={() => setTeleopMode("joystick")} />
            </div>

            {teleopMode === "keyboard" ? (
              <div className="mt-4 grid w-[156px] grid-cols-3 gap-2 justify-self-start">
                <div />
                <TeleopButton disabled={!teleopReady} label="Forward" icon={ArrowUp} onClick={() => handleDrive(linearSpeed, 0)} />
                <div />
                <TeleopButton disabled={!teleopReady} label="Left" icon={ArrowLeft} onClick={() => handleDrive(0, angularSpeed)} />
                <TeleopButton disabled={!teleopReady} label="Stop" icon={CircleStop} onClick={() => handleDrive(0, 0)} tone="stop" />
                <TeleopButton disabled={!teleopReady} label="Right" icon={ArrowRight} onClick={() => handleDrive(0, -angularSpeed)} />
                <div />
                <TeleopButton disabled={!teleopReady} label="Reverse" icon={ArrowDown} onClick={() => handleDrive(-linearSpeed, 0)} />
                <div />
              </div>
            ) : (
              <JoystickControl
                disabled={!teleopReady}
                linearSpeed={linearSpeed}
                angularSpeed={angularSpeed}
                onCommand={handleDrive}
              />
            )}

            <div className="mt-4 space-y-4 rounded-md border border-console-line bg-console-panel p-3 text-sm text-slate-600">
              <SpeedControl
                label="Linear"
                value={linearSpeed}
                min={0.05}
                max={0.5}
                step={0.05}
                unit="m/s"
                onChange={setLinearSpeed}
              />
              <SpeedControl
                label="Angular"
                value={angularSpeed}
                min={0.1}
                max={1.5}
                step={0.1}
                unit="rad/s"
                onChange={setAngularSpeed}
              />
            </div>
          </div>
        </aside>
      </section>
    </>
  );
}

function MappingStatusLight({ status }) {
  const styles = {
    idle: "bg-slate-300",
    active: "bg-emerald-500",
    saving: "bg-amber-400",
    error: "bg-orange-600"
  };

  const labels = {
    idle: "Mapping idle",
    active: "Mapping active",
    saving: "Saving map",
    error: "Mapping error"
  };

  return (
    <span className="inline-flex items-center gap-2 rounded-md border border-console-line bg-white px-2 py-1 text-xs font-semibold text-slate-600">
      <span className={`h-2.5 w-2.5 rounded-full ${styles[status] ?? styles.idle}`} aria-hidden="true" />
      {labels[status] ?? labels.idle}
    </span>
  );
}

function LiveMapCanvas({ liveMap, pose }) {
  const canvasRef = useRef(null);
  const hasMap = Array.isArray(liveMap.data) && liveMap.data.length > 0 && liveMap.width > 0 && liveMap.height > 0;

  useEffect(() => {
    if (!hasMap || !canvasRef.current) return;

    const canvas = canvasRef.current;
    const ctx = canvas.getContext("2d");
    const width = Number(liveMap.width);
    const height = Number(liveMap.height);
    const scale = Math.max(1, Math.floor(1200 / Math.max(width, height)));
    canvas.width = width * scale;
    canvas.height = height * scale;

    ctx.fillStyle = "#d8dde7";
    ctx.fillRect(0, 0, canvas.width, canvas.height);

    for (let y = 0; y < height; y += 1) {
      for (let x = 0; x < width; x += 1) {
        const value = liveMap.data[y * width + x];
        if (value < 0) {
          ctx.fillStyle = "#cfd5df";
        } else if (value >= 50) {
          ctx.fillStyle = "#111827";
        } else {
          ctx.fillStyle = "#ffffff";
        }

        const canvasY = height - y - 1;
        ctx.fillRect(x * scale, canvasY * scale, scale, scale);
      }
    }

    const resolution = Number(liveMap.resolution || 0);
    const origin = liveMap.origin || {};
    const poseX = Number(pose?.x ?? 0);
    const poseY = Number(pose?.y ?? 0);
    const yaw = Number(pose?.yaw ?? 0);

    if (resolution > 0) {
      const mapX = (poseX - Number(origin.x ?? 0)) / resolution;
      const mapY = (poseY - Number(origin.y ?? 0)) / resolution;
      const canvasX = mapX * scale;
      const canvasY = (height - mapY - 1) * scale;

      if (canvasX >= 0 && canvasX <= canvas.width && canvasY >= 0 && canvasY <= canvas.height) {
        ctx.save();
        ctx.translate(canvasX, canvasY);
        ctx.rotate(-DEG_TO_RAD * yaw + Math.PI / 2);
        ctx.fillStyle = "#2563eb";
        ctx.strokeStyle = "#ffffff";
        ctx.lineWidth = Math.max(2, scale * 0.55);
        ctx.beginPath();
        ctx.moveTo(0, -Math.max(10, scale * 2.4));
        ctx.lineTo(Math.max(7, scale * 1.8), Math.max(8, scale * 1.8));
        ctx.lineTo(0, Math.max(4, scale));
        ctx.lineTo(-Math.max(7, scale * 1.8), Math.max(8, scale * 1.8));
        ctx.closePath();
        ctx.stroke();
        ctx.fill();
        ctx.restore();
      }
    }
  }, [hasMap, liveMap, pose]);

  if (!hasMap) {
    return (
      <div className="map-grid absolute inset-0 flex items-center justify-center bg-[#f8fafc]">
        <div className="rounded-md border border-console-line bg-white px-4 py-3 text-sm text-slate-600">
          Waiting for `/map` from slam_toolbox
        </div>
      </div>
    );
  }

  return (
    <div className="absolute inset-0 flex items-center justify-center bg-[#d8dde7]">
      <canvas ref={canvasRef} className="h-full w-full object-contain [image-rendering:pixelated]" />
      <div className="absolute left-4 top-4 rounded-md border border-console-line bg-white px-3 py-2 text-xs text-slate-600">
        <div className="font-semibold text-console-ink">/map</div>
        <div className="mt-1">{liveMap.width} x {liveMap.height}</div>
        <div>{liveMap.resolution ?? "?"} m/px</div>
        <div>{liveMap.status ?? "Receiving map"}</div>
      </div>
    </div>
  );
}

function TeleopStatusLight({ status }) {
  const styles = {
    idle: "bg-slate-300",
    starting: "bg-amber-400",
    online: "bg-emerald-500",
    error: "bg-orange-600"
  };

  const labels = {
    idle: "Teleop idle",
    starting: "Teleop starting",
    online: "Teleop online",
    error: "Teleop error"
  };

  return (
    <span className="inline-flex items-center gap-2 rounded-md border border-console-line bg-white px-2 py-1 text-xs font-semibold text-slate-600">
      <span className={`h-2.5 w-2.5 rounded-full ${styles[status] ?? styles.idle}`} aria-hidden="true" />
      {labels[status] ?? labels.idle}
    </span>
  );
}

function ModeButton({ active, icon: Icon, label, onClick }) {
  return (
    <button
      type="button"
      onClick={onClick}
      className={`inline-flex h-8 items-center gap-2 rounded px-2 text-xs font-semibold transition ${
        active ? "bg-white text-console-ink shadow-sm" : "text-slate-500 hover:text-console-ink"
      }`}
    >
      <Icon className="h-3.5 w-3.5" aria-hidden="true" />
      {label}
    </button>
  );
}

function JoystickControl({ disabled, linearSpeed, angularSpeed, onCommand }) {
  const padRef = useRef(null);
  const commandRef = useRef({ linear_x: 0, angular_z: 0 });
  const onCommandRef = useRef(onCommand);
  const [isActive, setIsActive] = useState(false);
  const [stick, setStick] = useState({ x: 0, y: 0 });

  useEffect(() => {
    onCommandRef.current = onCommand;
  }, [onCommand]);

  function updateStick(event) {
    const pad = padRef.current;
    if (!pad || disabled) return;

    const rect = pad.getBoundingClientRect();
    const radius = rect.width / 2;
    const centerX = rect.left + radius;
    const centerY = rect.top + radius;
    const rawX = event.clientX - centerX;
    const rawY = event.clientY - centerY;
    const distance = Math.hypot(rawX, rawY);
    const limit = radius - 24;
    const clamp = distance > limit ? limit / distance : 1;
    const x = rawX * clamp;
    const y = rawY * clamp;
    const normalizedX = x / limit;
    const normalizedY = y / limit;

    setStick({ x, y });
    commandRef.current = {
      linear_x: Number((-normalizedY * linearSpeed).toFixed(3)),
      angular_z: Number((-normalizedX * angularSpeed).toFixed(3))
    };
  }

  function stopStick() {
    setIsActive(false);
    setStick({ x: 0, y: 0 });
    commandRef.current = { linear_x: 0, angular_z: 0 };
    if (!disabled) onCommandRef.current(0, 0);
  }

  useEffect(() => {
    if (!isActive || disabled) return undefined;

    const timer = window.setInterval(() => {
      onCommandRef.current(commandRef.current.linear_x, commandRef.current.angular_z);
    }, 120);

    return () => window.clearInterval(timer);
  }, [disabled, isActive]);

  useEffect(() => {
    return () => {
      onCommandRef.current(0, 0);
    };
  }, []);

  useEffect(() => {
    if (disabled) {
      setIsActive(false);
      setStick({ x: 0, y: 0 });
      commandRef.current = { linear_x: 0, angular_z: 0 };
    }
  }, [disabled]);

  return (
    <div className="mt-4">
      <div
        ref={padRef}
        role="application"
        aria-label="Joystick teleoperation"
        className={`relative h-44 w-44 touch-none rounded-full border border-console-line bg-console-panel ${
          disabled ? "cursor-not-allowed opacity-45" : "cursor-grab active:cursor-grabbing"
        }`}
        onPointerDown={(event) => {
          if (disabled) return;
          event.currentTarget.setPointerCapture(event.pointerId);
          setIsActive(true);
          updateStick(event);
        }}
        onPointerMove={(event) => {
          if (isActive) updateStick(event);
        }}
        onPointerUp={stopStick}
        onPointerCancel={stopStick}
        onLostPointerCapture={stopStick}
      >
        <div className="absolute left-1/2 top-3 h-[calc(100%-1.5rem)] w-px -translate-x-1/2 bg-slate-300" aria-hidden="true" />
        <div className="absolute left-3 top-1/2 h-px w-[calc(100%-1.5rem)] -translate-y-1/2 bg-slate-300" aria-hidden="true" />
        <div
          className="absolute left-1/2 top-1/2 flex h-12 w-12 items-center justify-center rounded-full border border-signal-info bg-white text-signal-info shadow-soft"
          style={{ transform: `translate(calc(-50% + ${stick.x}px), calc(-50% + ${stick.y}px))` }}
        >
          <Gamepad2 className="h-5 w-5" aria-hidden="true" />
        </div>
      </div>
      <div className="mt-2 grid grid-cols-2 gap-2 font-mono text-xs text-slate-500">
        <div>vx {commandRef.current.linear_x.toFixed(2)}</div>
        <div>wz {commandRef.current.angular_z.toFixed(2)}</div>
      </div>
    </div>
  );
}

function TeleopButton({ disabled, label, icon: Icon, onClick, tone = "default" }) {
  const className =
    tone === "stop"
      ? "bg-orange-100 text-orange-800 hover:bg-orange-200"
      : "bg-slate-100 text-console-ink hover:bg-slate-200";

  return (
    <button
      type="button"
      aria-label={label}
      title={label}
      disabled={disabled}
      onClick={onClick}
      className={`flex aspect-square items-center justify-center rounded-md border border-console-line transition disabled:cursor-not-allowed disabled:opacity-40 ${className}`}
    >
      <Icon className="h-5 w-5" aria-hidden="true" />
    </button>
  );
}

function SpeedControl({ label, value, min, max, step, unit, onChange }) {
  function update(nextValue) {
    const parsed = Number(nextValue);
    if (Number.isNaN(parsed)) return;
    onChange(Math.max(min, Math.min(max, parsed)));
  }

  return (
    <label className="block">
      <div className="mb-2 flex items-center justify-between gap-2">
        <span className="text-sm font-medium text-slate-600">{label}</span>
        <span className="font-mono text-xs text-slate-500">
          {value.toFixed(2)} {unit}
        </span>
      </div>
      <input
        type="range"
        min={min}
        max={max}
        step={step}
        value={value}
        onChange={(event) => update(event.target.value)}
        className="w-full"
      />
      <input
        type="number"
        min={min}
        max={max}
        step={step}
        value={value}
        onChange={(event) => update(event.target.value)}
        className="mt-2 h-9 w-full rounded-md border border-console-line px-2"
      />
    </label>
  );
}

export default MapsPage;
