import {
  Ban,
  Crosshair,
  Disc3,
  LocateFixed,
  MapPin,
  Minus,
  Navigation,
  Pencil,
  Play,
  Plus,
  Route,
  Save,
  Send,
  Sparkles,
  Square
} from "lucide-react";
import { useEffect, useRef, useState } from "react";
import PageHeader from "../components/PageHeader.jsx";
import {
  cancelCoverage,
  cancelNavigationGoal,
  executeCoverage,
  generateCoverage,
  prepareCoverage,
  renameMap,
  resetMapping,
  saveMap,
  selectMap,
  sendNavigationGoal,
  setInitialPose,
  startMapping,
  stopMapping
} from "../services/robotApi.js";

const DEG_TO_RAD = Math.PI / 180;

function MapsPage({ robot }) {
  const maps = Array.isArray(robot.maps) ? robot.maps : [];
  const pose = robot.pose ?? {};
  const navigation = robot.navigation ?? {};
  const liveMap = robot.liveMap ?? {};
  const coverage = robot.coverage ?? {};
  const [tool, setTool] = useState("pan");
  const [initialSelection, setInitialSelection] = useState(null);
  const [goalSelection, setGoalSelection] = useState(null);
  const [initialApplied, setInitialApplied] = useState(false);
  const [message, setMessage] = useState("Choose a map, then set the robot's initial pose.");
  const [busy, setBusy] = useState(false);
  const [mapName, setMapName] = useState("Cleaning map");
  const [editingMapId, setEditingMapId] = useState(null);
  const [editingMapName, setEditingMapName] = useState("");
  const [mapViewReset, setMapViewReset] = useState(0);
  const [useSimTime, setUseSimTime] = useState(false);

  function handleMapPoint(point) {
    if (tool === "initial") {
      setInitialSelection({ x: point.x, y: point.y, yaw_degrees: initialSelection?.yaw_degrees ?? 0 });
      setInitialApplied(false);
      setMessage("Initial position selected. Set its direction, then apply it to AMCL.");
    } else if (tool === "goal") {
      setGoalSelection({ x: point.x, y: point.y, yaw_degrees: goalSelection?.yaw_degrees ?? 0 });
      setMessage("Navigation goal selected. Set its direction, then send it to Nav2.");
    }
  }

  async function run(action) {
    setBusy(true);
    try {
      const result = await action();
      setMessage(result.message || "Command completed");
      return result;
    } catch (error) {
      setMessage(error.message);
      return null;
    } finally {
      setBusy(false);
    }
  }

  async function handleSelectMap(map) {
    const result = await run(() => selectMap(map.id));
    if (result) {
      setInitialApplied(false);
      setInitialSelection(null);
      setGoalSelection(null);
    }
  }

  async function handleInitialPose() {
    if (!initialSelection) return;
    const result = await run(() => setInitialPose(initialSelection));
    if (result) setInitialApplied(true);
  }

  async function handleGenerateCoverage() {
    if (!initialApplied) {
      setMessage("Apply the selected initial pose and wait for localization before generating.");
      return;
    }
    setBusy(true);
    try {
      const prepared = await prepareCoverage(useSimTime);
      setMessage(prepared.message);
      const result = await generateCoverage();
      setMessage(result.message || "Coverage path generated");
    } catch (error) {
      setMessage(error.message);
    } finally {
      setBusy(false);
    }
  }

  async function handleSaveMap() {
    await run(() => saveMap(mapName));
  }

  async function handleRename(map) {
    const result = await run(() => renameMap(map.id, editingMapName));
    if (result) setEditingMapId(null);
  }

  return (
    <>
      <PageHeader
        eyebrow="Cleaning"
        title="Map, localize, and clean"
        description="Select the operating map, place the robot, preview a complete coverage route, then start cleaning."
        actions={
          <button type="button" onClick={() => setMapViewReset((value) => value + 1)} className="secondary-button">
            <LocateFixed className="h-4 w-4" /> Center map
          </button>
        }
      />

      <div className="mb-4 grid gap-2 rounded-md border border-console-line bg-white p-3 sm:grid-cols-4">
        <WorkflowStep number="1" label="Choose map" done={Boolean(navigation.activeMap)} />
        <WorkflowStep number="2" label="Set initial pose" done={initialApplied} />
        <WorkflowStep number="3" label="Generate route" done={Number(coverage.path?.totalPoses) > 1} />
        <WorkflowStep number="4" label="Execute cleaning" active={["ENTERING", "FOLLOWING"].includes(coverage.state)} />
      </div>

      <section className="grid gap-4 xl:grid-cols-[minmax(0,1fr)_320px]">
        <div className="space-y-4">
          <div className="relative h-[65vh] min-h-[480px] overflow-hidden rounded-md border border-console-line bg-[#d8dde7]">
            <InteractiveMap
              liveMap={liveMap}
              pose={pose}
              initialPose={initialSelection}
              goalPose={goalSelection}
              coveragePath={coverage.path}
              tool={tool}
              onPoint={handleMapPoint}
              resetSignal={mapViewReset}
            />
            <div className="absolute left-4 top-4 z-10 flex flex-wrap gap-2 rounded-md border border-console-line bg-white p-2 shadow-soft">
              <ToolButton active={tool === "pan"} icon={Navigation} label="Pan" onClick={() => setTool("pan")} />
              <ToolButton active={tool === "initial"} icon={Crosshair} label="Initial pose" onClick={() => setTool("initial")} />
              <ToolButton active={tool === "goal"} icon={MapPin} label="Nav goal" onClick={() => setTool("goal")} />
            </div>
            <div className="absolute bottom-4 left-4 z-10 max-w-[calc(100%-2rem)] rounded-md border border-console-line bg-white px-3 py-2 text-sm shadow-soft">
              <span className="font-semibold">{navigation.activeMap ?? "No map selected"}</span>
              <span className="ml-2 text-slate-500">{message}</span>
            </div>
          </div>

          <div className="grid gap-4 lg:grid-cols-2">
            <LocalizationPanel
              selection={initialSelection}
              setSelection={setInitialSelection}
              applied={initialApplied}
              localization={navigation.localization}
              busy={busy}
              onChoose={() => setTool("initial")}
              onApply={handleInitialPose}
            />
            <NavigationPanel
              selection={goalSelection}
              setSelection={setGoalSelection}
              goal={navigation.goal ?? {}}
              busy={busy}
              onChoose={() => setTool("goal")}
              onSend={() => goalSelection && run(() => sendNavigationGoal(goalSelection))}
              onCancel={() => run(cancelNavigationGoal)}
            />
          </div>

          <CoveragePanel
            coverage={coverage}
            initialApplied={initialApplied}
            busy={busy}
            useSimTime={useSimTime}
            setUseSimTime={setUseSimTime}
            onGenerate={handleGenerateCoverage}
            onExecute={() => run(executeCoverage)}
            onCancel={() => run(cancelCoverage)}
          />
        </div>

        <aside className="space-y-4">
          <section className="rounded-md border border-console-line bg-white p-4 shadow-soft">
            <h2 className="text-base font-semibold">Operating map</h2>
            <p className="mt-1 text-xs text-slate-500">Selecting a saved map loads it into Nav2&apos;s map server.</p>
            <div className="mt-3 max-h-[36vh] space-y-2 overflow-y-auto">
              {maps.map((map) => {
                const active = map.name === navigation.activeMap;
                const saved = String(map.id).startsWith("saved-");
                return (
                  <div key={map.id} className={`rounded-md border p-2 ${active ? "border-emerald-400 bg-emerald-50" : "border-console-line bg-console-panel"}`}>
                    {editingMapId === map.id ? (
                      <div className="flex gap-2">
                        <input value={editingMapName} onChange={(event) => setEditingMapName(event.target.value)} className="h-8 min-w-0 flex-1 rounded border border-console-line px-2 text-xs" />
                        <button type="button" onClick={() => handleRename(map)} className="rounded bg-console-rail px-2 text-xs font-semibold text-white">Save</button>
                      </div>
                    ) : (
                      <div className="flex items-start gap-2">
                        <button type="button" onClick={() => handleSelectMap(map)} disabled={busy} className="min-w-0 flex-1 text-left">
                          <span className="block truncate text-sm font-semibold">{map.name}</span>
                          <span className="mt-1 block text-xs text-slate-500">{map.resolution || "Resolution unknown"}</span>
                        </button>
                        {saved ? <button type="button" onClick={() => { setEditingMapId(map.id); setEditingMapName(map.name); }} className="icon-button" aria-label={`Rename ${map.name}`}><Pencil className="h-3.5 w-3.5" /></button> : null}
                      </div>
                    )}
                  </div>
                );
              })}
            </div>
          </section>

          <section className="rounded-md border border-console-line bg-white p-4 shadow-soft">
            <div className="flex items-center gap-2"><Disc3 className="h-5 w-5 text-signal-info" /><h2 className="text-base font-semibold">Create a map</h2></div>
            <p className="mt-1 text-xs text-slate-500">Use SLAM while mapping, then save the result.</p>
            <div className="mt-3 flex flex-wrap gap-2">
              <button type="button" onClick={() => run(startMapping)} disabled={busy} className="secondary-button"><Play className="h-4 w-4" /> Start</button>
              <button type="button" onClick={() => run(stopMapping)} disabled={busy} className="secondary-button"><Square className="h-4 w-4" /> Stop</button>
              <button type="button" onClick={() => run(resetMapping)} disabled={busy} className="danger-button"><Ban className="h-4 w-4" /> Reset</button>
            </div>
            <label className="mt-3 block">
              <span className="mb-1 block text-xs font-medium text-slate-600">Map name</span>
              <input value={mapName} onChange={(event) => setMapName(event.target.value)} className="h-9 w-full rounded-md border border-console-line px-2" />
            </label>
            <button type="button" onClick={handleSaveMap} disabled={busy} className="primary-button mt-2 w-full"><Save className="h-4 w-4" /> Save map</button>
          </section>

          <section className="rounded-md border border-console-line bg-white p-4 text-sm shadow-soft">
            <h2 className="font-semibold">Live robot pose</h2>
            <div className="mt-2 font-mono text-xs text-slate-600">{pose.frame ?? "unknown"}: x {Number(pose.x ?? 0).toFixed(3)}, y {Number(pose.y ?? 0).toFixed(3)}, yaw {Number(pose.yaw ?? 0).toFixed(1)}°</div>
          </section>
        </aside>
      </section>
    </>
  );
}

function InteractiveMap({ liveMap, pose, initialPose, goalPose, coveragePath, tool, onPoint, resetSignal }) {
  const canvasRef = useRef(null);
  const dragRef = useRef(null);
  const [zoom, setZoom] = useState(1);
  const [offset, setOffset] = useState({ x: 0, y: 0 });
  const hasMap = Array.isArray(liveMap.data) && liveMap.data.length > 0 && liveMap.width > 0 && liveMap.height > 0;

  useEffect(() => {
    setZoom(1);
    setOffset({ x: 0, y: 0 });
  }, [resetSignal]);

  useEffect(() => {
    if (!hasMap || !canvasRef.current) return;
    const canvas = canvasRef.current;
    const context = canvas.getContext("2d");
    const width = Number(liveMap.width);
    const height = Number(liveMap.height);
    const scale = Math.max(1, Math.floor(1200 / Math.max(width, height)));
    canvas.width = width * scale;
    canvas.height = height * scale;
    context.fillStyle = "#cfd5df";
    context.fillRect(0, 0, canvas.width, canvas.height);

    for (let y = 0; y < height; y += 1) {
      for (let x = 0; x < width; x += 1) {
        const value = liveMap.data[y * width + x];
        context.fillStyle = value < 0 ? "#cfd5df" : value >= 50 ? "#111827" : "#ffffff";
        context.fillRect(x * scale, (height - y - 1) * scale, scale, scale);
      }
    }

    const pathPoints = coveragePath?.frame === (liveMap.frame || "map") ? coveragePath.points ?? [] : [];
    if (pathPoints.length > 1) {
      context.strokeStyle = "#10b981";
      context.lineWidth = Math.max(2, scale * 0.5);
      context.beginPath();
      pathPoints.forEach(([x, y], index) => {
        const point = worldToCanvas(liveMap, x, y, scale);
        if (index === 0) context.moveTo(point.x, point.y);
        else context.lineTo(point.x, point.y);
      });
      context.stroke();
    }

    drawPose(context, liveMap, pose, scale, "#2563eb", "R");
    drawPose(context, liveMap, initialPose, scale, "#f59e0b", "I");
    drawPose(context, liveMap, goalPose, scale, "#7c3aed", "G");
  }, [coveragePath, goalPose, hasMap, initialPose, liveMap, pose]);

  function beginPointer(event) {
    if (tool !== "pan") return;
    event.currentTarget.setPointerCapture(event.pointerId);
    dragRef.current = { pointerId: event.pointerId, x: event.clientX, y: event.clientY, offsetX: offset.x, offsetY: offset.y };
  }

  function movePointer(event) {
    const drag = dragRef.current;
    if (!drag || drag.pointerId !== event.pointerId) return;
    setOffset({ x: drag.offsetX + event.clientX - drag.x, y: drag.offsetY + event.clientY - drag.y });
  }

  function endPointer(event) {
    if (dragRef.current?.pointerId === event.pointerId) dragRef.current = null;
  }

  function selectPoint(event) {
    if (tool === "pan" || !hasMap) return;
    const canvas = canvasRef.current;
    const rect = canvas.getBoundingClientRect();
    const canvasX = (event.clientX - rect.left) * canvas.width / rect.width;
    const canvasY = (event.clientY - rect.top) * canvas.height / rect.height;
    onPoint(canvasToWorld(liveMap, canvasX, canvasY, canvas.width / Number(liveMap.width)));
  }

  if (!hasMap) {
    return <div className="map-grid absolute inset-0 flex items-center justify-center"><div className="rounded-md border border-console-line bg-white px-4 py-3 text-sm text-slate-600">Waiting for a map from ROS</div></div>;
  }

  return (
    <div className={`absolute inset-0 flex touch-none items-center justify-center overflow-hidden ${tool === "pan" ? "cursor-grab active:cursor-grabbing" : "cursor-crosshair"}`}>
      <canvas
        ref={canvasRef}
        onPointerDown={beginPointer}
        onPointerMove={movePointer}
        onPointerUp={endPointer}
        onPointerCancel={endPointer}
        onClick={selectPoint}
        onWheel={(event) => {
          event.preventDefault();
          setZoom((value) => Math.min(5, Math.max(0.5, value + (event.deltaY < 0 ? 0.2 : -0.2))));
        }}
        className="max-h-full max-w-full [image-rendering:pixelated]"
        style={{ transform: `translate(${offset.x}px, ${offset.y}px) scale(${zoom})`, transformOrigin: "center" }}
      />
      <div className="absolute right-4 top-4 z-10 flex items-center rounded-md border border-console-line bg-white shadow-soft">
        <button type="button" className="icon-button border-0" onClick={() => setZoom((value) => Math.max(0.5, value - 0.25))}><Minus className="h-4 w-4" /></button>
        <span className="min-w-14 text-center font-mono text-xs">{Math.round(zoom * 100)}%</span>
        <button type="button" className="icon-button border-0" onClick={() => setZoom((value) => Math.min(5, value + 0.25))}><Plus className="h-4 w-4" /></button>
      </div>
    </div>
  );
}

function worldToCanvas(map, worldX, worldY, scale) {
  const resolution = Number(map.resolution);
  const origin = map.origin ?? {};
  const yaw = Number(origin.yaw ?? 0) * DEG_TO_RAD;
  const dx = Number(worldX) - Number(origin.x ?? 0);
  const dy = Number(worldY) - Number(origin.y ?? 0);
  const gridX = (Math.cos(yaw) * dx + Math.sin(yaw) * dy) / resolution;
  const gridY = (-Math.sin(yaw) * dx + Math.cos(yaw) * dy) / resolution;
  return { x: gridX * scale, y: (Number(map.height) - gridY - 1) * scale };
}

function canvasToWorld(map, canvasX, canvasY, scale) {
  const resolution = Number(map.resolution);
  const origin = map.origin ?? {};
  const yaw = Number(origin.yaw ?? 0) * DEG_TO_RAD;
  const localX = canvasX / scale * resolution;
  const localY = (Number(map.height) - canvasY / scale - 1) * resolution;
  return {
    x: Number((Number(origin.x ?? 0) + Math.cos(yaw) * localX - Math.sin(yaw) * localY).toFixed(3)),
    y: Number((Number(origin.y ?? 0) + Math.sin(yaw) * localX + Math.cos(yaw) * localY).toFixed(3))
  };
}

function drawPose(context, map, pose, scale, color, label) {
  if (!pose || map.frame !== (pose.frame ?? "map") && pose.frame) return;
  const point = worldToCanvas(map, pose.x, pose.y, scale);
  const yaw = Number(pose.yaw_degrees ?? pose.yaw ?? 0) * DEG_TO_RAD;
  context.save();
  context.translate(point.x, point.y);
  context.rotate(-yaw);
  context.fillStyle = color;
  context.strokeStyle = "#ffffff";
  context.lineWidth = Math.max(1.5, scale * 0.35);
  context.beginPath();
  context.moveTo(Math.max(10, scale * 2.5), 0);
  context.lineTo(-Math.max(7, scale * 1.5), -Math.max(6, scale));
  context.lineTo(-Math.max(4, scale), 0);
  context.lineTo(-Math.max(7, scale * 1.5), Math.max(6, scale));
  context.closePath();
  context.stroke();
  context.fill();
  context.rotate(yaw);
  context.fillStyle = "#ffffff";
  context.font = `bold ${Math.max(9, scale * 1.5)}px sans-serif`;
  context.textAlign = "center";
  context.textBaseline = "middle";
  context.fillText(label, 0, 0);
  context.restore();
}

function LocalizationPanel({ selection, setSelection, applied, localization, busy, onChoose, onApply }) {
  return (
    <section className="rounded-md border border-console-line bg-white p-4 shadow-soft">
      <div className="flex items-center gap-2"><Crosshair className="h-5 w-5 text-amber-500" /><h2 className="text-base font-semibold">Robot initial pose</h2></div>
      <p className="mt-1 text-sm text-slate-600">Click the map where the robot is physically located. This publishes `/initialpose` to AMCL.</p>
      <PoseEditor pose={selection} setPose={setSelection} />
      <div className="mt-3 flex flex-wrap gap-2">
        <button type="button" onClick={onChoose} className="secondary-button"><Crosshair className="h-4 w-4" /> Pick on map</button>
        <button type="button" onClick={onApply} disabled={!selection || busy} className="primary-button"><Send className="h-4 w-4" /> Apply pose</button>
      </div>
      <p className={`mt-3 text-xs ${applied ? "text-emerald-700" : "text-slate-500"}`}>{applied ? `Applied · ${localization}` : "Not applied"}</p>
    </section>
  );
}

function NavigationPanel({ selection, setSelection, goal, busy, onChoose, onSend, onCancel }) {
  return (
    <section className="rounded-md border border-console-line bg-white p-4 shadow-soft">
      <div className="flex items-center gap-2"><MapPin className="h-5 w-5 text-violet-600" /><h2 className="text-base font-semibold">Point navigation</h2></div>
      <p className="mt-1 text-sm text-slate-600">Choose a destination and let Nav2 plan around obstacles.</p>
      <PoseEditor pose={selection} setPose={setSelection} />
      <div className="mt-3 flex flex-wrap gap-2">
        <button type="button" onClick={onChoose} className="secondary-button"><MapPin className="h-4 w-4" /> Pick goal</button>
        <button type="button" onClick={onSend} disabled={!selection || busy || goal.active} className="primary-button"><Navigation className="h-4 w-4" /> Navigate</button>
        <button type="button" onClick={onCancel} disabled={!goal.active || busy} className="danger-button"><Ban className="h-4 w-4" /> Cancel</button>
      </div>
      <p className="mt-3 text-xs text-slate-500">{goal.state ?? "IDLE"}{typeof goal.distanceRemaining === "number" ? ` · ${goal.distanceRemaining.toFixed(2)} m remaining` : ""}</p>
    </section>
  );
}

function CoveragePanel({ coverage, initialApplied, busy, useSimTime, setUseSimTime, onGenerate, onExecute, onCancel }) {
  const metrics = coverage.metrics ?? {};
  const hasPath = Number(coverage.path?.totalPoses) > 1;
  const active = ["ENTERING", "FOLLOWING", "CANCELING"].includes(coverage.state);
  return (
    <section className="rounded-md border border-console-line bg-white p-5 shadow-soft">
      <div className="flex flex-col gap-3 sm:flex-row sm:items-start sm:justify-between">
        <div>
          <div className="flex items-center gap-2"><Sparkles className="h-5 w-5 text-emerald-600" /><h2 className="text-lg font-semibold">Coverage cleaning</h2><StatePill state={coverage.state ?? "IDLE"} /></div>
          <p className="mt-1 text-sm text-slate-600">Generate a Boustrophedon route over reachable free space. Review the green route before execution.</p>
        </div>
        <label className="flex items-center gap-2 text-xs text-slate-600"><input type="checkbox" checked={useSimTime} onChange={(event) => setUseSimTime(event.target.checked)} /> Simulation clock</label>
      </div>
      <div className="mt-4 grid gap-2 sm:grid-cols-2 lg:grid-cols-4">
        <Metric label="Path poses" value={coverage.path?.totalPoses ?? 0} />
        <Metric label="Length" value={metrics.path_length_m == null ? "—" : `${metrics.path_length_m} m`} />
        <Metric label="Coverage" value={metrics.estimated_coverage_percent == null ? "—" : `${metrics.estimated_coverage_percent}%`} />
        <Metric label="Cells" value={metrics.cells ?? "—"} />
      </div>
      <p className="mt-3 text-sm text-slate-600" aria-live="polite">{coverage.detail ?? "Coverage planner idle"}</p>
      <div className="mt-4 flex flex-wrap gap-2">
        <button type="button" onClick={onGenerate} disabled={!initialApplied || busy || active} className="secondary-button"><Route className="h-4 w-4" /> Generate route</button>
        <button type="button" onClick={onExecute} disabled={!hasPath || busy || active} className="primary-button"><Play className="h-4 w-4" /> Start cleaning</button>
        <button type="button" onClick={onCancel} disabled={!active || busy} className="danger-button"><Ban className="h-4 w-4" /> Stop cleaning</button>
      </div>
    </section>
  );
}

function PoseEditor({ pose, setPose }) {
  if (!pose) return <div className="mt-3 rounded-md border border-dashed border-console-line p-3 text-sm text-slate-500">No point selected</div>;
  return (
    <div className="mt-3 grid grid-cols-3 gap-2">
      <PoseField label="X" value={pose.x} onChange={(value) => setPose({ ...pose, x: value })} />
      <PoseField label="Y" value={pose.y} onChange={(value) => setPose({ ...pose, y: value })} />
      <PoseField label="Yaw °" value={pose.yaw_degrees} onChange={(value) => setPose({ ...pose, yaw_degrees: Math.max(-180, Math.min(180, value)) })} />
    </div>
  );
}

function PoseField({ label, value, onChange }) {
  return <label><span className="mb-1 block text-xs font-medium text-slate-600">{label}</span><input type="number" step="0.1" value={value} onChange={(event) => onChange(Number(event.target.value))} className="h-9 w-full rounded-md border border-console-line px-2" /></label>;
}

function WorkflowStep({ number, label, done, active }) {
  const tone = done ? "border-emerald-300 bg-emerald-50 text-emerald-800" : active ? "border-blue-300 bg-blue-50 text-blue-800" : "border-console-line bg-console-panel text-slate-600";
  return <div className={`flex items-center gap-2 rounded-md border px-3 py-2 text-sm font-semibold ${tone}`}><span className="flex h-6 w-6 items-center justify-center rounded-full bg-white text-xs">{done ? "✓" : number}</span>{label}</div>;
}

function ToolButton({ active, icon: Icon, label, onClick }) {
  return <button type="button" onClick={onClick} className={`inline-flex h-9 items-center gap-2 rounded px-3 text-xs font-semibold ${active ? "bg-console-rail text-white" : "bg-console-panel text-slate-600"}`}><Icon className="h-4 w-4" />{label}</button>;
}

function Metric({ label, value }) {
  return <div className="rounded-md border border-console-line bg-console-panel p-3"><div className="text-xs text-slate-500">{label}</div><div className="mt-1 font-mono text-sm font-semibold">{value}</div></div>;
}

function StatePill({ state }) {
  const tone = ["ENTERING", "FOLLOWING"].includes(state) ? "bg-blue-100 text-blue-700" : state === "SUCCEEDED" || state === "READY" ? "bg-emerald-100 text-emerald-700" : state === "ERROR" ? "bg-red-100 text-red-700" : "bg-slate-100 text-slate-600";
  return <span className={`rounded px-2 py-1 text-xs font-semibold ${tone}`}>{state}</span>;
}

export default MapsPage;
