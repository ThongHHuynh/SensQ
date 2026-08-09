import {
  Anchor,
  ArrowDown,
  ArrowLeft,
  ArrowRight,
  ArrowUp,
  Ban,
  Camera,
  CircleStop,
  MapPin,
  Navigation,
  RotateCw,
  Ruler,
  Save
} from "lucide-react";
import { useEffect, useMemo, useRef, useState } from "react";
import PageHeader from "../components/PageHeader.jsx";
import {
  cancelDocking,
  fetchDockingConfig,
  saveDockStation,
  sendTeleopCommand,
  startDocking
} from "../services/robotApi.js";

function DockingPage({ robot }) {
  const [mode, setMode] = useState("dock");
  const [catalog, setCatalog] = useState({ docks: [], serverConfig: {}, arbiterConfig: {} });
  const [message, setMessage] = useState("Loading docking stations…");

  async function loadCatalog() {
    try {
      const payload = await fetchDockingConfig();
      setCatalog(payload);
      setMessage("Docking configuration is ready.");
    } catch (error) {
      setMessage(error.message);
    }
  }

  useEffect(() => {
    loadCatalog();
  }, []);

  return (
    <>
      <PageHeader
        eyebrow="Docking"
        title={mode === "dock" ? "Dock the robot" : "Create a docking station"}
        description={
          mode === "dock"
            ? "Navigate to a saved staging pose, acquire its AprilTag, and complete the final approach."
            : "Drive slowly until the tag is stable, capture a staging pose, then save the station."
        }
        actions={
          <div className="inline-flex rounded-md border border-console-line bg-white p-1">
            <ModeButton active={mode === "dock"} label="Docking" onClick={() => setMode("dock")} />
            <ModeButton active={mode === "create"} label="Create station" onClick={() => setMode("create")} />
          </div>
        }
      />

      {mode === "dock" ? (
        <DockMode robot={robot} catalog={catalog} message={message} />
      ) : (
        <CreateStationMode
          robot={robot}
          catalog={catalog}
          message={message}
          onCatalogChange={(payload, nextMessage) => {
            setCatalog(payload);
            setMessage(nextMessage);
          }}
        />
      )}
    </>
  );
}

function DockMode({ robot, catalog, message: catalogMessage }) {
  const docking = robot.docking ?? {};
  const [selectedDockId, setSelectedDockId] = useState("");
  const [navigateToStaging, setNavigateToStaging] = useState(true);
  const [useOffsetOverride, setUseOffsetOverride] = useState(false);
  const [finalDistance, setFinalDistance] = useState(0.3);
  const [lateralOffset, setLateralOffset] = useState(0);
  const [yawOffset, setYawOffset] = useState(0);
  const [message, setMessage] = useState(catalogMessage);
  const [pending, setPending] = useState(false);
  const selectedDock = catalog.docks.find((dock) => dock.dockId === selectedDockId);
  const active = Boolean(docking.active);
  const minimumDistance = Number(catalog.serverConfig.minimum_tag_distance ?? 0);

  useEffect(() => {
    if (!catalog.docks.length) return;
    const requested = catalog.docks.find((dock) => dock.dockId === docking.request?.dock_id);
    applyDock(requested ?? catalog.docks[0]);
  }, [catalog.docks, docking.request?.dock_id]);

  function applyDock(dock) {
    setSelectedDockId(dock.dockId);
    setFinalDistance(Number(dock.final_distance));
    setLateralOffset(Number(dock.lateral_offset));
    setYawOffset(Number(dock.yaw_offset));
  }

  async function handleStart(event) {
    event.preventDefault();
    setPending(true);
    try {
      const response = await startDocking({
        dock_id: selectedDockId,
        navigate_to_staging_pose: navigateToStaging,
        use_offset_override: useOffsetOverride,
        final_distance: Number(finalDistance),
        lateral_offset: Number(lateralOffset),
        yaw_offset: Number(yawOffset)
      });
      setMessage(response.message);
    } catch (error) {
      setMessage(error.message);
    } finally {
      setPending(false);
    }
  }

  async function handleCancel() {
    setPending(true);
    try {
      const response = await cancelDocking();
      setMessage(response.message);
    } catch (error) {
      setMessage(error.message);
    } finally {
      setPending(false);
    }
  }

  return (
    <section className="grid gap-4 xl:grid-cols-[1.05fr_0.95fr]">
      <form onSubmit={handleStart} className="rounded-md border border-console-line bg-white p-5 shadow-soft">
        <div className="flex items-center justify-between gap-3">
          <div>
            <h2 className="text-lg font-semibold">Saved station</h2>
            <p className="mt-1 text-sm text-slate-600">{catalogMessage}</p>
          </div>
          <StateBadge state={docking.state ?? "IDLE"} active={active} />
        </div>

        <label className="mt-5 block">
          <span className="mb-2 block text-sm font-medium text-slate-700">Station</span>
          <select
            value={selectedDockId}
            onChange={(event) => {
              const dock = catalog.docks.find((item) => item.dockId === event.target.value);
              if (dock) applyDock(dock);
            }}
            disabled={active || pending}
            className="h-11 w-full rounded-md border border-console-line bg-white px-3 disabled:bg-slate-100"
          >
            {catalog.docks.map((dock) => (
              <option key={dock.dockId} value={dock.dockId}>
                {dock.dockId} · {dock.reverse_docking ? "reverse" : "forward"}
              </option>
            ))}
          </select>
        </label>

        {selectedDock ? <DockSummary dock={selectedDock} /> : null}

        <CheckField
          label="Navigate to staging pose"
          detail="Let Nav2 position the robot before AprilTag search."
          checked={navigateToStaging}
          onChange={setNavigateToStaging}
          disabled={active || pending}
        />
        <CheckField
          label="Override final offsets for this run"
          detail="The saved station remains unchanged."
          checked={useOffsetOverride}
          onChange={setUseOffsetOverride}
          disabled={active || pending}
        />

        <div className="mt-4 grid gap-3 sm:grid-cols-3">
          <NumberField label="Final distance" unit="m" value={finalDistance} min={minimumDistance} onChange={setFinalDistance} disabled={!useOffsetOverride || active || pending} />
          <NumberField label="Lateral offset" unit="m" value={lateralOffset} onChange={setLateralOffset} disabled={!useOffsetOverride || active || pending} />
          <NumberField label="Yaw offset" unit="rad" value={yawOffset} onChange={setYawOffset} disabled={!useOffsetOverride || active || pending} />
        </div>

        <p className="mt-4 min-h-5 text-sm text-slate-600">{docking.result?.message ?? message}</p>
        <div className="mt-4 flex flex-wrap gap-2">
          <button type="submit" disabled={!selectedDock || active || pending} className="primary-button">
            <Anchor className="h-4 w-4" /> Dock robot
          </button>
          <button type="button" onClick={handleCancel} disabled={!active || pending} className="danger-button">
            <Ban className="h-4 w-4" /> Cancel
          </button>
        </div>
      </form>

      <section className="rounded-md border border-console-line bg-white p-5 shadow-soft" aria-live="polite">
        <h2 className="text-lg font-semibold">Live approach</h2>
        <div className="mt-4 grid gap-3 sm:grid-cols-2">
          <FeedbackCard label="State" value={docking.state ?? "IDLE"} icon={Navigation} />
          <FeedbackCard label="Distance" value={formatMetric(docking.distanceRemaining, "m")} icon={Ruler} />
          <FeedbackCard label="Lateral error" value={formatMetric(docking.lateralError, "m")} icon={MapPin} />
          <FeedbackCard label="Yaw error" value={formatMetric(docking.yawError, "rad")} icon={RotateCw} />
          <FeedbackCard label="Retries" value={String(docking.retryCount ?? 0)} icon={RotateCw} />
          <FeedbackCard label="Result code" value={String(docking.result?.errorCode ?? "—")} icon={Anchor} />
        </div>
      </section>
    </section>
  );
}

function CreateStationMode({ robot, catalog, message: catalogMessage, onCatalogChange }) {
  const tags = useMemo(
    () => (Array.isArray(robot.tagDetections) ? robot.tagDetections : []),
    [robot.tagDetections]
  );
  const [selectedTagId, setSelectedTagId] = useState("");
  const [dockId, setDockId] = useState("");
  const [staging, setStaging] = useState(null);
  const [reverseDocking, setReverseDocking] = useState(false);
  const [finalDistance, setFinalDistance] = useState(0.3);
  const [linearSpeed, setLinearSpeed] = useState(0.08);
  const [angularSpeed, setAngularSpeed] = useState(0.25);
  const [message, setMessage] = useState("Drive slowly and keep the tag centered in view.");
  const [saving, setSaving] = useState(false);
  const selectedTag = tags.find((tag) => String(tag.tagId) === String(selectedTagId));
  const minimumDistance = Number(catalog.serverConfig.minimum_tag_distance ?? 0.1);

  useEffect(() => {
    if (!selectedTagId && tags[0]) setSelectedTagId(String(tags[0].tagId));
  }, [selectedTagId, tags]);

  function captureStaging() {
    if (robot.pose?.frame !== "map") {
      setMessage("Localize the robot first; staging must be captured in the map frame.");
      return;
    }
    setStaging({ x: Number(robot.pose.x), y: Number(robot.pose.y), yaw: Number(robot.pose.yaw) });
    setMessage("Staging pose captured. Keep it far enough away for Nav2 and tag acquisition.");
  }

  async function handleSave(event) {
    event.preventDefault();
    if (!selectedTag || selectedTag.mapX == null || selectedTag.mapY == null) {
      setMessage("A stable map-frame tag detection is required.");
      return;
    }
    if (!staging) {
      setMessage("Capture the staging pose before saving.");
      return;
    }
    setSaving(true);
    try {
      const response = await saveDockStation({
        dock_id: dockId,
        tag_id: Number(selectedTag.tagId),
        staging_x: Number(staging.x),
        staging_y: Number(staging.y),
        staging_yaw_degrees: Number(staging.yaw),
        reverse_docking: reverseDocking,
        final_distance: Number(finalDistance),
        lateral_offset: 0,
        yaw_offset: 0
      });
      onCatalogChange(response.catalog, response.message);
      setMessage(response.message);
    } catch (error) {
      setMessage(error.message);
    } finally {
      setSaving(false);
    }
  }

  return (
    <section className="grid gap-4 xl:grid-cols-[0.9fr_1.1fr]">
      <div className="space-y-4">
        <section className="rounded-md border border-console-line bg-white p-5 shadow-soft">
          <div className="flex items-center gap-2">
            <Camera className="h-5 w-5 text-signal-info" />
            <h2 className="text-lg font-semibold">1. Find the AprilTag</h2>
          </div>
          <p className="mt-1 text-sm text-slate-600">{tags.length ? "Live detections update while the robot is stationary or moving." : "No tag is currently visible."}</p>

          <label className="mt-4 block">
            <span className="mb-2 block text-sm font-medium text-slate-700">Detected tag</span>
            <select value={selectedTagId} onChange={(event) => setSelectedTagId(event.target.value)} className="h-10 w-full rounded-md border border-console-line bg-white px-3">
              <option value="">Choose a visible tag</option>
              {tags.map((tag) => <option key={tag.tagId} value={tag.tagId}>Tag {tag.tagId}</option>)}
            </select>
          </label>

          {selectedTag ? (
            <div className="mt-4 grid gap-2 sm:grid-cols-2">
              <FeedbackCard label="Distance" value={formatMetric(selectedTag.distance, "m")} icon={Ruler} />
              <FeedbackCard label="Bearing" value={formatMetric(selectedTag.bearingDegrees, "°")} icon={Navigation} />
              <FeedbackCard label="Tag orientation" value={formatMetric(selectedTag.orientationDegrees, "°")} icon={RotateCw} />
              <FeedbackCard label="Map coordinate" value={selectedTag.mapX == null ? "Localize first" : `${selectedTag.mapX.toFixed(3)}, ${selectedTag.mapY.toFixed(3)}`} icon={MapPin} />
            </div>
          ) : null}
        </section>

        <section className="rounded-md border border-console-line bg-white p-5 shadow-soft">
          <h2 className="text-lg font-semibold">Slow teleoperation</h2>
          <p className="mt-1 text-sm text-slate-600">Hold a direction button; releasing it sends an immediate stop.</p>
          <div className="mt-4 grid w-40 grid-cols-3 gap-2">
            <div />
            <MotionButton label="Forward" icon={ArrowUp} command={[linearSpeed, 0]} />
            <div />
            <MotionButton label="Left" icon={ArrowLeft} command={[0, angularSpeed]} />
            <MotionButton label="Stop" icon={CircleStop} command={[0, 0]} stop />
            <MotionButton label="Right" icon={ArrowRight} command={[0, -angularSpeed]} />
            <div />
            <MotionButton label="Reverse" icon={ArrowDown} command={[-linearSpeed, 0]} />
            <div />
          </div>
          <div className="mt-4 grid gap-3 sm:grid-cols-2">
            <NumberField label="Linear speed" unit="m/s" value={linearSpeed} min={0.03} max={0.15} onChange={setLinearSpeed} />
            <NumberField label="Turn speed" unit="rad/s" value={angularSpeed} min={0.1} max={0.5} onChange={setAngularSpeed} />
          </div>
        </section>
      </div>

      <form onSubmit={handleSave} className="rounded-md border border-console-line bg-white p-5 shadow-soft">
        <h2 className="text-lg font-semibold">2. Configure and save</h2>
        <p className="mt-1 text-sm text-slate-600">The tag coordinate comes from TF. You choose where Nav2 should stage and whether the final approach is forward or reverse.</p>

        <label className="mt-5 block">
          <span className="mb-2 block text-sm font-medium text-slate-700">Station name</span>
          <input value={dockId} onChange={(event) => setDockId(event.target.value)} placeholder="charging_dock" pattern="[A-Za-z0-9][A-Za-z0-9_-]*" required className="h-10 w-full rounded-md border border-console-line px-3" />
        </label>

        <div className="mt-4 rounded-md border border-console-line bg-console-panel p-4">
          <div className="flex items-start justify-between gap-3">
            <div>
              <div className="text-sm font-semibold">Staging pose</div>
              <div className="mt-1 font-mono text-xs text-slate-600">{staging ? `x ${staging.x.toFixed(3)}, y ${staging.y.toFixed(3)}, yaw ${staging.yaw.toFixed(1)}°` : "Not captured"}</div>
            </div>
            <button type="button" onClick={captureStaging} className="secondary-button"><MapPin className="h-4 w-4" /> Use current pose</button>
          </div>
          {staging ? (
            <div className="mt-3 grid grid-cols-3 gap-2">
              <StationPoseField label="X (m)" value={staging.x} onChange={(value) => setStaging({ ...staging, x: value })} />
              <StationPoseField label="Y (m)" value={staging.y} onChange={(value) => setStaging({ ...staging, y: value })} />
              <StationPoseField label="Yaw (°)" value={staging.yaw} min={-180} max={180} onChange={(value) => setStaging({ ...staging, yaw: value })} />
            </div>
          ) : null}
        </div>

        <div className="mt-4">
          <span className="block text-sm font-medium text-slate-700">Final approach direction</span>
          <div className="mt-2 grid grid-cols-2 gap-2">
            <DirectionButton active={!reverseDocking} label="Forward" detail="Camera watches the tag throughout" onClick={() => setReverseDocking(false)} />
            <DirectionButton active={reverseDocking} label="Reverse" detail="Capture tag, rotate, then back in" onClick={() => setReverseDocking(true)} />
          </div>
        </div>

        <div className="mt-4">
          <NumberField label="Final base-to-tag distance" unit="m" value={finalDistance} min={minimumDistance} max={2} onChange={setFinalDistance} />
          <p className="mt-1 text-xs text-slate-500">Minimum configured safe distance: {minimumDistance.toFixed(2)} m.</p>
        </div>

        <div className="mt-5 rounded-md border border-blue-200 bg-blue-50 p-3 text-sm text-blue-800">
          Confirm the tag remains visible and the staging pose faces it before saving. {catalogMessage}
        </div>
        <p className="mt-4 min-h-5 text-sm text-slate-600" aria-live="polite">{message}</p>
        <button type="submit" disabled={saving || !selectedTag || !staging} className="primary-button mt-3">
          <Save className="h-4 w-4" /> {saving ? "Saving…" : "Save station"}
        </button>
      </form>
    </section>
  );
}

function MotionButton({ label, icon: Icon, command, stop = false }) {
  const timerRef = useRef(null);
  const publish = () => sendTeleopCommand({ linear_x: command[0], angular_z: command[1] }).catch(() => {});
  const halt = () => {
    if (timerRef.current) window.clearInterval(timerRef.current);
    timerRef.current = null;
    sendTeleopCommand({ linear_x: 0, angular_z: 0 }).catch(() => {});
  };
  useEffect(() => halt, []);
  return (
    <button
      type="button"
      title={label}
      aria-label={label}
      onPointerDown={(event) => {
        event.currentTarget.setPointerCapture(event.pointerId);
        publish();
        if (!stop) timerRef.current = window.setInterval(publish, 120);
      }}
      onPointerUp={halt}
      onPointerCancel={halt}
      onLostPointerCapture={halt}
      className={`flex aspect-square items-center justify-center rounded-md border border-console-line ${stop ? "bg-orange-100 text-orange-800" : "bg-slate-100 text-console-ink"}`}
    >
      <Icon className="h-5 w-5" />
    </button>
  );
}

function ModeButton({ active, label, onClick }) {
  return <button type="button" onClick={onClick} className={`h-9 rounded px-3 text-sm font-semibold ${active ? "bg-console-rail text-white" : "text-slate-600"}`}>{label}</button>;
}

function DirectionButton({ active, label, detail, onClick }) {
  return (
    <button type="button" onClick={onClick} className={`rounded-md border p-3 text-left ${active ? "border-blue-500 bg-blue-50" : "border-console-line bg-white"}`}>
      <span className="block text-sm font-semibold">{label}</span>
      <span className="mt-1 block text-xs text-slate-500">{detail}</span>
    </button>
  );
}

function CheckField({ label, detail, checked, onChange, disabled }) {
  return (
    <label className="mt-3 flex items-start gap-3 rounded-md border border-console-line bg-console-panel p-3">
      <input type="checkbox" checked={checked} onChange={(event) => onChange(event.target.checked)} disabled={disabled} className="mt-0.5 h-4 w-4" />
      <span><span className="block text-sm font-semibold">{label}</span><span className="block text-xs text-slate-500">{detail}</span></span>
    </label>
  );
}

function DockSummary({ dock }) {
  return (
    <dl className="mt-4 grid gap-2 sm:grid-cols-2">
      <SummaryItem label="Mode" value={dock.reverse_docking ? "Reverse" : "Forward"} />
      <SummaryItem label="AprilTag" value={`${dock.tag_id} · ${dock.tag_frame}`} />
      <SummaryItem label="Staging" value={formatPose(dock.staging_pose)} />
      <SummaryItem label="Final distance" value={`${Number(dock.final_distance).toFixed(2)} m`} />
    </dl>
  );
}

function SummaryItem({ label, value }) {
  return <div className="rounded-md border border-console-line bg-console-panel p-3"><dt className="text-xs text-slate-500">{label}</dt><dd className="mt-1 text-sm font-semibold">{value}</dd></div>;
}

function NumberField({ label, unit, value, min, max, onChange, disabled }) {
  return (
    <label className="block">
      <span className="mb-2 block text-sm font-medium text-slate-700">{label} ({unit})</span>
      <input type="number" step="0.01" min={min} max={max} value={value} onChange={(event) => onChange(Number(event.target.value))} disabled={disabled} required className="h-10 w-full rounded-md border border-console-line px-3 disabled:bg-slate-100" />
    </label>
  );
}

function StationPoseField({ label, value, min, max, onChange }) {
  return (
    <label>
      <span className="mb-1 block text-xs font-medium text-slate-600">{label}</span>
      <input type="number" step="0.01" min={min} max={max} value={value} onChange={(event) => onChange(Number(event.target.value))} className="h-9 w-full rounded-md border border-console-line bg-white px-2" />
    </label>
  );
}

function FeedbackCard({ label, value, icon: Icon }) {
  return <div className="flex items-center gap-3 rounded-md border border-console-line bg-console-panel p-3"><Icon className="h-5 w-5 text-signal-info" /><div><div className="text-xs text-slate-500">{label}</div><div className="mt-1 font-mono text-sm font-semibold">{value}</div></div></div>;
}

function StateBadge({ state, active }) {
  const tone = active ? "border-blue-200 bg-blue-50 text-blue-700" : state === "SUCCEEDED" ? "border-emerald-200 bg-emerald-50 text-emerald-700" : "border-console-line bg-console-panel text-slate-600";
  return <span className={`rounded-md border px-2 py-1 text-xs font-semibold ${tone}`}>{state}</span>;
}

function formatPose(pose) {
  return Array.isArray(pose) ? pose.map((value) => Number(value).toFixed(3)).join(", ") : "—";
}

function formatMetric(value, unit) {
  return typeof value === "number" ? `${value.toFixed(unit === "°" ? 1 : 3)} ${unit}` : "—";
}

export default DockingPage;
