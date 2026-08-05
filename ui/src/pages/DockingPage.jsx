import { Anchor, Ban, MapPin, Navigation, RotateCw, Ruler } from "lucide-react";
import { useEffect, useState } from "react";
import PageHeader from "../components/PageHeader.jsx";
import { cancelDocking, fetchDockingConfig, startDocking } from "../services/robotApi.js";

function DockingPage({ robot }) {
  const docking = robot.docking ?? {};
  const [catalog, setCatalog] = useState({ docks: [], serverConfig: {}, arbiterConfig: {} });
  const [selectedDockId, setSelectedDockId] = useState("");
  const [navigateToStaging, setNavigateToStaging] = useState(true);
  const [useOffsetOverride, setUseOffsetOverride] = useState(false);
  const [finalDistance, setFinalDistance] = useState(0);
  const [lateralOffset, setLateralOffset] = useState(0);
  const [yawOffset, setYawOffset] = useState(0);
  const [message, setMessage] = useState("Choose a configured dock to begin.");
  const [requestPending, setRequestPending] = useState(false);
  const requestedDockId = docking.request?.dock_id;
  const requestedNavigateToStaging = docking.request?.navigate_to_staging_pose;
  const requestedUseOverride = docking.request?.use_offset_override;
  const requestedFinalDistance = docking.request?.final_distance;
  const requestedLateralOffset = docking.request?.lateral_offset;
  const requestedYawOffset = docking.request?.yaw_offset;

  useEffect(() => {
    let mounted = true;
    fetchDockingConfig()
      .then((data) => {
        if (!mounted) return;
        setCatalog(data);
        const selected = data.docks[0];
        if (selected) applyDock(selected);
      })
      .catch((error) => {
        if (mounted) setMessage(error.message);
      });
    return () => {
      mounted = false;
    };
  }, []);

  useEffect(() => {
    if (!requestedDockId) return;
    const dock = catalog.docks.find((item) => item.dockId === requestedDockId);
    if (!dock) return;

    setSelectedDockId(dock.dockId);
    setNavigateToStaging(requestedNavigateToStaging ?? true);
    setUseOffsetOverride(requestedUseOverride ?? false);
    setFinalDistance(Number(requestedFinalDistance ?? dock.final_distance));
    setLateralOffset(Number(requestedLateralOffset ?? dock.lateral_offset));
    setYawOffset(Number(requestedYawOffset ?? dock.yaw_offset));
  }, [
    catalog.docks,
    requestedDockId,
    requestedFinalDistance,
    requestedLateralOffset,
    requestedNavigateToStaging,
    requestedUseOverride,
    requestedYawOffset
  ]);

  const selectedDock = catalog.docks.find((dock) => dock.dockId === selectedDockId);
  const active = Boolean(docking.active);
  const minimumDistance = Number(catalog.serverConfig.minimum_tag_distance ?? 0);

  function applyDock(dock) {
    setSelectedDockId(dock.dockId);
    setFinalDistance(Number(dock.final_distance));
    setLateralOffset(Number(dock.lateral_offset));
    setYawOffset(Number(dock.yaw_offset));
  }

  function handleDockChange(event) {
    const dock = catalog.docks.find((item) => item.dockId === event.target.value);
    if (dock) applyDock(dock);
  }

  async function handleStart(event) {
    event.preventDefault();
    setRequestPending(true);
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
      setRequestPending(false);
    }
  }

  async function handleCancel() {
    setRequestPending(true);
    try {
      const response = await cancelDocking();
      setMessage(response.message);
    } catch (error) {
      setMessage(error.message);
    } finally {
      setRequestPending(false);
    }
  }

  const resultMessage = docking.result?.message;

  return (
    <>
      <PageHeader
        eyebrow="Docking"
        title="Docking control"
        description="Stage with Nav2, acquire the configured AprilTag, and monitor the final approach."
      />

      <section className="grid gap-4 xl:grid-cols-[1.05fr_0.95fr]">
        <form onSubmit={handleStart} className="rounded-md border border-console-line bg-white p-5 shadow-soft">
          <div className="flex items-center justify-between gap-3">
            <div>
              <h2 className="text-lg font-semibold">Dock command</h2>
              <p className="mt-1 text-sm text-slate-600">Goal fields map directly to the ROS docking action.</p>
            </div>
            <DockStateBadge state={docking.state ?? "IDLE"} active={active} />
          </div>

          <label className="mt-5 block">
            <span className="mb-2 block text-sm font-medium text-slate-700">Dock</span>
            <select
              value={selectedDockId}
              onChange={handleDockChange}
              disabled={active || requestPending}
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

          <label className="mt-4 flex items-start gap-3 rounded-md border border-console-line bg-console-panel p-3">
            <input
              type="checkbox"
              checked={navigateToStaging}
              onChange={(event) => setNavigateToStaging(event.target.checked)}
              disabled={active || requestPending}
              className="mt-0.5 h-4 w-4"
            />
            <span>
              <span className="block text-sm font-semibold">Navigate to staging pose</span>
              <span className="block text-xs text-slate-500">Let Nav2 position the robot before tag search.</span>
            </span>
          </label>

          <label className="mt-3 flex items-start gap-3 rounded-md border border-console-line bg-console-panel p-3">
            <input
              type="checkbox"
              checked={useOffsetOverride}
              onChange={(event) => setUseOffsetOverride(event.target.checked)}
              disabled={active || requestPending}
              className="mt-0.5 h-4 w-4"
            />
            <span>
              <span className="block text-sm font-semibold">Override final offsets</span>
              <span className="block text-xs text-slate-500">Apply these values to this docking goal only.</span>
            </span>
          </label>

          <div className="mt-4 grid gap-3 sm:grid-cols-3">
            <NumberField
              label="Final distance"
              unit="m"
              value={finalDistance}
              min={minimumDistance}
              onChange={setFinalDistance}
              disabled={!useOffsetOverride || active || requestPending}
            />
            <NumberField
              label="Lateral offset"
              unit="m"
              value={lateralOffset}
              onChange={setLateralOffset}
              disabled={!useOffsetOverride || active || requestPending}
            />
            <NumberField
              label="Yaw offset"
              unit="rad"
              value={yawOffset}
              onChange={setYawOffset}
              disabled={!useOffsetOverride || active || requestPending}
            />
          </div>

          <p className="mt-4 min-h-5 text-sm text-slate-600">{resultMessage ?? message}</p>
          <div className="mt-4 flex flex-wrap gap-2">
            <button
              type="submit"
              disabled={!selectedDock || active || requestPending}
              className="inline-flex h-10 items-center gap-2 rounded-md bg-console-rail px-4 text-sm font-semibold text-white disabled:cursor-not-allowed disabled:opacity-50"
            >
              <Anchor className="h-4 w-4" aria-hidden="true" />
              Dock Robot
            </button>
            <button
              type="button"
              onClick={handleCancel}
              disabled={!active || requestPending}
              className="inline-flex h-10 items-center gap-2 rounded-md border border-red-200 bg-red-50 px-4 text-sm font-semibold text-red-700 disabled:cursor-not-allowed disabled:opacity-50"
            >
              <Ban className="h-4 w-4" aria-hidden="true" />
              Cancel Docking
            </button>
          </div>
        </form>

        <div className="space-y-4">
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

          <details className="rounded-md border border-console-line bg-white p-5 shadow-soft">
            <summary className="cursor-pointer text-base font-semibold">All docking configuration</summary>
            <div className="mt-4 space-y-5">
              <ConfigSection title={`Selected dock · ${selectedDock?.dockId ?? "None"}`} values={selectedDock} omit="dockId" />
              <ConfigSection title="Docking server" values={catalog.serverConfig} />
              <ConfigSection title="Velocity arbiter" values={catalog.arbiterConfig} />
            </div>
          </details>
        </div>
      </section>
    </>
  );
}

function DockSummary({ dock }) {
  const values = [
    ["Mode", dock.reverse_docking ? "Reverse" : "Forward"],
    ["AprilTag", `${dock.tag_id} · ${dock.tag_frame}`],
    ["Staging", formatPose(dock.staging_pose)],
    ["Reference", formatPose(dock.reference_pose)]
  ];
  return (
    <dl className="mt-4 grid gap-2 sm:grid-cols-2">
      {values.map(([label, value]) => (
        <div key={label} className="rounded-md border border-console-line bg-console-panel p-3">
          <dt className="text-xs font-medium text-slate-500">{label}</dt>
          <dd className="mt-1 text-sm font-semibold text-console-ink">{value}</dd>
        </div>
      ))}
    </dl>
  );
}

function NumberField({ label, unit, value, min, onChange, disabled }) {
  return (
    <label className="block">
      <span className="mb-2 block text-sm font-medium text-slate-700">{label} ({unit})</span>
      <input
        type="number"
        step="0.01"
        min={min}
        value={value}
        onChange={(event) => onChange(event.target.value)}
        disabled={disabled}
        required
        className="h-10 w-full rounded-md border border-console-line px-3 disabled:bg-slate-100 disabled:text-slate-500"
      />
    </label>
  );
}

function FeedbackCard({ label, value, icon: Icon }) {
  return (
    <div className="flex items-center gap-3 rounded-md border border-console-line bg-console-panel p-3">
      <Icon className="h-5 w-5 text-signal-info" aria-hidden="true" />
      <div>
        <div className="text-xs font-medium text-slate-500">{label}</div>
        <div className="mt-1 font-mono text-sm font-semibold text-console-ink">{value}</div>
      </div>
    </div>
  );
}

function ConfigSection({ title, values, omit }) {
  const entries = Object.entries(values ?? {}).filter(([key]) => key !== omit);
  return (
    <section>
      <h3 className="text-sm font-semibold text-console-ink">{title}</h3>
      <dl className="mt-2 grid gap-2 sm:grid-cols-2">
        {entries.map(([key, value]) => (
          <div key={key} className="rounded-md bg-console-panel p-2">
            <dt className="font-mono text-[11px] text-slate-500">{key}</dt>
            <dd className="mt-1 break-all text-xs font-medium text-console-ink">{formatConfig(value)}</dd>
          </div>
        ))}
      </dl>
    </section>
  );
}

function DockStateBadge({ state, active }) {
  const success = state === "SUCCEEDED";
  const failure = ["ABORTED", "ERROR", "REJECTED"].includes(state);
  const tone = active
    ? "border-blue-200 bg-blue-50 text-blue-700"
    : success
      ? "border-emerald-200 bg-emerald-50 text-emerald-700"
      : failure
        ? "border-red-200 bg-red-50 text-red-700"
        : "border-console-line bg-console-panel text-slate-600";
  return <span className={`rounded-md border px-2 py-1 text-xs font-semibold ${tone}`}>{state}</span>;
}

function formatPose(pose) {
  return Array.isArray(pose) ? pose.map((value) => Number(value).toFixed(3)).join(", ") : "—";
}

function formatMetric(value, unit) {
  return typeof value === "number" ? `${value.toFixed(3)} ${unit}` : "—";
}

function formatConfig(value) {
  if (typeof value === "boolean") return value ? "true" : "false";
  if (value === "") return "(empty)";
  if (Array.isArray(value) || (value && typeof value === "object")) return JSON.stringify(value);
  return String(value ?? "—");
}

export default DockingPage;
