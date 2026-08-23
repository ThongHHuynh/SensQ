import { Ban, CheckCircle2, Clock, MapPin, Play, Route } from "lucide-react";
import { useEffect, useMemo, useState } from "react";
import PageHeader from "../components/PageHeader.jsx";
import { cancelMission, fetchDockingConfig, getMissionHistory, startMission } from "../services/robotApi.js";

const PHASES = ["UNDOCKING", "NAVIGATING", "COVERING", "RETURNING", "DOCKING", "COMPLETE"];

function MissionPage({ robot }) {
  const mission = robot.mission ?? {};
  const [catalog, setCatalog] = useState({ docks: [] });
  const [missionType, setMissionType] = useState("coverage");
  const [dockId, setDockId] = useState("");
  const [autoDock, setAutoDock] = useState(true);
  const [pending, setPending] = useState(false);
  const [message, setMessage] = useState(null);
  const [history, setHistory] = useState([]);
  const [historyError, setHistoryError] = useState(null);
  const active = Boolean(mission.active);
  const phase = mission.phase ?? "IDLE";
  const phaseIndex = PHASES.indexOf(phase);

  useEffect(() => {
    fetchDockingConfig()
      .then((payload) => {
        setCatalog(payload);
        if (!dockId && payload.docks?.length) setDockId(payload.docks[0].dockId);
      })
      .catch((error) => setMessage(error.message));
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  async function loadHistory() {
    try {
      const payload = await getMissionHistory();
      setHistory(Array.isArray(payload.missions) ? payload.missions : []);
      setHistoryError(null);
    } catch (error) {
      setHistoryError(error.message);
    }
  }

  useEffect(() => {
    loadHistory();
  }, []);

  const previousActive = useMemo(() => active, [active]);
  useEffect(() => {
    if (previousActive && !active) loadHistory();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [active]);

  async function handleStart(event) {
    event.preventDefault();
    if (!dockId) return;
    setPending(true);
    try {
      const response = await startMission(missionType, dockId, autoDock);
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
      const response = await cancelMission();
      setMessage(response.message);
    } catch (error) {
      setMessage(error.message);
    } finally {
      setPending(false);
    }
  }

  return (
    <>
      <PageHeader
        eyebrow="Mission"
        title="Mission control"
        description="Run an unattended undock -> coverage clean -> return -> dock sequence and review past runs."
      />

      <section className="grid gap-4 xl:grid-cols-[0.95fr_1.05fr]">
        <form onSubmit={handleStart} className="rounded-md border border-console-line bg-white p-5 shadow-soft">
          <h2 className="text-lg font-semibold">Mission control panel</h2>

          <label className="mt-5 block">
            <span className="mb-2 block text-sm font-medium text-slate-700">Mission type</span>
            <select
              value={missionType}
              onChange={(event) => setMissionType(event.target.value)}
              disabled={active || pending}
              className="h-11 w-full rounded-md border border-console-line bg-white px-3 disabled:bg-slate-100"
            >
              <option value="coverage">Coverage Clean</option>
              <option value="patrol" disabled>
                Patrol (not yet implemented)
              </option>
            </select>
          </label>

          <label className="mt-4 block">
            <span className="mb-2 block text-sm font-medium text-slate-700">Dock</span>
            <select
              value={dockId}
              onChange={(event) => setDockId(event.target.value)}
              disabled={active || pending}
              className="h-11 w-full rounded-md border border-console-line bg-white px-3 disabled:bg-slate-100"
            >
              {catalog.docks?.length ? null : <option value="">Loading docks…</option>}
              {(catalog.docks ?? []).map((dock) => (
                <option key={dock.dockId} value={dock.dockId}>
                  {dock.dockId}
                </option>
              ))}
            </select>
          </label>

          <label className="mt-4 flex items-center gap-3 rounded-md border border-console-line bg-console-panel p-3">
            <input
              type="checkbox"
              className="h-4 w-4"
              checked={autoDock}
              onChange={(event) => setAutoDock(event.target.checked)}
              disabled={active || pending}
            />
            <span className="text-sm font-medium text-console-ink">Auto-dock on complete</span>
          </label>

          <p className="mt-4 min-h-5 text-sm text-slate-600">{mission.result?.message ?? message}</p>
          <div className="mt-4 flex flex-wrap gap-2">
            <button type="submit" disabled={!dockId || active || pending} className="primary-button">
              <Play className="h-4 w-4" /> Start mission
            </button>
            <button type="button" onClick={handleCancel} disabled={!active || pending} className="danger-button">
              <Ban className="h-4 w-4" /> Cancel mission
            </button>
          </div>
        </form>

        <section className="rounded-md border border-console-line bg-white p-5 shadow-soft" aria-live="polite">
          <h2 className="text-lg font-semibold">Live mission status</h2>

          <ol className="mt-5 flex flex-wrap items-center gap-2 text-xs font-semibold">
            {PHASES.map((step, index) => {
              const reached = phaseIndex >= 0 && index <= phaseIndex;
              return (
                <li key={step} className="flex items-center gap-2">
                  <span
                    className={`rounded-md border px-2 py-1 ${
                      reached ? "border-blue-500 bg-blue-50 text-blue-700" : "border-console-line bg-console-panel text-slate-500"
                    }`}
                  >
                    {step}
                  </span>
                  {index < PHASES.length - 1 ? <span className="text-slate-300">&rarr;</span> : null}
                </li>
              );
            })}
          </ol>

          <div className="mt-5">
            <div className="h-2 w-full overflow-hidden rounded-full bg-console-panel">
              <div
                className="h-full rounded-full bg-console-rail transition-all"
                style={{ width: `${Math.min(100, Math.max(0, Number(mission.progressPercent ?? 0)))}%` }}
              />
            </div>
            <div className="mt-2 flex items-center justify-between text-xs text-slate-500">
              <span>{Number(mission.progressPercent ?? 0).toFixed(0)}% complete</span>
              <span className="inline-flex items-center gap-1">
                <Clock className="h-3.5 w-3.5" /> {formatElapsed(mission.elapsedSeconds)}
              </span>
            </div>
          </div>

          <div className="mt-4 grid gap-3 sm:grid-cols-2">
            <FeedbackCard label="Phase" value={phase} icon={Route} />
            <FeedbackCard label="Zone" value={mission.currentZone ?? "—"} icon={MapPin} />
          </div>
          <p className="mt-4 text-sm text-slate-600">{mission.detail || "No mission running."}</p>
        </section>
      </section>

      <section className="mt-6 rounded-md border border-console-line bg-white p-5 shadow-soft">
        <div className="flex items-center justify-between">
          <h2 className="text-lg font-semibold">Mission history</h2>
          <button type="button" onClick={loadHistory} className="secondary-button">
            Refresh
          </button>
        </div>
        {historyError ? <p className="mt-3 text-sm text-red-600">{historyError}</p> : null}
        <div className="mt-4 overflow-x-auto">
          <table className="w-full min-w-[640px] border-collapse text-sm">
            <thead>
              <tr className="border-b border-console-line text-left text-xs uppercase tracking-normal text-slate-500">
                <th className="py-2 pr-3">Date</th>
                <th className="py-2 pr-3">Type</th>
                <th className="py-2 pr-3">Duration</th>
                <th className="py-2 pr-3">Area covered</th>
                <th className="py-2 pr-3">Status</th>
                <th className="py-2 pr-3">Message</th>
              </tr>
            </thead>
            <tbody>
              {history.length === 0 ? (
                <tr>
                  <td colSpan={6} className="py-4 text-center text-slate-500">
                    No missions have run yet.
                  </td>
                </tr>
              ) : (
                history.map((record) => (
                  <tr key={record.id} className="border-b border-console-line/60">
                    <td className="py-2 pr-3 font-mono text-xs">{formatDate(record.completedAt)}</td>
                    <td className="py-2 pr-3">{record.missionType}</td>
                    <td className="py-2 pr-3">{Number(record.durationSeconds ?? 0).toFixed(0)}s</td>
                    <td className="py-2 pr-3">{Number(record.areaCoveredM2 ?? 0).toFixed(1)} m²</td>
                    <td className="py-2 pr-3">
                      <span
                        className={`inline-flex items-center gap-1 rounded-md border px-2 py-1 text-xs font-semibold ${
                          record.success
                            ? "border-emerald-200 bg-emerald-50 text-emerald-700"
                            : "border-red-200 bg-red-50 text-red-700"
                        }`}
                      >
                        {record.success ? <CheckCircle2 className="h-3.5 w-3.5" /> : <Ban className="h-3.5 w-3.5" />}
                        {record.success ? "Success" : "Failed"}
                      </span>
                    </td>
                    <td className="py-2 pr-3 text-slate-600">{record.message}</td>
                  </tr>
                ))
              )}
            </tbody>
          </table>
        </div>
      </section>
    </>
  );
}

function FeedbackCard({ label, value, icon: Icon }) {
  return (
    <div className="flex items-center gap-3 rounded-md border border-console-line bg-console-panel p-3">
      <Icon className="h-5 w-5 text-signal-info" />
      <div>
        <div className="text-xs text-slate-500">{label}</div>
        <div className="mt-1 font-mono text-sm font-semibold">{value}</div>
      </div>
    </div>
  );
}

function formatElapsed(seconds) {
  const total = Math.max(0, Math.round(Number(seconds ?? 0)));
  const minutes = Math.floor(total / 60);
  const remainder = total % 60;
  return `${minutes}m ${remainder}s`;
}

function formatDate(value) {
  if (!value) return "—";
  const date = new Date(value.endsWith("Z") ? value : `${value}Z`);
  if (Number.isNaN(date.getTime())) return value;
  return date.toLocaleString();
}

export default MissionPage;
