import { useState } from "react";
import { Battery, Compass, Cpu, Radio } from "lucide-react";
import MetricCard from "../components/MetricCard.jsx";
import PageHeader from "../components/PageHeader.jsx";
import StatusBadge from "../components/StatusBadge.jsx";
import { startMobileBase, stopRobot } from "../services/robotApi.js";

function HomePage({ robot, setRobot, source, error }) {
  const launchState = robot.connection.launchState ?? "unknown";
  const [actionError, setActionError] = useState(null);
  const temperature =
    typeof robot.hardwareStatus.temperature === "number" ? `${robot.hardwareStatus.temperature.toFixed(1)} C` : "Waiting";
  const batteryPercent = typeof robot.battery.percent === "number" ? `${robot.battery.percent}%` : "Waiting";
  const batteryDetail =
    typeof robot.battery.voltage === "number" ? `${robot.battery.voltage} V, ${robot.battery.state}` : robot.battery.state;
  const connectionTone = source === "backend" ? "ok" : "warn";
  const motorTone = robot.hardwareStatus.are_motors_ready ? "ok" : "warn";
  const batteryTone = typeof robot.battery.percent === "number" ? "ok" : "default";
  const navigationTone = robot.navigation.state === "Mapping" || robot.navigation.localization !== "Waiting" ? "ok" : "default";

  async function handleStart() {
    try {
      const snapshot = await startMobileBase();
      setRobot(snapshot);
      setActionError(null);
    } catch (err) {
      setActionError(err.message);
    }
  }

  async function handleStop() {
    try {
      const snapshot = await stopRobot();
      setRobot(snapshot);
      setActionError(null);
    } catch (err) {
      setActionError(err.message);
    }
  }

  return (
    <>
      <PageHeader
        eyebrow="Home"
        title="Robot overview"
        description="Operator-facing summary for connection state, hardware readiness, battery, and navigation context."
        actions={
          <>
            <button
              type="button"
              onClick={handleStart}
              className="inline-flex h-10 items-center rounded-md bg-console-rail px-3 text-sm font-semibold text-white"
            >
              Start Mobile Base
            </button>
            <button
              type="button"
              onClick={handleStop}
              className="inline-flex h-10 items-center rounded-md border border-console-line bg-white px-3 text-sm font-semibold text-console-ink"
            >
              Stop
            </button>
          </>
        }
      />

      <section className="grid gap-4 md:grid-cols-2 xl:grid-cols-4">
        <MetricCard
          label="Connection"
          value={source === "backend" ? "Backend" : "Mock"}
          detail={actionError ?? (error ? `Backend unavailable: ${error}` : `Launch ${launchState}`)}
          tone={connectionTone}
          statusLabel={source === "backend" ? "Online" : "Mock"}
        />
        <MetricCard
          label="Motors"
          value={robot.hardwareStatus.are_motors_ready ? "Ready" : "Not ready"}
          detail={robot.hardwareStatus.debug_message}
          tone={motorTone}
          statusLabel={robot.hardwareStatus.are_motors_ready ? "Ready" : "Waiting"}
        />
        <MetricCard
          label="Battery"
          value={batteryPercent}
          detail={batteryDetail}
          tone={batteryTone}
          statusLabel={typeof robot.battery.percent === "number" ? "OK" : "Unknown"}
        />
        <MetricCard
          label="Navigation"
          value={robot.navigation.state}
          detail={robot.navigation.activeMap}
          tone={navigationTone}
          statusLabel={navigationTone === "ok" ? "Active" : "Idle"}
        />
      </section>

      <section className="mt-6 grid gap-4 lg:grid-cols-[1.3fr_0.7fr]">
        <div className="rounded-md border border-console-line bg-white p-5 shadow-soft">
          <div className="mb-4 flex items-center justify-between">
            <h2 className="text-lg font-semibold tracking-normal">Live robot context</h2>
            <StatusBadge status={robot.connection.mode} />
          </div>
          <div className="grid gap-3 sm:grid-cols-2">
            <InfoRow icon={Radio} label="Heartbeat" value={robot.connection.lastHeartbeat} />
            <InfoRow icon={Cpu} label="Temperature" value={temperature} />
            <InfoRow icon={Compass} label="Pose frame" value={robot.pose.frame} />
            <InfoRow icon={Battery} label="Localization" value={robot.navigation.localization} />
          </div>
        </div>

        <div className="rounded-md border border-console-line bg-white p-5 shadow-soft">
          <h2 className="text-lg font-semibold tracking-normal">Future control hooks</h2>
          <div className="mt-4 space-y-3 text-sm text-slate-600">
            <p>Backend can expose REST endpoints for snapshots and commands.</p>
            <p>Realtime state can move to WebSocket or rosbridge when ROS integration is added.</p>
            <p>Page components already read through `services/robotApi.js` instead of ROS directly.</p>
          </div>
        </div>
      </section>
    </>
  );
}

function InfoRow({ icon: Icon, label, value }) {
  return (
    <div className="flex items-center gap-3 rounded-md border border-console-line bg-console-panel p-3">
      <Icon className="h-5 w-5 text-signal-info" aria-hidden="true" />
      <div>
        <div className="text-xs font-semibold uppercase tracking-normal text-slate-500">{label}</div>
        <div className="text-sm font-medium text-console-ink">{value}</div>
      </div>
    </div>
  );
}

export default HomePage;
