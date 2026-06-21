import PageHeader from "../components/PageHeader.jsx";
import StatusBadge from "../components/StatusBadge.jsx";

function DeviceStatusPage({ robot }) {
  const devices = Array.isArray(robot.devices) ? robot.devices : [];
  const temperature =
    typeof robot.hardwareStatus.temperature === "number" ? `${robot.hardwareStatus.temperature.toFixed(1)} C` : "Waiting";
  const motorsReady =
    typeof robot.hardwareStatus.are_motors_ready === "boolean" ? String(robot.hardwareStatus.are_motors_ready) : "Waiting";
  const debugMessage = robot.hardwareStatus.debug_message || "Waiting for ROS hardware status";

  return (
    <>
      <PageHeader
        eyebrow="Device Status"
        title="Hardware and ROS topics"
        description="Status rows update from FastAPI WebSocket data once the ROS launch and hardware interface are running."
      />

      <section className="rounded-md border border-console-line bg-white shadow-soft">
        <div className="grid grid-cols-[1fr_auto] gap-4 border-b border-console-line px-4 py-3 text-sm font-semibold text-slate-500 md:grid-cols-[1fr_1fr_auto]">
          <span>Device</span>
          <span className="hidden md:block">Topic or interface</span>
          <span>Status</span>
        </div>
        <div className="divide-y divide-console-line">
          {devices.map((device) => (
            <div key={device.name || device.topic} className="grid grid-cols-[1fr_auto] gap-4 px-4 py-4 md:grid-cols-[1fr_1fr_auto]">
              <div>
                <div className="font-medium text-console-ink">{device.name || "Unknown device"}</div>
                <div className="mt-1 text-sm text-slate-500">{device.detail || "No status detail"}</div>
              </div>
              <div className="hidden self-center font-mono text-sm text-slate-600 md:block">{device.topic || "n/a"}</div>
              <div className="self-center">
                <StatusBadge status={device.status || "offline"} />
              </div>
            </div>
          ))}
          {devices.length === 0 ? <div className="px-4 py-6 text-sm text-slate-500">Waiting for device status.</div> : null}
        </div>
      </section>

      <section className="mt-6 rounded-md border border-console-line bg-white p-5 shadow-soft">
        <h2 className="text-lg font-semibold tracking-normal">HardwareStatus message</h2>
        <div className="mt-4 grid gap-3 md:grid-cols-3">
          <Field label="temperature" value={temperature} />
          <Field label="are_motors_ready" value={motorsReady} />
          <Field label="debug_message" value={debugMessage} />
        </div>
      </section>
    </>
  );
}

function Field({ label, value }) {
  return (
    <div className="rounded-md border border-console-line bg-console-panel p-3">
      <div className="font-mono text-xs text-slate-500">{label}</div>
      <div className="mt-2 text-sm font-medium text-console-ink">{value}</div>
    </div>
  );
}

export default DeviceStatusPage;
