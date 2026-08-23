import { useState } from "react";
import { Camera, Download, RefreshCw, Radio, ScanLine } from "lucide-react";
import PageHeader from "../components/PageHeader.jsx";
import StatusBadge from "../components/StatusBadge.jsx";
import { API_BASE_URL, fetchCameraSnapshot } from "../services/robotApi.js";

function VisualizationPage({ robot }) {
  const camera = robot.camera ?? {};
  const available = Boolean(camera.available);
  const cameraDevice = Array.isArray(robot.devices) ? robot.devices.find((device) => device.name === "Camera") : null;
  const cameraStatus = cameraDevice?.status ?? (available ? "online" : "offline");
  const resolution = camera.width && camera.height ? `${camera.width} x ${camera.height}` : "Unknown";
  const [streamKey, setStreamKey] = useState(0);
  const [snapshotPending, setSnapshotPending] = useState(false);
  const [message, setMessage] = useState(null);

  async function handleSnapshot() {
    setSnapshotPending(true);
    try {
      const blob = await fetchCameraSnapshot();
      const url = URL.createObjectURL(blob);
      const link = document.createElement("a");
      link.href = url;
      link.download = `camera-snapshot-${Date.now()}.jpg`;
      document.body.appendChild(link);
      link.click();
      link.remove();
      URL.revokeObjectURL(url);
      setMessage(null);
    } catch (error) {
      setMessage(error.message);
    } finally {
      setSnapshotPending(false);
    }
  }

  return (
    <>
      <PageHeader
        eyebrow="Visualization"
        title="Camera view"
        description="Live MJPEG preview of the robot's camera, streamed through the backend."
        actions={
          <>
            <button
              type="button"
              onClick={handleSnapshot}
              disabled={!available || snapshotPending}
              className="inline-flex h-10 items-center gap-2 rounded-md border border-console-line bg-white px-3 text-sm font-semibold text-console-ink disabled:opacity-50"
            >
              <Download className="h-4 w-4" aria-hidden="true" />
              {snapshotPending ? "Saving…" : "Snapshot"}
            </button>
            <button
              type="button"
              onClick={() => setStreamKey((key) => key + 1)}
              disabled={!available}
              className="inline-flex h-10 items-center gap-2 rounded-md border border-console-line bg-white px-3 text-sm font-semibold text-console-ink disabled:opacity-50"
            >
              <RefreshCw className="h-4 w-4" aria-hidden="true" />
              Reconnect
            </button>
          </>
        }
      />

      <section className="grid gap-4 xl:grid-cols-[1fr_280px]">
        <div className="overflow-hidden rounded-md border border-console-line bg-white shadow-soft">
          <div className="flex items-center justify-between border-b border-console-line px-4 py-3">
            <div className="flex items-center gap-2">
              <Camera className="h-5 w-5 text-signal-info" aria-hidden="true" />
              <h2 className="text-base font-semibold tracking-normal">Front camera</h2>
            </div>
            <StatusBadge status={cameraStatus} />
          </div>

          <div className="relative aspect-video bg-slate-950">
            {available ? (
              <img
                key={streamKey}
                src={`${API_BASE_URL}/api/camera/stream`}
                alt="Live camera stream"
                className="h-full w-full object-contain"
                onError={() => setMessage("Camera stream disconnected.")}
              />
            ) : (
              <div className="absolute inset-0 flex flex-col items-center justify-center gap-3 text-white/70">
                <Camera className="h-12 w-12" aria-hidden="true" />
                <span className="text-sm font-medium">No camera feed</span>
              </div>
            )}
            <div className="absolute left-4 top-4 rounded-md border border-white/15 bg-black/45 px-3 py-2 font-mono text-xs text-slate-200">
              {camera.topic ?? "/camera/image_raw"}
            </div>
            <div className="absolute bottom-4 left-4 right-4 flex items-center justify-between rounded-md border border-white/15 bg-black/45 px-3 py-2 text-xs text-slate-200">
              <span>{available ? "Live" : "Waiting for frames"}</span>
              <span className="font-mono">{resolution}</span>
            </div>
          </div>
          {message ? <p className="border-t border-console-line px-4 py-2 text-sm text-slate-600">{message}</p> : null}
        </div>

        <aside className="space-y-4">
          <div className="rounded-md border border-console-line bg-white p-4 shadow-soft">
            <div className="flex items-center gap-2">
              <Radio className="h-5 w-5 text-signal-info" aria-hidden="true" />
              <h2 className="text-base font-semibold tracking-normal">Stream</h2>
            </div>
            <div className="mt-4 space-y-3">
              <Field label="Topic" value={camera.topic ?? "/camera/image_raw"} />
              <Field label="Resolution" value={resolution} />
              <Field label="Frame rate" value="~15 fps (backend stream)" />
              <Field label="Status" value={cameraStatus} />
            </div>
          </div>

          <div className="rounded-md border border-console-line bg-white p-4 shadow-soft">
            <div className="flex items-center gap-2">
              <ScanLine className="h-5 w-5 text-signal-info" aria-hidden="true" />
              <h2 className="text-base font-semibold tracking-normal">Overlay</h2>
            </div>
            <div className="mt-3 grid gap-2">
              <Toggle label="Grid" checked />
              <Toggle label="Telemetry" checked />
              <Toggle label="Detections" />
            </div>
          </div>
        </aside>
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

function Toggle({ label, checked = false }) {
  return (
    <label className="flex h-10 items-center justify-between rounded-md border border-console-line bg-console-panel px-3 text-sm font-medium text-console-ink">
      <span>{label}</span>
      <input type="checkbox" className="h-4 w-4 accent-console-rail" defaultChecked={checked} />
    </label>
  );
}

export default VisualizationPage;
