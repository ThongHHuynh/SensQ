import { Camera, Maximize2, Radio, ScanLine } from "lucide-react";
import PageHeader from "../components/PageHeader.jsx";
import StatusBadge from "../components/StatusBadge.jsx";

function VisualizationPage({ robot }) {
  const cameraDevice = Array.isArray(robot.devices) ? robot.devices.find((device) => device.name === "Camera") : null;
  const cameraStatus = cameraDevice?.status ?? "offline";

  return (
    <>
      <PageHeader
        eyebrow="Visualization"
        title="Camera view"
        description="Camera topic preview surface for the operator console."
        actions={
          <button
            type="button"
            className="inline-flex h-10 items-center gap-2 rounded-md border border-console-line bg-white px-3 text-sm font-semibold text-console-ink"
          >
            <Maximize2 className="h-4 w-4" aria-hidden="true" />
            Fullscreen
          </button>
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
            <div className="absolute inset-0 bg-[linear-gradient(90deg,rgba(255,255,255,0.06)_1px,transparent_1px),linear-gradient(0deg,rgba(255,255,255,0.05)_1px,transparent_1px)] bg-[size:48px_48px]" />
            <div className="absolute inset-0 bg-[radial-gradient(circle_at_50%_42%,rgba(59,130,246,0.22),transparent_32%)]" />
            <div className="absolute left-4 top-4 rounded-md border border-white/15 bg-black/45 px-3 py-2 font-mono text-xs text-slate-200">
              /camera/image_raw
            </div>
            <div className="absolute bottom-4 left-4 right-4 flex items-center justify-between rounded-md border border-white/15 bg-black/45 px-3 py-2 text-xs text-slate-200">
              <span>Dummy frame</span>
              <span className="font-mono">1280 x 720</span>
            </div>
            <div className="absolute inset-0 flex items-center justify-center">
              <div className="flex h-28 w-28 items-center justify-center rounded-full border border-white/20 bg-white/10">
                <Camera className="h-12 w-12 text-white/80" aria-hidden="true" />
              </div>
            </div>
          </div>
        </div>

        <aside className="space-y-4">
          <div className="rounded-md border border-console-line bg-white p-4 shadow-soft">
            <div className="flex items-center gap-2">
              <Radio className="h-5 w-5 text-signal-info" aria-hidden="true" />
              <h2 className="text-base font-semibold tracking-normal">Stream</h2>
            </div>
            <div className="mt-4 space-y-3">
              <Field label="Topic" value="/camera/image_raw" />
              <Field label="Encoding" value="rgb8" />
              <Field label="Frame" value="camera_link" />
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
