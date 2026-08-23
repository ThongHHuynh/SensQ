import { useEffect, useState } from "react";
import PageHeader from "../components/PageHeader.jsx";
import { fetchSettings, updateSettings } from "../services/robotApi.js";

const DEFAULT_SETTINGS = {
  backend_url: "http://localhost:8000",
  ros_domain_id: "0",
  default_map: "",
  command_safety_mode: "confirm",
  use_sim_time: false
};

function SettingsPage({ robot }) {
  const maps = Array.isArray(robot.maps) ? robot.maps : [];
  const [settings, setSettings] = useState(DEFAULT_SETTINGS);
  const [loading, setLoading] = useState(true);
  const [saving, setSaving] = useState(false);
  const [toast, setToast] = useState(null);

  useEffect(() => {
    let isMounted = true;
    fetchSettings()
      .then((payload) => {
        if (isMounted) setSettings({ ...DEFAULT_SETTINGS, ...payload });
      })
      .catch((error) => {
        if (isMounted) setToast({ tone: "error", message: error.message });
      })
      .finally(() => {
        if (isMounted) setLoading(false);
      });
    return () => {
      isMounted = false;
    };
  }, []);

  useEffect(() => {
    if (!toast) return undefined;
    const timer = window.setTimeout(() => setToast(null), 4000);
    return () => window.clearTimeout(timer);
  }, [toast]);

  function setField(key, value) {
    setSettings((current) => ({ ...current, [key]: value }));
  }

  async function handleSave(event) {
    event.preventDefault();
    setSaving(true);
    try {
      const payload = await updateSettings(settings);
      setSettings({ ...DEFAULT_SETTINGS, ...payload });
      setToast({ tone: "success", message: "Settings saved." });
    } catch (error) {
      setToast({ tone: "error", message: error.message });
    } finally {
      setSaving(false);
    }
  }

  async function handleReset() {
    setSaving(true);
    try {
      const payload = await updateSettings(DEFAULT_SETTINGS);
      setSettings({ ...DEFAULT_SETTINGS, ...payload });
      setToast({ tone: "success", message: "Settings reset to defaults." });
    } catch (error) {
      setToast({ tone: "error", message: error.message });
    } finally {
      setSaving(false);
    }
  }

  return (
    <>
      <PageHeader
        eyebrow="Settings"
        title="Integration settings"
        description="Configuration for backend host, ROS domain, map behavior, and operator preferences. Saved settings persist in the backend database."
      />

      <form onSubmit={handleSave} className="grid gap-4 lg:grid-cols-2">
        <SettingsPanel title="Backend connection">
          <Label text="Backend URL">
            <input
              className="h-10 w-full rounded-md border border-console-line px-3"
              value={settings.backend_url ?? ""}
              onChange={(event) => setField("backend_url", event.target.value)}
              disabled={loading}
            />
          </Label>
          <Label text="ROS_DOMAIN_ID">
            <input
              className="h-10 w-full rounded-md border border-console-line px-3"
              value={settings.ros_domain_id ?? "0"}
              onChange={(event) => setField("ros_domain_id", event.target.value)}
              disabled={loading}
            />
          </Label>
          <Label text="Realtime transport">
            <select className="h-10 w-full rounded-md border border-console-line px-3" defaultValue="websocket">
              <option value="websocket">WebSocket bridge</option>
              <option value="polling">REST polling</option>
              <option value="rosbridge">rosbridge_suite</option>
            </select>
          </Label>
          <label className="flex items-center gap-3 rounded-md border border-console-line bg-console-panel p-3">
            <input
              type="checkbox"
              className="h-4 w-4"
              checked={Boolean(settings.use_sim_time)}
              onChange={(event) => setField("use_sim_time", event.target.checked)}
              disabled={loading}
            />
            <span className="text-sm font-medium text-console-ink">Use simulation time (use_sim_time)</span>
          </label>
        </SettingsPanel>

        <SettingsPanel title="Operator defaults">
          <Label text="Default map">
            <select
              className="h-10 w-full rounded-md border border-console-line px-3"
              value={settings.default_map ?? ""}
              onChange={(event) => setField("default_map", event.target.value)}
              disabled={loading}
            >
              <option value="">No default map</option>
              {maps.map((map) => (
                <option key={map.id || map.name} value={map.id || map.name}>
                  {map.name || "Unnamed map"}
                </option>
              ))}
            </select>
          </Label>
          <Label text="Command safety mode">
            <select
              className="h-10 w-full rounded-md border border-console-line px-3"
              value={settings.command_safety_mode ?? "confirm"}
              onChange={(event) => setField("command_safety_mode", event.target.value)}
              disabled={loading}
            >
              <option value="confirm">Confirm before motion commands</option>
              <option value="direct">Direct operator commands</option>
            </select>
          </Label>
          <label className="flex items-center gap-3 rounded-md border border-console-line bg-console-panel p-3">
            <input type="checkbox" className="h-4 w-4" defaultChecked />
            <span className="text-sm font-medium text-console-ink">Show simulated data badge</span>
          </label>
        </SettingsPanel>

        <div className="flex flex-wrap gap-2 lg:col-span-2">
          <button type="submit" disabled={loading || saving} className="primary-button">
            {saving ? "Saving…" : "Save"}
          </button>
          <button type="button" onClick={handleReset} disabled={loading || saving} className="secondary-button">
            Reset to defaults
          </button>
        </div>
      </form>

      {toast ? (
        <div
          role="status"
          className={`fixed bottom-6 right-6 z-50 rounded-md border px-4 py-3 text-sm font-medium shadow-soft ${
            toast.tone === "success" ? "border-emerald-200 bg-emerald-50 text-emerald-800" : "border-red-200 bg-red-50 text-red-800"
          }`}
        >
          {toast.message}
        </div>
      ) : null}
    </>
  );
}

function SettingsPanel({ title, children }) {
  return (
    <div className="rounded-md border border-console-line bg-white p-5 shadow-soft">
      <h2 className="text-lg font-semibold tracking-normal">{title}</h2>
      <div className="mt-4 space-y-4">{children}</div>
    </div>
  );
}

function Label({ text, children }) {
  return (
    <label className="block">
      <span className="mb-2 block text-sm font-medium text-slate-600">{text}</span>
      {children}
    </label>
  );
}

export default SettingsPage;
