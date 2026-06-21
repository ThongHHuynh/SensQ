import PageHeader from "../components/PageHeader.jsx";

function SettingsPage({ robot }) {
  const maps = Array.isArray(robot.maps) ? robot.maps : [];
  const connection = robot.connection ?? {};

  return (
    <>
      <PageHeader
        eyebrow="Settings"
        title="Integration settings"
        description="Configuration placeholders for backend host, ROS domain, map behavior, and operator preferences."
      />

      <section className="grid gap-4 lg:grid-cols-2">
        <SettingsPanel title="Backend connection">
          <Label text="Backend URL">
            <input className="h-10 w-full rounded-md border border-console-line px-3" defaultValue={connection.backendUrl ?? ""} />
          </Label>
          <Label text="ROS_DOMAIN_ID">
            <input className="h-10 w-full rounded-md border border-console-line px-3" defaultValue={connection.rosDomainId ?? "0"} />
          </Label>
          <Label text="Realtime transport">
            <select className="h-10 w-full rounded-md border border-console-line px-3" defaultValue="websocket">
              <option value="websocket">WebSocket bridge</option>
              <option value="polling">REST polling</option>
              <option value="rosbridge">rosbridge_suite</option>
            </select>
          </Label>
        </SettingsPanel>

        <SettingsPanel title="Operator defaults">
          <Label text="Default map">
            <select className="h-10 w-full rounded-md border border-console-line px-3" defaultValue="lab">
              {maps.map((map) => (
                <option key={map.id || map.name} value={map.id || map.name}>
                  {map.name || "Unnamed map"}
                </option>
              ))}
              {maps.length === 0 ? <option value="">Waiting for maps</option> : null}
            </select>
          </Label>
          <Label text="Command safety mode">
            <select className="h-10 w-full rounded-md border border-console-line px-3" defaultValue="confirm">
              <option value="confirm">Confirm before motion commands</option>
              <option value="direct">Direct operator commands</option>
            </select>
          </Label>
          <label className="flex items-center gap-3 rounded-md border border-console-line bg-console-panel p-3">
            <input type="checkbox" className="h-4 w-4" defaultChecked />
            <span className="text-sm font-medium text-console-ink">Show simulated data badge</span>
          </label>
        </SettingsPanel>
      </section>
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
