function StatusBadge({ status }) {
  const labels = {
    online: "Online",
    ready: "Ready",
    warning: "Warning",
    offline: "Offline",
    simulated: "Simulated",
    backend: "Backend"
  };

  const styles = {
    online: "bg-emerald-100 text-emerald-800",
    ready: "bg-emerald-100 text-emerald-800",
    warning: "bg-amber-100 text-amber-800",
    offline: "bg-slate-200 text-slate-700",
    simulated: "bg-blue-100 text-blue-800",
    backend: "bg-blue-100 text-blue-800"
  };

  return (
    <span className={`inline-flex items-center rounded-md px-2 py-1 text-xs font-semibold ${styles[status] ?? styles.offline}`}>
      {labels[status] ?? status}
    </span>
  );
}

export default StatusBadge;
