function MetricCard({ label, value, detail, tone = "default", statusLabel }) {
  const tones = {
    default: "border-console-line bg-white",
    ok: "border-emerald-200 bg-emerald-50",
    warn: "border-amber-200 bg-amber-50",
    danger: "border-orange-200 bg-orange-50",
    info: "border-blue-200 bg-blue-50"
  };

  return (
    <div className={`rounded-md border p-4 shadow-soft ${tones[tone]}`}>
      <div className="flex items-center justify-between gap-2">
        <div className="text-sm font-medium text-slate-500">{label}</div>
        {statusLabel ? <StatusLight tone={tone} label={statusLabel} /> : null}
      </div>
      <div className="mt-2 text-2xl font-semibold tracking-normal text-console-ink">{value}</div>
      {detail ? <div className="mt-2 text-sm text-slate-600">{detail}</div> : null}
    </div>
  );
}

function StatusLight({ tone, label }) {
  const colors = {
    default: "bg-slate-300",
    ok: "bg-emerald-500",
    warn: "bg-amber-400",
    danger: "bg-orange-600",
    info: "bg-blue-500"
  };

  return (
    <span className="inline-flex items-center gap-1.5 rounded-md border border-white/70 bg-white/70 px-2 py-1 text-xs font-semibold text-slate-600">
      <span className={`h-2.5 w-2.5 rounded-full ${colors[tone] ?? colors.default}`} aria-hidden="true" />
      {label}
    </span>
  );
}

export default MetricCard;
