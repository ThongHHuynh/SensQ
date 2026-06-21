function MetricCard({ label, value, detail, tone = "default" }) {
  const tones = {
    default: "border-console-line bg-white",
    ok: "border-emerald-200 bg-emerald-50",
    warn: "border-amber-200 bg-amber-50",
    danger: "border-orange-200 bg-orange-50",
    info: "border-blue-200 bg-blue-50"
  };

  return (
    <div className={`rounded-md border p-4 shadow-soft ${tones[tone]}`}>
      <div className="text-sm font-medium text-slate-500">{label}</div>
      <div className="mt-2 text-2xl font-semibold tracking-normal text-console-ink">{value}</div>
      {detail ? <div className="mt-2 text-sm text-slate-600">{detail}</div> : null}
    </div>
  );
}

export default MetricCard;
