function AppLayout({ tabs, activeTab, onTabChange, children }) {
  return (
    <div className="min-h-screen bg-[#eef1f6] text-console-ink">
      <aside className="fixed inset-y-0 left-0 z-20 hidden w-64 border-r border-white/10 bg-console-rail px-4 py-5 text-white md:flex md:flex-col">
        <div className="mb-8 px-2">
          <div className="text-xl font-semibold tracking-normal">SensQ</div>
          <div className="mt-1 text-sm text-slate-300">Robot Console</div>
        </div>
        <nav className="space-y-1" aria-label="Primary">
          {tabs.map((tab) => {
            const Icon = tab.icon;
            const isActive = tab.id === activeTab;

            return (
              <button
                key={tab.id}
                type="button"
                onClick={() => onTabChange(tab.id)}
                className={`flex h-11 w-full items-center gap-3 rounded-md px-3 text-left text-sm font-medium transition ${
                  isActive ? "bg-white text-console-rail" : "text-slate-300 hover:bg-white/10 hover:text-white"
                }`}
              >
                <Icon className="h-5 w-5 shrink-0" aria-hidden="true" />
                <span>{tab.label}</span>
              </button>
            );
          })}
        </nav>
        <div className="mt-auto rounded-md border border-white/10 bg-white/5 p-3 text-sm text-slate-300">
          <div className="font-medium text-white">ROS bridge</div>
          <div className="mt-1">Mocked data source</div>
        </div>
      </aside>

      <div className="md:pl-64">
        <header className="sticky top-0 z-10 border-b border-console-line bg-white/90 px-4 py-3 backdrop-blur md:hidden">
          <div className="mb-3 flex items-center justify-between">
            <div>
              <div className="text-lg font-semibold">SensQ</div>
              <div className="text-xs text-slate-500">Robot Console</div>
            </div>
          </div>
          <nav className="grid grid-cols-5 gap-1" aria-label="Primary">
            {tabs.map((tab) => {
              const Icon = tab.icon;
              const isActive = tab.id === activeTab;

              return (
                <button
                  key={tab.id}
                  type="button"
                  onClick={() => onTabChange(tab.id)}
                  className={`flex h-12 flex-col items-center justify-center rounded-md px-1 text-[10px] font-medium transition ${
                    isActive ? "bg-console-rail text-white" : "bg-console-panel text-slate-600"
                  }`}
                >
                  <Icon className="mb-1 h-4 w-4" aria-hidden="true" />
                  <span className="max-w-full truncate">{tab.label}</span>
                </button>
              );
            })}
          </nav>
        </header>

        <main className="mx-auto min-h-screen max-w-7xl px-4 py-6 sm:px-6 lg:px-8">{children}</main>
      </div>
    </div>
  );
}

export default AppLayout;
