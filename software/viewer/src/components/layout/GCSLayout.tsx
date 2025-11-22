'use client';

import React from 'react';

interface GCSLayoutProps {
  statusBar: React.ReactNode;
  map: React.ReactNode;
  telemetry: React.ReactNode;
}

export function GCSLayout({ statusBar, map, telemetry }: GCSLayoutProps) {
  return (
    <div className="flex flex-col h-screen w-screen overflow-hidden bg-background text-foreground">
      <header className="flex-none h-[60px] z-50">
        {statusBar}
      </header>

      <div className="flex-1 flex overflow-hidden">
        <main className="flex-1 relative min-w-0">
            {map}
        </main>

        <aside className="flex-none w-[320px] hidden xl:block z-40">
            {telemetry}
        </aside>
      </div>
    </div>
  );
}

export function GCSGridLayout({ statusBar, map, telemetry }: GCSLayoutProps) {
    return (
        <div className="grid h-screen w-screen overflow-hidden bg-background text-foreground font-sans
            grid-rows-[60px_minmax(0,1fr)]
            grid-cols-1 xl:grid-cols-[minmax(0,1fr)_320px]
        ">
            <div className="col-span-1 xl:col-span-2 z-50">
                {statusBar}
            </div>

            <div className="relative bg-zinc-900 overflow-hidden">
                {map}
            </div>

            <div className="overflow-y-auto border-t xl:border-t-0 xl:border-l bg-background z-40">
                {telemetry}
            </div>
        </div>
    );
}
