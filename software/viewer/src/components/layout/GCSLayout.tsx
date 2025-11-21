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
      {/* Top Status Bar - Fixed Height */}
      <header className="flex-none h-[60px] z-50">
        {statusBar}
      </header>

      {/* Main Content Area */}
      <div className="flex-1 flex overflow-hidden">
        {/* Map Viewport - Takes remaining width */}
        <main className="flex-1 relative min-w-0">
            {map}
        </main>

        {/* Telemetry Panel - Fixed Width on Desktop, stacked on small screens if needed, 
            but reqs say "Tablet (<1280px): Stack vertically" for entire layout or just panels?
            Reqs: "Desktop: grid-cols-[minmax(0,1fr)_320px]", "Tablet (<1280px): Stack vertically"
        */}
        <aside className="flex-none w-[320px] hidden xl:block z-40">
            {telemetry}
        </aside>
      </div>
      
      {/* Mobile/Tablet Stack View (conditionally rendered or handled via CSS Grid instead of Flex) */}
      {/* Let's try a pure Grid approach as per specs */}
    </div>
  );
}

// Actually, let's rewrite GCSLayout to use CSS Grid exactly as specified in the prompt
// Layout Structure (CSS Grid)
// grid-rows-[60px_minmax(0,1fr)]
// Desktop: grid-cols-[minmax(0,1fr)_320px]

export function GCSGridLayout({ statusBar, map, telemetry }: GCSLayoutProps) {
    return (
        <div className="grid h-screen w-screen overflow-hidden bg-background text-foreground font-sans
            grid-rows-[60px_minmax(0,1fr)]
            grid-cols-1 xl:grid-cols-[minmax(0,1fr)_320px]
        ">
            {/* Status Bar - Spans full width */}
            <div className="col-span-1 xl:col-span-2 z-50">
                {statusBar}
            </div>

            {/* Map Viewport */}
            <div className="relative bg-zinc-900 overflow-hidden">
                {map}
            </div>

            {/* Telemetry Panel */}
            <div className="overflow-y-auto border-t xl:border-t-0 xl:border-l bg-background z-40">
                {telemetry}
            </div>
        </div>
    );
}
