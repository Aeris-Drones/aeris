'use client';

import { ReactNode } from 'react';

interface GCSLayoutProps {
  map: ReactNode;
  statusPill: ReactNode;
  commandDock: ReactNode;
  layersPanel?: ReactNode;
  pipFeed?: ReactNode;
  alerts?: ReactNode;
  zoneToolbar?: ReactNode;
}

export function GCSLayout({
  map,
  statusPill,
  commandDock,
  layersPanel,
  pipFeed,
  alerts,
  zoneToolbar,
}: GCSLayoutProps) {
  return (
    <div className="relative h-dvh w-full overflow-hidden bg-[var(--surface-0)]">
      <div className="absolute inset-0 z-0">
        {map}
      </div>

      <div className="pointer-events-none absolute inset-0 z-10">
        <div className="pointer-events-auto absolute left-1/2 top-4 -translate-x-1/2">
          {statusPill}
        </div>

        {layersPanel && (
          <div className="pointer-events-auto absolute left-4 top-20">
            {layersPanel}
          </div>
        )}

        {pipFeed && (
          <div className="pointer-events-auto absolute bottom-[160px] right-4">
            {pipFeed}
          </div>
        )}

        {zoneToolbar && (
          <div className="pointer-events-auto absolute left-1/2 top-20 -translate-x-1/2">
            {zoneToolbar}
          </div>
        )}

        {alerts && (
          <div className="pointer-events-auto absolute right-4 top-20">
            {alerts}
          </div>
        )}
      </div>

      <div className="absolute inset-x-0 bottom-0 z-20">
        {commandDock}
      </div>
    </div>
  );
}
