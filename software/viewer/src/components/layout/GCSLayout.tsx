'use client';

import { ReactNode } from 'react';

/**
 * GCSLayout - Main viewport layout for the Ground Control Station
 * 
 * Per spec Section 3.2 Layout Zones:
 * | Zone           | Position              | Size           | Purpose                              |
 * |----------------|-----------------------|----------------|--------------------------------------|
 * | Status Pill    | Top center            | ~60px height   | Mission phase, timer, connection     |
 * | Layers Panel   | Top-left floating     | ~160px width   | Toggle map overlays                  |
 * | Map Viewport   | Center (behind all)   | 100% - dock    | 3D scene with Three.js               |
 * | PiP Feed       | Bottom-right floating | ~200px         | Drone camera feed                    |
 * | Command Dock   | Bottom fixed          | ~140px height  | Mission/Fleet/Detections/Controls    |
 * 
 * Phase 1: Just the layout regions, no actual content yet
 */

interface GCSLayoutProps {
  /** The map viewport - renders behind everything (Phase 1: dark placeholder) */
  map: ReactNode;
  /** Top center status pill */
  statusPill: ReactNode;
  /** Bottom fixed command dock with card slots */
  commandDock: ReactNode;
  /** Floating layers panel - top left (Phase 5) */
  layersPanel?: ReactNode;
  /** Picture-in-picture video feed - bottom right (Phase 7) */
  pipFeed?: ReactNode;
  /** Alert stack overlay (Phase 8) */
  alerts?: ReactNode;
  /** Zone drawing toolbar (Phase 6) */
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
      {/* Zone: Map Viewport - Center (behind all), 100% - dock */}
      <div className="absolute inset-0 z-0">
        {map}
      </div>

      {/* Floating UI Layer */}
      <div className="pointer-events-none absolute inset-0 z-10">
        {/* Zone: Status Pill - Top center, ~60px height */}
        <div className="pointer-events-auto absolute left-1/2 top-4 -translate-x-1/2">
          {statusPill}
        </div>

        {/* Zone: Layers Panel - Top-left floating, ~160px width (Phase 5) */}
        {layersPanel && (
          <div className="pointer-events-auto absolute left-4 top-20">
            {layersPanel}
          </div>
        )}

        {/* Zone: PiP Feed - Bottom-right floating, ~200px (Phase 7) */}
        {pipFeed && (
          <div className="pointer-events-auto absolute bottom-[160px] right-4">
            {pipFeed}
          </div>
        )}

        {/* Zone Drawing Toolbar (Phase 6) */}
        {zoneToolbar && (
          <div className="pointer-events-auto absolute left-1/2 top-20 -translate-x-1/2">
            {zoneToolbar}
          </div>
        )}

        {/* Alert Stack (Phase 8) - positioned below status pill */}
        {alerts && (
          <div className="pointer-events-auto absolute right-4 top-20">
            {alerts}
          </div>
        )}
      </div>

      {/* Zone: Command Dock - Bottom fixed, ~140px height */}
      <div className="absolute inset-x-0 bottom-0 z-20">
        {commandDock}
      </div>
    </div>
  );
}
