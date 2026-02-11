'use client';

import { ReactNode } from 'react';

/**
 * Props for the GCSLayout component.
 * Uses a slot pattern to compose the GCS UI from child components,
 * allowing flexible positioning without tight coupling.
 */
interface GCSLayoutProps {
  /** The main 3D map scene - fills the entire viewport */
  map: ReactNode;
  /** Status pill positioned at top center - shows mission state, connection, alerts */
  statusPill: ReactNode;
  /** Command dock at bottom - fleet, detections, and controls cards */
  commandDock: ReactNode;
  /** Optional layers panel - top-left, for toggling map visibility layers */
  layersPanel?: ReactNode;
  /** Optional picture-in-picture video feed - bottom-right overlay */
  pipFeed?: ReactNode;
  /** Optional alerts panel - top-right notification area */
  alerts?: ReactNode;
  /** Optional zone toolbar - top-center below status pill for zone drawing tools */
  zoneToolbar?: ReactNode;
}

/**
 * GCSLayout - Ground Control Station main layout component.
 *
 * Implements a layered z-index architecture:
 * - z-0: Base map layer (3D scene)
 * - z-10: Floating overlay UI (status, panels, alerts)
 * - z-20: Command dock at bottom
 *
 * Uses pointer-events-none on the overlay container to allow click-through
 * to the 3D map, with pointer-events-auto on individual UI elements.
 */
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
      {/* Base layer: 3D map scene */}
      <div className="absolute inset-0 z-0">
        {map}
      </div>

      {/* Overlay layer: floating UI elements */}
      <div className="pointer-events-none absolute inset-0 z-10">
        {/* Status pill - centered at top */}
        <div className="pointer-events-auto absolute left-1/2 top-4 -translate-x-1/2">
          {statusPill}
        </div>

        {/* Layers panel - top-left */}
        {layersPanel && (
          <div className="pointer-events-auto absolute left-4 top-20">
            {layersPanel}
          </div>
        )}

        {/* PiP video feed - bottom-right, above command dock */}
        {pipFeed && (
          <div className="pointer-events-auto absolute bottom-[160px] right-4">
            {pipFeed}
          </div>
        )}

        {/* Zone toolbar - centered below status pill */}
        {zoneToolbar && (
          <div className="pointer-events-auto absolute left-1/2 top-20 -translate-x-1/2">
            {zoneToolbar}
          </div>
        )}

        {/* Alerts panel - top-right */}
        {alerts && (
          <div className="pointer-events-auto absolute right-4 top-20">
            {alerts}
          </div>
        )}
      </div>

      {/* Command dock layer - fixed at bottom */}
      <div className="absolute inset-x-0 bottom-0 z-20">
        {commandDock}
      </div>
    </div>
  );
}
