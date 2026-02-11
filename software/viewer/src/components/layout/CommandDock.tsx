'use client';

import { ReactNode } from 'react';

/**
 * Props for the CommandDock component.
 * Uses a slot pattern to compose three primary command cards.
 */
interface CommandDockProps {
  /** Fleet overview card - displays connected drones and their status */
  fleetCard: ReactNode;
  /** Detections card - shows sensor detections and confidence scores */
  detectionsCard: ReactNode;
  /** Controls card - mission control buttons and actions */
  controlsCard: ReactNode;
}

/**
 * CommandDock - Bottom-mounted control panel for the GCS.
 *
 * Provides a consistent 3-column layout for the primary command interfaces:
 * - Fleet management (left)
 * - Detection review (center)
 * - Mission controls (right)
 *
 * Each card has a fixed width of 280px with a minimum height of 140px
 * to maintain visual balance across different content states.
 */
export function CommandDock({
  fleetCard,
  detectionsCard,
  controlsCard,
}: CommandDockProps) {
  return (
    <div
      className="flex items-stretch justify-center gap-4 p-4 pb-6"
      style={{ minHeight: '170px' }}
    >
      <div className="w-[280px] min-h-[140px]">{fleetCard}</div>
      <div className="w-[280px] min-h-[140px]">{detectionsCard}</div>
      <div className="w-[280px] min-h-[140px]">{controlsCard}</div>
    </div>
  );
}
