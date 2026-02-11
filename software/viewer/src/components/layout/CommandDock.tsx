'use client';

import { ReactNode } from 'react';

/** Props for the CommandDock component using the slot pattern for composition */
interface CommandDockProps {
  /** Fleet overview card - displays connected drones and their status */
  fleetCard: ReactNode;
  /** Detections card - shows sensor detections and confidence scores */
  detectionsCard: ReactNode;
  /** Controls card - mission control buttons and actions */
  controlsCard: ReactNode;
}

/**
 * Bottom-mounted control panel for the GCS.
 *
 * Provides a consistent 3-column layout for the primary command interfaces:
 * fleet management (left), detection review (center), and mission controls (right).
 * Fixed dimensions ensure visual balance across different content states.
 *
 * @example
 * ```tsx
 * <CommandDock
 *   fleetCard={<FleetCard />}
 *   detectionsCard={<DetectionsCard />}
 *   controlsCard={<ControlsCard />}
 * />
 * ```
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
