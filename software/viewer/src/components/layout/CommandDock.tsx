'use client';

import { ReactNode } from 'react';

/**
 * CommandDock - Bottom fixed command bar container
 * 
 * Per spec Section 3.2:
 * - Position: Bottom fixed
 * - Size: ~140px height
 * - Purpose: Fleet/Detections/Controls cards
 * 
 * Note: MissionCard removed - time/progress now in StatusPill
 */

interface CommandDockProps {
  /** Slot for FleetCard */
  fleetCard: ReactNode;
  /** Slot for DetectionsCard */
  detectionsCard: ReactNode;
  /** Slot for ControlsCard */
  controlsCard: ReactNode;
}

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
