'use client';

import { ReactNode } from 'react';

interface CommandDockProps {
  fleetCard: ReactNode;
  detectionsCard: ReactNode;
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
