'use client';

import React from 'react';
import { useMissionState } from '../../hooks/useMissionState';
import type { MissionPhase } from '../../types/mission';
import { Badge } from '@/components/ui/badge';
import { cn } from '@/lib/utils';
import { Activity } from 'lucide-react';

const phaseColors: Record<MissionPhase, string> = {
  IDLE: "bg-zinc-500 hover:bg-zinc-600",
  PLANNING: "bg-violet-500 hover:bg-violet-600",
  SEARCHING: "bg-blue-500 hover:bg-blue-600",
  TRACKING: "bg-orange-500 hover:bg-orange-600",
  COMPLETE: "bg-emerald-500 hover:bg-emerald-600",
  ABORTED: "bg-red-600 hover:bg-red-700",
};

export function MissionStateDisplay() {
  const missionState = useMissionState();

  return (
    <div className="flex items-center space-x-2">
       <Activity className="w-4 h-4 text-zinc-400" />
       <Badge className={cn("px-3 py-1 text-xs font-bold uppercase tracking-wider text-white border-0 transition-colors duration-300", phaseColors[missionState])}>
        {missionState}
      </Badge>
    </div>
  );
}
