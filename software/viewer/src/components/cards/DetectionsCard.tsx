'use client';

import { Card } from '@/components/ui/card';
import { Badge } from '@/components/ui/badge';
import { Flame, AudioLines, Wind, ChevronRight } from 'lucide-react';
import { cn } from '@/lib/utils';

export interface Detection {
  id: string;
  sensorType: 'thermal' | 'acoustic' | 'gas';
  confidence: number;
  timestamp: number;
  status: 'new' | 'reviewing' | 'confirmed' | 'dismissed';
}

export interface DetectionsCardProps {
  thermalCount: number;
  acousticCount: number;
  gasCount: number;
  pendingCount: number;
  confirmedCount: number;
  latestDetection?: Detection;
}

const sensorConfig = {
  thermal: {
    Icon: Flame,
    color: 'text-orange-500',
    bg: 'bg-orange-500/20',
    label: 'Thermal'
  },
  acoustic: {
    Icon: AudioLines,
    color: 'text-blue-500',
    bg: 'bg-blue-500/20',
    label: 'Acoustic'
  },
  gas: {
    Icon: Wind,
    color: 'text-yellow-500',
    bg: 'bg-yellow-500/20',
    label: 'Gas'
  },
};

export function DetectionsCard({
  thermalCount,
  acousticCount,
  gasCount,
  pendingCount,
  confirmedCount,
}: DetectionsCardProps) {
  const totalCount = thermalCount + acousticCount + gasCount;
  const hasPending = pendingCount > 0;

  return (
    <Card
      className={cn(
        'flex h-full cursor-pointer flex-col justify-between p-4 transition-all',
        'hover:bg-[var(--surface-3)]',
        'active:scale-[0.98]',
        hasPending && 'border-[var(--success)]/30'
      )}
    >
      <div className="flex items-center justify-between">
        <span className="text-xs font-medium uppercase tracking-wider text-white/50">
          Detections
        </span>
        {hasPending && (
          <Badge variant="success" className="px-2 py-0.5">
            <span className="text-xs">{pendingCount} pending</span>
          </Badge>
        )}
      </div>

      <div className="flex items-center gap-2">
        <div className={cn('flex items-center gap-1.5 rounded-lg px-2.5 py-0.5', sensorConfig.thermal.bg)}>
          <Flame className={cn('h-4 w-4', sensorConfig.thermal.color)} />
          <span className={cn('font-mono text-sm font-bold', sensorConfig.thermal.color)}>
            {thermalCount}
          </span>
        </div>

        <div className={cn('flex items-center gap-1.5 rounded-lg px-2.5 py-0.5', sensorConfig.acoustic.bg)}>
          <AudioLines className={cn('h-4 w-4', sensorConfig.acoustic.color)} />
          <span className={cn('font-mono text-sm font-bold', sensorConfig.acoustic.color)}>
            {acousticCount}
          </span>
        </div>

        <div className={cn('flex items-center gap-1.5 rounded-lg px-2.5 py-0.5', sensorConfig.gas.bg)}>
          <Wind className={cn('h-4 w-4', sensorConfig.gas.color)} />
          <span className={cn('font-mono text-sm font-bold', sensorConfig.gas.color)}>
            {gasCount}
          </span>
        </div>
      </div>

      <div className="flex items-center justify-between">
        <div className="flex items-center gap-1.5">
          <span className="font-mono text-sm text-[var(--success)]">
            {confirmedCount}
          </span>
          <span className="text-xs text-white/40">/</span>
          <span className="font-mono text-sm text-white/60">{totalCount}</span>
        </div>

        <div className="flex items-center gap-1 text-white/40">
          <span className="text-xs">Review</span>
          <ChevronRight className="h-4 w-4" />
        </div>
      </div>
    </Card>
  );
}
