'use client';

import { Button } from '@/components/ui/button';
import { MapPin, Video, Home, Signal, Gauge, Radio } from 'lucide-react';
import { cn } from '@/lib/utils';

/**
 * VehicleCard - Premium grid card for fleet sheet
 * 
 * Tesla/SpaceX-inspired design:
 * - Status indicator glow
 * - Battery arc visualization
 * - Clean metric tiles
 * - Subtle glass morphism
 */

export type VehicleStatus = 'active' | 'warning' | 'error' | 'returning' | 'idle';

export interface VehicleInfo {
  id: string;
  name: string;
  status: VehicleStatus;
  battery: number;
  altitude: number;
  linkQuality?: number;
  coverage?: number;
}

export interface VehicleCardProps {
  vehicle: VehicleInfo;
  isSelected: boolean;
  onLocate: () => void;
  onViewFeed: () => void;
  onRTH?: () => void;
}

const statusConfig: Record<VehicleStatus, { 
  label: string; 
  color: string;
  glow: string;
  dot: string;
}> = {
  active: { 
    label: 'ACTIVE', 
    color: 'text-emerald-400',
    glow: 'shadow-[0_0_30px_rgba(52,211,153,0.15)]',
    dot: 'bg-emerald-400'
  },
  warning: { 
    label: 'WARNING', 
    color: 'text-amber-400',
    glow: 'shadow-[0_0_30px_rgba(251,191,36,0.15)]',
    dot: 'bg-amber-400'
  },
  error: { 
    label: 'ERROR', 
    color: 'text-red-400',
    glow: 'shadow-[0_0_30px_rgba(248,113,113,0.2)]',
    dot: 'bg-red-400'
  },
  returning: { 
    label: 'RTH', 
    color: 'text-cyan-400',
    glow: 'shadow-[0_0_30px_rgba(34,211,238,0.15)]',
    dot: 'bg-cyan-400'
  },
  idle: { 
    label: 'STANDBY', 
    color: 'text-white/40',
    glow: '',
    dot: 'bg-white/40'
  },
};

function getBatteryColor(battery: number): string {
  if (battery > 50) return 'text-emerald-400';
  if (battery > 20) return 'text-amber-400';
  return 'text-red-400';
}

function getBatteryStroke(battery: number): string {
  if (battery > 50) return 'stroke-emerald-400';
  if (battery > 20) return 'stroke-amber-400';
  return 'stroke-red-400';
}

// Battery arc SVG component - more compact
function BatteryArc({ battery }: { battery: number }) {
  const radius = 26;
  const circumference = 2 * Math.PI * radius;
  const strokeDashoffset = circumference * (1 - battery / 100);
  
  return (
    <svg className="h-16 w-16 -rotate-90" viewBox="0 0 60 60">
      {/* Background arc */}
      <circle
        cx="30"
        cy="30"
        r={radius}
        fill="none"
        stroke="currentColor"
        strokeWidth="3"
        className="text-white/[0.06]"
      />
      {/* Battery arc */}
      <circle
        cx="30"
        cy="30"
        r={radius}
        fill="none"
        strokeWidth="3"
        strokeLinecap="round"
        strokeDasharray={circumference}
        strokeDashoffset={strokeDashoffset}
        className={getBatteryStroke(battery)}
      />
    </svg>
  );
}

export function VehicleCard({
  vehicle,
  isSelected,
  onLocate,
  onViewFeed,
  onRTH,
}: VehicleCardProps) {
  const status = statusConfig[vehicle.status];

  return (
    <div
      className={cn(
        'group relative overflow-hidden rounded-xl transition-all duration-300',
        'bg-white/[0.03] backdrop-blur-md',
        'border border-white/[0.06]',
        status.glow,
        isSelected && 'ring-2 ring-cyan-500/50',
        vehicle.status === 'error' && 'border-red-500/20'
      )}
    >
      {/* Header - more compact */}
      <div className="flex items-start justify-between p-3 pb-0">
        <div className="flex flex-col gap-0.5">
          <span className="text-lg font-light text-white">{vehicle.name}</span>
          <div className="flex items-center gap-2">
            <span className={cn('h-1.5 w-1.5 rounded-full animate-pulse', status.dot)} />
            <span className={cn('text-[10px] font-semibold tracking-wider', status.color)}>
              {status.label}
            </span>
          </div>
        </div>
        
        {/* Battery gauge - smaller */}
        <div className="relative">
          <BatteryArc battery={vehicle.battery} />
          <div className="absolute inset-0 flex flex-col items-center justify-center">
            <span className={cn('font-mono text-base font-light tabular-nums', getBatteryColor(vehicle.battery))}>
              {vehicle.battery}
            </span>
            <span className="text-[8px] text-white/30">%</span>
          </div>
        </div>
      </div>

      {/* Metrics grid - more compact */}
      <div className="grid grid-cols-3 gap-px bg-white/[0.02] mx-3 my-2 rounded-lg overflow-hidden">
        <div className="flex flex-col items-center gap-0.5 bg-white/[0.02] py-2">
          <Gauge className="h-3 w-3 text-white/30" />
          <span className="font-mono text-xs text-white/70">{vehicle.altitude}</span>
          <span className="text-[8px] text-white/30 uppercase tracking-wider">Alt (m)</span>
        </div>
        <div className="flex flex-col items-center gap-0.5 bg-white/[0.02] py-2">
          <Radio className="h-3 w-3 text-white/30" />
          <span className="font-mono text-xs text-white/70">{vehicle.linkQuality ?? '--'}</span>
          <span className="text-[8px] text-white/30 uppercase tracking-wider">Link %</span>
        </div>
        <div className="flex flex-col items-center gap-0.5 bg-white/[0.02] py-2">
          <Signal className="h-3 w-3 text-white/30" />
          <span className="font-mono text-xs text-white/70">{vehicle.coverage ?? '--'}</span>
          <span className="text-[8px] text-white/30 uppercase tracking-wider">Cover %</span>
        </div>
      </div>

      {/* Actions - more compact */}
      <div className="flex items-center gap-1.5 p-3 pt-0">
        <Button
          variant="ghost"
          size="sm"
          className={cn(
            'flex-1 h-8 rounded-lg text-xs',
            'text-white/50 hover:text-white hover:bg-white/10'
          )}
          onClick={onLocate}
        >
          <MapPin className="mr-1 h-3 w-3" />
          Locate
        </Button>
        <Button
          variant="ghost"
          size="sm"
          className={cn(
            'flex-1 h-8 rounded-lg text-xs',
            'text-white/50 hover:text-white hover:bg-white/10'
          )}
          onClick={onViewFeed}
        >
          <Video className="mr-1 h-3 w-3" />
          Feed
        </Button>
        {onRTH && vehicle.status !== 'returning' && vehicle.status !== 'idle' && (
          <Button
            variant="ghost"
            size="sm"
            className={cn(
              'h-8 px-2 rounded-lg text-xs',
              'text-amber-400/70 hover:text-amber-400 hover:bg-amber-500/10'
            )}
            onClick={onRTH}
            title="Return to Home"
          >
            <Home className="h-3 w-3" />
          </Button>
        )}
      </div>
    </div>
  );
}
