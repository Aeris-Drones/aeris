'use client';

import { useState } from 'react';
import {
  X,
  Maximize2,
  Minimize2,
  Signal,
  SignalZero,
  Battery,
  ArrowUp,
  Video,
  VideoOff,
} from 'lucide-react';
import { cn } from '@/lib/utils';

interface Vehicle {
  id: string;
  name: string;
  status: 'active' | 'idle' | 'warning' | 'returning' | 'error';
}

interface PiPVideoFeedProps {
  vehicleId: string;
  vehicleName: string;
  streamUrl?: string;
  batteryPercent: number;
  altitude: number;
  isLive: boolean;
  allVehicles: Vehicle[];
  onVehicleSwitch: (id: string) => void;
  onClose: () => void;
  onExpand: () => void;
}

function getBatteryColor(percent: number): string {
  if (percent > 60) return 'text-emerald-400';
  if (percent > 30) return 'text-amber-400';
  return 'text-red-400';
}

function getSignalIcon(isLive: boolean) {
  if (!isLive) return <SignalZero className="h-4 w-4" />;
  return <Signal className="h-4 w-4" />;
}

export function PiPVideoFeed({
  vehicleId,
  vehicleName,
  streamUrl,
  batteryPercent,
  altitude,
  isLive,
  allVehicles,
  onVehicleSwitch,
  onClose,
  onExpand,
}: PiPVideoFeedProps) {
  const [isExpanded, setIsExpanded] = useState(false);

  const handleExpand = () => {
    onExpand();
    setIsExpanded(!isExpanded);
  };

  if (isExpanded) {
    return (
      <div className="fixed inset-0 z-50 flex items-center justify-center bg-black/90 backdrop-blur-sm">
        <div className={cn(
          'w-full max-w-4xl mx-4 rounded-xl overflow-hidden',
          'bg-black/80 backdrop-blur-md',
          'border border-white/10',
          'shadow-2xl'
        )}>
          <div className="flex items-center justify-between px-4 py-3 border-b border-white/10">
            <div className="flex items-center gap-3">
              {isLive ? (
                <Video className="h-4 w-4 text-emerald-400" />
              ) : (
                <VideoOff className="h-4 w-4 text-white/30" />
              )}
              <span className="text-sm font-medium text-white">{vehicleName}</span>
              {isLive && (
                <span className="px-2 py-0.5 rounded bg-red-500/20 text-red-400 text-xs font-medium">
                  LIVE
                </span>
              )}
            </div>

            <div className="flex items-center gap-2">
              <button
                onClick={handleExpand}
                className="p-2 rounded hover:bg-white/10 text-white/50 hover:text-white transition-colors"
                title="Minimize"
              >
                <Minimize2 className="h-4 w-4" />
              </button>
              <button
                onClick={onClose}
                className="p-2 rounded hover:bg-white/10 text-white/50 hover:text-white transition-colors"
                title="Close"
              >
                <X className="h-4 w-4" />
              </button>
            </div>
          </div>

          <div className="relative aspect-video bg-black">
            {streamUrl ? (
              <div className="absolute inset-0 flex items-center justify-center">
                <span className="text-white/30 text-sm">Stream: {streamUrl}</span>
              </div>
            ) : (
              <div className="absolute inset-0 flex flex-col items-center justify-center gap-3">
                <div className="w-20 h-20 rounded-full bg-white/5 flex items-center justify-center">
                  <VideoOff className="h-8 w-8 text-white/20" />
                </div>
                <span className="text-white/30 text-sm">No video feed available</span>
              </div>
            )}

            <div className="absolute bottom-0 left-0 right-0 p-4 bg-gradient-to-t from-black/80 to-transparent">
              <div className="flex items-center justify-between text-sm">
                <div className="flex items-center gap-6">
                  <div className={cn('flex items-center gap-2', getBatteryColor(batteryPercent))}>
                    <Battery className="h-4 w-4" />
                    <span>{batteryPercent}%</span>
                  </div>
                  <div className="flex items-center gap-2 text-white/60">
                    <ArrowUp className="h-4 w-4" />
                    <span>{altitude}m</span>
                  </div>
                </div>

                <div className={cn(
                  'flex items-center gap-2',
                  isLive ? 'text-emerald-400' : 'text-white/30'
                )}>
                  {getSignalIcon(isLive)}
                  <span className="text-xs">{isLive ? 'Connected' : 'Offline'}</span>
                </div>
              </div>
            </div>
          </div>

          {allVehicles.length > 1 && (
            <div className="flex border-t border-white/10">
              {allVehicles.map((vehicle) => {
                const isActive = vehicle.id === vehicleId;
                const isOnline = vehicle.status === 'active' || vehicle.status === 'warning';

                return (
                  <button
                    key={vehicle.id}
                    onClick={() => onVehicleSwitch(vehicle.id)}
                    className={cn(
                      'flex-1 px-4 py-3 text-center transition-colors',
                      'border-r border-white/5 last:border-r-0',
                      isActive
                        ? 'bg-white/10 text-white'
                        : 'text-white/40 hover:text-white/60 hover:bg-white/5'
                    )}
                  >
                    <div className="flex items-center justify-center gap-2">
                      <span className="text-sm font-medium">{vehicle.name}</span>
                      <div className={cn(
                        'h-2 w-2 rounded-full',
                        isOnline ? 'bg-emerald-400' : 'bg-white/20'
                      )} />
                    </div>
                  </button>
                );
              })}
            </div>
          )}
        </div>
      </div>
    );
  }

  return (
    <div className={cn(
      'w-56 rounded-lg overflow-hidden',
      'bg-black/80 backdrop-blur-md',
      'border border-white/10',
      'shadow-xl shadow-black/50'
    )}>
      <div className="flex items-center justify-between px-3 py-2 border-b border-white/5">
        <div className="flex items-center gap-2">
          {isLive ? (
            <Video className="h-3.5 w-3.5 text-emerald-400" />
          ) : (
            <VideoOff className="h-3.5 w-3.5 text-white/30" />
          )}
          <span className="text-xs font-medium text-white">{vehicleName}</span>
          {isLive && (
            <span className="px-1.5 py-0.5 rounded bg-red-500/20 text-red-400 text-[10px] font-medium">
              LIVE
            </span>
          )}
        </div>

        <div className="flex items-center gap-1">
          <button
            onClick={handleExpand}
            className="p-1 rounded hover:bg-white/10 text-white/50 hover:text-white transition-colors"
            title="Expand"
          >
            <Maximize2 className="h-3.5 w-3.5" />
          </button>
          <button
            onClick={onClose}
            className="p-1 rounded hover:bg-white/10 text-white/50 hover:text-white transition-colors"
            title="Close"
          >
            <X className="h-3.5 w-3.5" />
          </button>
        </div>
      </div>

      <div className="relative aspect-video bg-black">
        {streamUrl ? (
          <div className="absolute inset-0 flex items-center justify-center">
            <span className="text-white/30 text-xs">Stream: {streamUrl}</span>
          </div>
        ) : (
          <div className="absolute inset-0 flex flex-col items-center justify-center gap-2">
            <div className="w-12 h-12 rounded-full bg-white/5 flex items-center justify-center">
              <VideoOff className="h-5 w-5 text-white/20" />
            </div>
            <span className="text-white/30 text-xs">No video feed</span>
          </div>
        )}

        <div className="absolute bottom-0 left-0 right-0 p-2 bg-gradient-to-t from-black/80 to-transparent">
          <div className="flex items-center justify-between text-[10px]">
            <div className="flex items-center gap-3">
              <div className={cn('flex items-center gap-1', getBatteryColor(batteryPercent))}>
                <Battery className="h-3 w-3" />
                <span>{batteryPercent}%</span>
              </div>
              <div className="flex items-center gap-1 text-white/60">
                <ArrowUp className="h-3 w-3" />
                <span>{altitude}m</span>
              </div>
            </div>

            <div className={cn(
              'flex items-center gap-1',
              isLive ? 'text-emerald-400' : 'text-white/30'
            )}>
              {isLive ? <Signal className="h-3 w-3" /> : <SignalZero className="h-3 w-3" />}
            </div>
          </div>
        </div>
      </div>

      {allVehicles.length > 1 && (
        <div className="flex border-t border-white/5">
          {allVehicles.slice(0, 4).map((vehicle) => {
            const isActive = vehicle.id === vehicleId;
            const isOnline = vehicle.status === 'active' || vehicle.status === 'warning';

            return (
              <button
                key={vehicle.id}
                onClick={() => onVehicleSwitch(vehicle.id)}
                className={cn(
                  'flex-1 px-2 py-2 text-center transition-colors',
                  'border-r border-white/5 last:border-r-0',
                  isActive
                    ? 'bg-white/10 text-white'
                    : 'text-white/40 hover:text-white/60 hover:bg-white/5'
                )}
              >
                <div className="flex flex-col items-center gap-0.5">
                  <span className="text-[10px] font-medium truncate max-w-full">
                    {vehicle.name.split(' ')[0]}
                  </span>
                  <div className={cn(
                    'h-1 w-1 rounded-full',
                    isOnline ? 'bg-emerald-400' : 'bg-white/20'
                  )} />
                </div>
              </button>
            );
          })}
        </div>
      )}
    </div>
  );
}
