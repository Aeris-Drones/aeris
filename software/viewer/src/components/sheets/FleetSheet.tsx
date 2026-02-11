'use client';

import { useState } from 'react';
import {
  Drawer,
  DrawerContent,
  DrawerHeader,
  DrawerTitle,
  DrawerTrigger,
} from '@/components/ui/drawer';
import { VehicleCard, VehicleInfo } from './VehicleCard';
import { Button } from '@/components/ui/button';
import { Plane, Battery, AlertTriangle, OctagonX, Pause } from 'lucide-react';
import { cn } from '@/lib/utils';

interface FleetSheetProps {
  vehicles: VehicleInfo[];
  selectedVehicleId?: string | null;
  onLocate: (id: string) => void;
  onViewFeed: (id: string) => void;
  onRTH: (id: string) => void;
  onRecallAll?: () => void;
  onHoldPositions?: () => void;
  trigger: React.ReactNode;
}

export function FleetSheet({
  vehicles,
  selectedVehicleId,
  onLocate,
  onViewFeed,
  onRTH,
  onRecallAll,
  onHoldPositions,
  trigger,
}: FleetSheetProps) {
  const [open, setOpen] = useState(false);

  const handleViewFeed = (id: string) => {
    onViewFeed(id);
    setOpen(false);
  };

  const handleLocate = (id: string) => {
    onLocate(id);
    setOpen(false);
  };
  const activeCount = vehicles.filter(v => v.status === 'active' || v.status === 'warning').length;
  const warningCount = vehicles.filter(v => v.status === 'warning' || v.status === 'error').length;
  const avgBattery = Math.round(vehicles.reduce((sum, v) => sum + v.battery, 0) / (vehicles.length || 1));

  return (
    <Drawer open={open} onOpenChange={setOpen}>
      <DrawerTrigger asChild>
        <div className="cursor-pointer">
          {trigger}
        </div>
      </DrawerTrigger>
      <DrawerContent className="max-h-[85vh] bg-[#0a0a0f]/95 backdrop-blur-2xl border-white/[0.06]">
        <div className="mx-auto w-full max-w-3xl">
          <DrawerHeader className="pb-0">
            <div className="flex items-center justify-between">
              <div className="flex items-center gap-3">
                <Plane className="h-5 w-5 text-white/40" />
                <DrawerTitle className="text-lg font-light text-white tracking-wide">
                  Fleet
                </DrawerTitle>
              </div>
              <div className="flex items-center gap-5 text-xs">
                <span className="text-emerald-400">
                  <span className="font-mono text-sm">{activeCount}</span>
                  <span className="ml-1 text-white/40">active</span>
                </span>
                {warningCount > 0 && (
                  <span className="flex items-center gap-1 text-amber-400">
                    <AlertTriangle className="h-3 w-3" />
                    <span className="font-mono text-sm">{warningCount}</span>
                  </span>
                )}
                <span className="flex items-center gap-1.5 text-white/50">
                  <Battery className="h-3.5 w-3.5" />
                  <span className="font-mono text-sm">{avgBattery}%</span>
                  <span className="text-white/30">avg</span>
                </span>
              </div>
            </div>
          </DrawerHeader>

          <div className="flex items-center gap-2 px-4 py-4 border-b border-white/[0.04]">
            {vehicles.map((v) => (
              <div
                key={v.id}
                className={cn(
                  'h-2 flex-1 rounded-full transition-all',
                  v.status === 'active' && 'bg-emerald-500',
                  v.status === 'warning' && 'bg-amber-500',
                  v.status === 'error' && 'bg-red-500',
                  v.status === 'returning' && 'bg-cyan-500',
                  v.status === 'idle' && 'bg-white/20',
                  selectedVehicleId === v.id && 'ring-2 ring-white/50'
                )}
                title={v.name}
              />
            ))}
          </div>

          <div className="max-h-[45vh] overflow-y-auto px-4 py-4">
            <div className="grid grid-cols-1 gap-3 sm:grid-cols-2">
              {vehicles.map((vehicle) => (
                <VehicleCard
                  key={vehicle.id}
                  vehicle={vehicle}
                  isSelected={selectedVehicleId === vehicle.id}
                  onLocate={() => handleLocate(vehicle.id)}
                  onViewFeed={() => handleViewFeed(vehicle.id)}
                  onRTH={() => onRTH(vehicle.id)}
                />
              ))}
            </div>

            {vehicles.length === 0 && (
              <div className="py-16 text-center">
                <Plane className="h-8 w-8 text-white/10 mx-auto mb-3" />
                <p className="text-sm text-white/30">No vehicles connected</p>
              </div>
            )}
          </div>

          <div className="flex items-center justify-center gap-3 px-4 py-4 border-t border-white/[0.04]">
            <Button
              variant="outline"
              className={cn(
                'flex-1 h-10 max-w-[200px]',
                'border-red-500/30 text-red-400',
                'hover:bg-red-500/10 hover:border-red-500/50'
              )}
              onClick={onRecallAll}
            >
              <OctagonX className="mr-2 h-4 w-4" />
              Recall All
            </Button>
            <Button
              variant="outline"
              className={cn(
                'flex-1 h-10 max-w-[200px]',
                'border-amber-500/30 text-amber-400',
                'hover:bg-amber-500/10 hover:border-amber-500/50'
              )}
              onClick={onHoldPositions}
            >
              <Pause className="mr-2 h-4 w-4" />
              Hold Positions
            </Button>
          </div>
        </div>
      </DrawerContent>
    </Drawer>
  );
}
