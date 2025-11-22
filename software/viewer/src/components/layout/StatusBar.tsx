'use client';

import React from 'react';
import { cn } from "@/lib/utils";
import { Badge } from "@/components/ui/badge";
import { Progress } from "@/components/ui/progress";
import { Separator } from "@/components/ui/separator";
import { ConnectionStatus } from "@/components/ConnectionStatus";
import { MissionStateDisplay } from "@/components/ui/MissionStateDisplay";
import { Wifi, Battery, Navigation } from 'lucide-react';

interface StatusBarProps {
  className?: string;
}

function getGpsStatus(satellites: number): 'success' | 'warning' | 'danger' {
  if (satellites >= 7) return 'success';
  if (satellites >= 4) return 'warning';
  return 'danger';
}

function getBatteryStatus(percent: number): 'success' | 'warning' | 'danger' {
  if (percent >= 70) return 'success';
  if (percent >= 30) return 'warning';
  return 'danger';
}

function getLinkStatus(strength: number): 'success' | 'warning' | 'danger' {
  if (strength >= 3) return 'success';
  if (strength >= 2) return 'warning';
  return 'danger';
}

export function StatusBar({ className }: StatusBarProps) {
  // Mock data - will be replaced by real ROS data later
  const satellites = 8;
  const batteryPercent = 85;
  const linkStrength = 4;
  const flightMode = "LOITER";

  const gpsStatus = getGpsStatus(satellites);
  const batteryStatus = getBatteryStatus(batteryPercent);
  const linkStatus = getLinkStatus(linkStrength);

  return (
    <div className={cn("h-[60px] w-full bg-background border-b flex items-center px-4 justify-between", className)}>
      <div className="flex items-center space-x-4">
        <div className="font-bold text-lg tracking-tight">AERIS <span className="text-muted-foreground font-normal text-sm">GCS</span></div>
        <Separator orientation="vertical" className="h-6" />
        <ConnectionStatus />
        <Separator orientation="vertical" className="h-6" />
        <MissionStateDisplay />
      </div>

      <div className="flex items-center space-x-6">
        <Badge variant="secondary" className="px-3 py-1 text-xs uppercase tracking-wider">
            {flightMode}
        </Badge>

        <Separator orientation="vertical" className="h-6" />

        <div className="flex items-center space-x-2 text-sm">
            <Navigation className={cn("w-4 h-4", {
                "text-success": gpsStatus === 'success',
                "text-warning": gpsStatus === 'warning',
                "text-danger": gpsStatus === 'danger'
            })} />
            <span className="font-mono">{satellites} SAT</span>
        </div>

        <div className="flex items-center space-x-2 text-sm">
            <Wifi className={cn("w-4 h-4", {
                "text-success": linkStatus === 'success',
                "text-warning": linkStatus === 'warning',
                "text-danger": linkStatus === 'danger'
            })} />
            <div className="flex space-x-0.5">
                {[1, 2, 3, 4].map((bar) => (
                    <div 
                        key={bar} 
                        className={cn("w-1 h-3 rounded-sm", 
                            bar <= linkStrength 
                            ? (linkStatus === 'success' ? "bg-success" : linkStatus === 'warning' ? "bg-warning" : "bg-danger")
                            : "bg-secondary"
                        )}
                    />
                ))}
            </div>
        </div>

        <div className="flex items-center space-x-2 text-sm min-w-[100px]">
            <Battery className={cn("w-4 h-4", {
                "text-success": batteryStatus === 'success',
                "text-warning": batteryStatus === 'warning',
                "text-danger": batteryStatus === 'danger'
            })} />
            <div className="flex-1 flex flex-col">
                <div className="flex justify-between text-[10px] mb-1 leading-none">
                    <span>BAT</span>
                    <span className="font-mono">{batteryPercent}%</span>
                </div>
                <Progress value={batteryPercent} indicatorColor={batteryStatus} className="h-1.5" />
            </div>
        </div>
      </div>
    </div>
  );
}
