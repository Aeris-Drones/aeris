'use client';

import React from 'react';
import { useLayerVisibility } from '../../context/LayerVisibilityContext';
import { Button } from '@/components/ui/button';
import { Switch } from '@/components/ui/switch';
import { Camera, Video, Map as MapIcon } from 'lucide-react';
import { cn } from '@/lib/utils';

interface ControlPanelProps {
  className?: string;
  id?: string;
  onCameraPreset: (preset: 'wide' | 'tracking' | 'overhead') => void;
}

export function ControlPanel({ className, id, onCameraPreset }: ControlPanelProps) {
  const { map, thermal, gas, acoustic, trajectories, toggleLayer } = useLayerVisibility();

  return (
    <div id={id} className={cn("flex flex-col gap-4 p-4 bg-zinc-800/90 backdrop-blur-md rounded-lg border border-zinc-700 shadow-xl w-64", className)}>

      {/* Camera Presets */}
      <div className="flex flex-col gap-2">
        <h3 className="text-xs font-semibold text-zinc-400 uppercase tracking-wider">Camera View</h3>
        <div className="grid grid-cols-3 gap-2">
            <Button variant="secondary" size="sm" className="h-8 px-0" onClick={() => onCameraPreset('wide')} title="Wide View">
                <Video className="h-4 w-4" />
            </Button>
            <Button variant="secondary" size="sm" className="h-8 px-0" onClick={() => onCameraPreset('tracking')} title="Tracking View">
                <Camera className="h-4 w-4" />
            </Button>
            <Button variant="secondary" size="sm" className="h-8 px-0" onClick={() => onCameraPreset('overhead')} title="Overhead View">
                <MapIcon className="h-4 w-4" />
            </Button>
        </div>
      </div>

      <div className="h-px bg-zinc-700" />

      {/* Layer Toggles */}
      <div className="flex flex-col gap-3">
        <h3 className="text-xs font-semibold text-zinc-400 uppercase tracking-wider">Layers</h3>

        <div className="flex items-center justify-between">
            <span className="text-sm text-zinc-200">Map Tiles</span>
            <Switch checked={map} onCheckedChange={() => toggleLayer('map')} />
        </div>

        <div className="flex items-center justify-between">
            <span className="text-sm text-zinc-200">Thermal</span>
            <Switch checked={thermal} onCheckedChange={() => toggleLayer('thermal')} />
        </div>

        <div className="flex items-center justify-between">
            <span className="text-sm text-zinc-200">Gas Plumes</span>
            <Switch checked={gas} onCheckedChange={() => toggleLayer('gas')} />
        </div>

        <div className="flex items-center justify-between">
            <span className="text-sm text-zinc-200">Acoustic</span>
            <Switch checked={acoustic} onCheckedChange={() => toggleLayer('acoustic')} />
        </div>

        <div className="flex items-center justify-between">
            <span className="text-sm text-zinc-200">Trajectories</span>
            <Switch checked={trajectories} onCheckedChange={() => toggleLayer('trajectories')} />
        </div>
      </div>

    </div>
  );
}
