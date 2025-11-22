'use client';

import React from 'react';
import { useLayerVisibility } from '../../context/LayerVisibilityContext';
import { Button } from '@/components/ui/button';
import { Switch } from '@/components/ui/switch';
import { Camera, Video, Map as MapIcon } from 'lucide-react';
import { cn } from '@/lib/utils';

interface ControlPanelProps {
  className?: string;
  onCameraPreset: (preset: 'wide' | 'tracking' | 'overhead') => void;
}

export function ControlPanel({ className, onCameraPreset }: ControlPanelProps) {
  const { map, thermal, gas, acoustic, trajectories, toggleLayer } = useLayerVisibility();

  return (
    <div className={cn("flex flex-col gap-4 p-4 bg-zinc-800/90 backdrop-blur-md rounded-lg border border-zinc-700 shadow-xl w-64", className)}>

      {/* Camera Presets */}
      <div className="flex flex-col gap-2">
        <h3 className="text-xs font-semibold text-zinc-400 uppercase tracking-wider">Camera View</h3>
        <div className="grid grid-cols-3 gap-2">
            <Button variant="secondary" size="sm" className="h-8 px-0" onClick={() => onCameraPreset('wide')} title="Wide View" aria-label="Wide View">
                <Video className="h-4 w-4" />
            </Button>
            <Button variant="secondary" size="sm" className="h-8 px-0" onClick={() => onCameraPreset('tracking')} title="Tracking View" aria-label="Tracking View">
                <Camera className="h-4 w-4" />
            </Button>
            <Button variant="secondary" size="sm" className="h-8 px-0" onClick={() => onCameraPreset('overhead')} title="Overhead View" aria-label="Overhead View">
                <MapIcon className="h-4 w-4" />
            </Button>
        </div>
      </div>

      <div className="h-px bg-zinc-700" />

      {/* Layer Toggles */}
      <div className="flex flex-col gap-3">
        <h3 className="text-xs font-semibold text-zinc-400 uppercase tracking-wider">Layers</h3>

        <div className="flex items-center justify-between">
            <label htmlFor="layer-map" className="text-sm text-zinc-200">Map Tiles</label>
            <Switch id="layer-map" checked={map} onCheckedChange={() => toggleLayer('map')} />
        </div>

        <div className="flex items-center justify-between">
            <label htmlFor="layer-thermal" className="text-sm text-zinc-200">Thermal</label>
            <Switch id="layer-thermal" checked={thermal} onCheckedChange={() => toggleLayer('thermal')} />
        </div>

        <div className="flex items-center justify-between">
            <label htmlFor="layer-gas" className="text-sm text-zinc-200">Gas Plumes</label>
            <Switch id="layer-gas" checked={gas} onCheckedChange={() => toggleLayer('gas')} />
        </div>

        <div className="flex items-center justify-between">
            <label htmlFor="layer-acoustic" className="text-sm text-zinc-200">Acoustic</label>
            <Switch id="layer-acoustic" checked={acoustic} onCheckedChange={() => toggleLayer('acoustic')} />
        </div>

        <div className="flex items-center justify-between">
            <label htmlFor="layer-trajectories" className="text-sm text-zinc-200">Trajectories</label>
            <Switch id="layer-trajectories" checked={trajectories} onCheckedChange={() => toggleLayer('trajectories')} />
        </div>
      </div>

    </div>
  );
}
