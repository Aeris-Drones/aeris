'use client';

import { useState } from 'react';
import {
  Map,
  Flame,
  Wind,
  AudioLines,
  Route,
  ChevronDown,
  ChevronUp,
  Layers,
  LucideIcon,
  Target,
} from 'lucide-react';
import { cn } from '@/lib/utils';
import { useLayerVisibility } from '@/context/LayerVisibilityContext';
import { useZoneContext } from '@/context/ZoneContext';
import type { ZonePriority } from '@/types/zone';

/**
 * Layer configuration for map visualization toggles.
 * IDs correspond to keys in LayerVisibilityContext.
 */
interface LayerItem {
  id: 'map' | 'thermal' | 'acoustic' | 'gas' | 'trajectories';
  label: string;
  icon: LucideIcon;
  color: string;
}

/**
 * Available map layers with semantic color coding.
 * Colors match sensor/detection conventions for consistency.
 */
const layers: LayerItem[] = [
  { id: 'map', label: 'Base Map', icon: Map, color: 'text-white' },
  { id: 'thermal', label: 'Thermal', icon: Flame, color: 'text-orange-400' },
  { id: 'acoustic', label: 'Acoustic', icon: AudioLines, color: 'text-sky-400' },
  { id: 'gas', label: 'Gas', icon: Wind, color: 'text-amber-400' },
  { id: 'trajectories', label: 'Paths', icon: Route, color: 'text-violet-400' },
];

/**
 * Zone priority levels for mission planning.
 * Priority 1 (Critical) is the highest urgency level.
 */
const zonePriorities: { priority: ZonePriority; label: string; color: string }[] = [
  { priority: 1, label: 'Critical', color: 'bg-red-500' },
  { priority: 2, label: 'High', color: 'bg-orange-500' },
  { priority: 3, label: 'Elevated', color: 'bg-yellow-500' },
];

/**
 * LayersPanel provides map layer toggles and zone creation controls.
 *
 * UI/UX Decisions:
 * - Collapsible design minimizes screen real estate when not in use
 * - Custom checkbox visuals replace native inputs for consistent styling
 * - Layer icons use color coding matching the data they represent
 * - Zone priority buttons use first-letter labels for compact display
 * - Disabled state during drawing prevents conflicting interactions
 *
 * Accessibility:
 * - All interactive elements are focusable buttons
 * - Visual feedback on hover and active states
 * - Layer state communicated through checkbox visuals and color
 * - Count indicator shows number of defined zones
 */
export function LayersPanel() {
  const [collapsed, setCollapsed] = useState(false);
  const visibility = useLayerVisibility();
  const { zones, isDrawing, startDrawing } = useZoneContext();

  const enabledCount = layers.filter(l => visibility[l.id]).length;

  // Collapsed state shows compact button with layer count
  if (collapsed) {
    return (
      <button
        onClick={() => setCollapsed(false)}
        className={cn(
          'flex items-center gap-2 px-3 py-2 rounded-lg',
          'bg-black/60 backdrop-blur-md',
          'border border-white/10',
          'text-white/70 hover:text-white',
          'transition-colors'
        )}
      >
        <Layers className="h-4 w-4" />
        <span className="text-xs font-medium">{enabledCount}/{layers.length}</span>
        <ChevronDown className="h-3 w-3" />
      </button>
    );
  }

  return (
    <div className={cn(
      'w-40 rounded-lg overflow-hidden',
      'bg-black/60 backdrop-blur-md',
      'border border-white/10'
    )}>
      <button
        onClick={() => setCollapsed(true)}
        className={cn(
          'w-full flex items-center justify-between px-3 py-2',
          'text-white/70 hover:text-white',
          'border-b border-white/5',
          'transition-colors'
        )}
      >
        <div className="flex items-center gap-2">
          <Layers className="h-4 w-4" />
          <span className="text-xs font-medium">Layers</span>
        </div>
        <ChevronUp className="h-3 w-3" />
      </button>

      {/* Layer toggle list with custom checkbox UI */}
      <div className="py-1">
        {layers.map((layer) => {
          const Icon = layer.icon;
          const enabled = visibility[layer.id];

          return (
            <button
              key={layer.id}
              onClick={() => visibility.toggleLayer(layer.id)}
              className={cn(
                'w-full flex items-center gap-3 px-3 py-2',
                'transition-all duration-150',
                enabled
                  ? 'text-white hover:bg-white/5'
                  : 'text-white/30 hover:text-white/50 hover:bg-white/5'
              )}
            >
              {/* Custom checkbox with animated state change */}
              <div className={cn(
                'h-4 w-4 rounded border flex items-center justify-center',
                'transition-colors duration-150',
                enabled
                  ? 'bg-white/20 border-white/40'
                  : 'border-white/20'
              )}>
                {enabled && (
                  <div className="h-2 w-2 rounded-sm bg-white" />
                )}
              </div>

              <Icon className={`h-4 w-4 transition-colors duration-150 ${enabled ? layer.color : 'text-white/30'}`} />

              <span className="text-xs">{layer.label}</span>
            </button>
          );
        })}
      </div>

      {/* Zone creation section with priority level buttons */}
      <div className="border-t border-white/5 py-2">
        <div className="px-3 pb-2">
          <div className="flex items-center gap-2 text-white/50">
            <Target className="h-3.5 w-3.5" />
            <span className="text-[10px] uppercase tracking-wider">Priority Zones</span>
            {zones.length > 0 && (
              <span className="ml-auto text-[10px] text-white/40">{zones.length}</span>
            )}
          </div>
        </div>

        <div className="px-2 flex gap-1">
          {zonePriorities.map(({ priority, label, color }) => (
            <button
              key={priority}
              onClick={() => startDrawing(priority)}
              disabled={isDrawing}
              className={cn(
                'flex-1 px-2 py-1.5 rounded text-[10px] font-medium',
                'border border-white/10',
                'transition-all duration-150',
                isDrawing
                  ? 'opacity-30 cursor-not-allowed'
                  : 'hover:bg-white/10 active:scale-95'
              )}
            >
              <div className="flex items-center justify-center gap-1.5">
                <div className={cn('h-2 w-2 rounded-full', color)} />
                <span className="text-white/70">{label[0]}</span>
              </div>
            </button>
          ))}
        </div>
      </div>
    </div>
  );
}
