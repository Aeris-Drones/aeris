'use client';

import { useState, useRef, useEffect } from 'react';
import { PenTool, Undo2, Check, X, AlertTriangle, AlertCircle, Info, Trash2, ChevronDown } from 'lucide-react';
import { cn } from '@/lib/utils';
import { useZoneContext } from '@/context/ZoneContext';
import type { ZonePriority } from '@/types/zone';

const priorityConfig: Record<ZonePriority, { label: string; color: string; bg: string; Icon: typeof AlertTriangle }> = {
  1: { label: 'CRITICAL', color: 'text-red-400', bg: 'bg-red-500/20', Icon: AlertTriangle },
  2: { label: 'HIGH', color: 'text-orange-400', bg: 'bg-orange-500/20', Icon: AlertCircle },
  3: { label: 'ELEVATED', color: 'text-yellow-400', bg: 'bg-yellow-500/20', Icon: Info },
};

function PriorityDropdown({
  value,
  onChange
}: {
  value: ZonePriority;
  onChange: (priority: ZonePriority) => void;
}) {
  const [open, setOpen] = useState(false);
  const ref = useRef<HTMLDivElement>(null);
  const current = priorityConfig[value];

  useEffect(() => {
    const handleClick = (e: MouseEvent) => {
      if (ref.current && !ref.current.contains(e.target as Node)) {
        setOpen(false);
      }
    };
    document.addEventListener('mousedown', handleClick);
    return () => document.removeEventListener('mousedown', handleClick);
  }, []);

  return (
    <div ref={ref} className="relative">
      <button
        onClick={() => setOpen(!open)}
        className={cn(
          'flex items-center gap-2 px-3 py-1.5 rounded-md',
          'bg-white/5 border border-white/10',
          'hover:bg-white/10 transition-colors',
          'text-sm font-medium'
        )}
      >
        <current.Icon className={cn('h-4 w-4', current.color)} />
        <span className={current.color}>{current.label}</span>
        <ChevronDown className={cn(
          'h-3.5 w-3.5 text-white/40 transition-transform',
          open && 'rotate-180'
        )} />
      </button>

      {open && (
        <div className={cn(
          'absolute top-full left-0 mt-1 z-50',
          'bg-black/90 backdrop-blur-md rounded-lg',
          'border border-white/10 overflow-hidden',
          'min-w-[140px]'
        )}>
          {([1, 2, 3] as ZonePriority[]).map((priority) => {
            const config = priorityConfig[priority];
            const isSelected = priority === value;
            return (
              <button
                key={priority}
                onClick={() => {
                  onChange(priority);
                  setOpen(false);
                }}
                className={cn(
                  'w-full flex items-center gap-2 px-3 py-2',
                  'text-left text-sm transition-colors',
                  isSelected ? config.bg : 'hover:bg-white/5'
                )}
              >
                <config.Icon className={cn('h-4 w-4', config.color)} />
                <span className={config.color}>{config.label}</span>
              </button>
            );
          })}
        </div>
      )}
    </div>
  );
}

export function ZoneToolbar() {
  const {
    zones,
    drawing,
    isDrawing,
    selectedZoneId,
    startDrawing,
    cancelDrawing,
    finishDrawing,
    undoLastPoint,
    setPriority,
    deleteZone,
    selectZone,
  } = useZoneContext();

  const [zoneName, setZoneName] = useState('');

  const pointCount = drawing.points.length;
  const canFinish = pointCount >= 3;
  const canUndo = pointCount > 0;

  const handleFinish = () => {
    const name = zoneName.trim() || undefined;
    finishDrawing(name);
    setZoneName('');
  };

  const handleCancel = () => {
    cancelDrawing();
    setZoneName('');
  };

  const handleDeleteSelected = () => {
    if (selectedZoneId) {
      deleteZone(selectedZoneId);
      selectZone(null);
    }
  };

  if (!isDrawing) {
    return (
      <div className={cn(
        'flex items-center gap-2 px-3 py-2 rounded-lg',
        'bg-black/60 backdrop-blur-md',
        'border border-white/10'
      )}>
        <span className="text-xs text-white/40">Priority:</span>
        <PriorityDropdown value={drawing.currentPriority} onChange={setPriority} />

        <div className="w-px h-5 bg-white/10" />

        <button
          onClick={() => startDrawing(drawing.currentPriority)}
          className={cn(
            'flex items-center gap-2 px-3 py-1.5 rounded-md',
            'bg-cyan-500/20 text-cyan-400',
            'hover:bg-cyan-500/30 transition-colors',
            'text-sm font-medium'
          )}
        >
          <PenTool className="h-4 w-4" />
          Draw Zone
        </button>

        {zones.length > 0 && (
          <>
            <div className="w-px h-5 bg-white/10" />
            <span className="text-xs text-white/40">{zones.length} zone{zones.length !== 1 ? 's' : ''}</span>

            {selectedZoneId && (
              <button
                onClick={handleDeleteSelected}
                className={cn(
                  'p-2 rounded-md transition-colors',
                  'text-red-400/70 hover:text-red-400 hover:bg-red-500/10'
                )}
                title="Delete selected zone"
              >
                <Trash2 className="h-4 w-4" />
              </button>
            )}
          </>
        )}
      </div>
    );
  }

  const currentPriority = priorityConfig[drawing.currentPriority];

  return (
    <div className={cn(
      'flex items-center gap-3 px-4 py-2.5 rounded-lg',
      'bg-black/60 backdrop-blur-md',
      'border border-white/10'
    )}>
      <div className={cn('flex items-center gap-2', currentPriority.color)}>
        <currentPriority.Icon className="h-4 w-4" />
        <span className="text-xs font-medium">{currentPriority.label}</span>
      </div>

      <div className="w-px h-5 bg-white/10" />

      <input
        type="text"
        value={zoneName}
        onChange={(e) => setZoneName(e.target.value)}
        placeholder="Zone name..."
        className={cn(
          'w-28 px-2 py-1 rounded bg-white/5 border border-white/10',
          'text-xs text-white placeholder:text-white/30',
          'focus:outline-none focus:border-white/20'
        )}
      />

      <div className="w-px h-5 bg-white/10" />

      <div className="flex items-center gap-2">
        <span className="text-xs text-white/50">Points:</span>
        <span className="font-mono text-sm text-white">{pointCount}</span>
        {pointCount < 3 && (
          <span className="text-xs text-white/30">(need {3 - pointCount} more)</span>
        )}
      </div>

      <div className="w-px h-5 bg-white/10" />

      <div className="flex items-center gap-1.5">
        <button
          onClick={undoLastPoint}
          disabled={!canUndo}
          className={cn(
            'p-2 rounded-md transition-colors',
            canUndo
              ? 'text-white/50 hover:text-white hover:bg-white/10'
              : 'text-white/20 cursor-not-allowed'
          )}
          title="Undo"
        >
          <Undo2 className="h-4 w-4" />
        </button>

        <button
          onClick={handleCancel}
          className={cn(
            'p-2 rounded-md transition-colors',
            'text-red-400/70 hover:text-red-400 hover:bg-red-500/10'
          )}
          title="Cancel"
        >
          <X className="h-4 w-4" />
        </button>

        <button
          onClick={handleFinish}
          disabled={!canFinish}
          className={cn(
            'flex items-center gap-1.5 px-3 py-1.5 rounded-md transition-colors',
            canFinish
              ? 'bg-emerald-500/20 text-emerald-400 hover:bg-emerald-500/30'
              : 'bg-white/5 text-white/20 cursor-not-allowed'
          )}
        >
          <Check className="h-4 w-4" />
          <span className="text-sm font-medium">Finish</span>
        </button>
      </div>
    </div>
  );
}
