'use client';

/**
 * AERIS GCS Zone Drawing Tool
 * 
 * Toolbar for drawing priority search zones on the map.
 */

import React from 'react';
import { motion, AnimatePresence } from 'framer-motion';
import { cn } from '@/lib/utils';
import { GlassPanel } from '@/components/ui/GlassPanel';
import { useZoneContext } from '@/context/ZoneContext';
import { getPriorityConfig, type ZonePriority } from '@/types/zone';
import { panelVariants, transitions } from '@/lib/animations';
import {
  PenTool,
  Square,
  Undo2,
  Check,
  X,
  AlertTriangle,
  AlertCircle,
  Info,
} from 'lucide-react';

// ============================================================================
// Priority Button
// ============================================================================

interface PriorityButtonProps {
  priority: ZonePriority;
  selected: boolean;
  onClick: () => void;
  disabled?: boolean;
}

function PriorityButton({ priority, selected, onClick, disabled }: PriorityButtonProps) {
  const config = getPriorityConfig(priority);
  
  return (
    <button
      onClick={onClick}
      disabled={disabled}
      className={cn(
        'flex-1 flex flex-col items-center justify-center gap-1 py-2 px-2 rounded-lg',
        'border transition-all duration-150',
        'min-h-[var(--touch-min)]',
        'disabled:opacity-50 disabled:cursor-not-allowed',
        selected
          ? `${config.bgColor} ${config.borderColor} ${config.color}`
          : 'bg-surface-2 border-glass-border text-muted-foreground hover:bg-surface-3'
      )}
    >
      {priority === 1 && <AlertTriangle className="w-4 h-4" />}
      {priority === 2 && <AlertCircle className="w-4 h-4" />}
      {priority === 3 && <Info className="w-4 h-4" />}
      <span className="text-xs font-medium">{config.label}</span>
    </button>
  );
}

// ============================================================================
// Main Component
// ============================================================================

interface ZoneDrawingToolProps {
  className?: string;
}

export function ZoneDrawingTool({ className }: ZoneDrawingToolProps) {
  const {
    drawing,
    isDrawing,
    startDrawing,
    cancelDrawing,
    finishDrawing,
    undoLastPoint,
    setPriority,
  } = useZoneContext();
  
  const pointCount = drawing.points.length;
  const canFinish = pointCount >= 3;
  const canUndo = pointCount > 0;
  
  // Not drawing - show start button
  if (!isDrawing) {
    return (
      <motion.div
        variants={panelVariants}
        initial="hidden"
        animate="visible"
        className={className}
      >
        <GlassPanel className="w-[200px] pointer-events-auto">
          <div className="space-y-3">
            <p className="text-xs text-muted-foreground text-center">
              Draw priority search zones
            </p>
            
            {/* Priority selection */}
            <div className="flex gap-2">
              <PriorityButton
                priority={1}
                selected={drawing.currentPriority === 1}
                onClick={() => setPriority(1)}
              />
              <PriorityButton
                priority={2}
                selected={drawing.currentPriority === 2}
                onClick={() => setPriority(2)}
              />
              <PriorityButton
                priority={3}
                selected={drawing.currentPriority === 3}
                onClick={() => setPriority(3)}
              />
            </div>
            
            {/* Start drawing button */}
            <button
              onClick={() => startDrawing(drawing.currentPriority)}
              className={cn(
                'w-full flex items-center justify-center gap-2 px-3 py-2.5',
                'bg-info/10 hover:bg-info/20 border border-info/30',
                'text-info text-sm font-medium rounded-lg',
                'transition-colors min-h-[var(--touch-min)]'
              )}
            >
              <PenTool className="w-4 h-4" />
              Start Drawing
            </button>
            
            <p className="text-[10px] text-muted-foreground/70 text-center">
              Press <kbd className="px-1 py-0.5 rounded bg-surface-3 font-mono">P</kbd> to toggle
            </p>
          </div>
        </GlassPanel>
      </motion.div>
    );
  }
  
  // Drawing mode - show controls
  return (
    <motion.div
      variants={panelVariants}
      initial="hidden"
      animate="visible"
      className={className}
    >
      <GlassPanel
        title="Drawing Zone"
        className="w-[240px] pointer-events-auto"
      >
        <div className="space-y-3">
          {/* Progress indicator */}
          <div className="flex items-center justify-between py-2 px-3 rounded-lg bg-surface-2/50">
            <div className="flex items-center gap-2">
              <Square className="w-4 h-4 text-info" />
              <span className="text-sm text-foreground">Points:</span>
            </div>
            <span className="font-mono text-sm font-semibold text-foreground">
              {pointCount}
            </span>
          </div>
          
          {/* Instructions */}
          <p className="text-xs text-muted-foreground">
            {pointCount === 0 && "Click on the map to add points"}
            {pointCount === 1 && "Click to add more points"}
            {pointCount === 2 && "Add at least 1 more point"}
            {pointCount >= 3 && "Click to add more or finish"}
          </p>
          
          {/* Priority indicator */}
          <div className="flex items-center gap-2">
            <span className="text-xs text-muted-foreground">Priority:</span>
            <span className={cn(
              'text-xs font-medium px-2 py-0.5 rounded',
              getPriorityConfig(drawing.currentPriority).bgColor,
              getPriorityConfig(drawing.currentPriority).color
            )}>
              {getPriorityConfig(drawing.currentPriority).label}
            </span>
          </div>
          
          {/* Action buttons */}
          <div className="flex gap-2">
            <button
              onClick={undoLastPoint}
              disabled={!canUndo}
              className={cn(
                'flex-1 flex items-center justify-center gap-1.5 px-3 py-2',
                'bg-surface-2 hover:bg-surface-3 border border-glass-border',
                'text-foreground text-sm font-medium rounded-lg',
                'transition-colors min-h-[var(--touch-min)]',
                'disabled:opacity-50 disabled:cursor-not-allowed'
              )}
            >
              <Undo2 className="w-4 h-4" />
              Undo
            </button>
            
            <button
              onClick={cancelDrawing}
              className={cn(
                'flex items-center justify-center gap-1.5 px-3 py-2',
                'bg-danger/10 hover:bg-danger/20 border border-danger/30',
                'text-danger text-sm font-medium rounded-lg',
                'transition-colors min-h-[var(--touch-min)]'
              )}
            >
              <X className="w-4 h-4" />
            </button>
          </div>
          
          {/* Finish button */}
          <AnimatePresence>
            {canFinish && (
              <motion.button
                initial={{ opacity: 0, y: 10 }}
                animate={{ opacity: 1, y: 0 }}
                exit={{ opacity: 0, y: 10 }}
                transition={transitions.fast}
                onClick={() => finishDrawing()}
                className={cn(
                  'w-full flex items-center justify-center gap-2 px-3 py-2.5',
                  'bg-success/10 hover:bg-success/20 border border-success/30',
                  'text-success text-sm font-medium rounded-lg',
                  'transition-colors min-h-[var(--touch-min)]'
                )}
              >
                <Check className="w-4 h-4" />
                Finish Zone
              </motion.button>
            )}
          </AnimatePresence>
        </div>
      </GlassPanel>
    </motion.div>
  );
}

export default ZoneDrawingTool;
