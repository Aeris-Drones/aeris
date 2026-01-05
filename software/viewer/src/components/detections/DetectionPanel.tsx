'use client';

import React from 'react';
import { motion } from 'framer-motion';
import { cn } from '@/lib/utils';
import { panelVariants } from '@/lib/animations';
import { DetectionList } from './DetectionList';
import { useDetectionCounts, useNewDetections } from '@/hooks/useDetections';
import { useDetectionContext } from '@/context/DetectionContext';
import type { Detection } from '@/types/detection';
import { Badge } from '@/components/ui/badge';
import { Bell, Check, AlertTriangle, X, Undo2 } from 'lucide-react';
import { AnimatePresence } from 'framer-motion';

interface DetectionPanelProps {
  onDetectionFocus?: (detection: Detection) => void;
  className?: string;
}

export function DetectionPanel({ onDetectionFocus, className }: DetectionPanelProps) {
  const counts = useDetectionCounts();
  const newDetections = useNewDetections();
  const { canUndo, undoLastAction } = useDetectionContext();

  return (
    <div className={cn('flex flex-col h-full bg-surface-1', className)}>
      {/* Header */}
      <div className="flex-none px-4 py-3 border-b border-glass-border">
        <div className="flex items-center justify-between mb-3">
          <div className="flex items-center gap-2">
            <Bell className="w-5 h-5 text-foreground" />
            <h2 className="text-base font-semibold text-foreground">Detections</h2>
          </div>

          {/* Total count badge */}
          <Badge
            variant={counts.new > 0 ? 'default' : 'secondary'}
            className={cn(
              'font-mono tabular-nums',
              counts.new > 0 && 'bg-confidence-high/20 text-confidence-high border-confidence-high/40'
            )}
          >
            {counts.total}
          </Badge>
        </div>

        {/* Stats row */}
        <div className="flex items-center gap-2 text-xs">
          {/* New detections */}
          {counts.new > 0 && (
            <div className="flex items-center gap-1 px-2 py-1 rounded-md bg-confidence-high/10 text-confidence-high border border-confidence-high/20">
              <AlertTriangle className="w-3.5 h-3.5" />
              <span className="font-medium">{counts.new} new</span>
            </div>
          )}

          {/* Confirmed */}
          {counts.confirmed > 0 && (
            <div className="flex items-center gap-1 px-2 py-1 rounded-md bg-success/10 text-success">
              <Check className="w-3.5 h-3.5" />
              <span className="font-medium">{counts.confirmed}</span>
            </div>
          )}

          {/* High confidence count */}
          {counts.highConfidence > 0 && (
            <div className="flex items-center gap-1 px-2 py-1 rounded-md bg-surface-2 text-muted-foreground">
              <span className="font-medium">{counts.highConfidence} high conf.</span>
            </div>
          )}
        </div>
      </div>

      {/* Undo toast */}
      <AnimatePresence>
        {canUndo && (
          <motion.div
            initial={{ opacity: 0, y: -10 }}
            animate={{ opacity: 1, y: 0 }}
            exit={{ opacity: 0, y: -10 }}
            className="flex-none mx-3 mt-2 px-3 py-2 rounded-lg bg-info/10 border border-info/20"
          >
            <div className="flex items-center justify-between gap-2">
              <p className="text-xs text-info font-medium">Action recorded</p>
              <button
                onClick={undoLastAction}
                className={cn(
                  'flex items-center gap-1 px-2 py-1 rounded-md',
                  'text-xs font-medium',
                  'bg-info/20 hover:bg-info/30 text-info',
                  'transition-colors'
                )}
              >
                <Undo2 className="w-3 h-3" />
                Undo
              </button>
            </div>
          </motion.div>
        )}
      </AnimatePresence>

      {/* Detection list */}
      <div className="flex-1 overflow-hidden">
        <DetectionList
          showFilters={true}
          showSort={true}
          onDetectionFocus={onDetectionFocus}
        />
      </div>

      {/* Keyboard shortcuts hint */}
      <div className="flex-none px-4 py-2 border-t border-glass-border bg-surface-2/50">
        <p className="text-xs text-muted-foreground">
          <kbd className="px-1.5 py-0.5 rounded bg-surface-3 font-mono text-[10px]">Y</kbd> confirm
          {' · '}
          <kbd className="px-1.5 py-0.5 rounded bg-surface-3 font-mono text-[10px]">N</kbd> dismiss
        </p>
      </div>
    </div>
  );
}

/**
 * Compact variant for mobile/tablet bottom sheet
 */
export function DetectionPanelCompact({ onDetectionFocus, className }: DetectionPanelProps) {
  const counts = useDetectionCounts();

  return (
    <div className={cn('flex flex-col h-full bg-surface-1 rounded-t-2xl overflow-hidden', className)}>
      {/* Drag handle */}
      <div className="flex-none flex items-center justify-center py-2">
        <div className="w-12 h-1 rounded-full bg-muted-foreground/30" />
      </div>

      {/* Header */}
      <div className="flex-none px-4 pb-3 border-b border-glass-border">
        <div className="flex items-center justify-between">
          <h3 className="text-sm font-semibold text-foreground">Detections</h3>
          <Badge variant="secondary" className="font-mono tabular-nums">
            {counts.total}
          </Badge>
        </div>
      </div>

      {/* Compact detection list */}
      <div className="flex-1 overflow-hidden">
        <DetectionList
          compact={true}
          showFilters={false}
          showSort={false}
          onDetectionFocus={onDetectionFocus}
        />
      </div>
    </div>
  );
}
