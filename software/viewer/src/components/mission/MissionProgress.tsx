'use client';

/**
 * AERIS GCS Mission Progress Panel
 * 
 * Floating glass panel showing mission coverage visualization,
 * area statistics, and estimated time remaining. Features an
 * animated circular progress ring with glow effects.
 */

import React, { useMemo } from 'react';
import { motion } from 'framer-motion';
import { GlassPanel } from '@/components/ui/GlassPanel';
import { AnimatedMetric } from '@/components/ui/AnimatedMetric';
import { useMissionControl } from '@/hooks/useMissionControl';
import { formatDuration } from '@/types/mission';
import { transitions, panelVariants } from '@/lib/animations';
import { cn } from '@/lib/utils';
import { MapPin, Clock, Plane, Target } from 'lucide-react';

// ============================================================================
// Progress Ring Component
// ============================================================================

interface ProgressRingProps {
  /** Progress value (0-100) */
  progress: number;
  /** Ring size in pixels */
  size?: number;
  /** Stroke width in pixels */
  strokeWidth?: number;
  /** Whether to show glow effect */
  glow?: boolean;
  /** Custom class name */
  className?: string;
}

function ProgressRing({
  progress,
  size = 120,
  strokeWidth = 8,
  glow = true,
  className,
}: ProgressRingProps) {
  const radius = (size - strokeWidth) / 2;
  const circumference = 2 * Math.PI * radius;
  const offset = circumference - (Math.min(100, Math.max(0, progress)) / 100) * circumference;
  
  // Determine color based on progress
  const getProgressColor = () => {
    if (progress >= 80) return 'var(--confidence-high)';
    if (progress >= 40) return 'var(--confidence-medium)';
    return 'var(--confidence-low)';
  };
  
  const progressColor = getProgressColor();
  
  return (
    <div
      className={cn('relative', className)}
      style={{ width: size, height: size }}
    >
      {/* Background ring */}
      <svg className="absolute inset-0 -rotate-90">
        <circle
          cx={size / 2}
          cy={size / 2}
          r={radius}
          fill="none"
          stroke="var(--surface-3)"
          strokeWidth={strokeWidth}
          className="opacity-50"
        />
      </svg>
      
      {/* Progress ring with animation */}
      <svg className="absolute inset-0 -rotate-90">
        <defs>
          {glow && (
            <filter id="progress-glow">
              <feGaussianBlur stdDeviation="3" result="coloredBlur" />
              <feMerge>
                <feMergeNode in="coloredBlur" />
                <feMergeNode in="SourceGraphic" />
              </feMerge>
            </filter>
          )}
        </defs>
        <motion.circle
          cx={size / 2}
          cy={size / 2}
          r={radius}
          fill="none"
          stroke={progressColor}
          strokeWidth={strokeWidth}
          strokeLinecap="round"
          strokeDasharray={circumference}
          initial={{ strokeDashoffset: circumference }}
          animate={{ strokeDashoffset: offset }}
          transition={transitions.spring}
          filter={glow ? 'url(#progress-glow)' : undefined}
        />
      </svg>
      
      {/* Center content */}
      <div className="absolute inset-0 flex flex-col items-center justify-center">
        <motion.span
          className="text-2xl font-bold font-mono tabular-nums"
          style={{ color: progressColor }}
          key={Math.floor(progress)}
          initial={{ scale: 0.9, opacity: 0.5 }}
          animate={{ scale: 1, opacity: 1 }}
          transition={transitions.spring}
        >
          {Math.round(progress)}%
        </motion.span>
        <span className="text-[10px] text-muted-foreground uppercase tracking-wider mt-0.5">
          Coverage
        </span>
      </div>
    </div>
  );
}

// ============================================================================
// Stat Row Component
// ============================================================================

interface StatRowProps {
  icon: React.ReactNode;
  label: string;
  value: string | number;
  unit?: string;
  animated?: boolean;
}

function StatRow({ icon, label, value, unit, animated = true }: StatRowProps) {
  return (
    <div className="flex items-center justify-between py-1.5">
      <div className="flex items-center gap-2 text-muted-foreground">
        {icon}
        <span className="text-sm">{label}</span>
      </div>
      <div className="flex items-baseline gap-1">
        {animated && typeof value === 'number' ? (
          <AnimatedMetric
            value={value}
            precision={value < 10 ? 1 : 0}
            className="text-sm font-semibold font-mono tabular-nums text-foreground"
          />
        ) : (
          <span className="text-sm font-semibold font-mono tabular-nums text-foreground">
            {value}
          </span>
        )}
        {unit && (
          <span className="text-xs text-muted-foreground">{unit}</span>
        )}
      </div>
    </div>
  );
}

// ============================================================================
// Main Component
// ============================================================================

interface MissionProgressProps {
  /** Custom class name */
  className?: string;
  /** Whether panel is collapsible */
  collapsible?: boolean;
  /** Default collapsed state */
  defaultCollapsed?: boolean;
}

export function MissionProgress({
  className,
  collapsible = true,
  defaultCollapsed = false,
}: MissionProgressProps) {
  const {
    phase,
    isActive,
    coveragePercent,
    searchAreaKm2,
    activeDrones,
    totalDrones,
    estimatedTimeRemaining,
    detectionCounts,
  } = useMissionControl();
  
  // Format area for display
  const areaDisplay = useMemo(() => {
    if (searchAreaKm2 < 0.01) {
      return { value: searchAreaKm2 * 1000000, unit: 'm²' };
    }
    return { value: searchAreaKm2, unit: 'km²' };
  }, [searchAreaKm2]);
  
  // Format ETA
  const etaDisplay = useMemo(() => {
    if (!estimatedTimeRemaining || estimatedTimeRemaining <= 0) {
      return '—';
    }
    return `~${formatDuration(estimatedTimeRemaining)}`;
  }, [estimatedTimeRemaining]);
  
  // Don't show panel if mission hasn't started
  if (phase === 'IDLE') {
    return null;
  }
  
  return (
    <motion.div
      variants={panelVariants}
      initial="hidden"
      animate="visible"
      exit="exit"
      className={className}
    >
      <GlassPanel
        title="Mission Progress"
        collapsible={collapsible}
        defaultCollapsed={defaultCollapsed}
        className="w-[280px] pointer-events-auto"
        headerClassName="pb-2"
      >
        <div className="space-y-4">
          {/* Progress Ring */}
          <div className="flex justify-center py-2">
            <ProgressRing
              progress={coveragePercent}
              size={120}
              strokeWidth={8}
              glow={isActive}
            />
          </div>
          
          {/* Stats Grid */}
          <div className="space-y-0.5 border-t border-glass-border pt-3">
            <StatRow
              icon={<MapPin className="w-3.5 h-3.5" />}
              label="Area"
              value={areaDisplay.value.toFixed(1)}
              unit={areaDisplay.unit}
            />
            
            <StatRow
              icon={<Clock className="w-3.5 h-3.5" />}
              label="ETA"
              value={etaDisplay}
              animated={false}
            />
            
            <StatRow
              icon={<Plane className="w-3.5 h-3.5" />}
              label="Drones"
              value={`${activeDrones}/${totalDrones}`}
              animated={false}
            />
            
            <StatRow
              icon={<Target className="w-3.5 h-3.5" />}
              label="Detections"
              value={detectionCounts.total}
            />
          </div>
          
          {/* Detection breakdown (collapsed by default) */}
          {detectionCounts.total > 0 && (
            <div className="border-t border-glass-border pt-3">
              <div className="flex gap-3 text-xs">
                <div className="flex items-center gap-1.5">
                  <div className="w-2 h-2 rounded-full bg-sensor-thermal" />
                  <span className="text-muted-foreground">Thermal</span>
                  <span className="font-mono font-semibold text-foreground">
                    {detectionCounts.thermal}
                  </span>
                </div>
                <div className="flex items-center gap-1.5">
                  <div className="w-2 h-2 rounded-full bg-sensor-acoustic" />
                  <span className="text-muted-foreground">Acoustic</span>
                  <span className="font-mono font-semibold text-foreground">
                    {detectionCounts.acoustic}
                  </span>
                </div>
                <div className="flex items-center gap-1.5">
                  <div className="w-2 h-2 rounded-full bg-sensor-gas" />
                  <span className="text-muted-foreground">Gas</span>
                  <span className="font-mono font-semibold text-foreground">
                    {detectionCounts.gas}
                  </span>
                </div>
              </div>
            </div>
          )}
        </div>
      </GlassPanel>
    </motion.div>
  );
}

export default MissionProgress;
