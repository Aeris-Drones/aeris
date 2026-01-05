'use client';

import React from 'react';
import { motion } from 'framer-motion';
import { cn } from '@/lib/utils';
import { getConfidenceLevel, type ConfidenceLevel } from '@/lib/design-tokens';
import { transitions } from '@/lib/animations';

interface ConfidenceIndicatorProps {
  /** Confidence value from 0 to 1 (or 0 to 100 if isPercentage is true) */
  confidence: number;
  /** Whether the confidence value is already a percentage (0-100) */
  isPercentage?: boolean;
  /** Size variant */
  size?: 'sm' | 'md' | 'lg';
  /** Show the percentage label */
  showLabel?: boolean;
  /** Enable pulse animation for high confidence */
  animated?: boolean;
  /** Additional class names */
  className?: string;
}

const sizeConfig = {
  sm: {
    container: 'w-12 h-12',
    strokeWidth: 3,
    fontSize: 'text-xs',
    radius: 20,
  },
  md: {
    container: 'w-16 h-16',
    strokeWidth: 4,
    fontSize: 'text-sm',
    radius: 26,
  },
  lg: {
    container: 'w-24 h-24',
    strokeWidth: 5,
    fontSize: 'text-base',
    radius: 42,
  },
};

const levelColors: Record<ConfidenceLevel, string> = {
  high: 'stroke-confidence-high',
  medium: 'stroke-confidence-medium',
  low: 'stroke-confidence-low',
  unverified: 'stroke-confidence-unverified',
};

const levelTextColors: Record<ConfidenceLevel, string> = {
  high: 'text-confidence-high',
  medium: 'text-confidence-medium',
  low: 'text-confidence-low',
  unverified: 'text-confidence-unverified',
};

const levelLabels: Record<ConfidenceLevel, string> = {
  high: 'High',
  medium: 'Medium',
  low: 'Low',
  unverified: 'Unverified',
};

export function ConfidenceIndicator({
  confidence,
  isPercentage = false,
  size = 'md',
  showLabel = true,
  animated = true,
  className,
}: ConfidenceIndicatorProps) {
  // Normalize to 0-100 range
  const percentage = isPercentage ? confidence : confidence * 100;
  const normalizedValue = Math.min(100, Math.max(0, percentage));
  const level = getConfidenceLevel(normalizedValue);

  const config = sizeConfig[size];
  const circumference = 2 * Math.PI * config.radius;
  const strokeDashoffset = circumference - (normalizedValue / 100) * circumference;

  return (
    <div
      className={cn(
        'relative inline-flex items-center justify-center',
        config.container,
        className
      )}
    >
      {/* Background ring */}
      <svg className="absolute inset-0 -rotate-90" viewBox="0 0 64 64">
        <circle
          cx="32"
          cy="32"
          r={config.radius}
          fill="none"
          strokeWidth={config.strokeWidth}
          className="stroke-surface-3"
        />
      </svg>

      {/* Progress ring */}
      <svg className="absolute inset-0 -rotate-90" viewBox="0 0 64 64">
        <motion.circle
          cx="32"
          cy="32"
          r={config.radius}
          fill="none"
          strokeWidth={config.strokeWidth}
          strokeLinecap="round"
          className={cn(levelColors[level], animated && level === 'high' && 'drop-shadow-[0_0_6px_var(--confidence-high)]')}
          initial={{ strokeDashoffset: circumference }}
          animate={{ strokeDashoffset }}
          transition={{
            type: 'spring',
            stiffness: 50,
            damping: 20,
          }}
          style={{
            strokeDasharray: circumference,
          }}
        />
      </svg>

      {/* Center content */}
      {showLabel && (
        <motion.span
          className={cn(
            'font-mono font-semibold tabular-nums',
            config.fontSize,
            levelTextColors[level]
          )}
          initial={{ opacity: 0, scale: 0.8 }}
          animate={{ opacity: 1, scale: 1 }}
          transition={transitions.spring}
        >
          {Math.round(normalizedValue)}%
        </motion.span>
      )}
    </div>
  );
}

// Linear bar variant for inline use
interface ConfidenceBarProps {
  confidence: number;
  isPercentage?: boolean;
  showLabel?: boolean;
  size?: 'sm' | 'md';
  className?: string;
}

const barSizeConfig = {
  sm: {
    height: 'h-1.5',
    labelSize: 'text-xs',
  },
  md: {
    height: 'h-2',
    labelSize: 'text-sm',
  },
};

export function ConfidenceBar({
  confidence,
  isPercentage = false,
  showLabel = true,
  size = 'md',
  className,
}: ConfidenceBarProps) {
  const percentage = isPercentage ? confidence : confidence * 100;
  const normalizedValue = Math.min(100, Math.max(0, percentage));
  const level = getConfidenceLevel(normalizedValue);
  const config = barSizeConfig[size];

  const bgColorClass = {
    high: 'bg-confidence-high',
    medium: 'bg-confidence-medium',
    low: 'bg-confidence-low',
    unverified: 'bg-confidence-unverified',
  }[level];

  return (
    <div className={cn('flex items-center gap-2', className)}>
      <div
        className={cn(
          'flex-1 rounded-full bg-surface-3 overflow-hidden',
          config.height
        )}
      >
        <motion.div
          className={cn('h-full rounded-full', bgColorClass)}
          initial={{ width: 0 }}
          animate={{ width: `${normalizedValue}%` }}
          transition={{
            type: 'spring',
            stiffness: 80,
            damping: 20,
          }}
        />
      </div>
      {showLabel && (
        <span
          className={cn(
            'font-mono tabular-nums font-medium min-w-[3ch] text-right',
            config.labelSize,
            levelTextColors[level]
          )}
        >
          {Math.round(normalizedValue)}%
        </span>
      )}
    </div>
  );
}

// Compact badge variant
interface ConfidenceBadgeProps {
  confidence: number;
  isPercentage?: boolean;
  className?: string;
}

export function ConfidenceBadge({
  confidence,
  isPercentage = false,
  className,
}: ConfidenceBadgeProps) {
  const percentage = isPercentage ? confidence : confidence * 100;
  const normalizedValue = Math.min(100, Math.max(0, percentage));
  const level = getConfidenceLevel(normalizedValue);

  const bgColorClass = {
    high: 'bg-confidence-high/20 border-confidence-high/40',
    medium: 'bg-confidence-medium/20 border-confidence-medium/40',
    low: 'bg-confidence-low/20 border-confidence-low/40',
    unverified: 'bg-confidence-unverified/20 border-confidence-unverified/40',
  }[level];

  return (
    <span
      className={cn(
        'inline-flex items-center gap-1.5 px-2 py-0.5',
        'text-xs font-medium rounded-md border',
        bgColorClass,
        levelTextColors[level],
        className
      )}
    >
      <span className="font-mono tabular-nums">{Math.round(normalizedValue)}%</span>
      <span className="text-[0.65rem] uppercase tracking-wider opacity-70">
        {levelLabels[level]}
      </span>
    </span>
  );
}

// Dot indicator for compact spaces
interface ConfidenceDotProps {
  confidence: number;
  isPercentage?: boolean;
  size?: 'sm' | 'md' | 'lg';
  pulse?: boolean;
  className?: string;
}

const dotSizes = {
  sm: 'w-2 h-2',
  md: 'w-3 h-3',
  lg: 'w-4 h-4',
};

export function ConfidenceDot({
  confidence,
  isPercentage = false,
  size = 'md',
  pulse = true,
  className,
}: ConfidenceDotProps) {
  const percentage = isPercentage ? confidence : confidence * 100;
  const normalizedValue = Math.min(100, Math.max(0, percentage));
  const level = getConfidenceLevel(normalizedValue);

  const bgColorClass = {
    high: 'bg-confidence-high',
    medium: 'bg-confidence-medium',
    low: 'bg-confidence-low',
    unverified: 'bg-confidence-unverified',
  }[level];

  return (
    <span
      className={cn(
        'inline-block rounded-full',
        dotSizes[size],
        bgColorClass,
        pulse && level === 'high' && 'animate-pulse',
        className
      )}
    />
  );
}
