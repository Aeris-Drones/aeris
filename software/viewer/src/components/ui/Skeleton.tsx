'use client';

/**
 * AERIS GCS Skeleton Components
 * 
 * Premium loading skeletons with shimmer effect
 * Match the shape of actual UI components
 */

import React from 'react';
import { motion } from 'framer-motion';
import { cn } from '@/lib/utils';

// ============================================================================
// Base Skeleton
// ============================================================================

interface SkeletonProps {
  className?: string;
  children?: React.ReactNode;
}

export function Skeleton({ className, children }: SkeletonProps) {
  return (
    <div
      className={cn(
        'relative overflow-hidden rounded-lg bg-surface-2',
        className
      )}
    >
      <motion.div
        className="absolute inset-0 -translate-x-full"
        style={{
          background:
            'linear-gradient(90deg, transparent, oklch(from var(--foreground) l c h / 0.05), transparent)',
        }}
        animate={{ translateX: ['(-100%)', '(100%)'] }}
        transition={{
          duration: 1.5,
          repeat: Infinity,
          ease: 'linear',
        }}
      />
      {children}
    </div>
  );
}

// ============================================================================
// Panel Skeleton
// ============================================================================

export function PanelSkeleton({ className }: { className?: string }) {
  return (
    <div
      className={cn(
        'rounded-2xl bg-glass-bg backdrop-blur-xl border border-glass-border p-4',
        className
      )}
    >
      {/* Header */}
      <div className="flex items-center gap-3 mb-4">
        <Skeleton className="w-8 h-8 rounded-lg" />
        <Skeleton className="h-4 w-24 rounded" />
        <Skeleton className="ml-auto h-4 w-12 rounded" />
      </div>

      {/* Content */}
      <div className="space-y-3">
        <Skeleton className="h-10 w-full rounded-lg" />
        <Skeleton className="h-10 w-full rounded-lg" />
        <Skeleton className="h-10 w-3/4 rounded-lg" />
      </div>
    </div>
  );
}

// ============================================================================
// Card Skeleton (for detection/vehicle cards)
// ============================================================================

export function CardSkeleton({ className }: { className?: string }) {
  return (
    <div
      className={cn(
        'rounded-xl bg-surface-2/50 border border-glass-border p-3',
        className
      )}
    >
      <div className="flex items-start gap-3">
        {/* Icon */}
        <Skeleton className="w-10 h-10 rounded-lg shrink-0" />

        {/* Content */}
        <div className="flex-1 min-w-0 space-y-2">
          <div className="flex items-center gap-2">
            <Skeleton className="h-4 w-20 rounded" />
            <Skeleton className="h-5 w-12 rounded-full" />
          </div>
          <Skeleton className="h-3 w-32 rounded" />
          <Skeleton className="h-3 w-24 rounded" />
        </div>

        {/* Actions */}
        <div className="flex gap-1">
          <Skeleton className="w-8 h-8 rounded-lg" />
          <Skeleton className="w-8 h-8 rounded-lg" />
        </div>
      </div>
    </div>
  );
}

// ============================================================================
// List Skeleton
// ============================================================================

export function ListSkeleton({
  count = 3,
  className,
}: {
  count?: number;
  className?: string;
}) {
  return (
    <div className={cn('space-y-2', className)}>
      {Array.from({ length: count }).map((_, i) => (
        <CardSkeleton key={i} />
      ))}
    </div>
  );
}

// ============================================================================
// Metric Skeleton
// ============================================================================

export function MetricSkeleton({ className }: { className?: string }) {
  return (
    <div className={cn('flex flex-col gap-1', className)}>
      <Skeleton className="h-3 w-12 rounded" />
      <Skeleton className="h-6 w-16 rounded" />
    </div>
  );
}

// ============================================================================
// Progress Ring Skeleton
// ============================================================================

export function ProgressRingSkeleton({
  size = 120,
  className,
}: {
  size?: number;
  className?: string;
}) {
  const strokeWidth = 8;
  const radius = (size - strokeWidth) / 2;
  const circumference = 2 * Math.PI * radius;

  return (
    <div className={cn('relative', className)} style={{ width: size, height: size }}>
      <svg width={size} height={size} className="rotate-[-90deg]">
        {/* Track */}
        <circle
          cx={size / 2}
          cy={size / 2}
          r={radius}
          fill="none"
          stroke="var(--surface-3)"
          strokeWidth={strokeWidth}
        />
        {/* Shimmer */}
        <motion.circle
          cx={size / 2}
          cy={size / 2}
          r={radius}
          fill="none"
          stroke="var(--surface-4)"
          strokeWidth={strokeWidth}
          strokeLinecap="round"
          strokeDasharray={circumference}
          animate={{
            strokeDashoffset: [circumference, 0],
          }}
          transition={{
            duration: 1.5,
            repeat: Infinity,
            ease: 'linear',
          }}
          style={{
            opacity: 0.3,
          }}
        />
      </svg>

      {/* Center content */}
      <div className="absolute inset-0 flex flex-col items-center justify-center">
        <Skeleton className="h-6 w-10 rounded" />
        <Skeleton className="h-3 w-16 rounded mt-1" />
      </div>
    </div>
  );
}

// ============================================================================
// Status Bar Skeleton
// ============================================================================

export function StatusBarSkeleton({ className }: { className?: string }) {
  return (
    <div className={cn('flex items-center justify-between h-12 px-4', className)}>
      {/* Left */}
      <div className="flex items-center gap-4">
        <div className="flex items-center gap-2">
          <Skeleton className="w-8 h-8 rounded-lg" />
          <Skeleton className="h-4 w-24 rounded hidden sm:block" />
        </div>
        <Skeleton className="w-px h-6 rounded" />
        <div className="flex items-center gap-2">
          <Skeleton className="h-6 w-20 rounded-md" />
          <Skeleton className="h-4 w-14 rounded" />
        </div>
      </div>

      {/* Center */}
      <div className="flex items-center gap-3">
        <Skeleton className="h-6 w-24 rounded-md" />
        <Skeleton className="h-6 w-28 rounded-md" />
      </div>

      {/* Right */}
      <Skeleton className="h-6 w-24 rounded-md" />
    </div>
  );
}

// ============================================================================
// Map Skeleton
// ============================================================================

export function MapSkeleton({ className }: { className?: string }) {
  return (
    <div
      className={cn(
        'relative bg-surface-1 rounded-2xl overflow-hidden',
        className
      )}
    >
      {/* Grid pattern */}
      <div
        className="absolute inset-0 opacity-10"
        style={{
          backgroundImage: `
            linear-gradient(var(--surface-3) 1px, transparent 1px),
            linear-gradient(90deg, var(--surface-3) 1px, transparent 1px)
          `,
          backgroundSize: '40px 40px',
        }}
      />

      {/* Floating loading indicator */}
      <div className="absolute inset-0 flex items-center justify-center">
        <div className="flex flex-col items-center gap-3 p-6 rounded-2xl bg-glass-bg backdrop-blur-xl border border-glass-border">
          <motion.div
            className="w-8 h-8 border-2 border-info/30 border-t-info rounded-full"
            animate={{ rotate: 360 }}
            transition={{ duration: 1, repeat: Infinity, ease: 'linear' }}
          />
          <span className="text-sm text-muted-foreground">Loading scene...</span>
        </div>
      </div>
    </div>
  );
}

// ============================================================================
// Full Page Skeleton
// ============================================================================

export function DashboardSkeleton() {
  return (
    <div className="h-screen w-screen bg-background flex flex-col">
      {/* Status Bar */}
      <div className="h-12 border-b border-glass-border bg-surface-1">
        <StatusBarSkeleton />
      </div>

      {/* Main Content */}
      <div className="flex-1 flex">
        {/* Sidebar (desktop) */}
        <div className="hidden lg:block w-[320px] border-r border-glass-border bg-surface-1 p-4">
          <PanelSkeleton className="h-full" />
        </div>

        {/* Map Area */}
        <div className="flex-1 relative">
          <MapSkeleton className="absolute inset-0" />

          {/* Floating Panels */}
          <div className="absolute top-4 left-4">
            <PanelSkeleton className="w-[240px]" />
          </div>

          <div className="absolute bottom-4 left-4">
            <PanelSkeleton className="w-[200px]" />
          </div>

          <div className="absolute bottom-4 right-4">
            <Skeleton className="w-[200px] h-[200px] rounded-2xl" />
          </div>
        </div>
      </div>
    </div>
  );
}

export default Skeleton;
