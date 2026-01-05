'use client';

import React from 'react';
import { motion, AnimatePresence } from 'framer-motion';
import { cn } from '@/lib/utils';
import { useGCSLayout } from '@/hooks/useBreakpoint';
import { slideLeftVariants, transitions } from '@/lib/animations';

interface GCSLayoutProps {
  /** Status bar content (always visible) */
  statusBar: React.ReactNode;
  /** Main 3D scene/map content */
  map: React.ReactNode;
  /** Sidebar content (detections, fleet, etc.) - shows on desktop */
  sidebar?: React.ReactNode;
  /** Floating panel layer content */
  floatingPanels?: React.ReactNode;
  /** Bottom sheet content for mobile/tablet */
  bottomSheet?: React.ReactNode;
}

/**
 * Responsive GCS Layout
 *
 * Desktop (xl+): StatusBar + Sidebar + Map with floating panels
 * Tablet (md-lg): StatusBar + Full map + Bottom sheet
 * Mobile (<md): StatusBar + Full map + Bottom sheet (compact)
 */
export function GCSLayout({
  statusBar,
  map,
  sidebar,
  floatingPanels,
  bottomSheet,
}: GCSLayoutProps) {
  const { showSidebar, showMobileBottomSheet, useCompactStatusBar } = useGCSLayout();

  return (
    <div className="flex flex-col h-screen w-screen overflow-hidden bg-surface-0 text-foreground">
      {/* Status Bar - always visible */}
      <header
        className={cn(
          'flex-none z-50 border-b border-glass-border',
          useCompactStatusBar ? 'h-12' : 'h-14'
        )}
      >
        {statusBar}
      </header>

      {/* Main content area */}
      <div className="flex-1 flex overflow-hidden relative">
        {/* Desktop Sidebar */}
        <AnimatePresence mode="wait">
          {showSidebar && sidebar && (
            <motion.aside
              variants={slideLeftVariants}
              initial="hidden"
              animate="visible"
              exit="exit"
              className={cn(
                'flex-none w-[360px] z-40',
                'border-r border-glass-border',
                'bg-surface-1',
                'overflow-hidden'
              )}
            >
              {sidebar}
            </motion.aside>
          )}
        </AnimatePresence>

        {/* Map/Scene container */}
        <main className="flex-1 relative min-w-0 bg-surface-0">
          {/* The 3D scene */}
          {map}

          {/* Floating panels layer */}
          {floatingPanels && (
            <div className="absolute inset-0 pointer-events-none z-30 [&>*]:pointer-events-auto">
              {floatingPanels}
            </div>
          )}
        </main>
      </div>

      {/* Mobile/Tablet Bottom Sheet */}
      <AnimatePresence>
        {showMobileBottomSheet && bottomSheet && (
          <motion.div
            initial={{ y: '100%' }}
            animate={{ y: 0 }}
            exit={{ y: '100%' }}
            transition={transitions.spring}
            className="absolute bottom-0 left-0 right-0 z-40"
          >
            {bottomSheet}
          </motion.div>
        )}
      </AnimatePresence>
    </div>
  );
}

/**
 * Legacy Grid Layout - keeping for backwards compatibility
 * @deprecated Use GCSLayout instead
 */
interface LegacyGCSLayoutProps {
  statusBar: React.ReactNode;
  map: React.ReactNode;
  telemetry: React.ReactNode;
}

export function GCSGridLayout({ statusBar, map, telemetry }: LegacyGCSLayoutProps) {
  return (
    <div
      className={cn(
        'grid h-screen w-screen overflow-hidden bg-background text-foreground font-sans',
        'grid-rows-[56px_minmax(0,1fr)]',
        'grid-cols-1 xl:grid-cols-[minmax(0,1fr)_360px]'
      )}
    >
      {/* Status bar spans full width */}
      <div className="col-span-1 xl:col-span-2 z-50 border-b border-glass-border">
        {statusBar}
      </div>

      {/* Main scene */}
      <div className="relative bg-surface-0 overflow-hidden">{map}</div>

      {/* Sidebar/Telemetry - hidden on smaller screens */}
      <div className="hidden xl:block overflow-y-auto border-l border-glass-border bg-surface-1 z-40">
        {telemetry}
      </div>
    </div>
  );
}

/**
 * Floating Panel Positions
 */
export type FloatingPanelPosition =
  | 'top-left'
  | 'top-right'
  | 'bottom-left'
  | 'bottom-right'
  | 'top-center'
  | 'bottom-center';

const positionStyles: Record<FloatingPanelPosition, string> = {
  'top-left': 'top-4 left-4',
  'top-right': 'top-4 right-4',
  'bottom-left': 'bottom-4 left-4',
  'bottom-right': 'bottom-4 right-4',
  'top-center': 'top-4 left-1/2 -translate-x-1/2',
  'bottom-center': 'bottom-4 left-1/2 -translate-x-1/2',
};

interface FloatingPanelContainerProps {
  position: FloatingPanelPosition;
  children: React.ReactNode;
  className?: string;
}

export function FloatingPanelContainer({
  position,
  children,
  className,
}: FloatingPanelContainerProps) {
  return (
    <div className={cn('absolute', positionStyles[position], className)}>
      {children}
    </div>
  );
}

/**
 * Floating Panels Wrapper
 * Use this to wrap multiple floating panels with proper positioning
 */
interface FloatingPanelsProps {
  children: React.ReactNode;
}

export function FloatingPanels({ children }: FloatingPanelsProps) {
  return (
    <div className="absolute inset-0 pointer-events-none z-30 [&>*]:pointer-events-auto">
      {children}
    </div>
  );
}
