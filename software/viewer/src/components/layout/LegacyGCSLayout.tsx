'use client';

import React from 'react';
import { motion, AnimatePresence } from 'framer-motion';
import { cn } from '@/lib/utils';
import { useGCSLayout } from '@/hooks/useBreakpoint';
import { slideLeftVariants, transitions } from '@/lib/animations';

/**
 * LegacyGCSLayout - Original layout for backwards compatibility
 * 
 * The new GCSLayout in ./GCSLayout.tsx uses the redesigned floating panel approach.
 * This legacy version supports the old page.tsx props structure.
 */

interface LegacyGCSLayoutProps {
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

export function LegacyGCSLayout({
  statusBar,
  map,
  sidebar,
  floatingPanels,
  bottomSheet,
}: LegacyGCSLayoutProps) {
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
