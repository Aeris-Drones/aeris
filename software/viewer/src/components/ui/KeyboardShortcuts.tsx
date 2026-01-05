'use client';

/**
 * AERIS GCS Keyboard Shortcuts Overlay
 * 
 * Press '?' to show/hide the keyboard shortcuts help
 */

import React, { useState, useEffect } from 'react';
import { motion, AnimatePresence } from 'framer-motion';
import { cn } from '@/lib/utils';
import { transitions, panelVariants } from '@/lib/animations';
import { X, Keyboard } from 'lucide-react';

interface ShortcutGroup {
  title: string;
  shortcuts: { key: string; description: string }[];
}

const SHORTCUTS: ShortcutGroup[] = [
  {
    title: 'Mission Control',
    shortcuts: [
      { key: 'Space', description: 'Pause / Resume mission' },
      { key: 'Esc', description: 'Cancel current action' },
    ],
  },
  {
    title: 'Camera',
    shortcuts: [
      { key: 'C', description: 'Toggle camera controls' },
      { key: '1', description: 'Wide view' },
      { key: '2', description: 'Track drone' },
      { key: '3', description: 'Overhead view' },
      { key: 'R', description: 'Reset camera' },
    ],
  },
  {
    title: 'Panels',
    shortcuts: [
      { key: 'F', description: 'Toggle fleet panel' },
      { key: 'Z', description: 'Toggle zones panel' },
      { key: '?', description: 'Show shortcuts' },
    ],
  },
  {
    title: 'Navigation',
    shortcuts: [
      { key: 'Click + Drag', description: 'Orbit camera' },
      { key: 'Scroll', description: 'Zoom in/out' },
      { key: 'Right Click', description: 'Pan camera' },
    ],
  },
];

export function KeyboardShortcutsOverlay() {
  const [isOpen, setIsOpen] = useState(false);

  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      if (e.target instanceof HTMLInputElement || e.target instanceof HTMLTextAreaElement) return;

      if (e.key === '?') {
        e.preventDefault();
        setIsOpen((prev) => !prev);
      }

      if (e.key === 'Escape' && isOpen) {
        setIsOpen(false);
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [isOpen]);

  return (
    <AnimatePresence>
      {isOpen && (
        <>
          {/* Backdrop */}
          <motion.div
            initial={{ opacity: 0 }}
            animate={{ opacity: 1 }}
            exit={{ opacity: 0 }}
            onClick={() => setIsOpen(false)}
            className="fixed inset-0 bg-black/60 backdrop-blur-sm z-[100]"
          />

          {/* Panel */}
          <motion.div
            variants={panelVariants}
            initial="hidden"
            animate="visible"
            exit="exit"
            className={cn(
              'fixed top-1/2 left-1/2 -translate-x-1/2 -translate-y-1/2 z-[101]',
              'w-[90vw] max-w-[600px] max-h-[80vh] overflow-auto',
              'bg-glass-bg backdrop-blur-xl rounded-2xl',
              'border border-glass-border shadow-2xl shadow-black/30'
            )}
          >
            {/* Header */}
            <div className="sticky top-0 flex items-center justify-between p-4 border-b border-glass-border bg-glass-bg backdrop-blur-xl">
              <div className="flex items-center gap-3">
                <div className="w-10 h-10 rounded-xl bg-info/10 flex items-center justify-center">
                  <Keyboard className="w-5 h-5 text-info" />
                </div>
                <div>
                  <h2 className="font-semibold text-lg text-foreground">Keyboard Shortcuts</h2>
                  <p className="text-xs text-muted-foreground">Press ? to toggle this panel</p>
                </div>
              </div>
              <button
                onClick={() => setIsOpen(false)}
                className={cn(
                  'p-2 rounded-lg',
                  'text-muted-foreground hover:text-foreground',
                  'hover:bg-surface-3 transition-colors',
                  'min-h-[var(--touch-min)] min-w-[var(--touch-min)]'
                )}
              >
                <X className="w-5 h-5" />
              </button>
            </div>

            {/* Content */}
            <div className="p-4 grid gap-6 sm:grid-cols-2">
              {SHORTCUTS.map((group) => (
                <div key={group.title}>
                  <h3 className="text-xs font-semibold uppercase tracking-wider text-muted-foreground mb-3">
                    {group.title}
                  </h3>
                  <div className="space-y-2">
                    {group.shortcuts.map((shortcut) => (
                      <div
                        key={shortcut.key}
                        className="flex items-center justify-between gap-3 py-1.5"
                      >
                        <span className="text-sm text-foreground/80">
                          {shortcut.description}
                        </span>
                        <kbd
                          className={cn(
                            'px-2 py-1 rounded-md text-xs font-mono',
                            'bg-surface-3 text-muted-foreground',
                            'border border-surface-4'
                          )}
                        >
                          {shortcut.key}
                        </kbd>
                      </div>
                    ))}
                  </div>
                </div>
              ))}
            </div>

            {/* Footer */}
            <div className="p-4 border-t border-glass-border bg-surface-1/50">
              <p className="text-xs text-muted-foreground text-center">
                Tip: Use keyboard shortcuts for faster mission control during operations
              </p>
            </div>
          </motion.div>
        </>
      )}
    </AnimatePresence>
  );
}

export default KeyboardShortcutsOverlay;
