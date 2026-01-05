'use client';

import React from 'react';
import { motion, AnimatePresence } from 'framer-motion';
import { cn } from '@/lib/utils';
import { panelVariants, transitions } from '@/lib/animations';
import { X, Minus, Maximize2 } from 'lucide-react';

export type GlassPanelPosition =
  | 'top-left'
  | 'top-right'
  | 'bottom-left'
  | 'bottom-right'
  | 'center'
  | 'custom';

interface GlassPanelProps {
  children: React.ReactNode;
  title?: string;
  icon?: React.ReactNode;
  /** Content to render on the right side of the header */
  headerRight?: React.ReactNode;
  position?: GlassPanelPosition;
  width?: number | string;
  maxHeight?: number | string;
  collapsible?: boolean;
  closable?: boolean;
  defaultCollapsed?: boolean;
  className?: string;
  headerClassName?: string;
  contentClassName?: string;
  onClose?: () => void;
  onCollapse?: (collapsed: boolean) => void;
  visible?: boolean;
  style?: React.CSSProperties;
}

const positionClasses: Record<GlassPanelPosition, string> = {
  'top-left': 'top-4 left-4',
  'top-right': 'top-4 right-4',
  'bottom-left': 'bottom-4 left-4',
  'bottom-right': 'bottom-4 right-4',
  'center': 'top-1/2 left-1/2 -translate-x-1/2 -translate-y-1/2',
  'custom': '',
};

export function GlassPanel({
  children,
  title,
  icon,
  headerRight,
  position = 'custom',
  width,
  maxHeight,
  collapsible = false,
  closable = false,
  defaultCollapsed = false,
  className,
  headerClassName,
  contentClassName,
  onClose,
  onCollapse,
  visible = true,
  style,
}: GlassPanelProps) {
  const [isCollapsed, setIsCollapsed] = React.useState(defaultCollapsed);

  const handleCollapse = () => {
    const newState = !isCollapsed;
    setIsCollapsed(newState);
    onCollapse?.(newState);
  };

  const handleClose = () => {
    onClose?.();
  };

  const hasHeader = title || collapsible || closable;

  return (
    <AnimatePresence>
      {visible && (
        <motion.div
          variants={panelVariants}
          initial="hidden"
          animate="visible"
          exit="exit"
          className={cn(
            // Base glass styling
            'bg-glass-bg backdrop-blur-xl',
            'border border-glass-border',
            'rounded-xl',
            'shadow-[var(--glass-shadow)]',
            // Positioning
            position !== 'custom' && 'absolute',
            position !== 'custom' && positionClasses[position],
            // Overflow handling
            'overflow-hidden',
            className
          )}
          style={{
            width: width,
            maxHeight: maxHeight,
            ...style,
          }}
        >
          {/* Header */}
          {hasHeader && (
            <div
              className={cn(
                'flex items-center justify-between',
                'px-4 py-3',
                'border-b border-glass-border',
                'bg-glass-highlight',
                headerClassName
              )}
            >
              <div className="flex items-center gap-2">
                {icon && (
                  <span className="text-muted-foreground">{icon}</span>
                )}
                {title && (
                  <h3 className="text-sm font-medium text-foreground tracking-tight">
                    {title}
                  </h3>
                )}
              </div>

              <div className="flex items-center gap-1">
                {headerRight}
                {collapsible && (
                  <button
                    onClick={handleCollapse}
                    className={cn(
                      'p-1.5 rounded-md',
                      'text-muted-foreground hover:text-foreground',
                      'hover:bg-surface-3 transition-colors',
                      'min-w-[var(--touch-min)] min-h-[var(--touch-min)]',
                      'flex items-center justify-center'
                    )}
                    aria-label={isCollapsed ? 'Expand panel' : 'Collapse panel'}
                  >
                    {isCollapsed ? (
                      <Maximize2 className="w-4 h-4" />
                    ) : (
                      <Minus className="w-4 h-4" />
                    )}
                  </button>
                )}
                {closable && (
                  <button
                    onClick={handleClose}
                    className={cn(
                      'p-1.5 rounded-md',
                      'text-muted-foreground hover:text-foreground',
                      'hover:bg-surface-3 transition-colors',
                      'min-w-[var(--touch-min)] min-h-[var(--touch-min)]',
                      'flex items-center justify-center'
                    )}
                    aria-label="Close panel"
                  >
                    <X className="w-4 h-4" />
                  </button>
                )}
              </div>
            </div>
          )}

          {/* Content */}
          <motion.div
            initial={false}
            animate={{
              height: isCollapsed ? 0 : 'auto',
              opacity: isCollapsed ? 0 : 1,
            }}
            transition={transitions.spring}
            className={cn('overflow-hidden', contentClassName)}
          >
            <div className="p-4">{children}</div>
          </motion.div>
        </motion.div>
      )}
    </AnimatePresence>
  );
}

// Floating Panel Layer - container for positioned panels
interface FloatingPanelLayerProps {
  children: React.ReactNode;
  className?: string;
}

export function FloatingPanelLayer({
  children,
  className,
}: FloatingPanelLayerProps) {
  return (
    <div
      className={cn(
        'absolute inset-0 pointer-events-none z-30',
        '[&>*]:pointer-events-auto',
        className
      )}
    >
      {children}
    </div>
  );
}

// Compact variant for small floating indicators
interface GlassIndicatorProps {
  children: React.ReactNode;
  className?: string;
  position?: GlassPanelPosition;
}

export function GlassIndicator({
  children,
  className,
  position = 'custom',
}: GlassIndicatorProps) {
  return (
    <motion.div
      variants={panelVariants}
      initial="hidden"
      animate="visible"
      exit="exit"
      className={cn(
        'bg-glass-bg backdrop-blur-xl',
        'border border-glass-border',
        'rounded-lg',
        'px-3 py-2',
        'shadow-[var(--glass-shadow)]',
        position !== 'custom' && 'absolute',
        position !== 'custom' && positionClasses[position],
        className
      )}
    >
      {children}
    </motion.div>
  );
}
