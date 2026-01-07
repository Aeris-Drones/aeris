'use client';

/**
 * AlertPanel - Dropdown panel showing all active alerts
 * 
 * Per spec Section 3.4 Alert Overlay:
 * Shows when bell icon is clicked in StatusPill
 */

import { useEffect, useRef } from 'react';
import { Card } from '@/components/ui/card';
import { Button } from '@/components/ui/button';
import { Badge } from '@/components/ui/badge';
import {
  X,
  AlertTriangle,
  AlertCircle,
  Info,
  MapPin,
  XCircle,
  BellOff,
} from 'lucide-react';
import { cn } from '@/lib/utils';
import { motion, AnimatePresence } from 'framer-motion';

export type AlertSeverity = 'critical' | 'warning' | 'info';

export interface Alert {
  id: string;
  severity: AlertSeverity;
  title: string;
  description?: string;
  action?: {
    label: string;
    onClick: () => void;
  };
  timestamp: Date;
  dismissible: boolean;
}

interface AlertPanelProps {
  alerts: Alert[];
  isOpen: boolean;
  onClose: () => void;
  onDismiss: (id: string) => void;
  onDismissAll: () => void;
}

const severityConfig: Record<AlertSeverity, {
  Icon: typeof AlertCircle;
  borderColor: string;
  bgColor: string;
  iconColor: string;
  badgeVariant: 'danger' | 'warning' | 'info';
}> = {
  critical: {
    Icon: AlertCircle,
    borderColor: 'border-l-[var(--danger)]',
    bgColor: 'bg-[var(--danger)]/10',
    iconColor: 'text-[var(--danger)]',
    badgeVariant: 'danger',
  },
  warning: {
    Icon: AlertTriangle,
    borderColor: 'border-l-[var(--warning)]',
    bgColor: 'bg-[var(--warning)]/10',
    iconColor: 'text-[var(--warning)]',
    badgeVariant: 'warning',
  },
  info: {
    Icon: Info,
    borderColor: 'border-l-[var(--info)]',
    bgColor: 'bg-[var(--info)]/10',
    iconColor: 'text-[var(--info)]',
    badgeVariant: 'info',
  },
};

function formatTimestamp(date: Date): string {
  const now = new Date();
  const diffMs = now.getTime() - date.getTime();
  const diffSec = Math.floor(diffMs / 1000);

  if (diffSec < 5) return 'Just now';
  if (diffSec < 60) return `${diffSec}s ago`;
  if (diffSec < 3600) return `${Math.floor(diffSec / 60)}m ago`;
  return `${Math.floor(diffSec / 3600)}h ago`;
}

function AlertItem({ 
  alert, 
  onDismiss 
}: { 
  alert: Alert; 
  onDismiss: (id: string) => void;
}) {
  const config = severityConfig[alert.severity];
  const { Icon } = config;

  return (
    <motion.div
      initial={{ opacity: 0, x: 20 }}
      animate={{ opacity: 1, x: 0 }}
      exit={{ opacity: 0, x: -20 }}
      className={cn(
        'relative flex flex-col gap-2 border-l-4 rounded-lg p-3 transition-all',
        config.borderColor,
        config.bgColor
      )}
    >
      {/* Header row */}
      <div className="flex items-start justify-between gap-3">
        <div className="flex items-start gap-3">
          <Icon className={cn('mt-0.5 h-4 w-4 flex-shrink-0', config.iconColor)} />
          <div className="flex flex-col gap-1">
            <div className="flex items-center gap-2">
              <Badge variant={config.badgeVariant} className="text-[10px] uppercase px-1.5 py-0">
                {alert.severity}
              </Badge>
              <span className="text-[10px] text-white/40">
                {formatTimestamp(alert.timestamp)}
              </span>
            </div>
            <span className="text-sm font-medium text-white">
              {alert.title}
            </span>
          </div>
        </div>

        {alert.dismissible && (
          <button
            onClick={() => onDismiss(alert.id)}
            className="flex-shrink-0 rounded p-1 text-white/40 transition-colors hover:bg-white/10 hover:text-white"
            aria-label="Dismiss alert"
          >
            <X className="h-3.5 w-3.5" />
          </button>
        )}
      </div>

      {/* Description */}
      {alert.description && (
        <p className="ml-7 text-xs text-white/60">
          {alert.description}
        </p>
      )}

      {/* Action button */}
      {alert.action && (
        <div className="ml-7">
          <Button
            size="sm"
            variant="outline"
            className="h-6 px-2 text-[10px]"
            onClick={alert.action.onClick}
          >
            {alert.action.label === 'LOCATE' && <MapPin className="mr-1 h-3 w-3" />}
            {alert.action.label}
          </Button>
        </div>
      )}
    </motion.div>
  );
}

export function AlertPanel({
  alerts,
  isOpen,
  onClose,
  onDismiss,
  onDismissAll,
}: AlertPanelProps) {
  const panelRef = useRef<HTMLDivElement>(null);

  // Close on click outside
  useEffect(() => {
    if (!isOpen) return;

    const handleClickOutside = (e: MouseEvent) => {
      if (panelRef.current && !panelRef.current.contains(e.target as Node)) {
        onClose();
      }
    };

    // Delay adding listener to avoid immediate close
    const timer = setTimeout(() => {
      document.addEventListener('mousedown', handleClickOutside);
    }, 100);

    return () => {
      clearTimeout(timer);
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, [isOpen, onClose]);

  // Close on escape
  useEffect(() => {
    if (!isOpen) return;

    const handleKeyDown = (e: KeyboardEvent) => {
      if (e.key === 'Escape') {
        onClose();
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [isOpen, onClose]);

  const dismissibleAlerts = alerts.filter(a => a.dismissible);
  const criticalCount = alerts.filter(a => a.severity === 'critical').length;

  return (
    <AnimatePresence>
      {isOpen && (
        <motion.div
          ref={panelRef}
          initial={{ opacity: 0, y: -10, scale: 0.95 }}
          animate={{ opacity: 1, y: 0, scale: 1 }}
          exit={{ opacity: 0, y: -10, scale: 0.95 }}
          transition={{ duration: 0.2 }}
          className={cn(
            'absolute right-0 top-full mt-2 z-50',
            'w-[380px] max-h-[60vh] overflow-hidden',
            'rounded-xl border border-white/10',
            'bg-[var(--surface-1)]/95 backdrop-blur-xl',
            'shadow-2xl shadow-black/30'
          )}
        >
          {/* Header */}
          <div className="flex items-center justify-between border-b border-white/10 px-4 py-3">
            <div className="flex items-center gap-2">
              <AlertTriangle className={cn(
                'h-4 w-4',
                criticalCount > 0 ? 'text-[var(--danger)]' : 'text-[var(--warning)]'
              )} />
              <span className="text-sm font-semibold text-white">
                Alerts ({alerts.length})
              </span>
            </div>
            <button
              onClick={onClose}
              className="rounded p-1 text-white/40 transition-colors hover:bg-white/10 hover:text-white"
            >
              <X className="h-4 w-4" />
            </button>
          </div>

          {/* Alert list */}
          <div className="max-h-[calc(60vh-100px)] overflow-y-auto p-3 space-y-2">
            {alerts.length === 0 ? (
              <div className="flex flex-col items-center justify-center py-8 text-white/40">
                <BellOff className="h-8 w-8 mb-2" />
                <span className="text-sm">No active alerts</span>
              </div>
            ) : (
              <AnimatePresence mode="popLayout">
                {alerts.map((alert) => (
                  <AlertItem
                    key={alert.id}
                    alert={alert}
                    onDismiss={onDismiss}
                  />
                ))}
              </AnimatePresence>
            )}
          </div>

          {/* Footer */}
          {dismissibleAlerts.length > 1 && (
            <div className="border-t border-white/10 px-4 py-2">
              <Button
                size="sm"
                variant="ghost"
                className="w-full h-8 text-xs text-white/50 hover:text-white"
                onClick={onDismissAll}
              >
                <XCircle className="mr-1.5 h-3.5 w-3.5" />
                Dismiss All
              </Button>
            </div>
          )}
        </motion.div>
      )}
    </AnimatePresence>
  );
}
