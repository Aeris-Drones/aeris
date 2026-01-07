'use client';

/**
 * AERIS GCS Alert System
 * 
 * Per spec Section 4.6 & 9.1:
 * - Uses shadcn Toast (sonner) as base
 * - Stacks from top, max 5 visible
 * - Critical alerts cannot be auto-dismissed
 * - Warning alerts auto-dismiss after 30s
 * - Info alerts auto-dismiss after 10s
 * - Audio cue for critical (optional)
 * 
 * Alert Overlay per spec Section 3.4:
 * ┌─ ALERT STACK ─────────────────────────────────────────────────────────┐
 * │  🔴 CRITICAL: Scout-2 COMMS LOST - Last seen 45s ago      [LOCATE]   │
 * │  ⚠️ WARNING: Scout-3 battery 8% - Auto RTH in 30s          [ABORT]   │
 * │  ⚠️ WARNING: Gas concentration rising in Sector D      [VIEW MAP]    │
 * │                                            [DISMISS ALL] [MINIMIZE]   │
 * └───────────────────────────────────────────────────────────────────────┘
 */

import { useEffect, useCallback, useRef } from 'react';
import { toast, Toaster } from 'sonner';
import { AlertCircle, AlertTriangle, Info, MapPin, X } from 'lucide-react';
import { cn } from '@/lib/utils';

// ============================================================================
// Types
// ============================================================================

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

// ============================================================================
// Configuration per spec
// ============================================================================

// All toasts auto-dismiss after 10 seconds - they're just notifications
// The actual alerts persist in the AlertPanel
const TOAST_DURATION = 10000; // 10 seconds for all toasts

const SEVERITY_ICONS: Record<AlertSeverity, React.ReactNode> = {
  critical: <AlertCircle className="h-5 w-5 text-[var(--danger)]" />,
  warning: <AlertTriangle className="h-5 w-5 text-[var(--warning)]" />,
  info: <Info className="h-5 w-5 text-[var(--info)]" />,
};

// ============================================================================
// Audio cue for critical alerts
// ============================================================================

function playCriticalAlertSound() {
  if (typeof window === 'undefined') return;

  try {
    const AudioContextClass = window.AudioContext || (window as unknown as { webkitAudioContext: typeof AudioContext }).webkitAudioContext;
    if (!AudioContextClass) return;

    const audioContext = new AudioContextClass();
    
    // Two-beep pattern for critical alerts
    [0, 0.15].forEach((delay) => {
      const oscillator = audioContext.createOscillator();
      const gainNode = audioContext.createGain();

      oscillator.connect(gainNode);
      gainNode.connect(audioContext.destination);

      oscillator.frequency.value = 880; // A5 note
      oscillator.type = 'sine';

      const startTime = audioContext.currentTime + delay;
      gainNode.gain.setValueAtTime(0.3, startTime);
      gainNode.gain.exponentialRampToValueAtTime(0.01, startTime + 0.2);

      oscillator.start(startTime);
      oscillator.stop(startTime + 0.2);
    });
  } catch {
    // Audio not supported or blocked
  }
}

// ============================================================================
// Alert Toast Function
// ============================================================================

export function showAlert(alert: Omit<Alert, 'timestamp'>, options?: { playSound?: boolean }) {
  const { id, severity, title, description, action } = alert;
  const { playSound = true } = options || {};

  // Play sound for critical alerts (only on new alerts, not re-shows)
  if (playSound && severity === 'critical') {
    playCriticalAlertSound();
  }

  // Map severity to sonner type
  const toastFn = severity === 'critical' ? toast.error : 
                  severity === 'warning' ? toast.warning : 
                  toast.info;

  // All toasts auto-dismiss after 10s - they're just notifications
  // The actual alerts persist in the AlertPanel (accessed via bell icon)
  toastFn(title, {
    id,
    description,
    duration: TOAST_DURATION,
    dismissible: true, // User can always dismiss the toast (alert stays in panel)
    icon: SEVERITY_ICONS[severity],
    action: action ? {
      label: action.label,
      onClick: action.onClick,
    } : undefined,
    classNames: {
      toast: cn(
        'group-[.toaster]:border-l-4',
        severity === 'critical' && 'group-[.toaster]:border-l-[var(--danger)] group-[.toaster]:bg-[var(--danger)]/10',
        severity === 'warning' && 'group-[.toaster]:border-l-[var(--warning)] group-[.toaster]:bg-[var(--warning)]/10',
        severity === 'info' && 'group-[.toaster]:border-l-[var(--info)] group-[.toaster]:bg-[var(--info)]/10',
      ),
      title: 'font-semibold',
      description: 'text-white/60',
      actionButton: cn(
        'px-3 py-1.5 text-xs font-medium rounded',
        severity === 'critical' && 'bg-[var(--danger)]/20 text-[var(--danger)] hover:bg-[var(--danger)]/30',
        severity === 'warning' && 'bg-[var(--warning)]/20 text-[var(--warning)] hover:bg-[var(--warning)]/30',
        severity === 'info' && 'bg-[var(--info)]/20 text-[var(--info)] hover:bg-[var(--info)]/30',
      ),
    },
  });

  return id;
}

export function dismissAlert(id: string) {
  toast.dismiss(id);
}

export function dismissAllAlerts() {
  toast.dismiss();
}

// ============================================================================
// AlertToaster - The toast container component
// ============================================================================

interface AlertToasterProps {
  /** Maximum visible toasts */
  visibleToasts?: number;
  /** Enable audio cues */
  enableAudio?: boolean;
}

export function AlertToaster({ 
  visibleToasts = 5,
  enableAudio = true,
}: AlertToasterProps) {
  return (
    <Toaster
      theme="dark"
      position="top-right"
      visibleToasts={visibleToasts}
      closeButton
      richColors
      toastOptions={{
        classNames: {
          toast: cn(
            'group toast',
            'bg-[var(--surface-1)]/95 backdrop-blur-xl',
            'border border-white/10',
            'shadow-lg shadow-black/20',
            'rounded-xl',
            'min-w-[360px] max-w-[420px]',
          ),
          title: 'text-sm font-semibold text-white',
          description: 'text-xs text-white/60 mt-1',
          actionButton: 'text-xs font-medium px-3 py-1.5 rounded',
          cancelButton: 'text-xs font-medium px-3 py-1.5 rounded bg-white/5 hover:bg-white/10',
          closeButton: 'text-white/40 hover:text-white bg-transparent hover:bg-white/10',
        },
      }}
    />
  );
}

// ============================================================================
// useAlerts Hook - for managing alert state
// ============================================================================

interface UseAlertsOptions {
  enableAudio?: boolean;
}

export function useAlerts(options: UseAlertsOptions = {}) {
  const { enableAudio = true } = options;
  const alertsRef = useRef<Map<string, Alert>>(new Map());

  const addAlert = useCallback((alert: Omit<Alert, 'id' | 'timestamp'> & { id?: string }) => {
    const id = alert.id || `alert-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
    const fullAlert: Alert = {
      ...alert,
      id,
      timestamp: new Date(),
    };

    alertsRef.current.set(id, fullAlert);

    // Play sound for critical alerts
    if (enableAudio && alert.severity === 'critical') {
      playCriticalAlertSound();
    }

    showAlert(fullAlert);
    return id;
  }, [enableAudio]);

  const dismiss = useCallback((id: string) => {
    alertsRef.current.delete(id);
    dismissAlert(id);
  }, []);

  const dismissAll = useCallback(() => {
    alertsRef.current.clear();
    dismissAllAlerts();
  }, []);

  return {
    addAlert,
    dismiss,
    dismissAll,
    getAlerts: () => Array.from(alertsRef.current.values()),
  };
}

// ============================================================================
// Re-export for convenience
// ============================================================================

export { toast, Toaster } from 'sonner';
