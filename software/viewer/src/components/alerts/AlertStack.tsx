'use client';

import { useCallback, useRef } from 'react';
import { toast, Toaster } from 'sonner';
import { AlertCircle, AlertTriangle, Info } from 'lucide-react';
import { cn } from '@/lib/utils';

/**
 * Alert severity levels with distinct visual treatments.
 * - critical: Requires immediate action, persists until dismissed
 * - warning: Important but not blocking, auto-dismisses after 30s
 * - info: General notification, auto-dismisses after 10s
 */
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

/**
 * Toast display durations by severity.
 * Critical alerts persist indefinitely (undefined) until user action.
 */
const TOAST_DURATIONS: Record<AlertSeverity, number | undefined> = {
  critical: undefined,
  warning: 30000,
  info: 10000,
};

/**
 * Icon components mapped by severity for consistent visual language.
 */
const SEVERITY_ICONS: Record<AlertSeverity, React.ReactNode> = {
  critical: <AlertCircle className="h-5 w-5 text-[var(--danger)]" />,
  warning: <AlertTriangle className="h-5 w-5 text-[var(--warning)]" />,
  info: <Info className="h-5 w-5 text-[var(--info)]" />,
};

/**
 * Plays an audible alert tone for critical notifications.
 * Uses Web Audio API to generate a dual-tone beep pattern.
 * Falls back silently if audio is not supported or blocked.
 *
 * Accessibility: Audio alerts supplement visual notifications for
 * operators who may not be looking at the screen.
 */
function playCriticalAlertSound() {
  if (typeof window === 'undefined') return;

  try {
    const AudioContextClass = window.AudioContext || (window as unknown as { webkitAudioContext: typeof AudioContext }).webkitAudioContext;
    if (!AudioContextClass) return;

    const audioContext = new AudioContextClass();

    // Dual-tone pattern for distinctiveness
    [0, 0.15].forEach((delay) => {
      const oscillator = audioContext.createOscillator();
      const gainNode = audioContext.createGain();

      oscillator.connect(gainNode);
      gainNode.connect(audioContext.destination);

      oscillator.frequency.value = 880;
      oscillator.type = 'sine';

      const startTime = audioContext.currentTime + delay;
      gainNode.gain.setValueAtTime(0.3, startTime);
      gainNode.gain.exponentialRampToValueAtTime(0.01, startTime + 0.2);

      oscillator.start(startTime);
      oscillator.stop(startTime + 0.2);
    });
  } catch {
    // Audio not supported or blocked - visual alert still functions
  }
}

/**
 * Displays an alert toast notification with severity-based styling.
 *
 * UI/UX Decisions:
 * - Critical alerts use left border accent for immediate attention
 * - Background tints match severity color for visual grouping
 * - Action buttons styled with severity-appropriate hover states
 * - Non-critical alerts are dismissible by default
 *
 * Accessibility:
 * - Icons indicate severity without relying solely on color
 * - Critical alerts play audible tone for attention
 * - Sufficient contrast for all severity levels
 *
 * @param alert - Alert data excluding timestamp (added automatically)
 * @param options - Optional configuration including sound playback
 * @returns The alert ID for programmatic dismissal
 */
export function showAlert(alert: Omit<Alert, 'timestamp'>, options?: { playSound?: boolean }) {
  const { id, severity, title, description, action } = alert;
  const { playSound = true } = options || {};

  if (playSound && severity === 'critical') {
    playCriticalAlertSound();
  }

  const toastFn = severity === 'critical' ? toast.error :
                  severity === 'warning' ? toast.warning :
                  toast.info;

  toastFn(title, {
    id,
    description,
    duration: TOAST_DURATIONS[severity],
    dismissible: severity !== 'critical',
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

/**
 * Dismisses a specific alert by ID.
 */
export function dismissAlert(id: string) {
  toast.dismiss(id);
}

/**
 * Dismisses all active alerts.
 */
export function dismissAllAlerts() {
  toast.dismiss();
}

interface AlertToasterProps {
  visibleToasts?: number;
}

/**
 * AlertToaster component configures the global toast container.
 *
 * UI/UX Decisions:
 * - Positioned top-right to avoid obscuring main content
 * - Limited to 5 visible toasts to prevent screen clutter
 * - Glassmorphism styling (backdrop-blur) for modern appearance
 * - Consistent sizing (min/max width) for predictable layout
 *
 * Accessibility:
 * - Close button on each toast for keyboard dismissal
 * - Rich colors provide semantic meaning
 */
export function AlertToaster({
  visibleToasts = 5,
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

interface UseAlertsOptions {
  enableAudio?: boolean;
}

/**
 * React hook for managing alerts with local state tracking.
 *
 * State Management:
 * - Maintains Map of active alerts for programmatic access
 * - Generates unique IDs using timestamp + random suffix
 * - Provides methods for adding, dismissing, and querying alerts
 *
 * @param options - Configuration including audio enablement
 * @returns Alert management functions and current alert list
 */
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

export { toast, Toaster } from 'sonner';
