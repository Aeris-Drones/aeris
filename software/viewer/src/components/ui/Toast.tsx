'use client';

/**
 * AERIS GCS Toast Notification System
 * 
 * Premium toast notifications with Sonner-like API
 * Glass-morphism design, action buttons, auto-dismiss
 */

import React, { createContext, useContext, useCallback, useReducer, useRef } from 'react';
import { motion, AnimatePresence } from 'framer-motion';
import { cn } from '@/lib/utils';
import { transitions } from '@/lib/animations';
import {
  CheckCircle2,
  AlertCircle,
  AlertTriangle,
  Info,
  X,
  Loader2,
} from 'lucide-react';

// ============================================================================
// Types
// ============================================================================

export type ToastType = 'success' | 'error' | 'warning' | 'info' | 'loading';

export interface ToastAction {
  label: string;
  onClick: () => void;
  variant?: 'default' | 'destructive';
}

export interface Toast {
  id: string;
  type: ToastType;
  title: string;
  description?: string;
  action?: ToastAction;
  duration?: number;
  dismissible?: boolean;
  createdAt: number;
}

interface ToastState {
  toasts: Toast[];
}

type ToastAction_Type =
  | { type: 'ADD_TOAST'; toast: Toast }
  | { type: 'REMOVE_TOAST'; id: string }
  | { type: 'UPDATE_TOAST'; id: string; updates: Partial<Toast> };

// ============================================================================
// Configuration
// ============================================================================

const TOAST_CONFIG = {
  maxToasts: 5,
  defaultDuration: 5000,
  loadingDuration: Infinity,
};

const TOAST_ICONS: Record<ToastType, React.ReactNode> = {
  success: <CheckCircle2 className="w-5 h-5" />,
  error: <AlertCircle className="w-5 h-5" />,
  warning: <AlertTriangle className="w-5 h-5" />,
  info: <Info className="w-5 h-5" />,
  loading: <Loader2 className="w-5 h-5 animate-spin" />,
};

const TOAST_STYLES: Record<ToastType, string> = {
  success: 'text-success border-success/30 bg-success/5',
  error: 'text-danger border-danger/30 bg-danger/5',
  warning: 'text-warning border-warning/30 bg-warning/5',
  info: 'text-info border-info/30 bg-info/5',
  loading: 'text-muted-foreground border-muted-foreground/30 bg-muted-foreground/5',
};

// ============================================================================
// Context & Reducer
// ============================================================================

function toastReducer(state: ToastState, action: ToastAction_Type): ToastState {
  switch (action.type) {
    case 'ADD_TOAST':
      return {
        toasts: [action.toast, ...state.toasts].slice(0, TOAST_CONFIG.maxToasts),
      };
    case 'REMOVE_TOAST':
      return {
        toasts: state.toasts.filter((t) => t.id !== action.id),
      };
    case 'UPDATE_TOAST':
      return {
        toasts: state.toasts.map((t) =>
          t.id === action.id ? { ...t, ...action.updates } : t
        ),
      };
    default:
      return state;
  }
}

interface ToastContextValue {
  toasts: Toast[];
  toast: (options: Omit<Toast, 'id' | 'createdAt'>) => string;
  success: (title: string, description?: string) => string;
  error: (title: string, description?: string) => string;
  warning: (title: string, description?: string) => string;
  info: (title: string, description?: string) => string;
  loading: (title: string, description?: string) => string;
  dismiss: (id: string) => void;
  update: (id: string, updates: Partial<Toast>) => void;
  promise: <T>(
    promise: Promise<T>,
    options: {
      loading: string;
      success: string | ((data: T) => string);
      error: string | ((err: Error) => string);
    }
  ) => Promise<T>;
}

const ToastContext = createContext<ToastContextValue | null>(null);

// ============================================================================
// Provider
// ============================================================================

export function ToastProvider({ children }: { children: React.ReactNode }) {
  const [state, dispatch] = useReducer(toastReducer, { toasts: [] });
  const timersRef = useRef<Map<string, NodeJS.Timeout>>(new Map());

  const dismiss = useCallback((id: string) => {
    const timer = timersRef.current.get(id);
    if (timer) {
      clearTimeout(timer);
      timersRef.current.delete(id);
    }
    dispatch({ type: 'REMOVE_TOAST', id });
  }, []);

  const scheduleRemoval = useCallback(
    (id: string, duration: number) => {
      if (duration === Infinity) return;

      const timer = setTimeout(() => {
        dismiss(id);
      }, duration);

      timersRef.current.set(id, timer);
    },
    [dismiss]
  );

  const toast = useCallback(
    (options: Omit<Toast, 'id' | 'createdAt'>) => {
      const id = crypto.randomUUID();
      const duration =
        options.duration ??
        (options.type === 'loading'
          ? TOAST_CONFIG.loadingDuration
          : TOAST_CONFIG.defaultDuration);

      const newToast: Toast = {
        ...options,
        id,
        createdAt: Date.now(),
        dismissible: options.dismissible ?? true,
      };

      dispatch({ type: 'ADD_TOAST', toast: newToast });
      scheduleRemoval(id, duration);

      return id;
    },
    [scheduleRemoval]
  );

  const update = useCallback(
    (id: string, updates: Partial<Toast>) => {
      dispatch({ type: 'UPDATE_TOAST', id, updates });

      // Reset timer if duration changed
      if (updates.type && updates.type !== 'loading') {
        const timer = timersRef.current.get(id);
        if (timer) {
          clearTimeout(timer);
        }
        scheduleRemoval(id, updates.duration ?? TOAST_CONFIG.defaultDuration);
      }
    },
    [scheduleRemoval]
  );

  const success = useCallback(
    (title: string, description?: string) =>
      toast({ type: 'success', title, description }),
    [toast]
  );

  const error = useCallback(
    (title: string, description?: string) =>
      toast({ type: 'error', title, description }),
    [toast]
  );

  const warning = useCallback(
    (title: string, description?: string) =>
      toast({ type: 'warning', title, description }),
    [toast]
  );

  const info = useCallback(
    (title: string, description?: string) =>
      toast({ type: 'info', title, description }),
    [toast]
  );

  const loading = useCallback(
    (title: string, description?: string) =>
      toast({ type: 'loading', title, description, dismissible: false }),
    [toast]
  );

  const promiseHandler = useCallback(
    async <T,>(
      promise: Promise<T>,
      options: {
        loading: string;
        success: string | ((data: T) => string);
        error: string | ((err: Error) => string);
      }
    ): Promise<T> => {
      const id = loading(options.loading);

      try {
        const data = await promise;
        update(id, {
          type: 'success',
          title:
            typeof options.success === 'function'
              ? options.success(data)
              : options.success,
          dismissible: true,
        });
        return data;
      } catch (err) {
        update(id, {
          type: 'error',
          title:
            typeof options.error === 'function'
              ? options.error(err as Error)
              : options.error,
          dismissible: true,
        });
        throw err;
      }
    },
    [loading, update]
  );

  return (
    <ToastContext.Provider
      value={{
        toasts: state.toasts,
        toast,
        success,
        error,
        warning,
        info,
        loading,
        dismiss,
        update,
        promise: promiseHandler,
      }}
    >
      {children}
      <ToastViewport />
    </ToastContext.Provider>
  );
}

// ============================================================================
// Hook
// ============================================================================

export function useToast() {
  const context = useContext(ToastContext);
  if (!context) {
    throw new Error('useToast must be used within a ToastProvider');
  }
  return context;
}

// ============================================================================
// Toast Component
// ============================================================================

function ToastItem({ toast }: { toast: Toast }) {
  const { dismiss } = useToast();

  return (
    <motion.div
      layout
      initial={{ opacity: 0, y: -20, scale: 0.95 }}
      animate={{ opacity: 1, y: 0, scale: 1 }}
      exit={{ opacity: 0, x: 100, scale: 0.95 }}
      transition={transitions.spring}
      className={cn(
        'relative flex items-start gap-3 p-4 rounded-xl',
        'bg-glass-bg backdrop-blur-xl',
        'border shadow-lg shadow-black/20',
        'min-w-[320px] max-w-[420px]',
        TOAST_STYLES[toast.type]
      )}
    >
      {/* Icon */}
      <div className="shrink-0">{TOAST_ICONS[toast.type]}</div>

      {/* Content */}
      <div className="flex-1 min-w-0">
        <p className="font-semibold text-sm text-foreground">{toast.title}</p>
        {toast.description && (
          <p className="text-xs text-muted-foreground mt-1">{toast.description}</p>
        )}
        
        {/* Action button */}
        {toast.action && (
          <button
            onClick={() => {
              toast.action!.onClick();
              dismiss(toast.id);
            }}
            className={cn(
              'mt-2 text-xs font-medium px-2 py-1 rounded',
              'transition-colors min-h-[var(--touch-min)]',
              toast.action.variant === 'destructive'
                ? 'bg-danger/10 text-danger hover:bg-danger/20'
                : 'bg-info/10 text-info hover:bg-info/20'
            )}
          >
            {toast.action.label}
          </button>
        )}
      </div>

      {/* Dismiss button */}
      {toast.dismissible && (
        <button
          onClick={() => dismiss(toast.id)}
          className={cn(
            'shrink-0 p-1 rounded-md',
            'text-muted-foreground hover:text-foreground',
            'hover:bg-surface-3 transition-colors'
          )}
        >
          <X className="w-4 h-4" />
        </button>
      )}

      {/* Progress bar for auto-dismiss */}
      {toast.type !== 'loading' && toast.dismissible && (
        <motion.div
          className="absolute bottom-0 left-0 h-0.5 bg-current opacity-30 rounded-full"
          initial={{ width: '100%' }}
          animate={{ width: '0%' }}
          transition={{ duration: (toast.duration ?? TOAST_CONFIG.defaultDuration) / 1000, ease: 'linear' }}
        />
      )}
    </motion.div>
  );
}

// ============================================================================
// Viewport
// ============================================================================

function ToastViewport() {
  const { toasts } = useToast();

  return (
    <div className="fixed top-4 right-4 z-[9999] flex flex-col gap-2 pointer-events-none">
      <AnimatePresence mode="popLayout">
        {toasts.map((toast) => (
          <div key={toast.id} className="pointer-events-auto">
            <ToastItem toast={toast} />
          </div>
        ))}
      </AnimatePresence>
    </div>
  );
}

export default ToastProvider;
