'use client';

/**
 * Sonner Toast Component
 * shadcn-compatible wrapper for sonner toasts
 * 
 * Per spec Section 9.1: AlertStack uses Toast (modified)
 */

import { Toaster as Sonner, toast } from 'sonner';
import { cn } from '@/lib/utils';

type ToasterProps = React.ComponentProps<typeof Sonner>;

function Toaster({ ...props }: ToasterProps) {
  return (
    <Sonner
      theme="dark"
      className="toaster group"
      position="top-right"
      toastOptions={{
        classNames: {
          toast: cn(
            'group toast group-[.toaster]:bg-[var(--surface-1)] group-[.toaster]:text-foreground',
            'group-[.toaster]:border-white/10 group-[.toaster]:shadow-lg',
            'group-[.toaster]:backdrop-blur-xl'
          ),
          title: 'group-[.toast]:text-foreground group-[.toast]:font-semibold',
          description: 'group-[.toast]:text-muted-foreground',
          actionButton:
            'group-[.toast]:bg-primary group-[.toast]:text-primary-foreground',
          cancelButton:
            'group-[.toast]:bg-muted group-[.toast]:text-muted-foreground',
          closeButton:
            'group-[.toast]:bg-transparent group-[.toast]:text-muted-foreground group-[.toast]:hover:text-foreground',
          error: 'group-[.toaster]:border-l-4 group-[.toaster]:border-l-[var(--danger)] group-[.toaster]:bg-[var(--danger)]/10',
          warning: 'group-[.toaster]:border-l-4 group-[.toaster]:border-l-[var(--warning)] group-[.toaster]:bg-[var(--warning)]/10',
          success: 'group-[.toaster]:border-l-4 group-[.toaster]:border-l-[var(--success)] group-[.toaster]:bg-[var(--success)]/10',
          info: 'group-[.toaster]:border-l-4 group-[.toaster]:border-l-[var(--info)] group-[.toaster]:bg-[var(--info)]/10',
        },
      }}
      {...props}
    />
  );
}

export { Toaster, toast };
