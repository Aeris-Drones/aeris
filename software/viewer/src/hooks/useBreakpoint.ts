'use client';

import { useState, useEffect, useCallback } from 'react';
import { breakpoints } from '@/lib/design-tokens';

export type BreakpointKey = keyof typeof breakpoints;

export type DeviceType = 'mobile' | 'tablet' | 'desktop';

interface BreakpointState {
  /** Current breakpoint name */
  current: BreakpointKey;
  /** Is screen width >= sm (640px) */
  isSm: boolean;
  /** Is screen width >= md (768px) */
  isMd: boolean;
  /** Is screen width >= lg (1024px) */
  isLg: boolean;
  /** Is screen width >= xl (1280px) */
  isXl: boolean;
  /** Is screen width >= 2xl (1536px) */
  is2xl: boolean;
  /** Device category based on breakpoint */
  device: DeviceType;
  /** Current viewport width */
  width: number;
  /** Current viewport height */
  height: number;
  /** Is touch device (based on pointer query) */
  isTouch: boolean;
  /** Is portrait orientation */
  isPortrait: boolean;
  /** Is landscape orientation */
  isLandscape: boolean;
}

function getDeviceType(width: number): DeviceType {
  if (width < breakpoints.md) return 'mobile';
  if (width < breakpoints.xl) return 'tablet';
  return 'desktop';
}

function getCurrentBreakpoint(width: number): BreakpointKey {
  if (width >= breakpoints['2xl']) return '2xl';
  if (width >= breakpoints.xl) return 'xl';
  if (width >= breakpoints.lg) return 'lg';
  if (width >= breakpoints.md) return 'md';
  return 'sm';
}

function getBreakpointState(): BreakpointState {
  // SSR-safe defaults
  if (typeof window === 'undefined') {
    return {
      current: 'xl',
      isSm: true,
      isMd: true,
      isLg: true,
      isXl: true,
      is2xl: false,
      device: 'desktop',
      width: 1280,
      height: 720,
      isTouch: false,
      isPortrait: false,
      isLandscape: true,
    };
  }

  const width = window.innerWidth;
  const height = window.innerHeight;
  const isTouch = window.matchMedia('(pointer: coarse)').matches;

  return {
    current: getCurrentBreakpoint(width),
    isSm: width >= breakpoints.sm,
    isMd: width >= breakpoints.md,
    isLg: width >= breakpoints.lg,
    isXl: width >= breakpoints.xl,
    is2xl: width >= breakpoints['2xl'],
    device: getDeviceType(width),
    width,
    height,
    isTouch,
    isPortrait: height > width,
    isLandscape: width >= height,
  };
}

/**
 * Hook for responsive breakpoint detection
 * Returns current breakpoint state with convenient boolean flags
 */
export function useBreakpoint(): BreakpointState {
  const [state, setState] = useState<BreakpointState>(getBreakpointState);

  useEffect(() => {
    // Update immediately on mount (handles SSR hydration)
    setState(getBreakpointState());

    const handleResize = () => {
      setState(getBreakpointState());
    };

    // Use ResizeObserver for more efficient updates
    const resizeObserver = new ResizeObserver(handleResize);
    resizeObserver.observe(document.documentElement);

    // Also listen to orientation changes
    window.addEventListener('orientationchange', handleResize);

    return () => {
      resizeObserver.disconnect();
      window.removeEventListener('orientationchange', handleResize);
    };
  }, []);

  return state;
}

/**
 * Hook that returns true if viewport is at or above the specified breakpoint
 */
export function useMediaQuery(query: string): boolean {
  const [matches, setMatches] = useState(false);

  useEffect(() => {
    const mediaQuery = window.matchMedia(query);
    setMatches(mediaQuery.matches);

    const handler = (event: MediaQueryListEvent) => {
      setMatches(event.matches);
    };

    mediaQuery.addEventListener('change', handler);
    return () => mediaQuery.removeEventListener('change', handler);
  }, [query]);

  return matches;
}

/**
 * Hook that returns true if viewport width is at or above the breakpoint
 */
export function useMinWidth(breakpoint: BreakpointKey): boolean {
  return useMediaQuery(`(min-width: ${breakpoints[breakpoint]}px)`);
}

/**
 * Hook that returns true if viewport width is below the breakpoint
 */
export function useMaxWidth(breakpoint: BreakpointKey): boolean {
  return useMediaQuery(`(max-width: ${breakpoints[breakpoint] - 1}px)`);
}

/**
 * Hook that returns true if device has touch input
 */
export function useIsTouch(): boolean {
  return useMediaQuery('(pointer: coarse)');
}

/**
 * Hook for detecting iPad Pro specifically (useful for optimizing layouts)
 */
export function useIsIPadPro(): boolean {
  const { width, height, isTouch } = useBreakpoint();

  // iPad Pro 12.9" is 1024x1366 in portrait, 1366x1024 in landscape
  // iPad Pro 11" is 834x1194 in portrait
  const isIPadSize =
    (width >= 768 && width <= 1366) ||
    (height >= 768 && height <= 1366);

  return isTouch && isIPadSize;
}

/**
 * Custom hook for layout-specific breakpoints used in GCS Dashboard
 */
export function useGCSLayout() {
  const bp = useBreakpoint();

  return {
    ...bp,
    // Layout-specific flags
    showSidebar: bp.isXl,
    showMobileBottomSheet: !bp.isXl,
    useCompactStatusBar: !bp.isLg,
    floatingPanelsPosition: bp.isXl ? 'standard' : 'mobile' as const,
  };
}
