'use client';

/**
 * AERIS GCS Haptic Feedback
 * 
 * Trigger haptic feedback on supported devices (primarily iPad/iOS)
 * Falls back gracefully on unsupported devices
 */

type HapticStyle = 'light' | 'medium' | 'heavy' | 'selection' | 'success' | 'warning' | 'error';

/**
 * Trigger haptic feedback if available
 */
export function haptic(style: HapticStyle = 'light'): void {
  // Check if we're in a browser environment
  if (typeof window === 'undefined') return;

  // Try Web Vibration API first (Android, some browsers)
  if ('vibrate' in navigator) {
    const patterns: Record<HapticStyle, number | number[]> = {
      light: 10,
      medium: 20,
      heavy: 30,
      selection: 5,
      success: [10, 50, 10],
      warning: [20, 50, 20],
      error: [30, 50, 30, 50, 30],
    };

    try {
      navigator.vibrate(patterns[style]);
    } catch {
      // Vibration API might throw in some contexts
    }
  }

  // For iOS/Safari, we rely on CSS :active states and 
  // the native haptic feedback from buttons
  // The Taptic Engine API isn't exposed to web, but we can 
  // trigger haptics through proper button interactions
}

/**
 * Component wrapper that adds haptic feedback on press
 */
export function withHaptic<P extends object>(
  WrappedComponent: React.ComponentType<P>,
  style: HapticStyle = 'light'
) {
  return function HapticWrapper(props: P) {
    const handlePointerDown = () => {
      haptic(style);
    };

    // @ts-expect-error - Adding pointer event handler
    return <WrappedComponent {...props} onPointerDown={handlePointerDown} />;
  };
}

/**
 * Hook for imperative haptic feedback
 */
export function useHaptic() {
  return {
    light: () => haptic('light'),
    medium: () => haptic('medium'),
    heavy: () => haptic('heavy'),
    selection: () => haptic('selection'),
    success: () => haptic('success'),
    warning: () => haptic('warning'),
    error: () => haptic('error'),
  };
}

export default haptic;
