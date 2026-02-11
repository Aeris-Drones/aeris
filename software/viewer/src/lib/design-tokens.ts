/**
 * Design tokens mapping CSS custom properties to typed JavaScript constants.
 *
 * All values reference CSS variables defined in the global stylesheet,
 * enabling runtime theming and consistent styling across the application.
 *
 * @example
 * import { surfaces, sensors } from './design-tokens';
 * style={{ background: surfaces[1], borderColor: sensors.thermal }}
 */

/** Surface elevation levels (0-4) for cards, modals, and popovers. */
export const surfaces = {
  0: 'var(--surface-0)',
  1: 'var(--surface-1)',
  2: 'var(--surface-2)',
  3: 'var(--surface-3)',
  4: 'var(--surface-4)',
} as const;

/** Glassmorphism tokens for translucent overlays and floating panels. */
export const glass = {
  bg: 'var(--glass-bg)',
  bgHover: 'var(--glass-bg-hover)',
  border: 'var(--glass-border)',
  borderHover: 'var(--glass-border-hover)',
  highlight: 'var(--glass-highlight)',
  shadow: 'var(--glass-shadow)',
} as const;

/** Confidence level colors for detection classification. Thresholds: >=80% high, >=50% medium, >=20% low. */
export const confidence = {
  high: 'var(--confidence-high)',
  medium: 'var(--confidence-medium)',
  low: 'var(--confidence-low)',
  unverified: 'var(--confidence-unverified)',
} as const;

/** Sensor type colors (thermal, acoustic, gas) with glow variants. */
export const sensors = {
  thermal: 'var(--sensor-thermal)',
  thermalGlow: 'var(--sensor-thermal-glow)',
  acoustic: 'var(--sensor-acoustic)',
  acousticVocal: 'var(--sensor-acoustic-vocal)',
  acousticGlow: 'var(--sensor-acoustic-glow)',
  gas: 'var(--sensor-gas)',
  gasGlow: 'var(--sensor-gas-glow)',
} as const;

/** Priority level colors: 1 = Critical, 2 = Important, 3 = Routine. */
export const priorities = {
  1: 'var(--priority-1)',
  2: 'var(--priority-2)',
  3: 'var(--priority-3)',
} as const;

/** Semantic status colors for UI feedback states. */
export const status = {
  success: 'var(--success)',
  warning: 'var(--warning)',
  danger: 'var(--danger)',
  info: 'var(--info)',
} as const;

/** Glow effect colors for status indicators and animated glows. */
export const glows = {
  success: 'var(--glow-success)',
  warning: 'var(--glow-warning)',
  danger: 'var(--glow-danger)',
  info: 'var(--glow-info)',
} as const;

/** Animation durations in milliseconds for JS/CSS synchronization. */
export const durations = {
  instant: 100,
  fast: 150,
  normal: 250,
  slow: 400,
  slower: 600,
} as const;

/** Cubic-bezier easing arrays for Framer Motion or CSS cubic-bezier(). */
export const easings = {
  /** Fast start, slow end - responsive feel */
  outExpo: [0.19, 1, 0.22, 1] as const,
  /** Slow start and end, fast middle - dramatic */
  inOutExpo: [0.87, 0, 0.13, 1] as const,
  /** Slight overshoot - playful */
  spring: [0.175, 0.885, 0.32, 1.275] as const,
  /** Bouncy overshoot - energetic */
  bounce: [0.68, -0.55, 0.265, 1.55] as const,
} as const;

/** Touch target sizes in rem units (min: 44px, comfortable: 48px, large: 56px). */
export const touchTargets = {
  min: 2.75,
  comfortable: 3,
  large: 3.5,
} as const;

/** Standard Tailwind CSS breakpoints in pixels. */
export const breakpoints = {
  sm: 640,
  md: 768,
  lg: 1024,
  xl: 1280,
  '2xl': 1536,
} as const;

/** Layout dimension constants in pixels for geometry calculations. */
export const layout = {
  statusBarHeight: 56,
  statusBarHeightMobile: 48,
  sidebarWidth: 360,
  sidebarWidthCollapsed: 64,
  panelPadding: 16,
  cardGap: 12,
  sectionGap: 24,
} as const;

export type SensorType = 'thermal' | 'acoustic' | 'gas';
export type ConfidenceLevel = 'high' | 'medium' | 'low' | 'unverified';
export type StatusLevel = 'success' | 'warning' | 'danger' | 'info';
export type PriorityLevel = 1 | 2 | 3;

/** Returns the CSS color variable for a sensor type. */
export function getSensorColor(type: SensorType): string {
  return sensors[type];
}

/** Returns the CSS color variable for a confidence level. */
export function getConfidenceColor(level: ConfidenceLevel): string {
  return confidence[level];
}

/**
 * Maps a percentage to a confidence level tier.
 * Thresholds: >=80% high, >=50% medium, >=20% low, <20% unverified.
 */
export function getConfidenceLevel(percentage: number): ConfidenceLevel {
  if (percentage >= 80) return 'high';
  if (percentage >= 50) return 'medium';
  if (percentage >= 20) return 'low';
  return 'unverified';
}

/** Returns the CSS color variable for a status level. */
export function getStatusColor(level: StatusLevel): string {
  return status[level];
}

/** Returns the CSS color variable for a priority level (1-3). */
export function getPriorityColor(level: PriorityLevel): string {
  return priorities[level];
}
