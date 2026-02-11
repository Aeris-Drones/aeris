/**
 * Design Tokens
 *
 * Centralized design system values for consistent UI styling.
 * All values reference CSS custom properties (variables) defined
 * in the global stylesheet, allowing runtime theming.
 *
 * Usage:
 *   import { surfaces, glass, sensors } from './design-tokens';
 *   style={{ backgroundColor: surfaces[1], borderColor: sensors.thermal }}
 */

/**
 * Surface elevation levels (0-4).
 *
 * Higher numbers represent elevated surfaces (cards, modals, popovers).
 * Each level maps to a CSS variable with appropriate background/border colors.
 */
export const surfaces = {
  0: 'var(--surface-0)',
  1: 'var(--surface-1)',
  2: 'var(--surface-2)',
  3: 'var(--surface-3)',
  4: 'var(--surface-4)',
} as const;

/**
 * Glassmorphism effect tokens.
 *
 * Used for translucent overlays and floating panels.
 * Combines backdrop blur with semi-transparent backgrounds.
 */
export const glass = {
  bg: 'var(--glass-bg)',
  bgHover: 'var(--glass-bg-hover)',
  border: 'var(--glass-border)',
  borderHover: 'var(--glass-border-hover)',
  highlight: 'var(--glass-highlight)',
  shadow: 'var(--glass-shadow)',
} as const;

/**
 * Confidence level colors for detection classification.
 *
 * Maps confidence tiers to semantic color variables.
 * - high (>=80%): Strong positive identification
 * - medium (50-79%): Probable detection
 * - low (20-49%): Possible detection
 * - unverified (<20%): Unconfirmed signal
 */
export const confidence = {
  high: 'var(--confidence-high)',
  medium: 'var(--confidence-medium)',
  low: 'var(--confidence-low)',
  unverified: 'var(--confidence-unverified)',
} as const;

/**
 * Sensor type color palette.
 *
 * Each sensor has a primary color and glow variant for visual effects.
 * - thermal: Heat/infrared detection
 * - acoustic: Sound/vibration detection
 * - gas: Chemical/sensor detection
 */
export const sensors = {
  thermal: 'var(--sensor-thermal)',
  thermalGlow: 'var(--sensor-thermal-glow)',
  acoustic: 'var(--sensor-acoustic)',
  acousticVocal: 'var(--sensor-acoustic-vocal)',
  acousticGlow: 'var(--sensor-acoustic-glow)',
  gas: 'var(--sensor-gas)',
  gasGlow: 'var(--sensor-gas-glow)',
} as const;

/**
 * Priority level colors (1-3).
 *
 * 1 = Critical (immediate attention)
 * 2 = Important (attention soon)
 * 3 = Routine (standard priority)
 */
export const priorities = {
  1: 'var(--priority-1)',
  2: 'var(--priority-2)',
  3: 'var(--priority-3)',
} as const;

/**
 * Semantic status colors.
 *
 * Standard feedback states for UI elements.
 */
export const status = {
  success: 'var(--success)',
  warning: 'var(--warning)',
  danger: 'var(--danger)',
  info: 'var(--info)',
} as const;

/**
 * Glow effect colors for status indicators.
 *
 * Used with box-shadow for pulsing/animated glows.
 */
export const glows = {
  success: 'var(--glow-success)',
  warning: 'var(--glow-warning)',
  danger: 'var(--glow-danger)',
  info: 'var(--glow-info)',
} as const;

/**
 * Animation durations in milliseconds.
 *
 * Use these for consistent timing in JavaScript animations
 * that need to match CSS transitions.
 */
export const durations = {
  instant: 100,
  fast: 150,
  normal: 250,
  slow: 400,
  slower: 600,
} as const;

/**
 * Cubic-bezier easing functions.
 *
 * These arrays can be used directly in Framer Motion or
 * converted to CSS cubic-bezier() strings.
 */
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

/**
 * Touch target sizes in rem units.
 *
 * Minimum recommended sizes for accessible touch interfaces.
 * - min: 44px equivalent (absolute minimum)
 * - comfortable: 48px equivalent (recommended)
 * - large: 56px equivalent (primary actions)
 */
export const touchTargets = {
  min: 2.75,
  comfortable: 3,
  large: 3.5,
} as const;

/**
 * Responsive breakpoint widths in pixels.
 *
 * Standard Tailwind CSS breakpoints for consistent media queries.
 */
export const breakpoints = {
  sm: 640,
  md: 768,
  lg: 1024,
  xl: 1280,
  '2xl': 1536,
} as const;

/**
 * Layout dimension constants in pixels.
 *
 * Use these for calculations involving layout geometry.
 */
export const layout = {
  statusBarHeight: 56,
  statusBarHeightMobile: 48,
  sidebarWidth: 360,
  sidebarWidthCollapsed: 64,
  panelPadding: 16,
  cardGap: 12,
  sectionGap: 24,
} as const;

// Type definitions for design token categories
export type SensorType = 'thermal' | 'acoustic' | 'gas';
export type ConfidenceLevel = 'high' | 'medium' | 'low' | 'unverified';
export type StatusLevel = 'success' | 'warning' | 'danger' | 'info';
export type PriorityLevel = 1 | 2 | 3;

/**
 * Gets the CSS color variable for a sensor type.
 *
 * @param type - Sensor type identifier
 * @returns CSS variable string for the sensor's primary color
 */
export function getSensorColor(type: SensorType): string {
  return sensors[type];
}

/**
 * Gets the CSS color variable for a confidence level.
 *
 * @param level - Confidence tier identifier
 * @returns CSS variable string for the confidence color
 */
export function getConfidenceColor(level: ConfidenceLevel): string {
  return confidence[level];
}

/**
 * Determines confidence level from a percentage value.
 *
 * Thresholds:
 * - >= 80%: high
 * - >= 50%: medium
 * - >= 20%: low
 * - < 20%: unverified
 *
 * @param percentage - Numeric percentage (0-100)
 * @returns ConfidenceLevel tier
 */
export function getConfidenceLevel(percentage: number): ConfidenceLevel {
  if (percentage >= 80) return 'high';
  if (percentage >= 50) return 'medium';
  if (percentage >= 20) return 'low';
  return 'unverified';
}

/**
 * Gets the CSS color variable for a status level.
 *
 * @param level - Status identifier
 * @returns CSS variable string for the status color
 */
export function getStatusColor(level: StatusLevel): string {
  return status[level];
}

/**
 * Gets the CSS color variable for a priority level.
 *
 * @param level - Priority number (1-3)
 * @returns CSS variable string for the priority color
 */
export function getPriorityColor(level: PriorityLevel): string {
  return priorities[level];
}
