export const surfaces = {
  0: 'var(--surface-0)',
  1: 'var(--surface-1)',
  2: 'var(--surface-2)',
  3: 'var(--surface-3)',
  4: 'var(--surface-4)',
} as const;

export const glass = {
  bg: 'var(--glass-bg)',
  bgHover: 'var(--glass-bg-hover)',
  border: 'var(--glass-border)',
  borderHover: 'var(--glass-border-hover)',
  highlight: 'var(--glass-highlight)',
  shadow: 'var(--glass-shadow)',
} as const;

export const confidence = {
  high: 'var(--confidence-high)',
  medium: 'var(--confidence-medium)',
  low: 'var(--confidence-low)',
  unverified: 'var(--confidence-unverified)',
} as const;

export const sensors = {
  thermal: 'var(--sensor-thermal)',
  thermalGlow: 'var(--sensor-thermal-glow)',
  acoustic: 'var(--sensor-acoustic)',
  acousticVocal: 'var(--sensor-acoustic-vocal)',
  acousticGlow: 'var(--sensor-acoustic-glow)',
  gas: 'var(--sensor-gas)',
  gasGlow: 'var(--sensor-gas-glow)',
} as const;

export const priorities = {
  1: 'var(--priority-1)',
  2: 'var(--priority-2)',
  3: 'var(--priority-3)',
} as const;

export const status = {
  success: 'var(--success)',
  warning: 'var(--warning)',
  danger: 'var(--danger)',
  info: 'var(--info)',
} as const;

export const glows = {
  success: 'var(--glow-success)',
  warning: 'var(--glow-warning)',
  danger: 'var(--glow-danger)',
  info: 'var(--glow-info)',
} as const;

export const durations = {
  instant: 100,
  fast: 150,
  normal: 250,
  slow: 400,
  slower: 600,
} as const;

export const easings = {
  outExpo: [0.19, 1, 0.22, 1] as const,
  inOutExpo: [0.87, 0, 0.13, 1] as const,
  spring: [0.175, 0.885, 0.32, 1.275] as const,
  bounce: [0.68, -0.55, 0.265, 1.55] as const,
} as const;

export const touchTargets = {
  min: 2.75,
  comfortable: 3,
  large: 3.5,
} as const;

export const breakpoints = {
  sm: 640,
  md: 768,
  lg: 1024,
  xl: 1280,
  '2xl': 1536,
} as const;

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

export function getSensorColor(type: SensorType): string {
  return sensors[type];
}

export function getConfidenceColor(level: ConfidenceLevel): string {
  return confidence[level];
}

export function getConfidenceLevel(percentage: number): ConfidenceLevel {
  if (percentage >= 80) return 'high';
  if (percentage >= 50) return 'medium';
  if (percentage >= 20) return 'low';
  return 'unverified';
}

export function getStatusColor(level: StatusLevel): string {
  return status[level];
}

export function getPriorityColor(level: PriorityLevel): string {
  return priorities[level];
}
