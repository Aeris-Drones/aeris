/**
 * Layout components barrel file.
 *
 * Exports the core layout primitives for the GCS UI:
 * - GCSLayout: Main layout shell with slot-based composition
 * - StatusPill: Mission status indicator for the header
 * - CommandDock: Bottom-mounted control panel container
 */
export { GCSLayout } from './GCSLayout';
export { StatusPill } from './StatusPill';
export type { MissionPhase, ConnectionStatus, StatusPillProps } from './StatusPill';
export { CommandDock } from './CommandDock';
