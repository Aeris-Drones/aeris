/**
 * High-level mission execution states managed by the mission controller.
 *
 * These phases represent the operational lifecycle of a search and track mission,
 * from initial planning through completion or abort.
 */
export type MissionControlPhase =
  | "IDLE"
  | "PLANNING"
  | "SEARCHING"
  | "TRACKING"
  | "COMPLETE"
  | "ABORTED";

/**
 * Computed UI state flags derived from mission phase and system status.
 *
 * These flags drive button enablement, status indicators, and form validation
 * in the mission control interface. All flags are derived from canonical state
 * to ensure UI consistency.
 */
export interface MissionControlFlags {
  /** Whether a mission is currently executing (not IDLE or COMPLETE) */
  isActive: boolean;
  /** Whether the mission is temporarily suspended */
  isPaused: boolean;
  /** Whether the mission has reached a terminal success state */
  isComplete: boolean;
  /** Whether the start command is valid given current conditions */
  canStart: boolean;
  /** Whether pause can be requested (mission is active and not paused) */
  canPause: boolean;
  /** Whether resume can be requested (mission is paused) */
  canResume: boolean;
  /** Whether abort can be requested (mission is active) */
  canAbort: boolean;
}

/**
 * Determines if the mission is in a paused state.
 *
 * A mission is considered paused when it has a recorded pause timestamp,
 * regardless of current phase. This allows pause to persist across phase
 * transitions (e.g., pausing during SEARCHING and resuming into TRACKING).
 *
 * @param phase - Current mission phase
 * @param pausedAt - Unix timestamp when pause was requested, undefined if not paused
 * @returns True if the mission is currently paused
 */
export function isMissionPaused(
  phase: MissionControlPhase,
  pausedAt: number | undefined
): boolean;

/**
 * Computes UI state flags from mission and system status.
 *
 * Centralizes business logic for command availability to ensure consistent
 * UI behavior across mission control components. Flags consider ROS
 * connection health, mission phase, and operational prerequisites.
 *
 * @param args - Mission state and system context
 * @returns Computed UI flags for button states and validation
 */
export function computeMissionControlFlags(args: {
  phase: MissionControlPhase;
  pausedAt: number | undefined;
  hasValidStartZone: boolean;
  rosConnected: boolean;
}): MissionControlFlags;

/**
 * Validates preconditions for aborting a mission.
 *
 * Abort requires an active ROS connection and an associated mission ID.
 * This check prevents spurious abort commands when not connected or when
 * no mission is loaded.
 *
 * @param args - Validation context
 * @returns Error message if validation fails, null if abort is permitted
 */
export function getAbortMissionValidationError(args: {
  rosConnected: boolean;
  missionId: string | undefined;
}): string | null;

/**
 * Wraps a service call with timeout handling.
 *
 * ROS service calls can hang indefinitely if the service node is unresponsive.
 * This utility ensures predictable failure behavior and prevents UI lockup
 * during service outages.
 *
 * @param invoke - Function that initiates the service call
 * @param timeoutMs - Maximum wait time before rejecting
 * @param label - Optional identifier for error logging
 * @returns Promise that resolves with the service result or rejects on timeout/error
 */
export function withServiceTimeout<T>(
  invoke: (
    resolve: (value: T) => void,
    reject: (error: unknown) => void
  ) => void,
  timeoutMs: number,
  label?: string
): Promise<T>;
