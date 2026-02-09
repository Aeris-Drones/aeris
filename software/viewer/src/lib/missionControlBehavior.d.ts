export type MissionControlPhase =
  | "IDLE"
  | "PLANNING"
  | "SEARCHING"
  | "TRACKING"
  | "COMPLETE"
  | "ABORTED";

export interface MissionControlFlags {
  isActive: boolean;
  isPaused: boolean;
  isComplete: boolean;
  canStart: boolean;
  canPause: boolean;
  canResume: boolean;
  canAbort: boolean;
}

export function isMissionPaused(
  phase: MissionControlPhase,
  pausedAt: number | undefined
): boolean;

export function computeMissionControlFlags(args: {
  phase: MissionControlPhase;
  pausedAt: number | undefined;
  hasValidStartZone: boolean;
  rosConnected: boolean;
}): MissionControlFlags;

export function getAbortMissionValidationError(args: {
  rosConnected: boolean;
  missionId: string | undefined;
}): string | null;

export function withServiceTimeout<T>(
  invoke: (
    resolve: (value: T) => void,
    reject: (error: unknown) => void
  ) => void,
  timeoutMs: number,
  label?: string
): Promise<T>;
