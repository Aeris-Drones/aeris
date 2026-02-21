export function resetReplayCaches(
  caches: Array<Map<unknown, unknown>>
): void;

export function pruneReplayCacheEntries<T extends { observedAtMs: number }>(
  cache: Map<string, T>,
  ttlMs: number,
  nowMs?: number
): void;

export function getFreshReplayEntry<T extends { observedAtMs: number }>(
  cache: Map<string, T>,
  key: string,
  ttlMs: number,
  nowMs?: number
): T | null;
