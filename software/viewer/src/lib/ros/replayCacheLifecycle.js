export function resetReplayCaches(caches) {
  for (const cache of caches) {
    if (cache && typeof cache.clear === 'function') {
      cache.clear();
    }
  }
}

export function pruneReplayCacheEntries(cache, ttlMs, nowMs = Date.now()) {
  const cutoff = nowMs - ttlMs;
  for (const [key, value] of cache.entries()) {
    const observedAtMs = Number(value?.observedAtMs);
    if (!Number.isFinite(observedAtMs) || observedAtMs < cutoff) {
      cache.delete(key);
    }
  }
}

export function getFreshReplayEntry(cache, key, ttlMs, nowMs = Date.now()) {
  const entry = cache.get(key);
  if (!entry) {
    return null;
  }
  const observedAtMs = Number(entry.observedAtMs);
  if (!Number.isFinite(observedAtMs) || nowMs - observedAtMs > ttlMs) {
    cache.delete(key);
    return null;
  }
  return entry;
}
