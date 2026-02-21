import test from 'node:test';
import assert from 'node:assert/strict';

import {
  getFreshReplayEntry,
  pruneReplayCacheEntries,
  resetReplayCaches,
} from './replayCacheLifecycle.js';

test('resetReplayCaches clears all lifecycle caches used by replay hooks', () => {
  const metadata = new Map([
    ['a', { observedAtMs: 100 }],
    ['b', { observedAtMs: 200 }],
  ]);
  const index = new Map([
    ['k', { observedAtMs: 150 }],
  ]);
  const latestByVehicle = new Map([
    ['scout_1', 'telemetry:scout_1:10:20'],
  ]);

  resetReplayCaches([metadata, index, latestByVehicle]);

  assert.equal(metadata.size, 0);
  assert.equal(index.size, 0);
  assert.equal(latestByVehicle.size, 0);
});

test('getFreshReplayEntry returns a replay entry when it is still within TTL', () => {
  const cache = new Map([
    ['dedupe-1', { observedAtMs: 1_000, deliveryMode: 'replayed' }],
  ]);

  const entry = getFreshReplayEntry(cache, 'dedupe-1', 500, 1_400);

  assert.deepEqual(entry, { observedAtMs: 1_000, deliveryMode: 'replayed' });
  assert.equal(cache.size, 1);
});

test('getFreshReplayEntry prunes and returns null for stale replay entries', () => {
  const cache = new Map([
    ['dedupe-1', { observedAtMs: 1_000, deliveryMode: 'replayed' }],
  ]);

  const entry = getFreshReplayEntry(cache, 'dedupe-1', 500, 1_501);

  assert.equal(entry, null);
  assert.equal(cache.size, 0);
});

test('pruneReplayCacheEntries removes stale and malformed observedAtMs entries', () => {
  const cache = new Map([
    ['fresh', { observedAtMs: 950 }],
    ['stale', { observedAtMs: 100 }],
    ['bad', { observedAtMs: Number.NaN }],
  ]);

  pruneReplayCacheEntries(cache, 100, 1_000);

  assert.deepEqual([...cache.keys()], ['fresh']);
});
