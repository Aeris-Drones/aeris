'use client';

import { useEffect, useMemo, useRef, useState } from 'react';
import ROSLIB from 'roslib';
import type { Detection } from '@/components/sheets/DetectionCard';
import type { VehicleState } from '@/lib/vehicle/VehicleManager';
import { normalizeFusedDetectionMessage } from '@/lib/ros/fusedDetections';
import {
  getFreshReplayEntry,
  pruneReplayCacheEntries,
  resetReplayCaches,
} from '@/lib/ros/replayCacheLifecycle';
import {
  findNearestVehicle,
  mergeLiveDetections,
  subscribeToFusedTopic,
  toVehicleName,
} from '@/lib/ros/fusedDetectionsFeed';
import { useROSConnection } from './useROSConnection';

interface UseFusedDetectionsOptions {
  topicName?: string;
  messageType?: string;
  replayAnnotationTopicName?: string;
  replayAnnotationMessageType?: string;
  replayAnnotationTtlMs?: number;
  maxDetections?: number;
  maxAgeMs?: number;
  maxFutureSkewMs?: number;
  fallbackVehicleId?: string;
}

interface UseFusedDetectionsResult {
  detections: Detection[];
  isConnected: boolean;
}

const DEFAULT_OPTIONS: Required<UseFusedDetectionsOptions> = {
  topicName: '/detections/fused',
  messageType: 'aeris_msgs/FusedDetection',
  replayAnnotationTopicName: '/mesh/replay_annotations',
  replayAnnotationMessageType: 'std_msgs/String',
  replayAnnotationTtlMs: 5 * 60 * 1000,
  maxDetections: 250,
  maxAgeMs: 5 * 60 * 1000,
  maxFutureSkewMs: 5_000,
  fallbackVehicleId: 'fusion',
};

interface ReplayMetadata {
  deliveryMode: 'live' | 'replayed';
  originalEventTs: number;
  replayedAtTs?: number;
  observedAtMs: number;
}

interface ReplayDetectionIndexEntry {
  detectionId: string;
  observedAtMs: number;
}

export function useFusedDetections(
  vehicles: VehicleState[],
  options: UseFusedDetectionsOptions = {}
): UseFusedDetectionsResult {
  const merged = { ...DEFAULT_OPTIONS, ...options };
  const { ros, isConnected } = useROSConnection();
  const [detections, setDetections] = useState<Detection[]>([]);
  const vehiclesRef = useRef<VehicleState[]>(vehicles);
  const replayMetadataRef = useRef<Map<string, ReplayMetadata>>(new Map());
  const replayDetectionIndexRef = useRef<Map<string, ReplayDetectionIndexEntry>>(new Map());

  const fallbackVehicleName = useMemo(
    () => toVehicleName(merged.fallbackVehicleId),
    [merged.fallbackVehicleId]
  );

  useEffect(() => {
    vehiclesRef.current = vehicles;
  }, [vehicles]);

  useEffect(() => {
    const replayMetadataCache = replayMetadataRef.current;
    const replayDetectionIndexCache = replayDetectionIndexRef.current;
    resetReplayCaches([replayMetadataCache, replayDetectionIndexCache]);

    if (!ros || !isConnected) {
      return;
    }

    const applyReplayMetadata = (
      dedupeKey: string | null,
      detection: Detection
    ): Detection => {
      if (!dedupeKey) {
        return detection;
      }

      const metadata = getFreshReplayEntry(
        replayMetadataRef.current,
        dedupeKey,
        merged.replayAnnotationTtlMs
      );
      if (!metadata) {
        return detection;
      }
      return {
        ...detection,
        deliveryMode: metadata.deliveryMode,
        originalEventTs: metadata.originalEventTs,
        replayedAtTs: metadata.replayedAtTs,
        isRetroactive: metadata.deliveryMode === 'replayed',
      };
    };

    const handleMessage = (rawMessage: ROSLIB.Message) => {
      let normalized;
      try {
        normalized = normalizeFusedDetectionMessage(rawMessage, {
          maxAgeMs: merged.maxAgeMs,
          maxFutureSkewMs: merged.maxFutureSkewMs,
          fallbackVehicleId: merged.fallbackVehicleId,
          fallbackVehicleName,
        });
      } catch (error) {
        console.warn('[useFusedDetections] Ignoring malformed fused detection payload:', error);
        return;
      }

      if (!normalized) {
        return;
      }

      const nearestVehicle = findNearestVehicle(vehiclesRef.current, normalized.position);
      const detection: Detection = nearestVehicle
        ? {
            ...normalized,
            vehicleId: nearestVehicle.id,
            vehicleName: toVehicleName(nearestVehicle.id),
          }
        : normalized;
      const dedupeKey = buildFusedDetectionDedupeKey(rawMessage);
      const detectionWithReplay = applyReplayMetadata(dedupeKey, detection);

      if (dedupeKey) {
        replayDetectionIndexRef.current.set(dedupeKey, {
          detectionId: detectionWithReplay.id,
          observedAtMs: Date.now(),
        });
        pruneReplayCacheEntries(replayDetectionIndexRef.current, merged.replayAnnotationTtlMs);
      }

      setDetections((previous) => {
        return mergeLiveDetections(previous, detectionWithReplay, {
          maxAgeMs: merged.maxAgeMs,
          maxDetections: merged.maxDetections,
        });
      });
    };
    const handleReplayAnnotation = (rawMessage: ROSLIB.Message) => {
      const parsed = parseReplayAnnotation(rawMessage);
      if (!parsed || parsed.routeKey !== 'fused_detection') {
        return;
      }
      const nowMs = Date.now();
      replayMetadataRef.current.set(parsed.dedupeKey, {
        deliveryMode: parsed.deliveryMode,
        originalEventTs: parsed.originalEventTs,
        replayedAtTs: parsed.replayedAtTs,
        observedAtMs: nowMs,
      });
      const indexedDetection = replayDetectionIndexRef.current.get(parsed.dedupeKey);
      const indexedDetectionId =
        indexedDetection && (nowMs - indexedDetection.observedAtMs) <= merged.replayAnnotationTtlMs
          ? indexedDetection.detectionId
          : extractFusedDetectionIdFromDedupeKey(parsed.dedupeKey);

      if (indexedDetectionId) {
        setDetections((previous) => {
          let changed = false;
          const next = previous.map((detection) => {
            if (detection.id !== indexedDetectionId) {
              return detection;
            }
            changed = true;
            return {
              ...detection,
              deliveryMode: parsed.deliveryMode,
              originalEventTs: parsed.originalEventTs,
              replayedAtTs: parsed.replayedAtTs,
              isRetroactive: parsed.deliveryMode === 'replayed',
            };
          });
          return changed ? next : previous;
        });
      }
      pruneReplayCacheEntries(replayMetadataRef.current, merged.replayAnnotationTtlMs);
      pruneReplayCacheEntries(replayDetectionIndexRef.current, merged.replayAnnotationTtlMs);
    };

    const unsubscribeFused = subscribeToFusedTopic({
      topicFactory: (args) => new ROSLIB.Topic(args),
      ros,
      topicName: merged.topicName,
      messageType: merged.messageType,
      onMessage: handleMessage,
    });
    const replayTopic = new ROSLIB.Topic({
      ros,
      name: merged.replayAnnotationTopicName,
      messageType: merged.replayAnnotationMessageType,
    });
    replayTopic.subscribe(handleReplayAnnotation);

    return () => {
      unsubscribeFused();
      replayTopic.unsubscribe();
      resetReplayCaches([replayMetadataCache, replayDetectionIndexCache]);
    };
  }, [
    fallbackVehicleName,
    isConnected,
    merged.fallbackVehicleId,
    merged.maxAgeMs,
    merged.maxDetections,
    merged.maxFutureSkewMs,
    merged.messageType,
    merged.replayAnnotationMessageType,
    merged.replayAnnotationTopicName,
    merged.replayAnnotationTtlMs,
    merged.topicName,
    ros,
  ]);

  return { detections, isConnected };
}

function buildFusedDetectionDedupeKey(rawMessage: ROSLIB.Message): string | null {
  const candidateId = String((rawMessage as { candidate_id?: unknown }).candidate_id ?? '');
  const stamp = (rawMessage as { stamp?: { sec?: unknown; nanosec?: unknown } }).stamp;
  const sec = stamp?.sec ?? 0;
  const nanosec = stamp?.nanosec ?? 0;
  return `fused_detection:${candidateId}:${sec}:${nanosec}`;
}

function parseReplayAnnotation(rawMessage: ROSLIB.Message): {
  dedupeKey: string;
  routeKey: string;
  deliveryMode: 'live' | 'replayed';
  originalEventTs: number;
  replayedAtTs?: number;
} | null {
  const payload = (rawMessage as { data?: unknown }).data;
  if (typeof payload !== 'string' || !payload.trim()) {
    return null;
  }
  try {
    const parsed = JSON.parse(payload) as Record<string, unknown>;
    const dedupeKey = String(parsed.dedupe_key ?? '').trim();
    const routeKey = String(parsed.route_key ?? '').trim();
    if (!dedupeKey || !routeKey) {
      return null;
    }

    const originalEventTs = coerceEpochMs(parsed.original_event_ts);
    if (originalEventTs === null) {
      return null;
    }
    const replayedAtTs = coerceEpochMs(parsed.replayed_at_ts);
    const deliveryMode =
      typeof parsed.delivery_mode === 'string' &&
      parsed.delivery_mode.trim().toLowerCase() === 'replayed'
        ? 'replayed'
        : 'live';
    return {
      dedupeKey,
      routeKey,
      deliveryMode,
      originalEventTs,
      replayedAtTs: replayedAtTs ?? undefined,
    };
  } catch {
    return null;
  }
}

function coerceEpochMs(value: unknown): number | null {
  if (typeof value === 'number' && Number.isFinite(value)) {
    return value > 1_000_000_000_000 ? value : value * 1000;
  }
  if (value && typeof value === 'object') {
    const stamp = value as { sec?: unknown; nanosec?: unknown };
    const sec = Number(stamp.sec);
    const nanosec = Number(stamp.nanosec ?? 0);
    if (Number.isFinite(sec)) {
      const safeNanosec = Number.isFinite(nanosec) ? nanosec : 0;
      return (sec * 1000) + (safeNanosec / 1_000_000);
    }
  }
  return null;
}

function extractFusedDetectionIdFromDedupeKey(dedupeKey: string): string | null {
  const [routeKey, candidateId] = dedupeKey.split(':', 3);
  if (routeKey !== 'fused_detection' || !candidateId?.trim()) {
    return null;
  }
  return candidateId.trim();
}
