import { useEffect, useState, useRef } from 'react';
import ROSLIB from 'roslib';
import { useROSConnection } from './useROSConnection';
import { VehicleManager, VehicleState } from '../lib/vehicle/VehicleManager';
import { parseVehicleTelemetry } from '../lib/ros/telemetry';
import {
  getFreshReplayEntry,
  pruneReplayCacheEntries,
  resetReplayCaches,
} from '../lib/ros/replayCacheLifecycle';
import { useCoordinateOrigin } from '../context/CoordinateOriginContext';

type ReturnTrajectoryMap = Record<string, [number, number, number][]>;
const REPLAY_METADATA_TTL_MS = 5 * 60 * 1000;

/**
 * Subscribes to vehicle telemetry from ROS and manages real-time state.
 *
 * Integrates with VehicleManager for coordinate transformations and
 * trajectory tracking. Auto-sets coordinate origin from first telemetry
 * if not already configured.
 *
 * ROS Topics:
 * - /vehicle/telemetry: Position, orientation, velocity (aeris_msgs/Telemetry)
 * - /mission/progress: Return-to-launch trajectories (std_msgs/String JSON)
 */
export function useVehicleTelemetry() {
  const { ros, isConnected } = useROSConnection();
  const { origin, setOrigin } = useCoordinateOrigin();
  const [vehicles, setVehicles] = useState<VehicleState[]>([]);
  const [returnTrajectories, setReturnTrajectories] = useState<ReturnTrajectoryMap>({});

  const [manager] = useState(() => new VehicleManager());
  const originRef = useRef(origin);
  const setOriginRef = useRef(setOrigin);
  const replayMetadataRef = useRef<Map<string, {
    deliveryMode: 'live' | 'replayed';
    originalEventTsMs: number;
    replayedAtTsMs: number | null;
    observedAtMs: number;
  }>>(new Map());
  const replayMessageIndexRef = useRef<Map<string, { vehicleId: string; observedAtMs: number }>>(new Map());
  const latestTelemetryKeyByVehicleRef = useRef<Map<string, string>>(new Map());

  useEffect(() => {
    originRef.current = origin;
    setOriginRef.current = setOrigin;
  }, [origin, setOrigin]);

  useEffect(() => {
    const replayMetadataCache = replayMetadataRef.current;
    const replayMessageIndexCache = replayMessageIndexRef.current;
    const latestTelemetryKeyCache = latestTelemetryKeyByVehicleRef.current;
    resetReplayCaches([
      replayMetadataCache,
      replayMessageIndexCache,
      latestTelemetryKeyCache,
    ]);

    if (!ros || !isConnected) return;

    const topic = new ROSLIB.Topic({
      ros: ros,
      name: '/vehicle/telemetry',
      messageType: 'aeris_msgs/Telemetry',
    });
    const progressTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/mission/progress',
      messageType: 'std_msgs/String',
    });
    const replayTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/mesh/replay_annotations',
      messageType: 'std_msgs/String',
    });

    const handleMessage = (message: ROSLIB.Message) => {
      let telemetry;
      try {
        telemetry = parseVehicleTelemetry(message);
      } catch (error) {
        console.warn('[useVehicleTelemetry] Ignoring invalid telemetry payload:', error);
        return;
      }
      const dedupeKey = buildTelemetryDedupeKey(message);
      if (dedupeKey) {
        replayMessageIndexRef.current.set(dedupeKey, {
          vehicleId: telemetry.vehicle_id,
          observedAtMs: Date.now(),
        });
        latestTelemetryKeyByVehicleRef.current.set(telemetry.vehicle_id, dedupeKey);
        pruneReplayCacheEntries(replayMessageIndexRef.current, REPLAY_METADATA_TTL_MS);

        const replayMeta = getFreshReplayEntry(
          replayMetadataRef.current,
          dedupeKey,
          REPLAY_METADATA_TTL_MS
        );
        if (replayMeta) {
          telemetry = {
            ...telemetry,
            replay: {
              deliveryMode: replayMeta.deliveryMode,
              originalEventTsMs: replayMeta.originalEventTsMs,
              replayedAtTsMs: replayMeta.replayedAtTsMs,
              isRetroactive: replayMeta.deliveryMode === 'replayed',
            },
          };
        }
      }

      const newOrigin = manager.processTelemetry(telemetry, originRef.current);

      if (newOrigin) {
        setOriginRef.current(newOrigin);
      }

      setVehicles([...manager.getVehicles()]);
    };

    const handleProgressMessage = (message: ROSLIB.Message) => {
      try {
        const payload = JSON.parse((message as { data: string }).data) as {
          vehicleId?: string;
          returnTrajectory?: {
            vehicleId?: string;
            points?: Array<{
              x?: number;
              z?: number;
              altitude_m?: number;
              altitudeM?: number;
            }>;
          } | null;
        };
        if (!('returnTrajectory' in payload)) {
          return;
        }
        if (payload.returnTrajectory === null || payload.returnTrajectory === undefined) {
          const vehicleId = (payload.vehicleId ?? '').trim();
          if (vehicleId) {
            setReturnTrajectories(previous => {
              const next = { ...previous };
              delete next[vehicleId];
              return next;
            });
          }
          return;
        }
        const returnTrajectory = payload.returnTrajectory;
        if (!returnTrajectory) {
          return;
        }
        const vehicleId = returnTrajectory.vehicleId?.trim();
        if (!vehicleId) {
          return;
        }
        const points = (returnTrajectory.points ?? [])
          .map(point => {
            const x = Number(point.x);
            const z = Number(point.z);
            const y = Number(
              point.altitude_m ?? point.altitudeM ?? 0
            );
            if (![x, y, z].every(Number.isFinite)) {
              return null;
            }
            return [x, y, z] as [number, number, number];
          })
          .filter((point): point is [number, number, number] => point !== null);
        setReturnTrajectories(previous => {
          const next = { ...previous };
          if (points.length < 2) {
            delete next[vehicleId];
          } else {
            next[vehicleId] = points;
          }
          return next;
        });
      } catch (error) {
        console.warn('[useVehicleTelemetry] Ignoring invalid mission progress payload:', error);
      }
    };
    const handleReplayMessage = (message: ROSLIB.Message) => {
      const parsed = parseReplayAnnotation(message);
      if (!parsed || parsed.routeKey !== 'telemetry') {
        return;
      }
      replayMetadataRef.current.set(parsed.dedupeKey, {
        deliveryMode: parsed.deliveryMode,
        originalEventTsMs: parsed.originalEventTsMs,
        replayedAtTsMs: parsed.replayedAtTsMs,
        observedAtMs: Date.now(),
      });
      const indexedTelemetry = replayMessageIndexRef.current.get(parsed.dedupeKey);
      if (indexedTelemetry) {
        const latestKey = latestTelemetryKeyByVehicleRef.current.get(indexedTelemetry.vehicleId);
        if (latestKey === parsed.dedupeKey) {
          manager.applyReplayMetadata(indexedTelemetry.vehicleId, {
            deliveryMode: parsed.deliveryMode,
            originalEventTsMs: parsed.originalEventTsMs,
            replayedAtTsMs: parsed.replayedAtTsMs,
            isRetroactive: parsed.deliveryMode === 'replayed',
          });
          setVehicles([...manager.getVehicles()]);
        }
      }
      pruneReplayCacheEntries(replayMetadataRef.current, REPLAY_METADATA_TTL_MS);
      pruneReplayCacheEntries(replayMessageIndexRef.current, REPLAY_METADATA_TTL_MS);
    };

    topic.subscribe(handleMessage);
    progressTopic.subscribe(handleProgressMessage);
    replayTopic.subscribe(handleReplayMessage);

    return () => {
      topic.unsubscribe();
      progressTopic.unsubscribe();
      replayTopic.unsubscribe();
      resetReplayCaches([
        replayMetadataCache,
        replayMessageIndexCache,
        latestTelemetryKeyCache,
      ]);
    };
  }, [ros, isConnected, manager]);

  useEffect(() => {
      const interval = setInterval(() => {
          const currentVehicles = manager.getVehicles();
          if (currentVehicles.length !== vehicles.length) {
             setVehicles([...currentVehicles]);
          }
      }, 1000);
      return () => clearInterval(interval);
  }, [vehicles.length, manager]);

  return { vehicles, manager, returnTrajectories };
}

function buildTelemetryDedupeKey(rawMessage: ROSLIB.Message): string | null {
  const vehicleId = String((rawMessage as { vehicle_id?: unknown }).vehicle_id ?? '');
  const stamp = (rawMessage as { timestamp?: { sec?: unknown; nanosec?: unknown } }).timestamp;
  const sec = stamp?.sec ?? 0;
  const nanosec = stamp?.nanosec ?? 0;
  return `telemetry:${vehicleId}:${sec}:${nanosec}`;
}

function parseReplayAnnotation(rawMessage: ROSLIB.Message): {
  dedupeKey: string;
  routeKey: string;
  deliveryMode: 'live' | 'replayed';
  originalEventTsMs: number;
  replayedAtTsMs: number | null;
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
    const originalEventTsMs = coerceEpochMs(parsed.original_event_ts);
    if (originalEventTsMs === null) {
      return null;
    }
    const replayedAtTsMs = coerceEpochMs(parsed.replayed_at_ts);
    const deliveryMode =
      typeof parsed.delivery_mode === 'string' &&
      parsed.delivery_mode.trim().toLowerCase() === 'replayed'
        ? 'replayed'
        : 'live';
    return {
      dedupeKey,
      routeKey,
      deliveryMode,
      originalEventTsMs,
      replayedAtTsMs,
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
