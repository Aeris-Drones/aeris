'use client';

import { useEffect, useMemo, useState } from 'react';
import ROSLIB from 'roslib';
import type { Detection } from '@/components/sheets/DetectionCard';
import type { VehicleState } from '@/lib/vehicle/VehicleManager';
import { useROSConnection } from './useROSConnection';

interface ThermalHotspotMessage {
  stamp?: { sec?: number; nanosec?: number };
  bbox_px?: unknown;
  temp_c?: unknown;
  confidence?: unknown;
  frame_id?: unknown;
}

interface UseThermalHotspotsOptions {
  topicName?: string;
  messageType?: string;
  maxDetections?: number;
  maxAgeMs?: number;
  pixelsToMeters?: number;
  imageCenterPx?: [number, number];
  fallbackVehicleId?: string;
}

interface UseThermalHotspotsResult {
  detections: Detection[];
  isConnected: boolean;
}

const DEFAULT_OPTIONS: Required<UseThermalHotspotsOptions> = {
  topicName: '/thermal/hotspots',
  messageType: 'aeris_msgs/ThermalHotspot',
  maxDetections: 200,
  maxAgeMs: 5 * 60 * 1000,
  pixelsToMeters: 0.1,
  imageCenterPx: [320, 240],
  fallbackVehicleId: 'scout1',
};

function normalizeVehicleKey(value: string): string {
  return value.trim().toLowerCase().replace(/[^a-z0-9]/g, '');
}

function toVehicleName(vehicleId: string): string {
  if (!vehicleId.trim()) {
    return 'Unknown Vehicle';
  }
  return vehicleId.replace(/[_-]/g, ' ').toUpperCase();
}

function parseTimestampMs(message: ThermalHotspotMessage): number {
  const sec = Number(message.stamp?.sec);
  const nanosec = Number(message.stamp?.nanosec);
  if (Number.isFinite(sec) && sec > 0) {
    const safeNanosec = Number.isFinite(nanosec) ? nanosec : 0;
    return (sec * 1000) + (safeNanosec / 1_000_000);
  }
  return Date.now();
}

function parseBoundingBox(
  rawBbox: unknown,
  [centerX, centerY]: [number, number]
): [number, number, number, number] {
  if (!Array.isArray(rawBbox) || rawBbox.length < 4) {
    return [centerX - 5, centerY - 5, centerX + 5, centerY + 5];
  }
  const values = rawBbox.slice(0, 4).map((value) => Number(value));
  if (!values.every((value) => Number.isFinite(value))) {
    return [centerX - 5, centerY - 5, centerX + 5, centerY + 5];
  }
  const xMin = Math.min(values[0], values[2]);
  const xMax = Math.max(values[0], values[2]);
  const yMin = Math.min(values[1], values[3]);
  const yMax = Math.max(values[1], values[3]);
  return [xMin, yMin, xMax, yMax];
}

function inferVehicleToken(frameId: string): string | null {
  const tokenMatch = frameId.match(/(scout[_-]?\d+|ranger[_-]?\d+)/i);
  if (!tokenMatch) {
    return null;
  }
  return tokenMatch[1] ?? null;
}

export function useThermalHotspots(
  vehicles: VehicleState[],
  options: UseThermalHotspotsOptions = {}
): UseThermalHotspotsResult {
  const merged = { ...DEFAULT_OPTIONS, ...options };
  const { ros, isConnected } = useROSConnection();
  const [detections, setDetections] = useState<Detection[]>([]);

  const vehicleByKey = useMemo(() => {
    const lookup = new Map<string, VehicleState>();
    for (const vehicle of vehicles) {
      lookup.set(normalizeVehicleKey(vehicle.id), vehicle);
    }
    return lookup;
  }, [vehicles]);

  const defaultVehicle = useMemo(() => {
    const explicit = vehicleByKey.get(normalizeVehicleKey(merged.fallbackVehicleId));
    if (explicit) {
      return explicit;
    }
    const scout = vehicles.find((vehicle) => vehicle.id.toLowerCase().startsWith('scout'));
    if (scout) {
      return scout;
    }
    return vehicles[0];
  }, [merged.fallbackVehicleId, vehicleByKey, vehicles]);

  useEffect(() => {
    if (!ros || !isConnected) {
      return;
    }

    const resolveVehicle = (frameId: string): VehicleState | undefined => {
      const frameToken = inferVehicleToken(frameId);
      if (frameToken) {
        const byToken = vehicleByKey.get(normalizeVehicleKey(frameToken));
        if (byToken) {
          return byToken;
        }
      }

      const normalizedFrame = normalizeVehicleKey(frameId);
      for (const [normalizedVehicleId, vehicle] of vehicleByKey.entries()) {
        if (normalizedFrame.includes(normalizedVehicleId)) {
          return vehicle;
        }
      }
      return defaultVehicle;
    };

    const topic = new ROSLIB.Topic({
      ros,
      name: merged.topicName,
      messageType: merged.messageType,
    });

    const handleMessage = (rawMessage: ROSLIB.Message) => {
      const message = rawMessage as unknown as ThermalHotspotMessage;
      const frameId = typeof message.frame_id === 'string' ? message.frame_id : '';
      const vehicle = resolveVehicle(frameId);

      const [xMin, yMin, xMax, yMax] = parseBoundingBox(
        message.bbox_px,
        merged.imageCenterPx
      );
      const centerX = (xMin + xMax) / 2;
      const centerY = (yMin + yMax) / 2;

      const baseX = vehicle?.position.x ?? 0;
      const baseY = vehicle?.position.y ?? 0;
      const baseZ = vehicle?.position.z ?? 0;

      const offsetX = (centerX - merged.imageCenterPx[0]) * merged.pixelsToMeters;
      const offsetZ = (centerY - merged.imageCenterPx[1]) * merged.pixelsToMeters;

      const fallbackVehicleId = defaultVehicle?.id ?? merged.fallbackVehicleId;
      const inferredVehicleId = inferVehicleToken(frameId) ?? '';
      const vehicleId = vehicle?.id || inferredVehicleId || fallbackVehicleId;

      const confidenceRaw = Number(message.confidence);
      const confidence = Number.isFinite(confidenceRaw)
        ? Math.max(0, Math.min(1, confidenceRaw))
        : 0;
      const tempRaw = Number(message.temp_c);
      const temperature = Number.isFinite(tempRaw) ? tempRaw : undefined;
      const timestamp = parseTimestampMs(message);

      const detection: Detection = {
        id: `thermal-${timestamp}-${xMin}-${yMin}-${xMax}-${yMax}-${vehicleId}`,
        sensorType: 'thermal',
        confidence,
        timestamp,
        status: 'new',
        vehicleId,
        vehicleName: toVehicleName(vehicleId),
        position: [baseX + offsetX, baseY, baseZ + offsetZ],
        temperature,
      };

      setDetections((previous) => {
        const cutoff = Date.now() - merged.maxAgeMs;
        const byId = new Map<string, Detection>();
        for (const prior of previous) {
          if (prior.timestamp >= cutoff) {
            byId.set(prior.id, prior);
          }
        }
        byId.set(detection.id, detection);
        return Array.from(byId.values())
          .sort((a, b) => b.timestamp - a.timestamp)
          .slice(0, merged.maxDetections);
      });
    };

    topic.subscribe(handleMessage);

    return () => {
      topic.unsubscribe();
    };
  }, [
    defaultVehicle,
    isConnected,
    merged.imageCenterPx,
    merged.maxAgeMs,
    merged.maxDetections,
    merged.messageType,
    merged.pixelsToMeters,
    merged.topicName,
    merged.fallbackVehicleId,
    ros,
    vehicleByKey,
  ]);

  return { detections, isConnected };
}
