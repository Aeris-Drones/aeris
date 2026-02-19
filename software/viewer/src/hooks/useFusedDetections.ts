'use client';

import { useEffect, useMemo, useState } from 'react';
import ROSLIB from 'roslib';
import type { Detection } from '@/components/sheets/DetectionCard';
import type { VehicleState } from '@/lib/vehicle/VehicleManager';
import { normalizeFusedDetectionMessage } from '@/lib/ros/fusedDetections';
import { useROSConnection } from './useROSConnection';

interface UseFusedDetectionsOptions {
  topicName?: string;
  messageType?: string;
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
  maxDetections: 250,
  maxAgeMs: 5 * 60 * 1000,
  maxFutureSkewMs: 5_000,
  fallbackVehicleId: 'fusion',
};

function toVehicleName(vehicleId: string): string {
  if (!vehicleId.trim()) {
    return 'Unknown Vehicle';
  }
  return vehicleId.replace(/[_-]/g, ' ').toUpperCase();
}

function findNearestVehicle(
  vehicles: VehicleState[],
  [x, , z]: [number, number, number]
): VehicleState | undefined {
  if (vehicles.length === 0) {
    return undefined;
  }

  let closestVehicle: VehicleState | undefined;
  let closestDistance = Number.POSITIVE_INFINITY;

  for (const vehicle of vehicles) {
    const dx = vehicle.position.x - x;
    const dz = vehicle.position.z - z;
    const distance = Math.sqrt((dx * dx) + (dz * dz));
    if (distance < closestDistance) {
      closestDistance = distance;
      closestVehicle = vehicle;
    }
  }

  return closestVehicle;
}

export function useFusedDetections(
  vehicles: VehicleState[],
  options: UseFusedDetectionsOptions = {}
): UseFusedDetectionsResult {
  const merged = { ...DEFAULT_OPTIONS, ...options };
  const { ros, isConnected } = useROSConnection();
  const [detections, setDetections] = useState<Detection[]>([]);

  const fallbackVehicleName = useMemo(
    () => toVehicleName(merged.fallbackVehicleId),
    [merged.fallbackVehicleId]
  );

  useEffect(() => {
    if (!ros || !isConnected) {
      return;
    }

    const topic = new ROSLIB.Topic({
      ros,
      name: merged.topicName,
      messageType: merged.messageType,
    });

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

      const nearestVehicle = findNearestVehicle(vehicles, normalized.position);
      const detection: Detection = nearestVehicle
        ? {
            ...normalized,
            vehicleId: nearestVehicle.id,
            vehicleName: toVehicleName(nearestVehicle.id),
          }
        : normalized;

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
          .sort((left, right) => right.timestamp - left.timestamp)
          .slice(0, merged.maxDetections);
      });
    };

    topic.subscribe(handleMessage);

    return () => {
      topic.unsubscribe();
    };
  }, [
    fallbackVehicleName,
    isConnected,
    merged.fallbackVehicleId,
    merged.maxAgeMs,
    merged.maxDetections,
    merged.maxFutureSkewMs,
    merged.messageType,
    merged.topicName,
    ros,
    vehicles,
  ]);

  return { detections, isConnected };
}
