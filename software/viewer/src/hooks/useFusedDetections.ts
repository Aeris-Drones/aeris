'use client';

import { useEffect, useMemo, useRef, useState } from 'react';
import ROSLIB from 'roslib';
import type { Detection } from '@/components/sheets/DetectionCard';
import type { VehicleState } from '@/lib/vehicle/VehicleManager';
import { normalizeFusedDetectionMessage } from '@/lib/ros/fusedDetections';
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

export function useFusedDetections(
  vehicles: VehicleState[],
  options: UseFusedDetectionsOptions = {}
): UseFusedDetectionsResult {
  const merged = { ...DEFAULT_OPTIONS, ...options };
  const { ros, isConnected } = useROSConnection();
  const [detections, setDetections] = useState<Detection[]>([]);
  const vehiclesRef = useRef<VehicleState[]>(vehicles);

  const fallbackVehicleName = useMemo(
    () => toVehicleName(merged.fallbackVehicleId),
    [merged.fallbackVehicleId]
  );

  useEffect(() => {
    vehiclesRef.current = vehicles;
  }, [vehicles]);

  useEffect(() => {
    if (!ros || !isConnected) {
      return;
    }

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

      setDetections((previous) => {
        return mergeLiveDetections(previous, detection, {
          maxAgeMs: merged.maxAgeMs,
          maxDetections: merged.maxDetections,
        });
      });
    };

    return subscribeToFusedTopic({
      topicFactory: (args) => new ROSLIB.Topic(args),
      ros,
      topicName: merged.topicName,
      messageType: merged.messageType,
      onMessage: handleMessage,
    });
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
  ]);

  return { detections, isConnected };
}
