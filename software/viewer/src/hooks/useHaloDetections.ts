'use client';

import { useEffect, useState } from 'react';
import ROSLIB from 'roslib';
import type { Detection } from '@/components/sheets/DetectionCard';
import {
  normalizeHaloConfidenceEventMessage,
  normalizeHaloEvidenceLogRecord,
  normalizeHaloEvidenceReplayPayload,
} from '@/lib/ros/haloConfidenceEvents';
import { mergeLiveDetections } from '@/lib/ros/fusedDetectionsFeed';
import { useSharedROSConnection } from '@/context/ROSConnectionContext';

interface UseHaloDetectionsOptions {
  topicName?: string;
  messageType?: string;
  maxDetections?: number;
  maxAgeMs?: number;
}

interface UseHaloDetectionsResult {
  detections: Detection[];
  isConnected: boolean;
}

const DEFAULT_OPTIONS: Required<UseHaloDetectionsOptions> = {
  topicName: '/halo/confidence_events',
  messageType: 'std_msgs/String',
  maxDetections: 100,
  maxAgeMs: 10 * 60 * 1000,
};
const MAX_HALO_PAYLOAD_STRING_DEPTH = 3;

export function useHaloDetections(
  options: UseHaloDetectionsOptions = {}
): UseHaloDetectionsResult {
  const merged = { ...DEFAULT_OPTIONS, ...options };
  const { ros, isConnected } = useSharedROSConnection();
  const [detectionState, setDetectionState] = useState<{
    connection: ROSLIB.Ros | null;
    detections: Detection[];
  }>({
    connection: null,
    detections: [],
  });

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
      let normalizedDetections: Detection[];
      try {
        normalizedDetections = normalizeHaloInboundPayload(rawMessage);
      } catch (error) {
        console.warn('[useHaloDetections] Ignoring malformed Halo payload:', error);
        return;
      }

      setDetectionState((previous) => {
        const currentDetections = previous.connection === ros ? previous.detections : [];
        const nextDetections = normalizedDetections.reduce(
          (accumulator, detection) =>
            mergeLiveDetections(accumulator, detection, {
              maxAgeMs: merged.maxAgeMs,
              maxDetections: merged.maxDetections,
            }),
          currentDetections
        );
        return {
          connection: ros,
          detections: nextDetections,
        };
      });
    };

    topic.subscribe(handleMessage);

    return () => {
      topic.unsubscribe();
      setDetectionState({ connection: null, detections: [] });
    };
  }, [isConnected, merged.maxAgeMs, merged.maxDetections, merged.messageType, merged.topicName, ros]);

  return {
    detections:
      isConnected && detectionState.connection === ros
        ? detectionState.detections
        : [],
    isConnected,
  };
}

export function normalizeHaloInboundPayload(
  rawMessage: ROSLIB.Message,
  depth = 0
): Detection[] {
  const payload = (rawMessage as { data?: unknown }).data ?? rawMessage;

  if (typeof payload === 'string') {
    const trimmed = payload.trim();
    if (!trimmed) {
      return [];
    }
    if (depth >= MAX_HALO_PAYLOAD_STRING_DEPTH) {
      throw new Error('Invalid Halo payload: string nesting depth exceeded');
    }
    let parsed: ROSLIB.Message;
    try {
      parsed = JSON.parse(trimmed) as ROSLIB.Message;
    } catch (error) {
      if (trimmed.includes('\n')) {
        return normalizeHaloEvidenceReplayPayload(trimmed);
      }
      throw error;
    }
    return normalizeHaloInboundPayload(parsed, depth + 1);
  }

  if (Array.isArray(payload)) {
    return normalizeHaloEvidenceReplayPayload(payload);
  }

  if (payload && typeof payload === 'object' && 'event' in payload) {
    return [normalizeHaloEvidenceLogRecord(payload)];
  }

  return [normalizeHaloConfidenceEventMessage(payload)];
}
