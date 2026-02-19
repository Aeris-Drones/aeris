import type { Detection } from '@/components/sheets/DetectionCard';
import type { VehicleState } from '@/lib/vehicle/VehicleManager';
import type ROSLIB from 'roslib';

export interface MergeLiveDetectionsOptions {
  nowMs?: number;
  maxAgeMs?: number;
  maxDetections?: number;
}

export function toVehicleName(vehicleId: string): string;
export function findNearestVehicle(
  vehicles: VehicleState[],
  position: [number, number, number]
): VehicleState | undefined;
export function mergeLiveDetections(
  previous: Detection[],
  incomingDetection: Detection | null,
  options?: MergeLiveDetectionsOptions
): Detection[];
export function subscribeToFusedTopic(options: {
  topicFactory: (args: { ros: ROSLIB.Ros; name: string; messageType: string }) => {
    subscribe: (callback: (message: ROSLIB.Message) => void) => void;
    unsubscribe: () => void;
  };
  ros: ROSLIB.Ros;
  topicName: string;
  messageType: string;
  onMessage: (message: ROSLIB.Message) => void;
}): () => void;
