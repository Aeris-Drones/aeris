import { useEffect, useState, useRef } from 'react';
import ROSLIB from 'roslib';
import { useROSConnection } from './useROSConnection';
import { VehicleManager, VehicleState } from '../lib/vehicle/VehicleManager';
import { parseVehicleTelemetry } from '../lib/ros/telemetry';
import { useCoordinateOrigin } from '../context/CoordinateOriginContext';

type ReturnTrajectoryMap = Record<string, [number, number, number][]>;

/**
 * Hook for managing vehicle telemetry data from ROS.
 *
 * Subscribes to two ROS topics:
 * - /vehicle/telemetry: Real-time vehicle state (position, orientation, velocity)
 * - /mission/progress: Mission status including return-to-launch trajectories
 *
 * The VehicleManager handles coordinate transformations and state tracking.
 * Coordinate origin is auto-set from the first received telemetry if not already set.
 */
export function useVehicleTelemetry() {
  const { ros, isConnected } = useROSConnection();
  const { origin, setOrigin } = useCoordinateOrigin();
  const [vehicles, setVehicles] = useState<VehicleState[]>([]);
  const [returnTrajectories, setReturnTrajectories] = useState<ReturnTrajectoryMap>({});

  // VehicleManager persists across renders to maintain trajectory history
  const [manager] = useState(() => new VehicleManager());

  // Refs for accessing current origin inside ROS callbacks without re-subscribing
  const originRef = useRef(origin);
  const setOriginRef = useRef(setOrigin);

  useEffect(() => {
    originRef.current = origin;
    setOriginRef.current = setOrigin;
  }, [origin, setOrigin]);

  useEffect(() => {
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

    /**
     * Handles incoming telemetry messages.
     *
     * Parses the ROS message, updates vehicle state via VehicleManager,
     * and auto-sets coordinate origin if this is the first message.
     */
    const handleMessage = (message: ROSLIB.Message) => {
      let telemetry;
      try {
        telemetry = parseVehicleTelemetry(message);
      } catch (error) {
        console.warn('[useVehicleTelemetry] Ignoring invalid telemetry payload:', error);
        return;
      }

      const newOrigin = manager.processTelemetry(telemetry, originRef.current);

      if (newOrigin) {
        setOriginRef.current(newOrigin);
      }

      setVehicles([...manager.getVehicles()]);
    };

    /**
     * Handles mission progress messages containing return trajectory data.
     *
     * Payload format (JSON string in std_msgs/String.data):
     * {
     *   vehicleId: string,
     *   returnTrajectory: {
     *     vehicleId: string,
     *     points: Array<{ x: number, z: number, altitude_m?: number, altitudeM?: number }>
     *   } | null
     * }
     *
     * Coordinates are converted from ROS (x=East, z=North) to Three.js (x, y=altitude, z).
     * Trajectories with fewer than 2 valid points are discarded.
     */
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
        const vehicleId = payload.returnTrajectory.vehicleId?.trim();
        if (!vehicleId) {
          return;
        }
        const points = (payload.returnTrajectory.points ?? [])
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

    topic.subscribe(handleMessage);
    progressTopic.subscribe(handleProgressMessage);

    return () => {
      topic.unsubscribe();
      progressTopic.unsubscribe();
    };
  }, [ros, isConnected, manager]);

  /**
   * Periodic cleanup effect: removes stale vehicles from React state.
   *
   * The VehicleManager.getVehicles() method filters out vehicles that haven't
   * received updates within the cleanup threshold (10 seconds). This effect
   * ensures the UI reflects those removals.
   *
   * Runs every 1000ms; comparison by length avoids unnecessary re-renders.
   */
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
