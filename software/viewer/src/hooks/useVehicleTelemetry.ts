import { useEffect, useState, useRef } from 'react';
import ROSLIB from 'roslib';
import { useROSConnection } from './useROSConnection';
import { VehicleManager, VehicleState } from '../lib/vehicle/VehicleManager';
import { parseVehicleTelemetry } from '../lib/ros/telemetry';
import { useCoordinateOrigin } from '../context/CoordinateOriginContext';

export function useVehicleTelemetry() {
  const { ros, isConnected } = useROSConnection();
  const { origin, setOrigin } = useCoordinateOrigin();
  const [vehicles, setVehicles] = useState<VehicleState[]>([]);

  // Initialize manager once using useState initializer
  const [manager] = useState(() => new VehicleManager());
  // Use refs to access latest origin/setOrigin without triggering effect re-runs
  const originRef = useRef(origin);
  const setOriginRef = useRef(setOrigin);

  // Keep refs in sync with latest values
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

    const handleMessage = (message: ROSLIB.Message) => {
      const telemetry = parseVehicleTelemetry(message);
      const newOrigin = manager.processTelemetry(telemetry, originRef.current);

      if (newOrigin) {
        setOriginRef.current(newOrigin);
      }

      // Trigger React update
      setVehicles([...manager.getVehicles()]);
    };

    topic.subscribe(handleMessage);
    console.log('[useVehicleTelemetry] Subscribed to /vehicle/telemetry');

    return () => {
      topic.unsubscribe();
    };
  }, [ros, isConnected, manager]);

  // Optional: Prune stale vehicles periodically
  useEffect(() => {
      const interval = setInterval(() => {
          const currentVehicles = manager.getVehicles();
          if (currentVehicles.length !== vehicles.length) {
             setVehicles([...currentVehicles]);
          }
      }, 1000);
      return () => clearInterval(interval);
  }, [vehicles.length, manager]);

  return { vehicles, manager };
}
