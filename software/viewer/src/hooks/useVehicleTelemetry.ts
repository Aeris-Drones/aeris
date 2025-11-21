import { useEffect, useState } from 'react';
import ROSLIB from 'roslib';
import { useROSConnection } from './useROSConnection';
import { VehicleManager, VehicleState } from '../lib/vehicle/VehicleManager';
import { VehicleTelemetryMessage } from '../lib/ros/telemetry';
import { useCoordinateOrigin } from '../context/CoordinateOriginContext';

export function useVehicleTelemetry() {
  const { ros, isConnected } = useROSConnection();
  const { origin, setOrigin } = useCoordinateOrigin();
  const [vehicles, setVehicles] = useState<VehicleState[]>([]);

  // Initialize manager once using useState initializer
  const [manager] = useState(() => new VehicleManager());

  useEffect(() => {
    if (!ros || !isConnected) return;

    const topic = new ROSLIB.Topic({
      ros: ros,
      name: '/vehicle/telemetry',
      messageType: 'aeris_msgs/Telemetry',
    });

    const handleMessage = (message: ROSLIB.Message) => {
      const telemetry = message as unknown as VehicleTelemetryMessage;
      const newOrigin = manager.processTelemetry(telemetry, origin);

      if (newOrigin) {
        setOrigin(newOrigin);
      }

      // Trigger React update
      setVehicles([...manager.getVehicles()]);
    };

    topic.subscribe(handleMessage);
    console.log('[useVehicleTelemetry] Subscribed to /vehicle/telemetry');

    return () => {
      topic.unsubscribe();
    };
  }, [ros, isConnected, origin, setOrigin, manager]);

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
