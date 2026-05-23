import { useEffect, useState } from 'react';
import ROSLIB from 'roslib';
import { useSharedROSConnection } from '@/context/ROSConnectionContext';
import {
  parseVehicleMaintenanceDiagnosticsArray,
  type VehicleMaintenanceDiagnosticsArrayMessage,
} from '@/lib/ros/maintenanceDiagnostics';

const EMPTY_STATE: VehicleMaintenanceDiagnosticsArrayMessage = {
  observedAtMs: null,
  vehicles: [],
};

export function useVehicleMaintenanceDiagnostics(): VehicleMaintenanceDiagnosticsArrayMessage {
  const { ros, isConnected } = useSharedROSConnection();
  const [state, setState] = useState<VehicleMaintenanceDiagnosticsArrayMessage>(EMPTY_STATE);

  useEffect(() => {
    if (!ros || !isConnected) {
      return;
    }

    const topic = new ROSLIB.Topic({
      ros,
      name: '/vehicle/maintenance_diagnostics',
      messageType: 'aeris_msgs/VehicleMaintenanceDiagnosticsArray',
    });

    const handleMessage = (message: ROSLIB.Message) => {
      try {
        setState(parseVehicleMaintenanceDiagnosticsArray(message));
      } catch (error) {
        console.warn('[useVehicleMaintenanceDiagnostics] Ignoring invalid maintenance payload:', error);
      }
    };

    topic.subscribe(handleMessage);
    return () => topic.unsubscribe(handleMessage);
  }, [ros, isConnected]);

  return ros && isConnected ? state : EMPTY_STATE;
}
