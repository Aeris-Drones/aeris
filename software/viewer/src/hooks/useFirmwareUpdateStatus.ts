import { useEffect, useState } from 'react';
import ROSLIB from 'roslib';
import { useSharedROSConnection } from '@/context/ROSConnectionContext';
import {
  parseFirmwareUpdateStatusArray,
  type FirmwareUpdateStatusArrayMessage,
} from '@/lib/ros/firmwareUpdateStatus';

const EMPTY_STATE: FirmwareUpdateStatusArrayMessage = {
  observedAtMs: null,
  updates: [],
};

export function useFirmwareUpdateStatus(): FirmwareUpdateStatusArrayMessage {
  const { ros, isConnected } = useSharedROSConnection();
  const [state, setState] = useState<{
    connection: ROSLIB.Ros | null;
    payload: FirmwareUpdateStatusArrayMessage;
  }>({
    connection: null,
    payload: EMPTY_STATE,
  });

  useEffect(() => {
    if (!ros || !isConnected) {
      return;
    }

    const topic = new ROSLIB.Topic({
      ros,
      name: '/vehicle/firmware_update_status',
      messageType: 'aeris_msgs/FirmwareUpdateStatusArray',
    });

    const handleMessage = (message: ROSLIB.Message) => {
      try {
        setState({
          connection: ros,
          payload: parseFirmwareUpdateStatusArray(message),
        });
      } catch (error) {
        console.warn('[useFirmwareUpdateStatus] Ignoring invalid firmware update payload:', error);
      }
    };

    topic.subscribe(handleMessage);
    return () => {
      topic.unsubscribe(handleMessage);
      setState({
        connection: null,
        payload: EMPTY_STATE,
      });
    };
  }, [ros, isConnected]);

  return ros && isConnected && state.connection === ros ? state.payload : EMPTY_STATE;
}
