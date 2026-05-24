import { useEffect, useState } from 'react';
import ROSLIB from 'roslib';
import { useSharedROSConnection } from '@/context/ROSConnectionContext';
import {
  parsePodInventoryArray,
  type PodInventoryArrayMessage,
} from '@/lib/ros/podInventory';

const EMPTY_STATE: PodInventoryArrayMessage = {
  observedAtMs: null,
  records: [],
};

export function usePodInventory(): PodInventoryArrayMessage {
  const { ros, isConnected } = useSharedROSConnection();
  const [state, setState] = useState<{
    connection: ROSLIB.Ros | null;
    payload: PodInventoryArrayMessage;
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
      name: '/device_manager/pod_inventory',
      messageType: 'aeris_msgs/PodInventoryArray',
    });

    const handleMessage = (message: ROSLIB.Message) => {
      try {
        setState({
          connection: ros,
          payload: parsePodInventoryArray(message),
        });
      } catch (error) {
        console.warn('[usePodInventory] Ignoring invalid pod inventory payload:', error);
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
