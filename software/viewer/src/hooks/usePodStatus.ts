import { useEffect, useState } from 'react';
import ROSLIB from 'roslib';
import { useSharedROSConnection } from '@/context/ROSConnectionContext';
import {
  parsePodStatusArray,
  type PodStatusArrayMessage,
} from '@/lib/ros/podStatus';

const EMPTY_STATE: PodStatusArrayMessage = {
  observedAtMs: null,
  pods: [],
};

export function usePodStatus(): PodStatusArrayMessage {
  const { ros, isConnected } = useSharedROSConnection();
  const [state, setState] = useState<PodStatusArrayMessage>(EMPTY_STATE);

  useEffect(() => {
    if (!ros || !isConnected) {
      return;
    }

    const topic = new ROSLIB.Topic({
      ros,
      name: '/device_manager/pods',
      messageType: 'aeris_msgs/PodStatusArray',
    });

    const handleMessage = (message: ROSLIB.Message) => {
      try {
        setState(parsePodStatusArray(message));
      } catch (error) {
        console.warn('[usePodStatus] Ignoring invalid pod status payload:', error);
      }
    };

    topic.subscribe(handleMessage);
    return () => topic.unsubscribe(handleMessage);
  }, [ros, isConnected]);

  return ros && isConnected ? state : EMPTY_STATE;
}
