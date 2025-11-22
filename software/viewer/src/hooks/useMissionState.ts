import { useEffect, useState } from 'react';
import ROSLIB from 'roslib';
import { useROSConnection } from './useROSConnection';

export type MissionPhase = 'IDLE' | 'SEARCHING' | 'TRACKING' | 'COMPLETE';

export function useMissionState() {
  const { ros, isConnected } = useROSConnection();
  const [missionState, setMissionState] = useState<MissionPhase>('IDLE');

  useEffect(() => {
    if (!ros || !isConnected) return;

    const topic = new ROSLIB.Topic({
      ros: ros,
      name: '/mission/state',
      messageType: 'std_msgs/String',
    });

    const handleMessage = (message: ROSLIB.Message) => {
      // @ts-expect-error - ROSLIB message typing is loose
      const data = message.data as string;

      // Validate that the received string is a valid MissionPhase
      if (['IDLE', 'SEARCHING', 'TRACKING', 'COMPLETE'].includes(data)) {
          setMissionState(data as MissionPhase);
      } else {
          console.warn(`Received unknown mission state: ${data}`);
      }
    };

    topic.subscribe(handleMessage);
    console.log('[useMissionState] Subscribed to /mission/state');

    return () => {
      topic.unsubscribe();
    };
  }, [ros, isConnected]);

  return missionState;
}
