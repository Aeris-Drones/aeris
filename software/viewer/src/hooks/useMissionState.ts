import { useEffect, useState } from 'react';
import ROSLIB from 'roslib';
import { useROSConnection } from './useROSConnection';

const MISSION_PHASES = ['IDLE', 'SEARCHING', 'TRACKING', 'COMPLETE'] as const;
export type MissionPhase = typeof MISSION_PHASES[number];

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

      if (MISSION_PHASES.includes(data as MissionPhase)) {
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
