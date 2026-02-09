import { useEffect, useState } from 'react';
import ROSLIB from 'roslib';
import { useROSConnection } from './useROSConnection';

const MISSION_PHASES = [
  'IDLE',
  'PLANNING',
  'SEARCHING',
  'TRACKING',
  'COMPLETE',
  'ABORTED',
] as const;
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
      try {
        const data = typeof (message as { data?: unknown }).data === 'string'
          ? (message as { data: string }).data
          : 'IDLE';
        const parsed = data.trim();
        let state = parsed;

        if (parsed.startsWith('{')) {
          const payload = JSON.parse(parsed) as { phase?: string; state?: string };
          state = payload.phase ?? payload.state ?? 'IDLE';
        }

        if (MISSION_PHASES.includes(state as MissionPhase)) {
          setMissionState(state as MissionPhase);
        } else {
          console.warn(`Received unknown mission state: ${state}`);
        }
      } catch (error) {
        console.warn('Failed to parse mission state message:', error);
      }
    };

    topic.subscribe(handleMessage);

    return () => {
      topic.unsubscribe();
    };
  }, [ros, isConnected]);

  return missionState;
}
