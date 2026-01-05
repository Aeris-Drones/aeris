import { useEffect, useState, useRef } from 'react';
import ROSLIB from 'roslib';
import { useROSConnection } from './useROSConnection';

interface AcousticBearing {
  vehicle_id: string;
  bearing_deg: number;
  confidence: number;
  snr_db: number;
  classification: string;
  timestamp: number;
}

interface AcousticBearingMsg {
  vehicle_id: string;
  bearing_deg: number;
  confidence: number;
  snr_db: number;
  classification: string;
}

export function useAcousticPerception() {
  const { ros, isConnected } = useROSConnection();
  const [detections, setDetections] = useState<Map<string, AcousticBearing>>(new Map());
  const detectionsRef = useRef<Map<string, AcousticBearing>>(new Map());

  const DETECTION_TTL = 2000; // milliseconds

  useEffect(() => {
    if (!ros || !isConnected) return;

    const topic = new ROSLIB.Topic({
      ros: ros,
      name: '/perception/acoustic',
      messageType: 'aeris_msgs/AcousticBearing',
    });

    const handleMessage = (message: ROSLIB.Message) => {
      const msg = message as AcousticBearingMsg;
      const detection: AcousticBearing = {
        vehicle_id: msg.vehicle_id,
        bearing_deg: msg.bearing_deg,
        confidence: msg.confidence,
        snr_db: msg.snr_db,
        classification: msg.classification,
        timestamp: Date.now(),
      };

      detectionsRef.current.set(detection.vehicle_id, detection);
      setDetections(new Map(detectionsRef.current));
    };

    topic.subscribe(handleMessage);

    return () => {
      topic.unsubscribe();
    };
  }, [ros, isConnected]);

  useEffect(() => {
    const interval = setInterval(() => {
        const now = Date.now();
        let changed = false;
        for (const [id, det] of detectionsRef.current.entries()) {
            if (now - det.timestamp > DETECTION_TTL) {
                detectionsRef.current.delete(id);
                changed = true;
            }
        }
        if (changed) {
            setDetections(new Map(detectionsRef.current));
        }
    }, 500);
    return () => clearInterval(interval);
  }, []);

  return { detections };
}
