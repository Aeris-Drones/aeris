import { useEffect, useState, useRef } from 'react';
import { useFrame } from '@react-three/fiber';
import ROSLIB from 'roslib';
import { useROSConnection } from './useROSConnection';

export interface ThermalHotspotData {
  id: string;
  latitude: number;
  longitude: number;
  altitude: number;
  temp_c: number;
  confidence: number;
  lastUpdate: number;
}

interface ThermalHotspotMsg {
  id: string;
  latitude: number;
  longitude: number;
  altitude: number;
  temp_c: number;
  confidence: number;
}

export function useThermalPerception() {
  const { ros, isConnected } = useROSConnection();

  // Store the actual data objects. We mutate these objects in place.
  const hotspotsMap = useRef<Map<string, ThermalHotspotData>>(new Map());

  // State for React to render the list. Only updates on add/remove.
  const [hotspots, setHotspots] = useState<ThermalHotspotData[]>([]);

  // Decay threshold
  const DECAY_MS = 5000;

  useEffect(() => {
    if (!ros || !isConnected) return;

    const topic = new ROSLIB.Topic({
      ros: ros,
      name: '/perception/thermal',
      messageType: 'aeris_msgs/ThermalHotspot',
    });

    const handleMessage = (message: ThermalHotspotMsg) => {
       const { id, latitude, longitude, altitude, temp_c, confidence } = message;
       const now = Date.now();

       let needsRender = false;
       let data = hotspotsMap.current.get(id);

       if (!data) {
           // New hotspot
           data = {
               id,
               latitude,
               longitude,
               altitude,
               temp_c,
               confidence,
               lastUpdate: now
           };
           hotspotsMap.current.set(id, data);
           needsRender = true;
       } else {
           // Update existing (mutation)
           data.latitude = latitude;
           data.longitude = longitude;
           data.altitude = altitude;
           data.temp_c = temp_c;
           data.confidence = confidence;
           data.lastUpdate = now;
       }

       if (needsRender) {
           setHotspots(Array.from(hotspotsMap.current.values()));
       }
    };

    topic.subscribe(handleMessage);

    return () => {
      topic.unsubscribe();
    };
  }, [ros, isConnected]);

  useFrame(() => {
      const now = Date.now();
      let needsRender = false;

      hotspotsMap.current.forEach((data, id) => {
          if (now - data.lastUpdate > DECAY_MS) {
              hotspotsMap.current.delete(id);
              needsRender = true;
          }
      });

      if (needsRender) {
          setHotspots(Array.from(hotspotsMap.current.values()));
      }
  });

  return hotspots;
}
