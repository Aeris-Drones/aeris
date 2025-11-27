import { useEffect, useRef, useState } from 'react';
import ROSLIB from 'roslib';
import { useROSConnection } from './useROSConnection';
import { useCoordinateOrigin } from '../context/CoordinateOriginContext';
import { geoToLocal } from '../lib/ros/mapTile';

export interface GasPolygonLocal {
  points: { x: number; z: number; y: number }[];
}

export interface GasPlume {
  polygons: GasPolygonLocal[];
  species: string;
  units: string;
  timestamp: number;
}

interface GasIsoplethMsg {
  species: string;
  units: string;
  polygons: { points: { x: number; y: number; z: number }[] }[];
}

const PLUME_TTL_MS = 5000; // 5 seconds, slower decay than acoustic

export function useGasPerception() {
  const { ros, isConnected } = useROSConnection();
  const { origin } = useCoordinateOrigin();

  const plumesRef = useRef<GasPlume[]>([]);
  const [plumes, setPlumes] = useState<GasPlume[]>([]);

  useEffect(() => {
    if (!ros || !isConnected) return;

    const topic = new ROSLIB.Topic({
      ros: ros,
      name: '/perception/gas',
      messageType: 'aeris_msgs/GasIsopleth',
    });

    const handleMessage = (message: any) => {
      const msg = message as GasIsoplethMsg;
      if (!origin) return; // cannot localize without an origin

      const localPolygons: GasPolygonLocal[] = (msg.polygons || []).map((poly: any) => {
        const points = (poly.points || []).map((pt: any) => {
          const local = geoToLocal({ lat: pt.x, lon: pt.y }, origin);
          return { x: local.x, z: local.z, y: pt.z ?? 0 };
        });
        return { points };
      });

      const plume: GasPlume = {
        polygons: localPolygons,
        species: msg.species,
        units: msg.units,
        timestamp: Date.now(),
      };

      plumesRef.current = [plume];
      setPlumes([...plumesRef.current]);
    };

    topic.subscribe(handleMessage);

    return () => {
      topic.unsubscribe();
    };
  }, [ros, isConnected, origin]);

  useEffect(() => {
    const interval = setInterval(() => {
      const now = Date.now();
      const next = plumesRef.current.filter((p) => now - p.timestamp <= PLUME_TTL_MS);
      if (next.length !== plumesRef.current.length) {
        plumesRef.current = next;
        setPlumes([...next]);
      }
    }, 500);

    return () => clearInterval(interval);
  }, []);

  return { plumes };
}
