import { useEffect, useRef, useState } from 'react';
import ROSLIB from 'roslib';
import { useROSConnection } from './useROSConnection';
import { useCoordinateOrigin } from '../context/CoordinateOriginContext';
import { geoToLocal } from '../lib/ros/mapTile';

export interface GasPolygonLocal {
  points: { x: number; z: number; y: number }[];
}

export interface GasCenterlinePointLocal {
  x: number;
  z: number;
  y: number;
}

export interface GasPlume {
  polygons: GasPolygonLocal[];
  centerline: GasCenterlinePointLocal[];
  species: string;
  units: string;
  timestamp: number;
}

interface GasIsoplethMsg {
  species?: string;
  units?: string;
  polygons?: { points: { x: number; y: number; z: number }[] }[];
  centerline?: { x: number; y: number; z: number }[];
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

    const handleMessage = (message: ROSLIB.Message) => {
      const msg = message as GasIsoplethMsg;
      if (!origin) return; // cannot localize without an origin

      const localPolygons: GasPolygonLocal[] = (msg.polygons || []).map((poly) => {
        const points = (poly.points || []).map((pt) => {
          const local = geoToLocal({ lat: pt.x, lon: pt.y }, origin);
          return { x: local.x, z: local.z, y: pt.z ?? 0 };
        });
        return { points };
      });

      const localCenterline: GasCenterlinePointLocal[] = (msg.centerline || []).map((pt) => {
        const local = geoToLocal({ lat: pt.x, lon: pt.y }, origin);
        return { x: local.x, z: local.z, y: pt.z ?? 0 };
      });

      const plume: GasPlume = {
        polygons: localPolygons,
        centerline: localCenterline,
        species: msg.species ?? '',
        units: msg.units ?? '',
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
