import { useEffect, useMemo, useRef, useState } from 'react';
import ROSLIB from 'roslib';
import * as THREE from 'three';
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

/** Wind information derived from plume centerline data */
export interface WindInfo {
  /** Normalized direction vector of wind flow */
  direction: THREE.Vector3;
  /** Estimated wind speed in m/s (default: 2.0) */
  speed: number;
}

interface GasIsoplethMsg {
  species?: string;
  units?: string;
  polygons?: { points: { x: number; y: number; z: number }[] }[];
  centerline?: { x: number; y: number; z: number }[];
}

const PLUME_TTL_MS = 5000; // 5 seconds, slower decay than acoustic
const DEFAULT_WIND_SPEED = 2.0; // m/s - used when no speed data available
const DEFAULT_FALLBACK_WIND = new THREE.Vector3(1.0, 0.2, 0.5).normalize();

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

  // Derive wind direction and speed from centerline data
  const wind = useMemo((): WindInfo => {
    // Find a plume with valid centerline data
    const plumeWithCenterline = plumes.find((plume) => plume.centerline.length >= 2);

    if (!plumeWithCenterline) {
      // No centerline data available, use fallback wind direction
      return {
        direction: DEFAULT_FALLBACK_WIND.clone(),
        speed: DEFAULT_WIND_SPEED,
      };
    }

    const centerline = plumeWithCenterline.centerline;
    const first = centerline[0];
    const last = centerline[centerline.length - 1];

    // Calculate direction vector from start to end of centerline
    const direction = new THREE.Vector3(
      last.x - first.x,
      0, // Keep wind horizontal for overlay visualization
      last.z - first.z
    );

    // If centerline points are too close, use fallback
    if (direction.lengthSq() < 1e-6) {
      return {
        direction: DEFAULT_FALLBACK_WIND.clone(),
        speed: DEFAULT_WIND_SPEED,
      };
    }

    // Normalize direction and estimate speed from centerline length
    // Speed estimation: use centerline length as a rough proxy
    // (in real implementation, this would come from sensor data)
    const centerlineLength = direction.length();
    direction.normalize();

    // Estimate speed: longer centerlines suggest faster wind (clamped 1-5 m/s)
    const estimatedSpeed = Math.min(5.0, Math.max(1.0, centerlineLength / 20));

    return {
      direction,
      speed: estimatedSpeed,
    };
  }, [plumes]);

  return { plumes, wind };
}
