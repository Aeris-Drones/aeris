import React from 'react';
import { VehicleState } from '../../lib/vehicle/VehicleManager';
import { Line } from '@react-three/drei';
import * as THREE from 'three';

interface AltitudeIndicatorProps {
  vehicle: VehicleState;
}

export function AltitudeIndicator({ vehicle }: AltitudeIndicatorProps) {
  const { position } = vehicle;

  // Line from vehicle position (x,y,z) to ground (x,0,z)
  const points = [
    position,
    new THREE.Vector3(position.x, 0, position.z)
  ];

  return (
    <Line
      points={points}
      color="white"
      lineWidth={1}
      dashed
      dashScale={5}
      gapSize={2}
      opacity={0.2}
      transparent
    />
  );
}
