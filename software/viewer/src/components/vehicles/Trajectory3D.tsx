import React, { useMemo } from 'react';
import { Line } from '@react-three/drei';
import { VehicleState } from '../../lib/vehicle/VehicleManager';

interface Trajectory3DProps {
  vehicle: VehicleState;
}

export function Trajectory3D({ vehicle }: Trajectory3DProps) {
  const { trajectory, color } = vehicle;

  const points = useMemo(() => {
      if (trajectory.length < 2) return [];
      return trajectory;
  }, [trajectory]);

  const vertexColors = useMemo(() => {
      if (points.length < 2) return [];
      const colors = [];
      for (let i = 0; i < points.length; i++) {
          // Gradient: Oldest (0) -> Newest (1)
          const alpha = i / (points.length - 1);
          colors.push([color.r * alpha, color.g * alpha, color.b * alpha]);
      }
      return colors;
  }, [points, color]);

  if (points.length < 2) return null;

  return (
    <Line
      points={points}
      color="white" // Base color white, vertex colors multiply?
      vertexColors={vertexColors as [number, number, number][]}
      lineWidth={2}
      transparent
      opacity={1} // Global opacity
    />
  );
}
