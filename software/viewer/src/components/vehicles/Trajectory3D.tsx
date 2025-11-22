import React, { useMemo } from 'react';
import { Line } from '@react-three/drei';
import { VehicleState } from '../../lib/vehicle/VehicleManager';
import { useLayerVisibility } from '../../context/LayerVisibilityContext';

interface Trajectory3DProps {
  vehicle: VehicleState;
}

export function Trajectory3D({ vehicle }: Trajectory3DProps) {
  const { trajectory, color } = vehicle;
  const { trajectories: showTrajectories } = useLayerVisibility();

  const points = useMemo(() => {
      if (trajectory.length < 2) return [];
      return trajectory;
  }, [trajectory]);

  const vertexColors = useMemo(() => {
      if (points.length < 2) return [];
      const colors = [];
      const minAlpha = 0.3;
      for (let i = 0; i < points.length; i++) {
          const alpha = minAlpha + (1 - minAlpha) * (i / (points.length - 1));
          colors.push([color.r * alpha, color.g * alpha, color.b * alpha]);
      }
      return colors;
  }, [points, color]);

  if (!showTrajectories || points.length < 2) return null;

  return (
    <Line
      points={points}
      color="white"
      vertexColors={vertexColors as [number, number, number][]}
      lineWidth={2}
    />
  );
}
