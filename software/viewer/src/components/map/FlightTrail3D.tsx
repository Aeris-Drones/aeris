'use client';

import React from 'react';
import { Line } from '@react-three/drei';

interface FlightTrail3DProps {
  points: [number, number, number][];
  color?: string;
  opacity?: number;
  lineWidth?: number;
  dashed?: boolean;
}

export function FlightTrail3D({
  points,
  color = '#22c55e',
  opacity = 0.6,
  lineWidth = 2,
  dashed = true,
}: FlightTrail3DProps) {
  if (points.length < 2) return null;

  return (
    <group>
      {/* Main trail line */}
      <Line
        points={points}
        color={color}
        lineWidth={lineWidth}
        transparent
        opacity={opacity}
        dashed={dashed}
        dashSize={3}
        gapSize={2}
      />

      {/* Trail dots at each point */}
      {points.map((point, i) => {
        // Skip first point (oldest), smaller dots for older points
        const t = i / (points.length - 1);
        const size = 0.5 + t * 1.5;
        const dotOpacity = 0.2 + t * 0.6;

        return (
          <mesh key={i} position={point}>
            <sphereGeometry args={[size, 8, 8]} />
            <meshBasicMaterial
              color={color}
              transparent
              opacity={dotOpacity}
            />
          </mesh>
        );
      })}

      {/* Ground projection (shadow trail) */}
      <Line
        points={points.map(([x, , z]) => [x, 0.5, z] as [number, number, number])}
        color={color}
        lineWidth={1}
        transparent
        opacity={0.15}
        dashed
        dashSize={5}
        gapSize={5}
      />
    </group>
  );
}
