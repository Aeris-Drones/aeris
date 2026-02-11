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

/**
 * Renders a 3D flight trail with visual depth cues.
 *
 * Visual hierarchy communicates temporal information:
 * - Trail dots grow larger and more opaque toward the recent end
 * - Ground projection provides spatial context without cluttering the view
 * - Dashed line style reduces visual weight for historical paths
 *
 * Used by MapScene3D for both live vehicle trajectories and planned return paths.
 *
 * @param points - Array of [x, y, z] coordinates in world space (Y-up)
 * @param color - Trail color (default: green for active flights)
 * @param opacity - Line opacity for layering control
 * @param lineWidth - Stroke width in pixels
 * @param dashed - Whether to use dashed line style
 */
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

      {points.map((point, i) => {
        // Visual weight increases toward recent positions for temporal cues
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
