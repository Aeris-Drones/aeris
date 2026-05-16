'use client';

import { useMemo } from 'react';
import { Html, Line } from '@react-three/drei';
import type { RouteRecommendation } from '@/lib/routeRecommendations';

interface RouteOverlay3DProps {
  route: RouteRecommendation;
}

function routeColor(status: RouteRecommendation['status']): string {
  switch (status) {
    case 'clear':
      return '#34d399';
    case 'stale':
      return '#22d3ee';
    case 'pending':
      return '#facc15';
  }
}

/**
 * Advisory responder route overlay.
 *
 * Routes are visual guidance for incident command only; they do not publish
 * vehicle commands or alter autonomous pathing.
 */
export function RouteOverlay3D({ route }: RouteOverlay3DProps) {
  const color = routeColor(route.status);
  const elevatedPoints = useMemo(
    () => route.polyline.map(([x, y, z]) => [x, Math.max(y, 4), z] as [number, number, number]),
    [route.polyline]
  );
  const midpoint = elevatedPoints[Math.floor(elevatedPoints.length / 2)];

  if (elevatedPoints.length < 2) {
    return null;
  }

  return (
    <group>
      <Line
        points={elevatedPoints}
        color={color}
        lineWidth={5}
        transparent
        opacity={route.status === 'stale' ? 0.58 : 0.9}
        dashed={route.status !== 'clear'}
        dashSize={6}
        gapSize={3}
      />
      <Line
        points={elevatedPoints.map(([x, , z]) => [x, 0.75, z] as [number, number, number])}
        color={color}
        lineWidth={2}
        transparent
        opacity={0.22}
        dashed
        dashSize={8}
        gapSize={6}
      />
      {elevatedPoints.map((point, index) => (
        <mesh key={`${route.id}-${index}`} position={point}>
          <sphereGeometry args={[index === 0 || index === elevatedPoints.length - 1 ? 3.5 : 2, 12, 12]} />
          <meshBasicMaterial color={color} transparent opacity={0.82} />
        </mesh>
      ))}
      {midpoint && (
        <Html
          position={[midpoint[0], midpoint[1] + 10, midpoint[2]]}
          center
          distanceFactor={110}
          occlude={false}
          style={{ pointerEvents: 'none' }}
        >
          <div className="rounded border border-emerald-300/30 bg-black/70 px-2 py-1 text-[10px] font-mono uppercase text-emerald-200 shadow-lg">
            {route.status === 'stale' ? 'Stale route' : 'Entry route'}
          </div>
        </Html>
      )}
    </group>
  );
}
