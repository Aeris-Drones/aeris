'use client';

import { useMemo } from 'react';
import * as THREE from 'three';
import { Line, Text } from '@react-three/drei';
import type { PriorityZone, ZonePoint, ZonePriority } from '@/types/zone';
import { calculateZoneCentroid } from '@/types/zone';

const priorityColors: Record<ZonePriority, { line: string; fill: string }> = {
  1: { line: '#ef4444', fill: '#ef4444' },
  2: { line: '#f97316', fill: '#f97316' },
  3: { line: '#eab308', fill: '#eab308' },
};

interface ZoneOverlay3DProps {
  zone: PriorityZone;
  isSelected: boolean;
  onClick?: () => void;
}

function pointsToVector3(points: ZonePoint[], y: number = 1): THREE.Vector3[] {
  return points.map(p => new THREE.Vector3(p.x, y, p.z));
}

export function ZoneOverlay3D({ zone, isSelected, onClick }: ZoneOverlay3DProps) {
  const colors = priorityColors[zone.priority];
  const isCompleted = zone.status === 'completed';
  const isSkipped = zone.status === 'skipped';
  const isDimmed = isCompleted || isSkipped;

  const linePoints = useMemo(() => {
    if (zone.polygon.length < 3) return [];
    const points = pointsToVector3(zone.polygon, 2);
    points.push(points[0].clone());
    return points;
  }, [zone.polygon]);

  const fillGeometry = useMemo(() => {
    if (zone.polygon.length < 3) return null;

    const shape = new THREE.Shape();
    shape.moveTo(zone.polygon[0].x, -zone.polygon[0].z);
    for (let i = 1; i < zone.polygon.length; i++) {
      shape.lineTo(zone.polygon[i].x, -zone.polygon[i].z);
    }
    shape.closePath();

    return new THREE.ShapeGeometry(shape);
  }, [zone.polygon]);

  const centroid = useMemo(() => {
    return calculateZoneCentroid(zone.polygon);
  }, [zone.polygon]);

  if (zone.polygon.length < 3) return null;

  return (
    <group onClick={onClick}>
      {fillGeometry && (
        <mesh
          geometry={fillGeometry}
          rotation={[-Math.PI / 2, 0, 0]}
          position={[0, 0.5, 0]}
        >
          <meshBasicMaterial
            color={isDimmed ? '#666666' : colors.fill}
            transparent
            opacity={isSelected ? 0.2 : 0.1}
            side={THREE.DoubleSide}
            depthWrite={false}
          />
        </mesh>
      )}

      <Line
        points={linePoints}
        color={isDimmed ? '#666666' : colors.line}
        lineWidth={isSelected ? 3 : 2}
        transparent
        opacity={isDimmed ? 0.4 : 0.8}
        dashed={isCompleted}
        dashSize={isCompleted ? 5 : undefined}
        gapSize={isCompleted ? 5 : undefined}
      />

      <Text
        position={[centroid.x, 8, centroid.z]}
        fontSize={6}
        color={isDimmed ? '#666666' : colors.line}
        anchorX="center"
        anchorY="middle"
        outlineWidth={0.3}
        outlineColor="#000000"
      >
        {zone.name}
      </Text>

      <Text
        position={[centroid.x, 3, centroid.z]}
        fontSize={4}
        color={isDimmed ? '#444444' : '#ffffff'}
        anchorX="center"
        anchorY="middle"
        outlineWidth={0.2}
        outlineColor="#000000"
      >
        P{zone.priority}
      </Text>
    </group>
  );
}

interface ZoneDrawingPreviewProps {
  points: ZonePoint[];
  priority: ZonePriority;
}

export function ZoneDrawingPreview({ points, priority }: ZoneDrawingPreviewProps) {
  const colors = priorityColors[priority];

  const linePoints = useMemo(() => {
    if (points.length < 1) return [];
    return pointsToVector3(points, 3);
  }, [points]);

  const fillGeometry = useMemo(() => {
    if (points.length < 3) return null;

    const shape = new THREE.Shape();
    shape.moveTo(points[0].x, -points[0].z);
    for (let i = 1; i < points.length; i++) {
      shape.lineTo(points[i].x, -points[i].z);
    }
    shape.closePath();

    return new THREE.ShapeGeometry(shape);
  }, [points]);

  if (points.length < 1) return null;

  return (
    <group>
      {fillGeometry && (
        <mesh
          geometry={fillGeometry}
          rotation={[-Math.PI / 2, 0, 0]}
          position={[0, 1, 0]}
        >
          <meshBasicMaterial
            color={colors.fill}
            transparent
            opacity={0.15}
            side={THREE.DoubleSide}
            depthWrite={false}
          />
        </mesh>
      )}

      {linePoints.length >= 2 && (
        <Line
          points={linePoints}
          color={colors.line}
          lineWidth={2}
          transparent
          opacity={0.6}
          dashed
          dashSize={3}
          gapSize={2}
        />
      )}

      {points.map((point, i) => (
        <mesh key={i} position={[point.x, 3, point.z]}>
          <sphereGeometry args={[2, 16, 16]} />
          <meshBasicMaterial color={colors.line} />
        </mesh>
      ))}
    </group>
  );
}
