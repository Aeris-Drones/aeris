'use client';

import { useMemo } from 'react';
import * as THREE from 'three';
import { Line, Text } from '@react-three/drei';
import type { PriorityZone, ZonePoint, ZonePriority } from '@/types/zone';
import { calculateZoneCentroid } from '@/types/zone';

/**
 * Priority color scheme: lower numbers indicate higher operational priority.
 * Red (1) = immediate attention, Orange (2) = elevated, Yellow (3) = standard.
 */
const priorityColors: Record<ZonePriority, { line: string; fill: string }> = {
  1: { line: '#ef4444', fill: '#ef4444' }, // Red - highest priority
  2: { line: '#f97316', fill: '#f97316' }, // Orange - medium priority
  3: { line: '#eab308', fill: '#eab308' }, // Yellow - lowest priority
};

interface ZoneOverlay3DProps {
  zone: PriorityZone;
  isSelected: boolean;
  onClick?: () => void;
}

/**
 * Converts zone points to Three.js vectors with height offset.
 * Zones are defined on the XZ ground plane; Y is added for rendering elevation.
 */
function pointsToVector3(points: ZonePoint[], y: number = 1): THREE.Vector3[] {
  return points.map(p => new THREE.Vector3(p.x, y, p.z));
}

/**
 * Renders a priority zone polygon on the 3D map with visual state indicators.
 *
 * Coordinate transformation is required because:
 * - Zone data uses XZ ground plane (y=0 in world space)
 * - Three.js Shape works in XY plane, so we negate Z and rotate -90° on X
 *
 * Visual states communicate operational status:
 * - Completed: Dashed outline indicates no longer active
 * - Skipped: Reduced opacity shows it was bypassed
 * - Selected: Thicker border for focus indication
 *
 * @param zone - Zone data from mission planning system
 * @param isSelected - Whether this zone is currently focused
 * @param onClick - Handler for zone selection interactions
 */
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

  // ShapeGeometry requires XY plane construction; rotation aligns to XZ ground
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

  const centroid = useMemo(() => calculateZoneCentroid(zone.polygon), [zone.polygon]);

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

/**
 * Shows in-progress zone polygon during drawing mode.
 *
 * Provides immediate visual feedback as operators click points on the map:
 * - Fill preview appears once polygon is valid (3+ points)
 * - Dashed line shows current perimeter
 * - Vertex markers indicate clicked locations
 *
 * Uses identical coordinate transformation as ZoneOverlay3D to ensure
 * the preview matches the final zone appearance.
 */
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
