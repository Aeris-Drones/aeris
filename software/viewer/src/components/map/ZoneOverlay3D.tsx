'use client';

import { useMemo } from 'react';
import * as THREE from 'three';
import { Line, Text } from '@react-three/drei';
import type { PriorityZone, ZonePoint, ZonePriority } from '@/types/zone';
import { calculateZoneCentroid } from '@/types/zone';

/**
 * Color mapping for zone priority levels.
 * Higher priority (1) gets more urgent colors.
 */
const priorityColors: Record<ZonePriority, { line: string; fill: string }> = {
  1: { line: '#ef4444', fill: '#ef4444' }, // Red - highest priority
  2: { line: '#f97316', fill: '#f97316' }, // Orange - medium priority
  3: { line: '#eab308', fill: '#eab308' }, // Yellow - lowest priority
};

/**
 * Props for the ZoneOverlay3D component.
 */
interface ZoneOverlay3DProps {
  /** Zone data including polygon points, priority, and status */
  zone: PriorityZone;
  /** Whether this zone is currently selected */
  isSelected: boolean;
  /** Click handler for zone selection */
  onClick?: () => void;
}

/**
 * Converts ZonePoint array to Three.js Vector3 array.
 * Note: Y-coordinate is set explicitly since zones exist on the XZ ground plane.
 *
 * @param points - Array of zone points with x, z coordinates
 * @param y - Height offset for the line (default: 1 unit above ground)
 * @returns Array of Vector3 points for Three.js Line
 */
function pointsToVector3(points: ZonePoint[], y: number = 1): THREE.Vector3[] {
  return points.map(p => new THREE.Vector3(p.x, y, p.z));
}

/**
 * ZoneOverlay3D - Renders a priority zone on the 3D map.
 *
 * Visual Components:
 * - Fill: Semi-transparent polygon showing zone area
 * - Outline: Colored line around the perimeter
 * - Labels: Zone name and priority level at centroid
 *
 * Coordinate System Notes:
 * - Zones are defined on the XZ ground plane (y=0)
 * - Fill geometry uses Shape which works in XY plane, hence the -z conversion
 * - Rotation [-PI/2, 0, 0] lays the shape flat on the XZ plane
 *
 * State Visuals:
 * - Completed zones: Dashed outline, grayed out
 * - Skipped zones: Grayed out with reduced opacity
 * - Selected zones: Thicker outline, higher fill opacity
 */
export function ZoneOverlay3D({ zone, isSelected, onClick }: ZoneOverlay3DProps) {
  const colors = priorityColors[zone.priority];
  const isCompleted = zone.status === 'completed';
  const isSkipped = zone.status === 'skipped';
  const isDimmed = isCompleted || isSkipped;

  /**
   * Line points for the zone outline.
   * Closes the polygon by cloning the first point to the end.
   * Memoized to prevent recalculation on re-renders.
   */
  const linePoints = useMemo(() => {
    if (zone.polygon.length < 3) return [];
    const points = pointsToVector3(zone.polygon, 2);
    points.push(points[0].clone());
    return points;
  }, [zone.polygon]);

  /**
   * Fill geometry for the zone polygon.
   *
   * Three.js Shape works in XY plane, but we need XZ for ground alignment.
   * The shape is built with (x, -z) coordinates, then rotated -90 degrees
   * around X axis to lay flat on the ground.
   *
   * Memoized to prevent expensive geometry regeneration.
   */
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

  /** Centroid for label positioning - calculated from polygon geometry */
  const centroid = useMemo(() => {
    return calculateZoneCentroid(zone.polygon);
  }, [zone.polygon]);

  // Don't render if polygon has insufficient points
  if (zone.polygon.length < 3) return null;

  return (
    <group onClick={onClick}>
      {/* Zone fill - semi-transparent polygon on ground */}
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

      {/* Zone outline - colored line around perimeter */}
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

      {/* Zone name label - positioned above centroid */}
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

      {/* Priority level label - below name */}
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

/**
 * Props for the ZoneDrawingPreview component.
 */
interface ZoneDrawingPreviewProps {
  /** Current points in the polygon being drawn */
  points: ZonePoint[];
  /** Priority level for visual color coding */
  priority: ZonePriority;
}

/**
 * ZoneDrawingPreview - Shows in-progress zone during drawing mode.
 *
 * Renders:
 * - Fill preview (when 3+ points exist)
 * - Dashed line connecting points
 * - Spheres at each vertex for visual feedback
 *
 * Uses the same coordinate transformation as ZoneOverlay3D
 * for consistency with final rendered zones.
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
      {/* Preview fill - more opaque than final zones */}
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

      {/* Preview outline - dashed line */}
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

      {/* Vertex markers - spheres at each clicked point */}
      {points.map((point, i) => (
        <mesh key={i} position={[point.x, 3, point.z]}>
          <sphereGeometry args={[2, 16, 16]} />
          <meshBasicMaterial color={colors.line} />
        </mesh>
      ))}
    </group>
  );
}
