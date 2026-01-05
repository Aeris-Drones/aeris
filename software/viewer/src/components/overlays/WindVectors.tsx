import React, { useEffect, useMemo, useRef } from 'react';
import * as THREE from 'three';
import type { GasPlume } from '../../hooks/useGasPerception';

/**
 * WindVectors renders 3D arrows showing wind direction and magnitude
 * at sample points within/around the gas plume.
 *
 * Acceptance Criteria addressed:
 * - AC1: Wind vector arrows showing direction and magnitude
 * - AC2: Arrow length represents wind speed
 */

interface WindVectorsProps {
  plumes: GasPlume[];
  windDirection: THREE.Vector3;
  windSpeed?: number;
  gridSpacing?: number;
  arrowColor?: THREE.ColorRepresentation;
  opacity?: number;
  depthTest?: boolean;
  renderOrder?: number;
}

const DEFAULT_ARROW_COLOR = new THREE.Color(0xffffff);
const DEFAULT_SPACING = 15; // meters - grid spacing between arrows
const DEFAULT_SPEED = 2.0; // m/s
const ARROW_LENGTH_SCALE = 3.0; // multiplier for arrow length vs wind speed
const MIN_ARROWS = 4; // minimum arrows to render if plume bounds allow
const MAX_ARROWS = 100; // maximum arrows to prevent performance issues
const DEFAULT_OPACITY = 0.45;

/**
 * Computes the axis-aligned bounding box for all plume polygons
 */
function computePlumeBounds(plumes: GasPlume[]) {
  const min = new THREE.Vector3(Infinity, Infinity, Infinity);
  const max = new THREE.Vector3(-Infinity, -Infinity, -Infinity);

  plumes.forEach((plume) => {
    plume.polygons.forEach((poly) => {
      poly.points.forEach((pt) => {
        min.x = Math.min(min.x, pt.x);
        min.y = Math.min(min.y, pt.y);
        min.z = Math.min(min.z, pt.z);
        max.x = Math.max(max.x, pt.x);
        max.y = Math.max(max.y, pt.y);
        max.z = Math.max(max.z, pt.z);
      });
    });
  });

  if (!isFinite(min.x) || !isFinite(max.x)) {
    return null;
  }

  return {
    min,
    max,
    center: new THREE.Vector3().addVectors(min, max).multiplyScalar(0.5),
    size: new THREE.Vector3().subVectors(max, min),
  };
}

/**
 * Generate grid sample points within the plume bounding box
 */
function generateGridPoints(
  bounds: { min: THREE.Vector3; max: THREE.Vector3; center: THREE.Vector3; size: THREE.Vector3 },
  spacing: number
): THREE.Vector3[] {
  const points: THREE.Vector3[] = [];

  const xCount = Math.max(2, Math.floor(bounds.size.x / spacing) + 1);
  const zCount = Math.max(2, Math.floor(bounds.size.z / spacing) + 1);

  const totalCount = xCount * zCount;
  let effectiveSpacing = spacing;

  if (totalCount > MAX_ARROWS) {
    const scaleFactor = Math.sqrt(totalCount / MAX_ARROWS);
    effectiveSpacing = spacing * scaleFactor;
  }

  const y = bounds.center.y + 2;

  for (let x = bounds.min.x; x <= bounds.max.x; x += effectiveSpacing) {
    for (let z = bounds.min.z; z <= bounds.max.z; z += effectiveSpacing) {
      points.push(new THREE.Vector3(x, y, z));

      if (points.length >= MAX_ARROWS) {
        return points;
      }
    }
  }

  return points;
}

export function WindVectors({
  plumes,
  windDirection,
  windSpeed = DEFAULT_SPEED,
  gridSpacing = DEFAULT_SPACING,
  arrowColor = DEFAULT_ARROW_COLOR,
  opacity = DEFAULT_OPACITY,
  depthTest = false,
  renderOrder = 6,
}: WindVectorsProps) {
  const shaftMeshRef = useRef<THREE.InstancedMesh>(null);
  const headMeshRef = useRef<THREE.InstancedMesh>(null);

  const arrowPositions = useMemo(() => {
    if (plumes.length === 0) return [];

    const bounds = computePlumeBounds(plumes);
    if (!bounds) return [];

    const points = generateGridPoints(bounds, gridSpacing);

    if (points.length < MIN_ARROWS && bounds.size.x > 0 && bounds.size.z > 0) {
      const fallbackPoints = [
        bounds.center.clone(),
        new THREE.Vector3(bounds.min.x, bounds.center.y + 2, bounds.min.z),
        new THREE.Vector3(bounds.max.x, bounds.center.y + 2, bounds.min.z),
        new THREE.Vector3(bounds.min.x, bounds.center.y + 2, bounds.max.z),
        new THREE.Vector3(bounds.max.x, bounds.center.y + 2, bounds.max.z),
      ];
      return fallbackPoints.slice(0, Math.max(MIN_ARROWS, points.length));
    }

    return points;
  }, [plumes, gridSpacing]);

  const normalizedDirection = useMemo(() => {
    if (windDirection.lengthSq() < 1e-6) {
      return new THREE.Vector3(1, 0, 0);
    }
    return windDirection.clone().normalize();
  }, [windDirection]);

  const directionQuat = useMemo(() => {
    const baseAxis = new THREE.Vector3(0, 1, 0);
    return new THREE.Quaternion().setFromUnitVectors(baseAxis, normalizedDirection);
  }, [normalizedDirection]);

  const arrowDimensions = useMemo(() => {
    const length = Math.max(2.0, windSpeed * ARROW_LENGTH_SCALE);
    const headLength = length * 0.3;
    const headRadius = length * 0.08;
    const shaftLength = Math.max(0.1, length - headLength);
    const shaftRadius = Math.max(0.03, headRadius * 0.35);
    return { length, headLength, headRadius, shaftLength, shaftRadius };
  }, [windSpeed]);

  const geometries = useMemo(() => {
    const shaft = new THREE.CylinderGeometry(
      arrowDimensions.shaftRadius,
      arrowDimensions.shaftRadius,
      arrowDimensions.shaftLength,
      8
    );
    shaft.translate(0, arrowDimensions.shaftLength / 2, 0);

    const head = new THREE.ConeGeometry(arrowDimensions.headRadius, arrowDimensions.headLength, 10);
    head.translate(0, arrowDimensions.shaftLength + arrowDimensions.headLength / 2, 0);

    return { shaft, head };
  }, [arrowDimensions]);

  useEffect(() => {
    return () => {
      geometries.shaft.dispose();
      geometries.head.dispose();
    };
  }, [geometries]);

  const material = useMemo(() => {
    return new THREE.MeshBasicMaterial({
      color: arrowColor,
      transparent: true,
      opacity: THREE.MathUtils.clamp(opacity, 0, 1),
      depthWrite: false,
      depthTest: depthTest,
      blending: THREE.AdditiveBlending,
      toneMapped: false,
    });
  }, [arrowColor, opacity, depthTest]);

  useEffect(() => {
    return () => material.dispose();
  }, [material]);

  useEffect(() => {
    const shaft = shaftMeshRef.current;
    const head = headMeshRef.current;
    if (!shaft || !head) return;

    const count = Math.min(arrowPositions.length, MAX_ARROWS);
    const matrix = new THREE.Matrix4();
    const scale = new THREE.Vector3(1, 1, 1);

    for (let i = 0; i < count; i++) {
      matrix.compose(arrowPositions[i], directionQuat, scale);
      shaft.setMatrixAt(i, matrix);
      head.setMatrixAt(i, matrix);
    }

    shaft.count = count;
    head.count = count;
    shaft.instanceMatrix.needsUpdate = true;
    head.instanceMatrix.needsUpdate = true;
  }, [arrowPositions, directionQuat]);

  if (arrowPositions.length === 0) return null;

  return (
    <group name="wind-vectors" renderOrder={renderOrder}>
      <instancedMesh
        ref={shaftMeshRef}
        args={[geometries.shaft, material, MAX_ARROWS]}
        frustumCulled={false}
        renderOrder={renderOrder}
      />
      <instancedMesh
        ref={headMeshRef}
        args={[geometries.head, material, MAX_ARROWS]}
        frustumCulled={false}
        renderOrder={renderOrder}
      />
    </group>
  );
}
