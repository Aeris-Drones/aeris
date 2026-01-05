import React, { useMemo, useRef, useEffect } from 'react';
import { useFrame } from '@react-three/fiber';
import * as THREE from 'three';
import { useGasPerception } from '../../hooks/useGasPerception';
import { useLayerVisibility } from '../../context/LayerVisibilityContext';
import { GasPlumeParticles } from './GasPlumeParticles';
import { WindVectors } from './WindVectors';
import { PlumeCenterline } from './PlumeCenterline';

// Color ramp: Purple (low) -> Green (medium) -> Yellow (high)
export const COLORS = {
  low: new THREE.Color(0x9333ea),    // Purple
  medium: new THREE.Color(0x22c55e), // Green
  high: new THREE.Color(0xf59e0b),   // Amber/Yellow
};

// Animation constants
const PULSE_SPEED = 2;
const PULSE_AMPLITUDE = 0.08;
const BASE_OPACITY = 0.4;
const EXTRUDE_DEPTH = 10; // meters

function getColorForIndex(index: number, total: number): THREE.Color {
  if (total <= 1) return COLORS.high;
  const t = index / (total - 1); // 0 = outer, 1 = inner
  if (t < 0.5) return COLORS.low.clone().lerp(COLORS.medium, t * 2);
  return COLORS.medium.clone().lerp(COLORS.high, (t - 0.5) * 2);
}

function buildShape(points: { x: number; z: number }[]): THREE.Shape | null {
  if (points.length < 3) return null;
  const shape = new THREE.Shape();
  shape.moveTo(points[0].x, -points[0].z);
  for (let i = 1; i < points.length; i++) {
    shape.lineTo(points[i].x, -points[i].z);
  }
  shape.closePath();
  return shape;
}

interface PlumeMeshData {
  geometry: THREE.ExtrudeGeometry;
  material: THREE.MeshBasicMaterial;
  altitude: number;
  key: string;
  layerRatio: number; // 0 = outer, 1 = inner (per-plume)
}

export function GasPlume() {
  const { plumes, wind } = useGasPerception();
  const { gas } = useLayerVisibility();
  const groupRef = useRef<THREE.Group>(null);
  const debugPerf = process.env.NEXT_PUBLIC_GAS_PERF === '1';
  const adaptiveScaling = process.env.NEXT_PUBLIC_GAS_PARTICLE_ADAPTIVE === '1';

  // Build mesh data with memoization
  const meshData = useMemo(() => {
    const data: PlumeMeshData[] = [];

    plumes.forEach((plume, plumeIdx) => {
      const total = plume.polygons.length;
      plume.polygons.forEach((poly, idx) => {
        const shape = buildShape(poly.points.map((p) => ({ x: p.x, z: p.z })));
        if (!shape) return;

        const geometry = new THREE.ExtrudeGeometry(shape, {
          depth: EXTRUDE_DEPTH,
          bevelEnabled: false,
        });
        geometry.rotateX(-Math.PI / 2); // extrude along +Y

        const color = getColorForIndex(idx, total);
        const material = new THREE.MeshBasicMaterial({
          color,
          transparent: true,
          opacity: BASE_OPACITY,
          side: THREE.DoubleSide,
          depthWrite: false,
          blending: THREE.AdditiveBlending,
        });

        const altitude = poly.points[0]?.y ?? 0;
        const layerRatio = total > 1 ? idx / (total - 1) : 0; // 0 = outer, 1 = inner

        data.push({
          geometry,
          material,
          altitude,
          key: `plume-${plumeIdx}-${idx}`,
          layerRatio,
        });
      });
    });

    return data;
  }, [plumes]);

  // Cleanup geometries and materials on unmount or data change
  useEffect(() => {
    return () => {
      meshData.forEach((item) => {
        item.geometry.dispose();
        item.material.dispose();
      });
    };
  }, [meshData]);

  // Pulsing animation for gaseous effect
  useFrame((state) => {
    if (!groupRef.current) return;

    const pulse = 1 + Math.sin(state.clock.elapsedTime * PULSE_SPEED) * PULSE_AMPLITUDE;

    groupRef.current.children.forEach((child) => {
      if (child instanceof THREE.Mesh && child.material instanceof THREE.MeshBasicMaterial) {
        const layerRatio = (child.userData.layerRatio as number) ?? 0;
        const layerOpacity = BASE_OPACITY + (layerRatio * 0.2);
        child.material.opacity = Math.min(layerOpacity * pulse, 0.8);
      }
    });
  });

  if (!gas) return null;
  if (plumes.length === 0) return null;

  // Get centerline from first plume for overlay rendering
  const centerline = plumes[0]?.centerline ?? [];

  return (
    <group ref={groupRef}>
      {/* Layer 1: Plume polygon meshes (base layer) */}
      {meshData.map((item) => (
        <mesh
          key={item.key}
          geometry={item.geometry}
          material={item.material}
          position={[0, item.altitude, 0]}
          userData={{ layerRatio: item.layerRatio }}
        />
      ))}

      {/* Layer 2: Particle system */}
      <GasPlumeParticles
        plumes={plumes}
        windDirection={wind.direction.clone().multiplyScalar(wind.speed)}
        adaptiveScaling={adaptiveScaling}
        debugPerf={debugPerf}
      />

      {/* Layer 3: Centerline glow tube (AC3, AC4) */}
      {centerline.length >= 2 && (
        <PlumeCenterline
          centerline={centerline}
          tubeRadius={0.8}
          glowColor="#FFFF00"
          pulseAnimation={true}
        />
      )}

      {/* Layer 4: Wind vector arrows (AC1, AC2) */}
      <WindVectors
        plumes={plumes}
        windDirection={wind.direction}
        windSpeed={wind.speed}
        gridSpacing={15}
        opacity={0.45}
        depthTest={false}
        renderOrder={6}
      />
    </group>
  );
}
