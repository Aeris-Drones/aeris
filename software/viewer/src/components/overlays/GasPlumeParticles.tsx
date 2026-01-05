import React, { useEffect, useMemo, useRef } from 'react';
import { useFrame } from '@react-three/fiber';
import * as THREE from 'three';
import type { GasPlume } from '../../hooks/useGasPerception';
import vertexShader from '../../lib/shaders/gasParticle.vert.glsl';
import fragmentShader from '../../lib/shaders/gasParticle.frag.glsl';
import { COLORS } from './GasPlume';

const DEFAULT_PARTICLE_COUNT = 50000;
const DEFAULT_LIFETIME = 4.0;

const UP_AXIS = new THREE.Vector3(0, 1, 0);
const DEFAULT_FALLBACK_WIND = new THREE.Vector3(1.0, 0.2, 0.5).normalize().multiplyScalar(2.0);

interface GasPlumeParticlesProps {
  plumes: GasPlume[];
  particleCount?: number;
  particleLifetime?: number;
  windDirection?: THREE.Vector3;
  turbulenceScale?: number;
  adaptiveScaling?: boolean;
  debugPerf?: boolean;
}

interface PolygonInfo {
  points: { x: number; y: number; z: number }[];
  bounds: { minX: number; maxX: number; minZ: number; maxZ: number };
  weight: number;
  concentration: number;
  altitude: number;
}

interface PlumeInfo {
  polygons: PolygonInfo[];
  weight: number;
  polygonWeights: number[];
  polygonWeightTotal: number;
}

export function GasPlumeParticles({
  plumes,
  particleCount = DEFAULT_PARTICLE_COUNT,
  particleLifetime = DEFAULT_LIFETIME,
  windDirection,
  turbulenceScale = 2.0,
  adaptiveScaling = false,
  debugPerf = false,
}: GasPlumeParticlesProps) {
  const materialRef = useRef<THREE.ShaderMaterial>(null);
  const perfRef = useRef({ elapsed: 0, frames: 0, lastCheck: 0 });
  const tempWind = useRef(new THREE.Vector3());
  const geometryRef = useRef<THREE.InstancedBufferGeometry | null>(null);

  const derivedWind = useMemo(() => {
    if (windDirection) return windDirection.clone();

    const plumeWithCenterline = plumes.find((plume) => plume.centerline.length >= 2);
    if (!plumeWithCenterline) return DEFAULT_FALLBACK_WIND.clone();

    const first = plumeWithCenterline.centerline[0];
    const last = plumeWithCenterline.centerline[plumeWithCenterline.centerline.length - 1];
    const vector = new THREE.Vector3(last.x - first.x, 0, last.z - first.z);
    if (vector.lengthSq() < 1e-6) return DEFAULT_FALLBACK_WIND.clone();

    return vector.normalize().multiplyScalar(2.0);
  }, [plumes, windDirection]);

  const positions = useMemo(() => new Float32Array(particleCount * 3), [particleCount]);
  const concentrations = useMemo(() => new Float32Array(particleCount), [particleCount]);
  const seeds = useMemo(() => {
    const data = new Float32Array(particleCount);
    for (let i = 0; i < particleCount; i++) {
      data[i] = pseudoRandom(i + 1);
    }
    return data;
  }, [particleCount]);

  const geometry = useMemo(() => {
    const geom = new THREE.InstancedBufferGeometry();

    geom.setAttribute('position', new THREE.Float32BufferAttribute([0, 0, 0], 3));
    geom.setAttribute('instancePosition', new THREE.InstancedBufferAttribute(positions, 3));
    geom.setAttribute('instanceSeed', new THREE.InstancedBufferAttribute(seeds, 1));
    geom.setAttribute('instanceConcentration', new THREE.InstancedBufferAttribute(concentrations, 1));

    geom.instanceCount = particleCount;
    return geom;
  }, [particleCount, positions, seeds, concentrations]);

  const uniforms = useMemo(() => {
    return {
      uTime: { value: 0 },
      uLifetime: { value: particleLifetime },
      uWindDirection: { value: derivedWind.clone() },
      uTurbulenceScale: { value: turbulenceScale },
      uColorLow: { value: COLORS.low.clone() },
      uColorMid: { value: COLORS.medium.clone() },
      uColorHigh: { value: COLORS.high.clone() },
    };
  }, [particleLifetime, derivedWind, turbulenceScale]);

  useEffect(() => {
    geometryRef.current = geometry;
    return () => {
      geometryRef.current = null;
      geometry.dispose();
    };
  }, [geometry]);

  useEffect(() => {
    if (plumes.length === 0) return;

    distributeParticles(plumes, positions, concentrations, particleCount);
    const posAttr = geometry.getAttribute('instancePosition') as THREE.InstancedBufferAttribute | undefined;
    if (posAttr) posAttr.needsUpdate = true;
    const concAttr = geometry.getAttribute('instanceConcentration') as THREE.InstancedBufferAttribute | undefined;
    if (concAttr) concAttr.needsUpdate = true;
  }, [plumes, positions, concentrations, particleCount, geometry]);

  useFrame((state, delta) => {
    const material = materialRef.current;
    if (!material) return;

    material.uniforms.uTime.value = state.clock.elapsedTime;
    tempWind.current
      .copy(derivedWind)
      .applyAxisAngle(UP_AXIS, Math.sin(state.clock.elapsedTime * 0.1) * 0.2);
    material.uniforms.uWindDirection.value.copy(tempWind.current);

    if (!adaptiveScaling && !debugPerf) return;

    const geom = geometryRef.current;
    if (!geom) return;

    const perf = perfRef.current;
    perf.elapsed += delta;
    perf.frames += 1;

    if (state.clock.elapsedTime - perf.lastCheck >= 1) {
      const avgFrame = perf.elapsed / Math.max(perf.frames, 1);
      const fps = avgFrame > 0 ? 1 / avgFrame : 0;

      if (debugPerf) {
        console.log(`[GasPlumeParticles] fps=${fps.toFixed(1)} instances=${geom.instanceCount}`);
      }

      if (!adaptiveScaling) {
        perf.elapsed = 0;
        perf.frames = 0;
        perf.lastCheck = state.clock.elapsedTime;
        return;
      }

      let nextCount = geom.instanceCount;

      if (avgFrame > 1 / 55 && nextCount > 5000) {
        nextCount = Math.max(5000, Math.floor(nextCount * 0.9));
      } else if (avgFrame < 1 / 60 && nextCount < particleCount) {
        nextCount = Math.min(particleCount, Math.floor(nextCount * 1.05));
      }

      if (nextCount !== geom.instanceCount) {
        geom.instanceCount = nextCount;
      }

      perf.elapsed = 0;
      perf.frames = 0;
      perf.lastCheck = state.clock.elapsedTime;
    }
  });

  if (plumes.length === 0) return null;

  return (
    <points renderOrder={2} geometry={geometry} frustumCulled={false}>
      <shaderMaterial
        ref={materialRef}
        vertexShader={vertexShader}
        fragmentShader={fragmentShader}
        uniforms={uniforms}
        transparent
        depthWrite={false}
        blending={THREE.AdditiveBlending}
      />
    </points>
  );
}

function distributeParticles(
  plumes: GasPlume[],
  positions: Float32Array,
  concentrations: Float32Array,
  count: number
) {
  const plumeInfos = buildPlumeInfos(plumes);
  if (plumeInfos.length === 0) return;

  const plumeWeights = plumeInfos.map((info) => info.weight);
  const plumeWeightTotal = plumeWeights.reduce((sum, weight) => sum + weight, 0);

  for (let i = 0; i < count; i++) {
    const plumeIndex = pickWeightedIndex(plumeWeights, plumeWeightTotal);
    const plumeInfo = plumeInfos[plumeIndex];
    const polygonIndex = pickWeightedIndex(plumeInfo.polygonWeights, plumeInfo.polygonWeightTotal);
    const polygon = plumeInfo.polygons[polygonIndex];

    const point = randomPointInPolygon(polygon);
    const offset = i * 3;
    positions[offset] = point.x;
    positions[offset + 1] = point.y;
    positions[offset + 2] = point.z;
    concentrations[i] = polygon.concentration;
  }
}

function buildPlumeInfos(plumes: GasPlume[]): PlumeInfo[] {
  return plumes
    .map((plume) => {
      const total = plume.polygons.length || 1;
      const polygons = plume.polygons
        .filter((poly) => poly.points.length >= 3)
        .map((poly, index) => {
          const bounds = computeBounds(poly.points);
          const layerRatio = total > 1 ? index / (total - 1) : 1;
          const weight = total > 1 ? (index + 1) / total : 1;
          const altitude = averageAltitude(poly.points);
          return {
            points: poly.points,
            bounds,
            weight,
            concentration: layerRatio,
            altitude,
          };
        });

      const polygonWeights = polygons.map((poly) => poly.weight);
      const polygonWeightTotal = polygonWeights.reduce((sum, weight) => sum + weight, 0);
      const weight = polygonWeightTotal;
      return polygons.length
        ? { polygons, weight, polygonWeights, polygonWeightTotal }
        : null;
    })
    .filter((value): value is PlumeInfo => Boolean(value));
}

function computeBounds(points: { x: number; z: number }[]) {
  let minX = Infinity;
  let maxX = -Infinity;
  let minZ = Infinity;
  let maxZ = -Infinity;

  points.forEach((point) => {
    minX = Math.min(minX, point.x);
    maxX = Math.max(maxX, point.x);
    minZ = Math.min(minZ, point.z);
    maxZ = Math.max(maxZ, point.z);
  });

  return { minX, maxX, minZ, maxZ };
}

function averageAltitude(points: { y: number }[]) {
  const total = points.reduce((sum, point) => sum + point.y, 0);
  return points.length ? total / points.length : 0;
}

function randomPointInPolygon(polygon: PolygonInfo) {
  const { minX, maxX, minZ, maxZ } = polygon.bounds;

  for (let attempt = 0; attempt < 12; attempt++) {
    const x = minX + Math.random() * (maxX - minX);
    const z = minZ + Math.random() * (maxZ - minZ);
    if (pointInPolygon(x, z, polygon.points)) {
      return { x, y: polygon.altitude, z };
    }
  }

  const fallback = polygon.points[0];
  return { x: fallback.x, y: polygon.altitude, z: fallback.z };
}

function pointInPolygon(x: number, z: number, points: { x: number; z: number }[]) {
  let inside = false;

  for (let i = 0, j = points.length - 1; i < points.length; j = i++) {
    const xi = points[i].x;
    const zi = points[i].z;
    const xj = points[j].x;
    const zj = points[j].z;

    const intersect = (zi > z) !== (zj > z) && x < ((xj - xi) * (z - zi)) / (zj - zi) + xi;
    if (intersect) inside = !inside;
  }

  return inside;
}

function pickWeightedIndex(weights: number[], totalWeight: number) {
  if (weights.length === 1) return 0;

  let target = Math.random() * totalWeight;
  for (let i = 0; i < weights.length; i++) {
    target -= weights[i];
    if (target <= 0) return i;
  }

  return weights.length - 1;
}

function pseudoRandom(seed: number) {
  const value = Math.sin(seed * 12.9898) * 43758.5453;
  return value - Math.floor(value);
}
