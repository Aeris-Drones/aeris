'use client';

import React, { useRef, useMemo } from 'react';
import { useFrame } from '@react-three/fiber';
import { Html, Line } from '@react-three/drei';
import * as THREE from 'three';

/** Props for the DetectionMarker3D component - represents a sensor detection in the 3D scene */
export interface DetectionMarker3DProps {
  /** Unique detection identifier */
  id: string;
  /** World position [x, y, z] - typically y=0 for ground-level detections */
  position: [number, number, number];
  /** Sensor type - determines marker shape and color */
  sensorType: 'thermal' | 'acoustic' | 'gas';
  /** Detection confidence score (0-1) */
  confidence: number;
  /** Review status - drives animation and visual emphasis */
  status: 'new' | 'reviewing' | 'confirmed' | 'dismissed';
  /** Optional geometry projected from fused payloads (thermal/acoustic/gas) */
  geometry?: [number, number, number][];
  /** Whether this detection is currently selected */
  isSelected: boolean;
  /** Click handler for selection */
  onClick: () => void;
}

/** Color mapping by sensor type for consistent visual language across the GCS */
const SENSOR_COLORS = {
  thermal: '#f97316',
  acoustic: '#3b82f6',
  gas: '#eab308',
};

/** Human-readable labels for sensor types - displayed in the marker UI */
const SENSOR_LABELS = {
  thermal: 'THERMAL',
  acoustic: 'ACOUSTIC',
  gas: 'GAS',
};

/**
 * 3D marker for sensor detections (thermal, acoustic, gas) in the Three.js scene.
 *
 * Visual design uses distinct shapes per sensor type for quick identification:
 * - Thermal: Sphere (radiating heat)
 * - Acoustic: Cone (directional sound)
 * - Gas: Octahedron (chemical/plume shape)
 *
 * Animation effects draw attention to new detections and selected items through
 * pulsing rings and breathing light effects.
 *
 * @example
 * ```tsx
 * <DetectionMarker3D
 *   id="det-001"
 *   position={[100, 0, 200]}
 *   sensorType="thermal"
 *   confidence={0.85}
 *   status="new"
 *   isSelected={false}
 *   onClick={() => selectDetection('det-001')}
 * />
 * ```
 */
export function DetectionMarker3D({
  position,
  sensorType,
  confidence,
  status,
  geometry,
  isSelected,
  onClick,
}: DetectionMarker3DProps) {
  const groupRef = useRef<THREE.Group>(null);
  const pulseRef = useRef<THREE.Mesh>(null);
  const glowRef = useRef<THREE.PointLight>(null);

  const color = SENSOR_COLORS[sensorType];
  const isNew = status === 'new';
  const isConfirmed = status === 'confirmed';

  /**
   * Per-frame animation loop for pulsing effects.
   *
   * Pulse ring scales with sine wave when new or selected.
   * Glow light intensity breathes based on state:
   * - Selected: Higher base intensity, slower pulse
   * - New: Medium intensity, faster pulse
   * - Default: Low steady intensity
   */
  useFrame((state) => {
    if (pulseRef.current && (isNew || isSelected)) {
      const scale = 1 + Math.sin(state.clock.elapsedTime * 4) * 0.2;
      pulseRef.current.scale.setScalar(scale);
    }
    if (glowRef.current) {
      glowRef.current.intensity = isSelected
        ? 2 + Math.sin(state.clock.elapsedTime * 3) * 0.5
        : isNew
          ? 1 + Math.sin(state.clock.elapsedTime * 4) * 0.3
          : 0.5;
    }
  });

  /**
   * Memoized geometry selection based on sensor type.
   * Each sensor has a distinct shape for quick visual identification.
   */
  const markerGeometry = useMemo(() => {
    switch (sensorType) {
      case 'thermal':
        return <sphereGeometry args={[6, 16, 16]} />;
      case 'acoustic':
        return <coneGeometry args={[6, 10, 8]} />;
      case 'gas':
        return <octahedronGeometry args={[7]} />;
    }
  }, [sensorType]);

  const overlayGeometry = useMemo(() => {
    const [originX, originY, originZ] = position;
    return (geometry ?? [])
      .filter((point) => point.every(Number.isFinite))
      .map(([x, y, z]) => [x - originX, y - originY, z - originZ] as [number, number, number]);
  }, [geometry, position]);

  const gasOrThermalPolygon = useMemo(() => {
    if ((sensorType === 'gas' || sensorType === 'thermal') && overlayGeometry.length >= 3) {
      return [...overlayGeometry, overlayGeometry[0]];
    }
    return [];
  }, [overlayGeometry, sensorType]);

  const acousticSegment = useMemo(() => {
    if (sensorType === 'acoustic' && overlayGeometry.length >= 2) {
      return [overlayGeometry[0], overlayGeometry[1]];
    }
    return [];
  }, [overlayGeometry, sensorType]);

  return (
    <group
      ref={groupRef}
      position={position}
      onClick={(e) => {
        e.stopPropagation();
        onClick();
      }}
    >
      <pointLight
        ref={glowRef}
        position={[0, 5, 0]}
        color={color}
        intensity={1}
        distance={40}
      />

      {(isNew || isSelected) && (
        <mesh
          ref={pulseRef}
          rotation={[-Math.PI / 2, 0, 0]}
          position={[0, 0.5, 0]}
        >
          <ringGeometry args={[10, 13, 32]} />
          <meshBasicMaterial
            color={color}
            transparent
            opacity={0.4}
            side={THREE.DoubleSide}
          />
        </mesh>
      )}

      {isSelected && (
        <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, 0.3, 0]}>
          <ringGeometry args={[14, 16, 32]} />
          <meshBasicMaterial
            color="#ffffff"
            transparent
            opacity={0.6}
            side={THREE.DoubleSide}
          />
        </mesh>
      )}

      <mesh position={[0, sensorType === 'acoustic' ? 8 : 6, 0]}>
        {markerGeometry}
        <meshStandardMaterial
          color={color}
          emissive={color}
          emissiveIntensity={isSelected ? 0.8 : isNew ? 0.5 : 0.2}
          transparent
          opacity={status === 'dismissed' ? 0.3 : 0.9}
        />
      </mesh>

      {isConfirmed && (
        <mesh position={[0, 16, 0]}>
          <sphereGeometry args={[3, 12, 12]} />
          <meshStandardMaterial
            color="#22c55e"
            emissive="#22c55e"
            emissiveIntensity={0.5}
          />
        </mesh>
      )}

      {gasOrThermalPolygon.length > 0 && (
        <Line
          points={gasOrThermalPolygon}
          color={color}
          transparent
          opacity={isSelected ? 0.95 : 0.6}
          lineWidth={2}
        />
      )}

      {acousticSegment.length > 0 && (
        <Line
          points={acousticSegment}
          color={color}
          transparent
          opacity={isSelected ? 0.95 : 0.55}
          lineWidth={2}
        />
      )}

      <mesh position={[0, 20, 0]}>
        <cylinderGeometry args={[0.3, 0.3, 40, 8]} />
        <meshBasicMaterial
          color={color}
          transparent
          opacity={isSelected ? 0.5 : 0.2}
        />
      </mesh>

      <Html
        position={[0, 35, 0]}
        center
        distanceFactor={100}
        occlude={false}
        style={{ pointerEvents: 'none' }}
      >
        <div
          className={`
            px-2 py-1 rounded text-xs font-mono whitespace-nowrap
            backdrop-blur-sm border transition-all
            ${isSelected
              ? 'bg-white/20 border-white/40 scale-110'
              : 'bg-black/60 border-white/10'
            }
          `}
          style={{ color }}
        >
          <span className="font-bold">{SENSOR_LABELS[sensorType]}</span>
          <span className="ml-2 text-white/70">{Math.round(confidence * 100)}%</span>
          {isConfirmed && <span className="ml-1 text-green-400">✓</span>}
        </div>
      </Html>
    </group>
  );
}
