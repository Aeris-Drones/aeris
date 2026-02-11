'use client';

import React, { useRef, useMemo } from 'react';
import { useFrame } from '@react-three/fiber';
import { Html } from '@react-three/drei';
import * as THREE from 'three';

/**
 * Props for the DetectionMarker3D component.
 * Represents a sensor detection (thermal, acoustic, or gas) in the 3D scene.
 */
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
  /** Whether this detection is currently selected */
  isSelected: boolean;
  /** Click handler for selection */
  onClick: () => void;
}

/** Color mapping by sensor type for consistent visual language */
const SENSOR_COLORS = {
  thermal: '#f97316',  // Orange-red - heat signature
  acoustic: '#3b82f6', // Blue - sound waves
  gas: '#eab308',      // Yellow - chemical warning
};

/** Human-readable labels for sensor types */
const SENSOR_LABELS = {
  thermal: 'THERMAL',
  acoustic: 'ACOUSTIC',
  gas: 'GAS',
};

/**
 * DetectionMarker3D - 3D marker for sensor detections.
 *
 * Visual Design:
 * - Shape varies by sensor type: sphere (thermal), cone (acoustic), octahedron (gas)
 * - Pulsing animation for new detections to draw attention
 * - Ground glow effect using point light
 * - Vertical beacon line for visibility from distance
 * - Selection ring and confirmed indicator
 *
 * Animation:
 * - Pulse ring scales with sine wave for new/selected detections
 * - Point light intensity varies for breathing effect
 * - Higher frequency animation for new detections, slower for selected
 */
export function DetectionMarker3D({
  position,
  sensorType,
  confidence,
  status,
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
   * Per-frame animation for pulsing effects.
   *
   * Pulse ring: Scales with sine wave when new or selected
   * Glow light: Intensity breathes based on selection/new state
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
   * Each sensor has a distinct shape for quick visual identification:
   * - Thermal: Sphere (radiating heat)
   * - Acoustic: Cone (directional sound)
   * - Gas: Octahedron (chemical/plume shape)
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

  return (
    <group
      ref={groupRef}
      position={position}
      onClick={(e) => {
        e.stopPropagation();
        onClick();
      }}
    >
      {/* Ground glow - point light for atmospheric effect */}
      <pointLight
        ref={glowRef}
        position={[0, 5, 0]}
        color={color}
        intensity={1}
        distance={40}
      />

      {/* Pulse ring - animated for new or selected detections */}
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

      {/* Selection ring - white ring when selected */}
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

      {/* Main marker geometry - positioned at different heights based on shape */}
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

      {/* Confirmed indicator - green sphere above marker */}
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

      {/* Vertical beacon line - extends upward for visibility */}
      <mesh position={[0, 20, 0]}>
        <cylinderGeometry args={[0.3, 0.3, 40, 8]} />
        <meshBasicMaterial
          color={color}
          transparent
          opacity={isSelected ? 0.5 : 0.2}
        />
      </mesh>

      {/* Info label - HTML overlay with sensor type and confidence */}
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
