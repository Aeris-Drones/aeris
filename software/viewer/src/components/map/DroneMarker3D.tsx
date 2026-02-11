'use client';

import React, { useRef } from 'react';
import { useFrame } from '@react-three/fiber';
import { Html } from '@react-three/drei';
import * as THREE from 'three';

/**
 * Props for the DroneMarker3D component.
 * Position uses Three.js coordinate system: [x, y, z] where y is up (altitude).
 */
export interface DroneMarker3DProps {
  /** World position [x, y, z] - y represents altitude above ground */
  position: [number, number, number];
  /** Heading in degrees, 0 = north/+z, increases clockwise */
  heading: number;
  /** Unique vehicle identifier */
  vehicleId: string;
  /** Display name for the vehicle */
  vehicleName: string;
  /** Operational status - drives color and visual indicators */
  status: 'active' | 'warning' | 'error' | 'returning';
  /** Whether this drone is currently selected */
  isSelected: boolean;
  /** Whether to show the flight trail */
  showTrail: boolean;
  /** Array of world positions for the flight trail */
  trailPoints: [number, number, number][];
  /** Click handler for selection */
  onClick: () => void;
}

/** Status color mapping for visual feedback */
const STATUS_COLORS = {
  active: '#22c55e',    // Green - normal operation
  warning: '#f59e0b',   // Amber - attention needed
  error: '#ef4444',     // Red - critical issue
  returning: '#3b82f6', // Blue - return to launch mode
};

/**
 * DroneMarker3D - 3D visual representation of a drone in the scene.
 *
 * Visual Elements:
 * - Ground ring: Status-colored ring at ground level showing position projection
 * - Selection pulse: Animated ring when selected
 * - Altitude line: Vertical line from ground to drone showing height
 * - Drone body: Box body with X-shaped arms and motor pods
 * - Nose cone: Direction indicator colored by status
 * - Label: HTML overlay showing vehicle name and ID
 *
 * Coordinate System Notes:
 * - The ground ring uses rotation [-PI/2, 0, 0] to lay flat on XZ plane
 * - Altitude line height is derived from position[1] (y-coordinate)
 * - Heading rotation is applied to the body group on the Y axis
 */
export function DroneMarker3D({
  position,
  heading,
  vehicleId,
  vehicleName,
  status,
  isSelected,
  onClick,
}: DroneMarker3DProps) {
  const groupRef = useRef<THREE.Group>(null);
  const ringRef = useRef<THREE.Mesh>(null);
  const pulseRef = useRef<THREE.Mesh>(null);
  const statusColor = STATUS_COLORS[status];

  /**
   * Animation loop for selection effects.
   * Runs on every frame via useFrame.
   *
   * Selection ring: Slow continuous rotation
   * Pulse ring: Sine wave scaling for breathing effect
   */
  useFrame((state) => {
    if (ringRef.current && isSelected) {
      ringRef.current.rotation.z = state.clock.elapsedTime * 0.5;
    }
    if (pulseRef.current && isSelected) {
      const scale = 1 + Math.sin(state.clock.elapsedTime * 3) * 0.15;
      pulseRef.current.scale.setScalar(scale);
    }
  });

  // Convert heading from degrees to radians for Three.js rotation
  const headingRad = (heading * Math.PI) / 180;

  return (
    <group
      ref={groupRef}
      position={position}
      onClick={(e) => {
        e.stopPropagation();
        onClick();
      }}
    >
      {/* Ground ring - shows drone position projected to ground plane */}
      <mesh
        ref={ringRef}
        rotation={[-Math.PI / 2, 0, 0]}
        position={[0, -position[1] + 1, 0]}
      >
        <ringGeometry args={[12, 15, 32]} />
        <meshBasicMaterial
          color={statusColor}
          transparent
          opacity={isSelected ? 0.8 : 0.4}
          side={THREE.DoubleSide}
        />
      </mesh>

      {/* Selection pulse ring - animated breathing effect when selected */}
      {isSelected && (
        <mesh
          ref={pulseRef}
          rotation={[-Math.PI / 2, 0, 0]}
          position={[0, -position[1] + 0.5, 0]}
        >
          <ringGeometry args={[16, 18, 32]} />
          <meshBasicMaterial
            color={statusColor}
            transparent
            opacity={0.3}
            side={THREE.DoubleSide}
          />
        </mesh>
      )}

      {/* Altitude line - vertical dashed line from ground to drone */}
      <mesh position={[0, -position[1] / 2, 0]}>
        <cylinderGeometry args={[0.3, 0.3, position[1], 8]} />
        <meshBasicMaterial color="#ffffff" transparent opacity={0.15} />
      </mesh>

      {/* Drone body group - rotated by heading */}
      <group rotation={[0, headingRad, 0]}>
        {/* Main body - centered box */}
        <mesh castShadow>
          <boxGeometry args={[8, 3, 8]} />
          <meshStandardMaterial
            color={isSelected ? '#ffffff' : '#2a2a3a'}
            emissive={statusColor}
            emissiveIntensity={isSelected ? 0.3 : 0.1}
          />
        </mesh>

        {/* X-shaped arms - two crossed boxes at 45 degrees */}
        <mesh position={[0, 0, 0]} rotation={[0, Math.PI / 4, 0]}>
          <boxGeometry args={[20, 1.5, 2]} />
          <meshStandardMaterial color="#1a1a2a" />
        </mesh>
        <mesh position={[0, 0, 0]} rotation={[0, -Math.PI / 4, 0]}>
          <boxGeometry args={[20, 1.5, 2]} />
          <meshStandardMaterial color="#1a1a2a" />
        </mesh>

        {/* Motor pods at arm ends */}
        {[
          [8, 0, 8],
          [-8, 0, 8],
          [8, 0, -8],
          [-8, 0, -8],
        ].map((pos, i) => (
          <mesh key={i} position={pos as [number, number, number]}>
            <cylinderGeometry args={[2.5, 2.5, 2, 16]} />
            <meshStandardMaterial color="#0a0a12" />
          </mesh>
        ))}

        {/* Direction indicator (nose cone) - shows heading direction */}
        <mesh position={[0, 0, -6]} rotation={[Math.PI / 2, 0, 0]}>
          <coneGeometry args={[2, 4, 8]} />
          <meshStandardMaterial
            color={statusColor}
            emissive={statusColor}
            emissiveIntensity={0.5}
          />
        </mesh>
      </group>

      {/* Name label - HTML overlay positioned above drone */}
      <Html
        position={[0, 15, 0]}
        center
        distanceFactor={100}
        occlude={false}
        style={{ pointerEvents: 'none' }}
      >
        <div
          className={`
            px-2 py-1 rounded-md text-xs font-medium whitespace-nowrap
            backdrop-blur-sm border transition-all
            ${isSelected
              ? 'bg-white/20 text-white border-white/40 scale-110'
              : 'bg-black/50 text-white/80 border-white/10'
            }
          `}
        >
          <span className="font-bold">{vehicleName}</span>
          <span className="ml-1.5 opacity-60">{vehicleId}</span>
        </div>
      </Html>
    </group>
  );
}
