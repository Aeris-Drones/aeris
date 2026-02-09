'use client';

import React, { useRef } from 'react';
import { useFrame } from '@react-three/fiber';
import { Html } from '@react-three/drei';
import * as THREE from 'three';

export interface DroneMarker3DProps {
  position: [number, number, number];
  heading: number; // degrees
  vehicleId: string;
  vehicleName: string;
  status: 'active' | 'warning' | 'error' | 'returning';
  isSelected: boolean;
  showTrail: boolean;
  trailPoints: [number, number, number][];
  onClick: () => void;
}

const STATUS_COLORS = {
  active: '#22c55e',   // green
  warning: '#f59e0b',  // amber
  error: '#ef4444',    // red
  returning: '#3b82f6', // blue
};

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

  // Animate selection ring
  useFrame((state) => {
    if (ringRef.current && isSelected) {
      ringRef.current.rotation.z = state.clock.elapsedTime * 0.5;
    }
    if (pulseRef.current && isSelected) {
      const scale = 1 + Math.sin(state.clock.elapsedTime * 3) * 0.15;
      pulseRef.current.scale.setScalar(scale);
    }
  });

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
      {/* Status color ring on ground */}
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

      {/* Selection pulse ring */}
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

      {/* Altitude line - vertical dashed line to ground */}
      <mesh position={[0, -position[1] / 2, 0]}>
        <cylinderGeometry args={[0.3, 0.3, position[1], 8]} />
        <meshBasicMaterial color="#ffffff" transparent opacity={0.15} />
      </mesh>

      {/* Drone body */}
      <group rotation={[0, headingRad, 0]}>
        {/* Main body */}
        <mesh castShadow>
          <boxGeometry args={[8, 3, 8]} />
          <meshStandardMaterial
            color={isSelected ? '#ffffff' : '#2a2a3a'}
            emissive={statusColor}
            emissiveIntensity={isSelected ? 0.3 : 0.1}
          />
        </mesh>

        {/* Arms */}
        <mesh position={[0, 0, 0]} rotation={[0, Math.PI / 4, 0]}>
          <boxGeometry args={[20, 1.5, 2]} />
          <meshStandardMaterial color="#1a1a2a" />
        </mesh>
        <mesh position={[0, 0, 0]} rotation={[0, -Math.PI / 4, 0]}>
          <boxGeometry args={[20, 1.5, 2]} />
          <meshStandardMaterial color="#1a1a2a" />
        </mesh>

        {/* Motor pods */}
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

        {/* Direction indicator (nose) */}
        <mesh position={[0, 0, -6]} rotation={[Math.PI / 2, 0, 0]}>
          <coneGeometry args={[2, 4, 8]} />
          <meshStandardMaterial
            color={statusColor}
            emissive={statusColor}
            emissiveIntensity={0.5}
          />
        </mesh>
      </group>

      {/* Name label */}
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
