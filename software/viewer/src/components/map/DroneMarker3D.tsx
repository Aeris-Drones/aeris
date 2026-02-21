'use client';

import React, { useRef } from 'react';
import { useFrame } from '@react-three/fiber';
import { Html } from '@react-three/drei';
import * as THREE from 'three';

/** Props for the DroneMarker3D component - uses Three.js coordinate system where y is up */
export interface DroneMarker3DProps {
  /** World position [x, y, z] - y represents altitude above ground */
  position: [number, number, number];
  /** Heading in degrees, 0 = north/+z, increases clockwise */
  heading: number;
  /** Unique vehicle identifier - displayed in the marker label */
  vehicleId: string;
  /** Display name for the vehicle - shown in the marker UI */
  vehicleName: string;
  /** Operational status - drives color coding and visual indicators */
  status: 'active' | 'warning' | 'error' | 'returning';
  /** Whether this drone is currently selected - enables selection effects */
  isSelected: boolean;
  /** Whether to show the flight trail - controlled by layer visibility */
  showTrail: boolean;
  /** Array of world positions for the flight trail visualization */
  trailPoints: [number, number, number][];
  /** Click handler for selection - typically updates selection state in parent */
  onClick: () => void;
  /** Delivery mode from replay metadata */
  deliveryMode?: 'live' | 'replayed';
  /** Whether this marker represents replayed telemetry */
  isRetroactive?: boolean;
}

/** Status color mapping for visual feedback across the GCS */
const STATUS_COLORS = {
  active: '#22c55e',
  warning: '#f59e0b',
  error: '#ef4444',
  returning: '#3b82f6',
};

/**
 * 3D visual representation of a drone in the Three.js scene.
 *
 * Renders a detailed drone model with status-driven color coding, selection effects,
 * and an altitude indicator. The visual design includes a ground ring for position
 * reference, a vertical altitude line, and an HTML label overlay.
 *
 * Coordinate system: Three.js standard where y is up. Heading rotation is applied
 * to the body group on the Y axis.
 *
 * @example
 * ```tsx
 * <DroneMarker3D
 *   position={[100, 50, 200]}
 *   heading={45}
 *   vehicleId="UAV-001"
 *   vehicleName="Alpha"
 *   status="active"
 *   isSelected={selectedId === 'UAV-001'}
 *   showTrail={true}
 *   trailPoints={[[100, 50, 190], [100, 50, 200]]}
 *   onClick={() => selectDrone('UAV-001')}
 * />
 * ```
 */
export function DroneMarker3D({
  position,
  heading,
  vehicleId,
  vehicleName,
  status,
  isSelected,
  onClick,
  deliveryMode,
  isRetroactive,
}: DroneMarker3DProps) {
  const groupRef = useRef<THREE.Group>(null);
  const ringRef = useRef<THREE.Mesh>(null);
  const pulseRef = useRef<THREE.Mesh>(null);
  const statusColor = STATUS_COLORS[status];
  const showReplayBadge = deliveryMode === 'replayed' || isRetroactive === true;

  /**
   * Per-frame animation loop for selection effects.
   *
   * Selection ring rotates continuously when selected.
   * Pulse ring scales with sine wave for breathing effect.
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

      <mesh position={[0, -position[1] / 2, 0]}>
        <cylinderGeometry args={[0.3, 0.3, position[1], 8]} />
        <meshBasicMaterial color="#ffffff" transparent opacity={0.15} />
      </mesh>

      <group rotation={[0, headingRad, 0]}>
        <mesh castShadow>
          <boxGeometry args={[8, 3, 8]} />
          <meshStandardMaterial
            color={isSelected ? '#ffffff' : '#2a2a3a'}
            emissive={statusColor}
            emissiveIntensity={isSelected ? 0.3 : 0.1}
          />
        </mesh>

        <mesh position={[0, 0, 0]} rotation={[0, Math.PI / 4, 0]}>
          <boxGeometry args={[20, 1.5, 2]} />
          <meshStandardMaterial color="#1a1a2a" />
        </mesh>
        <mesh position={[0, 0, 0]} rotation={[0, -Math.PI / 4, 0]}>
          <boxGeometry args={[20, 1.5, 2]} />
          <meshStandardMaterial color="#1a1a2a" />
        </mesh>

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

        <mesh position={[0, 0, -6]} rotation={[Math.PI / 2, 0, 0]}>
          <coneGeometry args={[2, 4, 8]} />
          <meshStandardMaterial
            color={statusColor}
            emissive={statusColor}
            emissiveIntensity={0.5}
          />
        </mesh>
      </group>

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
          {showReplayBadge && (
            <span className="ml-2 rounded border border-cyan-300/60 bg-cyan-400/15 px-1.5 py-0.5 text-[10px] uppercase tracking-wide text-cyan-200">
              Replayed
            </span>
          )}
        </div>
      </Html>
    </group>
  );
}
