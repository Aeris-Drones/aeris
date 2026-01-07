'use client';

import React, { useRef, useMemo } from 'react';
import { useFrame } from '@react-three/fiber';
import { Html } from '@react-three/drei';
import * as THREE from 'three';

export interface DetectionMarker3DProps {
  id: string;
  position: [number, number, number];
  sensorType: 'thermal' | 'acoustic' | 'gas';
  confidence: number;
  status: 'new' | 'reviewing' | 'confirmed' | 'dismissed';
  isSelected: boolean;
  onClick: () => void;
}

const SENSOR_COLORS = {
  thermal: '#f97316',  // orange-red
  acoustic: '#3b82f6', // blue
  gas: '#eab308',      // yellow
};

const SENSOR_LABELS = {
  thermal: 'THERMAL',
  acoustic: 'ACOUSTIC',
  gas: 'GAS',
};

export function DetectionMarker3D({
  id,
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

  // Pulsing animation for new detections
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

  // Create different shapes based on sensor type
  const markerGeometry = useMemo(() => {
    switch (sensorType) {
      case 'thermal':
        // Circle/sphere for thermal
        return <sphereGeometry args={[6, 16, 16]} />;
      case 'acoustic':
        // Cone for directional acoustic
        return <coneGeometry args={[6, 10, 8]} />;
      case 'gas':
        // Diamond/octahedron for gas plume
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
      {/* Ground glow effect */}
      <pointLight
        ref={glowRef}
        position={[0, 5, 0]}
        color={color}
        intensity={1}
        distance={40}
      />

      {/* Pulse ring for new/selected */}
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

      {/* Selection ring */}
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

      {/* Main marker */}
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

      {/* Confirmed checkmark indicator */}
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

      {/* Vertical beacon line */}
      <mesh position={[0, 20, 0]}>
        <cylinderGeometry args={[0.3, 0.3, 40, 8]} />
        <meshBasicMaterial
          color={color}
          transparent
          opacity={isSelected ? 0.5 : 0.2}
        />
      </mesh>

      {/* Info label */}
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
