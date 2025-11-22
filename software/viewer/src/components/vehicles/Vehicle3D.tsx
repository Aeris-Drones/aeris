import React, { useEffect, useRef } from 'react';
import { useFrame } from '@react-three/fiber';
import { VehicleState } from '../../lib/vehicle/VehicleManager';
import { Group } from 'three';
import { Html } from '@react-three/drei';

interface Vehicle3DProps {
  vehicle: VehicleState;
}

export function Vehicle3D({ vehicle }: Vehicle3DProps) {
  const groupRef = useRef<Group>(null);
  const { color, position, orientation, type, id } = vehicle;

  useEffect(() => {
    if (groupRef.current) {
      groupRef.current.position.copy(position);
      groupRef.current.quaternion.copy(orientation);
    }
  }, [id, position, orientation]);

  useFrame(() => {
    if (groupRef.current) {
      groupRef.current.position.lerp(position, 0.1);
      groupRef.current.quaternion.slerp(orientation, 0.1);
    }
  });

  return (
    <group ref={groupRef}>
      <mesh position={[0, 0, 0]}>
        <boxGeometry args={[2, 0.5, 2]} />
        <meshStandardMaterial color={color} />
      </mesh>

      <mesh position={[0, 0, 0]} rotation={[0, Math.PI / 4, 0]}>
        <boxGeometry args={[4, 0.1, 0.2]} />
        <meshStandardMaterial color="#444" />
      </mesh>
      <mesh position={[0, 0, 0]} rotation={[0, -Math.PI / 4, 0]}>
        <boxGeometry args={[4, 0.1, 0.2]} />
        <meshStandardMaterial color="#444" />
      </mesh>

      <mesh position={[0, 0, -1.5]} rotation={[Math.PI / 2, 0, 0]}>
        <coneGeometry args={[0.5, 1, 8]} />
        <meshStandardMaterial color="#FFFF00" />
      </mesh>

      <Html position={[0, 2, 0]} center distanceFactor={15}>
        <div className="bg-black/50 text-white text-xs px-1 rounded backdrop-blur-sm whitespace-nowrap">
          {id} ({type})
        </div>
      </Html>
    </group>
  );
}
