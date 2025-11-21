import React, { useRef } from 'react';
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

  // Smooth interpolation in render loop
  useFrame(() => {
    if (groupRef.current) {
      // Simple lerp for position
      groupRef.current.position.lerp(position, 0.1); // Adjust factor for smoothness/lag

      // Spherical lerp for rotation
      // groupRef.current.quaternion.slerp(orientation, 0.1);
      // Note: vehicle.orientation is currently calculated directly from telemetry.
      // If telemetry is jittery, this will be jittery.
      groupRef.current.quaternion.copy(orientation);
    }
  });

  return (
    <group ref={groupRef} position={position} quaternion={orientation}>
      {/* Main Body */}
      <mesh position={[0, 0, 0]}>
        <boxGeometry args={[2, 0.5, 2]} />
        <meshStandardMaterial color={color} />
      </mesh>

      {/* Arms (Cross shape) */}
      <mesh position={[0, 0, 0]} rotation={[0, Math.PI / 4, 0]}>
        <boxGeometry args={[4, 0.1, 0.2]} />
        <meshStandardMaterial color="#444" />
      </mesh>
      <mesh position={[0, 0, 0]} rotation={[0, -Math.PI / 4, 0]}>
        <boxGeometry args={[4, 0.1, 0.2]} />
        <meshStandardMaterial color="#444" />
      </mesh>

      {/* Heading Indicator (Cone) */}
      {/* Pointing along -Z (North) or +X (East)?
          If Heading 0 = North (-Z), then the drone forward is -Z.
          Let's assume drone 'forward' is its local -Z axis if standard Three.js object.
          Wait, usually forward is +Z or -Z.
          Let's put the cone on the "Front".
          If we assumed Yaw matches rotation around Y, and vehicle body is aligned.
          Let's assume forward is +X for now (standard math), but usually vehicles forward is X in ROS, -Z in OpenGL.
          Let's try -Z (North in our map).
      */}
      <mesh position={[0, 0, -1.5]} rotation={[Math.PI / 2, 0, 0]}>
        <coneGeometry args={[0.5, 1, 8]} />
        <meshStandardMaterial color="#FFFF00" />
      </mesh>

      {/* Label */}
      <Html position={[0, 2, 0]} center distanceFactor={15}>
        <div className="bg-black/50 text-white text-xs px-1 rounded backdrop-blur-sm whitespace-nowrap">
          {id} ({type})
        </div>
      </Html>
    </group>
  );
}
