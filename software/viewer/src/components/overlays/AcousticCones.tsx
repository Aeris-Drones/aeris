import React, { useRef, useMemo } from 'react';
import { useFrame } from '@react-three/fiber';
import * as THREE from 'three';
import { useAcousticPerception } from '../../hooks/useAcousticPerception';
import { VehicleState } from '../../lib/vehicle/VehicleManager';
import { useLayerVisibility } from '../../context/LayerVisibilityContext';

interface AcousticConesProps {
  vehicles: VehicleState[];
}

export function AcousticCones({ vehicles }: AcousticConesProps) {
  const { detections } = useAcousticPerception();
  const { acoustic } = useLayerVisibility();
  const groupRef = useRef<THREE.Group>(null);

  if (!acoustic) return null;

  useFrame((state) => {
    if (groupRef.current) {
      const pulse = 1 + Math.sin(state.clock.elapsedTime * 4) * 0.05;
      groupRef.current.children.forEach((child) => {
        child.scale.set(pulse, pulse, pulse);
      });
    }
  });

  const cones = useMemo(() => {
    const items: React.ReactNode[] = [];

    detections.forEach((detection) => {
      const vehicle = vehicles.find((v) => v.id === detection.vehicle_id);
      if (!vehicle) return;

      // Cone: 50m height, 30deg half-angle (60deg total)
      const height = 50;
      const radius = height * Math.tan((30 * Math.PI) / 180);

      // Calculate target position from bearing
      // Bearing 0 = North = -Z, Bearing 90 = East = +X
      const bearingRad = (detection.bearing_deg * Math.PI) / 180;
      const dirX = Math.sin(bearingRad);
      const dirZ = -Math.cos(bearingRad);

      const targetPos = new THREE.Vector3(
        vehicle.position.x + dirX * 50,
        vehicle.position.y,
        vehicle.position.z + dirZ * 50
      );

      let opacity = 0.3;
      if (detection.snr_db > 15) opacity = 0.8;
      else if (detection.snr_db >= 10) opacity = 0.5;

      // Color: Green for vocal, Cyan/Blue for standard
      let color = "#00FFFF";
      if (detection.classification === "vocal") {
        color = "#00FF00";
      } else {
          color = "#3B82F6";
      }

      items.push(
        <ConeInstance
            key={detection.vehicle_id}
            position={vehicle.position}
            target={targetPos}
            height={height}
            radius={radius}
            color={color}
            opacity={opacity}
        />
      );
    });

    return items;
  }, [detections, vehicles]);

  return <group ref={groupRef}>{cones}</group>;
}

interface ConeInstanceProps {
  position: THREE.Vector3;
  target: THREE.Vector3;
  height: number;
  radius: number;
  color: string;
  opacity: number;
}

function ConeInstance({ position, target, height, radius, color, opacity }: ConeInstanceProps) {
    return (
        <group position={position}>
            <group name="orienter"
                ref={(node) => {
                    if (node) node.lookAt(target);
                }}
            >
                <mesh rotation={[Math.PI / 2, 0, 0]} position={[0, 0, height/2]}>
                    <coneGeometry args={[radius, height, 32, 1, true]} />
                    <meshBasicMaterial
                        color={color}
                        transparent
                        opacity={opacity}
                        side={THREE.DoubleSide}
                        depthWrite={false}
                        blending={THREE.AdditiveBlending}
                    />
                </mesh>
            </group>
        </group>
    )
}
