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

  // Animation state for pulsing
  useFrame((state) => {
    if (groupRef.current) {
      const pulse = 1 + Math.sin(state.clock.elapsedTime * 4) * 0.05;
      groupRef.current.children.forEach((child) => {
        child.scale.set(pulse, 1, pulse); // Scale width only, keep length? Or uniform?
        // Story says: "Cone pulses slightly (scale 0.95-1.05)"
        // Assuming uniform scale is fine, or just radial.
        // Let's do uniform scale for simplicity
        child.scale.set(pulse, pulse, pulse);
      });
    }
  });

  const cones = useMemo(() => {
    const items: JSX.Element[] = [];

    detections.forEach((detection) => {
      const vehicle = vehicles.find((v) => v.id === detection.vehicle_id);
      if (!vehicle) return;

      // Cone Geometry
      // Height 50m.
      // 30 deg half-angle -> 60 deg cone.
      // Radius = Height * tan(30deg) = 50 * 0.577 = 28.8m
      const height = 50;
      const radius = height * Math.tan((30 * Math.PI) / 180);

      // Rotation
      // ConeGeometry points up (Y) by default.
      // We want it to point along the bearing in the XZ plane.
      // Bearing is Azimuth (0 = North = -Z).

      // Step 1: Align Cone with Z-axis (pointing -Z or +Z).
      // Default Cone is Y-up.
      // Rotate X by -90 deg -> Points +Z (or -Z depending on bottom/top).
      // ConeGeometry: radius, height. Base is at -height/2, Tip at +height/2.
      // Actually standard ConeGeometry: Base at -height/2, Tip at height/2?
      // Let's shift geometry so tip is at origin.

      // We'll use a local transformation in the mesh.

      // Calculate rotation for bearing
      // Azimuth 0 (North) -> -Z.
      // Azimuth 90 (East) -> +X.
      // In Three.js, rotation Y=0 is usually -Z if camera looks -Z?
      // Actually:
      // Object at (0,0,0). LookAt (0,0,-1) -> rotY = 0.
      // LookAt (1,0,0) -> rotY = +90? No, -90 or +90 depending on coord system.
      // Three.js is Right Handed. +Y Up. +Z towards viewer. -Z into screen.
      // Azimuth 0 = North = -Z.
      // Azimuth 90 = East = +X.
      // Angle from -Z to +X is -90 degrees (CCW) or +270?
      // Atan2(x, z): Atan2(0, -1) = 180?

      // Let's use simple math:
      // Bearing 0 -> Rotation Y = PI (to point to +Z) or 0 (to point to -Z)?
      // If we assume the cone points to +Z by default (after laying down):
      // We want bearing 0 to point -Z. So rotate 180 (PI).
      // Bearing 90 to point +X.
      // Rotation logic:
      // rotY = -bearing_rad if 0 is -Z?
      // Let's verify:
      // Bearing 0 -> rot 0 -> points -Z.
      // Bearing 90 -> rot -90 -> points +X?
      // Rot Y +90 moves +Z axis to +X axis?
      // (0,0,1) rotated +90Y -> (1,0,0).
      // So if cone points +Z initially:
      // Bearing 0 (North/-Z): Need rot 180.
      // Bearing 90 (East/+X): Need rot 90 (from +Z to +X).
      // Formula: rotY = (180 - bearing) * DegToRad?

      // Let's simplify: Use `lookAt`.
      // Target point:
      // x = sin(bearing) * dist
      // z = -cos(bearing) * dist (because North is -Z)
      // Wait:
      // Bearing 0: sin(0)=0, -cos(0)=-1 -> (0, -1) -> -Z. Correct.
      // Bearing 90: sin(90)=1, -cos(90)=0 -> (1, 0) -> +X. Correct.
      // So target vector is (sin(B), 0, -cos(B)).

      const bearingRad = (detection.bearing_deg * Math.PI) / 180;
      const dirX = Math.sin(bearingRad);
      const dirZ = -Math.cos(bearingRad);

      const targetPos = new THREE.Vector3(
        vehicle.position.x + dirX * 50,
        vehicle.position.y, // Keep level? Or 3D bearing? Story says "Directional cone". Usually planar for simple bearing.
        vehicle.position.z + dirZ * 50
      );

      // Opacity
      let opacity = 0.3;
      if (detection.snr_db > 15) opacity = 0.8;
      else if (detection.snr_db >= 10) opacity = 0.5;

      // Color
      // "Cyan/Blue: Standard", "Green: Vocal"
      let color = "#00FFFF"; // Cyan
      if (detection.classification === "vocal") {
        color = "#00FF00"; // Green
      } else {
          // Fallback/Standard
          color = "#3B82F6"; // Blue-500 from tailwind, or Cyan
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

function ConeInstance({ position, target, height, radius, color, opacity }: any) {
    const meshRef = useRef<THREE.Mesh>(null);

    useMemo(() => {
        if (meshRef.current) {
            meshRef.current.lookAt(target);
            // Since ConeGeometry is vertical (Y-up), and we looked at target (Z-fwd),
            // We need to rotate the geometry so the cone points along Z.
            // Actually, `lookAt` orients the +Z axis of the object to the target.
            // If we want the cone to point to target, the cone's "point" must be along +Z.
            // Standard Cone: Tip at (0, height/2, 0), Base at (0, -height/2, 0).
            // Rotate X -90 deg: Tip at (0, 0, -height/2)? No.
            // Let's just rotate the mesh -90 on X after lookAt.
            // Easier: Rotate the geometry itself once.
        }
    }, [target]);

    // Construct geometry with tip at +Z or -Z?
    // Let's align tip to +Z.
    // ConeGeometry(radius, height).
    // Default: aligned Y.
    // Rotate X = PI/2 -> aligned +Z (base at -Z) or similar.

    return (
        <group position={position}>
            <group name="orienter"
                ref={(node) => {
                    if (node) node.lookAt(target);
                }}
            >
                {/*
                   Cone along Z axis.
                   Tip should be at distance 'height' away? Or cone extends from 0 to height?
                   Story: "extends 50m from the drone".
                   So Base at 0? No, cone usually implies tip at source?
                   "Directional cones showing sound source bearings".
                   Usually this means the WIDE part is away, and the TIP is at the drone (the source of detection).
                   Or is it a "beam" where it gets wider?
                   A cone expanding OUTWARD means Tip at Drone, Base away.

                   So: Tip at (0,0,0). Base at (0,0, height).
                   Default Cone: Center is (0,0,0).
                   Translate Y by +height/2 -> Base at 0, Tip at height.
                   No, we want Tip at 0.
                   Translate Y by -height/2 -> Tip at 0, Base at -height.

                   Then Rotate X -90 -> Tip at 0, Base at +Z.
                   This aligns with `lookAt` (Z is forward).

                   Wait, ConeGeometry default: Tip at +Y/2, Base at -Y/2.
                   We want Tip at 0 (Drone), Base at +H (Away).
                   Shift Z +H/2. Center at +H/2.
                   We need Tip to be at -H/2 (relative) -> 0 (global).
                   We need Base to be at +H/2 (relative) -> +H (global).

                   Rotate X +90:
                   +Y (Tip) -> -Z.
                   -Y (Base) -> +Z.

                   Result:
                   Tip relative: -Z direction.
                   Base relative: +Z direction.

                   With LookAt(Target), Mesh +Z points to Target.
                   So Base points to Target. Tip points Away from Target?
                   Wait. Tip (relative -Z) points Away from Target (Mesh +Z).
                   So Tip points Backwards (towards Drone).
                   Base points Forwards (towards Target).

                   Shift Z +H/2:
                   Center moves to +H/2 (midway to target).
                   Tip (at relative -H/2 Z) moves to 0.
                   Base (at relative +H/2 Z) moves to H.

                   Correct: Rotation should be +PI/2.
                */}
                <mesh rotation={[Math.PI / 2, 0, 0]} position={[0, 0, height/2]}>
                    <coneGeometry args={[radius, height, 32, 1, true]} />
                    {/* openEnded=true? Maybe. */}
                    <meshBasicMaterial
                        color={color}
                        transparent
                        opacity={opacity}
                        side={THREE.DoubleSide}
                        depthWrite={false} // Important for transparency overlap
                        blending={THREE.AdditiveBlending}
                    />
                </mesh>
            </group>
        </group>
    )
}
