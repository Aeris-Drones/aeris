'use client';

import React, { useRef, forwardRef, useImperativeHandle } from 'react';
import * as THREE from 'three';
import { Canvas, ThreeEvent } from '@react-three/fiber';
import { CameraControls, Grid } from '@react-three/drei';
import { DroneMarker3D, DroneMarker3DProps } from './DroneMarker3D';
import { DetectionMarker3D, DetectionMarker3DProps } from './DetectionMarker3D';
import { FlightTrail3D } from './FlightTrail3D';
import { ZoneOverlay3D, ZoneDrawingPreview } from './ZoneOverlay3D';
import type { PriorityZone, ZonePoint, ZonePriority } from '@/types/zone';
import { useVehicleTelemetry } from '@/hooks/useVehicleTelemetry';

const mockDetections: Omit<DetectionMarker3DProps, 'isSelected' | 'onClick'>[] = [
  {
    id: 'det-1',
    position: [50, 0, -50],
    sensorType: 'thermal',
    confidence: 0.92,
    status: 'new',
  },
  {
    id: 'det-2',
    position: [-80, 0, 120],
    sensorType: 'acoustic',
    confidence: 0.78,
    status: 'reviewing',
  },
  {
    id: 'det-3',
    position: [200, 0, 50],
    sensorType: 'gas',
    confidence: 0.65,
    status: 'confirmed',
  },
  {
    id: 'det-4',
    position: [-150, 0, -80],
    sensorType: 'thermal',
    confidence: 0.88,
    status: 'new',
  },
];

export interface MapScene3DHandle {
  setCameraView: (view: 'wide' | 'tracking' | 'overhead') => void;
  teleportTo: (x: number, z: number) => void;
  getCameraState: () => { position: THREE.Vector3; target: THREE.Vector3 } | null;
}

interface MapScene3DProps {
  selectedDroneId?: string | null;
  selectedDetectionId?: string | null;
  onDroneSelect?: (id: string) => void;
  onDetectionSelect?: (id: string) => void;
  // Zone drawing props
  zones?: PriorityZone[];
  selectedZoneId?: string | null;
  onZoneSelect?: (id: string | null) => void;
  isDrawingZone?: boolean;
  drawingPoints?: ZonePoint[];
  drawingPriority?: ZonePriority;
  onAddZonePoint?: (point: ZonePoint) => void;
}

export const MapScene3D = forwardRef<MapScene3DHandle, MapScene3DProps>(
  ({ 
    selectedDroneId, 
    selectedDetectionId, 
    onDroneSelect, 
    onDetectionSelect,
    zones = [],
    selectedZoneId,
    onZoneSelect,
    isDrawingZone = false,
    drawingPoints = [],
    drawingPriority = 1,
    onAddZonePoint,
  }, ref) => {
    const { vehicles } = useVehicleTelemetry();
    const cameraControlsRef = useRef<CameraControls>(null);

    const telemetryDrones: Omit<DroneMarker3DProps, 'isSelected' | 'onClick'>[] = vehicles.map((vehicle) => {
      const trailPoints: [number, number, number][] = vehicle.trajectory.map((point) => [
        point.x,
        point.y,
        point.z,
      ]);

      return {
        vehicleId: vehicle.id,
        vehicleName: vehicle.id.replace(/_/g, ' ').toUpperCase(),
        position: [vehicle.position.x, vehicle.position.y, vehicle.position.z],
        heading: (vehicle.heading * 180) / Math.PI,
        status: 'active',
        showTrail: trailPoints.length > 1,
        trailPoints,
      };
    });

    useImperativeHandle(ref, () => ({
      setCameraView: (view: 'wide' | 'tracking' | 'overhead') => {
        const controls = cameraControlsRef.current;
        if (!controls) return;

        switch (view) {
          case 'wide':
            controls.setLookAt(0, 400, 400, 0, 0, 0, true);
            break;
          case 'overhead':
            controls.setLookAt(0, 600, 0, 0, 0, 0, true);
            break;
          case 'tracking':
            // Track first active telemetry drone.
            const activeDrone = telemetryDrones.find(d => d.status === 'active');
            if (activeDrone) {
              const [x, y, z] = activeDrone.position;
              controls.setLookAt(x - 80, y + 80, z + 80, x, y, z, true);
            }
            break;
        }
      },
      teleportTo: (x: number, z: number) => {
        const controls = cameraControlsRef.current;
        if (!controls) return;
        controls.setLookAt(x, 200, z + 150, x, 0, z, true);
      },
      getCameraState: () => {
        const controls = cameraControlsRef.current;
        if (!controls) return null;
        const position = new THREE.Vector3();
        const target = new THREE.Vector3();
        controls.getPosition(position);
        controls.getTarget(target);
        return { position, target };
      },
    }));

    return (
      <div className="w-full h-full">
        <Canvas
          camera={{ position: [0, 300, 300], fov: 50 }}
          gl={{ antialias: true, alpha: false }}
          onPointerMissed={() => {
            // Deselect when clicking empty space
            onDroneSelect?.('');
            onDetectionSelect?.('');
          }}
        >
          {/* Dark matter background */}
          <color attach="background" args={['#0d0d12']} />
          
          {/* Minimal lighting for dark aesthetic */}
          <ambientLight intensity={0.3} />
          <directionalLight position={[50, 100, 50]} intensity={0.5} />
          <pointLight position={[0, 200, 0]} intensity={0.2} color="#4488ff" />

          {/* Flight trails (render before drones so trails appear behind) */}
          {telemetryDrones.map((drone) =>
            drone.showTrail && drone.trailPoints.length > 1 ? (
              <FlightTrail3D
                key={`trail-${drone.vehicleId}`}
                points={drone.trailPoints}
                color={drone.status === 'active' ? '#22c55e' : '#f59e0b'}
              />
            ) : null
          )}

          {/* Drone markers */}
          {telemetryDrones.map((drone) => (
            <DroneMarker3D
              key={drone.vehicleId}
              {...drone}
              isSelected={selectedDroneId === drone.vehicleId}
              onClick={() => onDroneSelect?.(drone.vehicleId)}
            />
          ))}

          {/* Detection markers */}
          {mockDetections.map((detection) => (
            <DetectionMarker3D
              key={detection.id}
              {...detection}
              isSelected={selectedDetectionId === detection.id}
              onClick={() => onDetectionSelect?.(detection.id)}
            />
          ))}

          {/* Ground grid - dark matter style */}
          <Grid
            position={[0, -0.5, 0]}
            args={[1000, 1000]}
            cellSize={50}
            sectionSize={200}
            fadeDistance={600}
            fadeStrength={2}
            sectionColor="#1a1a2e"
            cellColor="#12121a"
            infiniteGrid
          />

          {/* Invisible ground plane - only for zone drawing clicks */}
          {isDrawingZone && (
            <mesh 
              rotation={[-Math.PI / 2, 0, 0]} 
              position={[0, -2, 0]}
              onClick={(e: ThreeEvent<MouseEvent>) => {
                if (onAddZonePoint) {
                  e.stopPropagation();
                  const point = e.point;
                  onAddZonePoint({ x: point.x, z: point.z });
                }
              }}
            >
              <planeGeometry args={[2000, 2000]} />
              <meshBasicMaterial visible={false} />
            </mesh>
          )}

          {/* Priority Zones */}
          {zones.map((zone) => (
            <ZoneOverlay3D
              key={zone.id}
              zone={zone}
              isSelected={selectedZoneId === zone.id}
              onClick={() => onZoneSelect?.(zone.id)}
            />
          ))}

          {/* Zone Drawing Preview */}
          {isDrawingZone && drawingPoints.length > 0 && (
            <ZoneDrawingPreview
              points={drawingPoints}
              priority={drawingPriority}
            />
          )}

          <CameraControls
            ref={cameraControlsRef}
            makeDefault
            minDistance={50}
            maxDistance={1000}
            minPolarAngle={0.1}
            maxPolarAngle={Math.PI / 2.1}
          />
        </Canvas>
      </div>
    );
  }
);

MapScene3D.displayName = 'MapScene3D';
