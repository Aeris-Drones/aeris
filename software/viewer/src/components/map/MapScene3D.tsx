'use client';

import React, { useMemo, useRef, forwardRef, useImperativeHandle } from 'react';
import * as THREE from 'three';
import { Canvas, ThreeEvent } from '@react-three/fiber';
import { CameraControls, Grid } from '@react-three/drei';
import { DroneMarker3D, DroneMarker3DProps } from './DroneMarker3D';
import { DetectionMarker3D, DetectionMarker3DProps } from './DetectionMarker3D';
import { FlightTrail3D } from './FlightTrail3D';
import { ZoneOverlay3D, ZoneDrawingPreview } from './ZoneOverlay3D';
import type { PriorityZone, ZonePoint, ZonePriority } from '@/types/zone';
import type { Detection } from '@/components/sheets/DetectionCard';
import { useVehicleTelemetry } from '@/hooks/useVehicleTelemetry';
import { useLayerVisibility } from '@/context/LayerVisibilityContext';

/**
 * Imperative handle interface exposed by MapScene3D.
 * Allows parent components to control camera programmatically.
 */
export interface MapScene3DHandle {
  /** Set predefined camera view: wide (isometric), tracking (follow drone), or overhead (top-down) */
  setCameraView: (view: 'wide' | 'tracking' | 'overhead') => void;
  /** Teleport camera to specific ground coordinates (x, z in world space) */
  teleportTo: (x: number, z: number) => void;
  /** Get current camera position and target for state persistence */
  getCameraState: () => { position: THREE.Vector3; target: THREE.Vector3 } | null;
}

/**
 * Props for the MapScene3D component.
 */
interface MapScene3DProps {
  /** Sensor detections to render in the scene */
  detections?: Detection[];
  /** Currently selected drone ID for highlighting */
  selectedDroneId?: string | null;
  /** Currently selected detection ID for highlighting */
  selectedDetectionId?: string | null;
  /** Callback when a drone marker is clicked */
  onDroneSelect?: (id: string) => void;
  /** Callback when a detection marker is clicked */
  onDetectionSelect?: (id: string) => void;
  /** Priority zones to render on the map */
  zones?: PriorityZone[];
  /** Currently selected zone ID */
  selectedZoneId?: string | null;
  /** Callback when a zone is clicked (null for deselect) */
  onZoneSelect?: (id: string | null) => void;
  /** Whether zone drawing mode is active - enables ground plane click handler */
  isDrawingZone?: boolean;
  /** Current points in the zone being drawn */
  drawingPoints?: ZonePoint[];
  /** Priority level for the zone being drawn */
  drawingPriority?: ZonePriority;
  /** Callback when a point is added during zone drawing */
  onAddZonePoint?: (point: ZonePoint) => void;
}

/**
 * MapScene3D - Main 3D map visualization component.
 *
 * Coordinate System:
 * - Uses standard Three.js right-handed coordinate system
 * - Y is up (altitude), X and Z form the ground plane
 * - Camera looks toward origin from initial position [0, 300, 300]
 *
 * Rendering Architecture:
 * - Canvas with antialiasing for smooth edges
 * - Dark matter aesthetic with custom background color
 * - Layered rendering: grid -> trails -> zones -> markers -> UI
 *
 * Camera Controls:
 * - CameraControls from @react-three/drei for orbit/pan/zoom
 * - Constrained polar angle to prevent going below ground
 * - Min/max distance limits for usable zoom range
 */
export const MapScene3D = forwardRef<MapScene3DHandle, MapScene3DProps>(
  ({
    detections = [],
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
    const { vehicles, returnTrajectories } = useVehicleTelemetry();
    const visibility = useLayerVisibility();
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

    const mapDetections: Omit<DetectionMarker3DProps, 'isSelected' | 'onClick'>[] = useMemo(
      () =>
        detections
          .filter((detection) => {
            if (detection.sensorType === 'thermal') return visibility.thermal;
            if (detection.sensorType === 'acoustic') return visibility.acoustic;
            if (detection.sensorType === 'gas') return visibility.gas;
            return true;
          })
          .map((detection) => ({
            id: detection.id,
            position: detection.position,
            sensorType: detection.sensorType,
            confidence: detection.confidence,
            status: detection.status,
            deliveryMode: detection.deliveryMode,
            isRetroactive: detection.isRetroactive,
            geometry: detection.geometry,
          })),
      [detections, visibility.acoustic, visibility.gas, visibility.thermal]
    );

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
          case 'tracking': {
            const activeDrone = telemetryDrones.find(d => d.status === 'active');
            if (activeDrone) {
              const [x, y, z] = activeDrone.position;
              controls.setLookAt(x - 80, y + 80, z + 80, x, y, z, true);
            }
            break;
          }
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
            onDroneSelect?.('');
            onDetectionSelect?.('');
          }}
        >
          <color attach="background" args={['#0d0d12']} />

          <ambientLight intensity={0.3} />
          <directionalLight position={[50, 100, 50]} intensity={0.5} />
          <pointLight position={[0, 200, 0]} intensity={0.2} color="#4488ff" />

          {/* Render order: trails first so markers appear on top */}
          {visibility.trajectories &&
            telemetryDrones.map((drone) =>
              drone.showTrail && drone.trailPoints.length > 1 ? (
                <FlightTrail3D
                  key={`trail-${drone.vehicleId}`}
                  points={drone.trailPoints}
                  color={drone.status === 'active' ? '#22c55e' : '#f59e0b'}
                />
              ) : null
            )}
          {visibility.trajectories &&
            Object.entries(returnTrajectories).map(([vehicleId, points]) =>
              points.length > 1 ? (
                <FlightTrail3D
                  key={`return-trajectory-${vehicleId}`}
                  points={points}
                  color="#38bdf8"
                  opacity={0.95}
                  lineWidth={4}
                  dashed={false}
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
          {mapDetections.map((detection) => (
            <DetectionMarker3D
              key={detection.id}
              {...detection}
              isSelected={selectedDetectionId === detection.id}
              onClick={() => onDetectionSelect?.(detection.id)}
            />
          ))}

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

          {/* Invisible hit target for zone drawing interactions */}
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

          {zones.map((zone) => (
            <ZoneOverlay3D
              key={zone.id}
              zone={zone}
              isSelected={selectedZoneId === zone.id}
              onClick={() => onZoneSelect?.(zone.id)}
            />
          ))}

          {isDrawingZone && drawingPoints.length > 0 && (
            <ZoneDrawingPreview
              points={drawingPoints}
              priority={drawingPriority}
            />
          )}

          {/* Constrained orbit controls prevent navigation below ground plane */}
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
