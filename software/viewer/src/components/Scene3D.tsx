'use client';

import React, { useRef, forwardRef, useImperativeHandle } from 'react';
import * as THREE from 'three';
import { Canvas } from '@react-three/fiber';
import { CameraControls, Grid, Stats } from '@react-three/drei';
import { useMapTiles } from '../hooks/useMapTiles';
import { useVehicleTelemetry } from '../hooks/useVehicleTelemetry';
import { Tile3D } from './tiles/Tile3D';
import { Vehicle3D } from './vehicles/Vehicle3D';
import { Trajectory3D } from './vehicles/Trajectory3D';
import { AltitudeIndicator } from './vehicles/AltitudeIndicator';
import { ThermalMarkers } from './overlays/ThermalMarkers';
import { AcousticCones } from './overlays/AcousticCones';
import { GasPlume } from './overlays/GasPlume';
import { TileData } from '../lib/map/MapTileManager';
import { VehicleState } from '../lib/vehicle/VehicleManager';
import { useLayerVisibility } from '../context/LayerVisibilityContext';

export interface Scene3DHandle {
    setCameraView: (view: 'wide' | 'tracking' | 'overhead') => void;
    teleportTo: (x: number, z: number) => void;
    getCameraState: () => { position: THREE.Vector3; target: THREE.Vector3 } | null;
}

export const Scene3D = forwardRef<Scene3DHandle, unknown>((props, ref) => {
  const { tiles, stats } = useMapTiles();
  const { vehicles } = useVehicleTelemetry();
  const cameraControlsRef = useRef<CameraControls>(null);
  const { map } = useLayerVisibility();

  useImperativeHandle(ref, () => ({
      setCameraView: (view: 'wide' | 'tracking' | 'overhead') => {
          const controls = cameraControlsRef.current;
          if (!controls) return;

          switch (view) {
              case 'wide':
                  controls.setLookAt(0, 500, 500, 0, 0, 0, true);
                  break;
              case 'overhead':
                  controls.setLookAt(0, 1000, 0, 0, 0, 0, true);
                  break;
              case 'tracking':
                  if (vehicles.length > 0) {
                      const v = vehicles[0];
                      controls.setLookAt(v.position.x - 50, v.position.y + 50, v.position.z + 50, v.position.x, v.position.y, v.position.z, true);
                  }
                  break;
          }
      },
      teleportTo: (x: number, z: number) => {
          const controls = cameraControlsRef.current;
          if (!controls) return;
          controls.setLookAt(x, 300, z + 200, x, 0, z, true);
      },
      getCameraState: () => {
          const controls = cameraControlsRef.current;
          if (!controls) return null;
          const position = new THREE.Vector3();
          const target = new THREE.Vector3();
          controls.getPosition(position);
          controls.getTarget(target);
          return { position, target };
      }
  }));

  return (
    <div className="w-full h-full relative bg-zinc-900">
      <Canvas
        camera={{ position: [0, 500, 500], fov: 50 }}
        shadows
        gl={{ antialias: true }}
      >
        <color attach="background" args={['#1a1a1a']} />
        
        <SceneRendering
            tiles={tiles}
            vehicles={vehicles}
            cameraControlsRef={cameraControlsRef}
            showMap={map}
        />

        <Stats className="custom-stats-position" /> 
      </Canvas>
      
      {/* HUD Overlay - stats display */}
      <div className="absolute top-4 right-4 bg-black/70 text-white p-4 rounded-md backdrop-blur-sm pointer-events-none font-mono text-sm border border-white/10 z-10">
        <h3 className="font-bold text-emerald-400 mb-2">Map Status</h3>
        <div className="flex flex-col gap-1">
          <div className="flex justify-between gap-4">
            <span className="text-white/60">Tiles Loaded:</span>
            <span>{stats.count}</span>
          </div>
          <div className="flex justify-between gap-4">
            <span className="text-white/60">Total Data:</span>
            <span>{(stats.totalBytes / 1024 / 1024).toFixed(2)} MB</span>
          </div>
          <div className="h-px bg-white/10 my-2" />
          <h3 className="font-bold text-emerald-400 mb-2">Vehicles</h3>
           <div className="flex justify-between gap-4">
            <span className="text-white/60">Active:</span>
            <span>{vehicles.length}</span>
          </div>
        </div>
      </div>

      <style jsx global>{`
        .custom-stats-position {
          top: auto !important;
          left: auto !important;
          bottom: 0px !important;
          right: 0px !important;
        }
      `}</style>
       
      <div className="absolute bottom-4 left-4 text-xs text-white/50 pointer-events-none z-10">
        Aeris GCS Viewer v0.1
      </div>
    </div>
  );
});

Scene3D.displayName = 'Scene3D';

interface SceneRenderingProps {
    tiles: TileData[];
    vehicles: VehicleState[];
    cameraControlsRef: React.RefObject<CameraControls | null>;
    showMap: boolean;
}

function SceneRendering({ tiles, vehicles, cameraControlsRef, showMap }: SceneRenderingProps) {
  return (
    <>
       <ambientLight intensity={0.5} />
        <directionalLight
          position={[10, 10, 5]}
          intensity={1}
          castShadow
        />

        {showMap && (
            <group>
            {tiles.map((tile) => (
                <Tile3D
                key={tile.id}
                position={tile.position}
                size={tile.size}
                url={tile.url}
                name={tile.id}
                />
            ))}
            </group>
        )}

        <group>
        {vehicles.map((vehicle) => (
          <React.Fragment key={vehicle.id}>
             <Vehicle3D vehicle={vehicle} />
             <Trajectory3D vehicle={vehicle} />
             <AltitudeIndicator vehicle={vehicle} />
          </React.Fragment>
        ))}
      </group>

        <ThermalMarkers />
        <GasPlume />
        <AcousticCones vehicles={vehicles} />

        <Grid
          position={[0, -0.1, 0]}
          args={[2000, 2000]}
          cellSize={100}
          sectionSize={500}
          fadeDistance={5000}
          sectionColor="#444444"
          cellColor="#222222"
          infiniteGrid
        />

        <CameraControls ref={cameraControlsRef} makeDefault />
    </>
  )
}
