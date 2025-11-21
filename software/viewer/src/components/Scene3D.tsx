'use client';

import React from 'react';
import { Canvas } from '@react-three/fiber';
import { OrbitControls, Grid, Stats } from '@react-three/drei';
import { useMapTiles } from '../hooks/useMapTiles';
import { Tile3D } from './tiles/Tile3D';

export function Scene3D() {
  const { tiles, stats } = useMapTiles();

  // Calculate center of current tiles to adjust camera target or just let user pan?
  // Usually we keep camera relative to origin or follow a drone.
  // For now, just rendering the tiles.

  return (
    <div className="w-full h-full relative bg-zinc-900">
      <Canvas
        camera={{ position: [0, 500, 500], fov: 50 }}
        shadows
        gl={{ antialias: true }}
      >
        <color attach="background" args={['#1a1a1a']} />
        
        <ambientLight intensity={0.5} />
        <directionalLight 
          position={[10, 10, 5]} 
          intensity={1} 
          castShadow 
        />

        {/* Render Tiles */}
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

        <Grid
          position={[0, -0.1, 0]} // Slightly below tiles (y=0)
          args={[2000, 2000]}
          cellSize={100}
          sectionSize={500}
          fadeDistance={5000}
          sectionColor="#444444"
          cellColor="#222222"
          infiniteGrid
        />

        <OrbitControls makeDefault />
        <Stats className="custom-stats-position" /> 
      </Canvas>
      
      {/* HUD Overlay */}
      <div className="absolute top-4 right-4 bg-black/70 text-white p-4 rounded-md backdrop-blur-sm pointer-events-none font-mono text-sm border border-white/10">
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
       
      <div className="absolute bottom-4 left-4 text-xs text-white/50 pointer-events-none">
        Aeris GCS Viewer v0.1
      </div>
    </div>
  );
}
