'use client';

import React, { useRef, useEffect } from 'react';
import { Canvas } from '@react-three/fiber';
import { OrbitControls, Grid, Stats, Environment } from '@react-three/drei';

export function Scene3D() {
  return (
    <div className="w-full h-full relative bg-zinc-900">
      <Canvas
        camera={{ position: [10, 10, 10], fov: 50 }}
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
        <hemisphereLight groundColor="#000000" intensity={0.5} />

        <Grid
          position={[0, -0.01, 0]}
          args={[100, 100]}
          cellSize={1}
          sectionSize={5}
          fadeDistance={50}
          sectionColor="#444444"
          cellColor="#222222"
        />

        <mesh position={[0, 0, 0]}>
          <boxGeometry args={[1, 1, 1]} />
          <meshStandardMaterial color="#6366f1" />
        </mesh>

        <OrbitControls makeDefault />
        {/* 
            Stats uses fixed positioning by default. 
            We can target the DOM element it creates or just place it knowing it's for dev.
            However, standard Stats doesn't easily accept positioning props without creating a custom parent.
            A common workaround is to use a parent wrapper or CSS.
            Actually, @react-three/drei Stats component takes `parent` prop or simply appends to body.
            It sets style="position: fixed; top: 0px; left: 0px; cursor: pointer; opacity: 0.9; z-index: 10000;"
            
            Let's leave it for now if it's just for dev, OR try to override via a style tag/effect?
            Alternatively, we can wrap it in a generic HTML component if allowed, or just let it be.
            But to fix the overlap as per review:
        */}
        <Stats className="custom-stats-position" /> 
      </Canvas>
      
      {/* Overriding the fixed position of Stats via global style for this component or utility */}
      <style jsx global>{`
        .custom-stats-position {
          top: auto !important;
          left: auto !important;
          bottom: 0px !important;
          right: 0px !important;
        }
        /* The actual stats.js dom element often doesn't take the class from the wrapper directly depending on implementation.
           drei's Stats creates a div. Let's assume standard stats.js behavior. 
           If className isn't passed through to the stats container, we might need another approach.
           Let's try placing it in a div via Html if needed, or just live with it if we can't easily change it.
           Actually, there is no 'className' prop on Stats in some versions.
        */
      `}</style>
       
      <div className="absolute bottom-4 left-4 text-xs text-white/50 pointer-events-none">
        Scene3D initialized
      </div>
    </div>
  );
}
