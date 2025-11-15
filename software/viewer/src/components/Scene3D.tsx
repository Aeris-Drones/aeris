'use client';

import { useRef } from 'react';
import { Canvas, useFrame } from '@react-three/fiber';
import { OrbitControls, Grid, Stats } from '@react-three/drei';
import * as THREE from 'three';

interface Scene3DProps {
  showStats?: boolean;
  showGrid?: boolean;
}

function GroundPlane() {
  return (
    <mesh rotation={[-Math.PI / 2, 0, 0]} receiveShadow>
      <planeGeometry args={[100, 100]} />
      <meshStandardMaterial color="#2a2a2a" />
    </mesh>
  );
}

function Lights() {
  const directionalLightRef = useRef<THREE.DirectionalLight>(null);

  return (
    <>
      {/* Ambient light for overall illumination */}
      <ambientLight intensity={0.4} />

      {/* Directional light simulating sunlight */}
      <directionalLight
        ref={directionalLightRef}
        position={[10, 20, 10]}
        intensity={1}
        castShadow
        shadow-mapSize-width={2048}
        shadow-mapSize-height={2048}
        shadow-camera-far={50}
        shadow-camera-left={-20}
        shadow-camera-right={20}
        shadow-camera-top={20}
        shadow-camera-bottom={-20}
      />

      {/* Hemisphere light for sky/ground color variation */}
      <hemisphereLight
        args={['#87CEEB', '#545454', 0.3]}
        position={[0, 50, 0]}
      />
    </>
  );
}

function RotatingCube() {
  const meshRef = useRef<THREE.Mesh>(null);

  useFrame((state, delta) => {
    if (meshRef.current) {
      meshRef.current.rotation.y += delta * 0.5;
      meshRef.current.rotation.x += delta * 0.2;
    }
  });

  return (
    <mesh ref={meshRef} position={[0, 1, 0]} castShadow>
      <boxGeometry args={[1, 1, 1]} />
      <meshStandardMaterial color="#00aaff" />
    </mesh>
  );
}

export function Scene3D({ showStats = true, showGrid = true }: Scene3DProps) {
  return (
    <div className="w-full h-full">
      <Canvas
        shadows
        camera={{
          position: [10, 10, 10],
          fov: 50,
          near: 0.1,
          far: 1000,
        }}
        gl={{
          antialias: true,
          alpha: false,
        }}
      >
        {/* Stats overlay for FPS monitoring */}
        {showStats && <Stats />}

        {/* Lighting setup */}
        <Lights />

        {/* Ground plane */}
        <GroundPlane />

        {/* Grid helper */}
        {showGrid && (
          <Grid
            args={[100, 100]}
            cellSize={1}
            cellThickness={0.5}
            cellColor="#444444"
            sectionSize={10}
            sectionThickness={1}
            sectionColor="#666666"
            fadeDistance={50}
            fadeStrength={1}
            followCamera={false}
            infiniteGrid
          />
        )}

        {/* Test object - rotating cube */}
        <RotatingCube />

        {/* Camera controls */}
        <OrbitControls
          enableDamping
          dampingFactor={0.05}
          minDistance={2}
          maxDistance={100}
          maxPolarAngle={Math.PI / 2}
        />
      </Canvas>
    </div>
  );
}
