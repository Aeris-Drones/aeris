import React, { useEffect, useRef } from 'react';
import * as THREE from 'three';

interface Tile3DProps {
  position: [number, number, number];
  size: number; // width/height (square)
  url: string;
  name?: string;
}

export function Tile3D({ position, size, url, name }: Tile3DProps) {
  // We handle texture loading manually to avoid Suspense bubbling which might flicker the whole scene
  // Given the requirement for progressive loading, individual tiles should pop in.

  const [texture, setTexture] = React.useState<THREE.Texture | null>(null);
  const textureRef = useRef<THREE.Texture | null>(null);

  useEffect(() => {
    let isMounted = true;
    const loader = new THREE.TextureLoader();

    loader.load(
      url,
      (tex) => {
        if (isMounted) {
          tex.colorSpace = THREE.SRGBColorSpace;
          textureRef.current = tex;
          setTexture(tex);
        } else {
          tex.dispose();
        }
      },
      undefined,
      (err) => {
        console.warn(`Failed to load texture for tile ${name}:`, err);
      }
    );

    return () => {
      isMounted = false;
      if (textureRef.current) {
        textureRef.current.dispose();
        textureRef.current = null;
      }
    };
  }, [url, name]);

  // Reuse geometry and material for performance?
  // R3F handles instancing automatically if we use <instances>, but here we have unique textures per tile.
  // So we must use individual meshes.
  // Geometry can be shared if size is constant, but size varies by latitude slightly.
  // However, it varies very little across a local map.
  // We'll just create new geometry per tile for correctness.

  if (!texture) return null;

  return (
    <mesh
      position={position}
      rotation={[-Math.PI / 2, 0, 0]}
    >
      <planeGeometry args={[size, size]} />
      <meshBasicMaterial map={texture} side={THREE.DoubleSide} />
    </mesh>
  );
}
