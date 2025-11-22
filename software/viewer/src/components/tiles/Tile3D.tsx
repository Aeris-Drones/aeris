import React, { useEffect, useRef } from 'react';
import * as THREE from 'three';

interface Tile3DProps {
  position: [number, number, number];
  size: number; // width/height (square)
  url: string;
  name?: string;
}

export function Tile3D({ position, size, url, name }: Tile3DProps) {
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
