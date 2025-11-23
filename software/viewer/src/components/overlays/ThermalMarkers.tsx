'use client';

import React, { useRef, useMemo } from 'react';
import { useFrame } from '@react-three/fiber';
import { Text, Billboard } from '@react-three/drei';
import * as THREE from 'three';
import { useThermalPerception, ThermalHotspotData } from '../../hooks/useThermalPerception';
import { useLayerVisibility } from '../../context/LayerVisibilityContext';
import { useCoordinateOrigin } from '../../context/CoordinateOriginContext';
import { geoToLocal } from '../../lib/ros/mapTile';

// Helper to generate texture
function createHotspotTexture() {
  // Check if we are in a browser environment
  if (typeof document === 'undefined') return null;

  const canvas = document.createElement('canvas');
  canvas.width = 64;
  canvas.height = 64;
  const ctx = canvas.getContext('2d');
  if (ctx) {
    // Draw a simple crosshair/diamond
    ctx.strokeStyle = 'white';
    ctx.lineWidth = 4;
    ctx.lineCap = 'round';

    // Crosshair
    ctx.beginPath();
    ctx.moveTo(32, 4); ctx.lineTo(32, 60);
    ctx.moveTo(4, 32); ctx.lineTo(60, 32);
    ctx.stroke();

    // Circle
    ctx.beginPath();
    ctx.arc(32, 32, 12, 0, Math.PI * 2);
    ctx.stroke();

    // Fill center slightly
    ctx.fillStyle = 'rgba(255, 255, 255, 0.5)';
    ctx.fill();
  }
  return new THREE.CanvasTexture(canvas);
}

const Hotspot3D = ({ data }: { data: ThermalHotspotData }) => {
    const { origin } = useCoordinateOrigin();
    const spriteRef = useRef<THREE.Sprite>(null);
    const textRef = useRef<any>(null); // Text component ref
    const texture = useMemo(() => createHotspotTexture(), []);

    useFrame(() => {
        if (!origin || !spriteRef.current) return;

        // Update Position
        const local = geoToLocal({ lat: data.latitude, lon: data.longitude }, origin);
        const x = local.x;
        const y = data.altitude;
        const z = local.z;

        spriteRef.current.position.set(x, y, z);
        if (textRef.current) {
             // Position text slightly above the marker
            textRef.current.position.set(x, y + 5, z);
            textRef.current.text = `${data.temp_c.toFixed(1)}°C`;

            // Update confidence/opacity of text if needed, or just color
            // textRef.current.color = ...
        }

        // Color Logic
        let color = new THREE.Color('#FFFF00'); // Yellow < 35
        if (data.temp_c > 50) color = new THREE.Color('#FF3333'); // Red
        else if (data.temp_c >= 35) color = new THREE.Color('#FFA500'); // Orange

        spriteRef.current.material.color = color;

        // Fade out logic (last 1 second)
        const age = Date.now() - data.lastUpdate;
        const DECAY_MS = 5000;
        const FADE_START = 4000;
        let opacity = 1.0;

        // Fade out at end of life
        if (age > FADE_START) {
            opacity = Math.max(0, 1.0 - ((age - FADE_START) / (DECAY_MS - FADE_START)));
        }

        // Also use confidence for base opacity if desired, but usually confidence is just metadata
        // Let's mix confidence into opacity slightly?
        // For now, just fade out.

        spriteRef.current.material.opacity = opacity;

        if (textRef.current) {
            textRef.current.fillOpacity = opacity;
        }
    });

    if (!texture) return null;

    return (
        <>
            <sprite ref={spriteRef} scale={[8, 8, 8]}>
                <spriteMaterial map={texture} transparent depthTest={false} />
            </sprite>
            <Billboard>
                 <Text
                    ref={textRef}
                    fontSize={3}
                    color="white"
                    anchorX="center"
                    anchorY="bottom"
                    outlineWidth={0.2}
                    outlineColor="#000000"
                >
                    {""}
                </Text>
            </Billboard>
        </>
    );
};

export function ThermalMarkers() {
  const hotspots = useThermalPerception();
  const { thermal } = useLayerVisibility();

  if (!thermal) return null;

  return (
    <group>
      {hotspots.map(hotspot => (
        <Hotspot3D key={hotspot.id} data={hotspot} />
      ))}
    </group>
  );
}
