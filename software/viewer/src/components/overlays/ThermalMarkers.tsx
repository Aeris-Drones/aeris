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
    const textRef = useRef<(THREE.Object3D & { text?: string; fillOpacity?: number }) | null>(null);
    const texture = useMemo(() => createHotspotTexture(), []);

    // Calculate position once (only changes when origin or data changes)
    const position = useMemo(() => {
        if (!origin) return { x: 0, y: 0, z: 0 };
        const local = geoToLocal({ lat: data.latitude, lon: data.longitude }, origin);
        return { x: local.x, y: data.altitude, z: local.z };
    }, [origin, data.latitude, data.longitude, data.altitude]);

    // Calculate color once (only changes when temperature changes)
    const color = useMemo(() => {
        if (data.temp_c > 50) return new THREE.Color('#FF3333'); // Red
        if (data.temp_c >= 35) return new THREE.Color('#FFA500'); // Orange
        return new THREE.Color('#FFFF00'); // Yellow
    }, [data.temp_c]);

    const tempLabel = useMemo(() => `${data.temp_c.toFixed(1)}°C`, [data.temp_c]);

    useFrame(() => {
        if (!spriteRef.current) return;

        // Update positions
        spriteRef.current.position.set(position.x, position.y, position.z);
        if (textRef.current) {
            textRef.current.position.set(position.x, position.y + 5, position.z);
            textRef.current.text = tempLabel;
        }

        spriteRef.current.material.color = color;

        // Only opacity needs frame-by-frame update for fade effect
        const age = Date.now() - data.lastUpdate;
        const DECAY_MS = 5000;
        const FADE_START = 4000;
        let opacity = 1.0;

        if (age > FADE_START) {
            opacity = Math.max(0, 1.0 - ((age - FADE_START) / (DECAY_MS - FADE_START)));
        }

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
