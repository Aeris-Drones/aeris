import React, { useEffect, useMemo, useRef } from 'react';
import { useFrame } from '@react-three/fiber';
import * as THREE from 'three';

/**
 * PlumeCenterline renders a glowing tube along the plume centerline path,
 * marking the highest concentration corridor through the gas plume.
 *
 * Acceptance Criteria addressed:
 * - AC3: Plume centerline appears as a highlighted glowing path
 * - AC4: Centerline glows distinctly (white/yellow) to stand out
 */

interface PlumeCenterlineProps {
    /** Array of 3D points defining the centerline path */
    centerline: Array<{ x: number; y: number; z: number }>;
    /** Tube radius in meters */
    tubeRadius?: number;
    /** Primary glow color (hex string or THREE.Color) */
    glowColor?: string;
    /** Enable pulse animation */
    pulseAnimation?: boolean;
}

const DEFAULT_RADIUS = 0.8; // meters - slightly thicker for visibility
const DEFAULT_GLOW_COLOR = '#FFFF00'; // Bright yellow
const PULSE_CYCLE_SPEED = 4.0; // Faster pulse for attention
const PULSE_MIN_INTENSITY = 0.6;
const PULSE_MAX_INTENSITY = 1.4;

export function PlumeCenterline({
    centerline,
    tubeRadius = DEFAULT_RADIUS,
    glowColor = DEFAULT_GLOW_COLOR,
    pulseAnimation = true,
}: PlumeCenterlineProps) {
    const meshRef = useRef<THREE.Mesh>(null);
    const materialRef = useRef<THREE.MeshStandardMaterial>(null);

    // Create tube geometry from centerline points
    const geometry = useMemo(() => {
        // Need at least 2 points to create a curve
        if (centerline.length < 2) return null;

        // Convert to THREE.Vector3 array
        const points = centerline.map((p) => new THREE.Vector3(p.x, p.y, p.z));

        // Create smooth curve through centerline points
        const curve = new THREE.CatmullRomCurve3(points, false, 'catmullrom', 0.5);

        // Create tube geometry along the curve
        // Parameters: curve, tubularSegments, radius, radialSegments, closed
        const tubularSegments = Math.max(16, centerline.length * 8);
        const radialSegments = 12;

        const tubeGeometry = new THREE.TubeGeometry(
            curve,
            tubularSegments,
            tubeRadius,
            radialSegments,
            false // not closed
        );

        return tubeGeometry;
    }, [centerline, tubeRadius]);

    useEffect(() => {
        return () => {
            geometry?.dispose();
        };
    }, [geometry]);

    // Pulse animation for glowing effect
    useFrame((state) => {
        if (!pulseAnimation || !materialRef.current) return;

        // Calculate pulse value (oscillates between min and max intensity)
        const t = state.clock.elapsedTime * PULSE_CYCLE_SPEED;
        const pulse =
            PULSE_MIN_INTENSITY +
            (PULSE_MAX_INTENSITY - PULSE_MIN_INTENSITY) * (0.5 + 0.5 * Math.sin(t));

        materialRef.current.emissiveIntensity = pulse;
    });

    // Don't render if no valid geometry
    if (!geometry || centerline.length < 2) return null;

    return (
        <mesh
            ref={meshRef}
            geometry={geometry}
            name="plume-centerline"
            renderOrder={5} // Render on top of plume polygons
        >
            <meshStandardMaterial
                ref={materialRef}
                color={glowColor}
                emissive={glowColor}
                emissiveIntensity={1.0}
                transparent
                opacity={0.95}
                side={THREE.DoubleSide}
                depthWrite={false}
                toneMapped={false} // Prevent tone mapping from dimming the glow
            />
        </mesh>
    );
}
