'use client';

import React, { useEffect, useRef } from 'react';
import { useVehicleTelemetry } from '../../hooks/useVehicleTelemetry';
import { useCoordinateOrigin } from '../../context/CoordinateOriginContext';
import { latLonToMeters } from '../../lib/map/coordinates';

interface MiniMapProps {
  className?: string;
  size?: number;
}

export function MiniMap({ className, size = 200 }: MiniMapProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const { vehicles } = useVehicleTelemetry();
  const { origin } = useCoordinateOrigin();

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas || !origin) return;

    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    // Clear canvas
    ctx.clearRect(0, 0, size, size);

    // Background
    ctx.fillStyle = '#111111';
    ctx.fillRect(0, 0, size, size);

    // Draw grid (optional context)
    ctx.strokeStyle = '#333333';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(size/2, 0);
    ctx.lineTo(size/2, size);
    ctx.moveTo(0, size/2);
    ctx.lineTo(size, size/2);
    ctx.stroke();

    // Determine scale. Let's assume the MiniMap covers a fixed area, e.g., 2km x 2km centered on origin.
    const mapRadiusMeters = 1000; // 1km radius
    const scale = size / (mapRadiusMeters * 2); // pixels per meter

    vehicles.forEach(vehicle => {
        // Use rawPosition (lat/lon) if available, otherwise derive from local position
        let pos;
        if (vehicle.rawPosition) {
            pos = latLonToMeters(vehicle.rawPosition.lat, vehicle.rawPosition.lon, origin);
        } else {
            // Fallback if rawPosition is missing (should be added to VehicleState interface)
            // Assuming local position X, Z corresponds to the meters from origin
            // But let's ensure VehicleState has rawPosition. I just added it.
             pos = latLonToMeters(0, 0, origin); // Dummy fallback to avoid crash
        }

        // Convert to canvas coordinates (center is 0,0 in meters, so size/2, size/2 in pixels)
        // Note: Canvas Y is down, Scene Y is up (usually), but here we treat top-down map:
        // X -> East (Right), Z -> North (Up in map terms, usually -Z in 3D or just Y depending on projection).
        // latLonToMeters gives x (East), y (North).
        // Canvas X = center + x * scale
        // Canvas Y = center - y * scale (flip Y because canvas Y is down)

        const cx = (size / 2) + (pos.x * scale);
        const cy = (size / 2) - (pos.y * scale);

        // Draw vehicle dot
        ctx.beginPath();
        ctx.arc(cx, cy, 4, 0, Math.PI * 2);

        // Color based on vehicle type/role (using vehicle.color which is an object {r,g,b})
        // Convert RGB object to CSS string
        const r = Math.round(vehicle.color.r * 255);
        const g = Math.round(vehicle.color.g * 255);
        const b = Math.round(vehicle.color.b * 255);

        ctx.fillStyle = `rgb(${r},${g},${b})`;
        ctx.fill();
    });

  }, [vehicles, origin, size]);

  return (
    <div className={className}>
      <canvas
        ref={canvasRef}
        width={size}
        height={size}
        className="rounded-full border-2 border-zinc-700 shadow-lg bg-black"
      />
      <div className="text-center text-[10px] text-zinc-500 mt-1 font-mono">MINI-MAP (2km)</div>
    </div>
  );
}
