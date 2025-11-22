'use client';

import React, { useEffect, useRef, useCallback } from 'react';
import { useVehicleTelemetry } from '../../hooks/useVehicleTelemetry';
import { useCoordinateOrigin } from '../../context/CoordinateOriginContext';
import { latLonToMeters } from '../../lib/map/coordinates';

interface CameraState {
  x: number;
  y: number;
  z: number;
}

interface MiniMapProps {
  className?: string;
  size?: number;
  cameraPosition?: CameraState;
  cameraTarget?: CameraState;
  onTeleport?: (x: number, z: number) => void;
}

export function MiniMap({ className, size = 200, cameraPosition, cameraTarget, onTeleport }: MiniMapProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const { vehicles } = useVehicleTelemetry();
  const { origin } = useCoordinateOrigin();

  const mapRadiusMeters = 1000; // 1km radius
  const scale = size / (mapRadiusMeters * 2); // pixels per meter

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas || !origin) return;

    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    ctx.clearRect(0, 0, size, size);

    ctx.fillStyle = '#111111';
    ctx.fillRect(0, 0, size, size);

    ctx.strokeStyle = '#333333';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(size/2, 0);
    ctx.lineTo(size/2, size);
    ctx.moveTo(0, size/2);
    ctx.lineTo(size, size/2);
    ctx.stroke();

    if (cameraPosition) {
      const camX = (size / 2) + (cameraPosition.x * scale);
      const camY = (size / 2) - (cameraPosition.z * scale);

      if (cameraTarget) {
        const targetX = (size / 2) + (cameraTarget.x * scale);
        const targetY = (size / 2) - (cameraTarget.z * scale);

        const dx = targetX - camX;
        const dy = targetY - camY;
        const angle = Math.atan2(dy, dx);

        ctx.save();
        ctx.translate(camX, camY);
        ctx.rotate(angle);

        ctx.beginPath();
        ctx.moveTo(0, 0);
        ctx.lineTo(30, -15);
        ctx.lineTo(30, 15);
        ctx.closePath();

        ctx.fillStyle = 'rgba(255, 255, 255, 0.15)';
        ctx.fill();
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.4)';
        ctx.lineWidth = 1;
        ctx.stroke();

        ctx.restore();
      }

      ctx.beginPath();
      ctx.arc(camX, camY, 5, 0, Math.PI * 2);
      ctx.fillStyle = '#ffffff';
      ctx.fill();
      ctx.strokeStyle = '#000000';
      ctx.lineWidth = 1;
      ctx.stroke();
    }

    vehicles.forEach(vehicle => {
      if (!vehicle.rawPosition) return;

      const pos = latLonToMeters(vehicle.rawPosition.lat, vehicle.rawPosition.lon, origin);

      const cx = (size / 2) + (pos.x * scale);
      const cy = (size / 2) - (pos.y * scale);

      ctx.beginPath();
      ctx.arc(cx, cy, 4, 0, Math.PI * 2);

      const r = Math.round(vehicle.color.r * 255);
      const g = Math.round(vehicle.color.g * 255);
      const b = Math.round(vehicle.color.b * 255);

      ctx.fillStyle = `rgb(${r},${g},${b})`;
      ctx.fill();
    });

  }, [vehicles, origin, size, cameraPosition, cameraTarget, scale]);

  const handleClick = useCallback((e: React.MouseEvent<HTMLCanvasElement>) => {
    if (!onTeleport) return;

    const canvas = canvasRef.current;
    if (!canvas) return;

    const rect = canvas.getBoundingClientRect();
    const clickX = e.clientX - rect.left;
    const clickY = e.clientY - rect.top;

    const worldX = (clickX - size / 2) / scale;
    const worldZ = -(clickY - size / 2) / scale;

    onTeleport(worldX, worldZ);
  }, [onTeleport, size, scale]);

  return (
    <div className={className}>
      <canvas
        ref={canvasRef}
        width={size}
        height={size}
        className="rounded-full border-2 border-zinc-700 shadow-lg bg-black cursor-crosshair"
        onClick={handleClick}
        title="Click to teleport camera"
      />
      <div className="text-center text-[10px] text-zinc-500 mt-1 font-mono">MINI-MAP (2km)</div>
    </div>
  );
}
