'use client';

import React, { useRef, useEffect, useState } from 'react';
import { GCSGridLayout } from "@/components/layout/GCSLayout";
import { StatusBar } from "@/components/layout/StatusBar";
import { TelemetryPanel } from "@/components/layout/TelemetryPanel";
import { Scene3D, Scene3DHandle } from "@/components/Scene3D";
import { ErrorBoundary } from "@/components/ErrorBoundary";
import { CoordinateOriginProvider } from "@/context/CoordinateOriginContext";
import { LayerVisibilityProvider } from "@/context/LayerVisibilityContext";
import { ControlPanel } from "@/components/ui/ControlPanel";
import { MiniMap } from "@/components/ui/MiniMap";

function HomeContent() {
  const sceneRef = useRef<Scene3DHandle>(null);
  const [cameraPosition, setCameraPosition] = useState<{ x: number; y: number; z: number } | undefined>();
  const [cameraTarget, setCameraTarget] = useState<{ x: number; y: number; z: number } | undefined>();
  const [showControlPanel, setShowControlPanel] = useState(true);

  const handleCameraPreset = (preset: 'wide' | 'tracking' | 'overhead') => {
    sceneRef.current?.setCameraView(preset);
  };

  const handleTeleport = (x: number, z: number) => {
    sceneRef.current?.teleportTo(x, z);
  };

  // Poll camera state for mini-map updates
  useEffect(() => {
    const interval = setInterval(() => {
      const state = sceneRef.current?.getCameraState();
      if (state) {
        setCameraPosition({ x: state.position.x, y: state.position.y, z: state.position.z });
        setCameraTarget({ x: state.target.x, y: state.target.y, z: state.target.z });
      }
    }, 100); // 10 Hz update

    return () => clearInterval(interval);
  }, []);

  // Global Keyboard Shortcuts
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
        if (e.target instanceof HTMLInputElement || e.target instanceof HTMLTextAreaElement) return;

        switch (e.key.toLowerCase()) {
            case ' ':
               e.preventDefault();
               // TODO: Implement pause/resume functionality
               break;
            case 'r':
               handleCameraPreset('wide');
               break;
            case 'h':
               setShowControlPanel(prev => !prev);
               break;
            case '1':
               handleCameraPreset('wide');
               break;
            case '2':
               handleCameraPreset('tracking');
               break;
            case '3':
               handleCameraPreset('overhead');
               break;
        }
    };

    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, []);

  return (
    <GCSGridLayout
      statusBar={<StatusBar />}
      map={
        <div className="relative w-full h-full">
            <Scene3D ref={sceneRef} />
            {showControlPanel && (
              <ControlPanel
                  className="absolute top-4 left-4 z-20"
                  onCameraPreset={handleCameraPreset}
              />
            )}
            <MiniMap
                className="absolute bottom-4 right-4 z-20"
                size={200}
                cameraPosition={cameraPosition}
                cameraTarget={cameraTarget}
                onTeleport={handleTeleport}
            />
        </div>
      }
      telemetry={<TelemetryPanel />}
    />
  );
}

export default function Home() {
  return (
    <ErrorBoundary>
      <CoordinateOriginProvider>
        <LayerVisibilityProvider>
             <HomeContent />
        </LayerVisibilityProvider>
      </CoordinateOriginProvider>
    </ErrorBoundary>
  );
}
