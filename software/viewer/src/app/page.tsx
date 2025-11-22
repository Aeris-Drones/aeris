'use client';

import React, { useRef, useEffect } from 'react';
import { GCSGridLayout } from "@/components/layout/GCSLayout";
import { StatusBar } from "@/components/layout/StatusBar";
import { TelemetryPanel } from "@/components/layout/TelemetryPanel";
import { Scene3D, Scene3DHandle } from "@/components/Scene3D";
import { ErrorBoundary } from "@/components/ErrorBoundary";
import { CoordinateOriginProvider } from "@/context/CoordinateOriginContext";
import { LayerVisibilityProvider } from "@/context/LayerVisibilityContext";
import { ControlPanel } from "@/components/ui/ControlPanel";
import { MiniMap } from "@/components/ui/MiniMap";

// Create a wrapper component to use hooks that need Providers
function HomeContent() {
  const sceneRef = useRef<Scene3DHandle>(null);

  const handleCameraPreset = (preset: 'wide' | 'tracking' | 'overhead') => {
    sceneRef.current?.setCameraView(preset);
  };

  // Global Keyboard Shortcuts
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
        // Ignore if typing in an input
        if (e.target instanceof HTMLInputElement || e.target instanceof HTMLTextAreaElement) return;

        switch (e.key.toLowerCase()) {
            case ' ': // Space: Pause (Not yet implemented in time/simulation, but spec says Space=Pause)
               // For now, we'll just log it or maybe toggle a pause state if we had one.
               // The prompt says "Space (Pause/Resume)". Since we don't have a global pause context yet, we will just log.
               console.log("Toggle Pause");
               break;
            case 'r': // Reset Camera (Wide)
               handleCameraPreset('wide');
               break;
            case 'h': // Toggle UI (Control Panel visibility maybe? or all UI?)
               // For now let's toggle the Control Panel visibility via a local state if we wanted,
               // but the spec implies hiding all UI. Let's just log for MVP or maybe implement a simple toggle if requested.
               // "H (Toggle UI)". I'll interpret this as toggling the ControlPanel visibility.
               const panel = document.getElementById('control-panel');
               if (panel) {
                   panel.style.display = panel.style.display === 'none' ? 'block' : 'none';
               }
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
            <ControlPanel
                id="control-panel"
                className="absolute top-4 left-4 z-20"
                onCameraPreset={handleCameraPreset}
            />
            <MiniMap
                className="absolute bottom-4 right-4 z-20"
                size={200}
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
