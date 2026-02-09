'use client';

import React, { useRef, useEffect, useState, useCallback } from 'react';
import { LegacyGCSLayout as GCSLayout } from "@/components/layout/LegacyGCSLayout";
import { Scene3D, Scene3DHandle } from "@/components/Scene3D";
import { ErrorBoundary } from "@/components/ErrorBoundary";
import { CoordinateOriginProvider } from "@/context/CoordinateOriginContext";
import { LayerVisibilityProvider } from "@/context/LayerVisibilityContext";
import { DetectionProvider } from "@/context/DetectionContext";
import { MissionProvider } from "@/context/MissionContext";
import { FleetProvider } from "@/context/FleetContext";
import { ZoneProvider } from "@/context/ZoneContext";
import { ToastProvider } from "@/components/ui/Toast";
import { MiniMap } from "@/components/ui/MiniMap";
import { KeyboardShortcutsOverlay } from "@/components/ui/KeyboardShortcuts";
import { MissionProgress, MissionControlPanel } from "@/components/mission";
import { DetectionPanel, DetectionPanelCompact } from "@/components/detections/DetectionPanel";
import { FleetPanel } from "@/components/fleet/FleetPanel";
import { ZoneDrawingTool, ZoneListPanel } from "@/components/zones";
import { GlassPanel } from "@/components/ui/GlassPanel";
import { cn } from "@/lib/utils";
import { 
  Camera, 
  Eye, 
  Layers, 
  Wifi, 
  WifiOff,
  Clock,
  Target,
  AlertCircle,
  CheckCircle2,
  Pause,
  Radio,
  Plane,
  MapPin
} from "lucide-react";
import { motion, AnimatePresence } from "framer-motion";
import { useMissionControl } from "@/hooks/useMissionControl";
import { formatMissionTime, getMissionPhaseConfig } from "@/types/mission";
import { transitions } from "@/lib/animations";

// ============================================================================
// New Mission-Centric Status Bar
// ============================================================================

function NewStatusBar() {
  const { phase, isPaused, elapsedSeconds, pendingDetections, confirmedSurvivors, rosConnected } = useMissionControl();
  const phaseConfig = getMissionPhaseConfig(phase);
  
  return (
    <div className="flex items-center justify-between h-full px-4 bg-surface-1">
      {/* Left: Logo + Mission Status */}
      <div className="flex items-center gap-4">
        {/* Logo */}
        <div className="flex items-center gap-2">
          <div className="w-8 h-8 rounded-lg bg-gradient-to-br from-info to-info/50 flex items-center justify-center">
            <Radio className="w-4 h-4 text-white" />
          </div>
          <span className="font-semibold text-foreground tracking-tight hidden sm:block">AERIS GCS</span>
        </div>
        
        {/* Divider */}
        <div className="w-px h-6 bg-glass-border" />
        
        {/* Mission Phase */}
        <div className="flex items-center gap-2">
          <motion.div
            key={phase}
            initial={{ opacity: 0, scale: 0.9 }}
            animate={{ opacity: 1, scale: 1 }}
            className={cn(
              "flex items-center gap-1.5 px-2.5 py-1 rounded-md text-xs font-semibold uppercase tracking-wider",
              phaseConfig.bgColor,
              phaseConfig.color
            )}
          >
            {(phase === 'SEARCHING' || phase === 'TRACKING') && !isPaused && (
              <span className="relative flex h-2 w-2">
                <span className="animate-ping absolute inline-flex h-full w-full rounded-full opacity-75 bg-current" />
                <span className="relative inline-flex rounded-full h-2 w-2 bg-current" />
              </span>
            )}
            {isPaused && <Pause className="w-3 h-3" />}
            {phase === 'COMPLETE' && <CheckCircle2 className="w-3 h-3" />}
            {phaseConfig.label}
          </motion.div>
          
          {/* Timer */}
          {phase !== 'IDLE' && (
            <div className="flex items-center gap-1.5 text-muted-foreground">
              <Clock className="w-3.5 h-3.5" />
              <span className={cn(
                "font-mono text-sm tabular-nums",
                isPaused && "text-warning animate-pulse"
              )}>
                {formatMissionTime(elapsedSeconds)}
              </span>
            </div>
          )}
        </div>
      </div>
      
      {/* Center: Detection Summary */}
      <div className="flex items-center gap-4">
        {pendingDetections > 0 && (
          <motion.div
            initial={{ opacity: 0, y: -10 }}
            animate={{ opacity: 1, y: 0 }}
            className="flex items-center gap-1.5 px-2.5 py-1 rounded-md bg-warning/10 border border-warning/20"
          >
            <AlertCircle className="w-3.5 h-3.5 text-warning" />
            <span className="text-xs font-medium text-warning">{pendingDetections} pending</span>
          </motion.div>
        )}
        
        {confirmedSurvivors > 0 && (
          <motion.div
            initial={{ opacity: 0, y: -10 }}
            animate={{ opacity: 1, y: 0 }}
            className="flex items-center gap-1.5 px-2.5 py-1 rounded-md bg-success/10 border border-success/20"
          >
            <Target className="w-3.5 h-3.5 text-success" />
            <span className="text-xs font-medium text-success">{confirmedSurvivors} confirmed</span>
          </motion.div>
        )}
      </div>
      
      {/* Right: Connection Status */}
      <div className="flex items-center gap-3">
        <div className={cn(
          "flex items-center gap-1.5 px-2 py-1 rounded-md text-xs",
          rosConnected 
            ? "bg-success/10 text-success" 
            : "bg-danger/10 text-danger"
        )}>
          {rosConnected ? <Wifi className="w-3.5 h-3.5" /> : <WifiOff className="w-3.5 h-3.5" />}
          <span className="hidden sm:inline">{rosConnected ? "Connected" : "Offline"}</span>
        </div>
      </div>
    </div>
  );
}

// ============================================================================
// Camera Controls (Floating Panel)
// ============================================================================

interface CameraControlsProps {
  onPreset: (preset: 'wide' | 'tracking' | 'overhead') => void;
  className?: string;
}

function CameraControls({ onPreset, className }: CameraControlsProps) {
  return (
    <GlassPanel
      title="Camera"
      icon={<Camera className="w-4 h-4" />}
      collapsible
      defaultCollapsed={false}
      className={cn("w-[200px]", className)}
    >
      <div className="flex flex-col gap-2">
        <button
          onClick={() => onPreset('wide')}
          className={cn(
            "flex items-center gap-2 px-3 py-2 rounded-lg text-sm",
            "bg-surface-2 hover:bg-surface-3 transition-colors",
            "min-h-[var(--touch-min)]"
          )}
        >
          <Eye className="w-4 h-4 text-muted-foreground" />
          Wide View
          <kbd className="ml-auto px-1.5 py-0.5 rounded bg-surface-3 font-mono text-[10px] text-muted-foreground">1</kbd>
        </button>
        <button
          onClick={() => onPreset('tracking')}
          className={cn(
            "flex items-center gap-2 px-3 py-2 rounded-lg text-sm",
            "bg-surface-2 hover:bg-surface-3 transition-colors",
            "min-h-[var(--touch-min)]"
          )}
        >
          <Target className="w-4 h-4 text-muted-foreground" />
          Track Drone
          <kbd className="ml-auto px-1.5 py-0.5 rounded bg-surface-3 font-mono text-[10px] text-muted-foreground">2</kbd>
        </button>
        <button
          onClick={() => onPreset('overhead')}
          className={cn(
            "flex items-center gap-2 px-3 py-2 rounded-lg text-sm",
            "bg-surface-2 hover:bg-surface-3 transition-colors",
            "min-h-[var(--touch-min)]"
          )}
        >
          <Layers className="w-4 h-4 text-muted-foreground" />
          Overhead
          <kbd className="ml-auto px-1.5 py-0.5 rounded bg-surface-3 font-mono text-[10px] text-muted-foreground">3</kbd>
        </button>
      </div>
    </GlassPanel>
  );
}

// ============================================================================
// Main Home Content
// ============================================================================

function HomeContent() {
  const sceneRef = useRef<Scene3DHandle>(null);
  const [cameraPosition, setCameraPosition] = useState<{ x: number; y: number; z: number } | undefined>();
  const [cameraTarget, setCameraTarget] = useState<{ x: number; y: number; z: number } | undefined>();
  const [showCameraControls, setShowCameraControls] = useState(false);
  const [showFleetPanel, setShowFleetPanel] = useState(false);
  const [showZonePanel, setShowZonePanel] = useState(false);
  const { pauseMission, resumeMission, isPaused, isActive } = useMissionControl();

  const handleCameraPreset = useCallback((preset: 'wide' | 'tracking' | 'overhead') => {
    sceneRef.current?.setCameraView(preset);
  }, []);

  const handleTeleport = useCallback((x: number, z: number) => {
    sceneRef.current?.teleportTo(x, z);
  }, []);

  // Poll camera state for mini-map updates
  useEffect(() => {
    const interval = setInterval(() => {
      const state = sceneRef.current?.getCameraState();
      if (state) {
        setCameraPosition({ x: state.position.x, y: state.position.y, z: state.position.z });
        setCameraTarget({ x: state.target.x, y: state.target.y, z: state.target.z });
      }
    }, 100);
    return () => clearInterval(interval);
  }, []);

  // Global Keyboard Shortcuts
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      if (e.target instanceof HTMLInputElement || e.target instanceof HTMLTextAreaElement) return;

      switch (e.key.toLowerCase()) {
        case ' ':
          e.preventDefault();
          if (isActive) {
            if (isPaused) {
              resumeMission();
            } else {
              pauseMission();
            }
          }
          break;
        case 'r':
          handleCameraPreset('wide');
          break;
        case 'c':
          setShowCameraControls(prev => !prev);
          break;
        case 'f':
          setShowFleetPanel(prev => !prev);
          break;
        case 'z':
          setShowZonePanel(prev => !prev);
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
  }, [handleCameraPreset, isActive, isPaused, pauseMission, resumeMission]);

  return (
    <GCSLayout
      statusBar={<NewStatusBar />}
      sidebar={<DetectionPanel />}
      bottomSheet={<DetectionPanelCompact />}
      map={<Scene3D ref={sceneRef} />}
      floatingPanels={
        <>
          {/* Top-left: Mission Progress + Camera Controls */}
          <div className="absolute top-4 left-4 flex flex-col gap-3">
            <MissionProgress collapsible defaultCollapsed={false} />
            <AnimatePresence>
              {showCameraControls && (
                <motion.div
                  initial={{ opacity: 0, y: -10 }}
                  animate={{ opacity: 1, y: 0 }}
                  exit={{ opacity: 0, y: -10 }}
                  transition={transitions.fast}
                >
                  <CameraControls onPreset={handleCameraPreset} />
                </motion.div>
              )}
            </AnimatePresence>
          </div>
          
          {/* Top-right: Fleet + Zone panels + toggle buttons */}
          <div className="absolute top-4 right-4 flex flex-col gap-3 items-end">
            {/* Toggle buttons */}
            <div className="flex gap-2">
              <button
                onClick={() => setShowFleetPanel(prev => !prev)}
                className={cn(
                  "flex items-center gap-2 px-3 py-2 rounded-lg",
                  "bg-glass-bg backdrop-blur-xl border border-glass-border",
                  "text-sm transition-colors min-h-[var(--touch-min)]",
                  showFleetPanel ? "text-info border-info/30" : "text-muted-foreground hover:text-foreground"
                )}
              >
                <Plane className="w-4 h-4" />
                <span className="hidden sm:inline">Fleet</span>
                <kbd className="px-1.5 py-0.5 rounded bg-surface-3 font-mono text-[10px]">F</kbd>
              </button>
              <button
                onClick={() => setShowZonePanel(prev => !prev)}
                className={cn(
                  "flex items-center gap-2 px-3 py-2 rounded-lg",
                  "bg-glass-bg backdrop-blur-xl border border-glass-border",
                  "text-sm transition-colors min-h-[var(--touch-min)]",
                  showZonePanel ? "text-info border-info/30" : "text-muted-foreground hover:text-foreground"
                )}
              >
                <MapPin className="w-4 h-4" />
                <span className="hidden sm:inline">Zones</span>
                <kbd className="px-1.5 py-0.5 rounded bg-surface-3 font-mono text-[10px]">Z</kbd>
              </button>
              <button
                onClick={() => setShowCameraControls(prev => !prev)}
                className={cn(
                  "flex items-center gap-2 px-3 py-2 rounded-lg",
                  "bg-glass-bg backdrop-blur-xl border border-glass-border",
                  "text-sm transition-colors min-h-[var(--touch-min)]",
                  showCameraControls ? "text-info border-info/30" : "text-muted-foreground hover:text-foreground"
                )}
              >
                <Camera className="w-4 h-4" />
                <kbd className="px-1.5 py-0.5 rounded bg-surface-3 font-mono text-[10px]">C</kbd>
              </button>
            </div>
            
            {/* Fleet Panel */}
            <AnimatePresence>
              {showFleetPanel && (
                <motion.div
                  initial={{ opacity: 0, x: 10 }}
                  animate={{ opacity: 1, x: 0 }}
                  exit={{ opacity: 0, x: 10 }}
                  transition={transitions.fast}
                >
                  <FleetPanel />
                </motion.div>
              )}
            </AnimatePresence>
            
            {/* Zone Panel */}
            <AnimatePresence>
              {showZonePanel && (
                <motion.div
                  initial={{ opacity: 0, x: 10 }}
                  animate={{ opacity: 1, x: 0 }}
                  exit={{ opacity: 0, x: 10 }}
                  transition={transitions.fast}
                >
                  <ZoneListPanel />
                </motion.div>
              )}
            </AnimatePresence>
          </div>
          
          {/* Bottom-left: Mission Control + Zone Drawing */}
          <div className="absolute bottom-4 left-4 flex flex-col gap-3">
            <ZoneDrawingTool />
            <MissionControlPanel />
          </div>
          
          {/* Bottom-right: Mini Map */}
          <div className="absolute bottom-4 right-4">
            <GlassPanel
              title="Map"
              collapsible
              defaultCollapsed={false}
              className="w-[220px]"
            >
              <MiniMap
                size={188}
                cameraPosition={cameraPosition}
                cameraTarget={cameraTarget}
                onTeleport={handleTeleport}
              />
            </GlassPanel>
          </div>
        </>
      }
    />
  );
}

export default function Home() {
  return (
    <ErrorBoundary>
      <ToastProvider>
        <CoordinateOriginProvider>
          <LayerVisibilityProvider>
            <DetectionProvider>
              <MissionProvider demoMode initialProgress={{ searchAreaKm2: 2.4, totalDrones: 4, activeDrones: 4 }}>
                <FleetProvider>
                  <ZoneProvider>
                    <HomeContent />
                    <KeyboardShortcutsOverlay />
                  </ZoneProvider>
                </FleetProvider>
              </MissionProvider>
            </DetectionProvider>
          </LayerVisibilityProvider>
        </CoordinateOriginProvider>
      </ToastProvider>
    </ErrorBoundary>
  );
}
