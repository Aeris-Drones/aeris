'use client';

/**
 * V2 Dashboard - Phase 8 Implementation
 * 
 * Per spec Section 8 Phase 8: Alerts & Polish (Week 8)
 * Goal: Alert system and final polish
 * 
 * Phase 1 (Complete): Design tokens, layout, StatusPill, CommandDock container
 * Phase 2 (Complete): 3D map, DroneMarker3D, DetectionMarker3D, FlightTrail3D
 * Phase 3 (Complete): FleetCard, DetectionsCard, ControlsCard, Progress in StatusPill
 * Phase 4 (Complete): DetectionSheet, DetectionCard, FleetSheet, VehicleCard
 * Phase 5 (Complete): LayersPanel, Layer visibility context
 * Phase 6 (Complete): ZoneToolbar, ZoneOverlay3D, click-to-draw zones
 * Phase 7 (Complete): PiPVideoFeed, Vehicle switcher tabs, Mock placeholder, Expand button
 * Phase 8 (Current):
 * ✓ AlertStack component
 * ✓ Alert severity styling
 * ✓ Auto-dismiss logic
 * - Audio cues (optional)
 * - Animation polish pass
 * - Touch gesture refinement
 * - Keyboard shortcuts
 * - Performance optimization
 * 
 * Deliverable: Production-ready dashboard
 */

import { useState, useRef, useCallback, useEffect, useMemo } from 'react';
import { GCSLayout } from '@/components/layout/GCSLayout';
import { StatusPill, MissionPhase } from '@/components/layout/StatusPill';
import { CommandDock } from '@/components/layout/CommandDock';
import { FleetCard, VehicleWarning } from '@/components/cards/FleetCard';
import { DetectionsCard } from '@/components/cards/DetectionsCard';
import { ControlsCard } from '@/components/cards/ControlsCard';
import { MapScene3D, MapScene3DHandle } from '@/components/map/MapScene3D';
import { FleetSheet } from '@/components/sheets/FleetSheet';
import { DetectionSheet } from '@/components/sheets/DetectionSheet';
import { Detection } from '@/components/sheets/DetectionCard';
import { VehicleInfo } from '@/components/sheets/VehicleCard';
import { LayersPanel } from '@/components/layers/LayersPanel';
import { LayerVisibilityProvider } from '@/context/LayerVisibilityContext';
import { ZoneProvider, useZoneContext } from '@/context/ZoneContext';
import { CoordinateOriginProvider } from '@/context/CoordinateOriginContext';
import { MissionProvider } from '@/context/MissionContext';
import { ZoneToolbar } from '@/components/zones/ZoneToolbar';
import { PiPVideoFeed } from '@/components/pip/PiPVideoFeed';
import { AlertToaster, showAlert, dismissAllAlerts, type Alert } from '@/components/alerts';
import { KeyboardShortcutsOverlay } from '@/components/ui/KeyboardShortcuts';
import { useMissionControl } from '@/hooks/useMissionControl';
import { useVehicleTelemetry } from '@/hooks/useVehicleTelemetry';

// Mock detections data with extended sensor readings
const mockDetections: Detection[] = [
  { 
    id: 'det-1', sensorType: 'thermal', confidence: 0.92, 
    timestamp: Date.now() - 30000, status: 'new', 
    vehicleId: 'scout-1', vehicleName: 'Scout 1', position: [50, 0, -50],
    temperature: 37.2, sector: 'Sector C-4', signatureType: 'Human signature likely'
  },
  { 
    id: 'det-2', sensorType: 'acoustic', confidence: 0.78, 
    timestamp: Date.now() - 120000, status: 'reviewing', 
    vehicleId: 'scout-2', vehicleName: 'Scout 2', position: [-80, 0, 120],
    decibels: 42, sector: 'Sector B-2', signatureType: 'Voice detected'
  },
  { 
    id: 'det-3', sensorType: 'gas', confidence: 0.65, 
    timestamp: Date.now() - 300000, status: 'confirmed', 
    vehicleId: 'scout-1', vehicleName: 'Scout 1', position: [200, 0, 50],
    concentration: 85, sector: 'Sector D-1', signatureType: 'Elevated CO levels'
  },
  { 
    id: 'det-4', sensorType: 'thermal', confidence: 0.88, 
    timestamp: Date.now() - 60000, status: 'new', 
    vehicleId: 'ranger-1', vehicleName: 'Ranger 1', position: [-150, 0, -80],
    temperature: 36.8, sector: 'Sector A-3', signatureType: 'Possible survivor'
  },
];
const INITIAL_ALERT_COUNT = 2;

export default function V2Page() {
  return (
    <CoordinateOriginProvider>
      <LayerVisibilityProvider>
        <ZoneProvider>
          <MissionProvider>
            <V2PageContent />
            <AlertToaster visibleToasts={5} />
            <KeyboardShortcutsOverlay />
          </MissionProvider>
        </ZoneProvider>
      </LayerVisibilityProvider>
    </CoordinateOriginProvider>
  );
}

function V2PageContent() {
  // Zone context
  const {
    zones,
    selectedZoneId,
    selectZone,
    drawing,
    isDrawing,
    addPoint,
  } = useZoneContext();
  
  // Selection state for markers
  const [selectedDroneId, setSelectedDroneId] = useState<string | null>(null);
  const [selectedDetectionId, setSelectedDetectionId] = useState<string | null>(null);
  
  const {
    phase: missionPhase,
    elapsedSeconds,
    coveragePercent,
    isPaused,
    canStart,
    canPause,
    canAbort,
    hasValidStartZone,
    selectedPattern,
    setSelectedPattern,
    startMissionError,
    startMission,
    pauseMission,
    resumeMission,
    abortMission,
    rosConnected,
  } = useMissionControl();
  const { vehicles: telemetryVehicles } = useVehicleTelemetry();

  // Detection state
  const [detections, setDetections] = useState<Detection[]>(mockDetections);

  // PiP video feed state
  const [pipVehicleId, setPipVehicleId] = useState<string | null>(null);

  // Ref for camera control
  const mapRef = useRef<MapScene3DHandle>(null);

  const fleetVehicles = useMemo<VehicleInfo[]>(() => {
    return telemetryVehicles.map((vehicle) => ({
      id: vehicle.id,
      name: vehicle.id.replace(/[_-]/g, ' ').toUpperCase(),
      status: 'active',
      battery: 100,
      altitude: Math.round(vehicle.position.y),
      linkQuality: 100,
      coverage: 0,
    }));
  }, [telemetryVehicles]);

  const vehiclePositionById = useMemo(() => {
    const entries = telemetryVehicles.map((vehicle) => [
      vehicle.id,
      [vehicle.position.x, vehicle.position.z] as [number, number],
    ]);
    return new Map(entries);
  }, [telemetryVehicles]);

  const fleetWarnings = useMemo<VehicleWarning[]>(
    () =>
      fleetVehicles
        .filter((vehicle) => vehicle.battery <= 50)
        .map((vehicle) => ({
          vehicleId: vehicle.id,
          message:
            vehicle.battery <= 25 ? 'Battery critical' : 'Battery below 50%',
          severity: vehicle.battery <= 25 ? 'critical' : 'warning',
        })),
    [fleetVehicles]
  );

  // Vehicle handlers (defined before useEffect to avoid dependency issues)
  const handleLocateVehicle = useCallback((id: string) => {
    setSelectedDroneId(id);
    if (mapRef.current) {
      const pos = vehiclePositionById.get(id);
      if (pos) {
        mapRef.current.teleportTo(pos[0], pos[1]);
      }
    }
  }, [vehiclePositionById]);

  const handleViewFeed = useCallback((id: string) => {
    setPipVehicleId(id);
  }, []);
  
  const storedAlerts = useMemo<Alert[]>(() => [
    {
      id: 'demo-critical',
      severity: 'critical',
      title: 'Scout-2 COMMS LOST',
      description: 'Last contact: 45 seconds ago - Initiating recovery',
      dismissible: false,
      timestamp: new Date(),
      action: { label: 'LOCATE', onClick: () => handleLocateVehicle('scout_2') },
    },
    {
      id: 'demo-warning',
      severity: 'warning',
      title: 'Ranger-1 low battery',
      description: '22% remaining - Auto RTH initiated',
      dismissible: true,
      timestamp: new Date(),
      action: { label: 'VIEW', onClick: () => handleViewFeed('ranger_1') },
    },
  ], [handleLocateVehicle, handleViewFeed]);
  const hasAddedInitialAlerts = useRef(false);
  const areAlertsOpenRef = useRef(false);
  useEffect(() => {
    if (hasAddedInitialAlerts.current) return;
    hasAddedInitialAlerts.current = true;
    storedAlerts.forEach((alert) => showAlert(alert));
  }, [storedAlerts]);

  const handleDroneSelect = (id: string) => {
    setSelectedDroneId(id || null);
    setSelectedDetectionId(null);
  };

  const handleDetectionSelect = useCallback((id: string) => {
    setSelectedDetectionId(id || null);
    setSelectedDroneId(null);
    
    // Zoom to detection when selected from map
    if (id) {
      const detection = detections.find(d => d.id === id);
      if (detection && mapRef.current) {
        mapRef.current.teleportTo(detection.position[0], detection.position[2]);
      }
    }
  }, [detections]);

  // Detection handlers
  const handleConfirmDetection = useCallback((id: string) => {
    setDetections(prev => prev.map(d => 
      d.id === id ? { ...d, status: 'confirmed' as const } : d
    ));
  }, []);

  const handleDismissDetection = useCallback((id: string) => {
    setDetections(prev => prev.map(d => 
      d.id === id ? { ...d, status: 'dismissed' as const } : d
    ));
  }, []);

  const handleLocateDetection = useCallback((id: string) => {
    const detection = detections.find(d => d.id === id);
    if (detection && mapRef.current) {
      mapRef.current.teleportTo(detection.position[0], detection.position[2]);
    }
    setSelectedDetectionId(id);
  }, [detections]);

  const handleClosePip = useCallback(() => {
    setPipVehicleId(null);
  }, []);

  const handleExpandPip = useCallback(() => {
    // Future: fullscreen modal
  }, []);

  const handleRTH = useCallback((id: string) => {
    // Would send RTH command
    void id;
  }, []);

  // Bell click - toggle toasts: show stored alerts when opening, dismiss all toasts when closing
  const handleAlertClick = useCallback(() => {
    areAlertsOpenRef.current = !areAlertsOpenRef.current;
    if (areAlertsOpenRef.current) {
      storedAlerts.forEach((alert) => showAlert(alert, { playSound: false }));
      return;
    }
    dismissAllAlerts();
  }, [storedAlerts]);

  // Keyboard shortcuts per spec Section 5.2
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      // Ignore if user is typing in an input
      if (e.target instanceof HTMLInputElement || e.target instanceof HTMLTextAreaElement) return;

      switch (e.key) {
        case ' ': // Space - Pause/Resume mission
          e.preventDefault();
          if (missionPhase === 'SEARCHING' || missionPhase === 'TRACKING') {
            if (isPaused) {
              resumeMission();
            } else {
              pauseMission();
            }
          }
          break;
        case 'Escape': // Cancel current action
          e.preventDefault();
          setSelectedDroneId(null);
          setSelectedDetectionId(null);
          break;
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
          // Select drone 1-6
          const droneIndex = parseInt(e.key) - 1;
          if (droneIndex < fleetVehicles.length) {
            handleLocateVehicle(fleetVehicles[droneIndex].id);
          }
          break;
        case 'r':
        case 'R':
          // Reset camera to default view
          if (mapRef.current) {
            mapRef.current.teleportTo(0, 0);
          }
          break;
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [missionPhase, isPaused, pauseMission, resumeMission, handleLocateVehicle, fleetVehicles]);

  // Calculate stats
  const activeVehicles = fleetVehicles.filter(v => v.status === 'active' || v.status === 'warning');
  const avgBattery = Math.round(
    fleetVehicles.reduce((sum, v) => sum + v.battery, 0) / (fleetVehicles.length || 1)
  );
  const avgAltitude = Math.round(
    activeVehicles.reduce((sum, v) => sum + v.altitude, 0) / (activeVehicles.length || 1)
  );

  // Detection stats
  const thermalCount = detections.filter(d => d.sensorType === 'thermal').length;
  const acousticCount = detections.filter(d => d.sensorType === 'acoustic').length;
  const gasCount = detections.filter(d => d.sensorType === 'gas').length;
  const pendingCount = detections.filter(d => d.status === 'new' || d.status === 'reviewing').length;
  const confirmedCount = detections.filter(d => d.status === 'confirmed').length;

  return (
    <GCSLayout
      map={
        <MapScene3D
          ref={mapRef}
          selectedDroneId={selectedDroneId}
          selectedDetectionId={selectedDetectionId}
          onDroneSelect={handleDroneSelect}
          onDetectionSelect={handleDetectionSelect}
          zones={zones}
          selectedZoneId={selectedZoneId}
          onZoneSelect={selectZone}
          isDrawingZone={isDrawing}
          drawingPoints={drawing.points}
          drawingPriority={drawing.currentPriority}
          onAddZonePoint={addPoint}
        />
      }
      
      statusPill={
        <StatusPill
          missionPhase={missionPhase as MissionPhase}
          elapsedTime={elapsedSeconds}
          progressPercent={Math.round(coveragePercent)}
          connectionStatus={rosConnected ? "connected" : "disconnected"}
          alertCount={INITIAL_ALERT_COUNT}
          hasUnreadAlerts={INITIAL_ALERT_COUNT > 0}
          onAlertClick={handleAlertClick}
        />
      }

      layersPanel={<LayersPanel />}

      zoneToolbar={<ZoneToolbar />}
      
      commandDock={
        <CommandDock
          fleetCard={
            <FleetSheet
              vehicles={fleetVehicles}
              selectedVehicleId={selectedDroneId}
              onLocate={handleLocateVehicle}
              onViewFeed={handleViewFeed}
              onRTH={handleRTH}
              trigger={
                <FleetCard
                  vehicles={fleetVehicles}
                  activeCount={activeVehicles.length}
                  totalCount={fleetVehicles.length}
                  avgBattery={avgBattery}
                  avgAltitude={avgAltitude}
                  warnings={fleetWarnings}
                />
              }
            />
          }
          detectionsCard={
            <DetectionSheet
              detections={detections}
              onConfirm={handleConfirmDetection}
              onDismiss={handleDismissDetection}
              onLocate={handleLocateDetection}
              trigger={
                <DetectionsCard
                  thermalCount={thermalCount}
                  acousticCount={acousticCount}
                  gasCount={gasCount}
                  pendingCount={pendingCount}
                  confirmedCount={confirmedCount}
                />
              }
            />
          }
          controlsCard={
            <ControlsCard
              missionPhase={missionPhase}
              isPaused={isPaused}
              canStart={canStart}
              canPause={canPause}
              canAbort={canAbort}
              hasValidStartZone={hasValidStartZone}
              selectedPattern={selectedPattern}
              setSelectedPattern={setSelectedPattern}
              startMissionError={startMissionError}
              onStart={startMission}
              onPause={pauseMission}
              onResume={resumeMission}
              onAbort={abortMission}
            />
          }
        />
      }

      pipFeed={
        pipVehicleId ? (() => {
          const vehicle = fleetVehicles.find(v => v.id === pipVehicleId);
          if (!vehicle) return null;

          const isLive = vehicle.status === 'active' || vehicle.status === 'warning';

          return (
            <PiPVideoFeed
              vehicleId={vehicle.id}
              vehicleName={vehicle.name}
              batteryPercent={vehicle.battery}
              altitude={vehicle.altitude}
              isLive={isLive}
              allVehicles={fleetVehicles.map(v => ({
                id: v.id,
                name: v.name,
                status: v.status,
              }))}
              onVehicleSwitch={setPipVehicleId}
              onClose={handleClosePip}
              onExpand={handleExpandPip}
            />
          );
        })() : undefined
      }
      alerts={undefined}
    />
  );
}
