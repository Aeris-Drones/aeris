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

import { useState, useRef, useCallback, useEffect } from 'react';
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
import { ZoneToolbar } from '@/components/zones/ZoneToolbar';
import { PiPVideoFeed } from '@/components/pip/PiPVideoFeed';
import { AlertToaster, showAlert, dismissAllAlerts, type Alert } from '@/components/alerts';
import { KeyboardShortcutsOverlay } from '@/components/ui/KeyboardShortcuts';

// Mock fleet data with extended info for sheets
const mockVehicles: VehicleInfo[] = [
  { id: 'scout-1', name: 'Scout 1', status: 'active', battery: 78, altitude: 50, linkQuality: 95, coverage: 32 },
  { id: 'scout-2', name: 'Scout 2', status: 'warning', battery: 45, altitude: 80, linkQuality: 88, coverage: 28 },
  { id: 'ranger-1', name: 'Ranger 1', status: 'returning', battery: 22, altitude: 60, linkQuality: 72, coverage: 45 },
  { id: 'ranger-2', name: 'Ranger 2', status: 'idle', battery: 100, altitude: 0, linkQuality: 100, coverage: 0 },
];

const mockWarnings: VehicleWarning[] = [
  { vehicleId: 'scout-2', message: 'Battery below 50%', severity: 'warning' },
  { vehicleId: 'ranger-1', message: 'Battery critical', severity: 'critical' },
];

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

export default function V2Page() {
  return (
    <LayerVisibilityProvider>
      <ZoneProvider>
        <V2PageContent />
        <AlertToaster visibleToasts={5} />
        <KeyboardShortcutsOverlay />
      </ZoneProvider>
    </LayerVisibilityProvider>
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
  
  // Stored alerts - clicking bell re-shows these as toasts
  const [storedAlerts, setStoredAlerts] = useState<Alert[]>([]);
  const [areAlertsOpen, setAreAlertsOpen] = useState(false);
  
  // Selection state for markers
  const [selectedDroneId, setSelectedDroneId] = useState<string | null>(null);
  const [selectedDetectionId, setSelectedDetectionId] = useState<string | null>(null);
  
  // Mission state
  const [missionPhase, setMissionPhase] = useState<MissionPhase>('SEARCHING');
  const [elapsedTime, setElapsedTime] = useState(847); // ~14 minutes
  const [isPaused, setIsPaused] = useState(false);
  const [progress, setProgress] = useState(42);

  // Detection state
  const [detections, setDetections] = useState<Detection[]>(mockDetections);

  // PiP video feed state
  const [pipVehicleId, setPipVehicleId] = useState<string | null>(null);

  // Ref for camera control
  const mapRef = useRef<MapScene3DHandle>(null);

  // Vehicle handlers (defined before useEffect to avoid dependency issues)
  const handleLocateVehicle = useCallback((id: string) => {
    setSelectedDroneId(id);
    if (mapRef.current) {
      const positions: Record<string, [number, number]> = {
        'scout-1': [0, 0],
        'scout-2': [150, -100],
        'ranger-1': [-120, 80],
        'ranger-2': [0, 0],
      };
      const pos = positions[id] || [0, 0];
      mapRef.current.teleportTo(pos[0], pos[1]);
    }
  }, []);

  const handleViewFeed = useCallback((id: string) => {
    setPipVehicleId(id);
  }, []);
  
  // Add demo alerts on mount (client-side only to avoid hydration issues)
  const hasAddedInitialAlerts = useRef(false);
  useEffect(() => {
    if (!hasAddedInitialAlerts.current) {
      hasAddedInitialAlerts.current = true;
      
      const alerts: Alert[] = [
        {
          id: 'demo-critical',
          severity: 'critical',
          title: 'Scout-2 COMMS LOST',
          description: 'Last contact: 45 seconds ago - Initiating recovery',
          dismissible: false,
          timestamp: new Date(),
          action: { label: 'LOCATE', onClick: () => handleLocateVehicle('scout-2') },
        },
        {
          id: 'demo-warning',
          severity: 'warning',
          title: 'Ranger-1 low battery',
          description: '22% remaining - Auto RTH initiated',
          dismissible: true,
          timestamp: new Date(),
          action: { label: 'VIEW', onClick: () => handleViewFeed('ranger-1') },
        },
      ];
      
      // Store alerts and show as toasts
      setStoredAlerts(alerts);
      alerts.forEach(alert => showAlert(alert));
    }
  }, [handleLocateVehicle, handleViewFeed]);

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

  // Mission control handlers
  const handleStart = useCallback(() => {
    setMissionPhase('SEARCHING');
    setElapsedTime(0);
    setProgress(0);
    setIsPaused(false);
  }, []);

  const handlePause = useCallback(() => {
    setIsPaused(true);
  }, []);

  const handleResume = useCallback(() => {
    setIsPaused(false);
  }, []);

  const handleAbort = useCallback(() => {
    setMissionPhase('ABORTED');
    setIsPaused(false);
  }, []);

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
    console.log('Expand PiP to fullscreen');
    // Future: fullscreen modal
  }, []);

  const handleRTH = useCallback((id: string) => {
    console.log('RTH for', id);
    // Would send RTH command
  }, []);

  // Bell click - toggle toasts: show stored alerts when opening, dismiss all toasts when closing
  const handleAlertClick = useCallback(() => {
    setAreAlertsOpen((prev) => {
      const next = !prev;
      if (next) {
        storedAlerts.forEach(alert => showAlert(alert, { playSound: false }));
      } else {
        dismissAllAlerts();
      }
      return next;
    });
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
              handleResume();
            } else {
              handlePause();
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
          if (droneIndex < mockVehicles.length) {
            handleLocateVehicle(mockVehicles[droneIndex].id);
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
  }, [missionPhase, isPaused, handlePause, handleResume, handleLocateVehicle]);

  // Calculate stats
  const activeVehicles = mockVehicles.filter(v => v.status === 'active' || v.status === 'warning');
  const avgBattery = Math.round(mockVehicles.reduce((sum, v) => sum + v.battery, 0) / mockVehicles.length);
  const avgAltitude = Math.round(activeVehicles.reduce((sum, v) => sum + v.altitude, 0) / (activeVehicles.length || 1));

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
          missionPhase={missionPhase}
          elapsedTime={elapsedTime}
          progressPercent={progress}
          connectionStatus="connected"
          alertCount={storedAlerts.length}
          hasUnreadAlerts={storedAlerts.length > 0}
          onAlertClick={handleAlertClick}
        />
      }

      layersPanel={<LayersPanel />}

      zoneToolbar={<ZoneToolbar />}
      
      commandDock={
        <CommandDock
          fleetCard={
            <FleetSheet
              vehicles={mockVehicles}
              selectedVehicleId={selectedDroneId}
              onLocate={handleLocateVehicle}
              onViewFeed={handleViewFeed}
              onRTH={handleRTH}
              trigger={
                <FleetCard
                  vehicles={mockVehicles}
                  activeCount={activeVehicles.length}
                  totalCount={mockVehicles.length}
                  avgBattery={avgBattery}
                  avgAltitude={avgAltitude}
                  warnings={mockWarnings}
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
              canStart={missionPhase === 'IDLE' || missionPhase === 'COMPLETE' || missionPhase === 'ABORTED'}
              canPause={missionPhase === 'SEARCHING' || missionPhase === 'TRACKING'}
              canAbort={missionPhase === 'SEARCHING' || missionPhase === 'TRACKING'}
              onStart={handleStart}
              onPause={handlePause}
              onResume={handleResume}
              onAbort={handleAbort}
            />
          }
        />
      }

      pipFeed={
        pipVehicleId ? (() => {
          const vehicle = mockVehicles.find(v => v.id === pipVehicleId);
          if (!vehicle) return null;

          const isLive = vehicle.status === 'active' || vehicle.status === 'warning';

          return (
            <PiPVideoFeed
              vehicleId={vehicle.id}
              vehicleName={vehicle.name}
              batteryPercent={vehicle.battery}
              altitude={vehicle.altitude}
              isLive={isLive}
              allVehicles={mockVehicles.map(v => ({
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
