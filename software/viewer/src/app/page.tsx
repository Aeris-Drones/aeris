'use client';

import { Suspense, useState, useRef, useCallback, useEffect, useMemo } from 'react';
import { usePathname, useRouter, useSearchParams } from 'next/navigation';
import { GCSLayout } from '@/components/layout/GCSLayout';
import { StatusPill, MissionPhase } from '@/components/layout/StatusPill';
import { CommandDock } from '@/components/layout/CommandDock';
import { FleetCard, VehicleWarning } from '@/components/cards/FleetCard';
import { DetectionsCard } from '@/components/cards/DetectionsCard';
import { ControlsCard } from '@/components/cards/ControlsCard';
import { EmergencyStopControl } from '@/components/mission/EmergencyStopControl';
import { MapScene3D, MapScene3DHandle } from '@/components/map/MapScene3D';
import { FleetSheet } from '@/components/sheets/FleetSheet';
import { DetectionSheet } from '@/components/sheets/DetectionSheet';
import { Detection } from '@/components/sheets/DetectionCard';
import { VehicleInfo } from '@/components/sheets/VehicleCard';
import { LayersPanel } from '@/components/layers/LayersPanel';
import { LayerVisibilityProvider } from '@/context/LayerVisibilityContext';
import { useLayerVisibility } from '@/context/LayerVisibilityContext';
import { ZoneProvider, useZoneContext } from '@/context/ZoneContext';
import { CoordinateOriginProvider } from '@/context/CoordinateOriginContext';
import { ROSConnectionProvider } from '@/context/ROSConnectionContext';
import { MissionProvider } from '@/context/MissionContext';
import { ZoneToolbar } from '@/components/zones/ZoneToolbar';
import { PiPVideoFeed } from '@/components/pip/PiPVideoFeed';
import { AlertToaster, showAlert, dismissAlert, type Alert } from '@/components/alerts';
import { KeyboardShortcutsOverlay } from '@/components/ui/KeyboardShortcuts';
import { Switch } from '@/components/ui/switch';
import { useMissionControl } from '@/hooks/useMissionControl';
import { useVehicleTelemetry } from '@/hooks/useVehicleTelemetry';
import { useFusedDetections } from '@/hooks/useFusedDetections';
import { applyDetectionStatusOverrides, computeDetectionCounts } from '@/lib/detectionViewState';
import { normalizeVehicleId } from '@/lib/missionProgressVehicleMeta';
import { applyVehicleMissionMeta } from '@/lib/fleetVehicleProjection';
import { normalizeMissionMetaForVehicle } from '@/lib/degradedVehicleState';
import { isIcViewModeQueryValue } from '@/lib/icViewMode';
import {
  deriveRouteRecommendations,
} from '@/lib/routeRecommendations';

/**
 * Mock detections for UI demonstration when ROS telemetry is unavailable.
 * In production, these are replaced by fused detections from
 * /detections/fused via rosbridge.
 */
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

const MOCK_ALERT_IDS = ['demo-critical', 'demo-warning'] as const;

/**
 * Root page component for the Aeris GCS.
 *
 * Provider hierarchy (outer to inner):
 * - CoordinateOriginProvider: Global lat/lon origin for local ENU coordinate transforms
 * - ROSConnectionProvider: Shared rosbridge connection state for downstream telemetry hooks
 * - LayerVisibilityProvider: Map layer toggle state (thermal, gas, acoustic, trajectories)
 * - ZoneProvider: Search zone CRUD and polygon drawing state
 * - MissionProvider: Mission lifecycle state and command history
 *
 * Separation of V2Page and V2PageContent allows providers to be established before
 * any hooks/contexts are accessed.
 */
export default function V2Page() {
  return (
    <CoordinateOriginProvider>
      <ROSConnectionProvider>
        <LayerVisibilityProvider>
          <ZoneProvider>
            <MissionProvider>
              <Suspense fallback={null}>
                <V2PageContent />
              </Suspense>
              <AlertToaster visibleToasts={5} />
              <KeyboardShortcutsOverlay />
            </MissionProvider>
          </ZoneProvider>
        </LayerVisibilityProvider>
      </ROSConnectionProvider>
    </CoordinateOriginProvider>
  );
}

function V2PageContent() {
  const router = useRouter();
  const pathname = usePathname();
  const searchParams = useSearchParams();
  const layerVisibility = useLayerVisibility();
  const {
    zones,
    structuralHazardZones,
    selectedZoneId,
    selectZone,
    drawing,
    isDrawing,
    addPoint,
    routeStagingArea,
    isPlacingRouteStagingArea,
    setRouteStagingAreaPosition,
  } = useZoneContext();

  const [selectedDroneId, setSelectedDroneId] = useState<string | null>(null);
  const [selectedDetectionId, setSelectedDetectionId] = useState<string | null>(null);
  const [routeNowMs, setRouteNowMs] = useState(() => Date.now());
  const [icViewModeEnabled, setIcViewModeEnabled] = useState(() =>
    isIcViewModeQueryValue(searchParams.get('ic'))
  );

  const {
    phase: missionPhase,
    elapsedSeconds,
    coveragePercent,
    isPaused,
    canStart,
    canPause,
    canAbort,
    abortUnavailableReason,
    hasValidStartZone,
    selectedPattern,
    setSelectedPattern,
    startMissionError,
    abortMissionError,
    startMission,
    pauseMission,
    resumeMission,
    abortMission,
    rosConnected,
    vehicleSlamModes,
    vehicleMissionMeta,
  } = useMissionControl();
  const {
    vehicles: telemetryVehicles,
    returnTrajectories,
  } = useVehicleTelemetry();

  const [detectionStatusOverrides, setDetectionStatusOverrides] = useState<Record<string, Detection['status']>>({});
  const [pipVehicleId, setPipVehicleId] = useState<string | null>(null);
  const mapRef = useRef<MapScene3DHandle>(null);
  const {
    detections: liveFusedDetections,
    isConnected: fusedFeedConnected,
  } = useFusedDetections(telemetryVehicles);
  const allowMockFallback =
    process.env.NODE_ENV !== 'production' && !rosConnected;

  useEffect(() => {
    setIcViewModeEnabled(isIcViewModeQueryValue(searchParams.get('ic')));
  }, [searchParams]);

  useEffect(() => {
    if (!icViewModeEnabled) {
      return;
    }

    (['map', 'thermal', 'gas', 'acoustic', 'trajectories', 'routes'] as const).forEach((layer) => {
      if (!layerVisibility[layer]) {
        layerVisibility.setLayer(layer, true);
      }
    });
  }, [icViewModeEnabled, layerVisibility]);

  useEffect(() => {
    if (icViewModeEnabled) {
      setPipVehicleId(null);
    }
  }, [icViewModeEnabled]);

  const setIcModeFromUi = useCallback((enabled: boolean) => {
    const params = new URLSearchParams(searchParams.toString());
    if (enabled) {
      params.set('ic', '1');
    } else {
      params.delete('ic');
    }

    const query = params.toString();
    router.replace(query ? `${pathname}?${query}` : pathname, { scroll: false });
  }, [pathname, router, searchParams]);

  const baseDetections = useMemo<Detection[]>(
    () => {
      if (fusedFeedConnected) {
        return liveFusedDetections;
      }
      return allowMockFallback ? mockDetections : [];
    },
    [allowMockFallback, fusedFeedConnected, liveFusedDetections]
  );
  const detections = useMemo<Detection[]>(
    () => applyDetectionStatusOverrides(baseDetections, detectionStatusOverrides),
    [baseDetections, detectionStatusOverrides]
  );

  useEffect(() => {
    const intervalId = window.setInterval(() => {
      setRouteNowMs(Date.now());
    }, 5_000);
    return () => window.clearInterval(intervalId);
  }, []);

  const routeRecommendations = useMemo(
    () =>
      deriveRouteRecommendations({
        detections,
        stagingArea: routeStagingArea,
        structuralHazards: structuralHazardZones,
        nowMs: routeNowMs,
      }),
    [detections, routeNowMs, routeStagingArea, structuralHazardZones]
  );

  /**
   * Transforms ROS telemetry into FleetCard-compatible vehicle summaries.
   * Altitude is extracted from the Y axis (ENU coordinate frame, up is positive).
   * ENU (East-North-Up) is the standard aerospace coordinate frame used throughout
   * the Aeris GCS for consistent spatial reasoning.
   */
  const fleetVehicles = useMemo<VehicleInfo[]>(() => {
    return telemetryVehicles.map((vehicle) => {
      const missionMeta = normalizeMissionMetaForVehicle(vehicle.id, vehicleMissionMeta);
      return applyVehicleMissionMeta(
        {
          status: (vehicle.deliveryMode === 'replayed' || vehicle.isRetroactive
            ? 'warning'
            : 'active') as VehicleInfo['status'],
          id: vehicle.id,
          name: vehicle.id.replace(/[_-]/g, ' ').toUpperCase(),
          battery: typeof vehicle.batteryPercent === 'number'
            ? Math.round(vehicle.batteryPercent)
            : null,
          altitude: Math.round(vehicle.position.y),
          linkQuality: typeof vehicle.linkQualityPercent === 'number'
            ? Math.round(vehicle.linkQualityPercent)
            : undefined,
          coverage: typeof vehicle.coveragePercent === 'number'
            ? Math.round(vehicle.coveragePercent)
            : undefined,
          slamMode: vehicleSlamModes[normalizeVehicleId(vehicle.id)],
          lastUpdate: vehicle.lastUpdate,
          deliveryMode: vehicle.deliveryMode,
          isRetroactive: vehicle.isRetroactive,
        },
        missionMeta
      );
    });
  }, [telemetryVehicles, vehicleMissionMeta, vehicleSlamModes]);

  /**
   * Map of vehicle IDs to their ground-plane (X, Z) positions for camera teleport.
   * X-Z plane represents the local horizontal plane in ENU coordinates.
   */
  const vehiclePositionById = useMemo(() => {
    const entries: [string, [number, number]][] = telemetryVehicles.map((vehicle) => [
      vehicle.id,
      [vehicle.position.x, vehicle.position.z] as [number, number],
    ]);
    return new Map(entries);
  }, [telemetryVehicles]);

  /**
   * Derives warnings from fleet telemetry for display in FleetCard.
   * Critical battery threshold at 25% triggers immediate operator attention
   * per Aeris flight safety protocols.
   */
  const fleetWarnings = useMemo<VehicleWarning[]>(
    () =>
      fleetVehicles
        .filter((vehicle) => vehicle.battery !== null && vehicle.battery <= 50)
        .map((vehicle) => ({
          vehicleId: vehicle.id,
          message:
            (vehicle.battery ?? 0) <= 25 ? 'Battery critical' : 'Battery below 50%',
          severity: (vehicle.battery ?? 0) <= 25 ? 'critical' : 'warning',
        })),
    [fleetVehicles]
  );

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
    if (icViewModeEnabled) {
      return;
    }

    setPipVehicleId(id);
  }, [icViewModeEnabled]);
  const locateVehicleActionRef = useRef(handleLocateVehicle);
  const viewFeedActionRef = useRef(handleViewFeed);

  useEffect(() => {
    locateVehicleActionRef.current = handleLocateVehicle;
  }, [handleLocateVehicle]);

  useEffect(() => {
    viewFeedActionRef.current = handleViewFeed;
  }, [handleViewFeed]);
  const [mockAlertTimestamps] = useState<Record<(typeof MOCK_ALERT_IDS)[number], Date>>(() => ({
    'demo-critical': new Date(),
    'demo-warning': new Date(),
  }));
  const dismissedMockAlertIdsRef = useRef<Set<string>>(new Set());
  const mockAlertDisplayNonceRef = useRef(0);

  /**
   * Static alerts for demonstration. In production, these are fed from
   * the ROS /alerts topic via the centralized alert management system.
   */
  const storedAlerts = useMemo<Alert[]>(
    () => {
      if (!allowMockFallback) {
        return [];
      }
      return [
        {
          id: 'demo-critical',
          severity: 'critical',
          title: 'Scout-2 COMMS LOST',
          description: 'Last contact: 45 seconds ago - Initiating recovery',
          dismissible: false,
          timestamp: mockAlertTimestamps['demo-critical'],
          action: { label: 'LOCATE', onClick: () => locateVehicleActionRef.current('scout_2') },
        },
        {
          id: 'demo-warning',
          severity: 'warning',
          title: 'Ranger-1 low battery',
          description: '22% remaining - Auto RTH initiated',
          dismissible: true,
          timestamp: mockAlertTimestamps['demo-warning'],
          action: icViewModeEnabled
            ? undefined
            : { label: 'VIEW', onClick: () => viewFeedActionRef.current('ranger_1') },
        },
      ];
    },
    [allowMockFallback, icViewModeEnabled, mockAlertTimestamps]
  );
  const hasSyncedMockAlerts = useRef(false);
  const areAlertsOpenRef = useRef(false);
  const getVisibleStoredAlerts = useCallback(
    () => storedAlerts.filter((alert) => !dismissedMockAlertIdsRef.current.has(alert.id)),
    [storedAlerts]
  );
  const dismissStoredAlerts = useCallback(() => {
    mockAlertDisplayNonceRef.current += 1;
    MOCK_ALERT_IDS.forEach((alertId) => dismissAlert(alertId));
  }, []);
  const showVisibleStoredAlerts = useCallback((playSound: boolean) => {
    const visibleStoredAlerts = getVisibleStoredAlerts();
    mockAlertDisplayNonceRef.current += 1;
    const displayNonce = mockAlertDisplayNonceRef.current;

    visibleStoredAlerts.forEach((alert) =>
      showAlert(
        {
          ...alert,
          onDismiss: alert.dismissible
            ? () => {
                if (displayNonce !== mockAlertDisplayNonceRef.current) {
                  return;
                }
                dismissedMockAlertIdsRef.current.add(alert.id);
                areAlertsOpenRef.current = getVisibleStoredAlerts().length > 0;
              }
            : undefined,
        },
        { playSound }
      )
    );

    areAlertsOpenRef.current = visibleStoredAlerts.length > 0;
  }, [getVisibleStoredAlerts]);

  useEffect(() => {
    if (!allowMockFallback) {
      hasSyncedMockAlerts.current = false;
      areAlertsOpenRef.current = false;
      dismissedMockAlertIdsRef.current.clear();
      dismissStoredAlerts();
      return;
    }

    const shouldShowAlerts = areAlertsOpenRef.current || !hasSyncedMockAlerts.current;
    if (!shouldShowAlerts) {
      hasSyncedMockAlerts.current = true;
      return;
    }

    dismissStoredAlerts();
    showVisibleStoredAlerts(!hasSyncedMockAlerts.current);
    hasSyncedMockAlerts.current = true;
  }, [allowMockFallback, dismissStoredAlerts, showVisibleStoredAlerts]);

  const handleDroneSelect = (id: string) => {
    setSelectedDroneId(id || null);
    setSelectedDetectionId(null);
  };

  /**
   * Handles detection selection from the map or detection list.
   * Clears vehicle selection and teleports camera to the detection location
   * for operator review. Coordinates are in local ENU frame relative to
   * the mission origin set at mission start.
   */
  const handleDetectionSelect = useCallback((id: string) => {
    setSelectedDetectionId(id || null);
    setSelectedDroneId(null);

    if (id) {
      const detection = detections.find(d => d.id === id);
      if (detection && mapRef.current) {
        mapRef.current.teleportTo(detection.position[0], detection.position[2]);
      }
    }
  }, [detections]);

  const handleConfirmDetection = useCallback((id: string) => {
    if (icViewModeEnabled) {
      return;
    }

    setDetectionStatusOverrides((previous) => ({
      ...previous,
      [id]: 'confirmed',
    }));
  }, [icViewModeEnabled]);

  const handleDismissDetection = useCallback((id: string) => {
    if (icViewModeEnabled) {
      return;
    }

    setDetectionStatusOverrides((previous) => ({
      ...previous,
      [id]: 'dismissed',
    }));
  }, [icViewModeEnabled]);

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
    if (icViewModeEnabled) {
      return;
    }
    void id;
  }, [icViewModeEnabled]);

  const handleAlertClick = useCallback(() => {
    if (!allowMockFallback || storedAlerts.length === 0) {
      return;
    }
    areAlertsOpenRef.current = !areAlertsOpenRef.current;
    if (areAlertsOpenRef.current) {
      showVisibleStoredAlerts(false);
      return;
    }
    dismissStoredAlerts();
  }, [allowMockFallback, dismissStoredAlerts, showVisibleStoredAlerts, storedAlerts.length]);

  /**
   * Global keyboard shortcuts for mission control.
   * Space toggles pause/resume during active mission phases.
   * Number keys 1-6 provide quick access to fleet vehicles.
   * Escape clears all selections.
   * 'r' resets camera to origin (useful after extensive map navigation).
   */
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      // Ignore if user is typing in an input
      if (e.target instanceof HTMLInputElement || e.target instanceof HTMLTextAreaElement) return;

      switch (e.key) {
        case 'i':
        case 'I':
          e.preventDefault();
          setIcModeFromUi(!icViewModeEnabled);
          break;
        case ' ': // Space - Pause/Resume mission
          e.preventDefault();
          if (icViewModeEnabled) {
            break;
          }
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
          if (icViewModeEnabled) {
            break;
          }
          setSelectedDroneId(null);
          setSelectedDetectionId(null);
          break;
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
          if (icViewModeEnabled) {
            break;
          }
          // Select drone 1-6
          const droneIndex = parseInt(e.key) - 1;
          if (droneIndex < fleetVehicles.length) {
            handleLocateVehicle(fleetVehicles[droneIndex].id);
          }
          break;
        case 'r':
        case 'R':
          if (icViewModeEnabled) {
            break;
          }
          // Reset camera to default view
          if (mapRef.current) {
            mapRef.current.teleportTo(0, 0);
          }
          break;
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [missionPhase, isPaused, pauseMission, resumeMission, handleLocateVehicle, fleetVehicles, icViewModeEnabled, setIcModeFromUi]);

  const activeVehicles = fleetVehicles.filter(v => v.status === 'active' || v.status === 'warning');
  const batteryReadings = fleetVehicles
    .map(v => v.battery)
    .filter((battery): battery is number => typeof battery === 'number');
  const avgBattery = batteryReadings.length > 0
    ? Math.round(batteryReadings.reduce((sum, battery) => sum + battery, 0) / batteryReadings.length)
    : null;
  const avgAltitude = Math.round(
    activeVehicles.reduce((sum, v) => sum + v.altitude, 0) / (activeVehicles.length || 1)
  );

  const detectionCounts = useMemo(
    () => computeDetectionCounts(detections),
    [detections]
  );
  const statusAlertCount = detectionCounts.pending;
  const icTacticalSummary = (
    <div className="pointer-events-auto flex max-w-[360px] flex-col gap-3 text-lg">
      <div className="rounded-lg border border-white/25 bg-black/75 px-4 py-3 shadow-[0_0_30px_rgba(0,0,0,0.35)]">
        <div className="text-xl font-semibold uppercase tracking-wide text-white/90">IC VIEW</div>
        <div className="mt-2 grid grid-cols-2 gap-3 text-white">
          <div>
            <div className="font-mono text-3xl font-semibold">{activeVehicles.length}/{fleetVehicles.length}</div>
            <div className="text-xl text-white/90">fleet active</div>
          </div>
          <div>
            <div className="font-mono text-3xl font-semibold">{detectionCounts.pending}</div>
            <div className="text-xl text-white/90">pending alerts</div>
          </div>
          <div>
            <div className="font-mono text-3xl font-semibold">{routeRecommendations.length}</div>
            <div className="text-xl text-white/90">entry routes</div>
          </div>
          <div>
            <div className="font-mono text-3xl font-semibold">{structuralHazardZones.length}</div>
            <div className="text-xl text-white/90">hazard zones</div>
          </div>
        </div>
      </div>
    </div>
  );

  return (
    <GCSLayout
      viewMode={icViewModeEnabled ? 'ic' : 'operator'}
      map={
        <MapScene3D
          vehicles={telemetryVehicles}
          vehicleMissionMeta={vehicleMissionMeta}
          returnTrajectories={returnTrajectories}
          routeRecommendations={routeRecommendations}
          ref={mapRef}
          detections={detections}
          selectedDroneId={selectedDroneId}
          selectedDetectionId={selectedDetectionId}
          onDroneSelect={icViewModeEnabled ? undefined : handleDroneSelect}
          onDetectionSelect={icViewModeEnabled ? handleLocateDetection : handleDetectionSelect}
          zones={zones}
          selectedZoneId={selectedZoneId}
          onZoneSelect={icViewModeEnabled ? undefined : selectZone}
          isDrawingZone={icViewModeEnabled ? false : isDrawing}
          drawingPoints={drawing.points}
          drawingPriority={drawing.currentPriority}
          onAddZonePoint={icViewModeEnabled ? undefined : addPoint}
          routeStagingArea={routeStagingArea}
          isPlacingRouteStagingArea={icViewModeEnabled ? false : isPlacingRouteStagingArea}
          onRouteStagingAreaSet={icViewModeEnabled ? undefined : setRouteStagingAreaPosition}
        />
      }

      statusPill={
        <StatusPill
          missionPhase={missionPhase as MissionPhase}
          elapsedTime={elapsedSeconds}
          progressPercent={Math.round(coveragePercent)}
          connectionStatus={rosConnected ? "connected" : "disconnected"}
          detectionCounts={detectionCounts}
          alertCount={statusAlertCount}
          hasUnreadAlerts={statusAlertCount > 0}
          onAlertClick={handleAlertClick}
          viewMode={icViewModeEnabled ? 'ic' : 'operator'}
        />
      }

      layersPanel={
        icViewModeEnabled ? (
          <div className="space-y-3">
            {icTacticalSummary}
            <div className="rounded-lg border border-white/20 bg-black/70 px-4 py-3 text-xl text-white/95">
              Read-only shared tactical picture
            </div>
          </div>
        ) : (
          <LayersPanel />
        )
      }

      topRightOverlay={icViewModeEnabled ? undefined : (
        <EmergencyStopControl
          key={missionPhase}
          missionPhase={missionPhase}
          canAbort={canAbort}
          abortUnavailableReason={abortUnavailableReason}
          abortError={abortMissionError}
          onAbort={abortMission}
        />
      )}

      zoneToolbar={icViewModeEnabled ? undefined : <ZoneToolbar />}

      commandDock={
        <CommandDock
          mode={icViewModeEnabled ? 'ic' : 'operator'}
          fleetCard={
            <FleetSheet
              vehicles={fleetVehicles}
              selectedVehicleId={selectedDroneId}
              onLocate={handleLocateVehicle}
              onViewFeed={icViewModeEnabled ? undefined : handleViewFeed}
              onRTH={icViewModeEnabled ? undefined : handleRTH}
              viewMode={icViewModeEnabled ? 'ic' : 'operator'}
              trigger={
                <FleetCard
                  vehicles={fleetVehicles}
                  activeCount={activeVehicles.length}
                  totalCount={fleetVehicles.length}
                  avgBattery={avgBattery}
                  avgAltitude={avgAltitude}
                  warnings={fleetWarnings}
                  viewMode={icViewModeEnabled ? 'ic' : 'operator'}
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
              readOnly={icViewModeEnabled}
              trigger={
                <DetectionsCard
                  thermalCount={detectionCounts.thermal}
                  acousticCount={detectionCounts.acoustic}
                  gasCount={detectionCounts.gas}
                  pendingCount={detectionCounts.pending}
                  confirmedCount={detectionCounts.confirmed}
                  viewMode={icViewModeEnabled ? 'ic' : 'operator'}
                />
              }
            />
          }
          controlsCard={icViewModeEnabled ? undefined : (
            <ControlsCard
              missionPhase={missionPhase}
              isPaused={isPaused}
              canStart={canStart}
              canPause={canPause}
              hasValidStartZone={hasValidStartZone}
              selectedPattern={selectedPattern}
              setSelectedPattern={setSelectedPattern}
              startMissionError={startMissionError}
              onStart={startMission}
              onPause={pauseMission}
              onResume={resumeMission}
            />
          )}
        />
      }

      pipFeed={
        pipVehicleId ? (() => {
          const vehicle = fleetVehicles.find(v => v.id === pipVehicleId);
          if (!vehicle) return null;

          const isLive =
            (vehicle.status === 'active' || vehicle.status === 'warning' || vehicle.status === 'returning') &&
            vehicle.deliveryMode !== 'replayed' &&
            vehicle.isRetroactive !== true &&
            vehicle.isLastKnown !== true;

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
      statusToggle={
        <label className="pointer-events-auto absolute right-4 top-4 flex h-12 items-center gap-3 rounded-full border border-white/20 bg-black/70 px-4 text-base font-semibold text-white shadow-[0_0_28px_rgba(0,0,0,0.35)]">
          <span>IC VIEW</span>
          <Switch
            checked={icViewModeEnabled}
            onCheckedChange={setIcModeFromUi}
            aria-label="Toggle IC view"
          />
        </label>
      }
    />
  );
}
