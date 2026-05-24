'use client';

import type { ReactNode } from 'react';
import { useMemo, useState } from 'react';
import { Activity, Battery, Radio, ShieldCheck, TriangleAlert } from 'lucide-react';
import { Card } from '@/components/ui/card';
import { Badge } from '@/components/ui/badge';
import { ViewerRouteNav } from '@/components/layout/ViewerRouteNav';
import { MaintenanceVehicleCard } from '@/components/maintenance/MaintenanceVehicleCard';
import { useFirmwareUpdateAction } from '@/hooks/useFirmwareUpdateAction';
import { useFirmwareUpdateStatus } from '@/hooks/useFirmwareUpdateStatus';
import { useVehicleTelemetry } from '@/hooks/useVehicleTelemetry';
import { useMissionControl } from '@/hooks/useMissionControl';
import { usePodStatus } from '@/hooks/usePodStatus';
import { useVehicleMaintenanceDiagnostics } from '@/hooks/useVehicleMaintenanceDiagnostics';
import {
  createDemoFleetDiagnostics,
  projectFleetDiagnostics,
} from '@/lib/maintenanceDiagnostics';
import { normalizeMissionMetaForVehicle } from '@/lib/degradedVehicleState';
import { createDemoFirmwareUpdateStatusArray } from '@/lib/ros/firmwareUpdateStatus';

function SummaryCard({
  label,
  value,
  detail,
  icon,
  variant = 'outline',
}: {
  label: string;
  value: string;
  detail: string;
  icon: ReactNode;
  variant?: 'outline' | 'success' | 'warning' | 'danger';
}) {
  return (
    <Card className="border-white/10 bg-white/[0.04] p-4">
      <div className="flex items-center justify-between gap-3">
        <div className="flex h-10 w-10 items-center justify-center rounded-full border border-white/10 bg-white/[0.04] text-white/80">
          {icon}
        </div>
        <Badge variant={variant}>{label}</Badge>
      </div>
      <div className="mt-4 text-3xl font-semibold text-white">{value}</div>
      <div className="mt-1 text-sm text-white/55">{detail}</div>
    </Card>
  );
}

export function MaintenanceDashboardPage() {
  const { vehicles } = useVehicleTelemetry();
  const { vehicleMissionMeta, rosConnected } = useMissionControl();
  const podStatus = usePodStatus();
  const diagnostics = useVehicleMaintenanceDiagnostics();
  const firmwareUpdates = useFirmwareUpdateStatus();
  const { requestUpdate, submittingVehicleId, errorsByVehicle } = useFirmwareUpdateAction();
  const [expandedVehicleIds, setExpandedVehicleIds] = useState<string[]>([]);

  const liveDashboard = useMemo(
    () =>
      projectFleetDiagnostics({
        telemetry: vehicles.map((vehicle) => {
          const missionMeta = normalizeMissionMetaForVehicle(vehicle.id, vehicleMissionMeta);
          return {
            id: vehicle.id,
            name: vehicle.id.replace(/[_-]/g, ' ').toUpperCase(),
            batteryPercent:
              typeof vehicle.batteryPercent === 'number' ? vehicle.batteryPercent : null,
            altitudeMeters: vehicle.position.y,
            linkQualityPercent: vehicle.linkQualityPercent,
            lastUpdate: vehicle.lastUpdate,
            deliveryMode: vehicle.deliveryMode,
            isRetroactive: vehicle.isRetroactive,
            missionOnline: missionMeta.online,
          };
        }),
        diagnostics: diagnostics.vehicles,
        pods: podStatus.pods,
      }),
    [diagnostics.vehicles, podStatus.pods, vehicleMissionMeta, vehicles]
  );

  const isDemoMode = !rosConnected && liveDashboard.vehicles.length === 0;
  const dashboard = isDemoMode ? createDemoFleetDiagnostics() : liveDashboard;
  const firmwareState = isDemoMode && firmwareUpdates.updates.length === 0
    ? createDemoFirmwareUpdateStatusArray()
    : firmwareUpdates;
  const firmwareByVehicleId = useMemo(
    () => new Map(firmwareState.updates.map((update) => [update.vehicleId, update])),
    [firmwareState.updates]
  );

  const toggleVehicle = (vehicleId: string) => {
    setExpandedVehicleIds((current) =>
      current.includes(vehicleId)
        ? current.filter((id) => id !== vehicleId)
        : [...current, vehicleId]
    );
  };

  return (
    <div className="min-h-dvh bg-[linear-gradient(180deg,#05070b_0%,#0a1017_52%,#070b11_100%)] text-white">
      <div className="mx-auto flex w-full max-w-7xl flex-col gap-6 px-4 py-5 md:px-6 lg:px-8">
        <header className="flex flex-col gap-4 border-b border-white/8 pb-5 lg:flex-row lg:items-end lg:justify-between">
          <div>
            <div className="text-xs font-semibold uppercase tracking-[0.22em] text-cyan-200/70">
              Fleet readiness
            </div>
            <h1 className="mt-2 text-3xl font-semibold tracking-tight text-white">Maintenance</h1>
            <p className="mt-2 max-w-2xl text-sm text-white/60">
              Fleet diagnostics with edge-managed firmware updates, rollback visibility, and calibration follow-up.
            </p>
          </div>
          <div className="flex flex-col items-start gap-3 lg:items-end">
            <ViewerRouteNav currentRoute="maintenance" />
            <Badge variant={isDemoMode ? 'warning' : rosConnected ? 'success' : 'outline'}>
              {isDemoMode ? 'Demo data' : rosConnected ? 'ROS connected' : 'Waiting for ROS'}
            </Badge>
          </div>
        </header>

        <section className="grid gap-4 md:grid-cols-2 xl:grid-cols-4">
          <SummaryCard
            label="Ready"
            value={String(dashboard.summary.readyCount)}
            detail="Vehicles clear for dispatch"
            icon={<ShieldCheck className="h-5 w-5" />}
            variant="success"
          />
          <SummaryCard
            label="Attention"
            value={String(dashboard.summary.warningCount)}
            detail="Vehicles with review items"
            icon={<TriangleAlert className="h-5 w-5" />}
            variant="warning"
          />
          <SummaryCard
            label="Blocked"
            value={String(dashboard.summary.blockedCount)}
            detail="Vehicles held from readiness"
            icon={<Activity className="h-5 w-5" />}
            variant="danger"
          />
          <SummaryCard
            label="Mesh avg"
            value={
              dashboard.summary.averageLinkQuality == null
                ? '--'
                : `${dashboard.summary.averageLinkQuality}%`
            }
            detail={`${dashboard.summary.connectedCount} vehicles with current telemetry`}
            icon={<Radio className="h-5 w-5" />}
          />
        </section>

        <section className="grid gap-4 xl:grid-cols-2">
          {dashboard.vehicles.length === 0 ? (
            <Card className="border-white/10 bg-white/[0.04] p-8">
              <div className="flex flex-col items-center justify-center gap-3 text-center text-white/55">
                <Battery className="h-8 w-8 text-white/35" />
                <div className="text-lg font-medium text-white">No fleet diagnostics yet</div>
                <div className="max-w-md text-sm">
                  Connect ROS telemetry and maintenance diagnostics to populate the readiness board.
                </div>
              </div>
            </Card>
          ) : (
            dashboard.vehicles.map((vehicle) => (
              <MaintenanceVehicleCard
                key={vehicle.id}
                vehicle={vehicle}
                expanded={expandedVehicleIds.includes(vehicle.id)}
                onToggle={() => toggleVehicle(vehicle.id)}
                firmwareStatus={firmwareByVehicleId.get(vehicle.id) ?? null}
                firmwareActionError={errorsByVehicle[vehicle.id] ?? null}
                isSubmittingFirmwareUpdate={submittingVehicleId === vehicle.id}
                onRequestFirmwareUpdate={requestUpdate}
              />
            ))
          )}
        </section>
      </div>
    </div>
  );
}
