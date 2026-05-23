'use client';

import type { ReactNode } from 'react';
import { Activity, Battery, ChevronDown, ChevronUp, Compass, Cpu, Gauge, Radio, ShieldAlert } from 'lucide-react';
import { Badge } from '@/components/ui/badge';
import { Button } from '@/components/ui/button';
import { Card } from '@/components/ui/card';
import { cn } from '@/lib/utils';
import {
  getDiagnosticBadgeVariant,
  getReadinessBadgeVariant,
  type MaintenanceCheckViewModel,
  type MaintenanceVehicleViewModel,
} from '@/lib/maintenanceDiagnostics';
import { formatLastContactAge } from '@/lib/degradedVehicleState';

interface MaintenanceVehicleCardProps {
  vehicle: MaintenanceVehicleViewModel;
  expanded: boolean;
  onToggle: () => void;
}

function DetailRow({
  check,
  icon,
  testId,
}: {
  check: MaintenanceCheckViewModel;
  icon: ReactNode;
  testId?: string;
}) {
  return (
    <div
      className="grid gap-2 border-t border-white/8 py-3 first:border-t-0 first:pt-0 md:grid-cols-[auto_1fr_auto]"
      data-testid={testId}
    >
      <div className="flex h-9 w-9 items-center justify-center rounded-full border border-white/10 bg-white/[0.04] text-white/70">
        {icon}
      </div>
      <div className="min-w-0">
        <div className="text-sm font-medium text-white">{check.label}</div>
        <div className="text-sm text-white/60">{check.summary}</div>
      </div>
      <Badge variant={getDiagnosticBadgeVariant(check.state)} className="justify-self-start md:justify-self-end">
        {check.state}
      </Badge>
    </div>
  );
}

export function MaintenanceVehicleCard({
  vehicle,
  expanded,
  onToggle,
}: MaintenanceVehicleCardProps) {
  const readinessVariant = getReadinessBadgeVariant(vehicle.readiness);
  const batteryLabel =
    typeof vehicle.batteryPercent === 'number' ? `${Math.round(vehicle.batteryPercent)}%` : '--';
  const lastContactLabel = vehicle.isOffline
    ? `Last contact ${formatLastContactAge(vehicle.lastContactAgeMs)} ago`
    : 'Live telemetry';
  const [imuCheck, compassCheck, accelerometerCheck] = vehicle.detailChecks;

  return (
    <Card
      className={cn(
        'border-white/10 bg-white/[0.04] p-5 shadow-[0_18px_40px_rgba(0,0,0,0.22)]',
        vehicle.readiness === 'blocked' && 'border-red-500/20',
        vehicle.readiness === 'warning' && 'border-amber-400/20'
      )}
    >
      <div className="flex flex-wrap items-start justify-between gap-3">
        <div className="min-w-0">
          <div className="flex items-center gap-2">
            <h2 className="truncate text-lg font-medium text-white">{vehicle.name}</h2>
            <Badge variant={readinessVariant}>{vehicle.readiness}</Badge>
          </div>
          <p className="mt-1 text-sm text-white/60">{vehicle.readinessSummary}</p>
        </div>
        <div className="flex flex-wrap items-center gap-3 text-sm text-white/65">
          <div className="flex items-center gap-1.5">
            <Battery className="h-4 w-4" />
            <span className="font-mono">{batteryLabel}</span>
          </div>
          <div className="flex items-center gap-1.5">
            <Gauge className="h-4 w-4" />
            <span className="font-mono">{Math.round(vehicle.altitudeMeters)}m</span>
          </div>
        </div>
      </div>

      <div className="mt-4 grid gap-3 sm:grid-cols-2">
        {[
          vehicle.summaryChecks.motor,
          vehicle.summaryChecks.sensors,
          vehicle.summaryChecks.mesh,
          vehicle.summaryChecks.calibration,
        ].map((check) => (
          <div key={check.label} className="rounded-lg border border-white/8 bg-black/18 px-3 py-3">
            <div className="flex items-center justify-between gap-2">
              <div className="text-sm font-medium text-white">{check.label}</div>
              <Badge variant={getDiagnosticBadgeVariant(check.state)}>{check.state}</Badge>
            </div>
            <div className="mt-2 text-sm text-white/60">{check.summary}</div>
          </div>
        ))}
      </div>

      <div className="mt-4 flex flex-wrap items-center justify-between gap-3 border-t border-white/8 pt-4">
        <div className="flex items-center gap-2 text-sm text-white/55">
          <Activity className="h-4 w-4" />
          <span>{lastContactLabel}</span>
        </div>
        <Button
          type="button"
          variant="ghost"
          size="sm"
          className="h-9 rounded-full border border-white/10 bg-white/[0.04] px-3 text-white hover:bg-white/10"
          onClick={onToggle}
          aria-expanded={expanded}
        >
          {expanded ? <ChevronUp className="mr-1.5 h-4 w-4" /> : <ChevronDown className="mr-1.5 h-4 w-4" />}
          {expanded ? 'Hide details' : 'Show details'}
        </Button>
      </div>

      {expanded && (
        <div className="mt-4 rounded-xl border border-white/8 bg-black/18 p-4">
          <div className="mb-3 text-xs font-semibold uppercase tracking-[0.18em] text-white/45">
            Calibration detail
          </div>
          <div className="space-y-1">
            {imuCheck ? (
              <DetailRow check={imuCheck} icon={<Cpu className="h-4 w-4" />} testId="maintenance-detail-imu" />
            ) : null}
            {compassCheck ? (
              <DetailRow check={compassCheck} icon={<Compass className="h-4 w-4" />} testId="maintenance-detail-compass" />
            ) : null}
            {accelerometerCheck ? (
              <DetailRow check={accelerometerCheck} icon={<Gauge className="h-4 w-4" />} testId="maintenance-detail-accelerometer" />
            ) : null}
          </div>

          <div className="mt-4 border-t border-white/8 pt-4">
            <div className="mb-3 flex items-center gap-2 text-xs font-semibold uppercase tracking-[0.18em] text-white/45">
              <ShieldAlert className="h-4 w-4" />
              Attached pods
            </div>
            {vehicle.pods.length === 0 ? (
              <div className="text-sm text-white/45">No pod status reported.</div>
            ) : (
              <div className="grid gap-2">
                {vehicle.pods.map((pod) => (
                  <div
                    key={`${vehicle.id}-${pod.slotId}-${pod.podSerial}`}
                    className="flex flex-col gap-2 rounded-lg border border-white/8 bg-white/[0.03] px-3 py-3 md:flex-row md:items-center md:justify-between"
                  >
                    <div className="min-w-0">
                      <div className="flex items-center gap-2">
                        <Radio className="h-4 w-4 text-white/55" />
                        <span className="truncate text-sm font-medium text-white">
                          {pod.podType} · {pod.podSerial}
                        </span>
                      </div>
                      <div className="mt-1 text-sm text-white/55">
                        {pod.slotId} · {pod.capabilities.join(', ') || 'No capabilities reported'}
                      </div>
                    </div>
                    <Badge variant={getDiagnosticBadgeVariant(pod.state)}>{pod.lifecycleLabel}</Badge>
                  </div>
                ))}
              </div>
            )}
          </div>
        </div>
      )}
    </Card>
  );
}
