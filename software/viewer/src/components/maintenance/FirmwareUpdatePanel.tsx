'use client';

import type { FormEvent } from 'react';
import { useMemo, useState } from 'react';
import {
  ArrowUpCircle,
  HardDriveDownload,
  RotateCcw,
  ShieldCheck,
  ShieldX,
} from 'lucide-react';
import { Badge } from '@/components/ui/badge';
import { Button } from '@/components/ui/button';
import { Progress } from '@/components/ui/progress';
import { cn } from '@/lib/utils';
import {
  getFirmwareUpdateBadgeVariant,
  hasFreshFirmwareUpdateStatus,
  isFirmwareUpdateActive,
  type FirmwareUpdateCommandInput,
  type FirmwareUpdateStatusSnapshot,
} from '@/lib/ros/firmwareUpdateStatus';

interface FirmwareUpdatePanelProps {
  vehicleId?: string;
  vehicleName: string;
  status?: FirmwareUpdateStatusSnapshot | null;
  isSubmitting: boolean;
  actionError: string | null;
  onSubmit: (request: FirmwareUpdateCommandInput) => void | Promise<unknown>;
}

function normalizePackageSeed(vehicleName: string): string {
  return vehicleName.trim().toLowerCase().replace(/[^a-z0-9]+/g, '-');
}

export function FirmwareUpdatePanel({
  vehicleId,
  vehicleName,
  status,
  isSubmitting,
  actionError,
  onSubmit,
}: FirmwareUpdatePanelProps) {
  const seed = useMemo(() => normalizePackageSeed(vehicleName), [vehicleName]);
  const [packageId, setPackageId] = useState(status?.packageId ?? `${seed}-fw`);
  const [targetVersion, setTargetVersion] = useState(
    status?.targetVersion ?? '2026.05.23'
  );
  const [packageUri, setPackageUri] = useState(
    `s3://updates/${seed}/${targetVersion}.bin`
  );
  const [packageSignature, setPackageSignature] = useState('');

  const active = isFirmwareUpdateActive(status);
  const disabled = active || isSubmitting;
  const progressValue = status?.progressPercent ?? 0;
  const currentVersion = status?.currentVersion ?? '--';
  const desiredVersion = status?.targetVersion ?? targetVersion;
  const activeSlot = status?.activeSlot ?? '--';
  const inactiveSlot = status?.inactiveSlot ?? '--';
  const statusVariant = getFirmwareUpdateBadgeVariant(status?.lifecycleState ?? 'idle');
  const statusLabel = status?.lifecycleLabel ?? 'Idle';
  const displayActionError =
    actionError && hasFreshFirmwareUpdateStatus(status) ? null : actionError;
  const detail =
    displayActionError ??
    status?.statusDetail ??
    status?.errorDetail ??
    'No edge firmware status reported yet.';
  const secondaryDetail =
    !displayActionError &&
    status?.statusDetail &&
    status?.errorDetail &&
    status.statusDetail !== status.errorDetail
      ? status.errorDetail
      : null;
  const slotMessage = `Active ${activeSlot} -> Standby ${inactiveSlot}`;

  const submitRequest = (event: FormEvent<HTMLFormElement>) => {
    event.preventDefault();
    void onSubmit({
      vehicleId: vehicleId ?? status?.vehicleId ?? seed,
      packageId,
      targetVersion,
      packageUri,
      packageSignature,
    });
  };

  return (
    <div className="mt-4 border-t border-white/8 pt-4">
      <div className="flex flex-wrap items-start justify-between gap-3">
        <div>
          <div className="flex items-center gap-2 text-xs font-semibold uppercase tracking-[0.18em] text-white/45">
            <HardDriveDownload className="h-4 w-4" />
            Firmware update
          </div>
          <div className="mt-2 flex flex-wrap items-center gap-2">
            <Badge variant={statusVariant}>{statusLabel}</Badge>
            {status?.rollbackPerformed ? (
              <Badge variant="warning">Rollback used</Badge>
            ) : null}
          </div>
        </div>
        <div className="text-right text-sm text-white/55">
          <div>{slotMessage}</div>
          <div className="mt-1">{progressValue.toFixed(1)}%</div>
        </div>
      </div>

      <div className="mt-3 grid gap-3 sm:grid-cols-2">
        <div className="rounded-lg border border-white/8 bg-white/[0.03] px-3 py-3">
          <div className="text-xs uppercase tracking-[0.16em] text-white/45">Current firmware</div>
          <div className="mt-2 flex items-center gap-2 text-sm text-white">
            <ShieldCheck className="h-4 w-4 text-emerald-300" />
            <span>{currentVersion}</span>
          </div>
        </div>
        <div className="rounded-lg border border-white/8 bg-white/[0.03] px-3 py-3">
          <div className="text-xs uppercase tracking-[0.16em] text-white/45">Target firmware</div>
          <div className="mt-2 flex items-center gap-2 text-sm text-white">
            <ArrowUpCircle className="h-4 w-4 text-cyan-300" />
            <span>{desiredVersion}</span>
          </div>
        </div>
      </div>

      <div className="mt-3">
        <Progress
          value={progressValue}
          indicatorColor={
            status?.lifecycleState === 'failed'
              ? 'danger'
              : status?.lifecycleState === 'rolled_back'
                ? 'warning'
                : status?.lifecycleState === 'complete'
                  ? 'success'
                  : 'default'
          }
        />
      </div>

      <div
        className={cn(
          'mt-3 rounded-lg border px-3 py-3 text-sm',
          displayActionError
            ? 'border-red-500/25 bg-red-500/10 text-red-100'
            : status?.lifecycleState === 'rolled_back' || status?.lifecycleState === 'failed'
              ? 'border-amber-400/25 bg-amber-500/10 text-amber-50'
              : 'border-white/8 bg-white/[0.03] text-white/70'
        )}
      >
        <div className="flex items-start gap-2">
          {displayActionError || status?.lifecycleState === 'failed' ? (
            <ShieldX className="mt-0.5 h-4 w-4 shrink-0" />
          ) : status?.lifecycleState === 'rolled_back' ? (
            <RotateCcw className="mt-0.5 h-4 w-4 shrink-0" />
          ) : (
            <HardDriveDownload className="mt-0.5 h-4 w-4 shrink-0" />
          )}
          <div className="space-y-1">
            <div>{detail}</div>
            {secondaryDetail ? <div className="text-xs text-white/75">{secondaryDetail}</div> : null}
          </div>
        </div>
      </div>

      <form className="mt-4 grid gap-3 md:grid-cols-2" onSubmit={submitRequest}>
        <label className="grid gap-1.5 text-sm text-white/70">
          <span className="text-xs uppercase tracking-[0.16em] text-white/45">Package ID</span>
          <input
            className="h-10 rounded-md border border-white/10 bg-black/20 px-3 text-white outline-none ring-0 placeholder:text-white/30"
            value={packageId}
            onChange={(event) => setPackageId(event.target.value)}
            placeholder="fw-2026.05.23"
            disabled={disabled}
          />
        </label>
        <label className="grid gap-1.5 text-sm text-white/70">
          <span className="text-xs uppercase tracking-[0.16em] text-white/45">Target version</span>
          <input
            className="h-10 rounded-md border border-white/10 bg-black/20 px-3 text-white outline-none ring-0 placeholder:text-white/30"
            value={targetVersion}
            onChange={(event) => setTargetVersion(event.target.value)}
            placeholder="2026.05.23"
            disabled={disabled}
          />
        </label>
        <label className="grid gap-1.5 text-sm text-white/70 md:col-span-2">
          <span className="text-xs uppercase tracking-[0.16em] text-white/45">Package URI</span>
          <input
            className="h-10 rounded-md border border-white/10 bg-black/20 px-3 text-white outline-none ring-0 placeholder:text-white/30"
            value={packageUri}
            onChange={(event) => setPackageUri(event.target.value)}
            placeholder="s3://updates/scout-2/fw-2026.05.23.bin"
            disabled={disabled}
          />
        </label>
        <label className="grid gap-1.5 text-sm text-white/70 md:col-span-2">
          <span className="text-xs uppercase tracking-[0.16em] text-white/45">Signature token</span>
          <input
            className="h-10 rounded-md border border-white/10 bg-black/20 px-3 text-white outline-none ring-0 placeholder:text-white/30"
            value={packageSignature}
            onChange={(event) => setPackageSignature(event.target.value)}
            placeholder="signed-manifest"
            disabled={disabled}
          />
        </label>
        <div className="md:col-span-2">
          <Button
            type="submit"
            variant="ghost"
            className="h-10 border border-white/10 bg-white/[0.04] px-4 text-white hover:bg-white/10"
            disabled={disabled}
          >
            {isSubmitting ? 'Submitting update...' : active ? 'Update in progress' : 'Start firmware update'}
          </Button>
        </div>
      </form>
    </div>
  );
}
