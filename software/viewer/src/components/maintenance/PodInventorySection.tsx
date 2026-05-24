'use client';

import { useMemo, useState } from 'react';
import { ClipboardCheck, PlugZap, Radio } from 'lucide-react';
import { Badge } from '@/components/ui/badge';
import { Button } from '@/components/ui/button';
import { Card } from '@/components/ui/card';
import type { MaintenancePodInventorySnapshot } from '@/lib/maintenanceDiagnostics';
import {
  formatInventoryTimestamp,
  getCalibrationBadgeVariant,
} from '@/lib/ros/podInventory';
import { cn } from '@/lib/utils';

export interface PodInventoryActionState {
  kind: 'success' | 'error';
  message: string;
}

interface PodInventorySectionProps {
  inventory: MaintenancePodInventorySnapshot[];
  submittingPodSerial?: string | null;
  actionStateBySerial?: Record<string, PodInventoryActionState | null>;
  onSubmitCalibration?: (request: {
    podSerial: string;
    lastCalibrationAtMs: number;
    nextCalibrationDueAtMs: number;
  }) => void | Promise<unknown>;
}

interface CalibrationDraft {
  lastCalibration: string;
  nextCalibrationDue: string;
}

function toDateInput(value?: number | null): string {
  if (value == null) {
    return '';
  }
  return new Date(value).toISOString().slice(0, 10);
}

function buildInitialDraft(row: MaintenancePodInventorySnapshot): CalibrationDraft {
  const today = new Date().toISOString().slice(0, 10);
  return {
    lastCalibration: toDateInput(row.lastCalibrationAtMs) || today,
    nextCalibrationDue:
      toDateInput(row.nextCalibrationDueAtMs) ||
      new Date(Date.now() + 180 * 24 * 60 * 60 * 1000).toISOString().slice(0, 10),
  };
}

export function mergeCalibrationDraft(
  row: MaintenancePodInventorySnapshot,
  currentDraft: CalibrationDraft | undefined,
  field: keyof CalibrationDraft,
  value: string
): CalibrationDraft {
  return {
    ...(currentDraft ?? buildInitialDraft(row)),
    [field]: value,
  };
}

function parseDateInput(value: string): number | null {
  if (!value) {
    return null;
  }
  const parsed = Date.parse(`${value}T00:00:00Z`);
  return Number.isFinite(parsed) ? parsed : null;
}

export function PodInventorySection({
  inventory,
  submittingPodSerial = null,
  actionStateBySerial = {},
  onSubmitCalibration = async () => {},
}: PodInventorySectionProps) {
  const [draftsBySerial, setDraftsBySerial] = useState<Record<string, CalibrationDraft>>({});

  const summary = useMemo(() => {
    const attached = inventory.filter((row) => row.attached).length;
    const dueSoon = inventory.filter((row) => row.calibrationState === 'due_soon').length;
    const overdue = inventory.filter((row) => row.calibrationState === 'overdue').length;
    return {
      known: inventory.length,
      attached,
      dueSoon,
      overdue,
    };
  }, [inventory]);

  const getDraft = (row: MaintenancePodInventorySnapshot): CalibrationDraft =>
    draftsBySerial[row.podSerial] ?? buildInitialDraft(row);

  const handleDraftChange = (
    row: MaintenancePodInventorySnapshot,
    podSerial: string,
    field: keyof CalibrationDraft,
    value: string
  ) => {
    setDraftsBySerial((current) => ({
      ...current,
      [podSerial]: mergeCalibrationDraft(row, current[podSerial], field, value),
    }));
  };

  const handleSubmit = async (row: MaintenancePodInventorySnapshot) => {
    const draft = getDraft(row);
    const lastCalibrationAtMs = parseDateInput(draft.lastCalibration);
    const nextCalibrationDueAtMs = parseDateInput(draft.nextCalibrationDue);
    if (
      lastCalibrationAtMs == null ||
      nextCalibrationDueAtMs == null ||
      nextCalibrationDueAtMs <= lastCalibrationAtMs
    ) {
      return;
    }
    await onSubmitCalibration({
      podSerial: row.podSerial,
      lastCalibrationAtMs,
      nextCalibrationDueAtMs,
    });
  };

  return (
    <section className="space-y-4">
      <div className="flex flex-wrap items-end justify-between gap-3">
        <div>
          <div className="text-sm font-medium text-white">Pod inventory</div>
          <div className="mt-1 text-sm text-white/55">
            Known fleet pods, attachment state, and calibration readiness.
          </div>
        </div>
        <div className="flex flex-wrap gap-3 text-sm text-white/60">
          <span>{summary.known} known</span>
          <span>{summary.attached} attached</span>
          <span>{summary.dueSoon} due soon</span>
          <span>{summary.overdue} overdue</span>
        </div>
      </div>

      <Card className="border-white/10 bg-white/[0.04] p-4">
        {inventory.length === 0 ? (
          <div className="py-8 text-center text-sm text-white/50">
            Pod inventory is waiting for Device Manager data.
          </div>
        ) : (
          <div className="space-y-3">
            {inventory.map((row) => {
              const actionState = actionStateBySerial[row.podSerial] ?? null;
              const draft = getDraft(row);
              const submitDisabled =
                !row.attached ||
                submittingPodSerial === row.podSerial ||
                !parseDateInput(draft.lastCalibration) ||
                !parseDateInput(draft.nextCalibrationDue) ||
                parseDateInput(draft.nextCalibrationDue)! <= parseDateInput(draft.lastCalibration)!;
              return (
                <div
                  key={row.podSerial}
                  className={cn(
                    'rounded-lg border border-white/8 bg-black/18 p-4',
                    row.calibrationState === 'overdue' && 'border-red-500/20',
                    row.calibrationState === 'due_soon' && 'border-amber-400/20'
                  )}
                >
                  <div className="grid gap-4 lg:grid-cols-[minmax(0,1.5fr)_minmax(0,1fr)_minmax(0,1fr)]">
                    <div className="min-w-0">
                      <div className="flex items-center gap-2">
                        <Radio className="h-4 w-4 text-white/55" />
                        <span className="truncate text-sm font-medium text-white">
                          {row.podType} · {row.podSerial}
                        </span>
                        <Badge variant={getCalibrationBadgeVariant(row.calibrationState)}>
                          {row.calibrationState.replace('_', ' ')}
                        </Badge>
                      </div>
                      <div className="mt-2 space-y-1 text-sm text-white/55">
                        <div>
                          {row.attached
                            ? `${row.vehicleId ?? 'Attached'} · ${row.slotId ?? 'slot unknown'}`
                            : 'Detached'}
                        </div>
                        <div>{row.calibrationDetail ?? 'Calibration schedule unavailable'}</div>
                        <div>
                          Last: {formatInventoryTimestamp(row.lastCalibrationAtMs)} · Next:{' '}
                          {formatInventoryTimestamp(row.nextCalibrationDueAtMs)}
                        </div>
                      </div>
                    </div>

                    <div className="grid gap-2 sm:grid-cols-2 lg:grid-cols-1">
                      <label className="text-sm text-white/60">
                        Completed
                        <input
                          type="date"
                          value={draft.lastCalibration}
                          disabled={!row.attached}
                          onChange={(event) =>
                            handleDraftChange(row, row.podSerial, 'lastCalibration', event.target.value)
                          }
                          className="mt-1 h-10 w-full rounded-md border border-white/10 bg-black/18 px-3 text-white outline-none"
                        />
                      </label>
                      <label className="text-sm text-white/60">
                        Next due
                        <input
                          type="date"
                          value={draft.nextCalibrationDue}
                          disabled={!row.attached}
                          onChange={(event) =>
                            handleDraftChange(row, row.podSerial, 'nextCalibrationDue', event.target.value)
                          }
                          className="mt-1 h-10 w-full rounded-md border border-white/10 bg-black/18 px-3 text-white outline-none"
                        />
                      </label>
                    </div>

                    <div className="flex flex-col justify-between gap-3">
                      <Button
                        type="button"
                        onClick={() => void handleSubmit(row)}
                        disabled={submitDisabled}
                        className="h-10 justify-center"
                      >
                        <ClipboardCheck className="mr-2 h-4 w-4" />
                        {submittingPodSerial === row.podSerial ? 'Logging calibration' : 'Log calibration'}
                      </Button>
                      <div className="text-sm">
                        {!row.attached ? (
                          <div className="flex items-center gap-2 text-white/50">
                            <PlugZap className="h-4 w-4" />
                            <span>Connect pod to log calibration</span>
                          </div>
                        ) : actionState ? (
                          <div
                            className={
                              actionState.kind === 'error' ? 'text-red-200' : 'text-emerald-200'
                            }
                          >
                            {actionState.message}
                          </div>
                        ) : (
                          <div className="text-white/45">
                            Edge verification writes both dates back to pod EEPROM.
                          </div>
                        )}
                      </div>
                    </div>
                  </div>
                </div>
              );
            })}
          </div>
        )}
      </Card>
    </section>
  );
}
