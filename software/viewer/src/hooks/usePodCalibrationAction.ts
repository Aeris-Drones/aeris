import { useCallback, useEffect, useMemo, useState } from 'react';
import ROSLIB from 'roslib';
import { useSharedROSConnection } from '@/context/ROSConnectionContext';
import { withServiceTimeout } from '@/lib/missionControlBehavior.js';
import type { MaintenancePodInventorySnapshot } from '@/lib/maintenanceDiagnostics';
import { parseLogPodCalibrationResponse } from '@/lib/ros/podInventory';

export const POD_CALIBRATION_SERVICE_TIMEOUT_MS = 15_000;

export interface PodCalibrationActionState {
  kind: 'success' | 'error';
  message: string;
}

export function reconcilePodCalibrationActionState(args: {
  actionStatesBySerial: Record<string, PodCalibrationActionState | null>;
  inventoryBaselineBySerial: Record<string, number | null>;
  records: Pick<MaintenancePodInventorySnapshot, 'podSerial' | 'lastCalibrationAtMs'>[];
}) {
  const { actionStatesBySerial, inventoryBaselineBySerial, records } = args;

  let nextStates = actionStatesBySerial;
  let nextBaselines = inventoryBaselineBySerial;

  for (const record of records) {
    if (!(record.podSerial in inventoryBaselineBySerial)) {
      continue;
    }

    const baseline = inventoryBaselineBySerial[record.podSerial];
    const nextValue = record.lastCalibrationAtMs;
    const hasFreshInventory =
      Number.isFinite(nextValue) &&
      (baseline == null || (nextValue ?? 0) > baseline);

    if (!hasFreshInventory) {
      continue;
    }

    const current = nextStates[record.podSerial];
    if (current?.kind === 'success') {
      if (nextStates === actionStatesBySerial) {
        nextStates = { ...actionStatesBySerial };
      }
      nextStates[record.podSerial] = null;
    }
    if (nextBaselines === inventoryBaselineBySerial) {
      nextBaselines = { ...inventoryBaselineBySerial };
    }
    delete nextBaselines[record.podSerial];
  }

  return {
    actionStatesBySerial: nextStates,
    inventoryBaselineBySerial: nextBaselines,
  };
}

export function usePodCalibrationAction(
  records: MaintenancePodInventorySnapshot[] = []
) {
  const { ros, isConnected: rosConnected } = useSharedROSConnection();
  const [submittingPodSerial, setSubmittingPodSerial] = useState<string | null>(null);
  const [inventoryBaselineBySerial, setInventoryBaselineBySerial] = useState<
    Record<string, number | null>
  >({});
  const [actionStatesBySerial, setActionStatesBySerial] = useState<
    Record<string, PodCalibrationActionState | null>
  >({});

  useEffect(() => {
    if (records.length === 0) {
      return;
    }
    const reconciled = reconcilePodCalibrationActionState({
      actionStatesBySerial,
      inventoryBaselineBySerial,
      records,
    });
    if (reconciled.actionStatesBySerial !== actionStatesBySerial) {
      setActionStatesBySerial(reconciled.actionStatesBySerial);
    }
    if (reconciled.inventoryBaselineBySerial !== inventoryBaselineBySerial) {
      setInventoryBaselineBySerial(reconciled.inventoryBaselineBySerial);
    }
  }, [actionStatesBySerial, inventoryBaselineBySerial, records]);

  const requestCalibration = useCallback(
    async (request: {
      podSerial: string;
      lastCalibrationAtMs: number;
      nextCalibrationDueAtMs: number;
    }) => {
      const previousInventory = records.find(
        (record) => record.podSerial === request.podSerial
      );
      setSubmittingPodSerial(request.podSerial);
      setInventoryBaselineBySerial((current) => ({
        ...current,
        [request.podSerial]: previousInventory?.lastCalibrationAtMs ?? null,
      }));
      try {
        const result = await withServiceTimeout(
          (resolve, reject) => {
            if (!ros || !rosConnected) {
              reject(new Error('ROS is not connected'));
              return;
            }

            const service = new ROSLIB.Service({
              ros,
              name: '/device_manager/log_pod_calibration',
              serviceType: 'aeris_msgs/srv/LogPodCalibration',
            });
            const serviceRequest = new ROSLIB.ServiceRequest({
              pod_serial: request.podSerial,
              last_calibration: toRosTime(request.lastCalibrationAtMs),
              next_calibration_due: toRosTime(request.nextCalibrationDueAtMs),
            });
            service.callService(
              serviceRequest,
              (response) => resolve(parseLogPodCalibrationResponse(response)),
              (error) => reject(new Error(String(error)))
            );
          },
          POD_CALIBRATION_SERVICE_TIMEOUT_MS,
          'pod calibration service call'
        );

        const typed = result as ReturnType<typeof parseLogPodCalibrationResponse>;
        setActionStatesBySerial((current) => ({
          ...current,
          [request.podSerial]: typed.accepted
            ? { kind: 'success', message: typed.message }
            : { kind: 'error', message: typed.message || typed.failureCode || 'Calibration update was rejected.' },
        }));
        if (!typed.accepted) {
          setInventoryBaselineBySerial((current) => {
            const next = { ...current };
            delete next[request.podSerial];
            return next;
          });
        }
        return typed;
      } catch (error) {
        const message = error instanceof Error ? error.message : String(error);
        setActionStatesBySerial((current) => ({
          ...current,
          [request.podSerial]: { kind: 'error', message },
        }));
        setInventoryBaselineBySerial((current) => {
          const next = { ...current };
          delete next[request.podSerial];
          return next;
        });
        return {
          accepted: false,
          message,
          failureCode: 'request_failed',
          record: null,
        };
      } finally {
        setSubmittingPodSerial((current) =>
          current === request.podSerial ? null : current
        );
      }
    },
    [records, ros, rosConnected]
  );

  return useMemo(
    () => ({
      requestCalibration,
      submittingPodSerial,
      actionStatesBySerial,
    }),
    [actionStatesBySerial, requestCalibration, submittingPodSerial]
  );
}

function toRosTime(valueMs: number) {
  const wholeSeconds = Math.floor(valueMs / 1000);
  const remainingMs = valueMs - wholeSeconds * 1000;
  return {
    sec: wholeSeconds,
    nanosec: Math.floor(remainingMs * 1_000_000),
  };
}
