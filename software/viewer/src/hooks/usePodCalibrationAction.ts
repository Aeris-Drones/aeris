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
  submittingPodSerial: string | null;
  records: Pick<MaintenancePodInventorySnapshot, 'podSerial' | 'lastCalibrationAtMs'>[];
}) {
  const { actionStatesBySerial, submittingPodSerial, records } = args;

  let nextStates = actionStatesBySerial;
  let nextSubmitting = submittingPodSerial;

  for (const record of records) {
    if (
      record.podSerial === submittingPodSerial &&
      Number.isFinite(record.lastCalibrationAtMs)
    ) {
      nextSubmitting = null;
      const current = nextStates[record.podSerial];
      if (current?.kind === 'success') {
        if (nextStates === actionStatesBySerial) {
          nextStates = { ...actionStatesBySerial };
        }
        nextStates[record.podSerial] = null;
      }
    }
  }

  return {
    actionStatesBySerial: nextStates,
    submittingPodSerial: nextSubmitting,
  };
}

export function usePodCalibrationAction(
  records: MaintenancePodInventorySnapshot[] = []
) {
  const { ros, isConnected: rosConnected } = useSharedROSConnection();
  const [submittingPodSerial, setSubmittingPodSerial] = useState<string | null>(null);
  const [actionStatesBySerial, setActionStatesBySerial] = useState<
    Record<string, PodCalibrationActionState | null>
  >({});

  useEffect(() => {
    if (records.length === 0) {
      return;
    }
    const reconciled = reconcilePodCalibrationActionState({
      actionStatesBySerial,
      submittingPodSerial,
      records,
    });
    if (reconciled.actionStatesBySerial !== actionStatesBySerial) {
      setActionStatesBySerial(reconciled.actionStatesBySerial);
    }
    if (reconciled.submittingPodSerial !== submittingPodSerial) {
      setSubmittingPodSerial(reconciled.submittingPodSerial);
    }
  }, [actionStatesBySerial, records, submittingPodSerial]);

  const requestCalibration = useCallback(
    async (request: {
      podSerial: string;
      lastCalibrationAtMs: number;
      nextCalibrationDueAtMs: number;
    }) => {
      setSubmittingPodSerial(request.podSerial);
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
        return typed;
      } catch (error) {
        const message = error instanceof Error ? error.message : String(error);
        setActionStatesBySerial((current) => ({
          ...current,
          [request.podSerial]: { kind: 'error', message },
        }));
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
    [ros, rosConnected]
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
