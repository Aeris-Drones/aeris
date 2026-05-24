import { useCallback, useEffect, useMemo, useState } from 'react';
import ROSLIB from 'roslib';
import { useSharedROSConnection } from '@/context/ROSConnectionContext';
import { withServiceTimeout } from '@/lib/missionControlBehavior.js';
import {
  isFirmwareUpdateActive,
  type FirmwareUpdateCommandInput,
  type FirmwareUpdateStatusSnapshot,
} from '@/lib/ros/firmwareUpdateStatus';

export const FIRMWARE_UPDATE_SERVICE_TIMEOUT_MS = 30_000; // Async start handshake should fail fast if rosbridge stops responding.

interface FirmwareUpdateActionResult {
  accepted: boolean;
  message: string;
}

export function reconcileFirmwareUpdateActionState(args: {
  errorsByVehicle: Record<string, string | null>;
  submittingVehicleId: string | null;
  statuses: Pick<FirmwareUpdateStatusSnapshot, 'vehicleId' | 'lifecycleState'>[];
}) {
  const { errorsByVehicle, submittingVehicleId, statuses } = args;

  let nextErrors = errorsByVehicle;
  for (const status of statuses) {
    if (!isFirmwareUpdateActive(status)) {
      continue;
    }
    if (nextErrors[status.vehicleId] == null) {
      continue;
    }
    if (nextErrors === errorsByVehicle) {
      nextErrors = { ...errorsByVehicle };
    }
    nextErrors[status.vehicleId] = null;
  }

  const nextSubmittingVehicleId =
    submittingVehicleId &&
    statuses.some(
      (status) =>
        status.vehicleId === submittingVehicleId &&
        isFirmwareUpdateActive(status)
    )
      ? null
      : submittingVehicleId;

  return {
    errorsByVehicle: nextErrors,
    submittingVehicleId: nextSubmittingVehicleId,
  };
}

export function useFirmwareUpdateAction(
  statuses: FirmwareUpdateStatusSnapshot[] = []
) {
  const { ros, isConnected: rosConnected } = useSharedROSConnection();
  const [submittingVehicleId, setSubmittingVehicleId] = useState<string | null>(null);
  const [errorsByVehicle, setErrorsByVehicle] = useState<Record<string, string | null>>({});

  useEffect(() => {
    if (statuses.length === 0) {
      return;
    }
    const reconciled = reconcileFirmwareUpdateActionState({
      errorsByVehicle,
      submittingVehicleId,
      statuses,
    });
    if (reconciled.errorsByVehicle !== errorsByVehicle) {
      setErrorsByVehicle(reconciled.errorsByVehicle);
    }
    if (reconciled.submittingVehicleId !== submittingVehicleId) {
      setSubmittingVehicleId(reconciled.submittingVehicleId);
    }
  }, [errorsByVehicle, statuses, submittingVehicleId]);

  const requestUpdate = useCallback(
    async (request: FirmwareUpdateCommandInput): Promise<FirmwareUpdateActionResult> => {
      setSubmittingVehicleId(request.vehicleId);
      try {
        const result = (await withServiceTimeout(
          (resolve, reject) => {
            if (!ros || !rosConnected) {
              reject(new Error('ROS is not connected'));
              return;
            }

            const service = new ROSLIB.Service({
              ros,
              name: '/vehicle/request_firmware_update',
              serviceType: 'aeris_msgs/srv/FirmwareUpdateCommand',
            });
            const serviceRequest = new ROSLIB.ServiceRequest({
              vehicle_id: request.vehicleId,
              package_id: request.packageId,
              target_version: request.targetVersion,
              package_uri: request.packageUri,
              package_signature: request.packageSignature,
            });
            service.callService(
              serviceRequest,
              (response) => {
                const typed = response as { accepted?: boolean; message?: string };
                resolve({
                  accepted: Boolean(typed.accepted),
                  message: typed.message ?? '',
                });
              },
              (error) => {
                reject(new Error(String(error)));
              }
            );
          },
          FIRMWARE_UPDATE_SERVICE_TIMEOUT_MS,
          'firmware update service call'
        )) as FirmwareUpdateActionResult;

        setErrorsByVehicle((current) => ({
          ...current,
          [request.vehicleId]: result.accepted ? null : result.message || 'Firmware update request was rejected.',
        }));
        return result;
      } catch (error) {
        const message = error instanceof Error ? error.message : String(error);
        setErrorsByVehicle((current) => ({
          ...current,
          [request.vehicleId]: message,
        }));
        return {
          accepted: false,
          message,
        };
      } finally {
        setSubmittingVehicleId((current) =>
          current === request.vehicleId ? null : current
        );
      }
    },
    [ros, rosConnected]
  );

  return useMemo(
    () => ({
      requestUpdate,
      submittingVehicleId,
      errorsByVehicle,
    }),
    [errorsByVehicle, requestUpdate, submittingVehicleId]
  );
}
