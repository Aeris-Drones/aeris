import { useCallback, useMemo, useState } from 'react';
import ROSLIB from 'roslib';
import { useSharedROSConnection } from '@/context/ROSConnectionContext';
import { withServiceTimeout } from '@/lib/missionControlBehavior.js';
import type { FirmwareUpdateCommandInput } from '@/lib/ros/firmwareUpdateStatus';

const FIRMWARE_UPDATE_SERVICE_TIMEOUT_MS = 8_000;

interface FirmwareUpdateActionResult {
  accepted: boolean;
  message: string;
}

export function useFirmwareUpdateAction() {
  const { ros, isConnected: rosConnected } = useSharedROSConnection();
  const [submittingVehicleId, setSubmittingVehicleId] = useState<string | null>(null);
  const [errorsByVehicle, setErrorsByVehicle] = useState<Record<string, string | null>>({});

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
