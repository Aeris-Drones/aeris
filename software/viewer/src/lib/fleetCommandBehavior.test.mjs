import test from "node:test";
import assert from "node:assert/strict";

import {
  buildVehicleCommandServiceRequest,
  callVehicleCommandService,
  formatVehicleCommandFailure,
  getVehicleCommandValidationError,
} from "./fleetCommandBehavior.js";

test("buildVehicleCommandServiceRequest includes vehicle_id and command", () => {
  const request = buildVehicleCommandServiceRequest({
    vehicleId: "scout2",
    command: "HOLD",
    missionId: "mission-42",
  });

  assert.deepEqual(request, {
    vehicle_id: "scout2",
    command: "HOLD",
    mission_id: "mission-42",
  });
});

test("callVehicleCommandService resolves rejected responses for UI handling", async () => {
  const calls = [];

  const result = await callVehicleCommandService({
    ros: {},
    request: {
      vehicle_id: "unknown",
      command: "HOLD",
      mission_id: "mission-42",
    },
    createService: ({ name, serviceType }) => {
      calls.push([name, serviceType]);
      return {
        callService(_request, onSuccess) {
          onSuccess({ success: false, message: "unknown vehicle_id 'unknown'" });
        },
      };
    },
    createServiceRequest: request => request,
  });

  assert.equal(calls.length, 1);
  assert.equal(result.success, false);
  assert.match(result.message, /unknown vehicle_id/);
});

test("vehicle command validation blocks disconnected transport", () => {
  const error = getVehicleCommandValidationError({
    rosConnected: false,
    vehicleId: "scout1",
  });
  assert.equal(
    error,
    "ROS is disconnected. Reconnect before sending vehicle commands."
  );
});

test("formatVehicleCommandFailure returns a clear error string", () => {
  assert.equal(
    formatVehicleCommandFailure("HOLD", "scout1", "vehicle is offline"),
    "HOLD rejected for scout1: vehicle is offline"
  );
});
