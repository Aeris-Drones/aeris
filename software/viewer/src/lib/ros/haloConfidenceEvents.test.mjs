import test from "node:test";
import assert from "node:assert/strict";

import { normalizeHaloConfidenceEventMessage } from "./haloConfidenceEvents.js";

test("normalizeHaloConfidenceEventMessage maps the canonical event into a display-friendly model", () => {
  const event = {
    event_id: "halo-confidence-001",
    source_id: "camera:halo_front_camera",
    source_name: "halo_front_camera",
    source_uri: "rtsp://halo/front",
    frame_id: "halo_rgb_front",
    frame_index: 21,
    timestamp_ns: 1_717_200_123_456_789,
    label: "Possible Survivor",
    detection_type: "candidate_human_presence",
    confidence: 0.72,
    confidence_level: "MEDIUM",
    region: { x: 11, y: 15, width: 20, height: 28 },
    location_hint: { label: "Zone E-2", x: 12.5, y: 0.0, z: -6.0 },
    evidence_ref: "evidence-017",
    evidence_uri: "file:///tmp/halo/evidence-017.json",
    recognition: {
      baseline_name: "halo_rgb_region_baseline",
      baseline_version: "0.1.0",
      confidence_components: { brightness: 0.91, fill: 0.87 },
    },
  };

  const detection = normalizeHaloConfidenceEventMessage(event);

  assert.equal(detection.id, "halo-confidence-001");
  assert.equal(detection.sensorType, "rgb");
  assert.equal(detection.confidence, 0.72);
  assert.equal(detection.confidenceLevel, "MEDIUM");
  assert.equal(detection.timestamp, 1_717_200_123_456);
  assert.deepEqual(detection.position, [12.5, 0, -6]);
  assert.deepEqual(detection.sourceModalities, ["rgb"]);
  assert.equal(detection.vehicleId, "camera:halo_front_camera");
  assert.equal(detection.vehicleName, "halo_front_camera");
  assert.equal(detection.sector, "Zone E-2");
  assert.equal(detection.signatureType, "Possible Survivor");
  assert.deepEqual(detection.region, { x: 11, y: 15, width: 20, height: 28 });
  assert.equal(detection.evidenceRef, "evidence-017");
  assert.equal(detection.evidenceUri, "file:///tmp/halo/evidence-017.json");
  assert.equal(detection.recognition.baseline_name, "halo_rgb_region_baseline");
});

test("normalizeHaloConfidenceEventMessage derives defaults and preserves optional field absence", () => {
  const detection = normalizeHaloConfidenceEventMessage({
    source_id: "camera:halo_front_camera",
    source_name: "halo_front_camera",
    frame_id: "halo_rgb_front",
    frame_index: 4,
    timestamp_ns: 123_456_789,
    detection_type: "candidate_human_presence",
    confidence: 1.4,
    recognition: {
      baseline_name: "halo_rgb_region_baseline",
      baseline_version: "0.1.0",
    },
  });

  assert.ok(detection.id.startsWith("halo-confidence-"));
  assert.equal(detection.confidence, 1);
  assert.equal(detection.confidenceLevel, "HIGH");
  assert.deepEqual(detection.position, [0, 0, 0]);
  assert.equal(detection.sector, "halo_rgb_front");
  assert.equal(detection.signatureType, "Candidate Human Presence");
  assert.equal(detection.region, undefined);
  assert.equal(detection.locationHint, undefined);
  assert.equal(detection.evidenceRef, undefined);
  assert.equal(detection.evidenceUri, undefined);
});

test("normalizeHaloConfidenceEventMessage supports string location hints", () => {
  const detection = normalizeHaloConfidenceEventMessage({
    source_id: "camera:halo_rear_camera",
    source_name: "halo_rear_camera",
    frame_id: "halo_rgb_rear",
    frame_index: 9,
    timestamp_ns: 999_000_321,
    label: "Candidate Human Presence",
    detection_type: "candidate_human_presence",
    confidence: 0.31,
    location_hint: "Zone W-1",
    recognition: {
      baseline_name: "halo_rgb_region_baseline",
      baseline_version: "0.1.0",
    },
  });

  assert.equal(detection.sector, "Zone W-1");
  assert.deepEqual(detection.position, [0, 0, 0]);
  assert.equal(detection.confidenceLevel, "LOW");
});

test("normalizeHaloConfidenceEventMessage rejects invalid payloads", () => {
  assert.throws(
    () => normalizeHaloConfidenceEventMessage({ confidence: 0.4 }),
    /source_id/
  );

  assert.throws(
    () => normalizeHaloConfidenceEventMessage({
      source_id: "camera:halo_front_camera",
      source_name: "halo_front_camera",
      frame_id: "halo_rgb_front",
      frame_index: 1,
      timestamp_ns: 10,
      detection_type: "candidate_human_presence",
      confidence: 0.5,
      region: { x: 1, y: 2, width: -2, height: 10 },
      recognition: {
        baseline_name: "halo_rgb_region_baseline",
        baseline_version: "0.1.0",
      },
    }),
    /region/
  );
});
