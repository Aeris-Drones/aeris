import test from "node:test";
import assert from "node:assert/strict";

import {
  normalizeHaloConfidenceEventMessage,
  normalizeHaloEvidenceLogRecord,
  normalizeHaloEvidenceReplayPayload,
} from "./haloConfidenceEvents.js";

test("normalizeHaloConfidenceEventMessage maps the canonical event into a display-friendly model", () => {
  const event = {
    event_id: "halo-confidence-001",
    source_id: "camera:halo_front_camera",
    source_name: "halo_front_camera",
    source_uri: "rtsp://halo/front",
    frame_id: "halo_rgb_front",
    frame_index: 21,
    timestamp_ns: "1717200123456789000",
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
  assert.equal(detection.timestampNs, "1717200123456789000");
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
    timestamp_ns: "123456789",
    detection_type: "candidate_human_presence",
    confidence: 1.4,
    recognition: {
      baseline_name: "halo_rgb_region_baseline",
      baseline_version: "0.1.0",
    },
  }, { nowMs: 1_800_000 });

  assert.ok(detection.id.startsWith("halo-confidence-"));
  assert.equal(detection.confidence, 1);
  assert.equal(detection.confidenceLevel, "HIGH");
  assert.equal(detection.timestamp, 1_800_000);
  assert.equal(detection.timestampNs, "123456789");
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
    timestamp_ns: "999000321",
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

test("normalizeHaloConfidenceEventMessage derives detection type from a label-only payload", () => {
  const detection = normalizeHaloConfidenceEventMessage({
    source_id: "camera:halo_side_camera",
    source_name: "halo_side_camera",
    frame_id: "halo_rgb_side",
    frame_index: 3,
    timestamp_ns: "1000000000",
    label: "Candidate Human Presence",
    confidence: 0.62,
    location_hint: { label: "Zone S-1", x: 1, y: 2, z: 3 },
    recognition: {
      baseline_name: "halo_rgb_region_baseline",
      baseline_version: "0.1.0",
      source_timestamp_ns: "900000000",
    },
  }, { nowMs: 2_000_000 });

  assert.equal(detection.detectionType, "candidate_human_presence");
  assert.equal(detection.label, "Candidate Human Presence");
  assert.equal(detection.signatureType, "Candidate Human Presence");
  assert.equal(detection.confidenceLevel, "MEDIUM");
  assert.equal(detection.timestamp, 2_000_000);
  assert.equal(detection.recognition.source_timestamp_ns, "900000000");
  assert.deepEqual(detection.position, [1, 2, 3]);
  assert.equal(detection.sector, "Zone S-1");
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
      timestamp_ns: "10",
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

  assert.throws(
    () => normalizeHaloConfidenceEventMessage({
      source_id: "camera:halo_front_camera",
      source_name: "halo_front_camera",
      frame_id: "halo_rgb_front",
      frame_index: true,
      timestamp_ns: "10",
      detection_type: "candidate_human_presence",
      confidence: 0.5,
      recognition: {
        baseline_name: "halo_rgb_region_baseline",
        baseline_version: "0.1.0",
      },
    }),
    /frame_index/
  );

  assert.throws(
    () => normalizeHaloConfidenceEventMessage({
      source_id: "camera:halo_front_camera",
      source_name: "halo_front_camera",
      frame_id: "halo_rgb_front",
      frame_index: 1,
      timestamp_ns: false,
      detection_type: "candidate_human_presence",
      confidence: 0.5,
      recognition: {
        baseline_name: "halo_rgb_region_baseline",
        baseline_version: "0.1.0",
      },
    }),
    /timestamp_ns/
  );

  assert.throws(
    () => normalizeHaloConfidenceEventMessage({
      source_id: "camera:halo_front_camera",
      source_name: "halo_front_camera",
      frame_id: "halo_rgb_front",
      frame_index: 1,
      timestamp_ns: "10",
      detection_type: "candidate_human_presence",
      confidence: null,
      recognition: {
        baseline_name: "halo_rgb_region_baseline",
        baseline_version: "0.1.0",
      },
    }),
    /confidence/
  );
});

test("normalizeHaloEvidenceLogRecord reuses the nested event and preserves evidence handles", () => {
  const detection = normalizeHaloEvidenceLogRecord({
    schema_version: 1,
    sequence: 4,
    recorded_at_ns: "1717200124456789000",
    run_id: "halo-demo-run",
    mode: "replay",
    evidence_path: "/tmp/halo/capture-017.png",
    evidence_ref: "evidence-017",
    evidence_uri: "file:///tmp/halo/capture-017.png",
    event: {
      event_id: "halo-confidence-001",
      source_id: "camera:halo_front_camera",
      source_name: "halo_front_camera",
      source_uri: "rtsp://halo/front",
      frame_id: "halo_rgb_front",
      frame_index: 21,
      timestamp_ns: "1717200123456789000",
      label: "Possible Survivor",
      detection_type: "candidate_human_presence",
      confidence: 0.72,
      confidence_level: "MEDIUM",
      location_hint: { label: "Zone E-2", x: 12.5, y: 0.0, z: -6.0 },
      recognition: {
        baseline_name: "halo_rgb_region_baseline",
        baseline_version: "0.1.0",
      },
    },
  });

  assert.equal(detection.id, "halo-confidence-001");
  assert.equal(detection.sensorType, "rgb");
  assert.equal(detection.deliveryMode, "replayed");
  assert.equal(detection.isRetroactive, true);
  assert.equal(detection.originalEventTs, 1_717_200_123_456);
  assert.equal(detection.replayedAtTs, 1_717_200_124_456);
  assert.equal(detection.evidencePath, "/tmp/halo/capture-017.png");
  assert.equal(detection.evidenceRef, "evidence-017");
  assert.equal(detection.evidenceUri, "file:///tmp/halo/capture-017.png");
});

test("normalizeHaloEvidenceLogRecord keeps live and evaluation wrapper modes distinct from replay", () => {
  const liveDetection = normalizeHaloEvidenceLogRecord({
    schema_version: 1,
    sequence: 0,
    run_id: "halo-live-run",
    mode: "live",
    event: {
      event_id: "halo-confidence-live-001",
      source_id: "camera:halo_front_camera",
      source_name: "halo_front_camera",
      frame_id: "halo_rgb_front",
      frame_index: 21,
      timestamp_ns: "1717200123456789000",
      label: "Possible Survivor",
      detection_type: "candidate_human_presence",
      confidence: 0.72,
      recognition: {
        baseline_name: "halo_rgb_region_baseline",
        baseline_version: "0.1.0",
      },
    },
  });
  const evaluationDetection = normalizeHaloEvidenceLogRecord({
    schema_version: 1,
    sequence: 1,
    run_id: "halo-eval-run",
    mode: "evaluation",
    recorded_at_ns: "1717200124456789000",
    event: {
      event_id: "halo-confidence-eval-001",
      source_id: "camera:halo_front_camera",
      source_name: "halo_front_camera",
      frame_id: "halo_rgb_front",
      frame_index: 22,
      timestamp_ns: "1717200123456789000",
      label: "Possible Survivor",
      detection_type: "candidate_human_presence",
      confidence: 0.79,
      recognition: {
        baseline_name: "halo_rgb_region_baseline",
        baseline_version: "0.1.0",
      },
    },
  });

  assert.equal(liveDetection.deliveryMode, "live");
  assert.equal(liveDetection.isRetroactive, false);
  assert.equal(liveDetection.replayedAtTs, undefined);
  assert.equal(evaluationDetection.deliveryMode, "live");
  assert.equal(evaluationDetection.isRetroactive, false);
  assert.equal(evaluationDetection.replayedAtTs, undefined);
});

test("normalizeHaloEvidenceReplayPayload parses JSONL records and record objects in order", () => {
  const jsonl = [
    JSON.stringify({
      schema_version: 1,
      sequence: 0,
      recorded_at_ns: "1717200124456789000",
      run_id: "halo-demo-run",
      mode: "replay",
      evidence_path: "/tmp/halo/capture-017.png",
      event: {
        event_id: "halo-confidence-001",
        source_id: "camera:halo_front_camera",
        source_name: "halo_front_camera",
        frame_id: "halo_rgb_front",
        frame_index: 21,
        timestamp_ns: "1717200123456789000",
        label: "Possible Survivor",
        confidence: 0.72,
        detection_type: "candidate_human_presence",
        recognition: {
          baseline_name: "halo_rgb_region_baseline",
          baseline_version: "0.1.0",
        },
      },
    }),
    JSON.stringify({
      schema_version: 1,
      sequence: 1,
      recorded_at_ns: "1717200125456789000",
      run_id: "halo-demo-run",
      mode: "replay",
      evidence_ref: "evidence-018",
      event: {
        event_id: "halo-confidence-002",
        source_id: "camera:halo_rear_camera",
        source_name: "halo_rear_camera",
        frame_id: "halo_rgb_rear",
        frame_index: 22,
        timestamp_ns: "1717200124456789000",
        label: "Debris Movement",
        confidence: 0.64,
        detection_type: "scene_change",
        recognition: {
          baseline_name: "halo_rgb_region_baseline",
          baseline_version: "0.1.0",
        },
      },
    }),
  ].join("\n");

  const detections = normalizeHaloEvidenceReplayPayload(jsonl);
  const objectDetections = normalizeHaloEvidenceReplayPayload([
    {
      schema_version: 1,
      sequence: 2,
      recorded_at_ns: "1717200126456789000",
      run_id: "halo-demo-run",
      mode: "replay",
      event: {
        event_id: "halo-confidence-003",
        source_id: "camera:halo_side_camera",
        source_name: "halo_side_camera",
        frame_id: "halo_rgb_side",
        frame_index: 23,
        timestamp_ns: "1717200125456789000",
        label: "Possible Survivor",
        confidence: 0.81,
        detection_type: "candidate_human_presence",
        recognition: {
          baseline_name: "halo_rgb_region_baseline",
          baseline_version: "0.1.0",
        },
      },
    },
  ]);

  assert.deepEqual(detections.map((detection) => detection.id), [
    "halo-confidence-001",
    "halo-confidence-002",
  ]);
  assert.equal(detections[0].deliveryMode, "replayed");
  assert.equal(detections[0].evidencePath, "/tmp/halo/capture-017.png");
  assert.equal(detections[1].evidenceRef, "evidence-018");
  assert.equal(objectDetections[0].id, "halo-confidence-003");
  assert.equal(objectDetections[0].isRetroactive, true);
});

test("normalizeHaloEvidenceReplayPayload preserves valid detections when a JSONL batch contains malformed records", () => {
  const mixedJsonl = [
    JSON.stringify({
      schema_version: 1,
      sequence: 0,
      run_id: "halo-demo-run",
      mode: "replay",
      event: {
        event_id: "halo-confidence-001",
        source_id: "camera:halo_front_camera",
        source_name: "halo_front_camera",
        frame_id: "halo_rgb_front",
        frame_index: 21,
        timestamp_ns: "1717200123456789000",
        label: "Possible Survivor",
        confidence: 0.72,
        detection_type: "candidate_human_presence",
        recognition: {
          baseline_name: "halo_rgb_region_baseline",
          baseline_version: "0.1.0",
        },
      },
    }),
    "{not-json}",
    JSON.stringify({
      schema_version: 1,
      sequence: 2,
      run_id: "halo-demo-run",
      mode: "replay",
      event: {
        event_id: "halo-confidence-003",
        source_id: "camera:halo_side_camera",
        source_name: "halo_side_camera",
        frame_id: "halo_rgb_side",
        frame_index: 23,
        timestamp_ns: "1717200125456789000",
        label: "Possible Survivor",
        confidence: 0.81,
        detection_type: "candidate_human_presence",
        recognition: {
          baseline_name: "halo_rgb_region_baseline",
          baseline_version: "0.1.0",
        },
      },
    }),
    JSON.stringify({
      schema_version: 1,
      sequence: 3,
      run_id: "halo-demo-run",
      mode: "replay",
      event: {
        source_id: "camera:halo_broken_camera",
        source_name: "halo_broken_camera",
        frame_id: "halo_rgb_broken",
        frame_index: 24,
        timestamp_ns: "1717200126456789000",
        confidence: 0.5,
        recognition: {
          baseline_name: "halo_rgb_region_baseline",
          baseline_version: "0.1.0",
        },
      },
    }),
  ].join("\n");

  const detections = normalizeHaloEvidenceReplayPayload(mixedJsonl);

  assert.deepEqual(detections.map((detection) => detection.id), [
    "halo-confidence-001",
    "halo-confidence-003",
  ]);
});

test("normalizeHaloEvidenceReplayPayload rejects malformed JSONL and malformed record payloads", () => {
  assert.throws(
    () => normalizeHaloEvidenceReplayPayload("{not-json}"),
    /Malformed Halo evidence replay payload/
  );

  assert.throws(
    () =>
      normalizeHaloEvidenceLogRecord({
        schema_version: 1,
        sequence: 0,
        recorded_at_ns: "1717200124456789000",
        run_id: "halo-demo-run",
        mode: "replay",
      }),
    /event/
  );

  assert.throws(
    () =>
      normalizeHaloEvidenceReplayPayload([
        {
          schema_version: 1,
          sequence: 0,
          recorded_at_ns: "1717200124456789000",
          run_id: "halo-demo-run",
          mode: "replay",
          event: {
            source_id: "camera:halo_front_camera",
            source_name: "halo_front_camera",
            frame_id: "halo_rgb_front",
            frame_index: 1,
            timestamp_ns: "10",
            confidence: 0.5,
            recognition: {
              baseline_name: "halo_rgb_region_baseline",
              baseline_version: "0.1.0",
            },
          },
        },
    ]),
    /no valid replay batch records/i
  );

  assert.throws(
    () =>
      normalizeHaloEvidenceLogRecord({
        schema_version: 2,
        sequence: 0,
        run_id: "halo-demo-run",
        mode: "replay",
        event: {
          event_id: "halo-confidence-001",
          source_id: "camera:halo_front_camera",
          source_name: "halo_front_camera",
          frame_id: "halo_rgb_front",
          frame_index: 1,
          timestamp_ns: "10",
          label: "Possible Survivor",
          detection_type: "candidate_human_presence",
          confidence: 0.5,
          recognition: {
            baseline_name: "halo_rgb_region_baseline",
            baseline_version: "0.1.0",
          },
        },
      }),
    /schema_version/
  );

  assert.throws(
    () =>
      normalizeHaloEvidenceLogRecord({
        schema_version: 1,
        sequence: -1,
        run_id: "halo-demo-run",
        mode: "replay",
        event: {
          event_id: "halo-confidence-001",
          source_id: "camera:halo_front_camera",
          source_name: "halo_front_camera",
          frame_id: "halo_rgb_front",
          frame_index: 1,
          timestamp_ns: "10",
          label: "Possible Survivor",
          detection_type: "candidate_human_presence",
          confidence: 0.5,
          recognition: {
            baseline_name: "halo_rgb_region_baseline",
            baseline_version: "0.1.0",
          },
        },
      }),
    /sequence/
  );

  assert.throws(
    () =>
      normalizeHaloEvidenceLogRecord({
        schema_version: 1,
        sequence: 0,
        run_id: "",
        mode: "replay",
        event: {
          event_id: "halo-confidence-001",
          source_id: "camera:halo_front_camera",
          source_name: "halo_front_camera",
          frame_id: "halo_rgb_front",
          frame_index: 1,
          timestamp_ns: "10",
          label: "Possible Survivor",
          detection_type: "candidate_human_presence",
          confidence: 0.5,
          recognition: {
            baseline_name: "halo_rgb_region_baseline",
            baseline_version: "0.1.0",
          },
        },
      }),
    /run_id/
  );

  assert.throws(
    () =>
      normalizeHaloEvidenceLogRecord({
        schema_version: 1,
        sequence: 0,
        run_id: "halo-demo-run",
        mode: "archive",
        event: {
          event_id: "halo-confidence-001",
          source_id: "camera:halo_front_camera",
          source_name: "halo_front_camera",
          frame_id: "halo_rgb_front",
          frame_index: 1,
          timestamp_ns: "10",
          label: "Possible Survivor",
          detection_type: "candidate_human_presence",
          confidence: 0.5,
          recognition: {
            baseline_name: "halo_rgb_region_baseline",
            baseline_version: "0.1.0",
          },
        },
      }),
    /mode/
  );

  assert.throws(
    () =>
      normalizeHaloEvidenceLogRecord({
        schema_version: 1,
        sequence: 0,
        run_id: "halo-demo-run",
        mode: "replay",
        recorded_at_ns: "not-an-int",
        event: {
          event_id: "halo-confidence-001",
          source_id: "camera:halo_front_camera",
          source_name: "halo_front_camera",
          frame_id: "halo_rgb_front",
          frame_index: 1,
          timestamp_ns: "10",
          label: "Possible Survivor",
          detection_type: "candidate_human_presence",
          confidence: 0.5,
          recognition: {
            baseline_name: "halo_rgb_region_baseline",
            baseline_version: "0.1.0",
          },
        },
      }),
    /recorded_at_ns/
  );

  assert.throws(
    () =>
      normalizeHaloEvidenceReplayPayload([
        "{not-json}",
        JSON.stringify({
          schema_version: 1,
          sequence: 3,
          run_id: "halo-demo-run",
          mode: "replay",
          event: {
            source_id: "camera:halo_broken_camera",
            source_name: "halo_broken_camera",
            frame_id: "halo_rgb_broken",
            frame_index: 24,
            timestamp_ns: "1717200126456789000",
            confidence: 0.5,
            recognition: {
              baseline_name: "halo_rgb_region_baseline",
              baseline_version: "0.1.0",
            },
          },
        }),
      ]),
    /no valid/i
  );
});
