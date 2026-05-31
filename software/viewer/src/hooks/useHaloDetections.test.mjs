import test, { after } from "node:test";
import assert from "node:assert/strict";
import fs from "node:fs";
import os from "node:os";
import path from "node:path";
import ts from "typescript";
import { pathToFileURL } from "node:url";

const ROOT = path.resolve(new URL(".", import.meta.url).pathname);
const sourcePath = path.join(ROOT, "useHaloDetections.ts");
const source = fs.readFileSync(sourcePath, "utf8");
const tempDir = fs.mkdtempSync(path.join(os.tmpdir(), "aeris-halo-detections-"));

after(() => {
  fs.rmSync(tempDir, { recursive: true, force: true });
});

fs.writeFileSync(path.join(tempDir, "react.mjs"), `
  export function useEffect() {}
  export function useState(initial) { return [initial, () => {}]; }
`, "utf8");

fs.writeFileSync(path.join(tempDir, "roslib.mjs"), `export default { Topic: class Topic {} };`, "utf8");
fs.writeFileSync(path.join(tempDir, "context.mjs"), `export function useSharedROSConnection() { return { ros: null, isConnected: false }; }`, "utf8");
fs.writeFileSync(path.join(tempDir, "haloConfidenceEvents.mjs"), `
  export function normalizeHaloConfidenceEventMessage(value) {
    return { kind: "event", value };
  }
  export function normalizeHaloEvidenceLogRecord(value) {
    return { kind: "record", value };
  }
  export function normalizeHaloEvidenceReplayPayload(value) {
    return [{ kind: "jsonl", value }];
  }
`, "utf8");
fs.writeFileSync(path.join(tempDir, "fusedDetectionsFeed.mjs"), `
  export function mergeLiveDetections(previous, incoming) {
    return [...previous, incoming];
  }
`, "utf8");

const compiled = ts.transpileModule(
  source
    .replaceAll("react", "./react.mjs")
    .replaceAll("roslib", "./roslib.mjs")
    .replaceAll("@/context/ROSConnectionContext", "./context.mjs")
    .replaceAll("@/lib/ros/haloConfidenceEvents", "./haloConfidenceEvents.mjs")
    .replaceAll("@/lib/ros/fusedDetectionsFeed", "./fusedDetectionsFeed.mjs"),
  {
    compilerOptions: {
      module: ts.ModuleKind.ES2022,
      target: ts.ScriptTarget.ES2022,
    },
  }
).outputText;

const modulePath = path.join(tempDir, "useHaloDetections.mjs");
fs.writeFileSync(modulePath, compiled, "utf8");

const { mergeHaloDetectionBatch, normalizeHaloInboundPayload } = await import(pathToFileURL(modulePath).href);

test("normalizeHaloInboundPayload parses pretty-printed JSON before falling back to JSONL", () => {
  const prettyPrintedEnvelope = JSON.stringify({
    schema_version: 1,
    sequence: 0,
    run_id: "halo-demo-run",
    mode: "replay",
    event: {
      event_id: "halo-confidence-001",
    },
  }, null, 2);

  const parsed = normalizeHaloInboundPayload({ data: prettyPrintedEnvelope });

  assert.deepEqual(parsed, [
    {
      kind: "record",
      value: {
        schema_version: 1,
        sequence: 0,
        run_id: "halo-demo-run",
        mode: "replay",
        event: {
          event_id: "halo-confidence-001",
        },
      },
    },
  ]);
});

test("normalizeHaloInboundPayload falls back to JSONL only when JSON parsing fails", () => {
  const jsonl = [
    JSON.stringify({ schema_version: 1, sequence: 0, run_id: "halo-demo-run", mode: "replay", event: { event_id: "one" } }),
    JSON.stringify({ schema_version: 1, sequence: 1, run_id: "halo-demo-run", mode: "replay", event: { event_id: "two" } }),
  ].join("\n");

  const parsed = normalizeHaloInboundPayload({ data: jsonl });

  assert.deepEqual(parsed, [{ kind: "jsonl", value: jsonl }]);
});

test("normalizeHaloInboundPayload guards against runaway doubly-encoded string recursion", () => {
  const doublyEncoded = JSON.stringify(
    JSON.stringify(
      JSON.stringify(
        JSON.stringify({ event_id: "deep" })
      )
    )
  );

  assert.throws(
    () => normalizeHaloInboundPayload({ data: doublyEncoded }),
    /depth/i
  );
});

test("mergeHaloDetectionBatch preserves multiple old replay detections from one inbound batch", () => {
  const nowMs = 1_800_000_000_000;
  const previous = [
    {
      id: "live-current",
      timestamp: nowMs - 5_000,
      deliveryMode: "live",
    },
    {
      id: "stale-live",
      timestamp: nowMs - (20 * 60 * 1000),
      deliveryMode: "live",
    },
  ];
  const incomingReplayBatch = [
    {
      id: "replay-1",
      timestamp: nowMs - (30 * 60 * 1000),
      deliveryMode: "replayed",
      isRetroactive: true,
    },
    {
      id: "replay-2",
      timestamp: nowMs - (31 * 60 * 1000),
      deliveryMode: "replayed",
      isRetroactive: true,
    },
  ];

  const merged = mergeHaloDetectionBatch(previous, incomingReplayBatch, {
    nowMs,
    maxAgeMs: 10 * 60 * 1000,
    maxDetections: 10,
  });

  assert.deepEqual(
    merged.map((detection) => detection.id),
    ["live-current", "replay-1", "replay-2"]
  );
});
