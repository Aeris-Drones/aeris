# Story 7.3: Recognition Baseline and Evaluation Harness

Status: review

## GitHub Tracking

- **GitHub Issue:** #121
- **Issue URL:** https://github.com/Aeris-Drones/aeris/issues/121
- **Area:** area:perception
- **Priority:** priority:p1

## Story

As the Halo developer,
I want a simple recognition baseline with measurable evaluation,
so that ML progress is grounded in data and metrics, not vibes.

## Acceptance Criteria

1. Given a captured RGB dataset manifest exists, when the baseline runs over the manifest, then it emits candidate human-presence detections with confidence scores and source/frame metadata.
2. The baseline records per-frame latency and keeps confidence values bounded and explainable.
3. An evaluation command or module reports frames processed, frames skipped, detection count, confidence summary, latency summary, and dataset coverage.
4. Optional manifest label/review fields are used when present to summarize false positives/negatives; unlabeled manifests still run cleanly.
5. Outputs are simple inspectable files suitable for the next confidence-event and evidence-logging work.
6. The design leaves room for a later ML model without hiding baseline behavior or claiming final recognition quality.
7. Focused tests cover manifest evaluation, confidence bounds, latency reporting, empty/invalid dataset behavior, and unlabeled vs labeled runs.

## Tasks / Subtasks

- [ ] Inspect the merged RGB dataset manifest contract and reuse it as the baseline input.
- [ ] Define the baseline output shape for candidate detections and evaluation summaries.
- [ ] Implement a simple, deterministic visual baseline that can run on manifest replay frames.
- [ ] Add an evaluation command or callable module that writes inspectable JSON/JSONL output.
- [ ] Use optional manifest labels/review fields when available without requiring labels on day one.
- [ ] Add tests for baseline output, metrics, invalid manifests, confidence bounds, and labeled/unlabeled evaluation behavior.
- [ ] Keep confidence events, evidence logging, GCS UI, thermal/audio, and drone hardware out of scope.

## Dev Notes

### Product Scope

This story advances this proof loop:

```text
captured RGB dataset -> baseline detection -> measurable evaluation
```

The point is not to claim "we built ML." The point is to create the measuring surface that makes later ML work meaningful. A boring baseline that produces honest metrics is better than a model-shaped demo that nobody can evaluate.

### Existing Code To Reuse

- `software/edge/src/aeris_perception/aeris_perception/rgb_dataset.py`
- `software/edge/src/aeris_perception/aeris_perception/rgb_ingest.py`
- `software/edge/src/aeris_perception/aeris_perception/rgb_ingest_node.py`
- `software/edge/src/aeris_perception/test/test_rgb_dataset.py`
- `software/edge/src/aeris_perception/test/test_rgb_ingest.py`
- `software/edge/src/aeris_perception/setup.py`

Use `load_rgb_dataset_manifest`, `RgbDatasetManifestEntry`, `ManifestReplayFrameSource`, `RgbFrame`, and `RgbFrameMetadata` instead of inventing a parallel dataset/frame contract.

### Baseline Guidance

- Start with an inspectable classical baseline or candidate generator in the perception package.
- Keep the interface model-ready: a later detector should be able to replace the baseline without changing the evaluator contract.
- Do not introduce a heavyweight training stack in this story.
- Do not make a survivor-detection claim. Phrase outputs as candidate visual detections or candidate human-presence detections.
- If the baseline cannot honestly identify people from the available data, still produce measured candidate detections and make that limitation visible in the report.

### Output Contract

Prefer simple files:

- candidate detections as JSONL or JSON records
- evaluation summary as JSON and/or a short human-readable text/markdown report

Each candidate detection should carry enough metadata for the next story:

- source name/id
- frame id
- frame index
- timestamp
- label/type
- confidence
- optional bounding region if available
- baseline name/version

The evaluation summary should include:

- manifest path
- frames processed
- frames skipped
- detections emitted
- confidence min/max/mean or buckets
- latency min/p50/p95/max or equivalent
- labeled frame count
- unlabeled frame count
- false positives/false negatives when labels make that possible

### Architecture Constraints

- Keep the work in `aeris_perception` unless a tiny setup entry point is needed.
- Follow current Python style: typed public functions, Black formatting, plain `assert` tests.
- Avoid ROS coupling for the evaluator; the first useful surface should run from a manifest without a live ROS graph.
- Production ROS nodes must stay sim/hardware compatible. Do not hardcode local camera paths or demo-only directories.
- Manifest errors should fail clearly. Empty datasets should not report success.
- Keep image handling compatible with existing PPM fallback so tests do not require OpenCV.

### Previous Story Intelligence

Story 7.2 established:

- JSONL manifests with optional `label` and `review` fields.
- Manifest replay as a configured RGB source.
- Fail-fast behavior for missing capture files and empty manifests.
- PPM image support for dependency-light tests.
- Capture failures should not pretend the pipeline succeeded.

Do not regress those choices.

### Testing Requirements

Run focused perception tests:

```bash
PYTHONPATH=software/edge/src/aeris_perception:software/edge/src/aeris_msgs python3 -m pytest software/edge/src/aeris_perception/test software/edge/src/aeris_msgs/test -q
```

Also run:

```bash
python3 -m compileall software/edge/src/aeris_perception/aeris_perception
```

If local ROS dependencies are unavailable, report the exact limitation and keep the non-ROS baseline/evaluator tests passing.

### Out of Scope

- Trained ML model work
- Confidence event contract
- Evidence logging
- GCS display changes
- Thermal/audio integration
- Drone, rover, or HTB-1 hardware integration
- Broad package rename/refactor

## Implementation Model Recommendation

### Primary Model

- `gpt-5.4 high`

### Secondary Reviewer

- `gpt-5.3-codex high` or `gpt-5.3-codex xhigh`

### Why This Choice

This is backend-heavy perception work with data contracts, metrics, and tests. It needs stronger implementation judgment than a small drafting task, but it should stay narrow enough for one focused dev pass plus independent review.

## Project Context Reference

- `_bmad-output/project-context.md`
- `_bmad-output/planning-artifacts/prd.md`
- `_bmad-output/planning-artifacts/epics.md`
- `_bmad-output/planning-artifacts/halo-two-week-sprint-plan-2026-05-28.md`
- `AGENTS.md`

## Dev Agent Record

### Agent Model Used

GPT-5 Codex

### Debug Log References

- Issue #121: https://github.com/Aeris-Drones/aeris/issues/121

### Completion Notes List

- Added a deterministic RGB region baseline that reuses the manifest replay frame contract and emits inspectable candidate human-presence detections with bounded confidence, source/frame metadata, optional regions, and per-frame latency.
- Added a manifest evaluator plus CLI entry point that writes JSONL detections and JSON evaluation summaries with dataset coverage, confidence/latency metrics, per-frame results, and labeled false-positive/false-negative reporting when manifest labels or review fields are present.
- Added focused tests covering baseline output, confidence bounds, labeled and unlabeled manifest evaluation, and clear failures for empty or malformed manifests.

### File List

- software/edge/src/aeris_perception/aeris_perception/rgb_recognition_baseline.py
- software/edge/src/aeris_perception/test/test_rgb_recognition_baseline.py
- software/edge/src/aeris_perception/setup.py
- _bmad-output/implementation-artifacts/7-3-recognition-baseline-and-evaluation-harness.md
- _bmad-output/implementation-artifacts/sprint-status.yaml
