# Story 6.2: Battery Monitoring & Low-Battery Warnings

Status: review

## GitHub Tracking

- **GitHub Issue:** #49
- **Issue URL:** https://github.com/Aeris-Drones/aeris/issues/49
- **Area:** area:fleet-management
- **Priority:** priority:p0

<!-- Note: Validation is optional. Run validate-create-story for quality check before dev-story. -->

## Story

As an operator,
I want to see battery levels and receive warnings before vehicles run critically low,
so that I can ensure safe RTL before battery depletion.

## Acceptance Criteria

1. **Given** the existing `VehicleCard` displays battery percentage from telemetry, **when** real battery data flows from PX4 via MAVLink through rosbridge, **then** battery levels update in real time on each `VehicleCard` with color-coded thresholds of green for greater than 50%, amber for greater than 25%, and red for less than or equal to 25%.
2. **And** a prominent warning alert appears when any vehicle drops below a configurable low-battery threshold with a default of 25%, without spamming duplicate alerts while the vehicle remains below threshold.
3. **And** the system provides estimated remaining flight time based on current battery-consumption data, showing an explicit unavailable state instead of inventing a guess when the upstream telemetry does not provide enough information.

## Tasks / Subtasks

- [x] Extend the typed telemetry contract so battery state is first-class data instead of a viewer-only extension (AC: 1, 2, 3)
  - [x] Update `software/edge/src/aeris_msgs/msg/Telemetry.msg` to carry the battery fields this story actually needs, at minimum battery percentage plus a typed remaining-flight-time field and any battery warning state needed by downstream consumers.
  - [x] Keep the typed ROS contract authoritative through the rosbridge path; do not rely on ad hoc JSON payloads or a viewer-only convention as the main battery interface.
  - [x] Add or update focused message/parser coverage so the TypeScript telemetry adapter and ROS-side message definition stay aligned.
- [x] Wire real battery telemetry through the edge-to-viewer path using existing repo seams (AC: 1, 3)
  - [x] Reuse the existing `/vehicle/telemetry` pipeline from `aeris_orchestrator`, `useVehicleTelemetry`, `parseVehicleTelemetry`, and `VehicleManager` rather than creating a parallel battery topic for this story.
  - [x] Ensure the typed telemetry contract has an explicit "estimate unavailable" representation that works in ROS messages, such as a documented sentinel value or a companion validity flag, and map that cleanly to `undefined` / unavailable UI state in the viewer.
  - [x] Keep battery estimate logic close to the typed telemetry source; do not reuse mission `estimated_time_remaining`, which represents mission progress time rather than remaining battery endurance.
- [x] Bring the operator UI into alignment with the Epic 6 thresholds and warning behavior (AC: 1, 2, 3)
  - [x] Update `VehicleCard` and any shared battery-color helpers so the display matches the required thresholds of `> 50`, `> 25`, and `<= 25`.
  - [x] Update fleet-level warning derivation in `software/viewer/src/app/page.tsx` and `FleetCard` so the low-battery alert threshold is configurable with a default of 25% instead of the current mixed 50% and 20% logic.
  - [x] Display remaining flight time in the vehicle summary using clear fallback copy such as `--` or `Unavailable` when the telemetry contract does not provide a trustworthy estimate.
- [x] Surface prominent threshold-crossing alerts through the existing alert stack instead of inventing a new notification system (AC: 2)
  - [x] Reuse `software/viewer/src/components/alerts/AlertStack.tsx` and the centralized alert helpers for battery warnings.
  - [x] Trigger alerts on threshold crossing and recovery transitions with stable IDs per vehicle so repeated telemetry frames below threshold do not create toast spam.
  - [x] Make critical low-battery alerts persistent enough to be operator-visible until acknowledged or the vehicle recovers above threshold.
- [x] Add targeted regression coverage for the backend contract and the viewer presentation rules (AC: 1, 2, 3)
  - [x] Add or update ROS/message-oriented tests covering typed battery fields and unavailable-value handling.
  - [x] Add viewer tests for telemetry parsing, battery threshold coloring, alert deduplication behavior, and remaining-flight-time fallback rendering.
  - [x] Keep validation focused on the battery-monitoring slice; do not widen this story into automatic RTL execution, which belongs to Story 8.2.

## Dev Notes

### Developer Context Section

- Story 6.2 starts from a partially prepared surface rather than a blank slate:
  - `software/viewer/src/lib/ros/telemetry.ts` already accepts optional battery percentage extensions.
  - `software/viewer/src/lib/vehicle/VehicleManager.ts` already stores `batteryPercent`.
  - `VehicleCard`, `FleetCard`, and `AlertStack` already provide the battery visualization and alert primitives this story should extend.
- The current implementation does **not** yet match the story contract:
  - `software/edge/src/aeris_msgs/msg/Telemetry.msg` does not define any battery fields today.
  - `VehicleCard` and `FleetCard` still use the older `> 50`, `> 20`, `<= 20` thresholds.
  - `software/viewer/src/app/page.tsx` currently derives fleet warnings from `<= 50`, which is broader than the low-battery warning requirement.
- Story 6.1 established the continuity guardrail for Epic 6:
  - build on typed ROS contracts and explicit backend seams rather than free-form JSON.
  - keep future maintenance work unblocked by preserving stable fleet-management data surfaces.
- Keep the responsibility split clean:
  - edge / telemetry contract work carries the authoritative battery data.
  - viewer work presents and alerts on that data.
  - automatic RTL policy and dispatch logic remain in Story 8.2, even though the operator warning is intended to support safe RTL decisions.

### Technical Requirements

- Extend the canonical `aeris_msgs/Telemetry` contract instead of leaving battery data as an undocumented rosbridge extension.
- Carry enough data to satisfy the story without UI guesswork:
  - battery percentage for display and thresholding
  - remaining flight time in seconds when upstream telemetry can provide it
  - optional warning / charge-state metadata if needed to keep threshold behavior aligned with PX4 or MAVLink semantics
- Document exactly how unavailable battery-time data is encoded in the ROS contract so Python publishers and the TypeScript parser agree on one representation.
- Preserve the existing `/vehicle/telemetry` topic as the primary integration surface for the viewer.
- Do not repurpose mission-progress ETA as a substitute for battery ETA; `MissionProgress.estimated_time_remaining` is mission-completion time.
- Centralize the low-battery threshold so `VehicleCard`, fleet summary logic, and alerts all evaluate the same default of 25% and can be reconfigured from one place.
- Treat "estimate unavailable" as a valid state throughout the stack. The UI must not fabricate remaining-flight-time values from percentage alone unless the upstream telemetry source explicitly provides a trustworthy estimate or a clearly documented derivation.
- Keep alert behavior edge-triggered and deduplicated per vehicle so high-rate telemetry does not flood the operator with repeated toasts.

### Architecture Compliance

- Preserve the architecture's thin-client rule from `docs/specs/AER-SW-ARCH-001.md`: the GCS visualizes and warns, but flight-critical authority remains with the FMU and upstream vehicle stack.
- Keep battery data on the defined telemetry/control path rather than introducing an ad hoc side channel:
  - ROS 2 / DDS on vehicle
  - rosbridge-backed viewer integration for the current GCS
- Keep this story scoped to monitoring and warnings. Do not silently implement automatic low-battery RTL dispatch here; that belongs to the later safety story covering FR42.
- Follow existing ROS package and viewer patterns rather than creating a new fleet-management subsystem for a single battery feature.

### Library/Framework Requirements

- Stay on the repo's current implementation stack:
  - ROS 2 Humble message definitions under `software/edge/src/aeris_msgs`
  - existing `aeris_orchestrator` telemetry publication seams
  - TypeScript parsing in `software/viewer/src/lib/ros/telemetry.ts`
  - React / Next.js viewer components already in `software/viewer/src/components`
  - centralized toast handling via `AlertStack` / `sonner`
- Prefer typed message updates and parser changes over new dependencies.
- If the edge side must interpret MAVLink battery semantics, ground the field mapping in the official MAVLink battery protocol and PX4 battery-status definitions rather than a custom interpretation.

### Project Structure Notes

- Likely ROS-side files to touch:
  - `software/edge/src/aeris_msgs/msg/Telemetry.msg`
  - `software/edge/src/aeris_msgs/CMakeLists.txt` or related package metadata only if the message contract changes require it
  - `software/edge/src/aeris_orchestrator/` telemetry publication code and focused tests
- Likely viewer-side files to touch:
  - `software/viewer/src/lib/ros/telemetry.ts`
  - `software/viewer/src/lib/ros/telemetry.test.mjs`
  - `software/viewer/src/lib/vehicle/VehicleManager.ts`
  - `software/viewer/src/components/sheets/VehicleCard.tsx`
  - `software/viewer/src/components/cards/FleetCard.tsx`
  - `software/viewer/src/app/page.tsx`
  - `software/viewer/src/components/alerts/AlertStack.tsx` only if the current helper surface needs a narrow enhancement for deduped threshold-crossing alerts
- Keep the implementation on the existing operator surface. The separate `/maintenance` route belongs to Story 6.3, not this one.

### References

- [Source: `_bmad-output/planning-artifacts/epics.md#Story-6.2:-Battery-Monitoring-&-Low-Battery-Warnings`]
- [Source: `_bmad-output/planning-artifacts/prd.md#Hardware-Management`]
- [Source: `_bmad-output/planning-artifacts/prd.md#Safety-&-Compliance`]
- [Source: `_bmad-output/planning-artifacts/ux-design-specification.md#Covered-by-Existing-UI`]
- [Source: `_bmad-output/implementation-artifacts/6-1-device-manager-pod-enumeration.md`]
- [Source: `docs/specs/AER-SW-ARCH-001.md#2.1-Edge-First-Autonomy`]
- [Source: `docs/specs/AER-SW-ARCH-001.md#2.2-Safety-Critical-Isolation`]
- [Source: `docs/specs/AER-SW-ARCH-001.md#5.1-In-Vehicle-Transport`]
- [Source: `docs/rosbridge-docker-setup.md`]
- [Source: `software/edge/src/aeris_msgs/msg/Telemetry.msg`]
- [Source: `software/viewer/src/lib/ros/telemetry.ts`]
- [Source: `software/viewer/src/lib/vehicle/VehicleManager.ts`]
- [Source: `software/viewer/src/components/sheets/VehicleCard.tsx`]
- [Source: `software/viewer/src/components/cards/FleetCard.tsx`]
- [Source: `software/viewer/src/components/alerts/AlertStack.tsx`]
- [Source: `software/viewer/src/app/page.tsx`]
- [Source: `software/viewer/src/hooks/useMissionControl.ts`]

## Implementation Model Recommendation

### Primary Model

- GPT-5 / Codex

### Secondary Model (Optional)

- Opus 4.6 for a targeted review pass if the telemetry-contract change and viewer threshold behavior start to sprawl across too many seams

### Why This Choice

- Story 6.2 is a **full-stack mixed** story:
  - ROS message-contract work
  - telemetry publication / parsing alignment
  - viewer threshold and alert behavior
  - focused regression tests across Python/ROS and TypeScript UI seams
- The main risk is contract drift between the edge telemetry source and the viewer presentation logic, not novel algorithm work.
- GPT-5 / Codex is the best primary choice because the implementation will mostly be careful repo-native edits across typed message definitions, existing hooks, and UI surfaces rather than broad external research.

### Testing Requirements

- Add message-level regression coverage proving the battery fields are available through the typed telemetry contract and remain optional-safe when upstream data is missing.
- Add viewer parsing tests for battery percentage, remaining-flight-time fields, and unavailable-value handling.
- Add UI tests that lock the threshold behavior to the story contract:
  - green above 50
  - amber above 25
  - red at or below 25
  - warning trigger default at 25 with no duplicate spam while below threshold
- Add alert-behavior coverage for threshold crossing, repeated below-threshold telemetry, and recovery above threshold.
- Keep automatic RTL tests out of scope here unless a tiny contract shim is unavoidable; full RTL trigger behavior belongs to Story 8.2.

## Latest Technical Information

Verified on **May 23, 2026** from official sources:

- MAVLink's battery protocol documents `BATTERY_STATUS` as the regularly updated battery message and notes that GCS implementations should not rely on `SYS_STATUS` for multi-battery interpretation. It also includes `battery_remaining` and `time_remaining` fields that are directly relevant to this story's percentage and remaining-flight-time requirements.
- PX4's battery-status documentation exposes a `remaining` field plus warning levels such as low, critical, and emergency, which makes it reasonable to preserve an upstream warning state in the telemetry contract if that reduces duplicated threshold logic.
- ROS 2 Humble remains the current repo baseline, so this story should extend the existing Humble message and rosbridge workflow rather than introducing a parallel transport.

Implementation implications for Story 6.2:

- Prefer carrying typed battery fields through the existing telemetry message over inferring battery state from unrelated mission progress data.
- If the vehicle stack can provide remaining battery time, preserve it as a typed telemetry field instead of recomputing a UI guess from percent alone.
- Keep GCS threshold behavior aligned with upstream warning semantics where practical, but preserve a single configurable default threshold of 25% for the operator warning surface.

Sources:

- [MAVLink Battery Protocol](https://mavlink.io/en/services/battery.html)
- [MAVLink Common Message Set](https://mavlink.io/en/messages/common.html)
- [PX4 BatteryStatus](https://docs.px4.io/main/en/msg_docs/BatteryStatus)
- [ROS 2 REP-2000](https://docs.ros.org/independent/api/rep/html/rep-2000.html)

## Project Context Reference

Primary context used for this story:

- `_bmad-output/planning-artifacts/epics.md`
- `_bmad-output/planning-artifacts/prd.md`
- `_bmad-output/planning-artifacts/ux-design-specification.md`
- `_bmad-output/implementation-artifacts/6-1-device-manager-pod-enumeration.md`
- `docs/specs/AER-SW-ARCH-001.md`
- `docs/rosbridge-docker-setup.md`
- `software/edge/src/aeris_msgs/msg/Telemetry.msg`
- `software/viewer/src/lib/ros/telemetry.ts`
- `software/viewer/src/lib/vehicle/VehicleManager.ts`
- `software/viewer/src/components/sheets/VehicleCard.tsx`
- `software/viewer/src/components/cards/FleetCard.tsx`
- `software/viewer/src/components/alerts/AlertStack.tsx`
- `software/viewer/src/app/page.tsx`
- `software/viewer/src/hooks/useMissionControl.ts`

## Dev Agent Record

### Agent Model Used

- GPT-5

### Debug Log References

- Workflow engine: `_bmad/core/tasks/workflow.xml`
- Workflow config: `_bmad/bmm/workflows/4-implementation/dev-story/workflow.yaml`
- Instructions: `_bmad/bmm/workflows/4-implementation/dev-story/instructions.xml`
- Validation: viewer node tests for telemetry parsing, battery thresholds, alert dedupe, and remaining-flight-time fallback; `npm run lint`; `npm run build`; `python3 -m pytest -q software/edge/src/aeris_msgs/test/test_telemetry_contract.py`

### Completion Notes List

- Extended `aeris_msgs/Telemetry` with typed `battery_percent`, `remaining_flight_time_sec`, and `remaining_flight_time_available` fields so the rosbridge path has an authoritative battery contract instead of a viewer-only extension.
- Added a shared viewer battery seam for threshold evaluation, remaining-flight-time formatting, and edge-triggered low-battery alert transitions; `VehicleCard`, `FleetCard`, PiP, and page alert logic now all align on `> 50`, `> 25`, and `<= 25`.
- Updated the vehicle telemetry parser, `VehicleManager`, and simulated rosbridge telemetry publisher so remaining-flight-time availability flows through `/vehicle/telemetry` without reusing mission ETA.
- Added focused regression coverage for the message contract, telemetry parsing, shared threshold logic, alert dedupe, and vehicle-card fallback rendering.
- Validation on this machine: `node --test src/lib/ros/telemetry.test.mjs src/lib/batteryMonitoring.test.mjs src/app/icViewModeSurface.test.mjs`, `npm run lint -- src/app/page.tsx src/components/cards/FleetCard.tsx src/components/pip/PiPVideoFeed.tsx src/components/sheets/VehicleCard.tsx src/lib/batteryMonitoring.ts src/lib/ros/telemetry.ts src/lib/vehicle/VehicleManager.ts src/types/vehicle.ts`, `npm run build`, and `python3 -m pytest -q software/edge/src/aeris_msgs/test/test_telemetry_contract.py`.
- Host verification does not have `colcon` or `ros2`, so ROS interface generation and rosbridge runtime validation still need the Humble container/workspace path.

## Change Log

- 2026-05-23: Added typed battery telemetry fields, shared threshold logic, low-battery alert dedupe, vehicle remaining-flight-time fallback rendering, and focused parser/message tests.

### File List

- `_bmad-output/implementation-artifacts/6-2-battery-monitoring-and-low-battery-warnings.md`
- `_bmad-output/implementation-artifacts/sprint-status.yaml`
- `software/edge/src/aeris_msgs/msg/Telemetry.msg`
- `software/edge/src/aeris_msgs/test/test_telemetry_contract.py`
- `software/viewer/src/app/icViewModeSurface.test.mjs`
- `software/viewer/src/app/page.tsx`
- `software/viewer/src/components/cards/FleetCard.tsx`
- `software/viewer/src/components/pip/PiPVideoFeed.tsx`
- `software/viewer/src/components/sheets/VehicleCard.tsx`
- `software/viewer/src/lib/batteryMonitoring.test.mjs`
- `software/viewer/src/lib/batteryMonitoring.ts`
- `software/viewer/src/lib/ros/telemetry.test.mjs`
- `software/viewer/src/lib/ros/telemetry.ts`
- `software/viewer/src/lib/vehicle/VehicleManager.ts`
- `software/viewer/src/types/vehicle.ts`
- `tools/sim/publish_telemetry.py`
