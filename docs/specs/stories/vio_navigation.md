# VIO Navigation Runbook

## Goal

Run mission progression in GPS-denied mode using VIO odometry as the Orchestrator position source, then validate drift metrics over a 10-minute profile.

## 1. Build and Source

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select aeris_msgs aeris_map aeris_orchestrator --symlink-install
source install/setup.bash
```

## 2. Launch GPS-Denied Simulation Profile

```bash
ros2 launch software/sim/launch/multi_drone_sim.launch.py \
  world:=software/sim/worlds/disaster_scene.sdf \
  scout_model_name:=scout1 \
  vehicles_config:=software/sim/config/multi_drone_gps_denied.json \
  gps_denied_mode:=true \
  position_source_mode:=vio_odometry \
  launch_orchestrator:=true
```

`launch_orchestrator:=true` starts `aeris_orchestrator mission` with matching
`gps_denied_mode` and `navigation_position_source` runtime parameters.

## 3. Launch OpenVINS + RTAB-Map

```bash
ros2 launch aeris_map rtabmap_vio_sim.launch.py
```

## 4. Verify Runtime Position Source

The progress payload should report VIO mode and per-vehicle source selection:

```bash
ros2 topic echo /mission/progress --once
```

Expected keys:
- `positionSourceMode: "vio_odometry"`
- `vehiclePositionSources.scout_1` starts with `vio_odometry`

## 5. Trigger VIO Return-to-Launch Scenario

1. Start a mission in the viewer and allow the scout to traverse a non-trivial path.
2. Trigger recall:

```bash
ros2 service call /vehicle_command aeris_msgs/srv/VehicleCommand \
  "{command: RECALL, vehicle_id: scout1, mission_id: <ACTIVE_MISSION_ID>}"
```

3. Validate canonical return telemetry:

```bash
ros2 topic echo /mission/progress --once
```

Expected `returnTrajectory` fields:
- `vehicleId`
- `state`
- `points`
- `etaSec`
- `lastUpdatedSec`
- optional `fallbackReason` when freshness or synthesis fails

Pass/fail thresholds for this story:
- launch-point horizontal error `<= 2.0 m`
- return trajectory update latency `<= 1.0 s`
- fallback reason present whenever freshness/synthesis fails

## 6. Run VIO Drift Validation (10 Minutes)

```bash
python3 software/sim/tools/validate_vio_navigation_drift.py \
  --profile software/sim/config/vio_navigation_profile.json \
  --run-id latest
```

Artifacts:
- `output/vio_navigation_drift/latest/drift_report.json`
- `output/vio_navigation_drift/latest/drift_samples.csv`
- `output/vio_navigation_drift/latest/drift_summary.txt`

## 7. Optional Smoke Harness

When the mission node is running in VIO mode, the smoke script can assert source selection:

```bash
EXPECTED_POSITION_SOURCE=vio_odometry \
python3 software/edge/src/aeris_orchestrator/test/sitl_tracking_smoke.py
```

For return-to-launch specific checks (canonical `returnTrajectory`, launch error, update latency):

```bash
python3 software/edge/src/aeris_orchestrator/test/sitl_vio_return_smoke.py
```
