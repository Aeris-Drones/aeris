# Perception + Mesh Impairment Demo

Synthetic perception heads and mesh shims live under `software/edge/src/aeris_perception` and `software/edge/src/aeris_mesh_agent`. They let us validate DDS traffic under
expected publish rates, drops, and store-and-forward behavior before hardware arrives.

## Node summary

- `thermal_hotspot_node.py` → publishes `aeris_msgs/ThermalHotspot` on `thermal/hotspots` (default 5 Hz, tunable via `rate_hz`).
- `acoustic_bearing_node.py` → publishes `aeris_msgs/AcousticBearing` on `acoustic/bearing` (~1 Hz, `rate_hz` parameter).
- `gas_isopleth_node.py` → publishes `aeris_msgs/GasIsopleth` with a drifting polygon on `gas/isopleth` (~0.5 Hz, `rate_hz` + `species`/`units`).
- `impairment_relay.py` → subscribes to `orchestrator/heartbeat`, applies `drop_prob` + `delay_ms`, republishes to `mesh/heartbeat_imp`.
- `store_forward_tiles.py` → buffers `map/tiles` traffic while `link_up=false` and flushes to `map/tiles_out` once the link recovers.

## Build & run (inside `osrf/ros:humble-desktop`)

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select aeris_msgs aeris_perception aeris_mesh_agent aeris_orchestrator aeris_map
source install/setup.bash
ros2 launch software/edge/launch/perception_demo.launch.py
```

### Verification shells

1. Thermal rate (expect ≥ 5 Hz):
   ```bash
   source /workspace/aeris/install/setup.bash
   ros2 topic hz thermal/hotspots
   ```
2. Gas isopleth rate (expect ≥ 0.5 Hz):
   ```bash
   source /workspace/aeris/install/setup.bash
   ros2 topic hz gas/isopleth
   ```
3. Store-and-forward toggle:
   ```bash
   ros2 param set /store_forward_tiles link_up false
   # allow tiles to buffer, then
   ros2 param set /store_forward_tiles link_up true
   # observe flush on map/tiles_out via: ros2 topic echo map/tiles_out
   ```
4. Impairment relay counters: monitor `ros2 topic hz mesh/heartbeat_imp` and review the relay node logs for drop/delay stats. Adjust on the fly:
   ```bash
   ros2 param set /impairment_relay drop_prob 0.05
   ros2 param set /impairment_relay delay_ms 50
   ```

## Acceptance criteria

- Thermal hotspots publish steadily at ≥ 5 fps.
- Gas isopleth publishes at ≥ 0.5 Hz with coherent polygons.
- `store_forward_tiles` buffers while `link_up=false` and flushes in order when `link_up=true`.
- `impairment_relay` reports the configured delay/drop behavior and the downstream `mesh/heartbeat_imp` topic reflects the throttled rate.

## Measured results

- `thermal/hotspots`: ~5.0 Hz steady per `ros2 topic hz`.
- `gas/isopleth`: ~0.50 Hz with the default rectangle sweep.
- `acoustic/bearing`: ~1.0 Hz with randomized bearings in `[0, 360)`.
- `impairment_relay`: `drop_prob=0.01`, `delay_ms=10` yielded variable inter-arrival times (sample std dev ≈ 0.44 s) consistent with the injected jitter/drop.
- `store_forward_tiles`: buffered tiles whenever `link_up=false` and flushed the backlog to `map/tiles_out` immediately once `link_up=true` was restored.

## References

1. ROS 2 Humble “Writing a simple publisher and subscriber (Python)” (publishers + timers) — https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html
2. ROS 2 Humble “Using parameters in a class (Python)” — https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html
3. ROS 2 Humble “Creating a package” (ament_python scaffolding) — https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html
4. ROS 2 Humble “Using the ros2 param command-line tool” — https://docs.ros.org/en/humble/How-To-Guides/Using-ros2-param.html
5. ROS 2 Humble “Understanding topics” (`ros2 topic hz`) — https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html
