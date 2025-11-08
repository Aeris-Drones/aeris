# Transport Validation Checklist

## Configuration Artifacts
- `software/edge/config/dds/cyclonedds.xml`: Default ROS 2 QoS for CycloneDDS with tuned buffers and topic-specific policies.
- `software/edge/config/dds/fastdds.xml`: Matching FastDDS participant/publisher/subscriber profiles plus transport descriptors.
- `software/edge/config/dds/sysctl_overlay.conf`: Socket buffer and congestion-control overrides applied on Jetson boots (`sysctl -p`).
- `software/edge/config/srt/abr_ladder.yaml`: SRT ABR ladder shared between map/video encoders and mesh agents.
- `software/edge/config/ptp/*.cfg`: PTP grandmaster/slave and phc2sys templates for GNSS-disciplined nodes.

## Applying the Configs
1. Copy the desired DDS XML to `/etc/ros/dds/cyclonedds.xml` or `/etc/ros/dds/fastdds.xml` and set `RMW_IMPLEMENTATION` accordingly.
2. Load socket tuning: `sudo sysctl -p software/edge/config/dds/sysctl_overlay.conf`.
3. Launch PTP:
   ```bash
   sudo ptp4l -f software/edge/config/ptp/ptp4l_gm.cfg -i eth0
   sudo phc2sys -f software/edge/config/ptp/phc2sys.cfg
   ```
   Swarm followers use `ptp4l_slave.cfg`.
4. Mesh/video nodes read the SRT ladder from `config/srt/abr_ladder.yaml` (see mesh agent README once implemented).

## DDS Flood Test
1. Build the workspace in Docker (`colcon build ...`), source `install/setup.bash`.
2. Start the high-rate publisher:
   ```bash
   ros2 run aeris_orchestrator heartbeat  # optional background load
   ros2 run aeris_map map_tile_publisher  # optional
   python3 software/edge/tools/dds_flood_tester.py --topic map/tiles --rate 800 --payload-bytes 2048
   ```
3. In another shell, monitor latency:
   ```bash
   ros2 topic hz map/tiles
   ros2 topic echo map/tiles | ts '%H:%M:%.S'
   ```
4. Capture DDS logs (`export CYCLONEDDS_URI=file://.../cyclonedds.xml`) and note packet drops. Adjust buffer sizes or QoS depths if `ros2 topic hz` deviates >5%.
