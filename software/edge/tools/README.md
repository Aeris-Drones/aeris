# Edge Tools

## bench_bootstrap.sh
Prepares a Jetson bench rig (or dry-run inside Docker) with the network/time-sync tuning used in Phase 0.

### Usage
```
cd /workspace/aeris            # inside the ROS 2 docker container or on Jetson
# Jetson hardware (needs sudo/root capabilities)
sudo ./software/edge/tools/bench_bootstrap.sh --ptp-interface eth0
# Docker/dev laptop (no privileged access) — only print commands
./software/edge/tools/bench_bootstrap.sh --dry-run --ptp-interface eth0
```
Options:
- `--dry-run`: print commands without executing (use this inside Docker where `sysctl`, `ptp4l`, and `phc2sys` cannot run).
- `--ptp-interface <iface>` or `PTP_INTERFACE=<iface>`: override the NIC passed to `ptp4l`.

The script performs:
1. Applies `software/edge/config/dds/sysctl_overlay.conf` via `sysctl -p`.
2. Starts `ptp4l` using `software/edge/config/ptp/ptp4l_gm.cfg` (logs to `/tmp/aeris_ptp4l.log`) and `phc2sys` using `software/edge/config/ptp/phc2sys.cfg`, so the bench hits the ≤ 2 µs skew budget.
3. Prints environment exports for CycloneDDS (`CYCLONEDDS_URI`) and Fast DDS (`FASTRTPS_DEFAULT_PROFILES_FILE`).
4. Echoes the TODO hook where the camera bring-up command will live once the final sensor is selected.

If you need this rig to run as a PTP slave instead of grandmaster, run the commented command the script prints:
```
sudo ptp4l -f software/edge/config/ptp/ptp4l_slave.cfg -i <iface>
```

> **Note:** Run this script as root (or with sudo) on actual Jetson hardware. Inside Docker, use `--dry-run` to verify commands without attempting privileged operations.
