# Edge Tools

This directory contains diagnostic and benchmarking tools for the Aeris edge compute platform. These tools are used for performance validation, network stress testing, and timing synchronization verification.

## Available Tools

| Tool | Purpose | ROS 2 Required |
|------|---------|----------------|
| `bench_bootstrap.sh` | Network and PTP time-sync setup | No |
| `dds_flood_tester.py` | DDS transport stress testing | Yes |
| `first_tile_timer.py` | Map tile latency benchmarking | Yes |
| `ptp_status.py` | PTP synchronization status | No |
| `tile_latency_probe.py` | End-to-end tile retrieval profiling | Yes |

## bench_bootstrap.sh

Prepares a Jetson bench rig (or dry-run inside Docker) with the network/time-sync tuning used in Phase 0.

### Prerequisites
- Linux system with `sysctl` support
- PTP-capable network interface (for hardware timestamping)
- Root privileges (on physical hardware)

### Usage

```bash
cd /workspace/aeris            # inside the ROS 2 docker container or on Jetson

# Jetson hardware (needs sudo/root capabilities)
sudo ./software/edge/tools/bench_bootstrap.sh --ptp-interface eth0

# Docker/dev laptop (no privileged access) — only print commands
./software/edge/tools/bench_bootstrap.sh --dry-run --ptp-interface eth0
```

### Options

| Option | Description |
|--------|-------------|
| `--dry-run` | Print commands without executing (use inside Docker where `sysctl`, `ptp4l`, and `phc2sys` cannot run) |
| `--ptp-interface <iface>` | Override the NIC passed to `ptp4l` |
| `PTP_INTERFACE=<iface>` | Environment variable alternative to `--ptp-interface` |

### What the Script Does

1. Applies `software/edge/config/dds/sysctl_overlay.conf` via `sysctl -p`
2. Starts `ptp4l` using `software/edge/config/ptp/ptp4l_gm.cfg` (logs to `/tmp/aeris_ptp4l.log`)
3. Starts `phc2sys` using `software/edge/config/ptp/phc2sys.cfg` for sub-microsecond sync
4. Prints environment exports for CycloneDDS (`CYCLONEDDS_URI`) and Fast DDS (`FASTRTPS_DEFAULT_PROFILES_FILE`)
5. Echoes the TODO hook where the camera bring-up command will live once the final sensor is selected

### Running as PTP Slave

To run as a PTP slave instead of grandmaster:

```bash
sudo ptp4l -f software/edge/config/ptp/ptp4l_slave.cfg -i <iface>
```

> **Note:** Run this script as root (or with sudo) on actual Jetson hardware. Inside Docker, use `--dry-run` to verify commands without attempting privileged operations.

## Python Tools

All Python tools support `--help` for detailed usage information.

### Quick Reference

```bash
# DDS stress test at 1KHz with 512-byte payloads
ros2 run aeris_tools dds_flood_tester --rate 1000 --payload-bytes 512

# Measure first tile latency with custom timeout
ros2 run aeris_tools first_tile_timer --timeout-sec 300

# Check PTP synchronization status
python3 software/edge/tools/ptp_status.py --tail 200

# Profile tile retrieval latency
ros2 run aeris_tools tile_latency_probe --samples 20 --timeout-sec 60
```

See individual tool docstrings for complete documentation.
