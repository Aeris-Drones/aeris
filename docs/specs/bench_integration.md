# Bench Integration + Evidence Runbook

This guide walks through preparing the Jetson Orin Nano bench rig, verifying time-sync, running the Aeris nodes, and producing an evidence bundle for review.

## Hardware reminders

- Jetson Orin Nano lacks NVENC, so preview video must remain software-encoded; focus on `/map/tiles` MBTiles output plus low-FPS previews.
- CSI lane budget: CAM1 exposes 4 lanes (or dual x2), CAM0 exposes 2 lanes—pick image sensors accordingly.
- Bench and simulation harnesses must keep clock skew ≤ 2 µs by running `ptp4l`/`phc2sys` with the configs under `software/edge/config/ptp/`.

## Bench bootstrap

1. SSH into the Jetson and elevate (`sudo -s`) so sysctl and linuxptp daemons can start.
2. Apply transport + PTP settings (runs sysctl, `ptp4l`, `phc2sys`):
   ```bash
   sudo software/edge/tools/bench_bootstrap.sh --ptp-interface <iface>
   ```
   - `ptp4l`/`phc2sys` invocation and offset monitoring follow the linuxptp guidance for boundary clocks (`ptp4l(8)`/`phc2sys(8)`).
3. Check sync status using the helper (falls back to `phc2sys -m` when logs are missing):
   ```bash
   python3 software/edge/tools/ptp_status.py
   ```
   Expect offsets within a few microseconds per the linuxptp docs once `CLOCK_REALTIME` locks to the NIC PHC.

## Running Aeris nodes on the bench

1. Start the ROS 2 Humble container (same command as the dev workflow) and bind `/workspace/aeris`.
2. Build + source the workspace slice:
   ```bash
   source /opt/ros/humble/setup.bash
   colcon build --packages-select aeris_msgs aeris_map aeris_orchestrator aeris_perception aeris_mesh_agent
   source install/setup.bash
   ```
3. Launch the desired nodes:
   - Mapping baseline: `ros2 run aeris_map map_tile_publisher` and `ros2 run aeris_orchestrator heartbeat`.
   - Perception/mesh demo (optional): `ros2 launch software/edge/launch/perception_demo.launch.py`.
4. Measure time-to-first-tile (acceptance keeps it < 90 s):
   ```bash
   python3 software/edge/tools/first_tile_timer.py --timeout-sec 120 & sleep 0.3
   ros2 run aeris_map map_tile_publisher
   ```

## Evidence bundle workflow

1. Identify directories with logs/config snapshots (e.g., `/var/log/ptp4l/`, `/workspace/aeris/logs/`).

2. **Running evidence bundler:**

   **Preferred** (after `colcon build --packages-select aeris_evidence` and re-sourcing `install/setup.bash`):
   ```bash
   ros2 run aeris_evidence evidence_bundle -- --input-dir /var/log/ptp4l \
     --input-dir /workspace/aeris/logs --out /tmp/aeris_evidence.tgz --window-sec 900
   ```

   **Fallback** (symlink-install or not built):
   ```bash
   PYTHONPATH=software/edge/src/aeris_evidence python3 -m aeris_evidence.evidence_bundle \
     --input-dir /var/log/ptp4l --input-dir /workspace/aeris/logs \
     --out /tmp/aeris_evidence.tgz --window-sec 900
   ```

3. **Optional Ed25519 signing:**
   ```bash
   ros2 run aeris_evidence evidence_bundle -- --input-dir /var/log/ptp4l \
     --out /tmp/aeris_evidence.tgz --signing-key /secure/keys/aeris_ed25519.key
   ```
   - Keys follow the PyNaCl Ed25519 format (32-byte seed); the manifest is signed per the PyNaCl digital-signature workflow and saved as `manifest.sig`.
   - Signing is optional; install PyNaCl to test Ed25519 signing, otherwise the CLI degrades gracefully.

4. Bundle contents:
   - `manifest.json` lists each file's original path, tar `arcname`, byte size, and SHA-256 digest (mirrors the file/digest pairing recommended by The Update Framework spec). Hashes are computed with Python's `hashlib.sha256`.
   - The `.tgz` also contains every matching file from the provided directories. Use `--manifest-only` to emit just the manifest (plus optional signature) for quick checks.

5. Output confirms file count and byte total. If no files match the sliding window, the CLI exits non-zero so you can widen `--window-sec`.

## Operational tips

- linuxptp monitoring: `ptp_status.py` reads `/tmp/aeris_phc2sys.log` (created by the bootstrap script) and reports the latest offset/state, matching the `phc2sys -m` output described in the linuxptp manual.
- CPU pinning: for noisy benches, pin publishers with `taskset -c 2 ros2 run aeris_map map_tile_publisher` and promote them with `sudo chrt -f 80 <cmd>` as documented in the `taskset(1)`/`chrt(1)` man pages.
- Keep `ptp4l`/`phc2sys` logs under `/tmp` or `/var/log` so the evidence bundle can scoop them up without extra privileges.

## Acceptance checklist

- `bench_bootstrap.sh` runs without errors on the Jetson, applying sysctl tweaks and launching linuxptp daemons.
- `ptp_status.py` reports a plausible offset (≤ 2 µs target) from the phc2sys log or one-shot query.
- `first_tile_timer.py` observes the first `/map/tiles` message in < 90 s while `map_tile_publisher` runs on the bench rig.
- `evidence_bundle` creates `/tmp/aeris_evidence.tgz` with `manifest.json` (plus `manifest.sig` when a key is provided) enumerating SHA-256 hashes for every captured file.

## References

1. linuxptp `ptp4l` manual — https://linuxptp.nwtime.org/documentation/ptp4l/
2. linuxptp `phc2sys` manual — https://linuxptp.nwtime.org/documentation/phc2sys/
3. PyNaCl digital signatures (Ed25519) — https://pynacl.readthedocs.io/en/latest/signing/
4. Python `hashlib` (SHA-256 usage) — https://docs.python.org/3/library/hashlib.html
5. The Update Framework spec (file/digest manifest pattern) — https://theupdateframework.github.io/specification/latest/
6. `taskset(1)` CPU affinity man page — https://man7.org/linux/man-pages/man1/taskset.1.html
7. `chrt(1)` real-time priority man page — https://man7.org/linux/man-pages/man1/chrt.1.html
