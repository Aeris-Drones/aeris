**1) Clarify the requirements (from specs and newest evidence)**

Edge‑first autonomy:  
All mission‑critical mapping, perception, mission logic run on the companion computer. *Orchestrator, Map, Perception, Mesh Agent, Device Manager, Evidence Service run on‑vehicle as isolated containers.*  
GCS remains thin; swarm must work fully offline.  
Time sync by GNSS‑disciplined PTP; degraded mode via mesh fallback. Fusion gating at ~2 ms skew.

Hardware standardized: Jetson Orin NX (Scout, down-clocked), Jetson AGX Orin (Ranger). *Optional Hailo-8 if mission needs change.*

Safety isolation:  
FMU (PX4 on RTOS, BSD-3-Clause) remains the final authority; Aeris OS issues advisory setpoints. FMU can refuse/modify for safety. Companion↔FMU link via MAVSDK.

KPIs and NFRs:  
TTFM < 3:00, first tile < 90 s, plume edge error < 25 m, ≥ 1 km² in 10 min (2 scouts), ≥ 99% control/telemetry reliability ≤ 2 hops, edge→GCS tile latency ≤ 2 s (95p), evidence export < 60 s.

Transport:  
Intra‑vehicle: ROS 2 + tuned DDS (Cyclone/FastDDS), colcon workspace, robust socket buffers.  
Inter‑vehicle: Control/Telemetry by gRPC/HTTP3 (QUIC; HTTP/2 if not mature); Video by SRT, MBTiles by HTTP3. DSCP tags for QoS: Control > Telemetry > Video > Bulk.

Pods:  
1‑Wire/EEPROM ID, soft-start (<50ms), attach→ready < 15 s, capability registry.

Time sync:  
linuxptp, GNSS grandmaster, fusion gating by skew.

Security/compliance:  
Evidence signed by Ed25519 + P-256 (fTPM + secure element), privacy export defaults (face/license plate redaction), SPDX headers and SBOM scan on every CI run.

***

**2) Explore repository strategies (with new rationale)**

A) Strict monorepo

**Pros:**

- Atomic changes across edge services, messages, CI, and docs.
- Easier global refactoring, consistent standards.
- Trunk‑based dev proven at extreme scale (Google, recent ACM study).
  
**Cons:**

- Build/test speed becomes a CI concern as repo (& sim assets) grow.
- Coarser access control.
- Needs strong caching and partial builds.

**Fit for Aeris?**
Good for integration, but now we know hardware/sim assets are likely to grow fast.  
*Hybrid monorepo recommended for scalable isolation.*

***

B) Polyrepo (many repos)

**Pros:**

- Fine boundaries, per-repo visibility, easier open-sourcing of selective parts.

**Cons:**

- Cross-component drift, heavy dependency choreography.
- System-wide refactors are slow, KPIs harder to enforce.

**Fit?**
Overhead outweighs benefits; cross-service latency/format guarantees matter for swarm reliability.

***

C) Hybrid modular monorepo (**recommended**)

**One primary repo for all Aeris‑owned code, docs, tests, and configs—**with clear module boundaries inside.  
Heavy external dependencies integrated as pinned submodules or container layers (rtabmap, lio‑sam, Kimera-VIO, OpenVINS—for R&D only, containerized to isolate GPL‑3).

Promote modules to separate repos only when stabilized and open-source partners require it.

This matches large robotics stacks’ approach (ROS 2 colcon workspaces, multi-package), keeps batch refactors easy, and preserves offline reproducibility (pinned tags, container images).

***

**3) Reuse vs. reinvention — subsystem decisions (with licenses and deep-research updates)**

**Goal:** Prefer permissive licenses, build custom only for unique IP or team-specific integration glue. When using copyleft (GPL‑3) for R&D, containerize to keep main repo BSD-compliant.

| Subsystem  | Recommendation & Updated Evidence         | Rationale & Citations     |
|------------|------------------------------------------|---------------------------|
| Flight stack (FMU) | Reuse PX4 (BSD-3-Clause, primary); ArduPilot (GPL-3, alt, OK since FMU is isolated) | BSD leaves options open, best safety isolation; PX4 strong community [PX4 Docs] |
| Companion↔FMU bridge | MAVSDK (BSD) for offboard setpoints & mission upload | Stable, offboard semantics, Python/C++; matches ROS2 [mavsdk.mavlink.io] |
| Intra-vehicle bus | ROS 2 (CycloneDDS/FastDDS), tuned QoS XML | Native to robotics, colcon workspace, proven with updated sysctl/DDs settings [ROS Docs] |
| VIO frontend | Kimera-VIO (BSD, DEFAULT, per latest research); OpenVINS (GPL-3, R&D only, containerized) | Kimera-VIO avoids copyleft; OpenVINS accuracy tested, but containerized per compliance [GitHub] |
| SLAM | RTAB-Map (BSD-3); LIO-SAM (BSD-3, LiDAR role) | Robust, ROS 2, latest mapping tests; BSD clear [GitHub] |
| Thermal pipeline | Aeris Perception (C++/Python) on OpenCV/TensorRT; model zoo option; privacy pipeline | GStreamer/NVVENC for ingest; custom cueing & thresholds per ThermalHotspot KPIs [Google Groups] |
| Acoustic bearings | MVDR+GCC-PHAT (Python→C++ node) | Real-time constraints, Python prototyping, C++ node for production [general DSP] |
| Gas plume | Kernel DM+V/W in Aeris Perception, modular isolation | Bespoke API, little OSS code in permissive license for fusion; modular for team innovation |
| Evidence/signing | Ed25519 + P-256 defaults, SHA‑256 hashing | TPM2 + fTPM, flexible for attestation; containerized CLI [spdx.dev] |
| Simulation | Gazebo (Ignition) with ROS bridge, PX4 SITL | Enables system-level testing and HIL runs; aligns with hardware [Microsoft Learn] |
| Logging | MCAP across all telemetry/sensor logs | Foxglove integration, rapid viz, cross-language support [foxglove.dev] |
| GCS | Thin web stack (React/MapLibre/deck.gl) for MBTiles overlays; keep QGC separate for flight tuning | Custom hazard UX, QGC (GPL-3) used only in tuning flows, not embedded [QGroundControl Docs] |
| Mesh/Transport | Babel mesh (WMM/DSCP), BATMAN-adv backup, gRPC/HTTP3 for ctrl/tel, SRT for video, MBTiles over HTTP3 | Updated to match UAV test evidence—diversity, QoS, evidence [kmcd.dev] |
| Time sync | linuxptp (ptp4l/phc2sys) configs per role | Standard PTPv2, GNSS master, fusion gate enforced per timestamp | 
| Pods/Device Manager | 1‑Wire/EEPROM ID, power budgeting, capability registry; <15s attach | Modular, matches rapid pod-fault recovery; updated per research |

**License cautions:** OpenVINS and VINS-Fusion (GPL-3) containerized in third_party for R&D; primary VIO stack is now Kimera-VIO (BSD) by default.

***

**4) Scalable repository & workspace layout (hybrid modular monorepo, with updated boundaries/configs)**

```plaintext
aeris/
├─ docs/                         # MkDocs + Material; ADRs; diagrams-as-code
│  ├─ adr/                       # Architecture Decision Records
│  ├─ specs/                     # rendered SW architecture, ICDs
│  ├─ research/                  # latest evidence packs, matrix CSVs
│  └─ ops/                       # runbooks, safety cases
├─ software/
│  ├─ edge/
│  │  ├─ src/
│  │  │  ├─ aeris_msgs/          # MapTile, ThermalHotspot, AcousticBearing, GasIsopleth
│  │  │  ├─ aeris_orchestrator/  # rclpy, heartbeat/setpoints
│  │  │  ├─ aeris_map/           # rclcpp, RTAB-Map/LIO-SAM fusion, MBTiles
│  │  │  ├─ aeris_perception/    # C++/Python nodes, thermal, acoustic, gas
│  │  │  ├─ aeris_mesh_agent/    # Rust/C++, Babel, SRT sender, ABR
│  │  │  ├─ aeris_device_manager/# C++, pod registry/hot-swap, 1-Wire/EEPROM
│  │  │  └─ aeris_evidence/      # Rust/Go/Python, bundle, TPM signing
│  │  ├─ config/
│  │  │  ├─ dds/                 # Cyclone/Fast DDS XML profiles, sysctl overlays
│  │  │  ├─ mesh/                # Babel/BATMAN configs, WMM/DSCP
│  │  │  ├─ srt/                 # ABR ladder YAML, updated rates
│  │  │  ├─ ptp/                 # ptp4l configs per role
│  │  │  └─ mapping/             # RTAB-Map, LIO-SAM params
│  │  ├─ third_party/            # pinned tags: rtabmap, lio-sam, kimera-vio, openvins (R&D)
│  │  ├─ launch/                 # per-play launch files
│  │  └─ tools/                  # MCAP utilities, latency harness, pipeline
│  ├─ gcs/                       # web UI + MBTiles overlays
│  ├─ sim/                       # Gazebo models, PX4 SITL scripts
│  └─ infra/                     # Containerfiles, devcontainer
├─ hardware/
│  ├─ fmu/                       # PX4/ArduPilot params, submodules
│  ├─ pods/                      # pod ICDs, power budgets, EEPROM
│  └─ mech/                      # CAD exports
├─ test/
│  ├─ scenarios/                 # MCAP/rosbag datasets
│  ├─ hitl/                      # HW-in-the-loop
│  └─ e2e/                       # system tests for KPIs, NFR traceability
├─ ci/
│  ├─ github/                    # GH Actions workflows
│  └─ scripts/                   # build/test/release, SBOM/licensing scan
├─ SECURITY.md  LICENSE  CODEOWNERS  CONTRIBUTING.md
└─ mkdocs.yml  .pre-commit-config.yaml  .editorconfig  .clang-format
```

**Why this works for Aeris:**
- Exact module boundaries, real-world evidence, license isolation,
- ROS2 workspace enables rapid evolution/partial rebuilds,
- Pinning submodules ensures reproducible, offline edge builds,
- Every field-proven KPI/test/threshold mapped to explicit folder.

***

**5) Build, packaging & runtime model (updated with all new evidence)**

**Languages:**  
C++20 for performance-critical nodes; Python 3.10+ for Orchestrator/tools; Rust or C++ for Mesh Agent.

**Build tools:**  
ROS 2 + colcon for edge; CMake (C++), setuptools/uv (Python).

**Containers:**  
Rootless Podman per service (read-only, seccomp), systemd user units for Jetson startup; container images signed.

**Transport:**  
CycloneDDS/FastDDS with updated XML and sysctl overlays; Babel mesh with WMM/DSCP; gRPC/HTTP3 (fallback HTTP/2); SRT with new ABR ladder YAML.

**Time sync:**  
linuxptp configs templated by role (GNSS grandmaster, mesh slaves), offset metrics fed to fusion gates.

**Logging:**  
MCAP for all channels; Foxglove compatibility; bag→MCAP tool in repo.

**Evidence:**  
Manifest with SHA‑256; Ed25519/P-256 signatures; face/license plate redaction defaults; CLI & pipeline.

***

**6) Interface definitions (using all PLAN 2 updates)**

**ROS 2 messages (aeris_msgs/):**
- MapTile.msg
- ThermalHotspot.msg
- AcousticBearing.msg
- GasIsopleth.msg

**gRPC protos (cross-vehicle ctrl/telemetry):**
`software/edge/proto/aeris/mesh/api/v1/control.proto`

**Service Control**
```proto
rpc StartPlay(StartPlayRequest) returns (StartPlayAck)
rpc SetWaypoints(WaypointSet) returns (Ack)
rpc GetLinkStats(LinkStatsReq) returns (LinkStats)
```

***

**7) CI/CD, quality gates, & supply chain (reflecting all updated checks and tools)**

**GitHub Actions:**
Matrix builds (x86_64 + Jetson cross), colcon/cache, container build+sign, clang-tidy/format, pytest, ruff, mypy, license scanning (ScanCode, SBOM), SOPS-encrypted secrets, protected main, review gates, trunk-based branching.

**Versioning:**  
SemVer per service image, repo tags for system release.

**Secrets:**  
SOPS-encrypted, TPM-backed keys.

**Artifacts:**  
Container images pushed to registry, debug/symbols for crash analysis.

***

**8) Documentation & onboarding (with new researcher-driven features)**

Docs-as-code (MkDocs Material), ADR flow, auto-diagrams (Mermaid/PlantUML), CONTRIBUTING.md + CODEOWNERS, issue templates, pre-commit (format, SPDX licensing).  
Onboarding script: `./tools/dev/bootstrap.sh` sets up devcontainer/toolchain, pulls submodules, caches datasets.

**Build/Run:**
```bash
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
ros2 run aeris_orchestrator orchestrator_node
```
**Containers:**
```bash
podman build -t ghcr.io/aeris/aeris_orchestrator:dev -f Containerfile .
```

***

**10) Package templates (see PLAN 1, swapped for new repo names and fields as per PLAN 2)**

*C++ (`ament_cmake`):*
```xml
<package format="3">
  <name>aeris_map</name>
  <version>0.1.0</version>
  <description>SLAM fusion + MBTiles</description>
  <maintainer email="dev@aeris.local">Aeris</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>
  <depend>aeris_msgs</depend>
  <export><build_type>ament_cmake</build_type></export>
</package>
```

*Python (`rclpy`):*
```ini
[options]
packages = find:
install_requires = rclpy
[options.entry_points]
console_scripts =
  orchestrator_node = aeris_orchestrator.main:run
```

***

**11) Security, privacy & compliance (now fully aligned to PLAN 2)**

- Least-privilege containers, read-only root, SOPS/secrets, device-plugin ACLs only for required hardware.
- Evidence CLI/service defaults to redaction (face/license-plate blur); policy documented.
- SPDX headers + ScanCode/REUSE in CI; SBOMs released on tags.
- All container builds cryptographically signed.

***

**12) Final recommendation**

**Adopt the Hybrid Modular Monorepo**:  
Preserves tight real‑time interface coherency across Orchestrator/Map/Perception/Mesh, enables atomic schema changes, supports reproducible offline builds (pinned submodules, containers), and aligns with your comms, PTP sync, and containerized service architecture.

Monorepo scales if you apply trunk‑based dev and strong CI.  
Hybrid gives easy future splits—now with updated GPL isolation, research‑backed mesh choices, and new onboarding/scripts.

***