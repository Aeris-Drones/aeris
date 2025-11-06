# **AER-SW-ARCH-001 — Software Architecture (v0.1)**

**Owner (A):** CTO  
**Author (R):** CTO  
**Consulted (C):** CSA, ChEng   
**Date**: 2025-10-18

**Scope:** This document defines the end-to-end software architecture for the Aeris system, including on-vehicle services (Aeris OS), inter-vehicle communication, and its relationship with the Ground Control Station (GCS). It translates requirements from `AER-REQ-001` into an implementable design that delivers a live, fused hazard map in minutes.

## **0\. Document Control**

### **0.1 Purpose**

To define the end-to-end software architecture for the Aeris system, including on-vehicle services (Aeris OS), inter-vehicle communication, and the relationship with the Ground Control Station (GCS). This document maps requirements from `AER-REQ-001` to concrete design choices, performance budgets, and testable interfaces.

### **0.2 Change Policy**

All modifications to this document will follow the process defined in `AER-STR-001`, Section 0\. All changes must be proposed via a pull request and approved by the Accountable (A) owner before being merged.

### **0.3 References**

This architecture is derived from and must remain consistent with the following core documents:

* [`AER-STR-001 — Team Charter`](?tab=t.0)  
* [`AER-ARCH-001 — System Architecture`](?tab=t.36s0p9u0ud88)  
* [`AER-REQ-001 — Product Requirements (PRD+SRS)`](?tab=t.tzoy6qm0m1o6)  
* [`AER-TST-001 — Test & Eval Master Plan (TEMP)`](?tab=t.5rwoo29othlw)

---

## **1\. Executive Summary**

**Aeris OS** is an edge-first, service-oriented software platform that runs on every Aeris vehicle. Its architecture is composed of fault-isolated, containerized services including an *Orchestrator* (mission logic), *Map Service* (real-time SLAM and fusion), *Perception* (AI-driven cues), and a *Mesh Agent* (resilient networking). This edge-native design ensures the swarm can execute its mission fully autonomously, even with total loss of contact with the Ground Control Station. The architecture is purpose-built to meet the project's most demanding KPIs, such as a time-to-first-map (TTFM) of under three minutes, robust offline operation, and the creation of secure, auditable evidence bundles.

---

## **2\. Guiding Architectural Principles**

*The following principles are the foundational rules that govern all software design and implementation decisions for the Aeris project. They are non-negotiable.*

### **2.1 Edge-First Autonomy**

The swarm is the server. All mission-critical processing—including perception, mapping, and mission logic—is executed "at the edge," directly on the drones. The GCS is a thin client for visualization and high-level command. Time synchronization uses **GNSS-disciplined IEEE-1588 PTP**; when GNSS is unavailable, PTP continues over AerisMesh with bounded accuracy, and the Map/Perception services will gate data fusion based on this clock skew.

### **2.2 Safety-Critical Isolation**

Flight-critical control is strictly isolated from mission software. The Flight Management Unit (FMU) runs a dedicated real-time operating system (RTOS) and is the final authority on flight. Aeris OS runs on a separate companion computer and issues only **advisory** setpoints (e.g., waypoints) via MAVLink. The FMU validates these against safety envelopes and may refuse or modify any command. No software bug within Aeris OS can cause a direct flight-critical failure.

### **2.3 Service-Oriented & Fault-Isolated**

Each responsibility is a containerized microservice managed by `Podman`. Service images are signed, run as non-root users with a read-only root filesystem, and are constrained by `cgroups` and `cpuset`. A crash in one service (e.g., Perception) will be isolated and will not cascade to crash other critical services like the Mesh Agent.

### **2.4 Modular & Extensible**

The architecture is explicitly designed to support the modular pod system. New sensors and capabilities are integrated as self-contained software plugins to the Perception and Orchestrator services, requiring no redesign of the core OS.

---

## **3\. System Overview & Process Model**

The software on each Aeris vehicle is divided into two distinct compute tiers.

### **3.1 Compute Tiers**

* **Flight Management Unit (FMU):** A hard real-time microcontroller (e.g., STM32H7-class) running an RTOS (e.g., NuttX), responsible for flight stabilization and failsafes.  
* **Companion Computer:** An embedded AI System-on-Module (e.g., Jetson Orin-class) running Aeris OS, responsible for all mission logic.

### **3.2 OS & Runtime**

* **Operating System:** Linux with a `PREEMPT_RT` kernel patch to ensure predictable scheduling. CPU cores and I/O priorities will be explicitly managed for real-time services like video encoding.  
* **Runtime Environment:** OCI containers via `Podman`, with per-service healthchecks, resource limits, and `seccomp` profiles to enforce the principle of least privilege.

### 

### 

### **3.3 Process Topology**

![][image1]

## 4\. Core Services (Aeris OS)

### 4.1 Orchestrator

**Purpose:** The mission "play" engine; translates high-level goals into autonomous trajectories and adapts to new cues.

**Key Functions:**

* Manages mission state machine (`SEARCHING`, `TRACKING_PLUME`, etc.)  
* Generates optimal search patterns and flight paths  
* Dynamically replans based on cues from the Perception service

**Performance:** Must maintain a control loop of **≤ 20 ms** and a replan latency of **≤ 200 ms** from trigger to new setpoint stream.

### 4.2 Map Service

**Purpose:** Creates and serves the single, authoritative fused map of the operational area.

**Key Functions & Algorithms:**

* **VIO Mode (No LiDAR):** Utilizes **RTAB-Map** as the back-end, fed by an **OpenVINS** front-end for stereo+IMU odometry.  
* **LiDAR Mode (Pod Active):** Switches to **LIO-SAM** for superior accuracy in degraded visual environments.  
* **Output:** Generates map tiles in **MBTiles 1.3** format and publishes them for the Mesh Agent.

**Performance:** Must generate the **first map tile \< 90 s** from liftoff and maintain an **edge-to-GCS tile latency of ≤ 2 s** (95th percentile).

### 4.3 Perception

**Purpose:** Converts raw sensor data into low-bandwidth, actionable intelligence.

**Key Functions & Algorithms:**

* **Thermal:** Hotspot detection at **≥ 5 fps**.  
* **Acoustic:** **MVDR / GCC-PHAT** beamforming to generate bearings with confidence scores from the 4-mic array.  
* **Gas:** **Kernel DM+V/W** algorithm to model wind-aware plume polygons at **0.5–1 Hz**.

### 4.4 Mesh Agent

**Purpose:** The intelligent traffic controller for all communications across the AerisMesh and 5G backhaul.

**Key Functions:**

* Enforces QoS priority queues (**Control \> Telemetry \> Video \> Bulk**)  
* Runs Adaptive Bitrate (ABR) logic for video streams over **SRT**  
* Manages a **store-and-forward** buffer for tiles and evidence when disconnected

### 4.5 Device Manager (Pod Service)

**Purpose:** Manages the physical and logical lifecycle of modular pods.

**Key Functions:** Executes the hot-swap state machine: 1-Wire detect → EEPROM CRC check → Power budget validation → Soft-start (inrush ≤2× nominal for \<50 ms) → Data link enumeration (ETH/USB) → Capability registration.

**Performance:** Must complete the entire ground hot-swap sequence in **\< 15 s**.

### 4.6 Evidence Service

**Purpose:** Creates a secure, auditable, and immutable record of the mission.

**Key Functions:**

* Aggregates mission data  
* Generates a manifest with **SHA-256 hashes**  
* Signs the manifest with **Ed25519 or P-256** signatures using keys stored in the hardware **TPM/fTPM**  
* Applies default **face/plate redaction** on export unless overridden by user role

**Performance:** Must export a 15-minute mission bundle in **\< 60 s**.

---

## 5\. Communications & Data Transport

### 5.1 In-Vehicle Transport

**Protocol:** **ROS 2 \+ DDS** (CycloneDDS or Fast DDS) for high-bandwidth local topics. The DDS implementation will be explicitly tuned for Wi-Fi reliability using static discovery, tuned fragmentation, and enlarged socket buffers.

### 5.2 Cross-Vehicle & GCS Transport

* **Control & Telemetry:** **gRPC streaming over HTTP/2** (with QUIC/HTTP/3 as a future target). Provides a reliable, low-latency, and strictly-defined API.  
* **Live Video:** **SRT over UDP**, with its bitrate controlled by the Mesh Agent's ABR logic.  
* **Map Tiles & Evidence:** **Opportunistic HTTP/3 (QUIC)**, leveraging the Mesh Agent's store-and-forward capability. Tiles are formatted as **MBTiles 1.3**.

---

## 6\. Cross-Cutting Concerns

### 6.1 Time Synchronization

* **Strategy:** Hierarchical **IEEE-1588 PTP**. The Ranger acts as a **GNSS-disciplined Grandmaster**. Scouts are slaves over the AerisMesh.  
* **Fusion Gate:** The Map Service will only fuse cross-sensor data with timestamps within a **\< 2 ms** threshold.

### 6.2 Security

* **Transport:** TLS 1.3 with mTLS for all control plane traffic.  
* **Authentication:** Role-Based Access Control (RBAC) with short-lived (≤ 8h), mission-scoped tokens.  
* **Data at Rest:** All stored evidence is encrypted using LUKS, with keys protected by the TPM/fTPM.

### 6.3 Observability

* **Logging:** Structured **NDJSON** logs with mission and time indices.  
* **Metrics:** Prometheus-style metrics exported for all key performance indicators (TTFM, VFIX, PLUME, MESH).

---

## **7\. Interfaces & APIs (Summaries)**

### 7.1 gRPC (Control/Telemetry)

```py
service Control {

  rpc StartPlay(StartPlayReq) returns (Ack);

  rpc Abort(AbortReq) returns (Ack);

  rpc SetGeofence(Geofence) returns (Ack);

  rpc Telemetry(stream TelemetryReq) returns (stream TelemetryMsg);

}
```

### 7.2 Message Schemas (JSON Examples)

**ThermalHotspot**

```py
{

  "ts_ptp": "2025-10-19T12:01:15.120Z",

  "bbox_px": [x, y, w, h],

  "temp_c": 36.8,

  "confidence": 0.87,

  "frame_id": "thermal_front"

}
```

**AcousticBearing**

```py
{

  "ts_ptp": "2025-10-19T12:01:15.180Z",

  "bearing_deg": 214.2,

  "confidence": 0.63,

  "snr_db": 7.5,

  "mic_array": "4ch_v1"

}
```

**GasIsopleth**

```py
{

  "ts_ptp": "2025-10-19T12:01:16.020Z",

  "species": "VOC",

  "units": "ppm",

  "polygons": [[[x,y],[...]]],

  "centerline": [[x,y],[...]]

}
```

**MapTileDescriptor**

```py
{

  "tile_id": "z/x/y",

  "format": "mbtiles-1.3",

  "layer_ids": ["base","thermal","gas","acoustic"],

  "hash_sha256": "…",

  "byte_size": 23712

}
```

**LinkMetrics**

```py
{

  "pdr": 0.995,

  "rtt_ms": 28.4,

  "throughput_kbps": 3200,

  "mcs": 7,

  "ab_level": "720p_15"

}
```

## 8\. Performance Budgets & SLOs

| Service | Budget/Target | Notes |
| ----- | ----- | ----- |
| Orchestrator | Loop ≤ 20 ms; Replan ≤ 200 ms | PREEMPT\_RT; CPU isolation |
| Map Service | First tile \< 90 s; Tile latency ≤ 2 s (95p) | RTAB-Map / LIO-SAM on dedicated cores |
| Perception | Thermal ≥ 5 fps; Gas 0.5–1 Hz | AI models on NPU/GPU |
| Mesh Agent | ABR reaction ≤ 1 s | QoS enforced at the kernel level |
| Evidence | Export 15-min \< 60 s | NVMe \+ hardware-accelerated crypto |

## **9\. Degraded Modes & Fault Handling**

* **Comms Loss (to GCS):** Continue autonomous mission; buffer tiles and evidence via store-and-forward.  
* **Clock Skew \> 2 ms:** Cease fusing cross-sensor overlays; annotate map with a time-sync warning.  
* **Thermal Throttling:** Reduce encoder framerates; pause non-critical perception tasks first.  
* **Pod Fault:** Device Manager initiates a soft power-cut; Orchestrator removes pod-dependent "plays" from the GCS.

---

## **10\. Deployment, Updates & Observability**

* **Deployment:** Services are deployed as signed OCI images with minimal capabilities and read-only root filesystems.  
* **Updates:** The system supports A/B image updates with automatic rollback on healthcheck failure.  
* **Observability:** A combination of NDJSON logs and Prometheus-style metrics provides comprehensive mission analysis.

---

## 

## 

## 

## **Appendix A — Reference Compute Profiles (Normative)**

Recommended hardware compute configurations for MVP builds. Select based on mission pod mix and power budget.

### **Scout (Default)**

* **SoM:** NVIDIA Jetson Orin Nano 8GB  
* **AI perf:** up to 40 TOPS (standard); "Super Dev Kit" mode up to 67 TOPS (dev kit)  
* **Power:** 10–15 W typical  
* **Memory:** 8 GB  
* **Storage:** NVMe ≥ 512 GB  
* **Use:** VIO mapping, thermal/acoustic/gas, single 1080p stream

### **Scout-LiDAR (Pro)**

* **SoM:** NVIDIA Jetson Orin NX 16GB  
* **AI perf:** up to ≈100 TOPS  
* **Power:** 15–25 W typical (configurable)  
* **Storage:** NVMe ≥ 512 GB  
* **Use:** LiDAR (LIO-SAM), 2× video encodes, heavier ML

### **Ranger**

* **SoM:** NVIDIA Jetson AGX Orin 64GB  
* **AI perf:** up to 275 TOPS  
* **Power:** 30–50 W typical (15–60 W range)  
* **Storage:** NVMe ≥ 1 TB  
* **Use:** Overwatch fusion, backhaul gateway, evidence vault

**Note:** Power caps must be tuned per airframe to meet endurance targets; use `nvpmodel` profiles and encoder offload to meet thermal limits.

---

## **Appendix B — Video ABR Ladder (SRT)**

| Level | Resolution | FPS | Target bitrate | Use |
| ----- | ----- | ----- | ----- | ----- |
| V0 | 640×480 | 10 | 0.35 Mbps | Thermal proof-of-life |
| V1 | 720p | 15 | 0.8 Mbps | Wide FOV situational |
| V2 | 1080p | 15 | 1.5 Mbps | Standard ops |
| V3 | 1080p | 30 | 3.0 Mbps | Visual detail |
| V4 | 1440p | 30 | 5.0 Mbps | Overwatch (Ranger) |

**Note:** Mesh Agent selects level based on PDR/RTT and buffer occupancy; drops to V0 on impairment; recovers stepwise.

---

## **Appendix C — DDS QoS Template (Cyclone/Fast DDS)**

```xml
# transport: UDP; Wi-Fi
history: KEEP_LAST 10
reliability: RELIABLE for control/telemetry, BEST_EFFORT for video/meta
durability: VOLATILE
deadline: 50ms (control), 500ms (telemetry)
liveliness: AUTOMATIC
resource_limits:
  max_samples: 1000
  max_instances: 128
  max_samples_per_instance: 256
discovery:
  static_peers: ["ranger.local", "scout1.local", "scout2.local"]
  writer_lifespan: 1s
network:
  socket_buffers: rx=8MB, tx=8MB
  fragmentation: enable
  max_message_size: 64KB
```

---

## **Appendix D — linuxptp Baseline**

**Configuration file: `/etc/ptp4l.conf`**

```shell
[global]
twoStepFlag           1
time_stamping         hardware
transportSpecific     0x1  # enterprise profile if required
delay_mechanism       E2E
network_transport     UDPv4
tx_timestamp_timeout  30
ds.macaddr            00:00:00:00:00:00

[eth0]
clock_type            OC
delay_filter          moving_average
delay_filter_length   10

```

**System services:**

```shell
ptp4l -i eth0 -f /etc/ptp4l.conf -m
phc2sys -s CLOCK_REALTIME -c eth0 -O 0 -m
```

---

## **Appendix E — Evidence Bundle Manifest (Example)**

```json
{
  "bundle_id": "AERIS-2025-10-19T12-10Z",
  "created_utc": "2025-10-19T12:15:00Z",
  "inputs": [
    {"type":"video","path":"video/main_20251019_1200Z.mp4","sha256":"…"},
    {"type":"tiles","path":"map/mbtiles/zoneA.mbtiles","sha256":"…"},
    {"type":"telemetry","path":"telemetry/mission.jsonl","sha256":"…"},
    {"type":"logs","path":"logs/*.jsonl","sha256":"…"}
  ],
  "signing": {
    "alg":"Ed25519",
    "signer":"edge-tpm-00A1",
    "signature":"BASE64…"
  },
  "redaction": {"faces":true,"plates":true}
}
```

---

## **Normative References**

* [MBTiles 1.3 Spec](https://github.com/mapbox/mbtiles-spec/blob/master/1.3/spec.md) — Mapbox  
* [SRT](https://github.com/Haivision/srt) — Haivision  
* [ROS 2 DDS tuning](https://docs.ros.org/en/rolling/How-To-Guides/DDS-tuning.html) — ROS 2 Docs  
* [linuxptp / ptp4l](https://linuxptp.nwtime.org/documentation/ptp4l/) — Man page  
* [RTAB-Map](https://introlab.github.io/rtabmap/) — Project site  
* [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM) — GitHub  
* [HTTP/3 (RFC 9114\)](https://www.rfc-editor.org/info/rfc9114) — IETF  
* [QUIC (RFC 9000\)](https://www.rfc-editor.org/rfc/rfc9000.pdf) — IETF  
* [Android Wi-Fi RTT (802.11mc)](https://developer.android.com/develop/connectivity/wifi/wifi-rtt) — Android Developers  
* [MAC randomization \- Apple](https://support.apple.com/guide/security/privacy-features-connecting-wireless-networks-secb9cb3140c/web)  
* [MAC randomization \- Android](https://source.android.com/docs/core/connect/wifi-mac-randomization-behavior)  
* [NVIDIA Jetson product performance](https://www.nvidia.com/ko-kr/autonomous-machines/embedded-systems/jetson-orin/) — Jetson Orin
