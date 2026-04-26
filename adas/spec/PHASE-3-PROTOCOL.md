# WIA-AUTO-002 PHASE 3 — Protocol Specification

**Standard:** WIA-AUTO-002
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 3 of 4)

---

## 7. Adaptive Cruise Control

### 7.1 ACC Fundamentals

#### 7.1.1 Control Objectives

```
1. Maintain set speed (when road is clear)
   v_target = v_set

2. Maintain safe following distance (when following)
   d_target = v × t_gap + d_min

   Where:
   - t_gap = Time gap (1.0-2.5s, user-selectable)
   - d_min = Minimum distance (2-5m)
```

#### 7.1.2 Target Selection

**Multi-target scenario:**
```
for each detected_object:
    if is_in_ego_lane(object) and object.x > 0:
        potential_targets.add(object)

target = min(potential_targets, key=lambda obj: obj.x)
```

### 7.2 Longitudinal Control

#### 7.2.1 Speed Control (Cruise Mode)

**PID Controller:**
```
a_cmd = Kₚ × (v_set - v_ego) +
        Kᵢ × ∫(v_set - v_ego) dt +
        Kd × d/dt(v_set - v_ego)
```

Typical gains:
- `Kₚ = 0.3-0.5`
- `Kᵢ = 0.05-0.1`
- `Kd = 0.1-0.2`

#### 7.2.2 Following Control

**Gap control:**
```
e_gap = d_actual - d_target
a_cmd = Kₚ_gap × e_gap + Kd_gap × (v_target - v_ego)
```

Where:
```
d_target = v_ego × t_gap + d_min
v_target = v_leading_vehicle
```

#### 7.2.3 MPC-based ACC

**Optimization problem:**
```
Minimize: J = Σᵢ [q₁(vᵢ - v_ref)² + q₂(dᵢ - d_ref)² + q₃aᵢ² + q₄Δaᵢ²]

Subject to:
  - v_min ≤ vᵢ ≤ v_max
  - a_min ≤ aᵢ ≤ a_max
  - |Δaᵢ| ≤ jerk_max
  - dᵢ ≥ d_safe
```

### 7.3 ACC Operating Modes

| Mode | Condition | Behavior |
|------|-----------|----------|
| **Cruise** | No target ahead | Maintain set speed |
| **Following** | Target in range | Match target speed, maintain gap |
| **Approaching** | Closing on slower target | Decelerate smoothly |
| **Cut-in** | Vehicle enters lane | Quick deceleration |
| **Target Lost** | Target leaves lane | Resume to set speed |
| **Standstill** | Traffic stop (Stop&Go) | Full stop, resume on signal |

### 7.4 Stop & Go Functionality

#### 7.4.1 Standstill Management

```
if v_ego < 3 km/h and v_target == 0:
    apply_standstill_brake()
    state = STANDSTILL

if state == STANDSTILL:
    if v_target > 0:
        if standstill_time < 3s:
            resume_automatically()
        else:
            request_driver_confirmation()
```

#### 7.4.2 Comfort Braking

**Jerk minimization:**
```
a(t) = a_initial × (1 - t/T)³ + a_final × (t/T)³
```

This ensures:
- Smooth acceleration transitions
- Passenger comfort
- Reduced wear on brake system

### 7.5 ACC Safety Features

#### 7.5.1 Override Conditions

Driver override:
- Accelerator pedal > 10% → Disable distance control, allow speedup
- Brake pedal > 5% → Full system deactivation
- Steering > 90° → System deactivation (emergency maneuver)

#### 7.5.2 Fault Handling

```
if sensor_degraded():
    if degradation_level == MINOR:
        reduce_max_speed(to=80 km/h)
        reduce_time_gap(to=2.5s)
    elif degradation_level == MAJOR:
        alert_driver("ACC Limited Functionality")
        limit_speed(to=60 km/h)
    elif degradation_level == CRITICAL:
        deactivate_acc()
        alert_driver("ACC Unavailable")
```

---

## 8. SAE Automation Levels

### 8.1 Level Definitions

#### 8.1.1 SAE Level 0: No Automation

```
Driver performs all tasks:
- Steering
- Acceleration/deceleration
- Monitoring environment
- Fallback performance

Allowed features:
- Warnings (FCW, LDW)
- Momentary intervention (AEB in emergency)
```

#### 8.1.2 SAE Level 1: Driver Assistance

```
System controls:
- Steering OR
- Acceleration/deceleration

Driver performs:
- All other tasks
- Continuous monitoring
- Immediate takeover capability

Examples:
- Adaptive Cruise Control (ACC) alone
- Lane Keeping Assist (LKA) alone
```

#### 8.1.3 SAE Level 2: Partial Automation

```
System controls:
- Steering AND
- Acceleration/deceleration
(Simultaneously in specific conditions)

Driver performs:
- Continuous monitoring (hands-on, eyes-on)
- Immediate takeover
- Fallback performance

Examples:
- Tesla Autopilot
- GM Super Cruise
- Mercedes Drive Pilot (in traffic jams)

Requirements:
- Driver monitoring system
- Clear ODD (Operational Design Domain)
- Takeover request system
```

#### 8.1.4 SAE Level 3: Conditional Automation

```
System performs:
- All driving tasks within ODD
- Environment monitoring
- Fallback (with warning)

Driver performs:
- Respond to takeover request
- Fallback outside ODD

ODD Examples:
- Highway driving < 60 km/h
- Clear weather, marked lanes
- Specific geographic areas

Examples:
- Honda Legend (Traffic Jam Pilot)
- Mercedes Drive Pilot (Level 3 certified)

Minimum Risk Condition:
- If driver doesn't respond: slow to stop, hazard lights
```

#### 8.1.5 SAE Level 4: High Automation

```
System performs:
- All driving tasks within ODD
- Complete fallback within ODD

Driver:
- Not required within ODD
- May take over if desired

ODD Examples:
- Geo-fenced areas
- Specific weather conditions
- Designated roads/routes

Examples:
- Waymo robotaxis (Phoenix, SF)
- Cruise robotaxis
- Nuro delivery vehicles

Fallback:
- System brings vehicle to Minimum Risk Condition
- May continue in degraded mode
```

#### 8.1.6 SAE Level 5: Full Automation

```
System performs:
- All driving tasks
- All locations, all conditions
- Complete fallback everywhere

No ODD restrictions

Driver:
- Optional (passenger)
- No driving controls required

Status: Not yet achieved (as of 2025)

Challenges:
- Edge cases (construction, accidents)
- Extreme weather
- Unmapped areas
- Regulatory approval
```

### 8.2 Implementation Requirements by Level

| Requirement | L0 | L1 | L2 | L3 | L4 | L5 |
|-------------|----|----|----|----|----|----|
| Sensor redundancy | No | No | Yes | Yes | Yes | Yes |
| Driver monitoring | No | No | Yes | Yes | No | No |
| V2X communication | No | No | Optional | Recommended | Required | Required |
| HD Maps | No | No | Optional | Required | Required | Required |
| Remote assistance | No | No | No | Optional | Required | Required |
| Cybersecurity | Basic | Basic | Enhanced | Critical | Critical | Critical |
| OTA updates | No | Optional | Required | Required | Required | Required |
| Data recording (black box) | No | Optional | Required | Required | Required | Required |

---

## 9. Data Formats and Protocols

### 9.1 Sensor Data Formats

#### 9.1.1 LiDAR Point Cloud (PCD)

```protobuf
message PointCloud {
  message Point {
    float x = 1;              // meters
    float y = 2;              // meters
    float z = 3;              // meters
    uint32 intensity = 4;     // 0-255
    uint64 timestamp = 5;     // microseconds
    uint32 ring = 6;          // laser ring ID
  }

  repeated Point points = 1;
  uint64 timestamp = 2;
  string frame_id = 3;
}
```

#### 9.1.2 Radar Detections

```protobuf
message RadarDetection {
  message Target {
    float range = 1;          // meters
    float azimuth = 2;        // radians
    float elevation = 3;      // radians
    float range_rate = 4;     // m/s (radial velocity)
    float rcs = 5;            // dBsm (Radar Cross Section)
    float snr = 6;            // dB (Signal-to-Noise Ratio)
  }

  repeated Target targets = 1;
  uint64 timestamp = 2;
  string sensor_id = 3;
}
```

#### 9.1.3 Camera Image

```protobuf
message CameraImage {
  uint32 width = 1;
  uint32 height = 2;
  string encoding = 3;        // "rgb8", "bgr8", "mono8", etc.
  bytes data = 4;             // Raw image data
  uint64 timestamp = 5;
  string camera_id = 6;
  CameraInfo camera_info = 7;
}

message CameraInfo {
  repeated float K = 1;       // 3x3 intrinsic matrix
  repeated float D = 2;       // Distortion coefficients
  repeated float R = 3;       // 3x3 rectification matrix
  repeated float P = 4;       // 3x4 projection matrix
}
```

### 9.2 Perception Output Formats

#### 9.2.1 Detected Objects

```json
{
  "timestamp": 1735200000000000,
  "frame_id": "base_link",
  "objects": [
    {
      "id": 12345,
      "class": "car",
      "class_confidence": 0.97,
      "position": {"x": 25.3, "y": -2.1, "z": 0.0},
      "velocity": {"x": 22.5, "y": 0.2, "z": 0.0},
      "acceleration": {"x": -0.5, "y": 0.0, "z": 0.0},
      "dimensions": {"length": 4.5, "width": 1.8, "height": 1.5},
      "heading": 0.05,
      "covariance": [
        [0.1, 0, 0, 0, 0, 0],
        [0, 0.1, 0, 0, 0, 0],
        [0, 0, 0.05, 0, 0, 0],
        [0, 0, 0, 0.05, 0, 0],
        [0, 0, 0, 0, 0.01, 0],
        [0, 0, 0, 0, 0, 0.01]


## Annex E — Implementation Notes for PHASE-3-PROTOCOL

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-3-PROTOCOL.

- **Operational scope** — implementations SHOULD declare their operational
  scope (single-tenant, multi-tenant, federated) in the OpenAPI document so
  that downstream auditors can score the deployment against the correct
  conformance tier in Annex A.
- **Schema evolution** — additive changes (new optional fields, new error
  codes) are non-breaking; renaming or removing fields, even in error
  payloads, MUST trigger a minor version bump.
- **Audit retention** — a 7-year retention window is sufficient to satisfy
  ISO/IEC 17065:2012 audit expectations in most jurisdictions; some
  regulators require longer retention, in which case the deployment policy
  MUST extend the retention window rather than relying on this PHASE's
  defaults.
- **Time synchronization** — sub-second deadlines depend on synchronized
  clocks. NTPv4 with stratum-2 servers is sufficient for most deadlines
  expressed in this PHASE; PTP is recommended for sites that require
  deterministic interlocks.
- **Error budget reporting** — implementations SHOULD publish a monthly
  error-budget summary (latency p95, error rate, violation hours) in the
  format defined by the WIA reporting profile to facilitate cross-vendor
  comparison without exposing tenant-specific data.

These notes are not requirements; they are a reference for field teams
mapping their existing operations onto WIA conformance.

## Annex F — Adoption Roadmap

The adoption roadmap for this PHASE document is non-normative and is intended to set expectations for early implementers about the relative stability of each section.

- **Stable** (sections marked normative with `MUST` / `MUST NOT`) — semantic versioning applies; breaking changes require a major version bump and at minimum 90 days of overlap with the prior major version on all WIA-published reference implementations.
- **Provisional** (sections in this Annex and Annex D) — items are tracked openly and may be promoted to normative status without a major version bump if community feedback supports promotion.
- **Reference** (test vectors, simulator behaviour, the reference TypeScript SDK) — versioned independently of this document so that mistakes in reference material can be corrected without amending the published PHASE document.

Implementers SHOULD subscribe to the WIA Standards GitHub release notifications to track promotions between these tiers. Comments on the roadmap are accepted via the GitHub issues tracker on the WIA-Official organization.

The roadmap is reviewed at every minor version of this PHASE document, and the review outcomes are recorded in the version-history table at the start of the document.

## Annex G — Test Vectors and Conformance Evidence

This annex describes how implementations capture and publish conformance
evidence for PHASE-3-PROTOCOL. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-3-protocol/`. Implementations claiming
  conformance MUST run all vectors in CI and publish the resulting
  pass/fail matrix in their compliance package.
- **Evidence package** — the compliance package is a tarball containing
  the SBOM (CycloneDX 1.5 or SPDX 2.3), the OpenAPI document, the test
  vector matrix, and a signed manifest. Signatures use Sigstore (DSSE
  envelope, Rekor transparency log entry) so that downstream consumers
  can verify provenance without trusting a private CA.
- **Quarterly recheck** — implementations re-publish the evidence package
  every quarter even if no source change occurred, so that consumers can
  detect environmental drift (compiler updates, dependency updates, OS
  updates) without polling vendor changelogs.
- **Cross-vendor crosswalk** — the WIA Standards working group maintains a
  crosswalk that maps each vector to the equivalent assertion in adjacent
  industry programs (where one exists), so an implementer that already
  certifies under one program can show conformance to PHASE-3-PROTOCOL with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-3-PROTOCOL does not require bespoke
auditor tooling.
