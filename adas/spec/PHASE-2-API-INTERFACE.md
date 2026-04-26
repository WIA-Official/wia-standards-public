# WIA-AUTO-002 PHASE 2 — API Interface Specification

**Standard:** WIA-AUTO-002
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 2 of 4)

---

3. Tracking
   - Kalman filter per target
   - Track-before-detect for weak targets

4. Classification
   - RCS-based (Radar Cross Section)
   - Micro-Doppler signature
```

### 4.3 Object Tracking

#### 4.3.1 Single Object Tracking

State vector:
```
x = [px, py, vx, vy, ax, ay, length, width, heading]ᵀ
```

Where:
- `px, py` = Position (m)
- `vx, vy` = Velocity (m/s)
- `ax, ay` = Acceleration (m/s²)
- `length, width` = Object dimensions (m)
- `heading` = Orientation (radians)

#### 4.3.2 Multi-Object Tracking

**Global Nearest Neighbor (GNN):**
```
Cost Matrix C[i,j] = distance(track_i, detection_j)
Assignment = Hungarian_Algorithm(C)
```

**Joint Probabilistic Data Association (JPDA):**
```
βᵢⱼ = probability that detection j originated from track i
x̂ᵢ = Σⱼ βᵢⱼ × x̂ᵢⱼ  (weighted average of all associations)
```

### 4.4 Object State Format

```json
{
  "object_id": 12345,
  "class": "car",
  "class_confidence": 0.97,
  "position": {
    "x": 25.3,
    "y": -2.1,
    "z": 0.0
  },
  "velocity": {
    "x": 22.5,
    "y": 0.2,
    "z": 0.0
  },
  "acceleration": {
    "x": -0.5,
    "y": 0.0,
    "z": 0.0
  },
  "dimensions": {
    "length": 4.5,
    "width": 1.8,
    "height": 1.5
  },
  "heading": 0.05,
  "tracking_age": 2.5,
  "prediction_horizon": 5.0
}
```

---

## 5. Lane Detection and Keeping

### 5.1 Lane Detection Algorithms

#### 5.1.1 Classical Approach

**Canny Edge Detection + Hough Transform:**

```
1. Pre-processing
   - Gaussian blur (5×5 kernel)
   - ROI masking (trapezoid)

2. Edge Detection
   - Canny edges (threshold: 50-150)

3. Line Detection
   - Hough Transform
   - θ ∈ [20°, 80°] ∪ [100°, 160°]
   - ρ resolution: 1 pixel
   - θ resolution: 1°

4. Lane Fitting
   - Linear regression or polynomial fit
   - Degree 2-3 polynomial for curves
```

#### 5.1.2 Deep Learning Approach

**Segmentation Network (U-Net, ENet):**

```
Input Image (1920×1080)
    ↓
Encoder (downsample to 60×34)
    ↓
Decoder (upsample to 1920×1080)
    ↓
Per-pixel Classification
    ↓
Lane Mask
```

**Output:** Pixel-wise lane mask with classes:
- Background
- Ego lane left
- Ego lane right
- Adjacent lane left
- Adjacent lane right

#### 5.1.3 Lane Model

**Polynomial representation:**
```
x(y) = a₀ + a₁y + a₂y² + a₃y³
```

**Clothoid (Euler spiral) representation:**
```
x(s) = ∫₀ˢ cos(θ(t)) dt
y(s) = ∫₀ˢ sin(θ(t)) dt
θ(s) = c₀ + c₁s + c₂s²
```

Where:
- `c₀` = Initial heading
- `c₁` = Curvature
- `c₂` = Curvature rate
- `s` = Arc length

### 5.2 Lane Keeping Assist (LKA)

#### 5.2.1 Lateral Position Control

**PID Controller:**
```
δ = Kₚ × e + Kᵢ × ∫e dt + Kd × de/dt
```

Where:
- `δ` = Steering angle (radians)
- `e` = Lateral offset from lane center (m)
- `Kₚ` = Proportional gain (typical: 0.5-2.0)
- `Kᵢ` = Integral gain (typical: 0.01-0.1)
- `Kd` = Derivative gain (typical: 0.1-0.5)

**Model Predictive Control (MPC):**
```
Minimize: J = Σᵢ (qₑ × eᵢ² + qₑ̇ × ėᵢ² + qδ × δᵢ² + qδ̇ × Δδᵢ²)
Subject to:
  - |δ| ≤ δₘₐₓ
  - |Δδ| ≤ Δδₘₐₓ
  - Vehicle dynamics constraints
```

#### 5.2.2 Lane Departure Warning (LDW)

**Time-to-Lane-Crossing (TLC):**
```
TLC = d_lane_edge / (v × sin(θ))
```

Where:
- `d_lane_edge` = Distance to lane boundary (m)
- `v` = Vehicle velocity (m/s)
- `θ` = Heading angle relative to lane (radians)

**Warning Levels:**
- `TLC > 2.0s`: No warning
- `1.0s < TLC ≤ 2.0s`: Visual warning
- `0.5s < TLC ≤ 1.0s`: Haptic warning (steering vibration)
- `TLC ≤ 0.5s`: Audible warning + corrective steering

### 5.3 Lane Change Assist (LCA)

#### 5.3.1 Safe Lane Change Conditions

```
1. Target lane is clear:
   - No vehicle within safe distance (front and rear)
   - d_front > v × t_gap + d_min
   - d_rear > v_rear × t_gap + d_min

2. Sufficient lateral space:
   - Lane width > vehicle width + margin
   - Margin ≥ 0.5m per side

3. Turn signal activated:
   - Duration ≥ 3 seconds before maneuver

4. Road conditions permit:
   - No construction zone
   - No solid lane markings
   - Weather conditions acceptable
```

#### 5.3.2 Lane Change Trajectory

**Cosine-based trajectory:**
```
y(x) = (w/2) × (1 - cos(πx/L))
```

Where:
- `w` = Lane width (m)
- `L` = Lane change length (m)
- `x` = Longitudinal distance (m)
- `y` = Lateral position (m)

**Duration:**
```
T = L / v
```

Typical values:
- `L = 30-50m` at highway speeds
- `T = 3-5 seconds`

---

## 6. Collision Avoidance Systems

### 6.1 Time-to-Collision (TTC)

#### 6.1.1 Basic TTC Formula

```
TTC = (d - d_safe) / (v_ego - v_target)
```

Where:
- `TTC` = Time to collision (seconds)
- `d` = Current distance (m)
- `d_safe` = Safe stopping distance (m)
- `v_ego` = Ego vehicle speed (m/s)
- `v_target` = Target object speed (m/s)

#### 6.1.2 Safe Stopping Distance

```
d_safe = v × t_reaction + (v² / (2 × μ × g))
```

Where:
- `t_reaction` = Reaction time (1.5s typical)
- `μ` = Friction coefficient
  - Dry asphalt: 0.8-0.9
  - Wet asphalt: 0.5-0.7
  - Snow: 0.2-0.3
  - Ice: 0.1-0.15
- `g` = Gravitational acceleration (9.81 m/s²)

#### 6.1.3 TTC Thresholds

| TTC Range | Action | Warning Level |
|-----------|--------|---------------|
| > 4.0s | Monitor only | None |
| 2.5-4.0s | Pre-charge brakes | Visual |
| 1.5-2.5s | Alert driver | Visual + Audible |
| 0.8-1.5s | Partial braking | Haptic + Audible |
| < 0.8s | Emergency braking | Full AEB activation |

### 6.2 Automatic Emergency Braking (AEB)

#### 6.2.1 Activation Logic

```python
def should_activate_aeb(ttc, certainty, velocity):
    if certainty < 0.9:
        return False  # Not confident enough

    if velocity < 5:  # m/s (~18 km/h)
        return False  # Too slow, driver can handle

    if ttc < 0.8:
        return True  # Critical situation

    if ttc < 1.5 and driver_not_reacting():
        return True  # Driver inattentive

    return False
```

#### 6.2.2 Braking Profile

**Jerk-limited braking:**
```
a(t) = a_max × (1 - e^(-t/τ))
```

Where:
- `a_max` = Maximum deceleration (8-10 m/s²)
- `τ` = Time constant (0.3-0.5s)
- Jerk limit: ±10 m/s³

**Distance to stop:**
```
d_stop = v²/(2 × a_max) + v × τ
```

### 6.3 Forward Collision Warning (FCW)

#### 6.3.1 Warning Strategy

**Multi-stage warning:**
```
Stage 1 (TTC < 3.0s): Visual indicator
Stage 2 (TTC < 2.0s): Visual + Audible beep
Stage 3 (TTC < 1.5s): Visual + Continuous tone + Brake prefill
Stage 4 (TTC < 0.8s): AEB activation
```

#### 6.3.2 False Positive Mitigation

```
Confidence Score = w₁ × sensor_quality +
                   w₂ × tracking_age +
                   w₃ × classification_confidence +
                   w₄ × trajectory_consistency

Activate only if: Confidence > 0.85
```

### 6.4 Pedestrian Detection and Protection

#### 6.4.1 Pedestrian Detection

**Multi-sensor approach:**
- Camera: HOG + CNN for pedestrian classification
- LiDAR: 3D point cloud clustering (height ~1.5-1.9m)
- Radar: Micro-Doppler signature (walking pattern)

**Detection challenges:**
- Occlusion (parked cars, street furniture)
- Varied appearance (clothing, posture)
- Unpredictable behavior
- Night-time detection

#### 6.4.2 Pedestrian Collision Mitigation

```
Risk Assessment:
1. Detect pedestrian
2. Predict trajectory (Kalman filter)
3. Calculate TTC
4. Assess collision probability

Actions:
- TTC > 2.5s: Monitor
- TTC 1.5-2.5s: Pre-brake, alert driver
- TTC < 1.5s: Full AEB + swerve if safe
```

**Swerve decision:**
```
if collision_imminent and can_swerve_safely():
    steering_angle = calculate_evasive_path()
    apply_steering_and_braking()
```

---


## Annex E — Implementation Notes for PHASE-2-API-INTERFACE

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-2-API-INTERFACE.

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
evidence for PHASE-2-API-INTERFACE. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-2-api-interface/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-2-API-INTERFACE with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-2-API-INTERFACE does not require bespoke
auditor tooling.
