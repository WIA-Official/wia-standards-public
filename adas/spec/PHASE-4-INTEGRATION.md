# WIA-AUTO-002 PHASE 4 — Integration Specification

**Standard:** WIA-AUTO-002
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 4 of 4)

---

],
      "tracking_age": 2.5,
      "prediction": {
        "horizon": 5.0,
        "trajectory": [
          {"t": 0.0, "x": 25.3, "y": -2.1},
          {"t": 1.0, "x": 47.8, "y": -2.0},
          {"t": 2.0, "x": 70.3, "y": -1.9},
          {"t": 3.0, "x": 92.8, "y": -1.8},
          {"t": 4.0, "x": 115.3, "y": -1.7},
          {"t": 5.0, "x": 137.8, "y": -1.6}
        ]
      }
    }
  ]
}
```

#### 9.2.2 Lane Information

```json
{
  "timestamp": 1735200000000000,
  "lanes": {
    "ego_left": {
      "coefficients": [0.1, 0.002, -0.00001, 0.0],
      "type": "dashed",
      "color": "white",
      "confidence": 0.95,
      "valid_range": {"y_min": 0, "y_max": 80}
    },
    "ego_right": {
      "coefficients": [-3.5, 0.001, -0.00001, 0.0],
      "type": "solid",
      "color": "white",
      "confidence": 0.93,
      "valid_range": {"y_min": 0, "y_max": 80}
    }
  },
  "ego_position": {
    "lateral_offset": -0.15,
    "heading_angle": 0.02
  }
}
```

### 9.3 Communication Protocols

#### 9.3.1 CAN Bus Messages

**ACC Status (ID: 0x300)**
```
Byte 0: ACC State (0=Off, 1=Standby, 2=Active, 3=Override, 4=Fault)
Byte 1: Set Speed (km/h)
Byte 2: Time Gap Setting (0.1s resolution)
Byte 3: Target Distance High Byte
Byte 4: Target Distance Low Byte (0.1m resolution)
Byte 5: Target Speed (km/h)
Byte 6-7: Reserved
```

**AEB Status (ID: 0x301)**
```
Byte 0: AEB State (0=Idle, 1=Armed, 2=Pre-braking, 3=Full-braking)
Byte 1-2: TTC (0.01s resolution)
Byte 3: Brake Pressure (0-100%)
Byte 4: Collision Risk (0=None, 1=Low, 2=Medium, 3=High, 4=Critical)
Byte 5-7: Reserved
```

#### 9.3.2 V2X (Vehicle-to-Everything)

**BSM (Basic Safety Message) - 802.11p**
```protobuf
message BasicSafetyMessage {
  uint32 msg_count = 1;       // Message sequence number
  bytes temporary_id = 2;     // Random vehicle ID (privacy)

  int32 lat = 3;              // Latitude (0.1 microdegree)
  int32 lon = 4;              // Longitude (0.1 microdegree)
  uint32 elevation = 5;       // Elevation (0.1m)

  uint32 speed = 6;           // Speed (0.02 m/s)
  uint32 heading = 7;         // Heading (0.0125 degrees)

  int32 accel_long = 8;       // Longitudinal accel (0.01 m/s²)
  int32 accel_lat = 9;        // Lateral accel (0.01 m/s²)
  int32 accel_vert = 10;      // Vertical accel (0.02 m/s²)

  VehicleSize size = 11;
  uint32 timestamp = 12;      // Milliseconds in minute
}
```

---

## 10. Safety Protocols and Validation

### 10.1 Functional Safety (ISO 26262)

#### 10.1.1 ASIL Levels

| Component | ASIL Level | Justification |
|-----------|------------|---------------|
| AEB | ASIL D | Critical safety function |
| Lane Keeping | ASIL B-D | Depends on implementation |
| ACC | ASIL B | Driver can override |
| Parking Assist | ASIL A | Low speed operation |
| Lane Departure Warning | ASIL A | Warning only |

#### 10.1.2 Safety Requirements

**For ASIL D (AEB):**
```
1. Sensor Redundancy: ≥2 independent sensors
2. Processing Redundancy: Dual ECUs with voting
3. Fault Detection: <100ms detection time
4. Fault Reaction: <200ms safe state transition
5. Diagnostic Coverage: >99%
6. Random Hardware Failure Rate: <10 FIT
7. Systematic Failure: Prevented through process
```

### 10.2 SOTIF (Safety of the Intended Functionality)

#### 10.2.1 Known Unsafe Scenarios

Examples requiring mitigation:
```
1. Overhanging cargo
   - LiDAR may not detect
   - Mitigation: Camera verification

2. Stationary vehicle on highway
   - Radar may filter as clutter
   - Mitigation: Camera + LiDAR fusion

3. Metallic debris on road
   - High radar RCS, low risk
   - Mitigation: Size estimation, classification

4. Black vehicles in tunnels
   - Low camera contrast
   - Mitigation: LiDAR primary, enhanced exposure

5. Pedestrians behind translucent barriers
   - Partial occlusion
   - Mitigation: Conservative detection, motion cues
```

#### 10.2.2 Performance Limits

```
Sensor Performance Envelope:
- LiDAR: Limited in heavy rain/fog
- Radar: Limited azimuth resolution
- Camera: Limited in low light, glare

System Performance Envelope:
- Max speed: 130 km/h (ACC), 80 km/h (LKA)
- Min visibility: 50m
- Road types: Paved, marked lanes
- Weather: Exclude heavy rain, snow, fog
```

### 10.3 Testing and Validation

#### 10.3.1 Test Scenarios (Euro NCAP)

**AEB Car-to-Car Tests:**
```
1. CCRs (Car-to-Car Rear Stationary)
   - Target: Stationary vehicle
   - Speed: 10, 20, 30, 40, 50 km/h

2. CCRm (Car-to-Car Rear Moving)
   - Target: Moving vehicle (20 km/h)
   - Speed: 30, 40, 50 km/h

3. CCRb (Car-to-Car Rear Braking)
   - Target: Decelerating vehicle (-6 m/s²)
   - Speed: 50 km/h
```

**AEB Pedestrian Tests:**
```
1. CPFA (Car-to-Pedestrian Farside Adult)
   - Adult crossing from right
   - Speed: 20, 30, 40, 50, 60 km/h

2. CPNC (Car-to-Pedestrian Nearside Child)
   - Child crossing from behind parked car
   - Speed: 20, 30, 40, 50 km/h

3. CPLA (Car-to-Pedestrian Longitudinal Adult)
   - Adult walking along road
   - Speed: 30, 40, 50, 60 km/h
```

#### 10.3.2 Simulation Testing

**Software-in-the-Loop (SIL):**
```
- Environment: Virtual scenarios
- Sensors: Simulated raw data
- Perception: Real algorithms
- Control: Real algorithms
- Vehicle: Simulated dynamics

Coverage: 10⁶ - 10⁹ km equivalent
```

**Hardware-in-the-Loop (HIL):**
```
- Environment: Simulated + real sensors
- ECUs: Real hardware
- Vehicle: Simulated dynamics
- CAN: Real bus

Coverage: 10⁵ - 10⁶ km equivalent
```

**Test Track:**
```
- Environment: Controlled real-world
- Sensors: Real
- Vehicle: Real with safety driver
- Targets: Soft dummies (EURO NCAP EVT, GVT)

Coverage: 10³ - 10⁴ km
```

**Public Road:**
```
- Environment: Real-world
- All systems real
- Safety driver + data logging

Coverage: 10⁶ - 10⁷ km required for Level 3+
```

### 10.4 Cybersecurity (ISO/SAE 21434)

#### 10.4.1 Threat Analysis

**Attack Vectors:**
```
1. Sensor Spoofing
   - GPS jamming/spoofing
   - Radar/LiDAR interference
   - Camera image injection

2. Communication Hijacking
   - CAN bus injection
   - V2X message forgery
   - OTA update tampering

3. Software Exploitation
   - Buffer overflow
   - Code injection
   - Privilege escalation
```

#### 10.4.2 Security Measures

```
1. Sensor Authentication
   - Encrypted communication
   - Digital signatures
   - Plausibility checks

2. Secure Boot
   - Verified boot chain
   - Code signing
   - Anti-rollback

3. Runtime Protection
   - Memory protection (MPU)
   - Intrusion detection
   - Anomaly monitoring

4. Communication Security
   - CAN authentication (CAN-FD + SecOC)
   - TLS for external communication
   - V2X message signing (PKI)

5. Update Security
   - Signed updates
   - Secure download (HTTPS)
   - Verification before installation
```

---

## 11. References

### 11.1 Standards and Regulations

1. **ISO 26262** - Road vehicles - Functional safety
2. **ISO/PAS 21448 (SOTIF)** - Safety Of The Intended Functionality
3. **ISO/SAE 21434** - Road vehicles - Cybersecurity engineering
4. **SAE J3016** - Taxonomy and Definitions for Terms Related to Driving Automation Systems
5. **Euro NCAP** - Assessment Protocol - Safety Assist
6. **UNECE R157** - Automated Lane Keeping Systems (ALKS)
7. **IEEE 802.11p** - Wireless Access in Vehicular Environments (WAVE)

### 11.2 Technical Papers

1. Kalman, R.E. (1960). "A New Approach to Linear Filtering and Prediction Problems"

### 11.3 Industry Resources

1. **Automotive Ethernet**: IEEE 802.3, 802.1 standards
2. **AUTOSAR**: Automotive Open System Architecture
3. **ROS 2**: Robot Operating System (for prototyping)
4. **OpenDRIVE**: Road network description format
5. **ASAM OpenX**: Simulation and testing standards

### 11.4 WIA Standards

- **WIA-INTENT**: Intent-based vehicle control interfaces
- **WIA-OMNI-API**: Universal automotive API gateway
- **WIA-SOCIAL**: V2V/V2X communication protocols
- **WIA-CLOUD**: Cloud services for connected vehicles
- **WIA-AI**: AI model training and deployment standards

---

## Appendix A: Example Calculations

### A.1 Safe Following Distance at 100 km/h

```
Given:
- Speed: v = 100 km/h = 27.78 m/s
- Reaction time: t = 1.5 s
- Friction (dry): μ = 0.8
- Gravity: g = 9.81 m/s²

Calculation:
- Reaction distance: d₁ = v × t = 27.78 × 1.5 = 41.67 m
- Braking distance: d₂ = v² / (2μg) = 27.78² / (2 × 0.8 × 9.81) = 49.17 m
- Total safe distance: d = d₁ + d₂ = 90.84 m

With 2-second time gap:
- d = 27.78 × 2 = 55.56 m < 90.84 m (unsafe for emergency stop)
- Recommended gap: 3.5 seconds minimum
```

### A.2 TTC Calculation Example

```
Given:
- Ego vehicle speed: v_ego = 90 km/h = 25 m/s
- Target vehicle speed: v_target = 50 km/h = 13.89 m/s
- Current distance: d = 50 m
- Safe distance: d_safe = 5 m

Relative velocity:
- v_rel = v_ego - v_target = 25 - 13.89 = 11.11 m/s

TTC:
- TTC = (d - d_safe) / v_rel
- TTC = (50 - 5) / 11.11 = 4.05 seconds

Action: Pre-charge brakes (TTC < 4.5s), visual warning
```

---

**弘익人間 (홍익인간) · Benefit All Humanity**

*WIA-AUTO-002 Specification v1.0*
*© 2025 SmileStory Inc. / WIA*
*MIT License*


## Annex E — Implementation Notes for PHASE-4-INTEGRATION

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-4-INTEGRATION.

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
evidence for PHASE-4-INTEGRATION. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-4-integration/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-4-INTEGRATION with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-4-INTEGRATION does not require bespoke
auditor tooling.
