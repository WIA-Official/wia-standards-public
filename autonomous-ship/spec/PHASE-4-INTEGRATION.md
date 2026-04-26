# WIA-AUTO-015 PHASE 4 — Integration Specification

**Standard:** WIA-AUTO-015
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 4 of 4)

---

"position": {"latitude": 30.000000, "longitude": 135.000000},
      "turn_radius": 0.5,
      "speed": 18.0,
      "eta": "2025-12-27T02:00:00.000Z"
    }
  ],
  "optimization": {
    "objective": "fuel_efficiency",
    "total_distance": 2850.5,
    "estimated_fuel": 1250.0,
    "estimated_duration": 168.5,
    "weather_routed": true
  },
  "constraints": {
    "max_wave_height": 6.0,
    "max_wind_speed": 30.0,
    "eca_compliant": true,
    "tsss_compliance": true
  }
}
```

### 10.4 System Status

```json
{
  "type": "system_status",
  "timestamp": "2025-12-26T12:00:00.000Z",
  "overall_health": "NOMINAL",
  "autonomy_level": 3,
  "subsystems": {
    "navigation": {
      "status": "OPERATIONAL",
      "gnss": {"fix_quality": "RTK", "satellites": 18},
      "imu": {"status": "NOMINAL", "drift": 0.001},
      "ecdis": {"status": "OPERATIONAL", "chart_coverage": 100}
    },
    "propulsion": {
      "status": "OPERATIONAL",
      "main_engine": {"rpm": 850, "load": 75, "temp": 85},
      "fuel": {"level": 82.5, "consumption_rate": 8.5}
    },
    "sensors": {
      "radar_x": {"status": "OPERATIONAL", "range": 48},
      "radar_s": {"status": "OPERATIONAL", "range": 96},
      "lidar": {"status": "OPERATIONAL", "points_per_sec": 1200000},
      "cameras": {"operational": 8, "degraded": 0, "failed": 0},
      "ais": {"status": "OPERATIONAL", "targets": 15}
    },
    "communication": {
      "satellite": {"status": "OPERATIONAL", "bandwidth": 2.5, "latency": 650},
      "cellular": {"status": "UNAVAILABLE"},
      "vhf": {"status": "OPERATIONAL"}
    }
  },
  "alerts": [
    {
      "severity": "INFO",
      "subsystem": "propulsion",
      "message": "Fuel level below 85%, recommend refueling at next port"
    }
  ]
}
```

---

## 11. API Interface

### 11.1 REST API Endpoints

```
Base URL: https://api.autonomous-ship.wia/v1

Authentication: Bearer token (JWT)

Endpoints:

GET /ships/{imo}/position
  Response: Current position and navigation status

POST /ships/{imo}/commands/set-course
  Body: {"heading": 90.0, "speed": 15.0}
  Response: Command acknowledgment

GET /ships/{imo}/route
  Response: Current route plan

POST /ships/{imo}/route
  Body: Route plan JSON
  Response: Route validation and acceptance

GET /ships/{imo}/collision-warnings
  Response: Active collision warnings

POST /ships/{imo}/avoid-collision
  Body: Maneuver parameters
  Response: Maneuver execution status

GET /ships/{imo}/system-status
  Response: Complete system health

GET /ships/{imo}/sensor-data
  Query: ?sensor=radar&timerange=last_hour
  Response: Historical sensor data

POST /ships/{imo}/autonomy-level
  Body: {"level": 3, "reason": "entering_open_ocean"}
  Response: Level change confirmation

WebSocket: wss://api.autonomous-ship.wia/v1/ships/{imo}/stream
  Real-time data streaming
```

### 11.2 SDK Methods

```typescript
class AutonomousShipSDK {
  // Initialization
  constructor(config: ShipConfig)

  // Navigation
  async startNavigation(params: NavigationParams): Promise<void>
  async stopNavigation(): Promise<void>
  async setCourse(heading: number, speed: number): Promise<void>
  async planRoute(origin: Position, destination: Position, options?: RouteOptions): Promise<Route>
  async updateRoute(route: Route): Promise<void>

  // Collision Avoidance
  async getCollisionWarnings(): Promise<CollisionWarning[]>
  async executeAvoidanceManeuver(maneuver: Maneuver): Promise<void>
  calculateCollisionRisk(ownShip: ShipState, target: ShipState): CollisionAssessment

  // Monitoring
  async getPosition(): Promise<Position>
  async getSystemStatus(): Promise<SystemStatus>
  async getSensorData(sensor: SensorType, timeRange?: TimeRange): Promise<SensorData>

  // Control
  async setAutonomyLevel(level: number): Promise<void>
  async emergencyStop(): Promise<void>
  async returnToPort(port: string): Promise<void>

  // Events
  on(event: 'collision-warning', handler: (warning: CollisionWarning) => void): void
  on(event: 'system-alert', handler: (alert: SystemAlert) => void): void
  on(event: 'position-update', handler: (position: Position) => void): void
}
```

---

## 12. Safety Protocols

### 12.1 Pre-Voyage Checklist

```
Autonomous Voyage Checklist:

Navigation Systems:
  ☐ GNSS fix quality: RTK or better
  ☐ IMU calibrated and operational
  ☐ ECDIS charts updated and covering full route
  ☐ Backup navigation systems tested

Sensors:
  ☐ Radar (X-band and S-band) operational
  ☐ LiDAR operational and calibrated
  ☐ Cameras (all 360° coverage) operational
  ☐ AIS receiver operational
  ☐ Weather sensors calibrated

Communication:
  ☐ Satellite link established (latency < 1 second)
  ☐ VHF radio operational
  ☐ Emergency beacon tested
  ☐ Shore control center connection verified

Propulsion & Control:
  ☐ Main engine operational
  ☐ Steering system tested (full range)
  ☐ Thrusters operational (if equipped)
  ☐ Emergency shutdown tested

Safety:
  ☐ Collision avoidance system tested
  ☐ Geofencing boundaries loaded
  ☐ Emergency protocols programmed
  ☐ Autonomous emergency anchoring tested

Cybersecurity:
  ☐ All software up to date
  ☐ Firewall rules validated
  ☐ Authentication systems tested
  ☐ Intrusion detection active

Regulatory:
  ☐ Autonomy level approved for route
  ☐ Flag state authorization obtained
  ☐ Coastal state permissions (if required)
  ☐ Insurance coverage confirmed
  ☐ Emergency contact list updated
```

### 12.2 Emergency Procedures

#### 12.2.1 Loss of Communication

```
Communication Loss Protocol:

1. Detection (after 60 seconds no contact):
   ALERT "Communication lost with shore control"
   START degraded_operations_mode

2. Actions:
   - Continue on planned route
   - Maintain enhanced lookout
   - Attempt to re-establish communication
   - Reduce speed by 20% for safety margin

3. If communication not restored in 10 minutes:
   - Execute pre-programmed contingency route
   - Broadcast AIS safety message
   - Attempt VHF contact with nearby vessels

4. If communication not restored in 60 minutes:
   - Proceed to nearest safe anchorage
   - Drop anchor in designated emergency position
   - Activate emergency beacon
   - Wait for assistance

5. Communication restoration:
   - Report status to shore control
   - Request permission to resume voyage
   - Conduct system diagnostic
   - Continue or abort as directed
```

#### 12.2.2 Sensor Failure

```
Sensor Degradation Response:

Loss of GNSS:
  → Switch to dead reckoning
  → Use radar fixes for position updates
  → Reduce speed to 50%
  → Notify shore control
  → Proceed to nearest port if degradation continues

Loss of Radar:
  → Rely on AIS and cameras
  → Activate all cameras
  → Reduce speed to 50%
  → Avoid traffic lanes
  → Request shore guidance

Loss of AIS:
  → Enhance radar surveillance
  → Visual/camera lookout
  → Broadcast VHF safety message
  → Continue with caution

Multiple Sensor Failures:
  → Reduce autonomy level
  → Emergency stop if < 50% sensor coverage
  → Request immediate shore intervention
  → Prepare for manual control/crew deployment
```

#### 12.2.3 Imminent Collision

```
Collision Emergency Protocol:

IF collision_probability > 0.9 AND TCPA < 2 minutes:

    1. Immediate Actions (automatic):
       - Sound collision alarm
       - Execute emergency maneuver
       - Full rudder + engine maneuver
       - Activate all warning signals

    2. Emergency Maneuver:
       IF room_to_starboard:
           HARD_TURN starboard
       ELSE IF room_to_port:
           HARD_TURN port
       ELSE:
           FULL_ASTERN + turn_away

    3. Notifications:
       - Broadcast VHF warning
       - Send AIS safety message
       - Alert shore control (highest priority)
       - Activate deck cameras

    4. Post-Maneuver:
       - Assess situation
       - Check for damage
       - Report to shore
       - Adjust autonomy level if needed
```

### 12.3 Fault Tolerance

```
System Redundancy Requirements:

Critical Systems (Triple Redundancy):
  - Navigation (GNSS + Dead Reckoning + Radar Fix)
  - Communication (Satellite + VHF + Emergency Beacon)
  - Power (Main + Auxiliary + Emergency)

Important Systems (Dual Redundancy):
  - Radar (X-band + S-band)
  - Steering (Main + Backup hydraulic)
  - Propulsion (Main + Auxiliary engine OR main + sail)

Failover Logic:

IF primary_system.failed:
    SWITCH_TO secondary_system
    ALERT shore_control
    LOG failure_event
    SCHEDULE maintenance

IF secondary_system.failed:
    SWITCH_TO tertiary_system (if available)
    ALERT CRITICAL shore_control
    REDUCE autonomy_level
    PROCEED_TO nearest_port

IF tertiary_system.failed:
    EXECUTE emergency_protocol
    REQUEST immediate_assistance
    PREPARE_FOR manual_control
```

---

## 13. References

### 13.1 International Maritime Regulations

1. **IMO MASS**: International Maritime Organization - Maritime Autonomous Surface Ships (MSC 103/WP.8, 2021)
2. **COLREG 1972**: Convention on the International Regulations for Preventing Collisions at Sea
3. **SOLAS**: International Convention for the Safety of Life at Sea
4. **STCW**: Standards of Training, Certification and Watchkeeping for Seafarers
5. **IMO 2020**: International Convention for the Prevention of Pollution from Ships (MARPOL) - Sulfur regulations

### 13.2 Technical Standards

6. **IEC 61162**: Maritime navigation and radiocommunication equipment and systems - Digital interfaces
7. **IEC 62288**: Maritime navigation and radiocommunication equipment and systems - Presentation of navigation-related information on shipborne navigational displays
8. **IEC 61924**: Maritime navigation and radiocommunication equipment and systems - Integrated navigation systems (INS)
9. **ISO 19847**: Ships and marine technology - Shipboard data servers to share field data at sea
10. **IMO Resolution A.694(17)**: General requirements for shipborne radio equipment forming part of the global maritime distress and safety system (GMDSS)

### 13.3 Cybersecurity Standards

11. **IEC 62443**: Industrial communication networks - Network and system security
12. **NIST CSF**: National Institute of Standards and Technology Cybersecurity Framework
13. **IMO MSC-FAL.1/Circ.3**: Guidelines on maritime cyber risk management
14. **BIMCO Guidelines**: The Guidelines on Cyber Security Onboard Ships (Version 4, 2020)

### 13.4 Navigation & Positioning

15. **WGS 84**: World Geodetic System 1984
16. **S-57**: IHO Transfer Standard for Digital Hydrographic Data
17. **S-100**: IHO Universal Hydrographic Data Model
18. **RTCM 10403**: Differential GNSS Services - Version 3

### 13.5 Scientific Papers

19. Fujii, Y., & Tanaka, K. (1971). "Traffic Capacity", Journal of Navigation, 24(4), 543-552
20. Goodwin, E.M. (1975). "A Statistical Study of Ship Domains", Journal of Navigation, 28(3), 328-344
21. Tam, C., Bucknall, R., Greig, A. (2009). "Review of Collision Avoidance and Path Planning Methods for Ships in Close Range Encounters", Journal of Navigation, 62(3), 455-476
22. Perera, L.P., Carvalho, J.P., Soares, C.G. (2011). "Autonomous Guidance and Navigation Based on the COLREGs Rules and Regulations of Collision Avoidance", Proceedings of International Workshop on Advanced Ship Design for Pollution Prevention

### 13.6 WIA Standards

23. **WIA-INTENT**: Intent-based control interfaces
24. **WIA-OMNI-API**: Universal API gateway for maritime data
25. **WIA-IoT**: Internet of Things sensor integration
26. **WIA-BLOCKCHAIN**: Immutable voyage data recording
27. **WIA-SOCIAL**: Fleet coordination and communication protocols

---

## Appendix A: Example Calculations

### A.1 Great Circle Distance

```
Calculate distance from Tokyo to Singapore:

Tokyo: 35.676192°N, 139.650311°E
Singapore: 1.289670°N, 103.850067°E

φ₁ = 35.676192° × π/180 = 0.6227 rad
φ₂ = 1.289670° × π/180 = 0.0225 rad
Δλ = (103.850067 - 139.650311)° × π/180 = -0.6252 rad

a = sin²((0.0225 - 0.6227)/2) + cos(0.6227) × cos(0.0225) × sin²(-0.6252/2)
a = sin²(-0.3001) + 0.8132 × 0.9997 × sin²(-0.3126)
a = 0.0872 + 0.8129 × 0.0945
a = 0.1640

c = 2 × atan2(√0.1640, √0.8360)
c = 2 × atan2(0.4050, 0.9143)
c = 2 × 0.4175 = 0.8350 rad

d = 3440.065 NM × 0.8350 = 2,872.5 NM

Estimated voyage time at 15 knots:
t = 2,872.5 NM / 15 knots = 191.5 hours ≈ 8 days
```

### A.2 Collision Risk Assessment

```
Own Ship:
  Position: 35.676192°N, 139.650311°E
  Heading: 090° (East)
  Speed: 15 knots

Target Ship:
  Position: 35.685000°N, 139.750000°E
  Heading: 270° (West)
  Speed: 12 knots

Step 1: Calculate relative position
Δlat = 35.685000 - 35.676192 = 0.008808°
Δlon = 139.750000 - 139.650311 = 0.099689°

Distance ≈ √((Δlat × 60)² + (Δlon × 60 × cos(35.68°))²)
Distance ≈ √((0.528)² + (4.85)²) ≈ 4.88 NM

Bearing ≈ atan2(Δlon × cos(35.68°), Δlat) = 83.8°

Step 2: Calculate relative velocity
Own velocity vector: (15 × cos(90°), 15 × sin(90°)) = (0, 15)
Target velocity: (12 × cos(270°), 12 × sin(270°)) = (0, -12)
Relative velocity: (0, 27) knots

Step 3: Calculate CPA and TCPA
Relative speed = 27 knots
Relative bearing to target = 83.8°
Course difference = |90° - 83.8°| = 6.2°

TCPA ≈ (Distance × cos(relative_bearing)) / relative_speed
TCPA ≈ (4.88 × cos(6.2°)) / 27 ≈ 0.179 hours ≈ 10.7 minutes

CPA ≈ Distance × sin(relative_bearing)
CPA ≈ 4.88 × sin(6.2°) ≈ 0.53 NM

Step 4: Risk Assessment
Risk_index = (2.0 / 0.53) × (20 / 10.7) = 3.77 × 1.87 = 7.05

Result: CRITICAL RISK - Immediate action required
Situation: Crossing (target on starboard)
Recommendation: Alter course to starboard by 30° or reduce speed by 50%
```

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA-AUTO-015 Specification v1.0*
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

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-4-INTEGRATION.
It is non-normative; the rules below describe the policy that the WIA
Standards working group commits to when amending this PHASE document.

- **Semantic versioning** — major / minor / patch components follow
  Semantic Versioning 2.0.0 (https://semver.org/spec/v2.0.0.html).
  Major bump indicates a backwards-incompatible change to a normative
  requirement; minor bump indicates new normative requirements that do
  not break existing implementations; patch bump indicates editorial
  changes only (clarifications, typo fixes, formatting).
- **Deprecation window** — when a normative requirement is removed or
  altered in a backwards-incompatible way, the prior major version is
  maintained in parallel for at least 180 days. During the parallel
  window, both major versions are marked Stable in the WIA Standards
  registry and either may be cited as "WIA-conformant".
- **Sunset notification** — deprecated major versions enter a 12-month
  sunset window during which the WIA registry marks the version as
  Deprecated. The deprecation entry includes a migration note pointing
  to the replacement requirement(s) and an explanation of why the
  change was made.
- **Editorial errata** — patch-level errata are issued without a
  deprecation window because they do not change normative behaviour.
  Errata are tracked in a public errata register and each entry is
  signed by the WIA Standards working group chair.
- **Implementation changelog mapping** — implementations SHOULD publish
  a changelog mapping each PHASE version they support to the specific
  build, container digest, or SDK version that satisfies the version.
  This allows downstream auditors to verify version conformance without
  re-running the entire test matrix on every release.

The policy is reviewed at the same cadence as the PHASE document and
any changes to the policy itself are tracked in the version-history
table at the start of the document.

## Annex I — Interoperability Profiles

This annex describes how implementations declare interoperability profiles
for PHASE-4-INTEGRATION. The profile mechanism is non-normative and exists so that
deployments of varying scope (single tenant, regional cluster, federated
network) can advertise the subset of normative requirements they satisfy
without misrepresenting partial conformance as full conformance.

- **Profile manifest** — every implementation publishes a profile manifest
  in JSON. The manifest enumerates the normative requirement IDs from this
  PHASE that are satisfied (`status: "supported"`), partially satisfied
  (`status: "partial"`, with a reason field), or excluded
  (`status: "excluded"`, with a justification). The manifest is signed
  using the same Sigstore key used for the SBOM in Annex G.
- **Federation profile** — federated deployments publish an aggregated
  manifest summarizing the union and intersection of member-implementation
  profiles. The aggregated manifest is consumed by directory services so
  that callers can route a request to the least common denominator profile
  required for an interaction.
- **Backwards-profile compatibility** — when a deployment migrates from one
  profile to a wider profile, the prior profile manifest remains valid and
  signed for the deprecation window defined in Annex H. This preserves
  audit traceability for auditors evaluating long-term interoperability.
- **Profile registry** — the WIA Standards working group maintains a
  public registry of named profiles. Common deployment shapes (e.g.,
  "Edge-only", "Federated-with-replay") are added to the registry by
  consensus. Registry entries are immutable; new shapes are added under
  new names rather than amending existing entries.
- **Profile versioning** — profile names are versioned with the same
  Semantic Versioning rules described in Annex H. A deployment that
  advertises `WIA-P4-INTEGRATION-Edge-only/2` is asserting conformance with
  the second major version of the named profile, not the second deployment
  of an unversioned profile.

The profile mechanism is intentionally lightweight; it is meant to make
real deployment shapes visible without forcing every deployment to
satisfy every normative requirement.

## Annex J — Reference Implementation Topology

The reference implementation topology described in this annex is
non-normative; it documents the deployment shape that the WIA
Standards working group used to validate the test vectors in Annex G
and is intended as a starting point, not a recommendation against
alternative topologies.

- **Single-tenant edge** — one runtime per organization, no shared
  state. Used for early-pilot deployments where conformance evidence
  is published manually. Sufficient for PHASE-4-INTEGRATION validation when the
  organization signs the manifest itself.
- **Multi-tenant gateway** — one shared runtime serves multiple
  tenants via header-based isolation. Typically backed by a
  rate-limited gateway (Envoy or NGINX) and a shared OAuth 2.1
  identity provider. The manifest is per-tenant; the runtime
  publishes a federation manifest that aggregates tenant manifests.
- **Federated mesh** — multiple runtimes peer to one another and
  publish their manifests to a directory service. Each peer signs
  its own manifest; the directory service signs the aggregated
  index. This is the topology used by cross-organization deployments
  that need to compose conformance.
- **Air-gapped batch** — no network connection between the runtime
  and the directory service. The runtime emits a signed evidence
  package on each batch and the operator transports the package via
  out-of-band channels. This is the topology used by regulators that
  prohibit live connectivity from sensitive environments.

Implementations declare their topology in the manifest (see Annex I).
A topology change MUST be reflected in a new manifest signature; the
prior topology's manifest remains valid for the deprecation window
described in Annex H to preserve audit traceability.
