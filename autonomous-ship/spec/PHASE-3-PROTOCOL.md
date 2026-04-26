# WIA-AUTO-015 PHASE 3 — Protocol Specification

**Standard:** WIA-AUTO-015
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 3 of 4)

---

For port approach (< 5 NM from berth):

```
Positioning accuracy requirements:
- Approach channel: ±2 meters
- Berthing: ±0.5 meters

Enhanced positioning:
1. DGPS/RTK GNSS
2. Port-based positioning system (GBAS)
3. LiDAR-based relative positioning
4. Visual landmark recognition

Fused Position:
position_fused = weighted_average([
    GNSS_position (weight: 0.4),
    GBAS_position (weight: 0.3),
    LiDAR_position (weight: 0.2),
    vision_position (weight: 0.1)
])
```

#### 7.3.2 Autonomous Docking

```
Docking Procedure:

1. Approach Phase (1000m - 200m)
   - Align with berth
   - Reduce speed gradually
   - Final speed: 0.5 - 1.0 knots

2. Final Approach (200m - 50m)
   - Fine position adjustment
   - Thruster control for lateral movement
   - Speed: < 0.5 knots

3. Berthing (50m - contact)
   - Precision positioning
   - Fender contact prediction
   - Speed at contact: < 0.1 knots

4. Making Fast
   - Hold position with thrusters
   - Coordinate with automated mooring system
   - Verify secure attachment

Control Algorithm (MPC - Model Predictive Control):

u_optimal = argmin Σ[t=0 to N] (
    ||position(t) - target_position||² +
    ||velocity(t) - target_velocity||² +
    λ × ||u(t)||²
)

Subject to:
    position(t+1) = f(position(t), velocity(t), u(t))
    |u(t)| ≤ u_max  (thruster limits)
    position(N) = berth_position
    velocity(N) = 0
```

---

## 8. Cybersecurity

### 8.1 Threat Model

#### 8.1.1 Threat Categories

```
Maritime Cyber Threats:

1. Unauthorized Access
   - Remote control hijacking
   - System penetration
   - Credential theft

2. Data Integrity Attacks
   - GNSS spoofing
   - AIS manipulation
   - Chart data corruption
   - Sensor data injection

3. Denial of Service
   - Communication jamming
   - System overload
   - Critical service disruption

4. Malware
   - Ransomware
   - Trojan horses
   - Logic bombs

5. Insider Threats
   - Malicious shore operators
   - Compromised maintenance personnel
```

#### 8.1.2 Attack Vectors

```
Common Attack Surfaces:

- Satellite communications (VSAT)
- Port facility interfaces
- Supply chain (software updates)
- Remote access (ROC connections)
- Sensor networks
- Third-party systems integration
```

### 8.2 Security Architecture

#### 8.2.1 Defense in Depth

```
Security Layers:

Layer 1 - Network Perimeter:
  - Firewalls (stateful inspection)
  - Intrusion Detection/Prevention Systems (IDS/IPS)
  - VPN for all remote connections
  - Network segmentation

Layer 2 - Access Control:
  - Multi-factor authentication (MFA)
  - Role-based access control (RBAC)
  - Principle of least privilege
  - Session management

Layer 3 - Application Security:
  - Input validation
  - Secure coding practices
  - Regular security audits
  - Penetration testing

Layer 4 - Data Security:
  - Encryption at rest (AES-256)
  - Encryption in transit (TLS 1.3)
  - Data integrity checks (HMAC)
  - Secure key management

Layer 5 - Monitoring & Response:
  - Security Information and Event Management (SIEM)
  - Continuous monitoring
  - Incident response plan
  - Forensic logging
```

#### 8.2.2 Critical System Isolation

```
Network Segmentation:

Zone 1 - Critical Control:
  - Navigation systems
  - Propulsion control
  - Emergency systems

Zone 2 - Operations:
  - Monitoring systems
  - Communication
  - Sensor processing

Zone 3 - Business:
  - Crew amenities
  - Administrative systems

Zone 4 - External:
  - Internet gateway (heavily firewalled)
  - Third-party connections

Inter-zone Communication:
  - One-way data diodes where possible
  - Strict firewall rules
  - Protocol whitelisting
  - Deep packet inspection
```

### 8.3 GNSS Spoofing Detection

```
Spoofing Detection Algorithm:

1. Multi-Constellation Consistency:
   IF GPS_position ≠ GLONASS_position:
       ALERT potential_spoofing

2. Position Consistency Check:
   predicted_position = extrapolate(
       previous_position,
       velocity,
       heading,
       Δt
   )

   IF |GNSS_position - predicted_position| > threshold:
       ALERT anomaly_detected

3. Signal Characteristics:
   IF signal_strength > expected OR
      signal_direction != satellite_direction:
       ALERT possible_spoofing

4. Cryptographic Authentication:
   IF available(authenticated_GNSS):
       USE Galileo_OS-NMA OR GPS_M-Code
       VERIFY digital_signature

5. Cross-Validation:
   secondary_position = triangulate(
       visual_landmarks OR
       radar_fixes OR
       celestial_navigation
   )

   IF |GNSS_position - secondary_position| > 100m:
       SWITCH_TO secondary_navigation
       ALERT shore_control
```

### 8.4 Secure Software Updates

```
Update Procedure:

1. Authentication:
   verify_signature(update_package, manufacturer_public_key)

2. Integrity Check:
   computed_hash = SHA-256(update_package)
   IF computed_hash != manifest_hash:
       REJECT update

3. Staging:
   install_to_secondary_partition()

4. Validation:
   boot_from_secondary()
   run_system_tests()

   IF all_tests_pass:
       make_secondary_primary()
       keep_backup_of_previous()
   ELSE:
       rollback_to_primary()
       ALERT update_failed

5. Audit:
   LOG {
       timestamp,
       update_version,
       operator_id,
       verification_status,
       test_results
   }
```

---

## 9. Communication Protocols

### 9.1 Ship-Shore Data Link

#### 9.1.1 Message Protocol

```
Message Structure:

{
    "header": {
        "message_id": "UUID",
        "timestamp": "ISO 8601",
        "priority": 0-3,
        "source": "ship_id | shore_id",
        "destination": "shore_id | ship_id",
        "message_type": "command | data | alert | ack",
        "encryption": "AES-256-GCM",
        "signature": "HMAC-SHA256"
    },
    "payload": {
        // Message-specific data
    },
    "checksum": "CRC32"
}
```

#### 9.1.2 Message Types

```
Command Messages:
- SET_COURSE
- SET_SPEED
- ALTER_ROUTE
- CHANGE_AUTONOMY_LEVEL
- EMERGENCY_STOP
- RETURN_TO_PORT

Data Messages:
- POSITION_REPORT (every 10 seconds)
- SENSOR_DATA (every 1 second)
- SYSTEM_STATUS (every 30 seconds)
- CAMERA_STREAM (continuous)

Alert Messages:
- COLLISION_WARNING
- SYSTEM_FAULT
- WEATHER_ALERT
- SECURITY_INCIDENT
- COMMUNICATION_DEGRADED

Acknowledgment:
- ACK (message received and validated)
- NACK (message rejected - checksum error)
- EXECUTING (command being executed)
- COMPLETED (command executed successfully)
- FAILED (command execution failed)
```

### 9.2 Bandwidth Optimization

#### 9.2.1 Adaptive Data Compression

```
Compression Strategy:

IF bandwidth > 1 Mbps:
    compression_level = LOW
    video_quality = HIGH
    sensor_data_rate = FULL

ELSE IF bandwidth > 500 kbps:
    compression_level = MEDIUM
    video_quality = MEDIUM
    sensor_data_rate = REDUCED

ELSE IF bandwidth > 100 kbps:
    compression_level = HIGH
    video_quality = LOW
    sensor_data_rate = CRITICAL_ONLY

ELSE:  # < 100 kbps
    compression_level = MAXIMUM
    video_quality = OFF
    sensor_data_rate = POSITION_ONLY
    ALERT shore "Degraded communications"
```

#### 9.2.2 Predictive Transmission

```
Smart Data Transmission:

1. Prioritize Critical Data:
   ALWAYS_SEND: [position, heading, speed, collision_alerts]

2. Event-Triggered Transmission:
   SEND_WHEN_CHANGED: [course, route, status]
   SEND_ONLY_IF |Δvalue| > threshold

3. Periodic Heartbeat:
   EVERY 10_seconds: position_report
   EVERY 30_seconds: system_status
   EVERY 5_minutes: full_diagnostics

4. On-Demand:
   SEND_WHEN_REQUESTED: [logs, camera_feeds, detailed_sensor_data]
```

---

## 10. Data Formats

### 10.1 Position Report

```json
{
  "type": "position_report",
  "timestamp": "2025-12-26T12:00:00.000Z",
  "ship": {
    "imo": "9876543",
    "mmsi": "123456789",
    "call_sign": "ABCD"
  },
  "position": {
    "latitude": 35.676192,
    "longitude": 139.650311,
    "altitude": 0.0,
    "accuracy": 2.5
  },
  "motion": {
    "heading": 90.5,
    "course_over_ground": 89.8,
    "speed_over_ground": 15.2,
    "speed_through_water": 15.5,
    "rate_of_turn": 0.5
  },
  "navigation": {
    "status": "under_way_using_engine",
    "destination": "JPYOK",
    "eta": "2025-12-27T06:00:00.000Z",
    "next_waypoint": {
      "latitude": 35.500000,
      "longitude": 140.000000
    },
    "distance_to_waypoint": 25.3,
    "cross_track_error": 0.05
  }
}
```

### 10.2 Collision Warning

```json
{
  "type": "collision_warning",
  "timestamp": "2025-12-26T12:00:00.000Z",
  "severity": "HIGH",
  "target": {
    "mmsi": "987654321",
    "name": "MV EXAMPLE",
    "type": "cargo",
    "position": {
      "latitude": 35.685000,
      "longitude": 139.750000
    },
    "heading": 270.0,
    "speed": 12.0
  },
  "collision_assessment": {
    "cpa": 0.8,
    "cpa_unit": "NM",
    "tcpa": 12.5,
    "tcpa_unit": "minutes",
    "risk_index": 0.85,
    "colreg_situation": "crossing",
    "give_way_vessel": "own_ship"
  },
  "recommended_action": {
    "type": "alter_course",
    "new_heading": 105.0,
    "reason": "COLREG Rule 15 - Crossing situation, target on starboard",
    "execution_time": "2025-12-26T12:02:00.000Z"
  }
}
```

### 10.3 Route Plan

```json
{
  "type": "route_plan",
  "route_id": "RT-20251226-001",
  "created": "2025-12-26T10:00:00.000Z",
  "origin": {
    "name": "Tokyo Port",
    "position": {"latitude": 35.676192, "longitude": 139.650311}
  },
  "destination": {
    "name": "Singapore Port",
    "position": {"latitude": 1.289670, "longitude": 103.850067}
  },
  "waypoints": [
    {
      "id": "WP001",
      "position": {"latitude": 35.500000, "longitude": 140.000000},
      "turn_radius": 0.5,
      "speed": 16.0,
      "eta": "2025-12-26T14:30:00.000Z"
    },
    {
      "id": "WP002",


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

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-3-PROTOCOL.
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
for PHASE-3-PROTOCOL. The profile mechanism is non-normative and exists so that
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
  advertises `WIA-P3-PROTOCOL-Edge-only/2` is asserting conformance with
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
  is published manually. Sufficient for PHASE-3-PROTOCOL validation when the
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
