# WIA-AUTO-017 PHASE 4 — Integration Specification

**Standard:** WIA-AUTO-017
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 4 of 4)

---

## 10. API Interface

### 10.1 REST API Endpoints

#### 10.1.1 Create Delivery Mission

```http
POST /api/v1/missions
Content-Type: application/json

{
  "pickup": {
    "location": {"lat": 37.7749, "lng": -122.4194, "alt": 0},
    "address": "123 Market St, San Francisco, CA",
    "contact": {"name": "John Doe", "phone": "+1-555-0100"}
  },
  "dropoff": {
    "location": {"lat": 37.7849, "lng": -122.4094, "alt": 0},
    "address": "456 Mission St, San Francisco, CA",
    "contact": {"name": "Jane Smith", "phone": "+1-555-0200"}
  },
  "package": {
    "weight": 2.5,
    "dimensions": {"length": 30, "width": 20, "height": 15},
    "fragile": false
  },
  "priority": "standard",
  "scheduled_time": "2025-01-01T10:00:00Z"
}

Response:
{
  "mission_id": "MSN-20250101-1234",
  "status": "scheduled",
  "estimated_pickup": "2025-01-01T10:00:00Z",
  "estimated_delivery": "2025-01-01T10:15:00Z",
  "drone_assigned": "WIA-DRN-X1-0042",
  "tracking_url": "https://track.wia.com/MSN-20250101-1234"
}
```

#### 10.1.2 Get Mission Status

```http
GET /api/v1/missions/{mission_id}

Response:
{
  "mission_id": "MSN-20250101-1234",
  "status": "in_flight",
  "current_position": {
    "lat": 37.7799,
    "lng": -122.4144,
    "alt": 35
  },
  "progress": 65,
  "eta": "2025-01-01T10:13:00Z",
  "battery_remaining": 78
}
```

### 10.2 WebSocket Real-Time Updates

```javascript
const ws = new WebSocket('wss://api.wia.com/v1/missions/MSN-20250101-1234/stream');

ws.onmessage = (event) => {
  const update = JSON.parse(event.data);
  console.log(update);
  // {
  //   "type": "position_update",
  //   "timestamp": "2025-01-01T10:05:30Z",
  //   "position": {"lat": 37.7799, "lng": -122.4144, "alt": 35},
  //   "speed": 15.5,
  //   "heading": 45,
  //   "battery": 78
  // }
};
```

### 10.3 SDK Methods

```typescript
interface DeliveryMissionParams {
  pickup: Location;
  dropoff: Location;
  package: Package;
  priority: 'express' | 'standard' | 'economy';
  scheduledTime?: Date;
}

interface MissionResult {
  missionId: string;
  status: MissionStatus;
  estimatedPickup: Date;
  estimatedDelivery: Date;
  droneAssigned: string;
  trackingUrl: string;
}

class DeliveryDroneSDK {
  async createMission(params: DeliveryMissionParams): Promise<MissionResult>;
  async getMissionStatus(missionId: string): Promise<MissionStatus>;
  async cancelMission(missionId: string): Promise<boolean>;
  async trackMission(missionId: string, callback: (update: PositionUpdate) => void): Promise<void>;
}
```

---

## 11. Regulatory Compliance

### 11.1 FAA Part 107 (United States)

Requirements for commercial drone operations:
- Drone weight < 55 lbs (25 kg)
- Maximum altitude: 400 ft AGL
- Maximum speed: 100 mph (44 m/s)
- Visual line of sight (waiver required for BVLOS)
- No operations over people (waiver required)
- Daylight operations only (waiver required for night)
- Remote pilot certificate required

### 11.2 EASA Regulations (European Union)

Categories:
- **Open**: Low risk, < 25 kg, < 120m altitude
- **Specific**: Medium risk, requires authorization
- **Certified**: High risk, requires certification

### 11.3 Remote ID

Broadcast requirements (effective 2023):
```
Required Information (every 1s):
- Drone ID (serial number)
- Operator ID
- Position (lat, lng, alt)
- Velocity
- Emergency status
- Control station location
```

### 11.4 Insurance Requirements

Minimum liability coverage:
- Micro/Light: $1M per occurrence
- Medium: $5M per occurrence
- Heavy: $10M per occurrence

### 11.5 Privacy Compliance

Requirements:
- No recording in private property without consent
- Data encryption for all transmissions
- Video retention: 24 hours maximum (unless incident)
- GDPR/CCPA compliance for customer data
- Camera angle restrictions

---

## 12. References

### 12.1 Standards and Regulations

1. FAA Part 107 - Small Unmanned Aircraft Systems
2. ASTM F3411 - Remote ID and Tracking
3. ASTM F3548 - UAS Traffic Management (UTM)
4. ISO 21384 - Unmanned Aircraft Systems
5. EASA Easy Access Rules for Unmanned Aircraft Systems

### 12.2 Technical References

1. "Multirotor Aerial Vehicles" - Mahony et al.
2. "Principles of Helicopter Aerodynamics" - J. Gordon Leishman
3. "Planning Algorithms" - Steven M. LaValle
4. "Probabilistic Robotics" - Thrun, Burgard, Fox
5. "Computer Vision: Algorithms and Applications" - Richard Szeliski

### 12.3 Physics Constants

| Constant | Symbol | Value |
|----------|--------|-------|
| Gravitational acceleration | g | 9.81 m/s² |
| Air density (sea level) | ρ | 1.225 kg/m³ |
| Speed of sound (sea level) | c | 343 m/s |
| Standard atmospheric pressure | P0 | 101,325 Pa |

### 12.4 WIA Standards

- WIA-INTENT: Intent-based interfaces
- WIA-OMNI-API: Universal API gateway
- WIA-SOCIAL: Social coordination protocols
- WIA-AIR-SHIELD: Airspace security and protection
- WIA-QUANTUM: Secure communication encryption

---

## Appendix A: Example Calculations

### A.1 Hover Power for Light Class Drone

```
Given:
- Mass: 8 kg (drone 5kg + payload 3kg)
- Rotor diameter: 0.3m (4 rotors)
- Air density: 1.225 kg/m³

Calculation:
- Total rotor area: A = 4 × π × (0.15)² = 0.283 m²
- Power: P = (8 × 9.81)^(3/2) / √(2 × 1.225 × 0.283)
- P = (78.48)^(1.5) / √(0.693)
- P = 696.7 / 0.832
- P ≈ 837W

Including efficiency (0.6):
- Actual power ≈ 1395W
```

### A.2 Maximum Range Calculation

```
Given:
- Battery: 14.8V, 10Ah = 148Wh
- Average power: 500W
- Cruise speed: 15 m/s
- Efficiency: 0.8
- Reserve: 30%

Calculation:
- Usable energy: 148 × 0.7 = 103.6 Wh
- Flight time: (103.6 × 0.8) / 500 = 0.166 hours ≈ 10 minutes
- Range: 0.166 × 3600 × 15 = 8964m ≈ 9km

With wind (5 m/s headwind):
- Effective speed: 15 - 5 = 10 m/s
- Range: 0.166 × 3600 × 10 = 5976m ≈ 6km
```

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA-AUTO-017 Specification v1.0*
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
