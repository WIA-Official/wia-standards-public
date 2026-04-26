# WIA-AUTO-005 PHASE 4 — Integration Specification

**Standard:** WIA-AUTO-005
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 4 of 4)

---

"model": "Model 3",
  "year": 2024,
  "battery": {
    "capacity": 75,
    "usableCapacity": 72,
    "chemistry": "NMC",
    "warrantyMiles": 120000,
    "warrantYears": 8
  },
  "charging": {
    "connectors": ["CCS", "Tesla"],
    "maxACPower": 11,
    "maxDCPower": 250,
    "maxCurrent": 500,
    "maxVoltage": 400
  },
  "efficiency": {
    "whPerMile": 250,
    "range": 288
  },
  "owner": {
    "userId": "user-abc123",
    "subscriptions": ["ChargeNet", "Electrify America"]
  }
}
```

---

## 9. API Interface

### 9.1 Calculate Charging Time

```typescript
interface ChargingTimeRequest {
  batteryCapacity: number;    // kWh
  currentSOC: number;          // 0-1
  targetSOC: number;           // 0-1
  chargingPower: number;       // kW
  efficiency?: number;         // 0-1, default 0.9
}

interface ChargingTimeResponse {
  hours: number;
  minutes: number;
  energyDelivered: number;     // kWh
  finalSOC: number;            // 0-1
  estimatedCost?: number;      // if pricing available
}
```

### 9.2 Validate Charging Session

```typescript
interface SessionValidation {
  connectorType: ConnectorType;
  vehicleType: 'BEV' | 'PHEV';
  maxPower: number;            // kW
  batteryCapacity: number;     // kWh
  currentSOC?: number;         // 0-1
}

interface ValidationResult {
  isValid: boolean;
  errors: string[];
  warnings: string[];
  compatibility: {
    connector: boolean;
    power: boolean;
    communication: boolean;
  };
  estimatedTime?: ChargingTimeResponse;
}
```

### 9.3 Start Charging Session

```typescript
interface StartSessionRequest {
  stationId: string;
  connectorId: number;
  userId: string;
  vehicleId?: string;
  paymentMethod: PaymentMethod;
  targetSOC?: number;          // 0-1
  departureTime?: Date;
  maxCost?: number;
}

interface StartSessionResponse {
  sessionId: string;
  status: 'started' | 'pending' | 'failed';
  estimatedCost: number;
  estimatedEndTime: Date;
  qrCode?: string;
  deepLink?: string;
}
```

---

## 10. Safety Protocols

### 10.1 Electrical Safety

**Ground Fault Protection:**
```
Fault Current Detection: >20 mA
Response Time: <40 ms
Action: Immediate shutdown and connector lock
```

**Overcurrent Protection:**
```
Max Current: As advertised by EVSE
Tolerance: +10% for 60 seconds, +5% continuous
Trip Level: 120% of rated current
```

**Voltage Monitoring:**
```
Acceptable Range: ±10% of nominal
Under-voltage: <90% for >1 second → shutdown
Over-voltage: >110% for >100ms → immediate shutdown
```

### 10.2 Thermal Management

**Temperature Limits:**
```
Cable: -40°C to 50°C (operating), 85°C (max)
Connector: -40°C to 50°C (operating), 90°C (max)
Inlet: -40°C to 50°C (operating), 105°C (max)

Temperature Monitoring:
- Every second during charging
- Multiple sensors (cable, connector, inlet)
- Derating if temperature >45°C
- Shutdown if temperature >85°C
```

**Cooling Requirements:**
```
Power >150 kW: Liquid cooling required
Power 50-150 kW: Active air cooling recommended
Power <50 kW: Passive cooling acceptable
```

### 10.3 Communication Safety

**Fault Detection:**
```
Control Pilot Signal Lost: Stop charging within 3 seconds
Proximity Detection Lost: Immediate shutdown
Communication Timeout: 30 seconds → graceful shutdown
State Transition Error: Immediate shutdown
```

**Emergency Stop:**
```
Physical E-Stop Button: <1 second response
Software Command: <3 second response
Remote Command: <10 second response
```

### 10.4 Cybersecurity

**Authentication:**
- TLS 1.3 for all communications
- Certificate-based authentication
- Token rotation every 24 hours
- Multi-factor authentication for admin

**Data Protection:**
- Encryption at rest (AES-256)
- Encryption in transit (TLS 1.3)
- PCI DSS compliance for payments
- GDPR compliance for user data

**Firmware Security:**
```
Secure Boot: Verify firmware signature
Encrypted Updates: AES-256 encrypted
Version Control: Rollback protection
Update Authentication: Signed by authorized CA
```

---

## 11. References

### 11.1 Standards

1. **SAE J1772** - AC Level 1 and Level 2 Charging
2. **IEC 62196** - Plugs, socket-outlets, vehicle connectors
3. **IEC 61851** - Electric vehicle conductive charging system
4. **ISO 15118** - Road vehicles - Vehicle to grid communication
5. **CHAdeMO** - DC fast charging protocol
6. **CCS** - Combined Charging System
7. **OCPP 1.6/2.0.1** - Open Charge Point Protocol
8. **SAE J2954** - Wireless Power Transfer (WPT)

### 11.2 Connector Types

| Standard | IEC Code | SAE Code | Region | Type |
|----------|----------|----------|--------|------|
| Type 1 | IEC 62196-2 | SAE J1772 | North America, Japan | AC |
| Type 2 | IEC 62196-2 | - | Europe, Global | AC |
| CCS1 | IEC 62196-3 | SAE J1772 Combo | North America | DC + AC |
| CCS2 | IEC 62196-3 | - | Europe, Global | DC + AC |
| CHAdeMO | - | - | Japan, Asia | DC |
| GB/T | GB/T 20234 | - | China | DC + AC |

### 11.3 Power Levels

| Level | Power Range | Voltage | Use Case | Typical Time |
|-------|-------------|---------|----------|--------------|
| 1 | 1.4-1.9 kW | 120V AC | Home (overnight) | 20-40 hours |
| 2 | 3.3-19.2 kW | 240V AC | Home/Public | 4-8 hours |
| 3 (DC) | 50-350 kW | 400-800V DC | Highway/Fast | 15-45 minutes |

### 11.4 Organizations

- **WIA**: World Certification Industry Association
- **SAE**: Society of Automotive Engineers
- **IEC**: International Electrotechnical Commission
- **ISO**: International Organization for Standardization
- **CharIN**: Charging Interface Initiative
- **OCA**: Open Charge Alliance

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA-AUTO-005 Specification v1.0*
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
