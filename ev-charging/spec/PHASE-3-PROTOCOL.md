# WIA-AUTO-005 PHASE 3 — Protocol Specification

**Standard:** WIA-AUTO-005
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 3 of 4)

---

## 6. Vehicle-to-Grid (V2G)

### 6.1 Bidirectional Charging

**Capabilities:**
- Export power from vehicle to grid
- Frequency regulation
- Peak shaving
- Emergency backup power

**Technical Requirements:**
```
Connector: CHAdeMO or CCS2 (with V2G support)
Protocol: ISO 15118-20
Inverter: Bidirectional DC/AC
Max Discharge Power: 10-100 kW (vehicle dependent)
```

**Power Flow:**
```
Grid → Charger → Vehicle (G2V): Charging
Vehicle → Charger → Grid (V2G): Discharging
Vehicle → Charger → Home (V2H): Home backup
Vehicle → Charger → Building (V2B): Building support
```

### 6.2 Grid Services

**Frequency Regulation:**
```
Grid Frequency: 60 Hz (NA) / 50 Hz (EU)
Tolerance: ±0.05 Hz (normal), ±0.2 Hz (emergency)

If Frequency < 59.95 Hz:
    Discharge 20% of available battery power
Else If Frequency > 60.05 Hz:
    Charge at 20% of max charging power
Else:
    Maintain current state
```

**Revenue Model:**
```
V2G Revenue = (Energy Discharged × Sell Price) - (Energy Charged × Buy Price) - Degradation Cost

Example:
Energy Discharged: 10 kWh @ $0.50/kWh = $5.00
Energy Charged: 12 kWh @ $0.10/kWh = $1.20 (includes losses)
Degradation: 10 kWh × $0.05/kWh = $0.50
Net Revenue: $5.00 - $1.20 - $0.50 = $3.30
```

### 6.3 Battery Degradation Management

**Cycle Counting:**
```
Full Cycle = 100% DOD (Depth of Discharge)
Partial Cycle Credit = (SOC_start - SOC_end) / 100

Example:
Charge from 40% to 80%: 0.4 cycles
Discharge from 80% to 60%: 0.2 cycles
Total: 0.6 cycles
```

**Degradation Limits:**
```
Max Daily Cycles: 2.0 full cycles
Max DOD per Cycle: 80%
Preferred SOC Range: 20-80%
Calendar Aging Factor: 0.02% per month
```

---

## 7. Billing and Payment

### 7.1 Pricing Models

**Energy-Based Pricing:**
```
Cost = Energy (kWh) × Price ($/kWh)

Example:
30 kWh @ $0.35/kWh = $10.50
```

**Time-Based Pricing:**
```
Cost = Time (minutes) × Price ($/minute)

Example:
45 minutes @ $0.25/minute = $11.25
```

**Hybrid Pricing:**
```
Cost = Session Fee + (Energy × Energy Price) + (Time × Time Price)

Example:
Session: $2.00
Energy: 30 kWh @ $0.25/kWh = $7.50
Time: 45 min @ $0.05/min = $2.25
Total: $11.75
```

**Idle Fees:**
```
If session completed and vehicle still connected:
    First 10 minutes: Free
    After 10 minutes: $0.50/minute
```

### 7.2 Payment Methods

**Supported Methods:**
1. **RFID Card**: Touch and charge
2. **Mobile App**: QR code or NFC
3. **Credit Card**: EMV chip or contactless
4. **Plug & Charge**: ISO 15118 automatic billing
5. **Subscription**: Monthly plans

**Transaction Flow:**
```
1. Authentication
   ↓
2. Authorization (payment method validation)
   ↓
3. Charging session start
   ↓
4. Real-time metering
   ↓
5. Session end
   ↓
6. Final billing calculation
   ↓
7. Payment processing
   ↓
8. Receipt generation
```

### 7.3 Roaming and Interoperability

**Roaming Protocols:**
- **OCPI** (Open Charge Point Interface)
- **eMIP** (eMobility Inter-operation Protocol)
- **OICP** (Open InterCharge Protocol)

**Roaming Fee Structure:**
```
Driver Payment → Home Operator → Roaming Hub → Host Operator → Charge Point

Fee Split Example:
Total Cost: $15.00
Host Operator: $12.00 (80%)
Roaming Hub: $1.50 (10%)
Home Operator: $1.50 (10%)
```

---

## 8. Data Formats

### 8.1 Charging Session Record

```json
{
  "sessionId": "CS-2025-12-26-001234",
  "stationId": "EVSE-NYC-001",
  "connectorId": 1,
  "connectorType": "CCS",
  "vehicleId": "VIN-1HGBH41JXMN109186",
  "userId": "user-abc123",
  "startTime": "2025-12-26T10:00:00Z",
  "endTime": "2025-12-26T10:45:00Z",
  "startSOC": 25,
  "endSOC": 80,
  "energyDelivered": 42.5,
  "averagePower": 56.7,
  "peakPower": 150,
  "chargingCurve": [
    {"time": "10:00", "power": 150, "soc": 25},
    {"time": "10:15", "power": 145, "soc": 45},
    {"time": "10:30", "power": 120, "soc": 65},
    {"time": "10:45", "power": 80, "soc": 80}
  ],
  "pricing": {
    "energyPrice": 0.35,
    "timePrice": 0.05,
    "sessionFee": 2.00,
    "idleFee": 0,
    "totalCost": 18.88
  },
  "payment": {
    "method": "credit_card",
    "cardLast4": "4242",
    "transactionId": "txn_1234567890",
    "status": "completed"
  }
}
```

### 8.2 Charging Station Status

```json
{
  "stationId": "EVSE-NYC-001",
  "location": {
    "latitude": 40.7128,
    "longitude": -74.0060,
    "address": "123 Main St, New York, NY 10001"
  },
  "operator": "ChargeNet USA",
  "connectors": [
    {
      "connectorId": 1,
      "type": "CCS",
      "maxPower": 150,
      "status": "Available",
      "pricing": {
        "currency": "USD",
        "energyPrice": 0.35,
        "sessionFee": 2.00
      }
    },
    {
      "connectorId": 2,
      "type": "CHAdeMO",
      "maxPower": 50,
      "status": "Charging",
      "currentSession": {
        "startTime": "2025-12-26T09:30:00Z",
        "currentSOC": 65,
        "estimatedEndTime": "2025-12-26T10:15:00Z"
      }
    }
  ],
  "amenities": ["WiFi", "Restroom", "Coffee"],
  "access": "24/7",
  "lastUpdated": "2025-12-26T10:30:00Z"
}
```

### 8.3 Vehicle Information

```json
{
  "vehicleId": "VIN-1HGBH41JXMN109186",
  "make": "Tesla",


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
