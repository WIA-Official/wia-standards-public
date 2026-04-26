# WIA-COMM-013 PHASE 3 — Protocol Specification

**Standard:** WIA-COMM-013
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 3 of 4)

---

## 7. Physical Security

### 7.1 Security Layers

```
Defense in Depth:

Layer 1: Perimeter
├─ Fencing (8-10 ft with barbed wire)
├─ Security lighting
├─ CCTV cameras (360° coverage)
└─ Vehicle barriers (bollards)

Layer 2: Building Entry
├─ Security guard/reception
├─ Mantrap/turnstiles
├─ ID verification
└─ Visitor logging

Layer 3: Data Hall Entry
├─ Biometric access (fingerprint/iris)
├─ Two-factor authentication
├─ Keycards (RFID/NFC)
└─ PIN codes

Layer 4: Cage/Suite
├─ Individual customer cages
├─ Locked cabinets
└─ Surveillance cameras

Layer 5: Server Level
├─ Locked racks
├─ Individual rack access logs
└─ Tamper detection
```

### 7.2 Access Control

**Multi-Factor Authentication:**
1. **Something you have**: Keycard, token
2. **Something you know**: PIN, password
3. **Something you are**: Biometric (fingerprint, iris, facial)

**Access Levels:**
- **Level 1**: Public areas (lobby, restrooms)
- **Level 2**: Office areas
- **Level 3**: Data hall viewing
- **Level 4**: Customer cage access
- **Level 5**: Rack-level access (escort required)

### 7.3 Surveillance

**CCTV Coverage:**
- **Exterior**: 24/7 recording, 90-day retention
- **Entry points**: All doors, mantraps
- **Data halls**: 360° coverage, no blind spots
- **Resolution**: 1080p minimum, 4K preferred
- **Storage**: Redundant NVR systems

---

## 8. Fire Suppression

### 8.1 Detection Systems

**Early Warning Fire Detection (EWFD):**
- **VESDA (Very Early Smoke Detection Apparatus)**
- **Air sampling**: Continuous monitoring
- **Sensitivity**: 100x more sensitive than standard smoke detectors
- **Alert levels**: Alert, Action, Fire 1, Fire 2

### 8.2 Suppression Systems

#### 8.2.1 Clean Agent Systems

**FM-200 (HFC-227ea):**
```
Characteristics:
- Discharge time: <10 seconds
- Concentration: 7-9%
- Safe for electronics
- Safe for occupied spaces
- Hold time: 10 minutes minimum
```

**Novec 1230:**
```
Characteristics:
- Environmentally friendly (0 ODP, low GWP)
- Concentration: 4-6%
- No residue
- Safe for electronics and people
```

**Inergen (IG-541):**
```
Composition: 52% N₂, 40% Ar, 8% CO₂
Characteristics:
- Natural gases
- Concentration: 35-43%
- Environmentally safe
- Requires larger storage
```

#### 8.2.2 Water-Based Systems

**Pre-Action Sprinkler:**
```
Activation:
1. Fire detected (smoke/heat)
2. Pre-action valve opens
3. Water fills pipes
4. Sprinkler head activates (heat)
5. Water discharge

Benefits:
+ Prevents accidental discharge
+ Early warning
+ Time for manual suppression
```

**Dry Pipe Sprinkler:**
```
Use Case: Unheated spaces, freezing risk
Pros: No water in pipes (prevents freezing)
Cons: Slower activation than wet pipe
```

### 8.3 Best Practices

- **Zoning**: Separate zones for data hall, UPS room, generator room
- **EPO (Emergency Power Off)**: Large red button near exits
- **Manual override**: Ability to delay suppression (30-60 sec)
- **Exhaust fans**: Smoke removal after suppression
- **Regular testing**: Quarterly for detection, annual for suppression

---

## 9. DCIM and Monitoring

### 9.1 DCIM (Data Center Infrastructure Management)

**Core Functions:**
1. **Asset Management**: Track all equipment (servers, racks, cables)
2. **Capacity Planning**: Power, cooling, space utilization
3. **Environmental Monitoring**: Temperature, humidity, airflow
4. **Power Monitoring**: Real-time power consumption, PUE
5. **Change Management**: Workflow automation
6. **Visualization**: 3D floor plans, thermal maps

**Leading Platforms:**
- Schneider Electric (EcoStruxure)
- Vertiv (Trellis)
- Nlyte
- Sunbird (dcTrack)
- CA (Nimsoft)

### 9.2 Environmental Monitoring

**Key Sensors:**
```
Temperature Sensors:
- Placement: Every 10 feet, top and bottom of racks
- Accuracy: ±0.5°C
- Range: 0-50°C

Humidity Sensors:
- Placement: Every 20 feet
- Accuracy: ±2% RH
- Range: 10-90% RH

Airflow Sensors:
- Placement: Cold aisle, hot aisle, CRAC/CRAH
- Measurement: CFM (cubic feet per minute)

Water Leak Detection:
- Placement: Under raised floor, around CRAC/CRAH
- Type: Rope sensor or spot detector
```

### 9.3 Power Monitoring

**Metering Points:**
```
Hierarchy:
1. Utility meter → Total facility power
2. UPS input/output → UPS efficiency
3. PDU → Distribution efficiency
4. Rack PDU → Rack-level power
5. Outlet level → Individual device power

Metrics:
- Real power (kW)
- Apparent power (kVA)
- Power factor
- Voltage, current, frequency
- Energy consumption (kWh)
```

### 9.4 Thermal Management

**Thermal Mapping:**
- **CFD (Computational Fluid Dynamics)**: Simulate airflow
- **Thermal cameras**: Identify hot spots
- **RTU (Real-Time Updates)**: Continuous monitoring

**Thresholds:**
- **Cold aisle**: 18-24°C (64-75°F)
- **Hot aisle**: <38°C (100°F)
- **Differential**: 10-15°C (18-27°F)
- **Alarm**: >27°C cold aisle, >40°C hot aisle

---

## 10. Edge Data Centers

### 10.1 Edge vs. Centralized

| Characteristic | Centralized | Edge |
|----------------|-------------|------|
| Location | Metro area | Distributed (near users) |
| Size | 10-100+ MW | 50-500 kW |
| Latency | 10-50 ms | <5 ms |
| Staffing | 24/7 on-site | Remote/automated |
| Redundancy | Tier III-IV | Tier II-III |
| Use Case | Cloud, storage | IoT, 5G, CDN |

### 10.2 Edge Design Considerations

**Form Factors:**
1. **Micro (1-10 kW)**: Single rack, IT closet
2. **Modular (10-100 kW)**: Container, prefab
3. **Mini (100-500 kW)**: Small building

**Challenges:**
- **Limited space**: Compact, high-density design
- **Remote management**: Lights-out operation
- **Harsh environments**: Industrial, outdoor
- **Limited power**: Grid constraints
- **Cooling**: No CRAC/CRAH, direct-expansion

**Solutions:**
- **All-in-one units**: Integrated rack, cooling, UPS
- **Automated**: Self-monitoring, self-healing
- **Ruggedized**: Temperature extremes (-40°C to 55°C)
- **Cellular backup**: 4G/5G connectivity

---


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
