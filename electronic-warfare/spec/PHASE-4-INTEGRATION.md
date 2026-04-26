# WIA-DEF-006 PHASE 4 — Integration Specification

**Standard:** WIA-DEF-006
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 4 of 4)

---

## 10. Implementation Guidelines

### 10.1 System Architecture

#### 10.1.1 EW System Components

```
┌─────────────┐
│   Antenna   │
└──────┬──────┘
       │
┌──────▼──────┐
│  RF Front   │
│    End      │
└──────┬──────┘
       │
┌──────▼──────┐
│   Signal    │
│  Processing │
└──────┬──────┘
       │
┌──────▼──────┐
│  Analysis   │
│   Engine    │
└──────┬──────┘
       │
┌──────▼──────┐
│   Control   │
│   System    │
└─────────────┘
```

#### 10.1.2 API Interface

```typescript
interface EWOperation {
  type: 'EA' | 'EP' | 'ES';
  frequency: number;        // Hz
  bandwidth: number;        // Hz
  power: number;           // watts
  duration: number;        // seconds
  mode: string;
}

interface JammingParams {
  targetFrequency: number;  // MHz
  targetDistance: number;   // meters
  targetPower: number;      // watts
  requiredJSRatio: number; // dB
  jammingType: 'noise' | 'deception' | 'protocol';
}

interface SIGINTResult {
  frequency: number;
  bandwidth: number;
  modulation: string;
  signalType: string;
  emitterLocation?: {
    latitude: number;
    longitude: number;
    accuracy: number;
  };
  confidence: number;      // 0-1
}
```

### 10.2 Data Formats

#### 10.2.1 Signal Description

```json
{
  "signal_id": "SIG-20251227-001",
  "timestamp": "2025-12-27T12:00:00Z",
  "frequency": 2450.0,
  "bandwidth": 20.0,
  "power": -65.0,
  "modulation": "OFDM",
  "duration": 1.5,
  "location": {
    "latitude": 37.5665,
    "longitude": 126.9780,
    "altitude": 100
  },
  "classification": {
    "type": "communication",
    "protocol": "802.11ac",
    "confidence": 0.95
  }
}
```

### 10.3 Error Handling

Standard error codes:

| Code | Meaning | Action |
|------|---------|--------|
| EW001 | Insufficient power | Increase transmitter power |
| EW002 | Frequency out of range | Adjust frequency |
| EW003 | Jamming ineffective | Change technique |
| EW004 | Spectrum conflict | Coordinate frequencies |
| EW005 | Safety violation | Emergency shutdown |
| EW006 | Equipment failure | System diagnostics |

---

## 11. Safety Protocols

### 11.1 RF Safety

#### 11.1.1 Maximum Permissible Exposure (MPE)

```
MPE = 180/f² (mW/cm²) for 0.3-3 GHz
MPE = 1.0 (mW/cm²) for 3-15 GHz
MPE = f/15 (mW/cm²) for 15-300 GHz
```

Where `f` = frequency in MHz

#### 11.1.2 Safe Distance

```
R_safe = √(EIRP / (4π × MPE))
```

### 11.2 Operational Safety

1. **Pre-operation Checks**:
   - Verify frequency clearance
   - Confirm no civilian interference
   - Check safety zones
   - Validate ROE compliance

2. **During Operation**:
   - Monitor power levels
   - Track spectrum usage
   - Log all activities
   - Maintain safety margins

3. **Post-operation**:
   - Shutdown verification
   - Effect assessment
   - Incident reporting
   - Equipment inspection

### 11.3 Regulatory Compliance

Adhere to:
- ITU Radio Regulations
- National spectrum policies
- Military frequency allocation
- International humanitarian law

---

## 12. References

### 12.1 Technical Standards

1. IEEE 802.11 - Wireless LAN
2. IEEE 802.16 - Broadband Wireless
3. ITU-R SM.1138 - Spectrum monitoring
4. MIL-STD-461 - EMI requirements
5. NATO STANAG 4357 - EW terminology

### 12.2 Scientific Literature

1. Adamy, D. (2009). "EW 101: A First Course in Electronic Warfare"
2. Poisel, R. (2013). "Modern Communications Jamming"
3. Schleher, D.C. (1999). "Electronic Warfare in the Information Age"
4. Wiley, R.G. (2006). "ELINT: The Interception and Analysis of Radar Signals"

### 12.3 Physical Constants

| Constant | Symbol | Value |
|----------|--------|-------|
| Speed of light | c | 2.998 × 10⁸ m/s |
| Boltzmann constant | k | 1.381 × 10⁻²³ J/K |
| Standard temperature | T₀ | 290 K |
| Free space impedance | Z₀ | 377 Ω |

### 12.4 WIA Standards

- WIA-INTENT: Intent-based system control
- WIA-OMNI-API: Universal API gateway
- WIA-SOCIAL: Coordination protocols
- WIA-QUANTUM: Quantum-resistant communications

---

## Appendix A: Example Calculations

### A.1 Communications Jamming

```
Given:
- Target frequency: 2400 MHz
- Target power: 100 watts
- Target antenna gain: 3 dBi (2.0 linear)
- Target distance: 10 km
- Jammer antenna gain: 6 dBi (4.0 linear)
- Required J/S: 20 dB (100 linear)

Calculation:
Pj = Ps × (Rs/Rj)² × (Gs/Gj) × J/S
Pj = 100 × (10000/5000)² × (2.0/4.0) × 100
Pj = 100 × 4 × 0.5 × 100
Pj = 20,000 watts (20 kW)
```

### A.2 Intercept Range

```
Given:
- EIRP: 1000 watts
- Receiver gain: 10 dBi (10 linear)
- Frequency: 10 GHz (λ = 0.03 m)
- Receiver sensitivity: -100 dBm (1 × 10⁻¹³ W)

Calculation:
R_int = √(EIRP × Gr × λ² / (4π × P_min))
R_int = √(1000 × 10 × 0.03² / (4π × 1×10⁻¹³))
R_int ≈ 260 km
```

---

**弘익人間 (홍익인간) · Benefit All Humanity**

*WIA-DEF-006 Specification v1.0*
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
