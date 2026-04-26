# WIA-AUG-014 PHASE 4 — Integration Specification

**Standard:** WIA-AUG-014
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 4 of 4)

---

```typescript
interface DeviceCoordination {
  coordinatorId: string;      // Master device
  participants: string[];     // Device IDs
  synchronization: {
    method: 'timestamp' | 'trigger' | 'sequence';
    precision: number;        // Microseconds
    maxSkew: number;          // Maximum allowed skew
  };
  commands: {
    broadcast: boolean;       // Send to all
    sequential: boolean;      // Ordered execution
    timeout: number;          // ms
  };
}
```

### 8.4 Profile Exchange

```typescript
interface UserProfile {
  profileId: string;
  userId: string;
  created: Date;
  updated: Date;
  calibration: {
    signalBaselines: Map<string, number[]>;
    decoderModels: Map<string, DecoderModel>;
    feedbackPreferences: FeedbackPreferences;
  };
  preferences: {
    latencyPriority: 'low' | 'balanced' | 'quality';
    feedbackIntensity: number;  // 0-1
    adaptiveMode: boolean;
  };
  history: {
    devices: string[];
    sessions: number;
    totalUsageHours: number;
  };
}
```

---

## 9. Implementation Guidelines

### 9.1 Certification Requirements

To achieve WIA-AUG-014 certification:

```
1. Protocol Compliance
   - Implement full protocol stack
   - Pass conformance tests
   - Interoperability verification

2. Latency Compliance
   - Meet tier requirements
   - Jitter within specifications
   - Document latency budget

3. Signal Quality
   - SNR requirements met
   - Artifact handling
   - Error correction

4. Feedback System
   - Supported modalities
   - Timing requirements
   - Safety limits

5. Calibration
   - Standard tasks implemented
   - Quality metrics tracked
   - Profile portability

6. Security
   - Encryption support
   - Authentication
   - Secure pairing
```

### 9.2 Minimum Implementation

```typescript
interface MinimumImplementation {
  required: {
    protocolVersion: string;
    signalTypes: SignalType[];     // At least one
    latencyTier: number;           // Tier 3 minimum
    calibration: 'session';
    encryption: boolean;           // Required
  };
  optional: {
    feedbackTypes: FeedbackType[];
    adaptiveCalibration: boolean;
    profilePortability: boolean;
    multiDeviceCoordination: boolean;
  };
}
```

### 9.3 API Interface

```typescript
// Core SDK Interface
interface HumanMachineInterfaceSDK {
  // Connection
  discover(): Promise<DeviceAdvertisement[]>;
  connect(deviceId: string): Promise<Connection>;
  disconnect(): Promise<void>;

  // Signals
  startAcquisition(config: AcquisitionConfig): void;
  stopAcquisition(): void;
  onSignal(callback: (signal: SignalPacket) => void): void;

  // Commands
  sendCommand(command: MotorCommand): Promise<CommandResult>;
  sendFeedback(feedback: HapticFeedback): Promise<void>;

  // Calibration
  startCalibration(type: CalibrationType): Promise<CalibrationSession>;
  loadProfile(profileId: string): Promise<void>;
  saveProfile(): Promise<string>;

  // State
  getDeviceState(): DeviceState;
  getMetrics(): QualityMetrics;
}
```

---

## 10. References

### 10.1 Scientific Literature

3. Bensmaia, S.J. & Miller, L.E. (2014). Restoring Sensorimotor Function Through Intracortical Interfaces

### 10.2 Standards

- IEEE 2413-2019: Standard for an Architectural Framework for the IoT
- ISO 11073: Health informatics — Medical / health device communication
- IEC 62443: Industrial communication networks — IT security

### 10.3 WIA Standards

- WIA-AUG-013: Augmentation Safety
- WIA-AUG-007: Bionic Limb Standards
- WIA-BCI: Brain-Computer Interface
- WIA-HAPTIC: Haptic Feedback Standards
- WIA-SEC: Security Standards

---

## Appendix A: Signal Type Definitions

```typescript
enum SignalType {
  // Neural (Type A)
  EEG = 'A1',
  ECoG = 'A2',
  LFP = 'A3',
  SINGLE_UNIT = 'A4',

  // Muscular (Type B)
  SURFACE_EMG = 'B1',
  INTRAMUSCULAR_EMG = 'B2',
  MMG = 'B3',

  // Biometric (Type C)
  ECG = 'C1',
  GSR = 'C2',
  RESPIRATION = 'C3',
  TEMPERATURE = 'C4',

  // Motion (Type D)
  ACCELEROMETER = 'D1',
  GYROSCOPE = 'D2',
  MAGNETOMETER = 'D3',
  FORCE = 'D4',

  // Feedback (Type E)
  HAPTIC = 'E1',
  THERMAL = 'E2',
  ELECTRICAL_STIM = 'E3',
  VISUAL_AUDIO = 'E4'
}
```

## Appendix B: Error Codes

```
| Code | Category | Description |
|------|----------|-------------|
| H001 | Connection | Device not found |
| H002 | Connection | Connection timeout |
| H003 | Connection | Authentication failed |
| H004 | Signal | Low signal quality |
| H005 | Signal | Channel failure |
| H006 | Signal | Buffer overflow |
| H007 | Latency | Tier requirement not met |
| H008 | Latency | Excessive jitter |
| H009 | Calibration | Calibration required |
| H010 | Calibration | Calibration failed |
| H011 | Command | Invalid command |
| H012 | Command | Execution timeout |
| H013 | Feedback | Feedback delivery failed |
| H014 | Safety | Safety limit exceeded |
| H015 | Battery | Low battery |
```

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA-AUG-014 Specification v1.0*
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
