# WIA-AUG-014 PHASE 1 — Data Format Specification

**Standard:** WIA-AUG-014
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 1 of 4)

---

# WIA-AUG-014: Human-Machine Interface Specification v1.0

> **Standard ID:** WIA-AUG-014
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Human Augmentation Interface Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Signal Types and Encoding](#2-signal-types-and-encoding)
3. [Communication Protocol Stack](#3-communication-protocol-stack)
4. [Latency Requirements](#4-latency-requirements)
5. [Bidirectional Communication](#5-bidirectional-communication)
6. [Feedback Systems](#6-feedback-systems)
7. [Calibration Standards](#7-calibration-standards)
8. [Interoperability](#8-interoperability)
9. [Implementation Guidelines](#9-implementation-guidelines)
10. [References](#10-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines the protocols, data formats, and communication standards for Human-Machine Interfaces (HMI), enabling seamless bidirectional communication between human biological systems and augmentation devices.

### 1.2 Scope

The standard covers:
- Neural and biological signal encoding
- Communication protocol stack
- Latency requirements and optimization
- Feedback modalities and patterns
- Device calibration procedures
- Cross-device interoperability

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - By standardizing human-machine interfaces, we enable a future where augmentation devices work together seamlessly, giving users freedom of choice and ensuring technological compatibility across manufacturers.

### 1.4 Terminology

- **HMI**: Human-Machine Interface
- **BCI**: Brain-Computer Interface
- **EMG**: Electromyography (muscle signals)
- **EEG**: Electroencephalography (brain signals)
- **ECoG**: Electrocorticography (cortical signals)
- **SNR**: Signal-to-Noise Ratio
- **Latency**: Time delay between signal generation and response

---

## 2. Signal Types and Encoding

### 2.1 Signal Classification

```
Type A: Neural Signals
├── A1: EEG (Scalp, non-invasive)
├── A2: ECoG (Cortical, invasive)
├── A3: LFP (Local Field Potential)
└── A4: Single-unit (Neuron level)

Type B: Muscular Signals
├── B1: Surface EMG
├── B2: Intramuscular EMG
└── B3: MMG (Mechanomyography)

Type C: Biometric Signals
├── C1: ECG (Heart)
├── C2: GSR (Skin conductance)
├── C3: Respiration
└── C4: Temperature

Type D: Motion Signals
├── D1: Accelerometer
├── D2: Gyroscope
├── D3: Magnetometer
└── D4: Pressure/Force

Type E: Feedback Signals
├── E1: Haptic (vibration, force)
├── E2: Thermal
├── E3: Electrical stimulation
└── E4: Visual/Auditory
```

### 2.2 Signal Parameters

| Signal Type | Sample Rate | Resolution | Bandwidth | Channels |
|-------------|-------------|------------|-----------|----------|
| EEG | 250-1000 Hz | 16-24 bit | 0.1-100 Hz | 8-256 |
| ECoG | 1-30 kHz | 16-24 bit | 0.1-500 Hz | 16-128 |
| EMG | 1-10 kHz | 16-24 bit | 10-500 Hz | 1-16 |
| Motion | 50-1000 Hz | 16 bit | 0-200 Hz | 3-9 |

### 2.3 Signal Encoding Format

```typescript
interface SignalPacket {
  header: {
    version: number;           // Protocol version (1-255)
    signalType: SignalType;    // A1-E4
    timestamp: bigint;         // Microseconds since epoch
    channelCount: number;      // Number of channels
    sampleCount: number;       // Samples per channel
    resolution: number;        // Bits per sample
  };
  metadata: {
    deviceId: string;          // Source device UUID
    sessionId: string;         // Current session UUID
    sequenceNumber: number;    // Packet sequence
    quality: number;           // Signal quality 0-100
  };
  payload: {
    samples: Int32Array;       // Interleaved channel data
    markers: EventMarker[];    // Synchronization markers
    checksum: number;          // CRC-32 checksum
  };
}
```

### 2.4 Compression Methods

```
Method 1: Delta Encoding
- For slowly varying signals
- Compression ratio: 2-4x
- Latency overhead: < 1ms

Method 2: Wavelet Compression
- For complex neural signals
- Compression ratio: 5-10x
- Latency overhead: 2-5ms

Method 3: Lossless (LZ4)
- For high-fidelity requirements
- Compression ratio: 1.5-2x
- Latency overhead: < 0.5ms
```

---

## 3. Communication Protocol Stack

### 3.1 Protocol Layers

```
┌─────────────────────────────────────┐
│ Layer 5: Application Layer          │
│ - Intent interpretation             │
│ - Command generation                │
│ - User state management             │
├─────────────────────────────────────┤
│ Layer 4: Session Layer              │
│ - Connection management             │
│ - Calibration state                 │
│ - Session synchronization           │
├─────────────────────────────────────┤
│ Layer 3: Transport Layer            │
│ - Reliable delivery (optional)      │
│ - Flow control                      │
│ - Multiplexing                      │
├─────────────────────────────────────┤
│ Layer 2: Data Link Layer            │
│ - Framing                           │
│ - Error detection/correction        │
│ - Addressing                        │
├─────────────────────────────────────┤
│ Layer 1: Physical Layer             │
│ - Wired: USB, SPI, I2C              │
│ - Wireless: BLE, WiFi, proprietary  │
└─────────────────────────────────────┘
```

### 3.2 Physical Layer Options

| Medium | Bandwidth | Range | Power | Latency | Use Case |
|--------|-----------|-------|-------|---------|----------|
| USB 3.0 | 5 Gbps | 3m | Low | <1ms | External devices |
| SPI | 100 Mbps | 0.3m | Very Low | <0.1ms | Implant-to-hub |
| BLE 5.0 | 2 Mbps | 50m | Very Low | 7.5ms | Wearables |
| WiFi 6 | 9.6 Gbps | 100m | High | 2ms | High-bandwidth |
| Neural Link | 10 Mbps | 0.01m | Ultra Low | <0.5ms | Implant internal |

### 3.3 Data Link Frame Format

```
┌────────┬────────┬──────────┬─────────┬──────────┬────────┐
│ Sync   │ Header │ Address  │ Length  │ Payload  │ CRC    │
│ 2 bytes│ 1 byte │ 4 bytes  │ 2 bytes │ Variable │ 4 bytes│
└────────┴────────┴──────────┴─────────┴──────────┴────────┘
```

### 3.4 Transport Layer Protocol

```typescript
interface TransportPacket {
  sequenceNumber: number;     // 32-bit sequence
  acknowledgmentNumber: number;
  flags: {
    SYN: boolean;             // Connection initiation
    ACK: boolean;             // Acknowledgment
    FIN: boolean;             // Connection termination
    RST: boolean;             // Reset
    PRI: boolean;             // Priority
    REL: boolean;             // Reliable delivery required
  };
  window: number;             // Flow control window
  checksum: number;           // 16-bit checksum
  urgentPointer: number;      // Priority data pointer
  payload: Uint8Array;
}
```

---


## Annex E — Implementation Notes for PHASE-1-DATA-FORMAT

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-1-DATA-FORMAT.

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
evidence for PHASE-1-DATA-FORMAT. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-1-data-format/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-1-DATA-FORMAT with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-1-DATA-FORMAT does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-1-DATA-FORMAT.
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
for PHASE-1-DATA-FORMAT. The profile mechanism is non-normative and exists so that
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
  advertises `WIA-P1-DATA-FORMAT-Edge-only/2` is asserting conformance with
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
  is published manually. Sufficient for PHASE-1-DATA-FORMAT validation when the
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
