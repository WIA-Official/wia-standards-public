# WIA-AUTO-008 PHASE 1 — Data Format Specification

**Standard:** WIA-AUTO-008
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 1 of 4)

---

# WIA-AUTO-008: Connected Car Specification v1.0

> **Standard ID:** WIA-AUTO-008
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Automotive Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Telematics Architecture](#2-telematics-architecture)
3. [Connectivity Technologies](#3-connectivity-technologies)
4. [OTA (Over-the-Air) Updates](#4-ota-over-the-air-updates)
5. [Remote Diagnostics](#5-remote-diagnostics)
6. [Data Collection and Privacy](#6-data-collection-and-privacy)
7. [Cloud Platform Integration](#7-cloud-platform-integration)
8. [Data Formats](#8-data-formats)
9. [API Interface](#9-api-interface)
10. [Security Protocols](#10-security-protocols)
11. [References](#11-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines the technical framework for connected car systems, enabling secure vehicle-to-cloud communication, real-time telemetry, over-the-air updates, and remote diagnostics capabilities.

### 1.2 Scope

The standard covers:
- Vehicle telematics data collection and transmission
- Connectivity protocol specifications
- OTA update procedures and security
- Remote diagnostic capabilities
- Cloud platform integration patterns
- Data privacy and security requirements

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard promotes safer, more efficient, and user-friendly connected vehicles that serve the greater good while respecting privacy and security.

### 1.4 Terminology

- **Telematics**: Remote collection and transmission of vehicle data
- **OTA (Over-the-Air)**: Wireless software/firmware updates
- **TCU (Telematics Control Unit)**: Vehicle's primary connectivity module
- **V2X**: Vehicle-to-Everything communication
- **DTC**: Diagnostic Trouble Code
- **ECU**: Electronic Control Unit
- **VIN**: Vehicle Identification Number

---

## 2. Telematics Architecture

### 2.1 System Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    Connected Vehicle                     │
├─────────────────────────────────────────────────────────┤
│  Sensors & ECUs                                          │
│  ├── Engine Control Unit (ECU)                          │
│  ├── Battery Management System (BMS)                    │
│  ├── GPS/GNSS Receiver                                  │
│  ├── OBD-II Interface                                   │
│  └── Environmental Sensors                              │
├─────────────────────────────────────────────────────────┤
│  Telematics Control Unit (TCU)                          │
│  ├── Data Aggregation                                   │
│  ├── Protocol Translation                               │
│  ├── Security Module                                    │
│  └── Communication Stack                                │
├─────────────────────────────────────────────────────────┤
│  Connectivity Layer                                      │
│  ├── 4G/5G Cellular                                     │
│  ├── WiFi (802.11ac/ax)                                 │
│  ├── Bluetooth (BLE 5.0+)                               │
│  └── V2X (DSRC/C-V2X)                                   │
└─────────────────────────────────────────────────────────┘
                         ↕
┌─────────────────────────────────────────────────────────┐
│                    Cloud Platform                        │
├─────────────────────────────────────────────────────────┤
│  Ingestion Layer                                         │
│  ├── MQTT/AMQP Broker                                   │
│  ├── HTTP/REST Gateway                                  │
│  └── WebSocket Server                                   │
├─────────────────────────────────────────────────────────┤
│  Processing Layer                                        │
│  ├── Stream Processing                                  │
│  ├── Data Validation                                    │
│  ├── Analytics Engine                                   │
│  └── ML/AI Models                                       │
├─────────────────────────────────────────────────────────┤
│  Storage Layer                                           │
│  ├── Time-Series Database                               │
│  ├── Document Store                                     │
│  ├── Object Storage                                     │
│  └── Data Warehouse                                     │
├─────────────────────────────────────────────────────────┤
│  Application Layer                                       │
│  ├── Fleet Management                                   │
│  ├── Remote Diagnostics                                 │
│  ├── OTA Updates                                        │
│  └── User Applications                                  │
└─────────────────────────────────────────────────────────┘
```

### 2.2 Data Flow Architecture

#### 2.2.1 Upstream (Vehicle to Cloud)

```
1. Sensor Reading → ECU Processing
2. ECU → CAN Bus → TCU
3. TCU → Data Aggregation
4. TCU → Data Compression
5. TCU → Encryption (TLS 1.3)
6. TCU → Cellular/WiFi → Cloud
7. Cloud → Ingestion → Validation
8. Cloud → Processing → Storage
```

#### 2.2.2 Downstream (Cloud to Vehicle)

```
1. Cloud → Command Generation
2. Cloud → Authentication & Authorization
3. Cloud → Encryption (TLS 1.3)
4. Cloud → Delivery via Cellular/WiFi
5. TCU → Signature Verification
6. TCU → Command Execution
7. TCU → Acknowledgment → Cloud
8. Cloud → Status Update
```

### 2.3 Telematics Control Unit (TCU) Requirements

#### 2.3.1 Hardware Requirements

- **Processor**: ARM Cortex-A series or equivalent (≥1 GHz dual-core)
- **Memory**: ≥512 MB RAM, ≥4 GB Flash
- **Connectivity**: 4G/5G modem, WiFi, Bluetooth, GNSS
- **Security**: Hardware security module (HSM) or TPM 2.0
- **Interfaces**: CAN, LIN, Ethernet, USB

#### 2.3.2 Software Requirements

- **Operating System**: Linux-based (Automotive Grade Linux recommended)
- **Security**: Secure boot, encrypted storage, certificate management
- **Protocols**: MQTT, HTTPS, WebSocket, CoAP
- **Standards**: ISO 26262 (functional safety), ASPICE compliance

---

## 3. Connectivity Technologies

### 3.1 Cellular (4G/5G)

#### 3.1.1 LTE (4G) Specifications

- **Bands**: Support for regional LTE bands
- **Throughput**:
  - Downlink: Up to 150 Mbps (Cat 4+)
  - Uplink: Up to 50 Mbps
- **Latency**: 30-50 ms typical
- **Use Cases**: Telematics, OTA updates, navigation

#### 3.1.2 5G NR Specifications

- **Bands**: Sub-6 GHz and mmWave support
- **Throughput**:
  - Downlink: Up to 10 Gbps
  - Uplink: Up to 1 Gbps
- **Latency**: <10 ms (URLLC mode: <1 ms)
- **Use Cases**: HD mapping, V2X, streaming, autonomous driving

### 3.2 WiFi (802.11)

#### 3.2.1 WiFi 5 (802.11ac)

- **Frequency**: 5 GHz
- **Bandwidth**: Up to 1.3 Gbps
- **Range**: 50-100 meters (indoor)
- **Use Cases**: Local OTA updates, diagnostics

#### 3.2.2 WiFi 6 (802.11ax)

- **Frequency**: 2.4 GHz / 5 GHz
- **Bandwidth**: Up to 9.6 Gbps (theoretical)
- **Range**: Similar to 802.11ac with better efficiency
- **Use Cases**: High-speed updates, media streaming

### 3.3 Bluetooth Low Energy (BLE)

#### 3.3.1 BLE 5.0+ Specifications

- **Range**: Up to 240 meters (outdoor)
- **Bandwidth**: 2 Mbps
- **Power**: Ultra-low power consumption
- **Use Cases**: Phone pairing, digital key, beacon communication

### 3.4 V2X (Vehicle-to-Everything)

#### 3.4.1 DSRC (Dedicated Short-Range Communications)

- **Frequency**: 5.9 GHz (ITS band)
- **Range**: Up to 300 meters
- **Latency**: <50 ms
- **Bandwidth**: 27 Mbps
- **Use Cases**: V2V collision avoidance, V2I traffic signals

#### 3.4.2 C-V2X (Cellular V2X)

- **Technology**: Based on LTE/5G
- **Modes**:
  - Direct communication (PC5)
  - Network communication (Uu)
- **Range**: Up to 1 km
- **Use Cases**: Advanced safety, cooperative driving, platooning

### 3.5 Satellite (GNSS)

#### 3.5.1 Supported Systems

- **GPS**: US Global Positioning System
- **GLONASS**: Russian satellite navigation
- **Galileo**: European GNSS
- **BeiDou**: Chinese navigation system
- **QZSS**: Japanese regional system

#### 3.5.2 Accuracy Requirements

- **Standard**: ±5 meters (horizontal)
- **DGPS/SBAS**: ±1 meter
- **RTK**: ±2 cm (with corrections)

---

## 4. OTA (Over-the-Air) Updates

### 4.1 Update Types

#### 4.1.1 Software Updates

```
Category: Application Layer
Components:
  - Infotainment system
  - Navigation software
  - Mobile app integration
  - User interface
  - Voice recognition

Size: 100 MB - 2 GB
Frequency: Monthly to quarterly
Risk Level: Low
Rollback: Supported
```

#### 4.1.2 Firmware Updates

```
Category: System Layer
Components:
  - ECU firmware
  - TCU firmware
  - Gateway modules
  - ADAS controllers
  - Battery management

Size: 10 MB - 500 MB
Frequency: Quarterly to annually
Risk Level: Medium to High
Rollback: Required
```

#### 4.1.3 Configuration Updates

```
Category: Settings Layer
Components:
  - Vehicle parameters
  - Feature flags
  - Regional settings
  - User preferences

Size: < 1 MB
Frequency: As needed
Risk Level: Low
Rollback: Optional
```

### 4.2 OTA Update Process

#### 4.2.1 Update Lifecycle

```
Phase 1: Discovery
  └─> Cloud: Publish new update
  └─> Vehicle: Poll for updates (or push notification)
  └─> Vehicle: Check compatibility
  └─> Vehicle: Notify user

Phase 2: Download
  └─> Vehicle: Request update package
  └─> Cloud: Authenticate vehicle
  └─> Cloud: Stream encrypted package
  └─> Vehicle: Verify integrity (hash)
  └─> Vehicle: Store in secure partition

Phase 3: Verification
  └─> Vehicle: Verify digital signature
  └─> Vehicle: Check dependencies
  └─> Vehicle: Validate platform compatibility
  └─> Vehicle: Create rollback point

Phase 4: Installation
  └─> Vehicle: Enter update mode
  └─> Vehicle: Flash new firmware/software
  └─> Vehicle: Perform post-install checks
  └─> Vehicle: Reboot if required

Phase 5: Validation
  └─> Vehicle: Boot with new version
  └─> Vehicle: Run system checks
  └─> Vehicle: Report status to cloud
  └─> Cloud: Mark update complete/failed

Phase 6: Rollback (if needed)
  └─> Vehicle: Detect boot failure
  └─> Vehicle: Restore previous version
  └─> Vehicle: Report failure to cloud
  └─> Cloud: Investigate and retry
```

#### 4.2.2 Update Package Format

```json
{
  "packageId": "PKG-2025-001-ECU-FW",
  "version": "2.5.0",
  "releaseDate": "2025-01-15T00:00:00Z",
  "targetComponent": "engine_ecu",
  "packageType": "firmware",
  "signature": {
    "algorithm": "RSA-4096",
    "value": "base64-encoded-signature",
    "certificate": "base64-encoded-cert"
  },
  "encryption": {
    "algorithm": "AES-256-GCM",
    "keyFingerprint": "sha256-hash"
  },
  "integrity": {
    "algorithm": "SHA-256",
    "checksum": "hex-encoded-hash"
  },
  "compatibility": {
    "vehicleModels": ["Model-X", "Model-Y"],
    "minHardwareVersion": "1.0",
    "dependencies": ["TCU-FW >= 3.0.0"]
  },
  "metadata": {
    "size": 52428800,
    "downloadUrl": "https://ota.example.com/packages/PKG-2025-001",
    "releaseNotes": "Improved fuel efficiency and performance",
    "criticality": "recommended"
  }
}
```

### 4.3 Security Requirements

#### 4.3.1 Code Signing

- **Algorithm**: RSA-4096 or ECDSA P-384
- **Certificate Chain**: Root CA → Intermediate CA → Signing Certificate
- **Validation**: Full chain verification required
- **Expiration**: Monitor certificate validity

#### 4.3.2 Encryption

- **Transport**: TLS 1.3 with perfect forward secrecy
- **Package**: AES-256-GCM
- **Key Management**: Hardware-backed key storage (HSM/TPM)
- **Key Rotation**: Annual or on compromise


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
