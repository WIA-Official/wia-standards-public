# WIA-COMM-011 PHASE 1 — Data Format Specification

**Standard:** WIA-COMM-011
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 1 of 4)

---

# WIA-COMM-011: Edge Computing Specification v1.0

> **Standard ID:** WIA-COMM-011
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Edge Computing Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Edge Computing Architecture](#2-edge-computing-architecture)
3. [Multi-access Edge Computing (MEC)](#3-multi-access-edge-computing-mec)
4. [Fog Computing Model](#4-fog-computing-model)
5. [Edge AI and ML Inference](#5-edge-ai-and-ml-inference)
6. [Latency Optimization](#6-latency-optimization)
7. [Data Locality and Privacy](#7-data-locality-and-privacy)
8. [Edge-Cloud Orchestration](#8-edge-cloud-orchestration)
9. [Container Orchestration at Edge](#9-container-orchestration-at-edge)
10. [Edge Security](#10-edge-security)
11. [Resource Management](#11-resource-management)
12. [Workload Placement](#12-workload-placement)
13. [5G Edge Integration](#13-5g-edge-integration)
14. [Industrial Edge Applications](#14-industrial-edge-applications)
15. [Performance Requirements](#15-performance-requirements)
16. [Implementation Guidelines](#16-implementation-guidelines)
17. [References](#17-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines the Edge Computing standard, covering Multi-access Edge Computing (MEC), fog computing, edge AI/ML inference, and integration with 5G networks to enable ultra-low-latency processing at the network edge.

### 1.2 Scope

The standard covers:
- Edge computing architecture and deployment models
- MEC framework and APIs
- Fog computing hierarchy
- Edge AI/ML optimization techniques
- Latency reduction strategies (<5ms target)
- Data locality and privacy preservation
- Edge-cloud orchestration and workload placement
- Container orchestration (Kubernetes at edge)
- Edge security and zero-trust architecture
- 5G and network integration

### 1.3 Philosophy

**弘익人間 (Benefit All Humanity)** - Edge computing brings intelligence and processing power closer to users, reducing latency, improving privacy, enabling real-time applications, and democratizing access to advanced computing capabilities.

### 1.4 Terminology

- **Edge Computing**: Computing performed at or near the data source
- **MEC**: Multi-access Edge Computing (ETSI standard)
- **Fog Computing**: Hierarchical computing from device to cloud
- **Edge Node**: Computing resource at network edge
- **Edge AI**: Artificial intelligence running on edge devices
- **Workload Placement**: Deciding where to execute tasks (edge vs cloud)
- **K3s**: Lightweight Kubernetes for edge
- **Edge TPU**: Tensor Processing Unit for edge AI
- **RAN**: Radio Access Network
- **NFV**: Network Functions Virtualization

---

## 2. Edge Computing Architecture

### 2.1 Architectural Layers

Edge computing consists of multiple hierarchical layers:

```
┌─────────────────────────────────────────────┐
│          Cloud Core (Centralized)           │
│  - Large-scale data processing              │
│  - ML model training                        │
│  - Long-term storage                        │
└─────────────────────────────────────────────┘
                    ▲
                    │ High bandwidth
                    │ 50-100ms latency
                    ▼
┌─────────────────────────────────────────────┐
│       Regional Edge (Metro Area)            │
│  - Content delivery                         │
│  - Data aggregation                         │
│  - ML inference                             │
└─────────────────────────────────────────────┘
                    ▲
                    │ Medium bandwidth
                    │ 10-50ms latency
                    ▼
┌─────────────────────────────────────────────┐
│         Access Edge (Local)                 │
│  - Base stations (5G/6G)                    │
│  - Local caching                            │
│  - Real-time processing                     │
└─────────────────────────────────────────────┘
                    ▲
                    │ High bandwidth
                    │ <5ms latency
                    ▼
┌─────────────────────────────────────────────┐
│       Device Edge (End Points)              │
│  - IoT devices                              │
│  - Smartphones                              │
│  - Edge gateways                            │
└─────────────────────────────────────────────┘
```

### 2.2 Edge Node Classification

| Category | Location | Latency to Device | Compute Power | Typical Use |
|----------|----------|------------------|---------------|-------------|
| Device Edge | On device | <1ms | Low (mW-W) | Sensor processing |
| Access Edge | Cell tower/AP | 1-5ms | Medium (100W-1kW) | Video analytics |
| Regional Edge | City/Metro | 5-20ms | High (10-100kW) | Content delivery |
| Cloud Edge | Data center | 20-50ms | Very high (MW+) | Batch processing |

### 2.3 Deployment Models

1. **On-Premises Edge**: Customer-owned infrastructure
2. **Operator Edge**: Telco-hosted at base stations
3. **Cloud Provider Edge**: AWS Wavelength, Azure Edge Zones
4. **Hybrid Edge**: Combination of above models
5. **Mobile Edge**: Vehicle-mounted or portable edge

---

## 3. Multi-access Edge Computing (MEC)

### 3.1 MEC Architecture (ETSI Standard)

MEC provides cloud computing capabilities at the edge of mobile networks:

```
┌──────────────────────────────────────────────────┐
│         MEC Application Layer                     │
│  ┌────────────┐  ┌────────────┐  ┌────────────┐ │
│  │   App 1    │  │   App 2    │  │   App 3    │ │
│  │ (AR/VR)    │  │ (Video)    │  │  (IoT)     │ │
│  └────────────┘  └────────────┘  └────────────┘ │
└──────────────────────────────────────────────────┘
                    ▲
                    │ MEC APIs
                    ▼
┌──────────────────────────────────────────────────┐
│         MEC Platform Layer                        │
│  - Service discovery                              │
│  - DNS handling                                   │
│  - Traffic management                             │
│  - Location services                              │
│  - Radio Network Information                      │
└──────────────────────────────────────────────────┘
                    ▲
                    │
                    ▼
┌──────────────────────────────────────────────────┐
│       Virtualization Infrastructure               │
│  - Compute (VMs/Containers)                       │
│  - Storage                                        │
│  - Network                                        │
└──────────────────────────────────────────────────┘
```

### 3.2 MEC Services

1. **Radio Network Information Service (RNIS)**
   - Real-time radio conditions
   - Cell load information
   - User location tracking

2. **Location Service**
   - Device geolocation
   - Proximity detection
   - Geo-fencing

3. **Bandwidth Management**
   - QoS allocation
   - Traffic prioritization
   - Congestion control

4. **Application Lifecycle Management**
   - Deployment
   - Scaling
   - Migration
   - Termination

### 3.3 MEC API Examples

```json
{
  "mecService": "RadioNetworkInformation",
  "request": {
    "associateId": "ue-12345",
    "type": "cell_info"
  },
  "response": {
    "ecgi": "310-410-0x1234",
    "cellLoad": 45,
    "signalStrength": -75,
    "latency": 3.2
  }
}
```

---

## 4. Fog Computing Model

### 4.1 Fog Computing Hierarchy

Fog computing extends cloud computing to the edge with multiple tiers:

```
Cloud Tier (Days-Weeks processing)
  └── Regional Fog (Hours processing)
      └── Local Fog (Minutes processing)
          └── Edge Fog (Seconds processing)
              └── Device Tier (Real-time ms processing)
```

### 4.2 Fog Node Characteristics

| Property | Cloud | Regional Fog | Local Fog | Edge Fog | Device |
|----------|-------|--------------|-----------|----------|--------|
| Latency | 50-100ms | 20-50ms | 5-20ms | 1-5ms | <1ms |
| Storage | Unlimited | TB-PB | GB-TB | MB-GB | KB-MB |
| Processing | Unlimited | High | Medium | Low | Minimal |
| Mobility | Static | Static | Static | Mobile | Mobile |
| Location Awareness | No | Partial | Yes | Yes | Yes |

### 4.3 Data Flow Management

Data flows through fog tiers based on requirements:

- **Real-time data**: Processed at edge/device (e.g., collision avoidance)
- **Near-real-time**: Processed at local fog (e.g., traffic analysis)
- **Batch data**: Processed at regional fog or cloud (e.g., analytics)

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
