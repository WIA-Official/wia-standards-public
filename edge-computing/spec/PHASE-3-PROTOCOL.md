# WIA-COMM-011 PHASE 3 — Protocol Specification

**Standard:** WIA-COMM-011
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 3 of 4)

---

## 9. Container Orchestration at Edge

### 9.1 Lightweight Kubernetes (K3s)

K3s is optimized for edge deployment:

```
K3s vs K8s:
- Binary size: 70MB vs 1GB
- Memory: 512MB vs 2GB
- Startup time: <30s vs 2-5min
- Dependencies: Minimal vs Many
```

### 9.2 Edge Container Platform

```
┌──────────────────────────────────────────────┐
│         Container Registry (Harbor)          │
│  - Image caching                             │
│  - Vulnerability scanning                    │
└──────────────────────────────────────────────┘
                    │
                    ▼
┌──────────────────────────────────────────────┐
│       K3s Cluster (Edge Nodes)               │
│  ┌─────────────────────────────────────┐    │
│  │  Control Plane (Master)             │    │
│  │  - API server                       │    │
│  │  - Scheduler                        │    │
│  │  - etcd (SQLite in K3s)             │    │
│  └─────────────────────────────────────┘    │
│                                               │
│  ┌─────────┐  ┌─────────┐  ┌─────────┐     │
│  │ Worker  │  │ Worker  │  │ Worker  │     │
│  │ Node 1  │  │ Node 2  │  │ Node 3  │     │
│  │         │  │         │  │         │     │
│  │ Pods    │  │ Pods    │  │ Pods    │     │
│  └─────────┘  └─────────┘  └─────────┘     │
└──────────────────────────────────────────────┘
```

### 9.3 Edge-Optimized Features

1. **Resource Management**
   - CPU/Memory limits
   - GPU sharing
   - Priority classes

2. **Storage**
   - Local volumes
   - Distributed storage (Longhorn)
   - Edge CDN integration

3. **Networking**
   - Host networking for low latency
   - Service mesh (Linkerd)
   - Edge ingress

4. **Deployment Strategies**
   - Blue-green deployment
   - Canary releases
   - Rollback capabilities

---

## 10. Edge Security

### 10.1 Zero-Trust Architecture

```
┌──────────────────────────────────────────────┐
│         Identity & Access Management         │
│  - Device authentication                     │
│  - Mutual TLS (mTLS)                         │
│  - Certificate rotation                      │
└──────────────────────────────────────────────┘
                    │
                    ▼
┌──────────────────────────────────────────────┐
│         Edge Security Gateway                │
│  - Firewall                                  │
│  - IDS/IPS                                   │
│  - DDoS protection                           │
└──────────────────────────────────────────────┘
                    │
                    ▼
┌──────────────────────────────────────────────┐
│       Application Security (Edge)            │
│  - Container isolation                       │
│  - Secrets management                        │
│  - Runtime protection                        │
└──────────────────────────────────────────────┘
```

### 10.2 Security Threats at Edge

| Threat | Impact | Mitigation |
|--------|--------|------------|
| Physical tampering | High | Secure boot, TPM |
| Network attacks | High | Encryption, segmentation |
| Malware | High | Signed containers, scanning |
| Data theft | High | Encryption at rest/transit |
| DDoS | Medium | Rate limiting, edge filtering |
| Supply chain | High | Verified images, SBOM |

### 10.3 Security Best Practices

1. **Device Security**
   - Secure boot (UEFI)
   - TPM 2.0 for key storage
   - Firmware integrity

2. **Network Security**
   - mTLS for all communication
   - Network segmentation
   - Encrypted tunnels (WireGuard)

3. **Application Security**
   - Signed container images
   - Minimal base images
   - Regular vulnerability scanning

4. **Data Security**
   - Encryption at rest (AES-256)
   - Encryption in transit (TLS 1.3)
   - Key rotation policies

---

## 11. Resource Management

### 11.1 Resource Constraints

Edge nodes typically have limited resources:

```
Typical Edge Node:
- CPU: 4-8 cores (vs 32+ in cloud)
- RAM: 8-16 GB (vs 64+ GB in cloud)
- Storage: 128-512 GB SSD (vs TB+ in cloud)
- GPU: 0-1 (vs 4-8 in cloud)
- Power: 50-200W (vs kW+ in cloud)
- Network: 100 Mbps - 10 Gbps (vs 100+ Gbps in cloud)
```

### 11.2 Resource Allocation Strategies

1. **CPU Scheduling**
   - Real-time priority for critical tasks
   - CPU pinning for consistent latency
   - CPU quotas to prevent starvation

2. **Memory Management**
   - Reserved memory for system
   - Memory limits per container
   - OOM (Out-of-Memory) policies

3. **Storage Optimization**
   - Image layer caching
   - Ephemeral storage for temporary data
   - Log rotation

4. **GPU Sharing**
   - Time-slicing for multiple workloads
   - MIG (Multi-Instance GPU) on supported hardware
   - GPU passthrough for dedicated access

### 11.3 Resource Monitoring

```yaml
resources:
  requests:
    cpu: "500m"        # 0.5 CPU core
    memory: "1Gi"      # 1 GiB RAM
    nvidia.com/gpu: 1  # 1 GPU
  limits:
    cpu: "2000m"       # Max 2 CPU cores
    memory: "4Gi"      # Max 4 GiB RAM
    nvidia.com/gpu: 1  # Max 1 GPU
```

---

## 12. Workload Placement

### 12.1 Placement Decision Factors

1. **Latency Requirements**
   - <1ms: Device only
   - 1-5ms: Access edge
   - 5-20ms: Regional edge
   - >20ms: Cloud acceptable

2. **Data Sensitivity**
   - Highly sensitive: Edge only
   - Moderate: Encrypted cloud
   - Public: Cloud or edge

3. **Computational Complexity**
   - Simple: Device/edge
   - Moderate: Edge
   - Complex: Cloud or powerful edge

4. **Network Bandwidth**
   - High bandwidth data: Process at edge
   - Low bandwidth: Cloud acceptable

### 12.2 Placement Algorithms

1. **Latency-Optimal Placement**
   - Place workload at nearest edge with capacity
   - Minimize end-to-end latency
   - Cost: Secondary consideration

2. **Cost-Optimal Placement**
   - Prefer cheaper cloud resources
   - Use edge only when latency requires
   - Cost: Primary consideration

3. **Hybrid Placement**
   - Balance latency and cost
   - Dynamic migration based on load
   - Multi-objective optimization

### 12.3 Workload Migration

```
Migration Triggers:
- Edge node failure
- Resource exhaustion
- Network congestion
- Planned maintenance
- Load balancing

Migration Process:
1. Select target node
2. Snapshot state (if stateful)
3. Transfer state to target
4. Start workload on target
5. Redirect traffic
6. Verify operation
7. Terminate source
```

---

## 13. 5G Edge Integration

### 13.1 5G Architecture with MEC

```
┌──────────────────────────────────────────────┐
│         5G Core (Cloud)                      │
│  - AMF, SMF, UPF                             │
│  - Centralized control                       │
└──────────────────────────────────────────────┘
                    │
                    ▼
┌──────────────────────────────────────────────┐
│       MEC Platform (Edge UPF)                │
│  - Local breakout                            │
│  - Traffic steering                          │
│  - QoS enforcement                           │
└──────────────────────────────────────────────┘
                    │
                    ▼
┌──────────────────────────────────────────────┐
│         gNB (5G Base Station)                │
│  - Radio interface                           │
│  - Beamforming                               │
└──────────────────────────────────────────────┘
                    │
                    ▼
              UE (Devices)
```

### 13.2 5G Features for Edge Computing

1. **Network Slicing**
   - Dedicated virtual networks per use case


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
