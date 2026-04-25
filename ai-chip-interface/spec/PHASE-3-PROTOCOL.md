# WIA-AI-011 Phase 3: Communication Protocols

**Version:** 1.0
**Status:** Draft
**Last Updated:** 2025-01-15

---

## Overview

Phase 3 establishes protocols for multi-device coordination, inter-chip communication, and distributed execution.

## Topology Discovery

```c
typedef struct {
    int num_devices;
    float bandwidth_gbps[MAX_DEVICES][MAX_DEVICES];
    float latency_us[MAX_DEVICES][MAX_DEVICES];
    wia_interconnect_type_t interconnect[MAX_DEVICES][MAX_DEVICES];
} wia_topology_t;

wia_status_t wia_discover_topology(wia_topology_t* topology);
wia_status_t wia_can_access_peer(wia_device_t dev0, wia_device_t dev1, bool* can_access);
```

## Peer-to-Peer Communication

```c
// Enable P2P access
wia_status_t wia_enable_peer_access(wia_device_t src, wia_device_t dst);
wia_status_t wia_disable_peer_access(wia_device_t src, wia_device_t dst);

// P2P transfers
wia_status_t wia_memcpy_peer(wia_buffer_t dst, wia_buffer_t src, size_t size, wia_stream_t stream);
```

## Collective Operations

```c
// Communicator
typedef struct {
    wia_device_t* devices;
    int num_devices;
    int rank;
} wia_communicator_t;

wia_status_t wia_create_communicator(wia_device_t* devices, int count, wia_communicator_t* comm);

// All-Reduce
typedef struct {
    wia_reduce_op_t op; // SUM, PROD, MIN, MAX, AVG
    wia_datatype_t datatype;
    wia_allreduce_algorithm_t algorithm; // RING, TREE, RABENSEIFNER
} wia_collective_config_t;

wia_status_t wia_all_reduce(wia_communicator_t comm, wia_buffer_t send_buf,
                            wia_buffer_t recv_buf, size_t count,
                            wia_collective_config_t* config, wia_stream_t stream);

// Broadcast
wia_status_t wia_broadcast(wia_communicator_t comm, wia_buffer_t buffer,
                           size_t count, int root, wia_datatype_t dtype, wia_stream_t stream);

// All-Gather
wia_status_t wia_all_gather(wia_communicator_t comm, wia_buffer_t send_buf,
                            wia_buffer_t recv_buf, size_t count,
                            wia_datatype_t dtype, wia_stream_t stream);

// Reduce-Scatter
wia_status_t wia_reduce_scatter(wia_communicator_t comm, wia_buffer_t send_buf,
                                wia_buffer_t recv_buf, size_t count,
                                wia_reduce_op_t op, wia_datatype_t dtype, wia_stream_t stream);
```

## Compression

```c
typedef struct {
    wia_compression_method_t method; // TOPK, THRESHOLD, QUANTIZE
    union {
        struct { float k_ratio; } topk;
        struct { float threshold; } threshold;
        struct { int bits; } quantize;
    } parameters;
    bool error_feedback;
} wia_compression_config_t;

wia_status_t wia_all_reduce_compressed(wia_communicator_t comm, wia_buffer_t send,
                                       wia_buffer_t recv, size_t count,
                                       wia_compression_config_t* compression,
                                       wia_stream_t stream);
```

## RDMA Support

```c
typedef struct {
    int num_nodes;
    int devices_per_node;
    wia_network_backend_t backend; // RDMA, TCP, CUSTOM
    const char* rdma_device;
} wia_distributed_config_t;

wia_status_t wia_create_distributed_communicator(wia_distributed_config_t* config,
                                                 wia_communicator_t* comm);
```

## Synchronization

```c
// Barrier across all devices in communicator
wia_status_t wia_barrier(wia_communicator_t comm, wia_stream_t stream);

// Wait for all operations on all devices
wia_status_t wia_communicator_synchronize(wia_communicator_t comm);
```

---

**弘益人間 (홍익인간) · Benefit All Humanity**

© 2025 WIA (World Certification Industry Association)

---

## Annex A — Conformance Tier Matrix

WIA conformance for ai-chip-interface is evaluated across three tiers:

| Tier | Scope | Mandatory artifacts | Audit cadence |
|------|-------|--------------------|----------------|
| Tier 1 — Self-declared | Internal use, pilot deployments | OpenAPI 3.0 contract, JSON Schema validation report, security threat model | Annual self-review |
| Tier 2 — Third-party assessed | External partners, B2B integrations | Tier 1 artifacts + signed third-party assessor report against this PHASE | Every 24 months |
| Tier 3 — Accredited | Public-facing or regulated deployments | Tier 2 artifacts + WIA accreditation, ISO/IEC 17065:2012 conformity assessment, evidence retention ≥ 7 years | Every 12 months |

Implementations MUST disclose their conformance tier in the OpenAPI `info.x-wia-tier` extension and on any public certification page. Tier downgrade events MUST be reported to the WIA registry within 30 days.

---

## Annex B — Cross-Walk to International Standards

The PHASE specification reuses or normatively references published standards alongside this document. Implementers SHOULD review the relevant standards alongside this PHASE document; where a conflict exists, the more specific WIA requirement governs unless explicitly superseded by a binding national regulation.

- ISO/IEC 17065:2012 — Conformity assessment — Requirements for bodies certifying products, processes and services
- ISO/IEC 27001:2022 — Information security management systems
- IETF RFC 9457 — Problem Details for HTTP APIs
- IETF RFC 7519 — JSON Web Token (JWT)
- IETF RFC 6749 — The OAuth 2.0 Authorization Framework
- W3C PROV-DM — provenance data model

This cross-walk is informative only. WIA does not republish the referenced documents; readers MUST obtain authoritative copies from the issuing body. Cross-walk entries are reviewed at every minor version of this PHASE.

---

## Annex C — Reference Implementations and Test Vectors

### C.1 Reference Implementations

WIA does not require implementers to use a particular library, but maintains pointers to the canonical reference implementation directories under the WIA-Official GitHub organization:

- `wia-standards/standards/ai-chip-interface/api/` — TypeScript SDK skeleton
- `wia-standards/standards/ai-chip-interface/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/ai-chip-interface/simulator/` — interactive browser-based simulator for the PHASE protocol

Each reference artifact ships with an MIT license and is intended as a starting point, not as a production-grade implementation.

### C.2 Test Vectors

A normative set of request/response pairs covering the schemas defined in this PHASE is published alongside this document. Implementations claiming Tier 2 or Tier 3 conformance MUST pass every published test vector in both serialization (request) and deserialization (response) directions, and MUST publish a signed report identifying which vectors were exercised.

Test vectors are versioned independently of the PHASE document; refer to the `test-vectors/` directory for the active version.

---

## Annex D — Open Questions and Future Work

This PHASE document captures the consensus position at v1.0. The following items are tracked for future minor or major revisions:

1. **Schema versioning policy** — formal MUST/SHOULD rules for backward-compatible vs. breaking changes between minor releases.
2. **Privacy-preserving aggregation** — guidance on differential privacy and secure aggregation patterns for telemetry channels, without prescribing a specific algorithm.
3. **Multilingual error catalogs** — localization strategy for the standard error codes defined in this PHASE.
4. **Long-term retention** — alignment with sectoral retention regulations across jurisdictions.
5. **Sustainability disclosure** — optional fields for energy and emissions reporting tied to operations covered by this PHASE.

Items in this annex are non-normative. Comments and proposals are accepted via the GitHub issues tracker on the WIA-Official organization.

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


## Annex G — Document Index

This PHASE document is one of four PHASE artifacts that together describe this WIA standard. The full set of related artifacts is:

- `spec/PHASE-1-DATA-FORMAT.md` — normative data formats and schema definitions.
- `spec/PHASE-2-API.md` — normative SDK and Client API contract that consumes the PHASE 1 data formats.
- `spec/PHASE-3-PROTOCOL.md` — normative real-time communication protocol bindings (where applicable).
- `spec/PHASE-4-INTEGRATION.md` — normative ecosystem integration patterns (LIMS, paging, federation, reporting).

Readers SHOULD consult all four PHASE documents in sequence when planning a deployment. The OpenAPI document published alongside this standard reflects the §4 endpoints in machine-readable form and is updated synchronously with this PHASE.

This index is informative; the normative content is in the body of each PHASE document.


Additional implementer note: when this PHASE references an external interface profile, implementers SHOULD pin the profile version in their OpenAPI `info.x-wia-interface-profile` extension so that downstream auditors can trace the exact dependency graph at evaluation time.
