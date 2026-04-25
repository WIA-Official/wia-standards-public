# WIA-AI-011 Phase 2: API Abstraction Layer

**Version:** 1.0
**Status:** Draft
**Last Updated:** 2025-01-15

---

## Overview

Phase 2 defines hardware-agnostic APIs for AI accelerator operations, enabling portable code while preserving access to hardware-specific optimizations.

## Core APIs

### Device Management

```c
// Device enumeration
wia_status_t wia_enumerate_devices(wia_device_t** devices, int* count);
wia_status_t wia_get_device_info(wia_device_t device, wia_device_info_t* info);
wia_status_t wia_query_capabilities(wia_device_t device, wia_capability_t* caps);

// Device selection
wia_status_t wia_set_current_device(wia_device_t device);
wia_status_t wia_get_current_device(wia_device_t* device);
```

### Context Management

```c
// Context creation/destruction
typedef struct {
    wia_device_t device;
    size_t memory_pool_size;
    int max_streams;
    bool profiling_enabled;
} wia_context_config_t;

wia_status_t wia_create_context(wia_context_config_t* config, wia_context_t* ctx);
wia_status_t wia_destroy_context(wia_context_t ctx);
wia_status_t wia_context_synchronize(wia_context_t ctx);
```

### Stream Operations

```c
// Stream management
wia_status_t wia_create_stream(wia_context_t ctx, wia_stream_type_t type, wia_stream_t* stream);
wia_status_t wia_destroy_stream(wia_stream_t stream);
wia_status_t wia_stream_synchronize(wia_stream_t stream);
wia_status_t wia_stream_query(wia_stream_t stream, bool* is_idle);

// Event synchronization
wia_status_t wia_create_event(wia_event_t* event);
wia_status_t wia_record_event(wia_event_t event, wia_stream_t stream);
wia_status_t wia_stream_wait_event(wia_stream_t stream, wia_event_t event);
```

### Memory Operations

```c
// Memory allocation
typedef struct {
    size_t size;
    wia_memory_type_t type; // DEVICE, PINNED, UNIFIED, MANAGED
    size_t alignment;
    wia_memory_flags_t flags;
    wia_device_t device_affinity;
} wia_memory_desc_t;

wia_status_t wia_allocate_memory(wia_context_t ctx, wia_memory_desc_t* desc, wia_buffer_t* buffer);
wia_status_t wia_free_memory(wia_buffer_t buffer);

// Data transfer
wia_status_t wia_memcpy(void* dst, const void* src, size_t size);
wia_status_t wia_memcpy_async(void* dst, const void* src, size_t size, wia_stream_t stream);
wia_status_t wia_memset(void* ptr, int value, size_t size);
```

### Tensor Operations

```c
// Tensor allocation
wia_status_t wia_allocate_tensor(wia_context_t ctx, wia_tensor_desc_t* desc, wia_tensor_t* tensor);

// Matrix operations
wia_status_t wia_matmul(wia_context_t ctx, wia_tensor_t A, wia_tensor_t B, wia_tensor_t C,
                        wia_matmul_config_t* config, wia_stream_t stream);

// Convolution
wia_status_t wia_conv2d(wia_context_t ctx, wia_tensor_t input, wia_tensor_t weight,
                        wia_tensor_t bias, wia_tensor_t output,
                        wia_conv2d_config_t* config, wia_stream_t stream);

// Activation functions
wia_status_t wia_relu(wia_context_t ctx, wia_tensor_t input, wia_tensor_t output, wia_stream_t stream);
wia_status_t wia_gelu(wia_context_t ctx, wia_tensor_t input, wia_tensor_t output, wia_stream_t stream);
wia_status_t wia_softmax(wia_context_t ctx, wia_tensor_t input, wia_tensor_t output,
                         wia_softmax_config_t* config, wia_stream_t stream);
```

### Error Handling

```c
typedef enum {
    WIA_SUCCESS = 0,
    WIA_ERROR_INVALID_ARGUMENT,
    WIA_ERROR_OUT_OF_MEMORY,
    WIA_ERROR_DEVICE_NOT_FOUND,
    WIA_ERROR_NOT_SUPPORTED,
    WIA_ERROR_RUNTIME_ERROR
} wia_status_t;

const char* wia_get_error_string(wia_status_t status);
wia_status_t wia_get_last_error(wia_context_t ctx, wia_error_info_t* info);
```

## Capability System

```c
typedef struct {
    // Compute capabilities
    float peak_tflops_fp32;
    float peak_tflops_fp16;
    float peak_tflops_int8;
    
    // Memory capabilities
    size_t total_memory;
    size_t memory_bandwidth_gbps;
    bool supports_unified_memory;
    bool supports_pinned_memory;
    
    // Feature support
    bool supports_fp16;
    bool supports_bfloat16;
    bool supports_int8;
    bool supports_sparse_ops;
    bool supports_dynamic_shapes;
    
    // Limits
    int max_tensor_dims;
    size_t max_tensor_size;
    int max_batch_size;
} wia_capability_t;
```

## Profiling

```c
typedef struct {
    bool profiling_enabled;
    wia_profile_mode_t mode; // SAMPLING, INSTRUMENTATION, HARDWARE_COUNTERS
} wia_profiling_config_t;

wia_status_t wia_start_profiling(wia_context_t ctx, wia_profiling_config_t* config, const char* output_file);
wia_status_t wia_stop_profiling(wia_context_t ctx);
wia_status_t wia_get_profile_results(wia_context_t ctx, wia_profile_results_t* results);
```

## Compliance Levels

### Level 1: Core
- Device enumeration
- Basic context management
- Memory allocation (device memory)
- MatMul, Conv2D
- FLOAT32 support

### Level 2: Extended
- Stream management
- Event synchronization
- Multiple memory types
- Full tensor operation library
- FP16, INT8 support

### Level 3: Advanced
- Multi-device contexts
- Profiling APIs
- Dynamic shapes
- All operations and data types

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


## Annex E — Implementation Notes for PHASE-2-API

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-2-API.

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
