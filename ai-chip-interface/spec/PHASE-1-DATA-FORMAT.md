# WIA-AI-011 Phase 1: Data Format Standardization

**Version:** 1.0
**Status:** Draft
**Last Updated:** 2025-01-15

---

## Overview

Phase 1 establishes foundational data format standards for AI chip interfaces, ensuring tensor data can be seamlessly shared across NPU, TPU, and GPU accelerators without conversion overhead or numerical degradation.

## Goals

1. Define unified tensor descriptor format
2. Standardize memory layouts (NCHW, NHWC, etc.)
3. Specify data type system (FP32, FP16, INT8, etc.)
4. Enable zero-copy data sharing
5. Support quantization metadata

## Tensor Descriptor Specification

### Core Structure

```json
{
  "version": "WIA-AI-011-v1.0",
  "tensor_id": "UUID",
  "name": "string",
  "shape": {
    "dimensions": [int, ...],
    "layout": "NCHW | NHWC | CHWN",
    "symbolic": boolean
  },
  "dtype": {
    "base_type": "FLOAT32 | FLOAT16 | BFLOAT16 | INT8 | INT32 | ...",
    "quantization": null | QuantizationParams,
    "byte_order": "little_endian | big_endian"
  },
  "memory": {
    "strides": [int, ...],
    "offset_bytes": int,
    "total_bytes": int,
    "alignment": int,
    "device": DeviceDescriptor
  },
  "properties": {
    "requires_grad": boolean,
    "is_pinned": boolean,
    "is_contiguous": boolean,
    "zero_copy_compatible": boolean
  },
  "metadata": {}
}
```

### Data Types

#### Floating Point
- **FLOAT64**: IEEE 754 double precision (64-bit)
- **FLOAT32**: IEEE 754 single precision (32-bit) - default for training
- **FLOAT16**: IEEE 754 half precision (16-bit)
- **BFLOAT16**: Brain floating point (16-bit, 8-bit exponent)
- **TF32**: TensorFloat-32 (19-bit, NVIDIA)
- **FP8_E4M3**: 8-bit float (4-bit exp, 3-bit mantissa)
- **FP8_E5M2**: 8-bit float (5-bit exp, 2-bit mantissa)

#### Integer
- **INT64**, **INT32**, **INT16**, **INT8**: Signed integers
- **UINT64**, **UINT32**, **UINT16**, **UINT8**: Unsigned integers
- **INT4**, **UINT4**: 4-bit integers (sub-byte)

### Memory Layouts

#### NCHW (Batch, Channel, Height, Width)
- Preferred by: NVIDIA cuDNN, PyTorch (GPU)
- Best for: Convolution operations
- Memory order: N → C → H → W

#### NHWC (Batch, Height, Width, Channel)
- Preferred by: TensorFlow, ARM, Mobile NPUs
- Best for: Cache locality in certain operations
- Memory order: N → H → W → C

#### Stride Representation
Strides define element spacing for each dimension:
```
element_offset = base_offset + Σ(index[i] * stride[i])
```

### Quantization

#### Symmetric Per-Tensor
```json
{
  "scheme": "per_tensor_symmetric",
  "scale": float,
  "zero_point": 0,
  "range": [int_min, int_max]
}
```

#### Asymmetric Per-Channel
```json
{
  "scheme": "per_channel_asymmetric",
  "scales": [float, ...],
  "zero_points": [int, ...],
  "channel_axis": int
}
```

## Memory Alignment

### Requirements
- **Minimum alignment**: 64 bytes
- **Recommended alignment**: 256 bytes
- **Device-specific**: Query via capability API

### Padding
Padding may be applied to dimensions for optimal performance:
```json
{
  "shape": [32, 11, 224, 224],
  "physical_shape": [32, 16, 224, 224],
  "padding": {
    "dims": [false, true, false, false],
    "values": [0, 5, 0, 0],
    "mode": "zero_fill"
  }
}
```

## Zero-Copy Protocol

### Shared Buffer Handle
```json
{
  "buffer_handle": {
    "type": "dma_buf_fd | cuda_ipc | metal_shared",
    "handle": int,
    "size_bytes": int,
    "permissions": "read_only | read_write"
  },
  "mapping": {
    "device_address": "0xaddress",
    "cpu_address": null | "0xaddress",
    "access_mode": "device_only | cpu_device"
  }
}
```

### Pinned Memory
- Page-locked memory for DMA transfers
- Stable physical addresses
- Flags: `WIA_MEM_PINNED`

## Sparse Tensor Support

### COO (Coordinate) Format
```json
{
  "format": "COO",
  "shape": [int, int],
  "nnz": int,
  "indices": {
    "dtype": "INT32 | INT64",
    "data": [[row, col], ...]
  },
  "values": {
    "dtype": "FLOAT32 | ...",
    "data": [float, ...]
  }
}
```

### CSR (Compressed Sparse Row)
```json
{
  "format": "CSR",
  "shape": [int, int],
  "row_pointers": [int, ...],
  "column_indices": [int, ...],
  "values": [float, ...]
}
```

## Dynamic Shapes

```json
{
  "shape": {
    "dimensions": ["batch", "sequence_length", 768],
    "symbolic": true,
    "constraints": {
      "batch": {"min": 1, "max": 256},
      "sequence_length": {"min": 1, "max": 2048, "multiple_of": 8}
    }
  }
}
```

## Compliance Requirements

### Level 1: Basic
- Support FLOAT32, FLOAT16
- Support NCHW or NHWC (at least one)
- Minimum 64-byte alignment

### Level 2: Standard
- Support FLOAT32, FLOAT16, BFLOAT16, INT8
- Support both NCHW and NHWC
- Stride-based layouts
- Quantization metadata

### Level 3: Advanced
- All data types
- All layouts
- Sparse tensors
- Zero-copy sharing
- Dynamic shapes

---

## Reference Implementation

See `api/typescript/` for reference implementation.

## Testing

Compliance tests available at: [WIA-AI-011-tests](https://github.com/WIA-Official/wia-ai-011-tests)

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


## Annex G — Document Index

This PHASE document is one of four PHASE artifacts that together describe this WIA standard. The full set of related artifacts is:

- `spec/PHASE-1-DATA-FORMAT.md` — normative data formats and schema definitions.
- `spec/PHASE-2-API.md` — normative SDK and Client API contract that consumes the PHASE 1 data formats.
- `spec/PHASE-3-PROTOCOL.md` — normative real-time communication protocol bindings (where applicable).
- `spec/PHASE-4-INTEGRATION.md` — normative ecosystem integration patterns (LIMS, paging, federation, reporting).

Readers SHOULD consult all four PHASE documents in sequence when planning a deployment. The OpenAPI document published alongside this standard reflects the §4 endpoints in machine-readable form and is updated synchronously with this PHASE.

This index is informative; the normative content is in the body of each PHASE document.


Additional implementer note: when this PHASE references an external interface profile, implementers SHOULD pin the profile version in their OpenAPI `info.x-wia-interface-profile` extension so that downstream auditors can trace the exact dependency graph at evaluation time.
