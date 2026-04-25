# WIA Sign Language Recognition Standard
## Phase 1: Data Format Specification

**Version:** 1.0.0  
**Status:** Final  
**Last Updated:** 2025-12-25  
**Author:** WIA Technical Committee  

---

## Table of Contents

1. [Introduction](#introduction)
2. [Video Data Model](#video-data-model)
3. [Gesture Representation](#gesture-representation)
4. [Keypoint Schema](#keypoint-schema)
5. [JSON Schema Definitions](#json-schema-definitions)
6. [Sign Language Encoding](#sign-language-encoding)
7. [Validation Rules](#validation-rules)
8. [Example Payloads](#example-payloads)

---

## 1. Introduction

This specification defines standardized data formats for sign language video, keypoints, and recognition results.

### 1.1 Design Principles

- **Universal**: Support all 300+ sign languages
- **Efficient**: Optimize for real-time streaming
- **Extensible**: Allow future enhancements
- **Privacy**: No PII in standard format

### 1.2 Scope

- Video format specifications
- Hand/body keypoint representation
- Gesture classification format
- Multi-language sign encoding

---

## 2. Video Data Model

### 2.1 Video Requirements

```typescript
interface VideoInput {
  /** Format: MP4, WebM, raw frames */
  format: 'mp4' | 'webm' | 'raw';
  
  /** Codec: H.264, VP9, AV1 */
  codec: 'h264' | 'vp9' | 'av1';
  
  /** Resolution (minimum 720p) */
  width: number;
  height: number;
  
  /** Frame rate (minimum 30fps) */
  fps: number;
  
  /** Bitrate for streaming */
  bitrate?: number;
}
```

### 2.2 Frame Data

```typescript
interface VideoFrame {
  /** Frame timestamp in milliseconds */
  timestamp: number;
  
  /** Frame index */
  frameIndex: number;
  
  /** Base64 encoded image or video chunk */
  data: string;
  
  /** Frame metadata */
  metadata?: FrameMetadata;
}
```

---

## 3. Gesture Representation

### 3.1 Sign Data Structure

```typescript
interface SignLanguageGesture {
  /** Unique gesture ID */
  gestureId: string;
  
  /** Sign language (ISO 639-3 code) */
  language: 'ase' | 'bfi' | 'kvk' | 'jsl' | string;
  
  /** Confidence score (0-1) */
  confidence: number;
  
  /** Text translation */
  text: string;
  
  /** Keypoint data */
  keypoints: KeypointData;
  
  /** Bounding box */
  boundingBox?: BoundingBox;
}
```

---

## 4. Keypoint Schema

### 4.1 Hand Landmarks (21 points per hand)

```typescript
interface HandKeypoints {
  /** Left hand landmarks */
  left: Landmark[];
  
  /** Right hand landmarks */
  right: Landmark[];
}

interface Landmark {
  /** X coordinate (normalized 0-1) */
  x: number;
  
  /** Y coordinate (normalized 0-1) */
  y: number;
  
  /** Z coordinate (depth, normalized) */
  z: number;
  
  /** Visibility score (0-1) */
  visibility: number;
}
```

### 4.2 Pose Keypoints (33 body landmarks)

```typescript
interface PoseKeypoints {
  /** Body landmarks (MediaPipe format) */
  landmarks: Landmark[];
}
```

### 4.3 Face Landmarks (468 facial points)

```typescript
interface FaceKeypoints {
  /** Facial landmarks */
  landmarks: Landmark[];
}
```

---

## 5. JSON Schema Definitions

### 5.1 Recognition Request

```json
{
  "format": "WIA-SIGN-LANGUAGE-v1.0",
  "timestamp": "2025-12-25T10:30:00Z",
  "input": {
    "type": "video",
    "language": "ase",
    "video": {
      "format": "mp4",
      "width": 1280,
      "height": 720,
      "fps": 30,
      "data": "base64_encoded_video"
    }
  },
  "options": {
    "realtime": true,
    "confidenceThreshold": 0.85
  }
}
```

### 5.2 Recognition Response

```json
{
  "format": "WIA-SIGN-LANGUAGE-v1.0",
  "timestamp": "2025-12-25T10:30:01Z",
  "result": {
    "gestures": [{
      "gestureId": "hello",
      "language": "ase",
      "confidence": 0.985,
      "text": "Hello",
      "keypoints": {
        "leftHand": [],
        "rightHand": [],
        "pose": [],
        "face": []
      }
    }]
  }
}
```

---

## 6. Sign Language Encoding

### 6.1 ISO 639-3 Language Codes

- `ase`: American Sign Language (ASL)
- `bfi`: British Sign Language (BSL)
- `kvk`: Korean Sign Language (KSL)
- `jsl`: Japanese Sign Language
- `csl`: Chinese Sign Language

### 6.2 Custom Extensions

Support for regional dialects and variations.

---

## 7. Validation Rules

- Video must be at least 720p @ 30fps
- Confidence scores between 0.0 and 1.0
- All keypoints must have x, y, z, visibility
- Timestamps in ISO 8601 format

---

## 8. Example Payloads

See full examples in documentation.

---

**弘益人間 · Benefit All Humanity**

© 2025 WIA - World Certification Industry Association

## P.1 Data Format Cross-References

This Phase defines the canonical data types referenced by the API surface (Phase 2),
the wire protocol (Phase 3), and integration scenarios (Phase 4). Implementations
MUST round-trip every canonical type through serialization and deserialization
without loss of precision or semantics.

### P.1.1 Canonical Encoding Rules

1. UTF-8 is the required character encoding for textual fields.
2. Numeric fields use IEEE 754 binary64 unless explicitly marked as fixed-point.
3. Timestamps use RFC 3339 with timezone offset; durations use ISO 8601.
4. UUIDs follow RFC 4122 v4 unless deterministic IDs are required.
5. Binary payloads are encoded as Base64 (RFC 4648 §4) in JSON contexts and as
   raw octet strings in Protocol Buffers / CBOR contexts.

### P.1.2 Schema Evolution

Schema changes follow these compatibility classes:

| Class | Allowed Changes | Wire-Compat |
|-------|-----------------|-------------|
| Patch | Doc fixes, examples, validator tightening within existing range | Forward & backward |
| Minor | New optional fields, new enum values with default fallback        | Forward |
| Major | Field rename, type change, removal, semantics change              | None |

### P.1.3 Validation Order

Validators MUST apply checks in this order: (1) syntactic well-formedness,
(2) schema conformance, (3) cross-field invariants, (4) external referential
integrity, (5) policy / authorization. A failure short-circuits subsequent
checks; the response message identifies the first failing rule by ID.


---

## Appendix · Common WIA Standard Provisions

> The following provisions apply to every WIA standard and are kept in sync
> across the WIA-Standards corpus. Standard-specific deviations, where they
> exist, are listed in the standard's normative body.

### A. Conformance & Compliance

#### A.1 Conformance Levels

WIA-Standards defines four conformance levels:

| Level | Required | Description |
|-------|----------|-------------|
| L1 — Format | Phase 1 | Implementation produces and consumes the canonical data format losslessly |
| L2 — Interface | Phase 1 + 2 | Implementation exposes the API surface with required behaviour |
| L3 — Protocol | Phase 1 + 2 + 3 | Implementation interoperates over at least one normative transport binding |
| L4 — Integration | All Phases | Implementation passes the conformance test suite end-to-end in a production-shaped deployment |

Conformance claims MUST cite the level and the version of the standard
against which the claim is made (e.g. "L3 conformant against v1.0").

#### A.2 Compliance Verification

The conformance test suite is published alongside this standard at
`/cli/conformance/` and `/api/conformance/`. Implementations claiming
L2 or higher MUST publish their test report. Independent re-tests are
encouraged; the WIA Working Group accepts third-party verification reports
under the policy in §E.

### B. Security Considerations

#### B.1 Threat Model

Implementers SHOULD apply STRIDE analysis covering: spoofing of identity,
tampering with messages or stored state, repudiation of operations,
information disclosure, denial of service, and elevation of privilege.

| Threat | Default Control | Where Strengthened |
|--------|-----------------|--------------------|
| Spoofing | Mutual TLS or signed tokens | Phase 3 §P.3 |
| Tampering | TLS in transit, AEAD at rest | Phase 1 §P.1 |
| Repudiation | Append-only audit log with notarization | Phase 4 §P.4 |
| Disclosure | Field-level encryption for PII / secrets | Phase 1 §P.1 |
| DoS | Rate limit per principal & global circuit breaker | Phase 2 §P.2 |
| EoP | Least-privilege RBAC + scoped tokens | Phase 2 §P.2 |

#### B.2 Cryptographic Suites

Mandatory: TLS 1.3 with AEAD ciphers (AES-128-GCM, AES-256-GCM,
CHACHA20-POLY1305). Forbidden: TLS 1.0, TLS 1.1, RC4, MD5, SHA-1 for
signatures, RSA below 2048 bits, ECDSA on curves smaller than P-256.

Post-quantum migration: implementations SHOULD adopt hybrid key
exchange combining a classical primitive with ML-KEM (FIPS 203) once a
profile is published; signature migration to ML-DSA (FIPS 204) is
expected within the L4 conformance window of v2.0.

#### B.3 Audit Requirements

L3 and L4 implementations MUST log: (a) every authentication decision,
(b) every authorization decision, (c) every state-changing operation,
(d) every export of data outside its sovereignty boundary. Logs are
write-once for at least 1 year and 90 days indexed for incident search.

### C. Versioning & Lifecycle

Versions follow Semantic Versioning 2.0.0 (MAJOR.MINOR.PATCH).

| Phase | Duration | Conformance Status |
|-------|---------:|-------------------|
| Draft | until ratification | Non-binding |
| Active | indefinite | Binding for new deployments |
| Maintenance | 24 months from successor's Active date | Binding for existing deployments |
| Retired | indefinite | Non-binding; conformance claims rescinded |

Deprecation MUST be announced at least one minor version before a feature
is removed in a major version.

### D. Internationalization & Accessibility

Implementations SHOULD support locale negotiation via the
`Accept-Language` header (RFC 4647). Date, time, number, and currency
formatting follow CLDR. User-facing surfaces MUST satisfy WCAG 2.1 AA at
minimum and SHOULD progress towards WCAG 2.2 AA. Right-to-left scripts
(Arabic, Hebrew, Persian, Urdu) and East-Asian wide characters MUST be
laid out correctly without line-breaking heuristics that split graphemes.

### E. Governance & IP Policy

This standard is maintained by the WIA Working Group under the WIA
governance charter. Editorial changes are merged via pull request. Normative
changes require working-group consensus and a 30-day public review.
Contributions are accepted under the Apache License 2.0 with explicit
patent grant. Members participate under the WIA Patent Policy,
which requires royalty-free licensing of any essential claim necessary
to implement a normative requirement.

### F. Normative References

The following references are normative; implementations MUST satisfy
the cited clauses:

- ISO/IEC 27001:2022 — Information security management systems
- ISO/IEC 27017:2015 — Cloud-services security controls
- ISO/IEC 27701:2019 — Privacy information management
- ISO/IEC 19790:2012 — Security requirements for cryptographic modules
- ISO 8601-1:2019 — Date and time representation
- IETF RFC 8446 — TLS 1.3
- IETF RFC 7519 — JSON Web Token
- IETF RFC 6749 — OAuth 2.0
- IETF RFC 9110 — HTTP Semantics
- IETF RFC 9112 — HTTP/1.1 message syntax
- IETF RFC 9113 — HTTP/2
- IETF RFC 9114 — HTTP/3
- IETF RFC 9000 — QUIC transport
- IETF RFC 4122 — UUID URN namespace
- IETF RFC 3339 — Date and time on the Internet
- IETF RFC 6838 — Media-type specifications and registration
- W3C TraceContext — Distributed tracing context
- W3C WCAG 2.1 — Accessibility guidelines
- FIPS PUB 197 — AES
- FIPS PUB 180-4 — SHA-2 family
- FIPS PUB 203 — ML-KEM (post-quantum KEM)
- FIPS PUB 204 — ML-DSA (post-quantum signature)

### G. Glossary

| Term | Definition |
|------|------------|
| Conformance | The state of satisfying every normative requirement at a given level |
| Implementation | A software, hardware, or composite artefact that claims conformance |
| Principal | The authenticated entity bound to a security context |
| Subject | The resource or person to which an operation applies |
| Sovereignty Boundary | The legal / regulatory perimeter outside of which data export is restricted |

---

*This Appendix is authored by the WIA Standards Working Group and is kept
in lockstep across Phases 1–4 of sign-language so that conformance claims at any
Phase remain unambiguous.*


---

## Annex F · Conformance Test Vectors and Worked Examples

The following test vectors and end-to-end examples are normative for L2
and higher conformance. Implementations MUST reproduce every byte of every
output vector when given the corresponding input vector under the
deterministic inputs specified.

### F.1 Determinism Requirements

Implementations MUST be deterministic for the following operations:

| Operation | Determinism Source |
|-----------|--------------------|
| Canonical JSON serialization | RFC 8785 (JCS) |
| Hash and digest computation | FIPS 180-4 SHA-256 / SHA-512 |
| UUID v4 in test vectors | Seeded PRNG (xorshift64*, seed disclosed in vector) |
| Floating-point reduction | IEEE 754 round-to-nearest-even |
| Time-dependent fields in tests | Frozen clock value disclosed in vector |

### F.2 Round-Trip Vector — Canonical Object

Given the canonical input object as Phase 1 §2 schema, the canonical
serialization MUST match the byte sequence below exactly:

```text
{"id":"00000000-0000-4000-8000-000000000001","version":"1.0.0"}
```

The SHA-256 of this byte sequence (no trailing newline) is:
`a7d3490a62ae446074a226efc6857ba917fd40cec806497bd4ccf373664bf836`. Implementations MUST emit the same digest when applied
to the same input.

### F.3 Negative Vector — Schema Violation

Given an input that violates the schema in Phase 1 §3 (missing required
field `id`), the conforming validator MUST emit error code `E_SCHEMA_001`
with field-path pointer `$.id` and the human-readable message
"required property 'id' is missing".

### F.4 Boundary Vector — Maximum Sizes

| Field | Maximum | Behavior on Excess |
|-------|---------|--------------------|
| `string` (default) | 4 KiB | reject with `E_LIMIT_STRING` |
| `bytes` (inline) | 1 MiB | reject with `E_LIMIT_BYTES` |
| nested object depth | 32 | reject with `E_LIMIT_DEPTH` |
| array length (default) | 65,536 | reject with `E_LIMIT_ARRAY` |
| number magnitude | 2^53−1 | reject with `E_LIMIT_NUMBER` |

Implementations MAY raise these limits per deployment, provided the
operative limit is published in the conformance report.

### F.5 Locale & Time-zone Vector

The canonical example timestamp `2026-04-25T03:30:00Z` is rendered in a
locale-dependent manner. Implementations MUST delegate locale rendering
to a CLDR-compliant library; representative stylistic targets for popular
locales follow (the exact byte sequence depends on the CLDR profile in
use):

| Locale | Rendering |
|--------|-----------|
| `en-US` | `April 25, 2026, 3:30 AM UTC` |
| `ko-KR` | `2026년 4월 25일 오전 3시 30분 (협정 세계시)` |
| `ja-JP` | `2026年4月25日 3時30分 (協定世界時)` |
| `de-DE` | `25. April 2026, 03:30 Uhr UTC` |
| `ar-SA` | right-to-left layout; Hijri calendar tag MAY be appended |

### F.6 Replay Protection Vector

Given a request signed with a nonce that has been seen within the last
300 seconds, the receiver MUST respond with status `409 Conflict` and
error body `{"error":"E_REPLAY","retry_after_seconds":<n>}` where `<n>`
is the number of seconds until the nonce window slides past the offending
nonce.

### F.7 Conformance Report Schema

Every conformance claim MUST be accompanied by a machine-readable report
matching the schema below:

```yaml
report:
  standard: "<standard-id>"
  version: "<MAJOR.MINOR.PATCH>"
  level: "L1|L2|L3|L4"
  implementation:
    name: "<vendor>/<product>"
    build: "<git sha or build id>"
    deployment_topology: "<topology from Phase 4 §P.4.1>"
  results:
    pass: <integer>
    fail: <integer>
    skip: <integer>
  signed_by: "<DN of signing authority>"
  signed_at: "<RFC 3339 timestamp>"
  signature: "<base64 detached signature>"
```

### F.8 Audit Trail Sample

A conforming implementation produces audit records of the following
shape for every state-changing operation:

```json
{
  "audit_id": "01HFXX0000000000000000000",
  "timestamp": "2026-04-25T03:30:00.123Z",
  "principal": "service-account/api-gateway",
  "action": "resource.update",
  "subject": "/v1/things/<id>",
  "request_digest": "sha256:9b1a…",
  "response_status": 200,
  "trace_id": "0af7651916cd43dd8448eb211c80319c"
}
```

Audit records are append-only and signed in batches of at most 1000
records per chain link using the cryptographic suite from Annex B.

