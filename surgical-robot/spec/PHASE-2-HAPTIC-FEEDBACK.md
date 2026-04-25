# WIA-MED-006 PHASE 2: Haptic Feedback Specification

## Overview
This specification defines haptic feedback requirements for surgical robotic systems.

## 1. Force Sensing

### 1.1 Sensor Requirements
- **Force Range**: 0-10N continuous, 0-20N peak
- **Resolution**: 0.1N
- **Accuracy**: ±0.1N or 2% of reading
- **Sampling Rate**: ≥1000 Hz
- **Sensor Types**: Strain gauge, piezoelectric, or 6-axis force/torque

### 1.2 Mounting Locations
- Tool shaft (bending/torsion)
- Tool jaw (grip force)
- Tool base (6-axis F/T)

## 2. Haptic Rendering

### 2.1 Processing Pipeline
1. **Sensor Data Acquisition** (1000 Hz)
2. **Signal Processing**: Noise filtering (Kalman), drift correction
3. **Force Mapping**: Scaling, saturation (max 5N), deadzone (<0.1N)
4. **Tissue Modeling**: K=1-50 N/mm, damping B=0.1-5 Ns/mm
5. **Actuator Drive**: Motor current commands

### 2.2 Performance
- **Update Rate**: 1000 Hz minimum (stability requirement)
- **Total Latency**: <1ms (transparency requirement)
- **Stability**: Guaranteed for passive environments

## 3. Haptic Actuators

### 3.1 Actuator Specifications
- **Force Range**: 0-5N continuous
- **Bandwidth**: DC to 100 Hz minimum
- **Resolution**: 0.05N
- **Backdrivability**: Low friction/inertia

### 3.2 Actuator Types (any compliant)
- DC motors with gearing
- Voice coil actuators
- Piezoelectric actuators
- Pneumatic actuators

## 4. Tissue Models

### 4.1 Standard Tissue Parameters
```
Fat:      K = 1-10 kPa,   High viscoelasticity
Muscle:   K = 10-100 kPa, Medium viscoelasticity
Liver:    K = 5-20 kPa,   High viscoelasticity
Kidney:   K = 10-30 kPa,  Medium viscoelasticity
Vessel:   K = 100-1000 kPa, Low viscoelasticity
Tumor:    K = 50-500 kPa, Low viscoelasticity
Bone:     K = 10-20 GPa,  Very low viscoelasticity
```

### 4.2 Modeling Approaches
- Linear elastic: F = K × x
- Viscoelastic (Kelvin-Voigt): F = K × x + B × v
- Nonlinear: F = K₁×x + K₂×x² + K₃×x³
- Hunt-Crossley (contact): F = K × x^n × (1 + λ × v)

## 5. Validation

### 5.1 Benchtop Testing
- Force accuracy verification
- Frequency response measurement
- Stability testing

### 5.2 Tissue Phantom Testing
- Silicone phantoms with calibrated stiffness
- Porcine tissue comparison
- User studies (N≥10 surgeons)

## 6. Optional Advanced Features

- Texture rendering (vibrotactile 10-300 Hz)
- Thermal feedback
- Multi-point tactile arrays
- Adaptive rendering based on user preference

---

**Status**: ✅ Complete  
**Version**: 1.0.0  
**Date**: 2025-01-01

© 2025 WIA · MIT License

## P.2 API Surface Cross-References

The API surface defined in this Phase consumes and emits the data formats from
Phase 1 and is transported by the protocol layer in Phase 3. Operators deploy
the surface using the integration patterns in Phase 4.

### P.2.1 Resource Naming

Resource paths follow REST conventions with snake_case segments. Identifier
segments use the canonical UUID encoding from Phase 1.

```
/v1/{collection}                        # collection
/v1/{collection}/{id}                    # member
/v1/{collection}/{id}/{sub_collection}   # nested collection
/v1/{collection}/{id}:{action}           # custom action (POST)
```

### P.2.2 Pagination

List endpoints support cursor-based pagination:

| Param | Default | Max | Description |
|-------|---------|-----|-------------|
| `page_size` | 50 | 500 | Items per page |
| `page_token` | empty | — | Opaque continuation token |

Servers MUST return `next_page_token` when the result set is truncated and an
empty string when the final page has been delivered.

### P.2.3 Idempotency

State-changing operations accept the `Idempotency-Key` header (RFC-style).
Servers MUST cache the response keyed by `(principal, key)` for at least 24 h
and replay the same response on retry.

### P.2.4 Field Masks

Partial-update operations use field masks (Google AIP-161 style) to avoid
clobbering unspecified fields. Masks are dot-paths into the canonical schema
with `*` wildcards.


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
in lockstep across Phases 1–4 of WIA-ROB-006: Surgical Robot Standard so that conformance claims at any
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
`64a8ed8a … 5b3f`. Implementations MUST emit the same digest when applied
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

The canonical example timestamp `2026-04-25T03:30:00Z` MUST render
identically in every locale tested by the conformance suite:

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

