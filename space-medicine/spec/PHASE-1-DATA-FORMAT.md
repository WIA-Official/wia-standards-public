# PHASE 1 — DATA FORMAT · WIA-SPACE-023: Space Medicine Standard Specification v1.0
**Version:** 1.0
**Status:** Active
**Philosophy:** 弘益人間 · Benefit All Humanity

## 1. Scope and Purpose

This Phase defines the DATA FORMAT layer of the *WIA-SPACE-023: Space Medicine Standard Specification v1.0* WIA standard.
It establishes the normative requirements that every conforming implementation
MUST satisfy at this layer, and the supplementary guidance (SHOULD / MAY)
that drives interoperability, security, and operational predictability
across vendors and deployments.

## 2. Normative Conventions

The key words **MUST**, **MUST NOT**, **REQUIRED**, **SHALL**, **SHALL NOT**,
**SHOULD**, **SHOULD NOT**, **RECOMMENDED**, **MAY**, and **OPTIONAL** in
this document are to be interpreted as described in BCP 14 (RFC 2119 and
RFC 8174) when, and only when, they appear in all capitals, as shown here.

## 3. Roles and Responsibilities

| Role | Definition | Responsibilities at this Phase |
|------|------------|--------------------------------|
| Producer | Entity that emits payloads or operations | Emit only well-formed, signed, and authorised messages |
| Consumer | Entity that receives or persists payloads | Validate, authorise, and audit every received item |
| Authority | Entity that issues credentials or attestations | Maintain revocation lists and attestation logs |
| Operator | Entity that runs the deployment | Monitor health, capacity, and compliance posture |

## 4. Phase-Specific Requirements

### 4.1 Functional Requirements

1. The implementation MUST expose the canonical surface defined in §5.
2. The implementation MUST reject malformed inputs with a deterministic
   error code and SHALL NOT silently coerce them into accepted forms.
3. The implementation MUST emit audit records for every state-changing
   operation conducted at this Phase.
4. The implementation MUST honour cancellation requests within the
   timeout window declared in §6.
5. The implementation SHOULD support graceful degradation when an
   upstream dependency is unavailable.

### 4.2 Non-Functional Requirements

| Quality | Target | Verification |
|---------|--------|--------------|
| Availability | ≥ 99.9 % monthly | Synthetic probes from three regions |
| Throughput | ≥ baseline declared in conformance report | Load test in F.7 of the Annex |
| Latency p99 | ≤ ceiling declared in conformance report | Continuous percentile tracking |
| Mean time to recovery | ≤ 15 minutes | Quarterly chaos drill |
| Audit completeness | 100 % of state-changing ops | Daily integrity sweep |

### 4.3 Failure Modes

The following failure modes are reserved by this Phase. Implementations
MUST distinguish them with the codes shown so that operators can write
common runbooks across vendors.

| Code | Symptom | Required Action |
|------|---------|-----------------|
| E_INPUT_MALFORMED | Inputs do not parse against the canonical schema | Reject, do not retry |
| E_AUTH_DENIED | Caller lacks authorisation | Reject, log security event |
| E_DEPENDENCY_DOWN | Upstream resource unavailable | Retry with back-off, surface degraded mode |
| E_CAPACITY | Resource is saturated | Reject with retry-after, page on-call if persistent |
| E_INTEGRITY | Persisted state diverges from expected hash | Quarantine, escalate to incident response |

## 5. Canonical Surface

The canonical surface for this Phase is documented as a machine-readable
artefact alongside the human-readable text:

- OpenAPI 3.1 description for HTTP-based interfaces
- Protocol Buffers IDL for binary interfaces
- AsyncAPI 2.6 description for event-driven interfaces

Implementations MUST publish their realisation of the canonical surface
under `/api/openapi.yaml`, `/api/protocol.proto`, or `/api/asyncapi.yaml`
respectively. Any deviation MUST be declared as a numbered exception in
the conformance report.

## 6. Timeouts and Limits

| Operation Class | Default Timeout | Maximum Timeout |
|-----------------|----------------:|----------------:|
| Synchronous read | 5 s | 30 s |
| Synchronous write | 10 s | 60 s |
| Long-running task | 5 min | 30 min |
| Streaming session | 30 min | 8 h |
| Batch import | 60 min | 24 h |

Operators MAY tighten these defaults for their deployment but MUST NOT
relax them beyond the stated maximums without an explicit waiver from
the WIA Working Group.

## 7. Observability

Implementations MUST emit, at minimum, the following telemetry signals:

- **Metrics** in Prometheus exposition format or OTLP, including counters,
  histograms, and gauges for every operation class in §6.
- **Logs** in structured JSON with correlation identifiers conforming to
  the W3C Trace Context.
- **Traces** with spans named after the operation class and tagged with
  the canonical resource identifier from §5.
- **Events** for state transitions of long-running tasks, published to a
  durable channel so that downstream consumers can rebuild state.

## 8. Security Posture

Refer to Annex B of this document for the cryptographic suite. At this
Phase specifically, the following posture is normative:

1. All inter-service traffic uses mutual TLS with certificates rotated
   at most every 90 days.
2. Tokens carrying caller identity are short-lived (≤ 15 minutes) and
   bound to the caller's network identity via a sender-constrained
   mechanism (DPoP or mTLS-bound).
3. Sensitive payload fields are encrypted at the application layer using
   per-tenant keys held in an HSM (FIPS 140-3 Level 3 or higher).
4. Audit logs are append-only, signed in batches, and replicated to a
   write-once-read-many store within 60 seconds of generation.

## 9. Conformance Reporting

Conformance to this Phase is reported under the schema in Annex F.
A claim of L2 conformance or higher MUST include the result of running
the conformance test suite published with this standard.


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
in lockstep across Phases 1–4 of WIA-SPACE-023: Space Medicine Standard Specification v1.0 so that conformance claims at any
Phase remain unambiguous.*

