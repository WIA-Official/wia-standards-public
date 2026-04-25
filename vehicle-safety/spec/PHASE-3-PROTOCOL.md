# WIA-AUTO-022 PHASE 3: Protocol

> **Version:** 1.0.0  
> **Status:** Active  
> **Last Updated:** 2025-12-27  
> **Focus:** ISO 26262, ASIL classification, testing protocols

---

## Overview

Phase 3 implements safety protocols and compliance frameworks governing system behavior. This phase integrates ISO 26262 functional safety, crash testing methodologies (FMVSS, Euro NCAP), and real-time communication protocols.

**弘益人間 (Benefit All Humanity)** - Rigorous protocols ensure safety systems perform reliably, protecting lives in critical moments.

---

## ISO 26262 Integration

### ASIL Classification

**Automotive Safety Integrity Levels:**

| ASIL | Severity | Exposure | Controllability | Example Systems |
|------|----------|----------|-----------------|-----------------|
| QM | No injury | - | - | Windshield wipers |
| ASIL-A | Light injuries | Low | Usually | Rear lights |
| ASIL-B | Moderate injuries | Medium | Normally | ESC |
| ASIL-C | Severe injuries | High | Difficult | AEB |
| ASIL-D | Life-threatening | Very high | Unlikely | Airbag control |

**Classification Formula:**
```
ASIL = f(Severity, Exposure, Controllability)

Severity (S):
  S0: No injuries
  S1: Light injuries
  S2: Severe injuries
  S3: Life-threatening/fatal

Exposure (E):
  E0: Incredible (< 0.001%)
  E1: Very low (< 0.1%)
  E2: Low (< 1%)
  E3: Medium (< 10%)
  E4: High (≥ 10%)

Controllability (C):
  C0: Controllable in general
  C1: Simply controllable
  C2: Normally controllable
  C3: Difficult to control
```

### Safety Requirements for ASIL-D (Airbag Systems)

1. **Dual Microcontroller Architecture**: Independent processors with cross-checking
2. **Watchdog Timers**: Independent clock sources, < 100ms timeout
3. **ECC RAM**: Error correction for all safety-critical data
4. **Diagnostic Coverage**: > 99% fault detection capability
5. **Single Point Fault Metric**: > 99%
6. **Latent Fault Metric**: > 90%

---

## FMVSS Compliance Testing

### FMVSS 208 - Occupant Protection

**Test Requirements:**
- 35 mph frontal rigid barrier crash (belted occupants)
- 25 mph frontal rigid barrier crash (unbelted occupants)
- Hybrid III 50th percentile male and 5th percentile female dummies

**Injury Criteria Thresholds:**
- HIC-15 < 700
- Chest acceleration (3ms) < 60g
- Femur load < 10 kN (each leg)

### FMVSS 214 - Side Impact Protection

**Test Requirements:**
- 33.5 mph moving deformable barrier (MDB)
- Side impact dummies (SID-IIs or WorldSID)

**Injury Criteria:**
- Thoracic Trauma Index (TTI) < 85
- Pelvic force < 6.0 kN
- Abdominal force < 2.5 kN

### FMVSS 216a - Roof Crush Resistance

**Test Requirements:**
- Quasi-static roof strength test
- Strength-to-Weight Ratio (SWR) ≥ 3.0
- Both driver and passenger sides tested

---

## Euro NCAP Protocol

### Rating Methodology (2025)

**Overall Rating Calculation:**
- Adult Occupant: 40% weight (≥ 80% for 5 stars)
- Child Occupant: 20% weight (≥ 80% for 5 stars)
- Vulnerable Road Users: 20% weight (≥ 70% for 5 stars)
- Safety Assist: 20% weight (≥ 70% for 5 stars)

### Test Scenarios

**Frontal Impact:**
- 40% offset deformable barrier at 40 mph
- Full-width rigid barrier at 31 mph

**Side Impact:**
- Moving deformable barrier at 50 km/h
- Pole impact at 32 km/h

**Pedestrian/Cyclist:**
- Pedestrian AEB tests at 20-60 km/h
- Cyclist AEB tests at crossing scenarios

**Active Safety:**
- AEB car-to-car tests
- Lane support systems evaluation
- Speed assistance systems assessment

---

## Real-Time Communication Protocols

### V2V Safety Messages

**Basic Safety Message (BSM):**
- Frequency: 10 Hz
- Latency: < 100ms
- Content: Position, velocity, heading, acceleration
- Range: 300 meters minimum

**Emergency Electronic Brake Light (EEBL):**
- Trigger: Hard braking event (> 0.4g deceleration)
- Priority: High
- Propagation: Multi-hop (up to 3 hops)

### Security

- Message authentication using ECDSA signatures
- Certificate management (short-lived pseudonymous certificates)
- Privacy protection (certificate rotation every 5 minutes)

---

## Cybersecurity Standards (ISO/SAE 21434)

### Threat Analysis

**Common Attack Vectors:**
1. Remote exploitation via connected services
2. Physical access through OBD-II port
3. Supply chain compromise
4. Wireless key fob relay attacks

### Security Controls

- Network segmentation (safety-critical on isolated CAN)
- Intrusion detection systems
- Secure boot (verified firmware signatures)
- Secure OTA updates (signed and encrypted)

---

## Testing Workflows

### Crash Test Execution

1. Test Setup (instrumentation, dummy positioning)
2. Pre-test Validation (system checks, calibration)
3. Test Execution (high-speed cameras, data acquisition)
4. Post-test Analysis (data extraction, injury calculation)
5. Automated Upload (results to certification portal)
6. Compliance Verification (automated checks against thresholds)
7. Certificate Generation (W3C Verifiable Credential)

### Automated Compliance Checking

```yaml
compliance_rules:
  - rule_id: NCAP-AO-001
    description: "Adult occupant frontal HIC-15 must be < 700"
    test: frontal_offset
    metric: driver.hic15
    threshold: 700
    operator: less_than
    
  - rule_id: FMVSS-208-001
    description: "Chest acceleration 3ms < 60g"
    test: frontal_rigid
    metric: driver.chest_acceleration_3ms
    threshold: 60
    operator: less_than
```

---

**弘益人間 (홍익인간) · Benefit All Humanity**

© 2025 SmileStory Inc. / WIA | MIT License

## P.3 Protocol Cross-References

The protocol defined here carries the data formats from Phase 1 and the API
operations from Phase 2 across trust boundaries. Phase 4 describes how the
protocol composes with adjacent infrastructure.

### P.3.1 Transport Bindings

| Binding | Default Port | Use |
|---------|-------------:|-----|
| HTTP/2 + TLS 1.3 | 443 | Public, request-response |
| HTTP/3 (QUIC) | 443 | Mobile, lossy networks |
| gRPC | 443 | Service-to-service |
| MQTT 5.0 | 8883 | Constrained / IoT devices |
| AMQP 0-9-1 | 5671 | Backplane / event streams |

### P.3.2 Message Envelope

Every protocol message carries a small envelope independent of payload:

```
+----------------+------------------+--------------------+
| message_id     | UUIDv4           | RFC 4122           |
| trace_id       | 16-byte hex      | W3C Trace Context  |
| span_id        | 8-byte hex       | W3C Trace Context  |
| origin_node    | DNS name or NIN  | RFC 1035           |
| issued_at      | RFC 3339         | UTC required       |
| ttl_seconds    | uint32           | 0 = no expiry      |
| content_type   | media type       | RFC 6838           |
| body           | opaque bytes     | per content_type   |
+----------------+------------------+--------------------+
```

### P.3.3 Reliability Model

The protocol provides at-least-once delivery by default. Receivers
deduplicate by `message_id`. Exactly-once semantics are achieved when both
peers participate in the idempotency contract from Phase 2 §P.2.3.

### P.3.4 Backpressure

Senders MUST honour HTTP `Retry-After`, gRPC `RESOURCE_EXHAUSTED`, or MQTT
flow-control packets. The recommended back-off is full jitter exponential with
cap 30 s and cumulative cap 5 min.


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
in lockstep across Phases 1–4 of vehicle-safety so that conformance claims at any
Phase remain unambiguous.*

