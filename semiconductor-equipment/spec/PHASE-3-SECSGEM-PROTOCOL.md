# WIA-SEMI-019 - Phase 3: SECS/GEM Protocol

> **Version:** 1.0  
> **Status:** Active  
> **Last Updated:** 2025-12-26

## 1. Overview

Phase 3 extends SEMI E5 (SECS-II), E30 (GEM), and E37 (HSMS) with WIA-specific enhancements while maintaining full backward compatibility with existing standards.

## 2. WIA SECS Messages

### 2.1 Stream 9 Extensions (S9F100-S9F199)

#### S9F100/S9F101 - WIA Equipment Specification

```
S9F100 W  // Request
  <L[0]>

S9F101    // Response
  <L[5]
    <A "WIA-SEMI-019">      // Standard
    <A "1.0">               // Version
    <A "ASML">              // Manufacturer
    <A "TWINSCAN-3600D">    // Model
    <A "{...}">             // Capabilities JSON
  >
```

#### S9F110/S9F111 - Enhanced Parameter Subscribe

```
S9F110 W  // Subscribe
  <L[3]
    <L[2]                   // Parameters
      <A "substrate_temp">
      <A "chamber_pressure">
    >
    <U4 100>                // Frequency Hz
    <A "BINARY">            // Format
  >

S9F111    // Response
  <L[2]
    <BOOLEAN TRUE>          // Accepted
    <A "STREAM_12345">      // Stream ID
  >
```

#### S9F120/S9F121 - Recipe Transfer

```
S9F120 W  // Upload Recipe
  <L[4]
    <A "RECIPE_ID">
    <A "1.0">               // Version
    <B ...>                 // Recipe data
    <A "sha256:...">        // Checksum
  >

S9F121    // Response
  <L[3]
    <BOOLEAN TRUE>          // Valid
    <A "STORED">
    <A "/recipes/...">      // Path
  >
```

## 3. Standardized SVID Ranges

| Range | Category | Examples |
|-------|----------|----------|
| 4000-4999 | Temperature | 4001: substrate_temp |
| 5000-5999 | Pressure | 5001: chamber_pressure |
| 6000-6999 | Flow | 6001: N2_flow |
| 7000-7999 | Power/RF | 7001: rf_power_fwd |
| 8000-8999 | Motion | 8001: robot_x |
| 9000-9999 | Metrology | 9001: thickness |

## 4. Enhanced Event Reporting

### 4.1 S6F11 - WIA Event Report

```
S6F11 W  // Event with context
  <L[7]
    <U4 1001>                    // CEID
    <A "WAFER_PROCESS_START">    // Event name
    <A "2025-12-26T15:30:45Z">   // Timestamp
    <L[3]                        // Context
      <L[2] <A "wafer_id"> <A "W123"> >
      <L[2] <A "lot_id"> <A "LOT001"> >
      <L[2] <A "recipe_id"> <A "RCP001"> >
    >
    <L[5]                        // Parameter snapshot
      <L[2] <A "temp"> <F4 425.0> >
      ...
    >
    <A "INFO">                   // Severity
    <A "CORR_12345">             // Correlation ID
  >
```

## 5. HSMS-TLS Security

### 5.1 TLS Configuration

- TLS 1.3 required
- Certificate-based authentication
- Encrypted SECS message payload
- Standard HSMS port 5000 (configurable)

### 5.2 S9F130/S9F131 - Authentication

```
S9F130 W  // Authenticate
  <L[3]
    <A "username">
    <A "role">
    <B ...>                 // Challenge response
  >

S9F131    // Response
  <L[4]
    <BOOLEAN TRUE>          // Success
    <A "SESSION_ABC">       // Token
    <U4 3600>               // Timeout
    <L[3]                   // Permissions
      <A "equipment.read">
      <A "command.start">
      <A "command.stop">
    >
  >
```

## 6. Process Job Management

### 6.1 S16F11/S16F12 - Create Process Job

```
S16F11 W  // Create Job
  <L[6]
    <A "JOB-001">           // Job ID
    <L[25] ... >            // Wafer IDs
    <A "RECIPE_ID">
    <L[5]                   // WIA extensions
      <L[2] <A "lot_id"> <A "LOT001"> >
      <L[2] <A "priority"> <U1 5> >
      <L[2] <A "correlation_id"> <A "MES-123"> >
      <L[2] <A "quality_level"> <A "PRODUCTION"> >
      <L[2] <A "metadata"> <A "{...}"> >
    >
    <BOOLEAN TRUE>          // Start immediate
    <A "2025-12-31T23:59Z"> // Expiration
  >

S16F12    // Response
  <BOOLEAN 0x00>  // Accepted
```

## 7. Alarm Management

### 7.1 S5F1 - Enhanced Alarm Report

```
S5F1 W  // Alarm with diagnostics
  <L[7]
    <U4 5001>                     // ALID
    <A "TEMP_EXCURSION_HIGH">     // Code
    <A "WARNING">                 // Severity
    <A "Temp exceeded by 5C">     // Message
    <A "2025-12-26T15:25:30Z">    // Timestamp
    <L[3]                         // Diagnostic data
      <L[2] <A "current"> <F4 430.2> >
      <L[2] <A "setpoint"> <F4 425.0> >
      <L[2] <A "deviation"> <F4 5.2> >
    >
    <L[2]                         // Actions
      <A "Check heater controller">
      <A "Verify cooling water flow">
    >
  >
```

## 8. Gold Certification Requirements

1. Implement all WIA SECS messages (S9F100-S9F199)
2. Use standardized SVID ranges for common parameters
3. Enhanced event reporting with context and correlation IDs
4. Support HSMS-TLS with certificate authentication
5. Process job management with WIA extensions
6. Enhanced alarm reporting with diagnostic data
7. Full backward compatibility with SEMI E5/E30/E37

---

**弘益人間 · Benefit All Humanity**

*WIA - World Certification Industry Association*  
*© 2025 MIT License*

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
in lockstep across Phases 1–4 of semiconductor-equipment so that conformance claims at any
Phase remain unambiguous.*

