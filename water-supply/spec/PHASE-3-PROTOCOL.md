# WIA-SOC-008 Phase 3: Communication Protocol Specification

**Version:** 1.0.0  
**Status:** Approved  
**Last Updated:** 2025-12-26

弘益人間 (Hongik Ingan) - Benefit All Humanity

---

## 1. Overview

Phase 3 defines communication protocols, security standards, device discovery, and firmware update mechanisms for water supply IoT infrastructure.

## 2. Supported Protocols

### 2.1 MQTT

**Broker Requirements:**
- MQTT 3.1.1 or 5.0
- TLS 1.3 encryption
- Authentication required
- QoS levels: 0, 1, 2 supported

**Topic Structure:**
```
wia/soc-008/{systemId}/{deviceType}/{deviceId}/{dataType}

Examples:
wia/soc-008/WS-2025-001/sensor/SENSOR-042/water-quality
wia/soc-008/WS-2025-001/meter/METER-123/consumption
wia/soc-008/WS-2025-001/pump/PUMP-07/status
```

### 2.2 CoAP

For low-power devices:
- CoAP RFC 7252
- DTLS 1.3 for security
- Observable resources
- Block-wise transfer

### 2.3 OPC UA

For industrial SCADA integration:
- OPC UA specification Part 1-14
- UA Binary or UA JSON encoding
- Certificate-based security
- Historical data access

## 3. Security

### 3.1 Encryption

**Transport Security:**
- TLS 1.3 (REQUIRED for internet)
- DTLS 1.3 (REQUIRED for UDP protocols)
- Minimum cipher suite: TLS_AES_128_GCM_SHA256
- Perfect forward secrecy (PFS) enabled

**Application Security:**
- End-to-end encryption for sensitive data
- Payload encryption: AES-256-GCM
- Key rotation: Every 90 days

### 3.2 Authentication

**Device Authentication:**
- X.509 certificates (RECOMMENDED)
- Pre-shared keys (PSK) for constrained devices
- OAuth 2.0 for cloud services

**User Authentication:**
- Multi-factor authentication (MFA) for operators
- Single sign-on (SSO) integration
- Role-based access control (RBAC)

### 3.3 Network Security

**Segmentation:**
- Isolated OT (Operational Technology) network
- DMZ for external access
- VLAN separation for different device types

**Firewall Rules:**
- Default deny all
- Whitelist approach for allowed traffic
- Intrusion detection systems (IDS)

## 4. Device Discovery

### 4.1 mDNS/DNS-SD

**Service Type:**
```
_wia-water._tcp.local
```

**TXT Records:**
```
txtvers=1
systemId=WS-2025-001
deviceType=sensor
capabilities=water-quality,pressure,flow
certLevel=STANDARD
version=1.0.0
```

### 4.2 DDS Discovery

For real-time systems:
- OMG DDS specification
- Quality of Service (QoS) policies
- Content-filtered topics

## 5. Data Transmission

### 5.1 Message Format

**MQTT Payload (JSON):**
```json
{
  "v": "1.0.0",
  "ts": "2025-12-26T14:32:15Z",
  "id": "SENSOR-042",
  "type": "water-quality",
  "data": {
    "pH": 7.3,
    "turbidity": 0.8,
    "chlorine": 0.5
  },
  "sig": "base64-signature"
}
```

### 5.2 Compression

- Gzip for large payloads (> 1KB)
- Protocol Buffers for high-frequency data
- CBOR for constrained devices

### 5.3 Batching

- Multiple readings in single message
- Maximum batch size: 100 readings or 1MB
- Maximum batch interval: 5 minutes

## 6. Firmware Updates

### 6.1 OTA (Over-The-Air) Updates

**Update Process:**
1. Version check
2. Download firmware package
3. Verify digital signature
4. Install during maintenance window
5. Rollback on failure

**Security:**
- Code signing (RSA 4096 or ECC P-384)
- Encrypted firmware images
- Secure boot verification

### 6.2 Update Scheduling

- Configurable maintenance windows
- Staggered rollout (10% → 50% → 100%)
- Automatic rollback threshold: 5% failure rate

## 7. Reliability

### 7.1 Quality of Service

| Data Type | QoS Level | Reliability |
|-----------|-----------|-------------|
| Critical Alerts | QoS 2 | Exactly once |
| Water Quality | QoS 1 | At least once |
| Telemetry | QoS 0 | At most once |

### 7.2 Offline Operation

- Local data buffering (minimum 24 hours)
- Automatic reconnection with exponential backoff
- Data synchronization on reconnection

### 7.3 Redundancy

- Dual-path communication (primary + backup)
- Automatic failover (< 30 seconds)
- Health monitoring and heartbeats

## 8. Performance

| Metric | Requirement |
|--------|-------------|
| Message Latency | < 500ms (p95) |
| Throughput | > 10,000 msg/sec per broker |
| Concurrent Connections | > 100,000 devices |
| Message Size | < 256KB |
| Bandwidth | Optimized for 2G/3G/4G/5G/LoRaWAN |

## 9. Compliance

- IEC 62443 (Industrial cybersecurity)
- NIST Cybersecurity Framework
- ISO/IEC 27001 (Information security)
- Local regulatory requirements

---

**弘益人間 · Benefit All Humanity**

© 2025 WIA / SmileStory Inc. · MIT License

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
in lockstep across Phases 1–4 of water-supply so that conformance claims at any
Phase remain unambiguous.*

