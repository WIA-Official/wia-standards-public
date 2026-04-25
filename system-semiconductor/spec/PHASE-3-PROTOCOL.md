# WIA-SEMI-001: Phase 3 - Protocol Implementation

Version: 1.0
Status: Final
Date: 2025-01-15

## Overview

Phase 3 establishes communication protocols enabling chips to interact with each other and coordinate system-wide behavior. This includes inter-chip communication, power coordination, thermal management, and security protocols.

## WIA-SemiLink Protocol

### Overview

WIA-SemiLink is a lightweight protocol for chip-to-chip communication over standard interfaces (I2C, SPI, UART, or custom serial).

### Packet Structure

```c
struct wia_semilink_packet {
    // Header (8 bytes)
    uint16_t magic;           // 0x5749 ('WI' in ASCII)
    uint16_t source_addr;     // Source chip address
    uint16_t dest_addr;       // Destination chip address
    uint8_t  protocol_ver;    // Protocol version (0x10 for v1.0)
    uint8_t  flags;           // Control flags

    // Payload metadata (4 bytes)
    uint16_t payload_len;     // Payload length (0-1024 bytes)
    uint8_t  message_type;    // Message type code
    uint8_t  sequence_num;    // Sequence number

    // Payload (variable, max 1024 bytes)
    uint8_t  payload[payload_len];

    // Footer (2 bytes)
    uint16_t crc16;           // CRC-16/CCITT checksum
};
```

### Message Types

| Code | Type | Description |
|------|------|-------------|
| 0x01 | COMMAND | Command to execute action |
| 0x02 | RESPONSE | Response to command |
| 0x03 | EVENT | Asynchronous event notification |
| 0x04 | TELEMETRY | Periodic telemetry data |
| 0x05 | HEARTBEAT | Keep-alive message |

### Standard Commands

#### IDENTIFY (0x10)

Request chip identification and capabilities.

**Request:**
```json
{
  "command": "IDENTIFY"
}
```

**Response:**
```json
{
  "chipId": "WIA-SOC-2025-001",
  "manufacturer": "Example Corp",
  "capabilities": ["power-mgmt", "thermal-mgmt", "telemetry"]
}
```

#### GET_POWER_STATE (0x20)

Query current power state.

**Response:**
```json
{
  "powerConsumption": {"value": 8.5, "unit": "W"},
  "powerMode": "balanced",
  "powerBudget": {"value": 10, "unit": "W"}
}
```

#### SET_POWER_LIMIT (0x21)

Set power consumption limit.

**Request:**
```json
{
  "command": "SET_POWER_LIMIT",
  "limit": {"value": 10, "unit": "W"},
  "enforcement": "hard"
}
```

#### GET_TEMPERATURE (0x30)

Query temperature sensors.

**Response:**
```json
{
  "sensors": [
    {"location": "CPU", "temp": 72.5, "unit": "°C"},
    {"location": "GPU", "temp": 68.2, "unit": "°C"}
  ]
}
```

## Power Management Coordination

### System-Wide Power Budgeting

Chips coordinate to stay within system power budget.

#### Power Budget Broadcast

Power manager broadcasts available budget:

```json
{
  "type": "POWER_BUDGET_UPDATE",
  "totalBudget": {"value": 45, "unit": "W"},
  "allocated": {"value": 32, "unit": "W"},
  "available": {"value": 13, "unit": "W"},
  "priority": "normal"
}
```

#### Power Request Protocol

1. Chip requests additional power:
```json
{
  "type": "POWER_REQUEST",
  "chipId": "WIA-SOC-2025-001",
  "currentPower": {"value": 8, "unit": "W"},
  "requestedPower": {"value": 12, "unit": "W"},
  "duration": 5000,
  "reason": "performance-boost"
}
```

2. Power manager responds:
```json
{
  "type": "POWER_GRANT",
  "requestId": "req_123",
  "grantedPower": {"value": 10, "unit": "W"},
  "duration": 5000,
  "mustRelease": "on-thermal-event"
}
```

### Dynamic Power Reallocation

Triggers for power reallocation:
- Workload shift between components
- Thermal emergency
- Battery critical
- AC power connect/disconnect

## Thermal Management Protocols

### Thermal Event Propagation

```json
{
  "type": "THERMAL_EVENT",
  "severity": "warning",
  "source": {
    "chipId": "WIA-SOC-2025-001",
    "location": "CPU-Core0"
  },
  "temperature": {"value": 92, "unit": "°C"},
  "threshold": {"value": 95, "unit": "°C"},
  "trend": "rising",
  "estimatedTimeToThreshold": 2.5,
  "recommendedActions": [
    "reduce-frequency",
    "migrate-workload"
  ]
}
```

### Severity Levels

| Level | Temp Range | Response | Response Time |
|-------|-----------|----------|---------------|
| Info | <80°C | Normal monitoring | N/A |
| Warning | 80-90°C | Prepare to reduce power | 5 sec |
| Critical | 90-100°C | Reduce frequency 10-25% | 1 sec |
| Emergency | >100°C | Emergency power reduction | 100 ms |

### Coordinated Thermal Response

All chips in system respond to thermal events:
1. Receive thermal event notification
2. Assess own contribution to thermal load
3. Reduce power proportionally
4. Report new state to thermal manager
5. Monitor temperature trend
6. Gradually restore performance when safe

## Security Protocols

### Secure Boot Protocol

#### Challenge-Response Handshake

1. Boot initiator challenges peripheral:
```json
{
  "type": "SECURE_BOOT_CHALLENGE",
  "nonce": "a1b2c3d4e5f6g7h8",
  "timestamp": "2025-01-15T10:00:00Z"
}
```

2. Peripheral responds with signed attestation:
```json
{
  "type": "SECURE_BOOT_RESPONSE",
  "chipId": "WIA-GPU-2025-001",
  "firmwareVersion": "2.1.5",
  "firmwareHash": "sha256:8f43e8...",
  "signature": "RSA-2048:a5b2c1...",
  "certificate": "X.509 cert...",
  "nonce": "a1b2c3d4e5f6g7h8"
}
```

3. Initiator verifies and allows boot:
```json
{
  "type": "SECURE_BOOT_DECISION",
  "allowed": true,
  "trustLevel": "full",
  "restrictions": []
}
```

### Runtime Security

- **Encryption**: AES-256-GCM for all inter-chip messages
- **Authentication**: HMAC-SHA256 message authentication
- **Key Rotation**: Every 24 hours
- **Replay Protection**: Sequence numbers
- **Anomaly Detection**: Monitor communication patterns

### Security Event Protocol

```json
{
  "type": "SECURITY_EVENT",
  "severity": "critical",
  "category": "authentication-failure",
  "details": {
    "sourceChip": "UNKNOWN",
    "failureReason": "invalid-signature",
    "attemptCount": 3,
    "blocked": true
  },
  "recommendedAction": "isolate-device"
}
```

## Performance Coordination

### Workload Distribution

#### Workload Submission

```json
{
  "type": "WORKLOAD_SUBMIT",
  "workloadId": "wl_001",
  "characteristics": {
    "type": "ai-inference",
    "model": "ResNet-50",
    "batchSize": 32,
    "precision": "INT8"
  },
  "constraints": {
    "maxLatency": 50,
    "powerBudget": {"value": 5, "unit": "W"}
  }
}
```

#### Capability Analysis

```json
{
  "type": "WORKLOAD_ANALYSIS",
  "candidates": [
    {
      "chipId": "WIA-NPU-2025-001",
      "score": 95,
      "estimatedLatency": 25,
      "estimatedPower": {"value": 4.2, "unit": "W"}
    }
  ],
  "recommendation": "WIA-NPU-2025-001"
}
```

### Cache Coherency Protocols

For shared-memory systems:

#### WIA-MESI Protocol

- **Modified**: Cache line modified, exclusive to this cache
- **Exclusive**: Clean line, exclusive to this cache
- **Shared**: Clean line, may be in other caches
- **Invalid**: Line not valid

State transitions follow standard MESI protocol with optimizations for system-on-chip architectures.

## Real-Time Monitoring

### Telemetry Streaming

```json
{
  "type": "TELEMETRY_BATCH",
  "chipId": "WIA-SOC-2025-001",
  "startTime": "2025-01-15T10:30:00.000Z",
  "sampleInterval": 10,
  "samples": [
    {
      "offset": 0,
      "power": {"value": 8.2, "unit": "W"},
      "temperature": {"value": 72, "unit": "°C"},
      "frequency": {"cpu": 3200, "gpu": 950, "unit": "MHz"}
    }
  ]
}
```

### Event Categories

- **Power Events**: Mode changes, throttling
- **Thermal Events**: Temperature thresholds
- **Performance Events**: Frequency changes
- **Error Events**: Hardware errors
- **Security Events**: Authentication failures

## Transport Layer

### Supported Physical Interfaces

| Interface | Speed | Use Case |
|-----------|-------|----------|
| I2C | Up to 3.4 Mbps | Low-bandwidth control |
| SPI | Up to 50 Mbps | Medium-bandwidth data |
| UART | Up to 12 Mbps | Debug and diagnostics |
| Custom Serial | Varies | Specialized applications |

### Flow Control

- Automatic flow control using RTS/CTS
- Software flow control using XON/XOFF
- Packet-level acknowledgments

### Error Detection and Recovery

- CRC-16 checksum on all packets
- Automatic retransmission on checksum failure
- Maximum 3 retries before reporting error
- Exponential backoff on retry

## Compliance Requirements

To be Phase 3 compliant:

1. ✓ Implement WIA-SemiLink protocol
2. ✓ Support power coordination protocol
3. ✓ Generate thermal events
4. ✓ Implement telemetry streaming
5. ✓ (Optional) Secure boot support
6. ✓ (Optional) Workload coordination

## Testing and Validation

```bash
# Protocol conformance test
wia-semi-test protocol --target soc.local

# Multi-chip coordination test
wia-semi-test coordination --chips chip1.local,chip2.local

# Thermal management test
wia-semi-test thermal --stress high
```

---

**Previous**: [Phase 2 - API Interface](PHASE-2-API-INTERFACE.md)
**Next**: [Phase 4 - System Integration](PHASE-4-INTEGRATION.md)

© 2025 SmileStory Inc. / WIA
弘益人間 - Benefit All Humanity

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
in lockstep across Phases 1–4 of system-semiconductor so that conformance claims at any
Phase remain unambiguous.*

