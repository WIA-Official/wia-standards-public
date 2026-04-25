# WIA-SOC-009 Phase 3: Communication Protocol Specification

**Version:** 1.0.0  
**Status:** Approved  
**Last Updated:** 2025-12-26

弘益人間 (Hongik Ingan) - Benefit All Humanity

---

## 1. Overview

Phase 3 defines communication protocols, security requirements, and network architecture for smart sewage systems. Systems MUST implement secure, reliable, and scalable communication infrastructure.

## 2. Network Architecture

### 2.1 Layered Architecture

```
┌─────────────────────────────────────┐
│   Application Layer (Cloud/Central) │
│   - Analytics & Visualization       │
│   - Regulatory Reporting            │
│   - Predictive Models               │
└─────────────────────────────────────┘
            ↕ HTTPS/TLS 1.3
┌─────────────────────────────────────┐
│   Edge Computing Layer              │
│   - Local Processing                │
│   - Data Aggregation                │
│   - Emergency Control               │
└─────────────────────────────────────┘
            ↕ MQTT/WebSocket
┌─────────────────────────────────────┐
│   Sensor/Device Layer               │
│   - Field Sensors                   │
│   - Controllers                     │
│   - Actuators                       │
└─────────────────────────────────────┘
```

### 2.2 Communication Technologies

**Long-Range, Low-Power:**
- LoRaWAN: Sensors in remote locations
- NB-IoT: Cellular coverage areas
- Sigfox: Ultra-low bandwidth applications

**Local Networks:**
- WiFi 6: High-bandwidth local communication
- Ethernet: Wired backbone connections
- Modbus TCP/IP: Industrial equipment integration

**Backbone:**
- Fiber Optic: High-speed inter-facility links
- 4G/5G: Mobile connectivity and redundancy

## 3. Protocol Stack

### 3.1 MQTT for Sensor Data

**Broker Configuration:**
- TLS 1.3 encryption mandatory
- Username/password + client certificates
- QoS levels: 0 (at most once), 1 (at least once), 2 (exactly once)
- Retained messages for last known state
- Last will and testament for connection monitoring

**Topic Structure:**
```
wia-soc-009/{systemId}/{location}/{sensorType}/{sensorId}
```

**Examples:**
```
wia-soc-009/SYS001/influent/flow/FLOW001
wia-soc-009/SYS001/zone3/quality/pH/PH003
wia-soc-009/SYS001/pump-station-5/equipment/PUMP017
```

**Message Payload:**
```json
{
  "timestamp": "2025-12-26T14:30:00Z",
  "value": 4.2,
  "unit": "m³/s",
  "quality": "good",
  "metadata": {
    "batteryLevel": 85,
    "signalStrength": -72
  }
}
```

### 3.2 WebSocket for Real-time Streaming

**Connection:**
```
wss://api.example.com/v1/ws/stream
```

**Authentication:**
- JWT token in connection header
- Token refresh mechanism
- Session timeout: 24 hours

**Message Types:**
- `data`: Sensor readings and events
- `control`: Commands and acknowledgments
- `heartbeat`: Keep-alive messages
- `error`: Error notifications

**Example Data Message:**
```json
{
  "type": "data",
  "channel": "flow",
  "timestamp": "2025-12-26T14:30:00Z",
  "payload": {
    "locationId": "MP-001",
    "flowRate": 4.2,
    "trend": "increasing"
  }
}
```

### 3.3 RESTful API over HTTPS

**Security Requirements:**
- TLS 1.3 minimum
- HTTP/2 or HTTP/3
- HSTS (HTTP Strict Transport Security)
- Certificate pinning for critical operations

**Rate Limiting:**
- Authenticated: 1000 requests/hour
- Public endpoints: 100 requests/hour
- Burst: 10 requests/second

**Compression:**
- gzip or brotli compression
- Reduces bandwidth by 70-85%

## 4. Security Requirements

### 4.1 Authentication & Authorization

**Authentication Methods:**
1. **OAuth 2.0** for user applications
2. **API Keys** for system-to-system integration
3. **Client Certificates** for critical infrastructure
4. **Multi-Factor Authentication** for administrative access

**Authorization:**
- Role-Based Access Control (RBAC)
- Roles: viewer, operator, engineer, administrator
- Granular permissions per resource type
- Audit logging of all access

### 4.2 Encryption

**Data in Transit:**
- TLS 1.3 for all external communications
- AES-256-GCM cipher suite
- Perfect Forward Secrecy (PFS)
- Certificate rotation every 90 days

**Data at Rest:**
- AES-256 encryption for databases
- Key management via HSM or cloud KMS
- Encrypted backups
- Secure deletion procedures

### 4.3 Network Security

**Firewalling:**
- Network segmentation (sensors, edge, management)
- Whitelist-based access control
- Intrusion Detection Systems (IDS)
- DDoS protection

**VPN for Remote Access:**
- IPSec or WireGuard
- Multi-factor authentication
- Split-tunneling prohibited
- Session logging

## 5. Data Transmission Protocols

### 5.1 Sensor to Edge Gateway

**Protocol:** MQTT over TLS

**Configuration:**
```yaml
mqtt:
  broker: edge-gateway.local:8883
  client_id: SENSOR-{id}
  tls:
    ca_cert: /path/to/ca.crt
    client_cert: /path/to/client.crt
    client_key: /path/to/client.key
  qos: 1
  retain: true
  keep_alive: 60
```

**Publish Frequency:**
- Critical parameters (flow, quality): Every 60 seconds
- Equipment status: Every 5 minutes
- Environmental sensors: Every 10 minutes
- Event-driven: Immediate on threshold violation

### 5.2 Edge to Cloud

**Protocol:** HTTPS/REST API

**Data Aggregation:**
- Batch sensor readings (up to 100 per request)
- Compress with gzip
- Send every 5 minutes or on buffer full
- Store locally if connection lost (up to 24 hours)

**Resilience:**
- Automatic retry with exponential backoff
- Store-and-forward architecture
- Dual connectivity (primary + backup)
- Offline operation capability

## 6. Edge Computing Specifications

### 6.1 Edge Gateway Requirements

**Hardware:**
- ARM Cortex-A or x86-64 processor
- 2GB+ RAM
- 16GB+ storage (SSD preferred)
- Multiple communication interfaces
- Temperature range: -20°C to +60°C

**Software:**
- Linux-based OS (Ubuntu, Debian, or similar)
- Docker containerization
- Automatic updates with rollback
- Monitoring and health reporting

### 6.2 Edge Processing Capabilities

**Local Analytics:**
- Anomaly detection (statistical methods)
- Threshold monitoring
- Data quality validation
- Sensor fusion

**Emergency Control:**
- Automatic pump control during overflows
- Valve actuation for diversion
- Chemical dosing adjustment
- Alert generation and local notification

**Data Management:**
- Time-series database (InfluxDB, TimescaleDB)
- 24-hour local retention minimum
- Automated backup to cloud
- Data compression and archiving

## 7. Device Management

### 7.1 Over-the-Air (OTA) Updates

**Firmware Update Process:**
1. Cryptographically signed firmware packages
2. Incremental updates to reduce bandwidth
3. Staged rollout (canary deployment)
4. Automatic rollback on failure
5. Version tracking and auditing

**Update Scheduling:**
- During low-activity periods
- Maintenance windows
- Emergency patches within 24 hours
- Coordination across device fleet

### 7.2 Remote Configuration

**Configuration Management:**
- Centralized configuration server
- Version control for configurations
- Validation before deployment
- Audit trail of all changes

**Configurable Parameters:**
- Sampling intervals
- Alert thresholds
- Calibration coefficients
- Communication settings

## 8. Reliability and Redundancy

### 8.1 Failover Mechanisms

**Communication Redundancy:**
- Primary and backup connectivity
- Automatic failover (< 30 seconds)
- Health monitoring of all links
- Regular failover testing

**Geographic Redundancy:**
- Data replication across regions
- Distributed edge gateways
- Multi-region cloud deployment
- Disaster recovery procedures

### 8.2 Quality of Service (QoS)

**Priority Levels:**
1. **Critical**: Safety alerts, overflow warnings (highest priority)
2. **High**: Water quality violations, equipment failures
3. **Medium**: Normal sensor readings, status updates
4. **Low**: Historical data sync, batch uploads

**Network Management:**
- Traffic shaping and prioritization
- Bandwidth reservation for critical data
- Congestion avoidance
- Jitter minimization for real-time streams

## 9. Interoperability Standards

### 9.1 Protocol Support

**Required:**
- MQTT 3.1.1 or 5.0
- HTTPS/REST (OpenAPI 3.0 documented)
- WebSocket (RFC 6455)
- JSON-LD for data exchange

**Recommended:**
- OPC UA for industrial equipment
- Modbus TCP for legacy systems
- BACnet for building automation integration
- CoAP for resource-constrained devices

### 9.2 Data Exchange Formats

**Primary:** JSON-LD with WIA-SOC-009 context

**Supported:**
- CSV for bulk data export
- Protocol Buffers for high-performance applications
- MessagePack for compact encoding
- XML for legacy system integration

## 10. Monitoring and Diagnostics

### 10.1 System Health Monitoring

**Metrics to Track:**
- Connection uptime (%)
- Message delivery rate (%)
- End-to-end latency (ms)
- Bandwidth utilization (%)
- Error rates by type
- Battery levels (for wireless sensors)

**Alerting:**
- Communication failures
- Degraded performance
- Security incidents
- Certificate expiration warnings

### 10.2 Diagnostic Tools

**Network Diagnostics:**
- Ping and traceroute utilities
- Bandwidth testing
- Packet capture for troubleshooting
- Connection quality metrics

**Protocol Analysis:**
- MQTT message inspection
- API request/response logging
- WebSocket frame analysis
- Performance profiling

---

© 2025 WIA · MIT License

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
in lockstep across Phases 1–4 of sewage-system so that conformance claims at any
Phase remain unambiguous.*

