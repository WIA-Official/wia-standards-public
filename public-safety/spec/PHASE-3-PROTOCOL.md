# WIA-SOC-004 Phase 3: Communication Protocol Specification

**Version:** 1.0.0
**Status:** Approved
**Last Updated:** 2025-12-26

弘益人間 (Hongik Ingan) - Benefit All Humanity

---

## 1. Overview

Phase 3 defines the communication protocols, network requirements, security standards, and real-time messaging specifications for public safety systems.

## 2. Network Protocols

### 2.1 HTTP/HTTPS

All REST API communications MUST use:
- HTTPS (TLS 1.3 or TLS 1.2 minimum)
- HTTP/2 or HTTP/3 preferred for performance
- Certificate validation REQUIRED
- HSTS (HTTP Strict Transport Security) RECOMMENDED

### 2.2 WebSocket (WSS)

Real-time communications MUST use:
- WebSocket Secure (WSS) over TLS 1.3
- Automatic reconnection with exponential backoff
- Heartbeat/ping-pong every 30 seconds
- Maximum message size: 1MB
- Compression RECOMMENDED (permessage-deflate)

### 2.3 MQTT (For IoT Devices)

Optional support for IoT sensors and field devices:
- MQTT 5.0 over TLS
- QoS Level 1 (at least once) for critical messages
- QoS Level 2 (exactly once) for dispatch commands
- Topic structure: `publicsafety/{jurisdiction}/{type}/{id}/{event}`
- Retained messages for status updates

Example MQTT Topics:
```
publicsafety/sf/incidents/INC-2025-0042/status
publicsafety/sf/units/AMB-101/location
publicsafety/sf/alerts/earthquake/broadcast
```

## 3. Security Requirements

### 3.1 Authentication

**Multi-Factor Authentication (MFA)**:
- REQUIRED for dispatch personnel
- REQUIRED for administrative access
- RECOMMENDED for field units

**Token-Based Authentication**:
- JWT (JSON Web Tokens) with RS256 signing
- Token expiration: 1 hour for active sessions
- Refresh tokens: valid for 24 hours
- Automatic token rotation

**API Keys**:
- Minimum 256-bit entropy
- Rotation every 90 days RECOMMENDED
- Scoped permissions per key

### 3.2 Authorization

Role-Based Access Control (RBAC):

```json
{
  "roles": {
    "dispatcher": {
      "permissions": [
        "incidents:read",
        "incidents:create",
        "incidents:update",
        "units:read",
        "dispatch:create",
        "communications:read",
        "communications:create"
      ]
    },
    "supervisor": {
      "inherits": ["dispatcher"],
      "permissions": [
        "incidents:delete",
        "units:manage",
        "analytics:read",
        "alerts:create"
      ]
    },
    "field_unit": {
      "permissions": [
        "incidents:read:assigned",
        "units:update:self",
        "communications:create",
        "location:update:self"
      ]
    },
    "admin": {
      "permissions": ["*"]
    }
  }
}
```

### 3.3 Encryption

**Data in Transit**:
- TLS 1.3 REQUIRED for all communications
- Perfect Forward Secrecy (PFS) REQUIRED
- Certificate pinning RECOMMENDED for mobile apps
- End-to-end encryption for sensitive communications

**Data at Rest**:
- AES-256-GCM for database encryption
- Encrypted backups
- Hardware Security Modules (HSM) for key storage
- Key rotation every 90 days

**PII Protection**:
- Tokenization for phone numbers
- Hashing for non-reversible identifiers
- Field-level encryption for medical data
- Anonymization for analytics

### 3.4 Audit Logging

All security-relevant events MUST be logged:

```json
{
  "@type": "AuditLog",
  "timestamp": "2025-12-26T14:30:00Z",
  "actor": {
    "userId": "DISP-042",
    "role": "dispatcher",
    "ipAddress": "10.0.1.42",
    "userAgent": "DispatchClient/1.0"
  },
  "action": "incident:create",
  "resource": {
    "type": "incident",
    "id": "INC-2025-0042"
  },
  "result": "success",
  "metadata": {
    "priority": "critical",
    "incidentType": "medical"
  }
}
```

Audit logs MUST:
- Be immutable (write-once, append-only)
- Be retained for minimum 7 years
- Support real-time analysis for security monitoring
- Include failed authentication attempts
- Trigger alerts on suspicious patterns

## 4. Real-Time Messaging

### 4.1 WebSocket Protocol

**Connection Establishment**:
```javascript
const ws = new WebSocket('wss://api.publicsafety.example.com/v1/ws');

ws.onopen = () => {
  ws.send(JSON.stringify({
    action: 'authenticate',
    token: 'eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...'
  }));
};
```

**Message Format**:
```json
{
  "messageId": "MSG-2025-0042",
  "timestamp": "2025-12-26T14:30:00.123Z",
  "type": "event|command|response",
  "action": "incident.created|unit.dispatched|alert.issued",
  "data": {...}
}
```

**Heartbeat**:
```json
{
  "type": "ping",
  "timestamp": "2025-12-26T14:30:00Z"
}
```

Response:
```json
{
  "type": "pong",
  "timestamp": "2025-12-26T14:30:00Z"
}
```

### 4.2 Event Subscription

**Wildcard Patterns**:
- `incidents.*` - All incident events
- `incidents.*.critical` - Critical incidents only
- `units.AMB-101.*` - Specific unit events
- `alerts.*` - All alert notifications

**Subscription Message**:
```json
{
  "action": "subscribe",
  "topics": [
    "incidents.*.critical",
    "units.*.location",
    "alerts.earthquake"
  ],
  "filters": {
    "jurisdiction": "sf",
    "priority": ["high", "critical"]
  }
}
```

**Unsubscribe**:
```json
{
  "action": "unsubscribe",
  "topics": ["incidents.*"]
}
```

### 4.3 Quality of Service

**Message Priority**:
- Critical: Alerts, dispatch commands (immediate delivery)
- High: Status updates, location updates (< 1 second delay)
- Normal: Analytics, logs (best effort)

**Delivery Guarantees**:
- At-least-once delivery for critical messages
- Acknowledgment required:
```json
{
  "type": "ack",
  "messageId": "MSG-2025-0042",
  "timestamp": "2025-12-26T14:30:01Z"
}
```

**Retry Logic**:
- Exponential backoff: 1s, 2s, 4s, 8s, 16s
- Maximum 5 retries
- Dead letter queue for failed messages

## 5. Network Requirements

### 5.1 Bandwidth

Minimum requirements per connection:
- Dispatcher console: 1 Mbps down, 512 Kbps up
- Field unit: 256 Kbps down, 128 Kbps up
- Mobile app: 128 Kbps down, 64 Kbps up

### 5.2 Latency

Maximum acceptable latency:
- Critical commands: < 100ms
- Status updates: < 500ms
- Location updates: < 1000ms
- Analytics queries: < 5000ms

### 5.3 Availability

System requirements:
- 99.99% uptime (52 minutes downtime/year)
- Automatic failover < 30 seconds
- Geographic redundancy
- Load balancing across multiple data centers

### 5.4 Disaster Recovery

Backup systems MUST:
- Maintain secondary data center
- Replicate data in real-time
- Support automatic failover
- Recover within RTO (Recovery Time Objective): 1 hour
- RPO (Recovery Point Objective): 0 (no data loss)

## 6. Radio Communication Integration

### 6.1 P25 Integration

Support for Project 25 (P25) digital radio:
- P25 Phase 2 TDMA
- Encryption (AES-256)
- Interoperability with legacy systems
- Gateway for radio-to-IP bridge

### 6.2 DMR Integration

Digital Mobile Radio support:
- DMR Tier II and III
- ETSI TS 102 361 compliance
- GPS location from radio units

### 6.3 LTE/5G Integration

FirstNet and commercial LTE:
- Priority and Preemption support
- Mission Critical Push-To-Talk (MCPTT)
- 3GPP standards compliance
- Quality of Service (QoS) Class Identifier

## 7. Emergency Alert System Integration

### 7.1 Wireless Emergency Alerts (WEA)

FEMA IPAWS integration:
- CAP (Common Alerting Protocol) v1.2
- CMAC authentication
- WEA 3.0 features (links, multimedia)
- Language support

### 7.2 Emergency Alert System (EAS)

Broadcast integration:
- EAS CAP compliance
- FCC Part 11 requirements
- SAME (Specific Area Message Encoding)
- FIPS county codes

## 8. Interoperability Standards

### 8.1 NIEM Compliance

National Information Exchange Model:
- NIEM 5.0 schema
- Emergency Management domain
- Justice domain
- Core components

### 8.2 EDXL Standards

Emergency Data Exchange Language:
- EDXL-DE (Distribution Element)
- EDXL-RM (Resource Messaging)
- EDXL-SitRep (Situation Reporting)
- EDXL-HAVE (Hospital Availability Exchange)

### 8.3 CAP (Common Alerting Protocol)

Alert message structure:
```xml
<?xml version="1.0" encoding="UTF-8"?>
<alert xmlns="urn:oasis:names:tc:emergency:cap:1.2">
  <identifier>INC-2025-0042</identifier>
  <sender>sf.publicsafety@example.com</sender>
  <sent>2025-12-26T14:30:00-08:00</sent>
  <status>Actual</status>
  <msgType>Alert</msgType>
  <scope>Public</scope>
  <info>
    <category>Safety</category>
    <event>Earthquake</event>
    <urgency>Immediate</urgency>
    <severity>Extreme</severity>
    <certainty>Observed</certainty>
    <area>
      <areaDesc>San Francisco Bay Area</areaDesc>
      <polygon>37.8,-122.5 37.8,-122.3 37.7,-122.3 37.7,-122.5</polygon>
    </area>
  </info>
</alert>
```

## 9. Performance Monitoring

### 9.1 Metrics

Track and report:
- API response times (p50, p95, p99)
- WebSocket connection count
- Message throughput (messages/second)
- Error rates
- Network latency
- Database query performance

### 9.2 Health Checks

**API Health Endpoint**:
```
GET /health
```

Response:
```json
{
  "status": "healthy",
  "timestamp": "2025-12-26T14:30:00Z",
  "version": "1.0.0",
  "components": {
    "database": "healthy",
    "cache": "healthy",
    "messageQueue": "healthy",
    "externalAPIs": "degraded"
  },
  "metrics": {
    "activeConnections": 1250,
    "requestsPerSecond": 450,
    "avgResponseTime": 45
  }
}
```

---

© 2025 WIA · MIT License

---

## Annex A — Conformance Tier Matrix

WIA conformance for WIA-SOC-PUBLIC-SAFETY (Public Safety) is evaluated across three tiers, applied to dispatch · alerting · responder coordination · evidence chain-of-custody:

| Tier | Scope | Mandatory artifacts | Audit cadence |
|------|-------|--------------------|----------------|
| Tier 1 — Self-declared | Internal use, pilot deployments | OpenAPI 3.0 contract, JSON Schema validation report, security threat model | None (annual self-review recommended) |
| Tier 2 — Third-party assessed | External partners, B2B integrations | Tier 1 artifacts + signed third-party assessor report against this PHASE | Every 24 months |
| Tier 3 — Accredited | Public-facing or regulated deployments | Tier 2 artifacts + WIA accreditation, ISO/IEC 17065:2012 conformity assessment, evidence retention ≥ 7 years | Every 12 months |

Implementations MUST clearly disclose their conformance tier in the OpenAPI `info.x-wia-tier` extension and on any public certification page. Tier downgrade events MUST be reported to the WIA registry within 30 days.

---

## Annex B — Cross-Walk to International Standards

The PHASE specification reuses or normatively references the following published standards. Implementers SHOULD review the listed standards alongside this PHASE document; where a conflict exists, the more specific WIA requirement governs unless explicitly superseded by a binding national regulation.

- OASIS CAP 1.2 — Common Alerting Protocol
- OASIS EDXL-DE 2.0 — Emergency Data Exchange Language
- ISO 22320:2018 — Emergency management — Guidelines for incident management
- NIST SP 800-53 Rev. 5 — Security and privacy controls (federal reference baseline)
- IETF RFC 8259 — JSON data interchange (alert payload encoding)

This cross-walk is informative only. WIA does not republish the referenced documents; readers MUST obtain authoritative copies from the issuing body. Cross-walk entries are reviewed at every minor version of this PHASE.

---

## Annex C — Reference Implementations and Test Vectors

### C.1 Reference Implementations

WIA does not require implementers to use a particular library, but maintains pointers to the canonical reference implementation directories under the WIA-Official GitHub organization:

- `wia-standards/standards/public-safety/api/` — TypeScript SDK skeleton
- `wia-standards/standards/public-safety/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/public-safety/simulator/` — interactive browser-based simulator for the PHASE protocol

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
