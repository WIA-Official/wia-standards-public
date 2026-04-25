# WIA-SOC-006 Phase 3: Communication Protocol Specification

**Version:** 1.0.0  
**Status:** Approved  
**Last Updated:** 2025-12-26

弘益人間 (Hongik Ingan) - Benefit All Humanity

---

## 1. Overview

Phase 3 defines the network protocols, security measures, and data transmission standards for disaster management systems. This ensures secure, reliable, and interoperable communication across all emergency response entities.

## 2. Network Architecture

### 2.1 Protocol Stack

```
Application Layer:    HTTP/3, WebSocket, MQTT
Transport Layer:      TLS 1.3, QUIC
Network Layer:        IPv4/IPv6
Data Link Layer:      Ethernet, WiFi, LTE/5G, Satellite
```

### 2.2 Primary Protocols

- **HTTP/3**: RESTful API, bulk data transfer
- **WebSocket**: Real-time bidirectional communication
- **MQTT**: IoT sensors, lightweight device communication
- **AMQP**: Message queuing for inter-agency coordination

## 3. Security Architecture

### 3.1 Encryption Standards

**Data in Transit:**
- TLS 1.3 (REQUIRED)
- Perfect Forward Secrecy (PFS) REQUIRED
- Cipher suites (minimum):
  - TLS_AES_256_GCM_SHA384
  - TLS_CHACHA20_POLY1305_SHA256

**Data at Rest:**
- AES-256-GCM (REQUIRED)
- Key rotation every 90 days (RECOMMENDED)
- Hardware Security Modules (HSM) for key storage (RECOMMENDED)

### 3.2 Authentication Mechanisms

**OAuth 2.0 Flow:**
```
1. Client requests authorization
2. Authorization server validates credentials
3. Server issues access token (JWT)
4. Client uses token for API requests
5. Token expires after configurable period
```

**JWT Structure:**
```json
{
  "header": {
    "alg": "RS256",
    "typ": "JWT"
  },
  "payload": {
    "sub": "agency-id-123",
    "name": "County Emergency Services",
    "role": "emergency_manager",
    "permissions": ["read:events", "write:alerts", "deploy:resources"],
    "iat": 1640534400,
    "exp": 1640538000
  }
}
```

### 3.3 Authorization Levels

**Roles:**
- `public`: Read public alerts only
- `first_responder`: Read all, write deployment status
- `emergency_manager`: Read all, write events/alerts/deployments
- `system_admin`: Full access including user management
- `super_admin`: Unrestricted access

**Permission Model:**
```
Resource:Action format
Examples:
  - events:read
  - events:write
  - alerts:issue
  - resources:deploy
  - data:delete
```

## 4. Real-Time Communication

### 4.1 WebSocket Protocol

**Connection Handshake:**
```
Client: 
GET /ws HTTP/1.1
Host: api.disaster-management.org
Upgrade: websocket
Connection: Upgrade
Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==
Sec-WebSocket-Version: 13
Authorization: Bearer <token>

Server:
HTTP/1.1 101 Switching Protocols
Upgrade: websocket
Connection: Upgrade
Sec-WebSocket-Accept: s3pPLMBiTxaQ9kYGzzhZRbK+xOo=
```

**Message Format:**
```json
{
  "type": "event|alert|deployment|status|command",
  "timestamp": "ISO8601",
  "sequenceId": 12345,
  "data": { /* payload */ },
  "metadata": {
    "sender": "system-id",
    "priority": "high"
  }
}
```

**Heartbeat:**
```json
{
  "type": "ping",
  "timestamp": "2025-12-26T14:00:00Z"
}
```

**Response:**
```json
{
  "type": "pong",
  "timestamp": "2025-12-26T14:00:01Z"
}
```

### 4.2 MQTT for IoT Sensors

**Topic Structure:**
```
disasters/{eventId}/sensors/{sensorType}/{sensorId}

Examples:
  disasters/EVT-2025-001/sensors/weather/WS-12345
  disasters/EVT-2025-001/sensors/seismic/SE-67890
  disasters/EVT-2025-001/sensors/water/WL-11111
```

**QoS Levels:**
- QoS 0: Sensor telemetry (fire-and-forget)
- QoS 1: Alert triggers (at least once)
- QoS 2: Critical commands (exactly once)

**Last Will and Testament:**
```json
{
  "topic": "disasters/sensors/status",
  "payload": {
    "sensorId": "WS-12345",
    "status": "offline",
    "lastSeen": "2025-12-26T14:00:00Z"
  },
  "qos": 1,
  "retain": true
}
```

## 5. Message Queuing

### 5.1 AMQP Architecture

**Exchange Types:**
- `direct`: Point-to-point agency communication
- `topic`: Filtered event broadcasting
- `fanout`: Emergency broadcasts to all agencies

**Routing Keys:**
```
{severity}.{category}.{region}

Examples:
  critical.tornado.midwest
  warning.flood.coastal
  advisory.fire.mountain
```

**Message Properties:**
```json
{
  "content_type": "application/json",
  "content_encoding": "utf-8",
  "delivery_mode": 2,
  "priority": 9,
  "correlation_id": "uuid",
  "reply_to": "response-queue",
  "expiration": "300000",
  "message_id": "msg-uuid",
  "timestamp": 1640534400000,
  "type": "alert",
  "app_id": "emergency-management-system"
}
```

## 6. Service Discovery

### 6.1 DNS-SD (DNS Service Discovery)

**Service Records:**
```
_disaster-api._tcp.local.     PTR emergency-service-1
_disaster-mqtt._tcp.local.    PTR mqtt-broker-1
_disaster-ws._tcp.local.      PTR websocket-server-1
```

**SRV Records:**
```
emergency-service-1._disaster-api._tcp.local. 
  SRV 0 5 443 api-server-1.local.
```

### 6.2 Multicast DNS (mDNS)

For local emergency network discovery when internet is unavailable.

```
emergency-system.local.  A  192.168.1.100
```

## 7. Data Synchronization

### 7.1 Conflict Resolution

**Last-Write-Wins (LWW):**
```json
{
  "data": {...},
  "version": 5,
  "timestamp": "2025-12-26T14:00:00Z",
  "lastModifiedBy": "agency-123"
}
```

**Vector Clocks:**
```json
{
  "data": {...},
  "vectorClock": {
    "agency-1": 3,
    "agency-2": 5,
    "agency-3": 2
  }
}
```

### 7.2 Delta Sync

Only transmit changes, not full documents:

```json
{
  "operation": "update",
  "resourceId": "EVT-2025-001",
  "changes": [
    {
      "path": "/status",
      "op": "replace",
      "value": "critical"
    },
    {
      "path": "/casualties",
      "op": "add",
      "value": 5
    }
  ],
  "baseVersion": 4,
  "newVersion": 5
}
```

## 8. Network Resilience

### 8.1 Failover Strategy

**Primary → Secondary → Tertiary:**
```
1. Cloud API Server (primary)
2. Regional Backup Server (secondary)
3. Local Emergency Server (tertiary)
4. Offline Mode (last resort)
```

**Health Checks:**
```
GET /health

Response:
{
  "status": "healthy",
  "services": {
    "database": "up",
    "cache": "up",
    "queue": "up"
  },
  "timestamp": "2025-12-26T14:00:00Z"
}
```

### 8.2 Circuit Breaker Pattern

**States:**
- CLOSED: Normal operation
- OPEN: Too many failures, reject requests
- HALF_OPEN: Test if service recovered

**Configuration:**
```json
{
  "failureThreshold": 5,
  "successThreshold": 2,
  "timeout": 60000,
  "resetTimeout": 300000
}
```

### 8.3 Retry Strategy

**Exponential Backoff:**
```
Attempt 1: immediate
Attempt 2: 1s delay
Attempt 3: 2s delay
Attempt 4: 4s delay
Attempt 5: 8s delay
Max: 32s delay
```

**Retry Policy:**
```json
{
  "maxAttempts": 5,
  "initialDelay": 1000,
  "maxDelay": 32000,
  "multiplier": 2,
  "retryableErrors": [
    "TIMEOUT",
    "CONNECTION_FAILED",
    "SERVICE_UNAVAILABLE"
  ]
}
```

## 9. Bandwidth Optimization

### 9.1 Compression

**Supported Algorithms:**
- Gzip (widely supported)
- Brotli (higher compression for static content)
- Zstandard (real-time compression for streams)

**Headers:**
```
Accept-Encoding: br, gzip, zstd
Content-Encoding: br
```

### 9.2 Data Prioritization

**Priority Levels:**
1. Critical alerts (highest)
2. Deployment commands
3. Status updates
4. Sensor telemetry
5. Historical queries (lowest)

**QoS Marking (DSCP):**
- Critical: EF (Expedited Forwarding)
- High: AF41 (Assured Forwarding)
- Normal: AF21
- Low: Best Effort

## 10. Offline Capabilities

### 10.1 Offline-First Architecture

**Local Storage:**
- IndexedDB for structured data
- Service Worker cache for assets
- LocalStorage for user preferences

**Sync Queue:**
```json
{
  "pendingOperations": [
    {
      "id": "op-123",
      "type": "POST",
      "endpoint": "/deployments",
      "data": {...},
      "timestamp": "2025-12-26T14:00:00Z",
      "retryCount": 0
    }
  ]
}
```

### 10.2 Conflict Resolution

When syncing after reconnection:

```
1. Check version numbers
2. If local > remote: upload local changes
3. If remote > local: download remote changes
4. If concurrent edits: present merge UI to user
```

## 11. Broadcasting

### 11.1 Emergency Alert System (EAS) Integration

**SAME Header Format:**
```
ZCZC-ORG-EEE-PSSCCC+TTTT-JJJHHMM-LLLLLLLL-
```

Where:
- ORG: Originator code
- EEE: Event code (TOR, EQW, etc.)
- PSSCCC: Location codes
- TTTT: Valid time period
- JJJHHMM: Date/time
- LLLLLLLL: Station ID

### 11.2 Wireless Emergency Alerts (WEA)

**CMAC Format:**
```xml
<alert>
  <identifier>43b080713727</identifier>
  <sender>w-nws.webmaster@noaa.gov</sender>
  <sent>2025-12-26T14:00:00-05:00</sent>
  <status>Actual</status>
  <msgType>Alert</msgType>
  <scope>Public</scope>
  <info>...</info>
</alert>
```

## 12. Monitoring & Telemetry

### 12.1 Metrics Collection

**Key Metrics:**
- Request latency (p50, p95, p99)
- Error rates
- Connection counts
- Message throughput
- Queue depths

**Format (OpenTelemetry):**
```json
{
  "metric": "api.request.duration",
  "value": 145.3,
  "unit": "ms",
  "labels": {
    "endpoint": "/events",
    "method": "POST",
    "status": "200"
  },
  "timestamp": "2025-12-26T14:00:00Z"
}
```

### 12.2 Distributed Tracing

**Trace Context Headers:**
```
traceparent: 00-0af7651916cd43dd8448eb211c80319c-b7ad6b7169203331-01
tracestate: congo=t61rcWkgMzE
```

---

© 2025 WIA · MIT License

---

## Annex A — Conformance Tier Matrix

WIA conformance for disaster-management-system is evaluated across three tiers:

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

- `wia-standards/standards/disaster-management-system/api/` — TypeScript SDK skeleton
- `wia-standards/standards/disaster-management-system/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/disaster-management-system/simulator/` — interactive browser-based simulator for the PHASE protocol

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
