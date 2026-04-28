# WIA Urban Forest Creation Protocol Standard
## Phase 3 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT

---

## Table of Contents

1. [Overview](#overview)
2. [Communication Protocols](#communication-protocols)
3. [Data Exchange](#data-exchange)
4. [IoT Integration](#iot-integration)
5. [Real-time Monitoring](#real-time-monitoring)
6. [Security](#security)

---

## Overview

### 1.1 Purpose

The WIA Urban Forest Protocol defines communication standards for real-time forest monitoring, IoT sensor integration, and data exchange between stakeholders.

### 1.2 Protocol Stack

```
┌─────────────────────────────────┐
│   Application Layer (HTTP/WS)  │
├─────────────────────────────────┤
│   Security (TLS 1.3, OAuth)    │
├─────────────────────────────────┤
│   Transport (TCP/UDP/MQTT)     │
├─────────────────────────────────┤
│   Network (IPv4/IPv6)          │
└─────────────────────────────────┘
```

---

## Communication Protocols

### 2.1 REST API (HTTP/HTTPS)

**Use Case**: Standard CRUD operations

```http
POST /api/v1/forest HTTP/1.1
Host: api.wia.live
Authorization: Bearer {token}
Content-Type: application/json

{
  "city": "Seoul",
  "trees": [...]
}
```

### 2.2 WebSocket (Real-time Updates)

**Use Case**: Live monitoring dashboard

```javascript
const ws = new WebSocket('wss://api.wia.live/ws/forest/FOREST-2025-SEOUL-001');

ws.on('message', (data) => {
  const update = JSON.parse(data);
  console.log('Tree health update:', update);
});
```

**Message Format:**
```json
{
  "event": "tree_health_update",
  "forestId": "FOREST-2025-SEOUL-001",
  "treeId": "TREE-051",
  "data": {
    "health": "healthy",
    "soilMoisture": 45,
    "temperature": 22
  },
  "timestamp": "2025-01-15T10:30:00Z"
}
```

### 2.3 MQTT (IoT Sensors)

**Use Case**: Sensor data collection

```
Topic: wia/forest/{forestId}/sensor/{sensorId}
QoS: 1 (At least once delivery)
Retain: false
```

**Payload:**
```json
{
  "sensorId": "SOIL-001",
  "forestId": "FOREST-2025-SEOUL-001",
  "type": "soil_moisture",
  "value": 45.2,
  "unit": "percentage",
  "timestamp": "2025-01-15T10:30:00Z"
}
```

---

## Data Exchange

### 3.1 Batch Upload Protocol

**Endpoint:** `POST /api/v1/forest/batch`

```json
{
  "batchId": "BATCH-2025-001",
  "forests": [
    {
      "forestId": "FOREST-2025-SEOUL-001",
      "trees": [...]
    },
    {
      "forestId": "FOREST-2025-SEOUL-002",
      "trees": [...]
    }
  ]
}
```

**Response:**
```json
{
  "status": "success",
  "processed": 2,
  "failed": 0,
  "results": [
    {"forestId": "FOREST-2025-SEOUL-001", "status": "created"},
    {"forestId": "FOREST-2025-SEOUL-002", "status": "created"}
  ]
}
```

### 3.2 Data Synchronization

**Pull-based sync:**
```http
GET /api/v1/forest/sync?since=2025-01-15T00:00:00Z
Authorization: Bearer {token}
```

**Response:**
```json
{
  "status": "success",
  "updates": [
    {
      "forestId": "FOREST-2025-SEOUL-001",
      "changes": ["tree_added", "health_updated"],
      "timestamp": "2025-01-15T10:30:00Z"
    }
  ],
  "nextSyncToken": "sync-token-12345"
}
```

---

## IoT Integration

### 4.1 Sensor Types

| Sensor Type | Protocol | Update Frequency | Data Format |
|------------|----------|------------------|-------------|
| Soil Moisture | MQTT | Every 1 hour | JSON |
| Temperature | MQTT | Every 30 min | JSON |
| Air Quality | MQTT | Every 15 min | JSON |
| Tree Growth | HTTP | Weekly | JSON |
| Camera | HTTP | On-demand | Base64 Image |

### 4.2 Soil Moisture Sensor

**MQTT Topic:** `wia/forest/{forestId}/sensor/soil`

**Payload:**
```json
{
  "sensorId": "SOIL-001",
  "location": {
    "latitude": 37.5665,
    "longitude": 126.9780
  },
  "data": {
    "moisture": 45.2,
    "temperature": 18.5,
    "pH": 6.8
  },
  "battery": 85,
  "timestamp": "2025-01-15T10:30:00Z"
}
```

### 4.3 Air Quality Sensor

**MQTT Topic:** `wia/forest/{forestId}/sensor/air`

**Payload:**
```json
{
  "sensorId": "AIR-001",
  "data": {
    "pm25": 12.5,
    "pm10": 18.3,
    "co2": 410,
    "humidity": 65
  },
  "timestamp": "2025-01-15T10:30:00Z"
}
```

### 4.4 Growth Camera

**Endpoint:** `POST /api/v1/forest/{forestId}/tree/{treeId}/image`

**Request:**
```json
{
  "treeId": "TREE-051",
  "imageType": "growth_measurement",
  "image": "data:image/jpeg;base64,/9j/4AAQSkZJRg...",
  "metadata": {
    "capturedBy": "camera-001",
    "angle": "front",
    "distance": "2m"
  },
  "timestamp": "2025-01-15T10:30:00Z"
}
```

---

## Real-time Monitoring

### 5.1 Event Stream (Server-Sent Events)

**Endpoint:** `GET /api/v1/forest/{forestId}/events`

```javascript
const eventSource = new EventSource(
  'https://api.wia.live/forest/FOREST-2025-SEOUL-001/events'
);

eventSource.addEventListener('health_alert', (e) => {
  const alert = JSON.parse(e.data);
  console.log('Health alert:', alert);
});
```

**Event Types:**
```json
{
  "event": "health_alert",
  "data": {
    "treeId": "TREE-051",
    "severity": "warning",
    "message": "Soil moisture below threshold",
    "value": 15,
    "threshold": 20
  }
}
```

### 5.2 Alert Protocol

**WebSocket:** `wss://api.wia.live/ws/alerts`

```json
{
  "type": "alert",
  "severity": "critical",
  "forestId": "FOREST-2025-SEOUL-001",
  "treeId": "TREE-051",
  "alert": "tree_health_critical",
  "message": "Tree health status changed to critical",
  "recommendations": [
    "Schedule immediate inspection",
    "Check for pest infestation",
    "Verify irrigation system"
  ],
  "timestamp": "2025-01-15T10:30:00Z"
}
```

---

## Security

### 6.1 TLS Configuration

**Minimum TLS Version:** 1.3

**Cipher Suites:**
- TLS_AES_256_GCM_SHA384
- TLS_CHACHA20_POLY1305_SHA256
- TLS_AES_128_GCM_SHA256

### 6.2 API Authentication

**OAuth 2.0 Scopes:**

| Scope | Description |
|-------|-------------|
| `forest:read` | Read forest data |
| `forest:write` | Create/update forests |
| `forest:delete` | Delete forests |
| `sensor:read` | Read sensor data |
| `sensor:write` | Write sensor data |
| `carbon:read` | Read carbon data |
| `analytics:read` | Access analytics |

### 6.3 Data Encryption

**At Rest:**
- AES-256-GCM encryption
- Encrypted database fields
- Secure key management (AWS KMS, Azure Key Vault)

**In Transit:**
- TLS 1.3 for all HTTP traffic
- MQTT over TLS
- WebSocket Secure (WSS)

### 6.4 Sensor Authentication

**MQTT Client Certificates:**
```
Topic: wia/forest/{forestId}/sensor/{sensorId}
Client Certificate: Required
Certificate Validation: CN must match sensorId
```

---

---

## 7. Federation handshake per WIA-INTENT

Hosts that federate with peer urban-forest hosts SHOULD complete a
challenge-response handshake before exchanging any per-tenant
envelope. The handshake follows the WIA Standards canonical
federation pattern: (1) the initiating host publishes a
capabilities document at `/.well-known/wia-urban-forest-capabilities`;
(2) the responder fetches the capabilities document, validates the
host certificate against the operator's trust list per WIA-AIR-SHIELD,
and records the per-host fingerprint; (3) the initiator presents a
signed `federation-intent` envelope per WIA-INTENT §3 carrying
the operator's tenant identifier, the requested operation class
(read · write · subscribe), and the per-operation scope limit
(e.g., a per-bbox geofence so a federated request cannot pull
the full per-host inventory); (4) the responder replies with a
signed `federation-grant` envelope containing the granted scope,
the granted retention window, and the per-grant audit hook;
(5) every subsequent envelope crossing the federation boundary
references the per-grant identifier so the responder can revoke
a single grant without rotating the host certificate.

## 8. Audit transport across federation

Federation envelopes carry a W3C Trace Context `traceparent`
header propagated end-to-end so the audit transport can reconstruct
a single operation across both hosts. Each host emits a structured
audit record at the host's audit transport per Phase 2 §11 with
`wia.federation.peer` carrying the responder's host fingerprint
and `wia.federation.grant_id` carrying the per-grant identifier.
Investigators reconstructing a federation crossing follow the
trace identifier from the initiator's audit transport into the
responder's audit transport without out-of-band coordination.

## 9. Continuity-of-operations envelope

Hosts running this Phase publish a continuity-of-operations
envelope per ISO 22301:2019 + ISO/IEC 27031 + NIST SP 800-34
Rev 1 covering: per-host RTO (Recovery Time Objective) per the
operator's business-impact analysis; per-host RPO (Recovery
Point Objective) tied to the host's audit-stream replication
policy; per-host backup envelope (per-region cross-replicated
immutable backup with the per-tier retention envelope per the
per-jurisdiction record-retention policy); per-host failover-
rehearsal envelope (typically quarterly per the operator's BC/DR
program); per-host vendor-exit envelope so the operator can
migrate the host to an alternate implementation without losing
audit-trail continuity. The DR envelope composes with WIA Secure
Enclave for sealed-backup envelopes and with WIA-AIR-SHIELD for
runtime trust-list re-hydration on the failover instance.

## 10. Supply-chain envelope per SLSA

Every host implementation publishes a software-bill-of-materials
(SBOM) per the operator's chosen specification: SPDX 2.3 / 3.0 per
ISO/IEC 5962 + Linux Foundation SPDX, or CycloneDX 1.6 per OWASP
Foundation. The SBOM enumerates every direct + transitive
dependency with the per-component name + version + licence +
supplier + per-component hash + per-component PURL (Package URL
per package-url spec) + per-component CPE (Common Platform
Enumeration per NIST). Supply-chain attestation follows in-toto
per CNCF in-toto + SLSA (Supply-chain Levels for Software
Artifacts) per OpenSSF SLSA Framework — typically targeting
SLSA Level 3 for hosted production deployments. The SBOM ships
alongside the host's release artefact so downstream consumers
can verify the per-release supply chain before adoption.

弘益人間 — Benefit All Humanity.

---

## 11. Privacy envelope across federation

Federation grants that allow a peer host to subscribe to citizen-
contributed data (e.g., crowd-sourced tree photographs per the
per-host citizen-science envelope) MUST honour the operator's
per-jurisdiction privacy law and MUST carry a per-grant data-
processing agreement (DPA) reference per GDPR Art 28 + Art 46
SCC envelope. Hosts that cannot present a valid DPA reference
for the requested operation class MUST decline the grant with a
`federation-grant-denied` envelope citing `dpa_required`. Subject-
rights endpoints (access, rectification, erasure, portability,
restriction, objection) flow through the host that originally
collected the data and MUST cascade across grants per the per-
DPA processing-chain envelope. Erasure cascades follow the per-
host audit-stream replication policy so that an erasure request
honoured at the originating host is provably honoured at every
peer host that received the per-grant subscription.

弘益人間 — Benefit All Humanity.
