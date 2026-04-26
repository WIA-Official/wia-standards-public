# WIA Crop Monitoring Protocol Standard
## Phase 3 Specification

---

**Version**: 1.0.0
**Status**: Complete
**Date**: 2025-01
**Standard ID**: WIA-AGRI-006
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #84CC16 (Lime - AGRI)

---

## Table of Contents

1. [Overview](#overview)
2. [Communication Protocols](#communication-protocols)
3. [IoT Sensor Protocol](#iot-sensor-protocol)
4. [Camera Streaming Protocol](#camera-streaming-protocol)
5. [Data Synchronization](#data-synchronization)
6. [Security & Encryption](#security--encryption)
7. [Network Requirements](#network-requirements)
8. [Protocol Examples](#protocol-examples)
9. [Version History](#version-history)

---

## Overview

### 1.1 Purpose

The WIA Crop Monitoring Protocol Standard defines communication protocols for IoT sensors, cameras, drones, and agricultural equipment to stream real-time crop data. This ensures reliable, secure, and efficient data transmission from field to cloud.

**Core Protocols**:
- **MQTT** for lightweight IoT sensor messaging
- **WebSocket** for real-time camera streaming
- **CoAP** for constrained devices (low power)
- **HTTP/2** for high-bandwidth data uploads
- **LoRaWAN** for long-range, low-power sensors

### 1.2 Protocol Stack

```
┌─────────────────────────────────────┐
│   Application Layer                 │
│   (Crop Data Format - Phase 1)      │
├─────────────────────────────────────┤
│   Transport Layer                   │
│   MQTT | WebSocket | CoAP | HTTP/2  │
├─────────────────────────────────────┤
│   Network Layer                     │
│   IPv4/IPv6 | LoRaWAN               │
├─────────────────────────────────────┤
│   Physical Layer                    │
│   WiFi | 4G/5G | Ethernet | LoRa    │
└─────────────────────────────────────┘
```

### 1.3 Design Principles

1. **Low Bandwidth**: Optimized for rural areas with limited connectivity
2. **Reliability**: Auto-reconnect and offline buffering
3. **Real-Time**: Sub-second latency for critical alerts
4. **Security**: End-to-end encryption (TLS 1.3)
5. **Scalability**: Support 10,000+ sensors per farm

---

## Communication Protocols

### 2.1 Protocol Selection Matrix

| Use Case | Protocol | Bandwidth | Power | Range |
|----------|----------|-----------|-------|-------|
| Soil sensors | MQTT/CoAP | Low | Low | Medium |
| Weather stations | MQTT | Medium | Medium | Medium |
| Fixed cameras | WebSocket | High | High | Short |
| Drone imaging | HTTP/2 | Very High | High | Variable |
| Remote fields | LoRaWAN | Very Low | Very Low | Long |

### 2.2 MQTT for IoT Sensors

**Connection:**
```
Broker: mqtt.crop-monitoring.wiastandards.com
Port: 8883 (TLS)
Protocol: MQTT 3.1.1 / MQTT 5.0
QoS: 1 (At least once delivery)
```

**Topic Structure:**
```
wia/crop/{farmId}/{fieldId}/{sensorType}/{sensorId}

Examples:
wia/crop/FARM-KR-12345/FIELD-A-01/soil-moisture/SM-001
wia/crop/FARM-KR-12345/FIELD-A-01/temperature/TEMP-005
```

**Message Payload (JSON):**
```json
{
  "sensorId": "SM-001",
  "timestamp": "2025-06-15T10:30:00Z",
  "measurements": {
    "soilMoisture": {"value": 85, "unit": "%"},
    "soilTemperature": {"value": 22.5, "unit": "°C"}
  },
  "battery": 87,
  "signalStrength": -65
}
```

### 2.3 WebSocket for Camera Streaming

**Connection:**
```
wss://stream.crop-monitoring.wiastandards.com/v1/camera/{cameraId}
Protocol: WebSocket (RFC 6455)
Compression: permessage-deflate
```

**Handshake:**
```http
GET /v1/camera/CAM-001 HTTP/1.1
Host: stream.crop-monitoring.wiastandards.com
Upgrade: websocket
Connection: Upgrade
Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==
Sec-WebSocket-Version: 13
Authorization: Bearer {api_key}
```

**Frame Format:**
```json
{
  "type": "video-frame",
  "cameraId": "CAM-001",
  "timestamp": "2025-06-15T10:30:00.123Z",
  "sequenceNumber": 12345,
  "codec": "h264",
  "resolution": "1920x1080",
  "fps": 30,
  "data": "base64-encoded-frame-data"
}
```

### 2.4 CoAP for Low-Power Devices

**Endpoint:**
```
coaps://coap.crop-monitoring.wiastandards.com:5684/sensors
Method: POST
```

**Request:**
```
POST /sensors/SM-001/data
Content-Format: application/json

{
  "moisture": 85,
  "temp": 22.5,
  "ts": 1625097600
}
```

**Response:**
```
2.01 Created
Location-Path: /sensors/SM-001/data/1625097600
```

### 2.5 LoRaWAN for Remote Fields

**Configuration:**
```
Frequency: 915 MHz (US) / 868 MHz (EU) / 920 MHz (Asia)
Data Rate: DR0-DR5 (250 bps - 5.5 kbps)
Adaptive Data Rate (ADR): Enabled
Confirmed Uplinks: For critical data only
```

**Payload (Compact Binary):**
```
[Sensor ID: 2 bytes][Timestamp: 4 bytes][Moisture: 1 byte][Temp: 2 bytes]

Example:
0x00 0x01 0x60 0xB9 0x4A 0x80 0x55 0x0E 0x1A
```

---

## IoT Sensor Protocol

### 3.1 Sensor Registration

**MQTT Topic:**
```
wia/crop/register
```

**Registration Message:**
```json
{
  "sensorId": "SM-001",
  "farmId": "FARM-KR-12345",
  "sensorType": "soil-moisture",
  "location": {"latitude": 37.5665, "longitude": 126.9780},
  "capabilities": ["moisture", "temperature", "ec"],
  "transmissionInterval": 300,
  "protocol": "MQTT",
  "firmwareVersion": "1.2.3"
}
```

**Response:**
```json
{
  "status": "registered",
  "sensorId": "SM-001",
  "assignedTopic": "wia/crop/FARM-KR-12345/FIELD-A-01/soil-moisture/SM-001",
  "config": {
    "transmissionInterval": 300,
    "qos": 1,
    "retain": false
  }
}
```

### 3.2 Periodic Data Transmission

**Transmission Schedule:**
- Normal mode: Every 5 minutes
- Alert mode: Every 30 seconds
- Sleep mode: Every 30 minutes

**Data Message (MQTT):**
```json
{
  "sensorId": "SM-001",
  "timestamp": "2025-06-15T10:30:00Z",
  "measurements": {
    "soilMoisture": {"value": 85, "unit": "%"},
    "soilTemperature": {"value": 22.5, "unit": "°C"},
    "electricalConductivity": {"value": 1.2, "unit": "dS/m"}
  },
  "battery": 87,
  "signalStrength": -65
}
```

### 3.3 Alert Protocol

**Critical Alert (QoS 2):**
```json
{
  "type": "alert",
  "severity": "critical",
  "sensorId": "SM-001",
  "timestamp": "2025-06-15T10:30:00Z",
  "alert": {
    "code": "SOIL_MOISTURE_LOW",
    "message": "Soil moisture below critical threshold",
    "value": 25,
    "threshold": 30
  }
}
```

---

## Camera Streaming Protocol

### 4.1 Video Stream Configuration

**Stream Setup (WebSocket):**
```json
{
  "action": "start-stream",
  "cameraId": "CAM-001",
  "config": {
    "resolution": "1920x1080",
    "fps": 30,
    "codec": "h264",
    "bitrate": 2000000,
    "keyFrameInterval": 60
  }
}
```

### 4.2 AI Frame Analysis Request

**Analysis Request:**
```json
{
  "action": "analyze-frame",
  "cameraId": "CAM-001",
  "timestamp": "2025-06-15T10:30:00Z",
  "analysisTypes": ["disease-detection", "pest-detection", "growth-stage"]
}
```

**Analysis Response:**
```json
{
  "type": "analysis-result",
  "timestamp": "2025-06-15T10:30:00Z",
  "results": {
    "diseaseDetection": [
      {
        "disease": "Late Blight",
        "confidence": 0.87,
        "boundingBox": {"x": 100, "y": 150, "w": 200, "h": 180}
      }
    ],
    "growthStage": "BBCH-51",
    "healthScore": 72
  }
}
```

### 4.3 Drone Video Upload

**HTTP/2 Multipart Upload:**
```http
POST /upload/drone-footage
Authorization: Bearer {api_key}
Content-Type: multipart/form-data

--boundary
Content-Disposition: form-data; name="metadata"

{
  "droneId": "DRONE-001",
  "flightId": "FLIGHT-20250615-001",
  "fieldId": "FIELD-A-01",
  "timestamp": "2025-06-15T10:30:00Z"
}

--boundary
Content-Disposition: form-data; name="video"; filename="flight.mp4"
Content-Type: video/mp4

[Binary video data]
--boundary--
```

---

## Data Synchronization

### 5.1 Offline Buffering

**Local Storage (SQLite):**
```sql
CREATE TABLE sensor_buffer (
  id INTEGER PRIMARY KEY,
  sensor_id TEXT,
  timestamp TEXT,
  payload TEXT,
  retry_count INTEGER DEFAULT 0,
  synced BOOLEAN DEFAULT 0
);
```

**Sync Process:**
1. Attempt to send data via MQTT
2. If connection fails, store in local buffer
3. Retry every 5 minutes with exponential backoff
4. Once connected, upload all buffered data
5. Delete from buffer after successful upload

### 5.2 Time Synchronization

**NTP Configuration:**
```
NTP Servers:
- time.crop-monitoring.wiastandards.com
- pool.ntp.org

Sync Interval: Every 1 hour
Drift Tolerance: ±1 second
```

**Time Sync Message (MQTT):**
```json
{
  "action": "time-sync",
  "sensorId": "SM-001",
  "localTime": "2025-06-15T10:30:00Z",
  "requestTimestamp": 1625097600000
}
```

**Response:**
```json
{
  "action": "time-sync-response",
  "serverTime": "2025-06-15T10:30:01.234Z",
  "responseTimestamp": 1625097601234,
  "drift": 1234
}
```

---

## Security & Encryption

### 6.1 Transport Layer Security

**TLS 1.3 Configuration:**
```
Cipher Suites:
- TLS_AES_128_GCM_SHA256
- TLS_AES_256_GCM_SHA384
- TLS_CHACHA20_POLY1305_SHA256

Certificate Validation: Required
Client Certificates: Optional (for enterprise)
```

### 6.2 Message Authentication

**HMAC Signature:**
```python
import hmac
import hashlib

message = json.dumps(payload)
secret_key = "sensor_secret_key"
signature = hmac.new(
    secret_key.encode(),
    message.encode(),
    hashlib.sha256
).hexdigest()

mqtt_publish(topic, {
    "payload": payload,
    "signature": signature
})
```

### 6.3 Data Encryption

**AES-256 Encryption (Sensitive Data):**
```javascript
const crypto = require('crypto');

function encryptData(data, key) {
  const iv = crypto.randomBytes(16);
  const cipher = crypto.createCipheriv('aes-256-gcm', key, iv);

  let encrypted = cipher.update(JSON.stringify(data), 'utf8', 'hex');
  encrypted += cipher.final('hex');

  return {
    encrypted: encrypted,
    iv: iv.toString('hex'),
    authTag: cipher.getAuthTag().toString('hex')
  };
}
```

---

## Network Requirements

### 6.1 Bandwidth Requirements

| Device Type | Upstream | Downstream | Latency |
|-------------|----------|------------|---------|
| Soil Sensor | 1 kbps | 0.5 kbps | < 5 sec |
| Weather Station | 5 kbps | 1 kbps | < 2 sec |
| Fixed Camera | 2 Mbps | 100 kbps | < 500 ms |
| Drone | 10 Mbps | 1 Mbps | < 1 sec |

### 6.2 Connection Reliability

**Auto-Reconnect Logic:**
```javascript
let reconnectAttempts = 0;
const maxReconnectAttempts = 10;

function connectMQTT() {
  client.on('error', (err) => {
    console.error('Connection error:', err);
    reconnect();
  });

  client.on('close', () => {
    reconnect();
  });
}

function reconnect() {
  if (reconnectAttempts < maxReconnectAttempts) {
    const delay = Math.min(1000 * Math.pow(2, reconnectAttempts), 60000);
    setTimeout(() => {
      reconnectAttempts++;
      connectMQTT();
    }, delay);
  }
}
```

---

## Protocol Examples

### 7.1 Complete MQTT Example (Node.js)

```javascript
const mqtt = require('mqtt');

const client = mqtt.connect('mqtts://mqtt.crop-monitoring.wiastandards.com:8883', {
  clientId: 'SM-001',
  username: 'FARM-KR-12345',
  password: 'wia_api_key_1234567890abcdef',
  clean: true,
  reconnectPeriod: 5000,
  connectTimeout: 30000
});

client.on('connect', () => {
  console.log('Connected to WIA Crop Monitoring MQTT Broker');

  // Subscribe to commands
  client.subscribe('wia/crop/FARM-KR-12345/FIELD-A-01/commands/SM-001');

  // Publish sensor data every 5 minutes
  setInterval(() => {
    const data = {
      sensorId: 'SM-001',
      timestamp: new Date().toISOString(),
      measurements: {
        soilMoisture: { value: Math.random() * 50 + 50, unit: '%' },
        soilTemperature: { value: Math.random() * 10 + 20, unit: '°C' }
      },
      battery: 87,
      signalStrength: -65
    };

    client.publish(
      'wia/crop/FARM-KR-12345/FIELD-A-01/soil-moisture/SM-001',
      JSON.stringify(data),
      { qos: 1 }
    );
  }, 300000);
});
```

### 7.2 WebSocket Camera Stream (Python)

```python
import asyncio
import websockets
import json
import base64

async def stream_camera():
    uri = "wss://stream.crop-monitoring.wiastandards.com/v1/camera/CAM-001"
    headers = {"Authorization": "Bearer wia_api_key_1234567890abcdef"}

    async with websockets.connect(uri, extra_headers=headers) as websocket:
        # Start streaming
        await websocket.send(json.dumps({
            "action": "start-stream",
            "config": {
                "resolution": "1920x1080",
                "fps": 30,
                "codec": "h264"
            }
        }))

        while True:
            # Capture frame (simulated)
            frame_data = capture_frame()

            message = {
                "type": "video-frame",
                "timestamp": datetime.utcnow().isoformat() + "Z",
                "data": base64.b64encode(frame_data).decode()
            }

            await websocket.send(json.dumps(message))
            await asyncio.sleep(1/30)  # 30 fps

asyncio.run(stream_camera())
```

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-01 | Initial protocol specification |

---

**Philosophy**: 弘益人間 (Benefit All Humanity)
**License**: MIT
**Contact**: protocol-support@wiastandards.com
**Documentation**: https://docs.crop-monitoring.wiastandards.com/protocol


## Annex E — Implementation Notes for PHASE-3-PROTOCOL

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-3-PROTOCOL.

- **Operational scope** — implementations SHOULD declare their operational
  scope (single-tenant, multi-tenant, federated) in the OpenAPI document so
  that downstream auditors can score the deployment against the correct
  conformance tier in Annex A.
- **Schema evolution** — additive changes (new optional fields, new error
  codes) are non-breaking; renaming or removing fields, even in error
  payloads, MUST trigger a minor version bump.
- **Audit retention** — a 7-year retention window is sufficient to satisfy
  ISO/IEC 17065:2012 audit expectations in most jurisdictions; some
  regulators require longer retention, in which case the deployment policy
  MUST extend the retention window rather than relying on this PHASE's
  defaults.
- **Time synchronization** — sub-second deadlines depend on synchronized
  clocks. NTPv4 with stratum-2 servers is sufficient for most deadlines
  expressed in this PHASE; PTP is recommended for sites that require
  deterministic interlocks.
- **Error budget reporting** — implementations SHOULD publish a monthly
  error-budget summary (latency p95, error rate, violation hours) in the
  format defined by the WIA reporting profile to facilitate cross-vendor
  comparison without exposing tenant-specific data.

These notes are not requirements; they are a reference for field teams
mapping their existing operations onto WIA conformance.

## Annex F — Adoption Roadmap

The adoption roadmap for this PHASE document is non-normative and is intended to set expectations for early implementers about the relative stability of each section.

- **Stable** (sections marked normative with `MUST` / `MUST NOT`) — semantic versioning applies; breaking changes require a major version bump and at minimum 90 days of overlap with the prior major version on all WIA-published reference implementations.
- **Provisional** (sections in this Annex and Annex D) — items are tracked openly and may be promoted to normative status without a major version bump if community feedback supports promotion.
- **Reference** (test vectors, simulator behaviour, the reference TypeScript SDK) — versioned independently of this document so that mistakes in reference material can be corrected without amending the published PHASE document.

Implementers SHOULD subscribe to the WIA Standards GitHub release notifications to track promotions between these tiers. Comments on the roadmap are accepted via the GitHub issues tracker on the WIA-Official organization.

The roadmap is reviewed at every minor version of this PHASE document, and the review outcomes are recorded in the version-history table at the start of the document.

## Annex G — Test Vectors and Conformance Evidence

This annex describes how implementations capture and publish conformance
evidence for PHASE-3-PROTOCOL. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-3-protocol/`. Implementations claiming
  conformance MUST run all vectors in CI and publish the resulting
  pass/fail matrix in their compliance package.
- **Evidence package** — the compliance package is a tarball containing
  the SBOM (CycloneDX 1.5 or SPDX 2.3), the OpenAPI document, the test
  vector matrix, and a signed manifest. Signatures use Sigstore (DSSE
  envelope, Rekor transparency log entry) so that downstream consumers
  can verify provenance without trusting a private CA.
- **Quarterly recheck** — implementations re-publish the evidence package
  every quarter even if no source change occurred, so that consumers can
  detect environmental drift (compiler updates, dependency updates, OS
  updates) without polling vendor changelogs.
- **Cross-vendor crosswalk** — the WIA Standards working group maintains a
  crosswalk that maps each vector to the equivalent assertion in adjacent
  industry programs (where one exists), so an implementer that already
  certifies under one program can show conformance to PHASE-3-PROTOCOL with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-3-PROTOCOL does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-3-PROTOCOL.
It is non-normative; the rules below describe the policy that the WIA
Standards working group commits to when amending this PHASE document.

- **Semantic versioning** — major / minor / patch components follow
  Semantic Versioning 2.0.0 (https://semver.org/spec/v2.0.0.html).
  Major bump indicates a backwards-incompatible change to a normative
  requirement; minor bump indicates new normative requirements that do
  not break existing implementations; patch bump indicates editorial
  changes only (clarifications, typo fixes, formatting).
- **Deprecation window** — when a normative requirement is removed or
  altered in a backwards-incompatible way, the prior major version is
  maintained in parallel for at least 180 days. During the parallel
  window, both major versions are marked Stable in the WIA Standards
  registry and either may be cited as "WIA-conformant".
- **Sunset notification** — deprecated major versions enter a 12-month
  sunset window during which the WIA registry marks the version as
  Deprecated. The deprecation entry includes a migration note pointing
  to the replacement requirement(s) and an explanation of why the
  change was made.
- **Editorial errata** — patch-level errata are issued without a
  deprecation window because they do not change normative behaviour.
  Errata are tracked in a public errata register and each entry is
  signed by the WIA Standards working group chair.
- **Implementation changelog mapping** — implementations SHOULD publish
  a changelog mapping each PHASE version they support to the specific
  build, container digest, or SDK version that satisfies the version.
  This allows downstream auditors to verify version conformance without
  re-running the entire test matrix on every release.

The policy is reviewed at the same cadence as the PHASE document and
any changes to the policy itself are tracked in the version-history
table at the start of the document.

## Annex I — Interoperability Profiles

This annex describes how implementations declare interoperability profiles
for PHASE-3-PROTOCOL. The profile mechanism is non-normative and exists so that
deployments of varying scope (single tenant, regional cluster, federated
network) can advertise the subset of normative requirements they satisfy
without misrepresenting partial conformance as full conformance.

- **Profile manifest** — every implementation publishes a profile manifest
  in JSON. The manifest enumerates the normative requirement IDs from this
  PHASE that are satisfied (`status: "supported"`), partially satisfied
  (`status: "partial"`, with a reason field), or excluded
  (`status: "excluded"`, with a justification). The manifest is signed
  using the same Sigstore key used for the SBOM in Annex G.
- **Federation profile** — federated deployments publish an aggregated
  manifest summarizing the union and intersection of member-implementation
  profiles. The aggregated manifest is consumed by directory services so
  that callers can route a request to the least common denominator profile
  required for an interaction.
- **Backwards-profile compatibility** — when a deployment migrates from one
  profile to a wider profile, the prior profile manifest remains valid and
  signed for the deprecation window defined in Annex H. This preserves
  audit traceability for auditors evaluating long-term interoperability.
- **Profile registry** — the WIA Standards working group maintains a
  public registry of named profiles. Common deployment shapes (e.g.,
  "Edge-only", "Federated-with-replay") are added to the registry by
  consensus. Registry entries are immutable; new shapes are added under
  new names rather than amending existing entries.
- **Profile versioning** — profile names are versioned with the same
  Semantic Versioning rules described in Annex H. A deployment that
  advertises `WIA-P3-PROTOCOL-Edge-only/2` is asserting conformance with
  the second major version of the named profile, not the second deployment
  of an unversioned profile.

The profile mechanism is intentionally lightweight; it is meant to make
real deployment shapes visible without forcing every deployment to
satisfy every normative requirement.

## Annex J — Reference Implementation Topology

The reference implementation topology described in this annex is
non-normative; it documents the deployment shape that the WIA
Standards working group used to validate the test vectors in Annex G
and is intended as a starting point, not a recommendation against
alternative topologies.

- **Single-tenant edge** — one runtime per organization, no shared
  state. Used for early-pilot deployments where conformance evidence
  is published manually. Sufficient for PHASE-3-PROTOCOL validation when the
  organization signs the manifest itself.
- **Multi-tenant gateway** — one shared runtime serves multiple
  tenants via header-based isolation. Typically backed by a
  rate-limited gateway (Envoy or NGINX) and a shared OAuth 2.1
  identity provider. The manifest is per-tenant; the runtime
  publishes a federation manifest that aggregates tenant manifests.
- **Federated mesh** — multiple runtimes peer to one another and
  publish their manifests to a directory service. Each peer signs
  its own manifest; the directory service signs the aggregated
  index. This is the topology used by cross-organization deployments
  that need to compose conformance.
- **Air-gapped batch** — no network connection between the runtime
  and the directory service. The runtime emits a signed evidence
  package on each batch and the operator transports the package via
  out-of-band channels. This is the topology used by regulators that
  prohibit live connectivity from sensitive environments.

Implementations declare their topology in the manifest (see Annex I).
A topology change MUST be reflected in a new manifest signature; the
prior topology's manifest remains valid for the deprecation window
described in Annex H to preserve audit traceability.
