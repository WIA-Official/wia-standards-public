# WIA Green Infrastructure Communication Protocol
## Phase 3 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #EF4444 (Red - ENE Category)

---

## Table of Contents

1. [Overview](#overview)
2. [Communication Protocols](#communication-protocols)
3. [IoT Device Integration](#iot-device-integration)
4. [Data Transmission](#data-transmission)
5. [Security](#security)
6. [Real-time Monitoring](#real-time-monitoring)
7. [Examples](#examples)

---

## Overview

### 1.1 Purpose

The WIA Green Infrastructure Communication Protocol defines standards for IoT sensor networks, real-time monitoring, and data exchange between green infrastructure systems and management platforms.

### 1.2 Supported Protocols

| Protocol | Use Case | Port |
|----------|----------|------|
| HTTPS/REST | API communication | 443 |
| MQTT | IoT sensor data | 1883/8883 |
| WebSocket | Real-time updates | 443 |
| CoAP | Constrained devices | 5683 |

---

## Communication Protocols

### 2.1 MQTT for IoT Sensors

#### Topic Structure

```
wia/green-infrastructure/{infrastructure_id}/{sensor_type}/{action}
```

**Examples:**
```
wia/green-infrastructure/GI-2025-001/soil_moisture/data
wia/green-infrastructure/GI-2025-001/temperature/data
wia/green-infrastructure/GI-2025-001/flow_meter/data
wia/green-infrastructure/GI-2025-001/status/alert
```

#### Message Format

```json
{
  "infrastructureId": "GI-2025-001",
  "sensorId": "SENSOR-GI-001",
  "type": "soil_moisture",
  "data": {
    "moisture": 65,
    "temperature": 22.5,
    "unit": "celsius"
  },
  "timestamp": "2025-01-15T10:30:00Z",
  "quality": "excellent"
}
```

#### QoS Levels

| Level | Description | Use Case |
|-------|-------------|----------|
| 0 | At most once | Non-critical data |
| 1 | At least once | Standard sensor readings |
| 2 | Exactly once | Critical alerts, commands |

### 2.2 WebSocket for Real-time Updates

#### Connection

```javascript
const ws = new WebSocket('wss://api.wia.live/green-infrastructure/v1/stream');

ws.onopen = () => {
  ws.send(JSON.stringify({
    action: 'subscribe',
    infrastructureId: 'GI-2025-001',
    dataTypes: ['soil_moisture', 'temperature', 'alerts']
  }));
};

ws.onmessage = (event) => {
  const data = JSON.parse(event.data);
  console.log('Received:', data);
};
```

#### Message Types

**Sensor Update:**
```json
{
  "type": "sensor_update",
  "infrastructureId": "GI-2025-001",
  "sensorId": "SENSOR-GI-001",
  "data": {
    "moisture": 65,
    "temperature": 22.5
  },
  "timestamp": "2025-01-15T10:30:00Z"
}
```

**Alert Notification:**
```json
{
  "type": "alert",
  "infrastructureId": "GI-2025-001",
  "severity": "warning",
  "message": "Soil moisture below threshold",
  "timestamp": "2025-01-15T10:30:00Z"
}
```

### 2.3 CoAP for Constrained Devices

```
coap://api.wia.live:5683/infrastructure/GI-2025-001/sensor
```

**Request:**
```
POST /infrastructure/GI-2025-001/sensor
Content-Format: application/json

{"moisture": 65, "temperature": 22.5}
```

---

## IoT Device Integration

### 3.1 Device Registration

```http
POST /api/v1/devices/register
```

**Request:**
```json
{
  "deviceId": "SENSOR-GI-001",
  "type": "soil_moisture",
  "infrastructureId": "GI-2025-001",
  "location": "zone_a",
  "manufacturer": "Green Sensors Inc.",
  "model": "SM-3000",
  "firmware": "v2.1.0"
}
```

### 3.2 Device Configuration

**MQTT Configuration:**
```json
{
  "broker": "mqtt.wia.live",
  "port": 8883,
  "username": "device_SENSOR-GI-001",
  "password": "encrypted_password",
  "clientId": "SENSOR-GI-001",
  "keepAlive": 60,
  "qos": 1
}
```

### 3.3 Data Collection Schedule

| Sensor Type | Frequency | Battery Impact |
|-------------|-----------|----------------|
| Soil Moisture | 15 minutes | Low |
| Temperature | 5 minutes | Low |
| Flow Meter | Real-time (on-change) | Medium |
| Vegetation Health | Daily | Very Low |
| Water Quality | 1 hour | Medium |

---

## Data Transmission

### 4.1 Batch Transmission

For battery-powered devices:

```json
{
  "deviceId": "SENSOR-GI-001",
  "readings": [
    {
      "timestamp": "2025-01-15T10:00:00Z",
      "moisture": 65,
      "temperature": 22.5
    },
    {
      "timestamp": "2025-01-15T10:15:00Z",
      "moisture": 64,
      "temperature": 22.6
    },
    {
      "timestamp": "2025-01-15T10:30:00Z",
      "moisture": 63,
      "temperature": 22.7
    }
  ],
  "battery": 85
}
```

### 4.2 Data Compression

For limited bandwidth:

```json
{
  "d": "SENSOR-GI-001",
  "r": [
    {"t": "2025-01-15T10:00:00Z", "m": 65, "temp": 22.5},
    {"t": "2025-01-15T10:15:00Z", "m": 64, "temp": 22.6}
  ],
  "b": 85
}
```

### 4.3 Error Recovery

**Retry Logic:**
- Attempt 1: Immediate
- Attempt 2: After 30 seconds
- Attempt 3: After 2 minutes
- Attempt 4: After 5 minutes
- Fallback: Store locally and sync when connected

---

## Security

### 5.1 Authentication

#### Device Certificates (TLS)

```
Device Certificate: device-cert.pem
Private Key: device-key.pem
CA Certificate: ca-cert.pem
```

#### API Keys

```http
X-API-Key: wia_gi_12345678901234567890
X-Device-ID: SENSOR-GI-001
```

### 5.2 Encryption

| Protocol | Encryption |
|----------|------------|
| HTTPS | TLS 1.3 |
| MQTT | TLS 1.2+ |
| WebSocket | WSS (TLS) |
| CoAP | DTLS 1.2 |

### 5.3 Data Integrity

**Message Signing:**
```json
{
  "data": {...},
  "signature": "sha256_hmac_signature",
  "timestamp": "2025-01-15T10:30:00Z"
}
```

---

## Real-time Monitoring

### 6.1 Alert System

**Alert Levels:**
| Level | Trigger | Response Time |
|-------|---------|---------------|
| Critical | System failure | Immediate |
| Warning | Threshold exceeded | < 5 minutes |
| Info | Status change | < 15 minutes |

**Alert Message:**
```json
{
  "alertId": "ALERT-001",
  "infrastructureId": "GI-2025-001",
  "severity": "warning",
  "type": "low_moisture",
  "message": "Soil moisture below 40% in zone_a",
  "sensorId": "SENSOR-GI-001",
  "value": 38,
  "threshold": 40,
  "timestamp": "2025-01-15T10:30:00Z",
  "actions": ["notify_admin", "trigger_irrigation"]
}
```

### 6.2 Dashboard Updates

**Subscription:**
```json
{
  "action": "subscribe",
  "infrastructureIds": ["GI-2025-001", "GI-2025-002"],
  "metrics": ["all"],
  "updateFrequency": "real-time"
}
```

**Update Message:**
```json
{
  "type": "dashboard_update",
  "infrastructureId": "GI-2025-001",
  "metrics": {
    "soilMoisture": 65,
    "temperature": 22.5,
    "flowRate": 15.5,
    "vegetationHealth": "good"
  },
  "timestamp": "2025-01-15T10:30:00Z"
}
```

---

## Examples

### 7.1 MQTT Sensor Publishing

```python
import paho.mqtt.client as mqtt
import json

client = mqtt.Client(client_id="SENSOR-GI-001")
client.username_pw_set("device_username", "device_password")
client.tls_set(ca_certs="ca-cert.pem")
client.connect("mqtt.wia.live", 8883, 60)

topic = "wia/green-infrastructure/GI-2025-001/soil_moisture/data"
payload = {
    "sensorId": "SENSOR-GI-001",
    "moisture": 65,
    "temperature": 22.5,
    "timestamp": "2025-01-15T10:30:00Z"
}

client.publish(topic, json.dumps(payload), qos=1)
```

### 7.2 WebSocket Subscription

```javascript
const ws = new WebSocket('wss://api.wia.live/green-infrastructure/v1/stream');

ws.onopen = () => {
  ws.send(JSON.stringify({
    action: 'subscribe',
    infrastructureId: 'GI-2025-001',
    dataTypes: ['soil_moisture', 'temperature', 'alerts']
  }));
};

ws.onmessage = (event) => {
  const data = JSON.parse(event.data);

  if (data.type === 'alert') {
    console.log('Alert:', data.message);
    // Trigger notification
  } else if (data.type === 'sensor_update') {
    console.log('Sensor Update:', data.data);
    // Update dashboard
  }
};
```

### 7.3 REST API with Polling

```javascript
async function pollSensorData(infrastructureId) {
  const response = await fetch(
    `https://api.wia.live/green-infrastructure/v1/infrastructure/${infrastructureId}/readings`,
    {
      headers: {
        'Authorization': 'Bearer YOUR_TOKEN'
      }
    }
  );

  const data = await response.json();
  return data;
}

// Poll every 30 seconds
setInterval(() => {
  pollSensorData('GI-2025-001')
    .then(data => console.log(data));
}, 30000);
```

---

<div align="center">

**WIA Green Infrastructure Protocol v1.0.0**

**弘益人間 (홍익인간)** - Benefit All Humanity

---

**© 2025 WIA**

**MIT License**

</div>


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
