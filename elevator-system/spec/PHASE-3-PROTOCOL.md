# WIA Elevator System Standard
## Phase 3: Safety & Communication Protocols v1.0

**Status:** APPROVED  
**Date:** 2025-12-26  
**弘益人間** · Benefit All Humanity

---

## 1. Introduction

Phase 3 defines safety interlocks, communication protocols, and emergency procedures for WIA-compliant elevator systems. This ensures EN 81-20/50 compliance, secure data transmission, and reliable emergency response.

### 1.1 Scope

- Digital safety protocols
- MQTT messaging
- CoAP for IoT devices
- TLS/DTLS encryption
- Emergency communication
- Fire service protocols
- Certificate management

---

## 2. Safety Interlock Protocols

### 2.1 Door Safety Interlock

**Digital Safety Message:**
```json
{
  "type": "SAFETY_INTERLOCK",
  "component": "DOOR",
  "timestamp": "2025-12-26T10:30:00Z",
  "checks": {
    "doorLockVerified": true,
    "obstacleDetection": "CLEAR",
    "reopenSensor": "ACTIVE",
    "forceLimit": "WITHIN_SPEC"
  },
  "result": "SAFE_TO_OPERATE"
}
```

**Safety Rules:**
1. Elevator MUST NOT move if `doorLockVerified = false`
2. Doors MUST reopen if `obstacleDetection = OBSTRUCTION`
3. Force limit MUST NOT exceed 150N (EN 81-20 clause 5.3.6.2)

### 2.2 Overload Protection

```json
{
  "type": "SAFETY_INTERLOCK",
  "component": "LOAD",
  "maxWeight": 1000,
  "currentWeight": 1050,
  "overloadThreshold": 950,
  "result": "OVERLOAD_DETECTED",
  "action": "PREVENT_DOOR_CLOSE"
}
```

### 2.3 Overspeed Governor

```json
{
  "type": "SAFETY_INTERLOCK",
  "component": "OVERSPEED",
  "ratedSpeed": 2.5,
  "currentSpeed": 2.8,
  "tripSpeed": 3.0,
  "result": "APPROACHING_TRIP",
  "action": "REDUCE_SPEED"
}
```

---

## 3. MQTT Protocol Specification

### 3.1 Topic Structure

```
wia/elevator/{buildingId}/{elevatorId}/{category}/{subcategory}
```

**Examples:**
- `wia/elevator/BLD-2025/ELV-001/status` - Status updates
- `wia/elevator/BLD-2025/ELV-001/telemetry/sensors` - Sensor data
- `wia/elevator/BLD-2025/dispatch/request` - Dispatch requests
- `wia/elevator/BLD-2025/ELV-001/maintenance/alert` - Maintenance alerts

### 3.2 QoS Levels

- **QoS 0 (At most once):** Telemetry data (non-critical)
- **QoS 1 (At least once):** Status updates, events
- **QoS 2 (Exactly once):** Control commands, safety messages

### 3.3 Message Format

**Status Update:**
```json
{
  "topic": "wia/elevator/BLD-2025/ELV-001/status",
  "qos": 1,
  "retain": true,
  "payload": {
    "elevatorId": "ELV-001",
    "timestamp": "2025-12-26T10:30:00Z",
    "status": {
      "currentFloor": 5,
      "direction": "UP",
      "doorStatus": "CLOSED"
    }
  }
}
```

**Control Command:**
```json
{
  "topic": "wia/elevator/BLD-2025/ELV-001/command",
  "qos": 2,
  "payload": {
    "commandId": "CMD-12345",
    "command": "GO_TO_FLOOR",
    "parameters": {
      "targetFloor": 10
    },
    "timestamp": "2025-12-26T10:30:05Z"
  }
}
```

### 3.4 Last Will and Testament (LWT)

Configure LWT for offline detection:
```json
{
  "topic": "wia/elevator/BLD-2025/ELV-001/status/lwt",
  "qos": 1,
  "retain": true,
  "message": {
    "elevatorId": "ELV-001",
    "status": "OFFLINE",
    "timestamp": "2025-12-26T10:30:00Z"
  }
}
```

---

## 4. CoAP Protocol for IoT Sensors

### 4.1 CoAP URIs

```
coap://elevator.local/status
coap://elevator.local/sensors/load
coap://elevator.local/sensors/temperature
```

### 4.2 CoAP Methods

- **GET:** Retrieve sensor data
- **POST:** Update configuration
- **PUT:** Full resource update
- **OBSERVE:** Subscribe to changes

### 4.3 Example Request/Response

**GET Request:**
```
GET coap://elevator.local/sensors/load
Accept: application/json
```

**Response:**
```
2.05 Content
Content-Format: application/json

{
  "sensorId": "LOAD-001",
  "elevatorId": "ELV-001",
  "weight": 640,
  "unit": "kg",
  "timestamp": "2025-12-26T10:30:00Z"
}
```

### 4.4 DTLS Security

All CoAP communications MUST use DTLS 1.3:
```
coaps://elevator.local/sensors/load
```

---

## 5. TLS/DTLS Encryption

### 5.1 TLS Requirements

- **Minimum version:** TLS 1.3
- **Cipher suites:** 
  - `TLS_AES_256_GCM_SHA384`
  - `TLS_CHACHA20_POLY1305_SHA256`
- **Certificate validation:** REQUIRED
- **Mutual TLS:** RECOMMENDED

### 5.2 Certificate Requirements

**X.509 v3 Certificate:**
```
Subject: CN=ELV-001, O=Building Management, C=US
Issuer: CN=WIA Certificate Authority
Valid From: 2025-01-01T00:00:00Z
Valid To: 2027-01-01T00:00:00Z
Key Usage: Digital Signature, Key Encipherment
Extended Key Usage: Server Authentication, Client Authentication
```

### 5.3 Certificate Pinning

Pin WIA root CA certificate:
```
SHA256:a1b2c3d4e5f6...
```

---

## 6. Emergency Communication Protocols

### 6.1 Emergency Stop

**Emergency stop message (QoS 2):**
```json
{
  "type": "EMERGENCY_STOP",
  "elevatorId": "ELV-001",
  "timestamp": "2025-12-26T10:31:00Z",
  "triggeredBy": "PASSENGER_BUTTON",
  "location": {
    "floor": 7,
    "position": 21.5
  },
  "actions": [
    "STOP_MOVEMENT",
    "ACTIVATE_ALARM",
    "NOTIFY_EMERGENCY_SERVICES",
    "ENABLE_TWO_WAY_COMM"
  ]
}
```

### 6.2 Two-Way Emergency Communication

**Protocol:** VoIP over TLS
```json
{
  "type": "EMERGENCY_CALL",
  "elevatorId": "ELV-001",
  "callId": "EMERG-2025-001",
  "codec": "G.711",
  "sampleRate": 8000,
  "videoEnabled": true,
  "recipientList": [
    "emergency-center@building.com",
    "security@building.com"
  ]
}
```

---

## 7. Fire Service Operation Mode

### 7.1 Fire Alarm Integration

**Fire alarm trigger:**
```json
{
  "type": "FIRE_ALARM_ACTIVATION",
  "buildingId": "BLD-2025",
  "timestamp": "2025-12-26T10:35:00Z",
  "alarmZone": "FLOOR-5",
  "severity": "CRITICAL",
  "elevatorActions": {
    "allElevators": "RECALL_TO_DESIGNATED_FLOOR",
    "recallFloor": 1,
    "disableNormalOperation": true,
    "enableFiremanService": true
  }
}
```

### 7.2 Fireman Service Control

**Digital fireman interface:**
```json
{
  "mode": "FIREMAN_SERVICE_PHASE_II",
  "elevatorId": "ELV-001",
  "controlMethod": "DIGITAL_PANEL",
  "authentication": {
    "keyType": "FIRE_DEPARTMENT_KEY",
    "keyId": "FD-KEY-2025-123"
  },
  "manualControl": {
    "targetFloor": 5,
    "doorControl": "MANUAL",
    "carLightsOn": true
  }
}
```

---

## 8. Earthquake Protocol

### 8.1 Seismic Sensor Integration

**Earthquake detection:**
```json
{
  "type": "SEISMIC_EVENT",
  "buildingId": "BLD-2025",
  "timestamp": "2025-12-26T10:40:00Z",
  "magnitude": 5.2,
  "intensity": "MODERATE",
  "elevatorActions": {
    "allElevators": "STOP_AT_NEAREST_FLOOR",
    "openDoors": true,
    "disableOperation": true,
    "awaitInspection": true
  }
}
```

---

## 9. Compliance Testing

### 9.1 Safety Interlock Tests

1. ✅ Door interlock prevents movement
2. ✅ Overload prevents door closing
3. ✅ Overspeed triggers emergency brake
4. ✅ Emergency stop halts operation

### 9.2 Protocol Tests

1. ✅ MQTT QoS levels enforced
2. ✅ TLS 1.3 encryption verified
3. ✅ Certificate validation working
4. ✅ Emergency communication functional

---

**© 2025 WIA - World Certification Industry Association**  
**MIT License**  
**弘益人間 · Benefit All Humanity**


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
