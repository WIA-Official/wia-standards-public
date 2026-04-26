# WIA Elevator System Standard
## Phase 4: System Integration Specification v1.0

**Status:** APPROVED  
**Date:** 2025-12-26  
**弘益人間** · Benefit All Humanity

---

## 1. Introduction

Phase 4 defines integration with Building Management Systems (BMS), access control, fire alarms, cloud platforms, and third-party applications. This enables elevators to participate in the broader smart building ecosystem.

### 1.1 Scope

- BMS integration (BACnet, Modbus)
- Access control systems
- Fire alarm coordination
- Cloud platform adapters
- Energy management
- Predictive maintenance
- Third-party app ecosystem

---

## 2. BMS Integration

### 2.1 BACnet Integration

**Elevator as BACnet Device:**
```
Device Instance: 2025001
Device Name: "ELV-001"
Device Type: "Elevator Controller"
Protocol Services: ReadProperty, WriteProperty, Subscribe-COV
```

**BACnet Objects:**
- `ANALOG_VALUE` - Current floor, speed, load weight
- `BINARY_VALUE` - Door status, direction
- `MULTI_STATE_VALUE` - Elevator state (IDLE, MOVING_UP, MOVING_DOWN)
- `NOTIFICATION_CLASS` - Alarms and events

**Example BACnet Read:**
```
ReadProperty Request:
  Object: ANALOG_VALUE:1 (Current Floor)
  Property: PRESENT_VALUE

Response:
  Present Value: 5.0
```

### 2.2 Modbus Integration

**Modbus Register Map:**

| Register | Type    | Description         | Unit | Access |
|----------|---------|---------------------|------|--------|
| 40001    | Holding | Current Floor       | -    | R      |
| 40002    | Holding | Direction (0/1/2)   | -    | R      |
| 40003    | Holding | Door Status (0/1/2) | -    | R      |
| 40004    | Holding | Load Weight         | kg   | R      |
| 40005    | Holding | Target Floor        | -    | R/W    |
| 40006    | Holding | Mode (0=Auto, 1=Maint) | - | R/W    |

**Modbus Request Example:**
```
Function Code: 03 (Read Holding Registers)
Starting Address: 40001
Quantity: 6

Response:
  [5, 1, 0, 640, 10, 0]
  (Floor 5, Moving UP, Doors OPEN, 640kg load, Target floor 10, Auto mode)
```

---

## 3. Access Control Integration

### 3.1 OSDP Protocol

**Elevator as OSDP Peripheral Device:**
```json
{
  "protocol": "OSDP",
  "version": "2.2",
  "address": 1,
  "cardReaderIntegration": {
    "enabled": true,
    "location": "ELEVATOR_CAR",
    "formats": ["WIEGAND-26", "WIEGAND-37", "ABA-TRACK2"]
  }
}
```

**Badge Read Event:**
```json
{
  "event": "CARD_READ",
  "elevatorId": "ELV-001",
  "cardData": "12345678",
  "timestamp": "2025-12-26T10:30:00Z",
  "action": "REQUEST_FLOOR_ACCESS",
  "authorization": {
    "userId": "USER-001",
    "authorizedFloors": [1, 5, 10, 15],
    "accessGranted": true
  }
}
```

### 3.2 Floor Access Control

**Access rules:**
```json
{
  "userId": "USER-001",
  "buildingId": "BLD-2025",
  "accessRules": {
    "timeRestrictions": {
      "weekdays": "06:00-20:00",
      "weekends": "DENIED"
    },
    "floorAccess": [
      {"floor": 1, "access": "ALWAYS"},
      {"floor": 5, "access": "BUSINESS_HOURS"},
      {"floor": 10, "access": "ALWAYS"},
      {"floor": 15, "access": "ESCORT_REQUIRED"}
    ]
  }
}
```

---

## 4. Fire Alarm System Integration

### 4.1 Fire Alarm Protocol

**Interface:** Digital I/O + MQTT

**Fire alarm signal:**
```json
{
  "type": "FIRE_ALARM_SIGNAL",
  "buildingId": "BLD-2025",
  "timestamp": "2025-12-26T10:45:00Z",
  "alarmPanel": "PANEL-ZONE-5",
  "alarmType": "SMOKE_DETECTOR",
  "floor": 5,
  "elevatorResponse": {
    "allElevators": [
      {
        "elevatorId": "ELV-001",
        "action": "RECALL_TO_FLOOR_1",
        "status": "EXECUTING"
      },
      {
        "elevatorId": "ELV-002",
        "action": "RECALL_TO_FLOOR_1",
        "status": "EXECUTING"
      }
    ]
  }
}
```

### 4.2 Firefighter Override

```json
{
  "type": "FIREFIGHTER_CONTROL",
  "elevatorId": "ELV-001",
  "authentication": {
    "method": "DIGITAL_KEY",
    "keyId": "FD-MASTER-KEY-001"
  },
  "phase": "PHASE_2_MANUAL_CONTROL",
  "permissions": {
    "manualFloorSelection": true,
    "doorControl": true,
    "bypassSafety": ["DOOR_TIMER_ONLY"]
  }
}
```

---

## 5. Cloud Platform Integration

### 5.1 AWS IoT Core

**Thing Configuration:**
```json
{
  "thingName": "ELV-001-BLD-2025",
  "thingType": "WIA-Elevator",
  "attributes": {
    "buildingId": "BLD-2025",
    "elevatorId": "ELV-001",
    "manufacturer": "WIA-Compliant-Mfg",
    "model": "WIA-ELEV-2025",
    "installDate": "2025-01-15"
  }
}
```

**MQTT Topics:**
```
$aws/things/ELV-001-BLD-2025/shadow/update
$aws/things/ELV-001-BLD-2025/shadow/get
wia/elevator/telemetry
wia/elevator/events
```

**Device Shadow:**
```json
{
  "state": {
    "reported": {
      "currentFloor": 5,
      "direction": "UP",
      "doorStatus": "CLOSED",
      "loadWeight": 640,
      "timestamp": "2025-12-26T10:30:00Z"
    },
    "desired": {
      "targetFloor": 10
    }
  }
}
```

### 5.2 Azure IoT Hub

**Device Twin:**
```json
{
  "deviceId": "ELV-001-BLD-2025",
  "properties": {
    "desired": {
      "maintenanceMode": false,
      "telemetryInterval": 60
    },
    "reported": {
      "status": {
        "currentFloor": 5,
        "direction": "UP"
      },
      "telemetry": {
        "loadWeight": 640,
        "temperature": 22.5
      }
    }
  }
}
```

### 5.3 Google Cloud IoT Core

**Device Configuration:**
```json
{
  "id": "ELV-001-BLD-2025",
  "name": "projects/wia-elevators/locations/us-central1/registries/building-2025/devices/ELV-001",
  "credentials": [
    {
      "publicKey": {
        "format": "RSA_X509_PEM",
        "key": "-----BEGIN CERTIFICATE-----\n..."
      }
    }
  ],
  "metadata": {
    "buildingId": "BLD-2025",
    "elevatorId": "ELV-001"
  }
}
```

---

## 6. Energy Management Integration

### 6.1 Energy Dashboard Integration

**Energy API endpoint:**
```
POST /api/v1/building/energy/elevator

{
  "elevatorId": "ELV-001",
  "timestamp": "2025-12-26T10:00:00Z",
  "energyData": {
    "consumed": 42.5,
    "regenerated": 12.8,
    "net": 29.7,
    "efficiency": 30.1,
    "unit": "kWh"
  }
}
```

### 6.2 Demand Response

**Grid signal integration:**
```json
{
  "type": "DEMAND_RESPONSE_EVENT",
  "buildingId": "BLD-2025",
  "timestamp": "2025-12-26T14:00:00Z",
  "event": "PEAK_DEMAND_REDUCTION",
  "duration": 3600,
  "elevatorStrategy": {
    "reduceSpeed": 20,
    "groupDispatching": "AGGRESSIVE",
    "parkElevators": ["ELV-003", "ELV-004"],
    "estimatedReduction": "15kW"
  }
}
```

---

## 7. Predictive Maintenance Platform

### 7.1 ML Model Integration

**Training data export:**
```json
{
  "elevatorId": "ELV-001",
  "exportPeriod": {
    "start": "2025-01-01T00:00:00Z",
    "end": "2025-12-26T23:59:59Z"
  },
  "features": {
    "vibrationData": [...],
    "temperatureData": [...],
    "loadCycles": [...],
    "doorCycles": [...],
    "tripCount": 182394
  },
  "labels": {
    "brakeReplacement": "2025-06-15",
    "ropeReplacement": null,
    "motorBearingFailure": null
  }
}
```

### 7.2 Prediction Response

```json
{
  "elevatorId": "ELV-001",
  "timestamp": "2025-12-26T10:00:00Z",
  "predictions": [
    {
      "component": "BRAKE_LINING",
      "prediction": "REPLACEMENT_NEEDED",
      "confidence": 0.92,
      "estimatedDays": 18,
      "recommendedAction": "SCHEDULE_MAINTENANCE"
    },
    {
      "component": "DOOR_OPERATOR",
      "prediction": "NORMAL",
      "confidence": 0.88,
      "estimatedDays": 120
    }
  ]
}
```

---

## 8. Third-Party Application Ecosystem

### 8.1 App Marketplace

WIA-compliant elevators support third-party apps through standardized APIs:

**App categories:**
- Analytics & reporting
- Predictive maintenance
- Traffic optimization
- Mobile passenger apps
- Facility management
- Energy optimization

### 8.2 App Integration

**OAuth 2.0 app registration:**
```json
{
  "appId": "app_abc123",
  "appName": "ElevatorAnalytics Pro",
  "developer": "Analytics Co.",
  "permissions": [
    "elevator:read",
    "building:read",
    "telemetry:read"
  ],
  "webhookUrl": "https://app.analytics.com/webhooks/elevator",
  "redirectUri": "https://app.analytics.com/oauth/callback"
}
```

---

## 9. Data Privacy & Multi-Tenancy

### 9.1 Data Isolation

```json
{
  "tenantId": "TENANT-001",
  "buildingIds": ["BLD-2025", "BLD-2026"],
  "elevatorIds": ["ELV-001", "ELV-002", "ELV-003"],
  "dataAccess": {
    "telemetry": "FULL",
    "maintenance": "FULL",
    "crossTenantData": "DENIED"
  }
}
```

### 9.2 GDPR Compliance

**Data retention policy:**
```json
{
  "elevatorTelemetry": {
    "retention": "90 days",
    "aggregatedRetention": "7 years",
    "anonymization": "REQUIRED"
  },
  "accessLogs": {
    "retention": "30 days",
    "encryption": "AES-256"
  },
  "maintenanceRecords": {
    "retention": "LIFETIME"
  }
}
```

---

## 10. Compliance Checklist

Phase 4 compliance requires:
1. ✅ BACnet OR Modbus integration
2. ✅ Access control integration
3. ✅ Fire alarm coordination
4. ✅ At least one cloud platform adapter
5. ✅ Energy reporting API
6. ✅ Third-party API support
7. ✅ Data privacy compliance

---

**© 2025 WIA - World Certification Industry Association**  
**MIT License**  
**弘益人間 · Benefit All Humanity**


## Annex E — Implementation Notes for PHASE-4-INTEGRATION

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-4-INTEGRATION.

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
evidence for PHASE-4-INTEGRATION. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-4-integration/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-4-INTEGRATION with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-4-INTEGRATION does not require bespoke
auditor tooling.
