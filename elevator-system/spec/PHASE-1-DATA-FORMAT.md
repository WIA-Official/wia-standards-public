# WIA Elevator System Standard
## Phase 1: Data Format Specification v1.0

**Status:** APPROVED  
**Date:** 2025-12-26  
**弘益人間** · Benefit All Humanity

---

## 1. Introduction

Phase 1 of the WIA Elevator System Standard defines standardized JSON schemas for all elevator telemetry, sensor data, status reporting, events, and metadata. This ensures interoperability across different manufacturers and enables consistent data exchange throughout the elevator ecosystem.

### 1.1 Scope

This specification covers:
- Elevator status and telemetry data formats
- Sensor data structures and types
- Event and alarm reporting schemas
- Maintenance record formats
- Traffic analytics data models
- Energy consumption metrics
- Accessibility feature reporting

### 1.2 Normative References

- **JSON Schema Draft 2020-12:** https://json-schema.org/specification.html
- **ISO 8601:** Date and time format
- **EN 81-20/50:** Safety rules for construction and installation of elevators
- **ASME A17.1:** Safety Code for Elevators and Escalators

---

## 2. Core Data Types

### 2.1 Elevator Status Schema

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "$id": "https://wiastandards.com/schemas/elevator/status/v1.0",
  "title": "Elevator Status",
  "type": "object",
  "required": ["elevatorId", "buildingId", "timestamp", "status"],
  "properties": {
    "elevatorId": {
      "type": "string",
      "pattern": "^ELV-[0-9]{3,6}$",
      "description": "Unique elevator identifier"
    },
    "buildingId": {
      "type": "string",
      "description": "Building identifier"
    },
    "timestamp": {
      "type": "string",
      "format": "date-time",
      "description": "ISO 8601 timestamp"
    },
    "status": {
      "type": "object",
      "required": ["currentFloor", "direction", "doorStatus"],
      "properties": {
        "currentFloor": {
          "type": "integer",
          "minimum": -10,
          "maximum": 200,
          "description": "Current floor number (negative for basement)"
        },
        "direction": {
          "type": "string",
          "enum": ["UP", "DOWN", "IDLE"],
          "description": "Current direction of travel"
        },
        "doorStatus": {
          "type": "string",
          "enum": ["OPEN", "CLOSED", "OPENING", "CLOSING"],
          "description": "Door state"
        },
        "occupancy": {
          "type": "integer",
          "minimum": 0,
          "description": "Number of passengers (estimated)"
        },
        "maxCapacity": {
          "type": "integer",
          "minimum": 1,
          "description": "Maximum passenger capacity"
        },
        "speed": {
          "type": "number",
          "minimum": 0,
          "maximum": 20,
          "description": "Current speed in m/s"
        },
        "position": {
          "type": "number",
          "description": "Absolute position in shaft (meters)"
        }
      }
    }
  }
}
```

### 2.2 Sensor Data Schema

All sensor readings follow the standardized telemetry format:

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "$id": "https://wiastandards.com/schemas/elevator/sensors/v1.0",
  "title": "Elevator Sensor Telemetry",
  "type": "object",
  "required": ["elevatorId", "timestamp", "sensors"],
  "properties": {
    "elevatorId": {
      "type": "string",
      "pattern": "^ELV-[0-9]{3,6}$"
    },
    "timestamp": {
      "type": "string",
      "format": "date-time"
    },
    "sensors": {
      "type": "object",
      "properties": {
        "loadWeight": {
          "type": "number",
          "minimum": 0,
          "maximum": 5000,
          "description": "Cabin load in kg"
        },
        "temperature": {
          "type": "number",
          "minimum": -20,
          "maximum": 80,
          "description": "Machine room temperature in °C"
        },
        "vibration": {
          "type": "number",
          "minimum": 0,
          "description": "RMS vibration in m/s²"
        },
        "doorSensor": {
          "type": "string",
          "enum": ["NORMAL", "OBSTRUCTION", "ERROR"],
          "description": "Door safety sensor status"
        },
        "emergencyButton": {
          "type": "boolean",
          "description": "Emergency stop button pressed"
        },
        "overloadSensor": {
          "type": "boolean",
          "description": "Overload condition detected"
        },
        "ropeWear": {
          "type": "number",
          "minimum": 0,
          "maximum": 100,
          "description": "Rope wear percentage (0=new, 100=replace)"
        },
        "brakeWear": {
          "type": "number",
          "minimum": 0,
          "maximum": 100,
          "description": "Brake lining wear percentage"
        }
      }
    }
  }
}
```

---

## 3. Event and Alarm Schemas

### 3.1 Event Data Model

```json
{
  "eventId": "EVT-2025-12-26-00123",
  "elevatorId": "ELV-001",
  "buildingId": "BLD-2025",
  "timestamp": "2025-12-26T10:30:00Z",
  "eventType": "DOOR_CYCLE",
  "severity": "INFO",
  "floor": 5,
  "metadata": {
    "doorCycleTime": 4.2,
    "reopenCount": 0
  }
}
```

**Event Types:**
- `DOOR_CYCLE` - Door open/close event
- `FLOOR_CALL` - Floor button pressed
- `CAR_CALL` - Car button pressed
- `TRIP_COMPLETE` - Journey completed
- `MAINTENANCE_START` - Maintenance mode activated
- `FAULT_DETECTED` - System fault detected
- `EMERGENCY_STOP` - Emergency stop activated

**Severity Levels:**
- `INFO` - Informational event
- `WARNING` - Attention required
- `ERROR` - Operational error
- `CRITICAL` - Safety-critical event

### 3.2 Alarm Schema

```json
{
  "alarmId": "ALM-2025-12-26-00045",
  "elevatorId": "ELV-001",
  "buildingId": "BLD-2025",
  "timestamp": "2025-12-26T10:31:15Z",
  "alarmType": "OVERLOAD",
  "severity": "WARNING",
  "description": "Elevator overloaded by 150kg",
  "acknowledged": false,
  "acknowledgedBy": null,
  "acknowledgedAt": null,
  "resolved": false,
  "resolvedAt": null
}
```

---

## 4. Maintenance Record Schema

```json
{
  "$id": "https://wiastandards.com/schemas/elevator/maintenance/v1.0",
  "title": "Maintenance Record",
  "type": "object",
  "required": ["maintenanceId", "elevatorId", "serviceDate", "technicianId"],
  "properties": {
    "maintenanceId": {
      "type": "string",
      "description": "Unique maintenance record identifier"
    },
    "elevatorId": {
      "type": "string",
      "pattern": "^ELV-[0-9]{3,6}$"
    },
    "serviceDate": {
      "type": "string",
      "format": "date"
    },
    "technicianId": {
      "type": "string",
      "description": "Certified technician identifier"
    },
    "serviceType": {
      "type": "string",
      "enum": ["ROUTINE", "PREVENTIVE", "CORRECTIVE", "EMERGENCY", "INSPECTION"]
    },
    "workPerformed": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "task": {
            "type": "string",
            "description": "Task description"
          },
          "partsReplaced": {
            "type": "array",
            "items": {
              "type": "object",
              "properties": {
                "partNumber": {"type": "string"},
                "partName": {"type": "string"},
                "quantity": {"type": "integer"},
                "serialNumber": {"type": "string"}
              }
            }
          }
        }
      }
    },
    "nextServiceDate": {
      "type": "string",
      "format": "date",
      "description": "Recommended next service date"
    },
    "tripsSinceService": {
      "type": "integer",
      "description": "Number of trips since last service"
    },
    "operatingHours": {
      "type": "number",
      "description": "Total operating hours"
    },
    "notes": {
      "type": "string",
      "description": "Technician notes"
    }
  }
}
```

---

## 5. Traffic Analytics Schema

```json
{
  "$id": "https://wiastandards.com/schemas/elevator/traffic/v1.0",
  "title": "Traffic Analytics",
  "type": "object",
  "properties": {
    "elevatorId": {"type": "string"},
    "buildingId": {"type": "string"},
    "period": {
      "type": "object",
      "properties": {
        "start": {"type": "string", "format": "date-time"},
        "end": {"type": "string", "format": "date-time"}
      }
    },
    "metrics": {
      "type": "object",
      "properties": {
        "totalTrips": {"type": "integer"},
        "totalPassengers": {"type": "integer"},
        "averageWaitTime": {"type": "number", "description": "seconds"},
        "averageTravelTime": {"type": "number", "description": "seconds"},
        "peakHourTrips": {"type": "integer"},
        "energyConsumed": {"type": "number", "description": "kWh"},
        "energyRegenerated": {"type": "number", "description": "kWh"},
        "floorDistribution": {
          "type": "object",
          "additionalProperties": {"type": "integer"}
        }
      }
    }
  }
}
```

---

## 6. Energy Consumption Schema

```json
{
  "elevatorId": "ELV-001",
  "buildingId": "BLD-2025",
  "timestamp": "2025-12-26T10:00:00Z",
  "energyMetrics": {
    "consumed": {
      "value": 42.5,
      "unit": "kWh",
      "period": "1h"
    },
    "regenerated": {
      "value": 12.8,
      "unit": "kWh",
      "period": "1h"
    },
    "netConsumption": {
      "value": 29.7,
      "unit": "kWh"
    },
    "efficiency": {
      "value": 30.1,
      "unit": "percent",
      "description": "Regeneration efficiency"
    },
    "peakPower": {
      "value": 25.5,
      "unit": "kW",
      "timestamp": "2025-12-26T10:15:23Z"
    }
  }
}
```

---

## 7. Accessibility Features Schema

```json
{
  "elevatorId": "ELV-001",
  "accessibilityFeatures": {
    "audioAnnouncements": {
      "enabled": true,
      "language": "en-US",
      "volume": 75
    },
    "brailleButtons": {
      "present": true,
      "layout": "ADA-compliant"
    },
    "visualIndicators": {
      "enabled": true,
      "type": "LED-matrix",
      "fontSize": 24
    },
    "doorTimings": {
      "openDuration": 5.0,
      "closeDuration": 3.0,
      "reopenSensitivity": "HIGH"
    },
    "wheelchairAccommodation": {
      "depth": 1400,
      "width": 1100,
      "unit": "mm"
    },
    "emergencyComm": {
      "type": "TWO-WAY-VOICE",
      "videoEnabled": true
    }
  }
}
```

---

## 8. Validation and Compliance

### 8.1 Schema Validation

All data MUST be validated against the appropriate JSON Schema before transmission or storage. Implementations MUST use a JSON Schema validator compliant with Draft 2020-12.

### 8.2 Required Fields

Fields marked as `required` in schemas MUST always be present. Omission of required fields results in INVALID data.

### 8.3 Data Types

All data types MUST conform to JSON primitive types:
- `string` - UTF-8 encoded text
- `number` - IEEE 754 floating point
- `integer` - Whole numbers
- `boolean` - true/false
- `null` - Absence of value
- `object` - Key-value pairs
- `array` - Ordered lists

### 8.4 Timestamps

All timestamps MUST use ISO 8601 format with UTC timezone: `YYYY-MM-DDTHH:MM:SS.sssZ`

### 8.5 Units of Measurement

- **Distance:** meters (m)
- **Speed:** meters per second (m/s)
- **Weight:** kilograms (kg)
- **Temperature:** Celsius (°C)
- **Energy:** kilowatt-hours (kWh)
- **Power:** kilowatts (kW)
- **Time:** seconds (s)

---

## 9. Versioning

This specification uses Semantic Versioning (semver.org):
- **MAJOR** version for incompatible changes
- **MINOR** version for backward-compatible additions
- **PATCH** version for backward-compatible fixes

Current version: **v1.0.0**

---

## 10. Compliance

Implementations claiming WIA Phase 1 compliance MUST:
1. Support all required data schemas
2. Validate all data against JSON Schemas
3. Use ISO 8601 timestamps
4. Follow SI units of measurement
5. Pass WIA Phase 1 validation test suite

---

**© 2025 WIA - World Certification Industry Association**  
**MIT License**  
**弘益人間 · Benefit All Humanity**


## Annex E — Implementation Notes for PHASE-1-DATA-FORMAT

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-1-DATA-FORMAT.

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
evidence for PHASE-1-DATA-FORMAT. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-1-data-format/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-1-DATA-FORMAT with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-1-DATA-FORMAT does not require bespoke
auditor tooling.
