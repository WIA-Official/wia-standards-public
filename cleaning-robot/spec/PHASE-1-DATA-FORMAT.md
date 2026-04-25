# WIA-ROB-011 Phase 1: Data Format Specification

**Version:** 1.0.0  
**Status:** Approved  
**Last Updated:** 2025-12-26

弘益人間 (Hongik Ingan) - Benefit All Humanity

---

## 1. Overview

Phase 1 defines the standardized data formats for cleaning robots, including robot state representation, map data structures, sensor readings, and cleaning logs. All data MUST use JSON-LD format for semantic interoperability.

## 2. Core Data Types

### 2.1 Robot Identity

```json
{
  "@context": "https://wiastandards.com/rob-011/v1",
  "@type": "CleaningRobot",
  "robotId": "ROB-2025-XXXX-YYYY",
  "manufacturer": "string",
  "model": "string",
  "serialNumber": "string",
  "firmwareVersion": "semver",
  "hardwareVersion": "string",
  "manufactureDate": "ISO8601 date",
  "capabilities": ["array", "of", "capability", "strings"]
}
```

### 2.2 Robot State

```json
{
  "@type": "RobotState",
  "timestamp": "ISO8601 datetime",
  "battery": {
    "percentage": 0-100,
    "voltage": "float (V)",
    "current": "float (A)",
    "temperature": "float (°C)",
    "health": 0-100,
    "cycleCount": "integer"
  },
  "pose": {
    "x": "float (meters)",
    "y": "float (meters)",
    "theta": "float (radians)",
    "floor": "integer",
    "confidence": 0-1
  },
  "mode": "idle|cleaning|charging|returning|error",
  "cleaningMode": "auto|spot|edge|zigzag|spiral",
  "suctionPower": 0-100,
  "brushSpeed": 0-100,
  "waterFlow": 0-100,
  "sensors": {
    "lidar": "SensorReading",
    "camera": "SensorReading",
    "cliff": ["boolean", "array"],
    "bumper": "boolean",
    "dirtLevel": 0-100
  }
}
```

### 2.3 Map Data Format

```json
{
  "@type": "OccupancyGrid",
  "resolution": 0.05,
  "width": "integer (cells)",
  "height": "integer (cells)",
  "origin": {
    "x": "float (meters)",
    "y": "float (meters)",
    "theta": "float (radians)"
  },
  "data": "base64 encoded uint8 array",
  "rooms": [
    {
      "id": "UUID",
      "name": "string",
      "polygon": [[x, y], ...],
      "surfaceType": "hardwood|tile|carpet|vinyl|marble",
      "lastCleaned": "ISO8601 datetime",
      "cleanCount": "integer"
    }
  ],
  "noGoZones": [
    {
      "id": "UUID",
      "polygon": [[x, y], ...],
      "type": "no_go|restricted"
    }
  ],
  "dockLocation": {
    "x": "float",
    "y": "float",
    "theta": "float"
  }
}
```

### 2.4 Cleaning Session Log

```json
{
  "@type": "CleaningSession",
  "sessionId": "UUID",
  "startTime": "ISO8601 datetime",
  "endTime": "ISO8601 datetime",
  "duration": "integer (seconds)",
  "areaCleaned": "float (m²)",
  "distanceTraveled": "float (m)",
  "batteryConsumed": "float (Wh)",
  "cleaningMode": "string",
  "rooms": ["room_id", "array"],
  "interruptions": [
    {
      "timestamp": "ISO8601 datetime",
      "reason": "stuck|battery_low|manual_stop|error",
      "duration": "integer (seconds)"
    }
  ],
  "statistics": {
    "avgSpeed": "float (m/s)",
    "avgSuction": "float (%)",
    "dirtCollected": "float (g)",
    "coverage": "float (%)"
  }
}
```

## 3. Sensor Data Formats

### 3.1 LiDAR Scan

```json
{
  "@type": "LidarScan",
  "timestamp": "ISO8601 datetime",
  "angleMin": "float (radians)",
  "angleMax": "float (radians)",
  "angleIncrement": "float (radians)",
  "rangeMin": "float (meters)",
  "rangeMax": "float (meters)",
  "ranges": ["float", "array"],
  "intensities": ["float", "array (optional)"]
}
```

### 3.2 Camera Image

```json
{
  "@type": "CameraImage",
  "timestamp": "ISO8601 datetime",
  "width": "integer",
  "height": "integer",
  "encoding": "rgb8|bgr8|jpeg|png",
  "data": "base64 string",
  "detectedObjects": [
    {
      "class": "string",
      "confidence": 0-1,
      "boundingBox": {"x": int, "y": int, "w": int, "h": int}
    }
  ]
}
```

## 4. Event Data Format

```json
{
  "@type": "RobotEvent",
  "eventId": "UUID",
  "timestamp": "ISO8601 datetime",
  "type": "info|warning|error|critical",
  "category": "navigation|cleaning|battery|sensor|communication",
  "message": "string",
  "details": {
    "arbitrary": "key-value pairs"
  }
}
```

## 5. Command Format

```json
{
  "@type": "RobotCommand",
  "commandId": "UUID",
  "timestamp": "ISO8601 datetime",
  "action": "start|stop|pause|resume|dock|set_mode",
  "parameters": {
    "mode": "string (optional)",
    "rooms": ["array (optional)"],
    "powerLevel": "integer (optional)"
  }
}
```

## 6. Validation Rules

1. All timestamps MUST use ISO 8601 format with timezone
2. All measurements MUST use SI units
3. All arrays MUST have consistent element types
4. All UUIDs MUST be version 4
5. Required fields MUST NOT be null
6. Enum values MUST match specification exactly

## 7. Extensibility

Implementations MAY add custom fields prefixed with "x_" to avoid conflicts with future standard additions.

Example:
```json
{
  "@type": "RobotState",
  "battery": 75,
  "x_customField": "vendor-specific data"
}
```

---

© 2025 WIA · MIT License

## 8. Data Versioning

All data formats include version information for backward compatibility:

```json
{
  "@context": "https://wiastandards.com/rob-011/v1",
  "@version": "1.0.0",
  "schemaVersion": "1.0.0"
}
```

### Migration Guidelines

When updating data formats:
1. Maintain backward compatibility for minor versions
2. Provide migration tools for major versions
3. Document all breaking changes
4. Support multiple versions during transition periods

## 9. Data Examples

### Complete Robot State Example

```json
{
  "@context": "https://wiastandards.com/rob-011/v1",
  "@type": "RobotState",
  "timestamp": "2025-12-26T14:30:00Z",
  "battery": {
    "percentage": 75,
    "voltage": 14.8,
    "current": 2.5,
    "temperature": 35.2,
    "health": 95,
    "cycleCount": 245,
    "charging": false,
    "timeToEmpty": 3600
  },
  "pose": {
    "x": 2.5,
    "y": 3.1,
    "theta": 1.57,
    "floor": 0,
    "confidence": 0.95
  },
  "mode": "cleaning",
  "cleaningMode": "auto",
  "suctionPower": 80,
  "brushSpeed": 100,
  "sensors": {
    "lidar": {
      "timestamp": "2025-12-26T14:30:00Z",
      "ranges": [1.2, 1.5, 2.0]
    },
    "cliff": [false, false, false, false],
    "bumper": false,
    "dirtLevel": 45
  }
}
```

---

## Annex A — Conformance Tier Matrix

WIA conformance for cleaning-robot is evaluated across three tiers:

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

- `wia-standards/standards/cleaning-robot/api/` — TypeScript SDK skeleton
- `wia-standards/standards/cleaning-robot/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/cleaning-robot/simulator/` — interactive browser-based simulator for the PHASE protocol

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
