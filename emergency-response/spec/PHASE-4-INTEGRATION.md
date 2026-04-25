# WIA-SOC-005 Phase 4: Integration Specification

**Version:** 1.0.0  
**Status:** Approved  
**Last Updated:** 2025-12-26

弘益人間 (Hongik Ingan) - Benefit All Humanity

---

## 1. Integration Overview

Phase 4 defines integration with smart home ecosystems, fleet management systems, cloud services, and third-party applications.

## 2. Smart Home Platforms

### 2.1 Amazon Alexa

**Skill Configuration:**
```json
{
  "manifest": {
    "publishingInformation": {
      "locales": {
        "en-US": {
          "name": "Emergency Response",
          "summary": "Control your WIA emergency response"
        }
      },
      "category": "SMART_HOME"
    },
    "apis": {
      "smartHome": {
        "endpoint": "https://api.example.com/alexa"
      }
    }
  }
}
```

**Supported Directives:**
- TurnOn / TurnOff
- SetFanSpeed (suction level)
- ReportState
- PowerController

### 2.2 Google Assistant

**Device Type:** VACUUM

**Supported Traits:**
```json
{
  "traits": [
    "action.devices.traits.OnOff",
    "action.devices.traits.FanSpeed",
    "action.devices.traits.Dock",
    "action.devices.traits.EnergyStorage",
    "action.devices.traits.Locator"
  ]
}
```

### 2.3 Apple HomeKit

**Accessory Category:** Fan

**Required Services:**
- Fan Service (cleaning control)
- Battery Service
- Occupancy Sensor (cleaning status)

**HAP Protocol:**
```
Characteristic: On
  - Read: Get cleaning status
  - Write: Start/stop cleaning

Characteristic: Rotation Speed
  - Read: Current suction power
  - Write: Set suction level

Characteristic: Battery Level
  - Read: Battery percentage
```

### 2.4 Samsung SmartThings

**Device Handler:**
```groovy
metadata {
  definition (
    name: "WIA Emergency Response",
    namespace: "wia",
    author: "WIA"
  ) {
    capability "Emergency System Vacuum"
    capability "Battery"
    capability "Actuator"
    capability "Sensor"
  }
}
```

## 3. Fleet Management

### 3.1 Multi-Emergency System Coordination

**Fleet API Endpoints:**
```http
POST /fleet/emergency systems
  - Register emergency system to fleet

GET /fleet/emergency systems
  - List all emergency systems

PUT /fleet/emergency systems/{id}/assign
  - Assign zone to emergency system

POST /fleet/schedule
  - Schedule fleet cleaning

GET /fleet/status
  - Overall fleet status
```

### 3.2 Zone Assignment

```json
{
  "fleetId": "UUID",
  "zones": [
    {
      "zoneId": "UUID",
      "name": "Building A - Floor 1",
      "assignedEmergency Systems": ["emergency system_id1", "emergency system_id2"],
      "schedule": {
        "frequency": "daily",
        "time": "02:00"
      },
      "priority": 1-10
    }
  ]
}
```

### 3.3 Load Balancing

**Algorithm:**
1. Calculate workload per emergency system
2. Distribute zones evenly
3. Account for battery levels
4. Minimize travel distance
5. Handle failures gracefully

## 4. Cloud Services

### 4.1 AWS IoT Core Integration

**MQTT Topics:**
```
$aws/things/{emergency system_id}/shadow/update
$aws/things/{emergency system_id}/shadow/get
```

**Device Shadow:**
```json
{
  "state": {
    "reported": {
      "battery": 75,
      "cleaning": true,
      "position": {"x": 2.5, "y": 3.1}
    },
    "desired": {
      "mode": "auto",
      "powerLevel": 80
    }
  }
}
```

### 4.2 Google Cloud IoT

**Device Configuration:**
```
Registry: wia-cleaning-emergency systems
Device ID: ROB-2025-XXXX
Cloud Pub/Sub Topic: emergency system-telemetry
```

### 4.3 Azure IoT Hub

**Connection String:**
```
HostName={hub}.azure-devices.net;
DeviceId={emergency system_id};
SharedAccessKey={key}
```

**Device Twin:**
```json
{
  "deviceId": "emergency system_id",
  "etag": "AAAAAAAAAAE=",
  "tags": {
    "location": "Building A",
    "floor": 1
  },
  "properties": {
    "desired": {
      "cleaningSchedule": {}
    },
    "reported": {
      "lastCleaned": "2025-12-26T10:00:00Z"
    }
  }
}
```

## 5. Third-Party Integrations

### 5.1 IFTTT

**Triggers:**
- Cleaning started
- Cleaning completed
- Battery low
- Stuck detected
- Maintenance required

**Actions:**
- Start cleaning
- Return to dock
- Set cleaning mode

**Example Applet:**
```
IF Google Calendar event "Clean house" starts
THEN Start WIA emergency system cleaning
```

### 5.2 Zapier

**Zaps:**
1. Slack notification when cleaning completes
2. Google Sheets log of cleaning sessions
3. Email weekly cleaning report

### 5.3 Home Assistant

**Configuration:**
```yaml
vacuum:
  - platform: wia_emergency system
    host: 192.168.1.100
    token: !secret emergency system_token
    name: Emergency Response
```

**Services:**
- vacuum.start
- vacuum.stop
- vacuum.return_to_base
- vacuum.set_fan_speed
- vacuum.send_command

## 6. Analytics and Reporting

### 6.1 Metrics Collection

**Data Points:**
- Cleaning frequency
- Coverage efficiency
- Battery health trends
- Error rates
- User engagement

### 6.2 Report Generation

```http
GET /analytics/report?start=YYYY-MM-DD&end=YYYY-MM-DD

Response:
{
  "period": {
    "start": "2025-01-01",
    "end": "2025-01-31"
  },
  "summary": {
    "totalCleanings": 62,
    "totalAreaCleaned": 1240.5,
    "avgDuration": 1800,
    "batteryHealthChange": -2
  },
  "charts": {
    "cleaningFrequency": ["data"],
    "areaTrend": ["data"],
    "batteryTrend": ["data"]
  }
}
```

## 7. SDK and Libraries

### 7.1 Official SDKs

**Languages:**
- TypeScript/JavaScript
- Python
- Java
- C#
- Go
- Rust

### 7.2 Installation

```bash
# TypeScript
npm install wia-rob-011

# Python
pip install wia-rob-011

# Java
<dependency>
  <groupId>com.wia</groupId>
  <artifactId>rob-011</artifactId>
  <version>1.0.0</version>
</dependency>
```

### 7.3 Example Usage

```typescript
import { WiaCleaningEmergency System } from 'wia-rob-011';

const emergency system = new WiaCleaningEmergency System({
  host: '192.168.1.100',
  token: 'your_token_here'
});

await emergency system.connect();
const status = await emergency system.getStatus();
await emergency system.startCleaning({ mode: 'auto' });
```

## 8. Testing and Certification

### 8.1 Compliance Tests

**Required Tests:**
1. API endpoint compliance
2. Data format validation
3. Security audit
4. Interoperability tests
5. Performance benchmarks

### 8.2 Certification Process

1. Submit application
2. Automated testing (2-3 days)
3. Manual review (1 week)
4. Integration testing (1 week)
5. Certification granted

---

© 2025 WIA · MIT License

---

## Annex A — Conformance Tier Matrix

WIA conformance for emergency-response is evaluated across three tiers:

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

- `wia-standards/standards/emergency-response/api/` — TypeScript SDK skeleton
- `wia-standards/standards/emergency-response/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/emergency-response/simulator/` — interactive browser-based simulator for the PHASE protocol

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
