# WIA Green Infrastructure Integration Standard
## Phase 4 Specification

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
2. [Integration Partners](#integration-partners)
3. [City Planning Integration](#city-planning-integration)
4. [Water Management Integration](#water-management-integration)
5. [Environmental Agencies](#environmental-agencies)
6. [IoT Platform Integration](#iot-platform-integration)
7. [Building Management Systems](#building-management-systems)
8. [Examples](#examples)

---

## Overview

### 1.1 Purpose

The WIA Green Infrastructure Integration Standard defines how green infrastructure monitoring systems integrate with city planning departments, water management systems, environmental agencies, IoT platforms, and building management systems.

### 1.2 Integration Principles

1. **Interoperability**: Standard APIs and data formats
2. **Real-time**: Live data exchange for decision-making
3. **Scalability**: From single buildings to city-wide networks
4. **Security**: Encrypted, authenticated data exchange
5. **Compliance**: Regulatory reporting and certification

---

## Integration Partners

### 2.1 Partner Categories

| Category | Systems | Data Exchange |
|----------|---------|---------------|
| City Planning | GIS, Zoning, Permitting | Infrastructure locations, performance metrics |
| Water Management | Stormwater, Flood Control | Retention data, flow rates, water quality |
| Environmental | Air Quality, Climate Action | Carbon sequestration, temperature, biodiversity |
| IoT Platforms | AWS IoT, Azure IoT, Google Cloud | Sensor data, device management |
| Building Systems | BMS, HVAC, Energy Management | Temperature, moisture, energy savings |

### 2.2 Integration Levels

| Level | Description | Complexity |
|-------|-------------|------------|
| Level 1 | Data Export (CSV, JSON) | Low |
| Level 2 | REST API Integration | Medium |
| Level 3 | Real-time Streaming | High |
| Level 4 | Bidirectional Control | Very High |

---

## City Planning Integration

### 3.1 GIS Integration

#### Data Export Format (GeoJSON)

```json
{
  "type": "FeatureCollection",
  "features": [
    {
      "type": "Feature",
      "geometry": {
        "type": "Point",
        "coordinates": [126.9780, 37.5665]
      },
      "properties": {
        "infrastructureId": "GI-2025-001",
        "type": "green_roof",
        "area": 500,
        "status": "active",
        "performance": {
          "stormwaterRetention": 450,
          "carbonSequestration": 510,
          "coolingEffect": 3.5
        }
      }
    }
  ]
}
```

#### WMS/WFS Integration

```
WMS Endpoint: https://api.wia.live/green-infrastructure/v1/wms
WFS Endpoint: https://api.wia.live/green-infrastructure/v1/wfs
```

**Layers:**
- `green_roofs`
- `permeable_pavement`
- `bioswales`
- `rain_gardens`
- `tree_canopy`

### 3.2 Zoning Compliance

```http
POST /api/v1/integration/zoning/validate
```

**Request:**
```json
{
  "infrastructureId": "GI-2025-001",
  "location": {
    "gps": {"latitude": 37.5665, "longitude": 126.9780}
  },
  "type": "green_roof",
  "area": 500
}
```

**Response:**
```json
{
  "compliant": true,
  "requirements": {
    "minimumGreenSpace": 400,
    "actual": 500,
    "percentage": 125
  },
  "certifications": ["LEED", "Green Building Code"],
  "incentives": {
    "taxCredit": 5000,
    "fastTrackPermit": true
  }
}
```

### 3.3 Urban Heat Island Mapping

```http
GET /api/v1/integration/heat-island/impact
```

**Query Parameters:**
- `district`: District code
- `period`: Time period for analysis
- `resolution`: Grid resolution (meters)

**Response:**
```json
{
  "district": "Gangnam-gu",
  "period": "2024-summer",
  "coverage": {
    "totalArea": 39500000,
    "greenInfrastructure": 250000,
    "percentage": 0.63
  },
  "temperatureReduction": {
    "average": 2.3,
    "peak": 4.5,
    "unit": "celsius"
  },
  "heatmap": "https://api.wia.live/green-infrastructure/v1/heatmap/gangnam-2024-summer.png"
}
```

---

## Water Management Integration

### 4.1 Stormwater Management

#### Real-time Flow Data

```http
POST /api/v1/integration/water/stormwater
```

**Webhook Payload:**
```json
{
  "eventId": "STORM-2025-001",
  "timestamp": "2025-01-15T10:30:00Z",
  "infrastructures": [
    {
      "infrastructureId": "GI-2025-001",
      "type": "green_roof",
      "retention": {
        "capacity": 75,
        "current": 45,
        "percentage": 60
      },
      "overflow": false
    }
  ],
  "totalRetention": 1250,
  "peakFlowReduction": 0.55
}
```

#### Flood Risk Assessment

```http
GET /api/v1/integration/water/flood-risk
```

**Response:**
```json
{
  "district": "Songpa-gu",
  "riskLevel": "moderate",
  "greenInfrastructure": {
    "count": 45,
    "totalRetention": 5400,
    "coverage": "15%"
  },
  "recommendations": [
    "Install additional bioswales in high-risk zones",
    "Expand permeable pavement coverage",
    "Monitor retention capacity during heavy rainfall"
  ]
}
```

### 4.2 Water Quality Monitoring

```json
{
  "infrastructureId": "GI-2025-001",
  "waterQuality": {
    "inlet": {
      "tss": 150,
      "nitrogen": 5.2,
      "phosphorus": 0.8,
      "ph": 7.2
    },
    "outlet": {
      "tss": 30,
      "nitrogen": 2.9,
      "phosphorus": 0.4,
      "ph": 7.5
    },
    "removal": {
      "tss": 0.80,
      "nitrogen": 0.44,
      "phosphorus": 0.50
    }
  },
  "timestamp": "2025-01-15T10:30:00Z"
}
```

---

## Environmental Agencies

### 5.1 Carbon Reporting

```http
POST /api/v1/integration/environmental/carbon-report
```

**Request:**
```json
{
  "reportingPeriod": {
    "start": "2024-01-01T00:00:00Z",
    "end": "2024-12-31T23:59:59Z"
  },
  "district": "Seoul",
  "infrastructures": ["all"]
}
```

**Response:**
```json
{
  "reportId": "CARBON-2024-SEOUL",
  "period": "2024",
  "summary": {
    "totalInfrastructures": 450,
    "totalArea": 225000,
    "carbonSequestration": {
      "total": 286875,
      "unit": "kg_co2_per_year"
    },
    "equivalentTrees": 12995,
    "equivalentCars": 62
  },
  "breakdown": [
    {
      "type": "green_roof",
      "count": 250,
      "sequestration": 127500
    },
    {
      "type": "tree_canopy",
      "count": 150,
      "sequestration": 131250
    }
  ]
}
```

### 5.2 Biodiversity Tracking

```json
{
  "infrastructureId": "GI-2025-001",
  "biodiversity": {
    "speciesCount": 35,
    "categories": {
      "plants": 20,
      "insects": 10,
      "birds": 5
    },
    "nativeSpecies": 28,
    "invasiveSpecies": 0,
    "pollinatorSupport": "high",
    "habitatValue": 0.75
  },
  "monitoring": {
    "method": "camera_trap",
    "frequency": "monthly",
    "lastSurvey": "2025-01-01T00:00:00Z"
  }
}
```

### 5.3 Sustainability Certification

```http
POST /api/v1/integration/environmental/certification
```

**Request:**
```json
{
  "infrastructureId": "GI-2025-001",
  "certificationType": "LEED",
  "performanceData": {
    "stormwaterRetention": 450,
    "carbonSequestration": 510,
    "energySavings": 1250
  }
}
```

**Response:**
```json
{
  "certificationId": "LEED-GOLD-2025-001",
  "status": "approved",
  "level": "Gold",
  "score": 68,
  "credits": {
    "stormwater": 4,
    "energy": 3,
    "materials": 2,
    "indoor_environment": 3
  },
  "validUntil": "2030-01-15T00:00:00Z"
}
```

---

## IoT Platform Integration

### 6.1 AWS IoT Core

```javascript
// AWS IoT Device SDK
const awsIot = require('aws-iot-device-sdk');

const device = awsIot.device({
  keyPath: 'private.key',
  certPath: 'certificate.pem',
  caPath: 'root-CA.crt',
  clientId: 'SENSOR-GI-001',
  host: 'your-iot-endpoint.iot.us-east-1.amazonaws.com'
});

device.on('connect', () => {
  device.publish('green-infrastructure/GI-2025-001/sensor', JSON.stringify({
    sensorId: 'SENSOR-GI-001',
    moisture: 65,
    temperature: 22.5,
    timestamp: new Date().toISOString()
  }));
});
```

### 6.2 Azure IoT Hub

```csharp
// Azure IoT Device SDK
using Microsoft.Azure.Devices.Client;

DeviceClient deviceClient = DeviceClient.CreateFromConnectionString(
    connectionString,
    TransportType.Mqtt
);

var telemetry = new {
    infrastructureId = "GI-2025-001",
    sensorId = "SENSOR-GI-001",
    moisture = 65,
    temperature = 22.5,
    timestamp = DateTime.UtcNow
};

Message message = new Message(Encoding.UTF8.GetBytes(JsonConvert.SerializeObject(telemetry)));
await deviceClient.SendEventAsync(message);
```

### 6.3 Google Cloud IoT

```python
# Google Cloud IoT Core
import jwt
import datetime
from google.cloud import iot_v1

client = iot_v1.DeviceManagerClient()

device_path = client.device_path(
    project_id, cloud_region, registry_id, device_id
)

payload = {
    'infrastructureId': 'GI-2025-001',
    'sensorId': 'SENSOR-GI-001',
    'moisture': 65,
    'temperature': 22.5,
    'timestamp': datetime.datetime.utcnow().isoformat()
}

client.send_command_to_device(
    request={"name": device_path, "binary_data": json.dumps(payload).encode()}
)
```

---

## Building Management Systems

### 7.1 BMS Integration

```http
POST /api/v1/integration/bms/connect
```

**Request:**
```json
{
  "buildingId": "BLDG-2025-001",
  "bmsType": "Honeywell",
  "endpoints": {
    "temperature": "bacnet://192.168.1.100/device/1/object/analog-input/1",
    "moisture": "bacnet://192.168.1.100/device/1/object/analog-input/2"
  },
  "infrastructureId": "GI-2025-001"
}
```

### 7.2 HVAC Optimization

```json
{
  "buildingId": "BLDG-2025-001",
  "greenRoof": {
    "infrastructureId": "GI-2025-001",
    "coolingEffect": 3.5,
    "insulationValue": "R-25"
  },
  "hvacImpact": {
    "coolingLoadReduction": 0.20,
    "energySavings": {
      "value": 1250,
      "unit": "kwh_per_year"
    },
    "costSavings": {
      "value": 187.50,
      "unit": "USD_per_year"
    }
  },
  "recommendations": [
    "Reduce HVAC runtime during peak hours",
    "Adjust temperature setpoints based on green roof performance"
  ]
}
```

---

## Examples

### 8.1 Complete City Integration

```javascript
// Integrate with city planning, water management, and environmental systems
const cityIntegration = {
  // GIS Export
  exportToGIS: async () => {
    const geojson = await fetch('/api/v1/integration/gis/export');
    return geojson.json();
  },

  // Stormwater Data
  sendStormwaterData: async (data) => {
    await fetch('/api/v1/integration/water/stormwater', {
      method: 'POST',
      body: JSON.stringify(data)
    });
  },

  // Carbon Report
  generateCarbonReport: async (period) => {
    const report = await fetch(`/api/v1/integration/environmental/carbon-report?period=${period}`);
    return report.json();
  }
};
```

### 8.2 Multi-Platform IoT Integration

```python
# Connect to multiple IoT platforms simultaneously
class MultiPlatformIoT:
    def __init__(self, infrastructure_id):
        self.infrastructure_id = infrastructure_id
        self.aws_client = connect_aws()
        self.azure_client = connect_azure()
        self.gcp_client = connect_gcp()

    def publish_sensor_data(self, data):
        # Publish to all platforms
        self.aws_client.publish(f'gi/{self.infrastructure_id}/sensor', data)
        self.azure_client.send_event(data)
        self.gcp_client.send_command(data)
```

---

<div align="center">

**WIA Green Infrastructure Integration v1.0.0**

**弘益人間 (홍익인간)** - Benefit All Humanity

---

**© 2025 WIA**

**MIT License**

</div>


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

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-4-INTEGRATION.
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
for PHASE-4-INTEGRATION. The profile mechanism is non-normative and exists so that
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
  advertises `WIA-P4-INTEGRATION-Edge-only/2` is asserting conformance with
  the second major version of the named profile, not the second deployment
  of an unversioned profile.

The profile mechanism is intentionally lightweight; it is meant to make
real deployment shapes visible without forcing every deployment to
satisfy every normative requirement.
