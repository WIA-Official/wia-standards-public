# WIA-IND-029 PHASE 3 — Protocol Specification

**Standard:** WIA-IND-029
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 3 of 4)

---

```

#### 5.1.3 Metal Properties

**Titanium Ti64:**
```json
{
  "material": "Ti64-titanium-alloy",
  "technology": "DMLS",
  "properties": {
    "density": 4.43,
    "tensileStrength": 1100,
    "yieldStrength": 1000,
    "elongation": 10,
    "hardness": 350,
    "meltingPoint": 1660,
    "thermalConductivity": 6.7
  },
  "processParameters": {
    "laserPower": 285,
    "scanSpeed": 1250,
    "layerThickness": 0.03,
    "hatchSpacing": 0.1,
    "atmosphere": "argon",
    "oxygenLevel": "<100ppm"
  },
  "postProcessing": {
    "heatTreatment": {
      "temperature": 650,
      "duration": 120,
      "atmosphere": "vacuum"
    },
    "surfaceFinishing": ["bead-blasting", "machining"],
    "supportRemoval": "wire-EDM"
  }
}
```

### 5.2 Material Management System

#### 5.2.1 Material Inventory

```json
{
  "inventory": [
    {
      "spoolId": "SPOOL-001",
      "material": "PETG",
      "brand": "Polymaker",
      "color": "red",
      "weight": {
        "total": 1000,
        "remaining": 650,
        "unit": "grams"
      },
      "properties": {
        "diameter": 1.75,
        "tolerance": 0.03
      },
      "location": "RACK-A-3",
      "status": "in-use",
      "printerId": "PRINTER-002",
      "expiryDate": "2026-12-31",
      "lastUsed": "2025-12-27T10:30:00Z"
    }
  ]
}
```

#### 5.2.2 Material Tracking

**RFID/NFC Tags:**
- Automatic spool detection
- Weight measurement
- Usage tracking
- Low stock alerts

```json
{
  "materialTracking": {
    "spoolId": "SPOOL-001",
    "rfidTag": "E200341234567890",
    "usageHistory": [
      {
        "jobId": "JOB-123",
        "startWeight": 750,
        "endWeight": 650,
        "used": 100,
        "timestamp": "2025-12-27T10:30:00Z"
      }
    ],
    "alerts": [
      {
        "type": "low-stock",
        "threshold": 100,
        "triggered": false
      }
    ]
  }
}
```

---

## 6. Print Job Management

### 6.1 Job Lifecycle

#### 6.1.1 Job States

```
Job State Machine:
┌─────────┐     ┌─────────┐     ┌─────────┐     ┌─────────┐
│ Created │────▶│  Queued │────▶│ Running │────▶│Completed│
└─────────┘     └─────────┘     └─────────┘     └─────────┘
                     │               │                │
                     │               ▼                │
                     │          ┌────────┐            │
                     │          │ Paused │            │
                     │          └────────┘            │
                     │               │                │
                     ▼               ▼                ▼
                ┌─────────┐     ┌─────────┐     ┌────────┐
                │Cancelled│     │ Failed  │     │Archived│
                └─────────┘     └─────────┘     └────────┘
```

#### 6.1.2 Job Definition

```json
{
  "job": {
    "jobId": "JOB-12345",
    "name": "Bracket v3 - Red PETG",
    "status": "running",
    "priority": "normal",
    "modelId": "MODEL-567",
    "slicedFileId": "GCODE-789",
    "printerId": "PRINTER-001",
    "materialSpool": "SPOOL-001",
    "copies": 5,
    "currentCopy": 2,
    "estimatedTime": 14400,
    "elapsedTime": 7200,
    "estimatedMaterial": 85,
    "usedMaterial": 42.5,
    "progress": 50,
    "currentLayer": 250,
    "totalLayers": 500,
    "createdAt": "2025-12-27T08:00:00Z",
    "startedAt": "2025-12-27T09:00:00Z",
    "estimatedCompletion": "2025-12-27T13:00:00Z"
  }
}
```

### 6.2 Queue Management

#### 6.2.1 Priority Queue

```json
{
  "queue": {
    "printerId": "PRINTER-001",
    "jobs": [
      {
        "jobId": "JOB-100",
        "priority": "urgent",
        "queuePosition": 1,
        "estimatedStart": "2025-12-27T14:00:00Z"
      },
      {
        "jobId": "JOB-101",
        "priority": "high",
        "queuePosition": 2,
        "estimatedStart": "2025-12-27T18:00:00Z"
      },
      {
        "jobId": "JOB-102",
        "priority": "normal",
        "queuePosition": 3,
        "estimatedStart": "2025-12-27T22:00:00Z"
      }
    ]
  }
}
```

#### 6.2.2 Load Balancing

Distribute jobs across multiple printers:

```json
{
  "loadBalancing": {
    "algorithm": "weighted-round-robin",
    "factors": {
      "printerCapability": 0.3,
      "queueLength": 0.3,
      "materialAvailability": 0.2,
      "estimatedCompletion": 0.2
    },
    "constraints": {
      "technologyMatch": true,
      "materialMatch": true,
      "buildVolumeCheck": true,
      "maintenanceWindows": true
    }
  }
}
```

### 6.3 Real-Time Monitoring

#### 6.3.1 Telemetry Data

```json
{
  "telemetry": {
    "jobId": "JOB-12345",
    "printerId": "PRINTER-001",
    "timestamp": "2025-12-27T12:30:00Z",
    "temperatures": {
      "hotend": { "current": 238, "target": 240 },
      "bed": { "current": 79, "target": 80 },
      "chamber": { "current": 42, "target": 45 }
    },
    "position": {
      "x": 125.5,
      "y": 87.3,
      "z": 25.6,
      "e": 1250.8
    },
    "speeds": {
      "print": 60,
      "fan": 100
    },
    "progress": {
      "percentage": 51.2,
      "layer": 256,
      "totalLayers": 500
    },
    "estimates": {
      "timeRemaining": 7020,
      "materialRemaining": 42.3
    }
  }
}
```

#### 6.3.2 Camera Monitoring

```json
{
  "camera": {
    "printerId": "PRINTER-001",
    "streamUrl": "rtsp://printer-001.local:554/stream",
    "snapshotUrl": "http://printer-001.local:8080/snapshot.jpg",
    "resolution": "1920x1080",
    "fps": 15,
    "aiMonitoring": {
      "enabled": true,
      "features": [
        "spaghetti-detection",
        "first-layer-check",
        "warping-detection",
        "filament-runout"
      ]
    }
  }
}
```

---

## 7. Quality Assurance and Inspection

### 7.1 In-Process Monitoring

#### 7.1.1 First Layer Detection

Critical for print success:

```json
{
  "firstLayerCheck": {
    "enabled": true,
    "method": "ai-vision",
    "criteria": {
      "bedAdhesion": 95,
      "uniformity": 90,
      "gapDetection": true,
      "warpingDetection": true
    },
    "action": {
      "onFailure": "pause-and-alert",
      "retryAttempts": 1
    }
  }
}
```

#### 7.1.2 Spaghetti Detection

AI monitoring for print failures:

```json
{
  "spaghettiDetection": {
    "enabled": true,
    "checkInterval": 30,
    "confidence": 0.85,
    "action": "pause-and-alert",
    "notificationChannels": ["email", "sms", "app"]
  }
}
```

### 7.2 Post-Print Inspection

#### 7.2.1 Dimensional Accuracy

```json
{
  "dimensionalInspection": {
    "method": "3d-scanning",
    "scanner": "structured-light",
    "resolution": 0.05,
    "measurements": [
      {
        "feature": "overall-length",
        "nominal": 50.0,
        "tolerance": 0.2,
        "measured": 49.95,
        "deviation": -0.05,
        "status": "pass"
      },
      {
        "feature": "hole-diameter",
        "nominal": 8.0,
        "tolerance": 0.1,
        "measured": 7.92,
        "deviation": -0.08,
        "status": "pass"
      },
      {
        "feature": "wall-thickness",
        "nominal": 2.0,
        "tolerance": 0.15,
        "measured": 2.12,
        "deviation": 0.12,
        "status": "pass"
      }
    ],
    "overallResult": "pass",
    "cpk": 1.45
  }
}
```

#### 7.2.2 Visual Inspection (AI)

```json
{
  "visualInspection": {
    "method": "ai-vision",
    "images": [
      "top-view.jpg",
      "bottom-view.jpg",
      "side-view-1.jpg",
      "side-view-2.jpg"
    ],
    "defectDetection": {
      "warping": { "detected": false, "severity": 0 },
      "layerShift": { "detected": false },
      "stringing": { "detected": true, "severity": 2, "acceptable": true },
      "gaps": { "detected": false },
      "surfaceFinish": { "rating": 8.5, "target": 7.0, "status": "pass" }
    },
    "overallQuality": 9.2,
    "passed": true
  }
}
```

#### 7.2.3 Mechanical Testing

```json
{


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
