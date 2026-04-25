# WIA-FUSION Phase 2: API Interface Specification

**Version:** 1.0.0
**Status:** Official
**Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

This specification defines the RESTful API interfaces for nuclear fusion energy systems, enabling programmatic access to plasma data, control systems, and analytics.

### 1.1 Base URL

```
Production: https://api.fusion.wia.live/v1
Staging: https://api-staging.fusion.wia.live/v1
```

### 1.2 Authentication

```yaml
type: Bearer Token (JWT)
header: Authorization: Bearer <token>
expiry: 24 hours
refresh: /auth/refresh
```

---

## 2. API Endpoints

### 2.1 Plasma State API

#### POST /plasma/state
Record plasma state data.

```yaml
Request:
  Content-Type: application/json
  Body:
    plasma_state:
      shot_id: string (required)
      timestamp: ISO8601 (required)
      reactor: enum[ITER|KSTAR|JET|SPARC|custom]
      core_parameters:
        temperature_keV: { ion: number, electron: number }
        density_m3: { value: number, unit: "1e20/m3" }
        confinement_time_s: number
      performance:
        q_factor: number
        fusion_power_mw: number

Response: 201 Created
  {
    "success": true,
    "shot_id": "KSTAR-20240415-001",
    "recorded_at": "2024-04-15T10:30:00Z"
  }
```

#### GET /plasma/state/{shot_id}
Retrieve plasma state for a specific shot.

```yaml
Parameters:
  shot_id: string (path, required)
  fields: string (query, optional) - Comma-separated field list

Response: 200 OK
  {
    "plasma_state": { ... }
  }
```

#### GET /plasma/state/latest
Get the most recent plasma state.

```yaml
Parameters:
  reactor: string (query, optional)

Response: 200 OK
  {
    "plasma_state": { ... },
    "age_ms": 150
  }
```

---

### 2.2 Stability Analysis API

#### GET /plasma/stability/{shot_id}
Analyze plasma stability.

```yaml
Parameters:
  shot_id: string (path, required)
  time_window_s: number (query, optional, default: 1.0)

Response: 200 OK
  {
    "stability_analysis": {
      "disruption_risk": 0.15,
      "confidence": 0.92,
      "mhd_modes": ["m=2,n=1", "m=3,n=2"],
      "elm_type": "Type-I",
      "elm_frequency_hz": 50,
      "recommendations": [
        "Maintain current heating profile",
        "Monitor m=2,n=1 mode amplitude"
      ]
    }
  }
```

---

### 2.3 AI Control Optimization API

#### POST /plasma/control/optimize
Request AI-optimized control parameters.

```yaml
Request:
  {
    "current_state": {
      "temperature_keV": 10,
      "density_m3": 1.0,
      "confinement_time_s": 2.5,
      "heating_power_mw": { "nbi": 30, "icrh": 10, "ecrh": 5 }
    },
    "target": {
      "q_factor": 10,
      "steady_state_duration_s": 300
    },
    "constraints": {
      "max_heating_power_mw": 50,
      "max_divertor_heat_mw_m2": 10
    }
  }

Response: 200 OK
  {
    "optimized_control": {
      "heating_power_mw": {
        "nbi": 33,
        "icrh": 10,
        "ecrh": 7
      },
      "plasma_shape": {
        "elongation": 1.85,
        "triangularity": 0.5
      },
      "predicted_performance": {
        "q_factor": 10.2,
        "confinement_improvement": "+15%"
      },
      "confidence": 0.88
    }
  }
```

---

### 2.4 Energy Balance API

#### GET /fusion/energy-balance/{shot_id}
Analyze energy balance for a shot.

```yaml
Parameters:
  shot_id: string (path, required)

Response: 200 OK
  {
    "energy_balance": {
      "input_power_mw": {
        "ohmic": 1,
        "nbi": 33,
        "icrh": 10,
        "ecrh": 6,
        "total": 50
      },
      "output_power_mw": {
        "fusion": 500,
        "radiation": 50,
        "conduction": 30,
        "convection": 20
      },
      "q_factor": 10.0,
      "energy_confinement_time_s": 3.2,
      "h_factor": 1.0
    }
  }
```

---

### 2.5 Disruption Prediction API

#### POST /fusion/predict/disruption
AI-powered disruption prediction.

```yaml
Request:
  {
    "plasma_state": {
      "temperature_keV": 10,
      "density_m3": 1.0,
      "plasma_current_ma": 15,
      "beta_percent": 2.5,
      "li": 0.85,
      "q95": 3.0
    },
    "diagnostics": {
      "mhd_amplitude": 0.3,
      "locked_mode_indicator": false,
      "radiation_peaking": 1.2
    }
  }

Response: 200 OK
  {
    "prediction": {
      "disruption_probability": 0.15,
      "time_to_disruption_s": null,
      "confidence": 0.92,
      "risk_factors": [
        { "factor": "mhd_activity", "contribution": 0.08 },
        { "factor": "density_limit", "contribution": 0.05 },
        { "factor": "beta_limit", "contribution": 0.02 }
      ],
      "recommended_actions": [
        { "action": "reduce_density", "priority": "low" },
        { "action": "monitor_mhd", "priority": "medium" }
      ]
    }
  }
```

---

### 2.6 Shot Database API

#### GET /shots
List plasma shots with filtering.

```yaml
Parameters:
  reactor: string (query, optional)
  date_from: ISO8601 (query, optional)
  date_to: ISO8601 (query, optional)
  min_q_factor: number (query, optional)
  limit: integer (query, default: 100, max: 1000)
  offset: integer (query, default: 0)

Response: 200 OK
  {
    "shots": [
      {
        "shot_id": "KSTAR-20240415-001",
        "timestamp": "2024-04-15T10:30:00Z",
        "reactor": "KSTAR",
        "duration_s": 48,
        "q_factor": 1.2,
        "max_temperature_keV": 10
      }
    ],
    "total": 1500,
    "limit": 100,
    "offset": 0
  }
```

---

## 3. WebSocket Streaming API

### 3.1 Real-time Plasma Stream

```yaml
Endpoint: wss://api.fusion.wia.live/v1/stream/plasma
Protocol: WebSocket
Authentication: Query parameter ?token=<jwt>
```

#### Subscribe Message
```json
{
  "action": "subscribe",
  "channels": ["plasma_state", "stability", "control"],
  "reactor": "KSTAR",
  "sample_rate_hz": 100
}
```

#### Data Message
```json
{
  "channel": "plasma_state",
  "timestamp": "2024-04-15T10:30:00.150Z",
  "data": {
    "temperature_keV": 10.2,
    "density_m3": 1.05,
    "q_factor": 1.18
  }
}
```

---

## 4. Error Handling

### 4.1 Error Response Format

```json
{
  "error": {
    "code": "VALIDATION_ERROR",
    "message": "Invalid plasma state data",
    "details": [
      { "field": "temperature_keV", "error": "must be positive" }
    ],
    "request_id": "req-12345"
  }
}
```

### 4.2 Error Codes

| HTTP Status | Code | Description |
|-------------|------|-------------|
| 400 | VALIDATION_ERROR | Invalid request data |
| 401 | UNAUTHORIZED | Missing or invalid token |
| 403 | FORBIDDEN | Insufficient permissions |
| 404 | NOT_FOUND | Resource not found |
| 429 | RATE_LIMITED | Too many requests |
| 500 | INTERNAL_ERROR | Server error |
| 503 | SERVICE_UNAVAILABLE | Maintenance mode |

---

## 5. Rate Limiting

```yaml
Default Limits:
  - Anonymous: 100 requests/hour
  - Authenticated: 10,000 requests/hour
  - Premium: 100,000 requests/hour

Headers:
  X-RateLimit-Limit: 10000
  X-RateLimit-Remaining: 9950
  X-RateLimit-Reset: 1713177600
```

---

## 6. SDK Examples

### 6.1 TypeScript/JavaScript

```typescript
import { FusionClient } from '@wia/fusion-sdk';

const client = new FusionClient({
  apiKey: process.env.WIA_FUSION_API_KEY,
  reactor: 'KSTAR'
});

// Record plasma state
await client.plasma.recordState({
  temperature_keV: { ion: 10, electron: 10 },
  density_m3: 1.0,
  confinement_time_s: 3.0
});

// Get disruption prediction
const prediction = await client.fusion.predictDisruption({
  beta_percent: 2.5,
  mhd_amplitude: 0.3
});

console.log(`Disruption risk: ${prediction.probability * 100}%`);
```

### 6.2 Python

```python
from wia_fusion import FusionClient

client = FusionClient(api_key=os.environ['WIA_FUSION_API_KEY'])

# Stream real-time data
async for state in client.stream_plasma('KSTAR'):
    print(f"Q-factor: {state.q_factor}")
    if state.disruption_risk > 0.5:
        await client.control.emergency_shutdown()
```

---

## 7. Versioning

API versions follow semantic versioning:
- `/v1/` - Current stable version
- `/v2-beta/` - Next version preview

Deprecation notices are provided 6 months in advance.

---

**弘益人間 - Benefit All Humanity**

© 2025 WIA - World Certification Industry Association

---

## Annex A — Conformance Tier Matrix

WIA conformance for WIA-FUSION is evaluated across three tiers:

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

- `wia-standards/standards/WIA-FUSION/api/` — TypeScript SDK skeleton
- `wia-standards/standards/WIA-FUSION/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/WIA-FUSION/simulator/` — interactive browser-based simulator for the PHASE protocol

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


## Annex E — Implementation Notes for PHASE-2-API-INTERFACE

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-2-API-INTERFACE.

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
