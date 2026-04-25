# WIA-PLASTIC-ENZYME Phase 2: API Interface Specification

> **Version:** 1.0.0
> **Status:** Official
> **Last Updated:** 2025-01-01
> **Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

Phase 2 of the WIA-PLASTIC-ENZYME standard defines the API interfaces that enable software systems to interact with enzymatic plastic degradation services. The architecture follows REST principles with JSON payloads.

### 1.1 Design Principles

- **RESTful:** Resource-oriented URLs with standard HTTP methods
- **JSON-first:** All requests and responses in JSON format
- **Versioned:** API versioning in URL path (/v1/, /v2/)
- **Authenticated:** API key or OAuth 2.0 authentication
- **Rate-limited:** Configurable rate limits per client

### 1.2 Base URL

```
Production: https://api.wia.live/plastic-enzyme/v1
Sandbox:    https://sandbox.wia.live/plastic-enzyme/v1
```

---

## 2. Authentication

### 2.1 API Key Authentication

```http
GET /api/v1/enzyme/library
Authorization: Bearer wia_key_xxxxxxxxxxxxx
```

### 2.2 OAuth 2.0

```http
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials
&client_id=your_client_id
&client_secret=your_client_secret
&scope=enzyme:read process:write
```

### 2.3 Available Scopes

| Scope | Description |
|-------|-------------|
| `enzyme:read` | Read enzyme library and profiles |
| `enzyme:write` | Create and update enzyme profiles |
| `process:read` | Read degradation process data |
| `process:write` | Create and update process records |
| `quality:read` | Access quality metrics |
| `quality:certify` | Issue quality certifications |

---

## 3. API Endpoints

### 3.1 Plastic Identification

```yaml
POST /api/v1/plastic/identify
```

**Description:** Identify plastic type from spectroscopic or image data

**Request Body:**
```json
{
  "method": "ftir_spectrum",
  "data": {
    "wavenumbers": [4000, 3999, 3998],
    "absorbance": [0.02, 0.021, 0.019]
  }
}
```

**Response:**
```json
{
  "plastic_type": "PET",
  "confidence": 0.98,
  "crystallinity_estimate": 28.5,
  "contaminants_detected": ["PP traces"],
  "recommended_pretreatment": "mechanical_grinding"
}
```

### 3.2 Enzyme Matching

```yaml
GET /api/v1/enzyme/match/{plastic_type}
```

**Parameters:**
| Name | Type | Description |
|------|------|-------------|
| plastic_type | path | PET, PBAT, PLA, PCL, PHA |
| temperature | query | Operating temperature (°C) |
| ph | query | Operating pH |
| target_efficiency | query | Target degradation % |

**Response:**
```json
{
  "recommendations": [
    {
      "enzyme_id": "urn:wia:enzyme:turbopetase-v2",
      "name": "TurboPETase-v2",
      "match_score": 98.5,
      "predicted_efficiency": 97.2,
      "optimal_concentration_mg_g": 2.5,
      "estimated_time_hours": 48
    }
  ],
  "cocktail_suggestion": {
    "primary": "urn:wia:enzyme:turbopetase-v2",
    "secondary": "urn:wia:enzyme:mhetase-std",
    "ratio": "2:1",
    "synergy_bonus": "+12%"
  }
}
```

### 3.3 Enzyme Library

```yaml
GET /api/v1/enzyme/library
```

**Query Parameters:**
| Name | Type | Description |
|------|------|-------------|
| classification | string | Filter by enzyme type |
| min_temperature | number | Minimum optimal temperature |
| max_temperature | number | Maximum optimal temperature |
| engineered | boolean | Filter engineered enzymes |
| page | integer | Page number |
| per_page | integer | Results per page (max 100) |

**Response:**
```json
{
  "total": 47,
  "page": 1,
  "per_page": 20,
  "enzymes": [
    {
      "enzyme_id": "urn:wia:enzyme:turbopetase-v2",
      "name": "TurboPETase-v2",
      "classification": "PETase",
      "source_organism": "Engineered E. coli",
      "optimal_temperature": 55,
      "kcat_km": 68700,
      "reference": "doi:10.1038/s41467-024-45662-9"
    }
  ]
}
```

### 3.4 Degradation Optimization

```yaml
POST /api/v1/degradation/optimize
```

**Request:**
```json
{
  "plastic": {
    "type": "PET",
    "weight_kg": 1000,
    "crystallinity_percent": 30,
    "form": "flakes"
  },
  "constraints": {
    "max_temperature_c": 60,
    "max_time_hours": 72,
    "target_efficiency": 95
  },
  "optimization_goal": "cost"
}
```

**Response:**
```json
{
  "optimal_conditions": {
    "temperature_c": 55,
    "ph": 8.0,
    "enzyme_loading_mg_g": 2.5,
    "substrate_loading_percent": 12
  },
  "enzyme_cocktail": [
    {"enzyme_id": "urn:wia:enzyme:turbopetase-v2", "ratio": 0.7},
    {"enzyme_id": "urn:wia:enzyme:mhetase-std", "ratio": 0.3}
  ],
  "predicted_outcomes": {
    "efficiency_percent": 97.5,
    "time_hours": 48,
    "tpa_yield_kg": 820,
    "cost_per_kg": 0.45
  }
}
```

### 3.5 Degradation Prediction

```yaml
POST /api/v1/degradation/predict
```

**Request:**
```json
{
  "plastic": {"type": "PET", "weight_kg": 100, "crystallinity_percent": 25},
  "enzyme": {"enzyme_id": "urn:wia:enzyme:turbopetase-v2", "concentration_mg_g": 2.0},
  "conditions": {"temperature_c": 55, "ph": 8.0, "duration_hours": 48}
}
```

**Response:**
```json
{
  "predictions": {
    "degradation_percent": {"mean": 95.5, "std": 2.1, "ci_95": [91.3, 99.7]},
    "tpa_yield_kg": {"mean": 81.2, "std": 1.8},
    "eg_yield_kg": {"mean": 27.8, "std": 0.6}
  },
  "time_course": [
    {"hour": 0, "degradation": 0},
    {"hour": 12, "degradation": 45.2},
    {"hour": 24, "degradation": 78.5},
    {"hour": 48, "degradation": 95.5}
  ]
}
```

---

## 4. CRUD Operations

### 4.1 Enzyme Profiles

| Method | Endpoint | Description |
|--------|----------|-------------|
| GET | /api/v1/enzymes | List all enzymes |
| GET | /api/v1/enzymes/{id} | Get enzyme by ID |
| POST | /api/v1/enzymes | Create enzyme profile |
| PUT | /api/v1/enzymes/{id} | Update enzyme profile |
| DELETE | /api/v1/enzymes/{id} | Delete enzyme profile |

### 4.2 Processes

| Method | Endpoint | Description |
|--------|----------|-------------|
| GET | /api/v1/processes | List all processes |
| GET | /api/v1/processes/{id} | Get process by ID |
| POST | /api/v1/processes | Create process record |
| PUT | /api/v1/processes/{id} | Update process record |
| POST | /api/v1/processes/{id}/output | Record process output |

---

## 5. Webhooks

### 5.1 Webhook Registration

```http
POST /api/v1/webhooks
Content-Type: application/json

{
  "url": "https://your-system.com/wia-events",
  "events": ["process.completed", "quality.certified"],
  "secret": "your-webhook-secret"
}
```

### 5.2 Webhook Events

| Event | Description |
|-------|-------------|
| `process.started` | Degradation process initiated |
| `process.completed` | Degradation process finished |
| `quality.tested` | Quality testing completed |
| `quality.certified` | WIA certification issued |

### 5.3 Webhook Payload

```json
{
  "event": "process.completed",
  "timestamp": "2025-01-15T10:30:00Z",
  "data": {
    "process_id": "urn:wia:process:2025-001-abc",
    "status": "completed",
    "efficiency": 97.5
  },
  "signature": "sha256=xxxxxxxx"
}
```

---

## 6. Rate Limits

| Tier | Rate Limit | Monthly Quota |
|------|------------|---------------|
| Free | 60 requests/minute | 10,000 requests |
| Professional | 600 requests/minute | 500,000 requests |
| Enterprise | Unlimited | Unlimited |

**Rate Limit Headers:**
```http
X-RateLimit-Limit: 60
X-RateLimit-Remaining: 45
X-RateLimit-Reset: 1704067200
```

---

## 7. Error Handling

### 7.1 Error Response Format

```json
{
  "error": {
    "code": "ENZYME_NOT_FOUND",
    "message": "The specified enzyme ID does not exist",
    "details": {
      "enzyme_id": "urn:wia:enzyme:invalid-id"
    },
    "request_id": "req_abc123",
    "documentation_url": "https://docs.wia.live/errors/ENZYME_NOT_FOUND"
  }
}
```

### 7.2 Error Codes

| Code | HTTP Status | Description |
|------|-------------|-------------|
| INVALID_REQUEST | 400 | Malformed request body |
| UNAUTHORIZED | 401 | Missing or invalid authentication |
| FORBIDDEN | 403 | Insufficient permissions |
| NOT_FOUND | 404 | Resource not found |
| RATE_LIMITED | 429 | Rate limit exceeded |
| INTERNAL_ERROR | 500 | Server error |

---

## 8. SDK Support

### 8.1 Official SDKs

| Language | Package | Installation |
|----------|---------|--------------|
| TypeScript | @wia/plastic-enzyme-sdk | npm install @wia/plastic-enzyme-sdk |
| Python | wia-plastic-enzyme | pip install wia-plastic-enzyme |
| Rust | wia-plastic-enzyme | cargo add wia-plastic-enzyme |
| Go | wia-plastic-enzyme | go get github.com/wia/plastic-enzyme-go |

---

## 9. Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-01-01 | Initial release |

---

**弘益人間 (Benefit All Humanity)**

© 2025 WIA - World Certification Industry Association

---

## Annex A — Conformance Tier Matrix

WIA conformance for WIA-PLASTIC-ENZYME is evaluated across three tiers:

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

- `wia-standards/standards/WIA-PLASTIC-ENZYME/api/` — TypeScript SDK skeleton
- `wia-standards/standards/WIA-PLASTIC-ENZYME/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/WIA-PLASTIC-ENZYME/simulator/` — interactive browser-based simulator for the PHASE protocol

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

