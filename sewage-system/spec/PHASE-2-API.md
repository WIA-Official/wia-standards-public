# WIA-SOC-009 Phase 2: API Interface Specification

**Version:** 1.0.0  
**Status:** Approved  
**Last Updated:** 2025-12-26

弘益人間 (Hongik Ingan) - Benefit All Humanity

---

## 1. Overview

Phase 2 defines the RESTful API and WebSocket interfaces for smart sewage systems. All endpoints MUST support JSON-LD content negotiation and return appropriate HTTP status codes.

## 2. Base URL Structure

```
https://api.{municipality}.sewage.example.com/v1/
```

Authentication: Bearer token (OAuth 2.0) or API key in header

## 3. System Information Endpoints

### 3.1 Get System Information

**GET** `/system/info`

Returns comprehensive system information including capacity, service area, and capabilities.

**Response 200:**
```json
{
  "systemId": "SYS-2025-001",
  "municipality": "Example City",
  "capacity": 150000,
  "servingPopulation": 250000,
  "capabilities": [
    "real_time_monitoring",
    "predictive_maintenance",
    "water_quality_testing",
    "overflow_prediction"
  ]
}
```

### 3.2 Get System Status

**GET** `/system/status`

Returns current operational status of the entire sewage system.

**Response 200:**
```json
{
  "timestamp": "2025-12-26T14:30:00Z",
  "overallStatus": "normal|warning|critical",
  "flowRate": 4.2,
  "activePumps": 12,
  "treatmentEfficiency": 97.5,
  "alerts": 3,
  "lastUpdate": "2025-12-26T14:30:00Z"
}
```

## 4. Monitoring Endpoints

### 4.1 Get Sensor Readings

**GET** `/sensors/readings`

Query parameters:
- `sensorId` (optional): Filter by specific sensor
- `type` (optional): Filter by sensor type (flow|quality|level|pressure)
- `zone` (optional): Filter by geographic zone
- `from` (optional): Start timestamp (ISO8601)
- `to` (optional): End timestamp (ISO8601)
- `limit` (default: 100, max: 1000): Number of results

**Response 200:**
```json
{
  "count": 150,
  "next": "https://api.example.com/v1/sensors/readings?offset=100",
  "results": [
    {
      "sensorId": "SENS-001",
      "type": "flow",
      "timestamp": "2025-12-26T14:30:00Z",
      "value": 4.2,
      "unit": "m³/s",
      "quality": "good"
    }
  ]
}
```

### 4.2 Get Water Quality

**GET** `/water-quality/current`

Query parameters:
- `location` (optional): Specific sampling location
- `parameters` (optional): Comma-separated list of parameters

**Response 200:**
```json
{
  "timestamp": "2025-12-26T14:30:00Z",
  "location": "Influent",
  "parameters": {
    "pH": 7.2,
    "DO": 5.8,
    "BOD": 180,
    "COD": 420,
    "TSS": 220,
    "ammoniaNitrogen": 35
  },
  "compliance": "compliant"
}
```

### 4.3 Get Flow Data

**GET** `/flow/current`

Returns current flow rates at all monitoring points.

**Response 200:**
```json
{
  "timestamp": "2025-12-26T14:30:00Z",
  "monitoringPoints": [
    {
      "locationId": "MP-001",
      "name": "Main Influent",
      "flowRate": 4.2,
      "velocity": 1.8,
      "depth": 2.3,
      "trend": "increasing|stable|decreasing"
    }
  ],
  "totalInflow": 4.2,
  "totalOutflow": 4.0,
  "bypass": 0.0
}
```

## 5. Treatment Process Endpoints

### 5.1 Get Treatment Status

**GET** `/treatment/status`

**Response 200:**
```json
{
  "timestamp": "2025-12-26T14:30:00Z",
  "stages": [
    {
      "stage": "primary",
      "status": "operational",
      "efficiency": 65,
      "flowRate": 4.2,
      "removalRate": {
        "TSS": 60,
        "BOD": 30
      }
    },
    {
      "stage": "secondary",
      "status": "operational",
      "efficiency": 95,
      "flowRate": 4.0,
      "removalRate": {
        "BOD": 90,
        "ammonia": 85
      }
    }
  ],
  "overallEfficiency": 97.5
}
```

### 5.2 Update Treatment Parameters

**POST** `/treatment/control`

Request body:
```json
{
  "stage": "secondary",
  "parameter": "aeration_rate",
  "value": 85,
  "reason": "Optimize DO levels"
}
```

**Response 200:**
```json
{
  "commandId": "CMD-12345",
  "status": "accepted",
  "executedAt": "2025-12-26T14:31:00Z"
}
```

## 6. Alert and Event Endpoints

### 6.1 Get Active Alerts

**GET** `/alerts/active`

Query parameters:
- `severity` (optional): Filter by severity level
- `category` (optional): Filter by alert category

**Response 200:**
```json
{
  "count": 3,
  "alerts": [
    {
      "alertId": "ALT-001",
      "timestamp": "2025-12-26T14:25:00Z",
      "severity": "warning",
      "category": "water_quality",
      "message": "Elevated ammonia levels detected",
      "location": "Zone 3",
      "parameters": {
        "threshold": 2.0,
        "actual": 2.3,
        "deviation": 15
      },
      "acknowledged": false
    }
  ]
}
```

### 6.2 Acknowledge Alert

**POST** `/alerts/{alertId}/acknowledge`

Request body:
```json
{
  "acknowledgedBy": "operator_name",
  "notes": "Investigating source"
}
```

**Response 200:**
```json
{
  "alertId": "ALT-001",
  "acknowledgedAt": "2025-12-26T14:32:00Z",
  "status": "acknowledged"
}
```

### 6.3 Get Event History

**GET** `/events/history`

Query parameters:
- `from` (required): Start date (ISO8601)
- `to` (required): End date (ISO8601)
- `type` (optional): Event type filter
- `limit` (default: 100): Number of results

**Response 200:**
```json
{
  "count": 42,
  "events": [
    {
      "eventId": "EVT-001",
      "timestamp": "2025-12-25T18:30:00Z",
      "type": "overflow",
      "location": "CSO-5",
      "severity": "high",
      "duration": 1800,
      "volume": 150,
      "resolved": true
    }
  ]
}
```

## 7. Reporting Endpoints

### 7.1 Generate Compliance Report

**POST** `/reports/compliance`

Request body:
```json
{
  "period": "monthly|quarterly|annual",
  "startDate": "2025-01-01",
  "endDate": "2025-01-31",
  "format": "pdf|json|csv"
}
```

**Response 202:** (Async processing)
```json
{
  "reportId": "RPT-001",
  "status": "processing",
  "estimatedCompletion": "2025-12-26T14:35:00Z",
  "downloadUrl": null
}
```

**GET** `/reports/{reportId}/status`

**Response 200:**
```json
{
  "reportId": "RPT-001",
  "status": "completed",
  "downloadUrl": "https://api.example.com/v1/reports/RPT-001/download",
  "expiresAt": "2025-12-27T14:35:00Z"
}
```

### 7.2 Get Performance Metrics

**GET** `/metrics/performance`

Query parameters:
- `period` (required): daily|weekly|monthly|yearly
- `startDate` (required): Start date
- `endDate` (optional): End date

**Response 200:**
```json
{
  "period": "monthly",
  "startDate": "2025-12-01",
  "endDate": "2025-12-31",
  "metrics": {
    "averageInflowRate": 3.8,
    "totalVolumeTreated": 9936000,
    "overallEfficiency": 97.2,
    "complianceRate": 100,
    "overflowEvents": 2,
    "energyConsumption": 125000,
    "chemicalCost": 18500,
    "maintenanceCost": 42000
  }
}
```

## 8. Predictive Analytics Endpoints

### 8.1 Get Overflow Prediction

**GET** `/predictions/overflow`

Query parameters:
- `lookahead` (default: 24): Hours to predict ahead
- `zone` (optional): Specific zone

**Response 200:**
```json
{
  "timestamp": "2025-12-26T14:30:00Z",
  "predictions": [
    {
      "location": "CSO-5",
      "riskLevel": "high|medium|low",
      "probability": 0.75,
      "expectedTime": "2025-12-26T20:00:00Z",
      "estimatedVolume": 200,
      "confidence": 0.85
    }
  ]
}
```

### 8.2 Get Equipment Failure Prediction

**GET** `/predictions/equipment`

**Response 200:**
```json
{
  "predictions": [
    {
      "equipmentId": "PUMP-003",
      "equipmentType": "centrifugal_pump",
      "failureProbability": 0.68,
      "expectedFailureDate": "2026-01-15",
      "confidence": 0.82,
      "recommendedAction": "Schedule preventive maintenance",
      "estimatedCost": 5000
    }
  ]
}
```

## 9. WebSocket Real-time Streaming

**WebSocket** `/ws/stream`

Connection headers:
- `Authorization`: Bearer {token}
- `Sec-WebSocket-Protocol`: wia-soc-009-v1

Subscribe to channels:
```json
{
  "action": "subscribe",
  "channels": ["flow", "water_quality", "alerts", "equipment"]
}
```

Receive real-time updates:
```json
{
  "channel": "flow",
  "timestamp": "2025-12-26T14:30:05Z",
  "data": {
    "locationId": "MP-001",
    "flowRate": 4.3,
    "change": 0.1
  }
}
```

## 10. Error Responses

Standard HTTP status codes:
- `200 OK`: Success
- `201 Created`: Resource created
- `202 Accepted`: Async operation initiated
- `400 Bad Request`: Invalid request
- `401 Unauthorized`: Authentication required
- `403 Forbidden`: Insufficient permissions
- `404 Not Found`: Resource not found
- `429 Too Many Requests`: Rate limit exceeded
- `500 Internal Server Error`: Server error
- `503 Service Unavailable`: Temporary unavailability

Error format:
```json
{
  "error": {
    "code": "INVALID_PARAMETER",
    "message": "Parameter 'flowRate' must be positive",
    "details": {
      "parameter": "flowRate",
      "value": -1.5
    }
  }
}
```

---

© 2025 WIA · MIT License

## P.2 API Surface Cross-References

The API surface defined in this Phase consumes and emits the data formats from
Phase 1 and is transported by the protocol layer in Phase 3. Operators deploy
the surface using the integration patterns in Phase 4.

### P.2.1 Resource Naming

Resource paths follow REST conventions with snake_case segments. Identifier
segments use the canonical UUID encoding from Phase 1.

```
/v1/{collection}                        # collection
/v1/{collection}/{id}                    # member
/v1/{collection}/{id}/{sub_collection}   # nested collection
/v1/{collection}/{id}:{action}           # custom action (POST)
```

### P.2.2 Pagination

List endpoints support cursor-based pagination:

| Param | Default | Max | Description |
|-------|---------|-----|-------------|
| `page_size` | 50 | 500 | Items per page |
| `page_token` | empty | — | Opaque continuation token |

Servers MUST return `next_page_token` when the result set is truncated and an
empty string when the final page has been delivered.

### P.2.3 Idempotency

State-changing operations accept the `Idempotency-Key` header (RFC-style).
Servers MUST cache the response keyed by `(principal, key)` for at least 24 h
and replay the same response on retry.

### P.2.4 Field Masks

Partial-update operations use field masks (Google AIP-161 style) to avoid
clobbering unspecified fields. Masks are dot-paths into the canonical schema
with `*` wildcards.


---

## Appendix · Common WIA Standard Provisions

> The following provisions apply to every WIA standard and are kept in sync
> across the WIA-Standards corpus. Standard-specific deviations, where they
> exist, are listed in the standard's normative body.

### A. Conformance & Compliance

#### A.1 Conformance Levels

WIA-Standards defines four conformance levels:

| Level | Required | Description |
|-------|----------|-------------|
| L1 — Format | Phase 1 | Implementation produces and consumes the canonical data format losslessly |
| L2 — Interface | Phase 1 + 2 | Implementation exposes the API surface with required behaviour |
| L3 — Protocol | Phase 1 + 2 + 3 | Implementation interoperates over at least one normative transport binding |
| L4 — Integration | All Phases | Implementation passes the conformance test suite end-to-end in a production-shaped deployment |

Conformance claims MUST cite the level and the version of the standard
against which the claim is made (e.g. "L3 conformant against v1.0").

#### A.2 Compliance Verification

The conformance test suite is published alongside this standard at
`/cli/conformance/` and `/api/conformance/`. Implementations claiming
L2 or higher MUST publish their test report. Independent re-tests are
encouraged; the WIA Working Group accepts third-party verification reports
under the policy in §E.

### B. Security Considerations

#### B.1 Threat Model

Implementers SHOULD apply STRIDE analysis covering: spoofing of identity,
tampering with messages or stored state, repudiation of operations,
information disclosure, denial of service, and elevation of privilege.

| Threat | Default Control | Where Strengthened |
|--------|-----------------|--------------------|
| Spoofing | Mutual TLS or signed tokens | Phase 3 §P.3 |
| Tampering | TLS in transit, AEAD at rest | Phase 1 §P.1 |
| Repudiation | Append-only audit log with notarization | Phase 4 §P.4 |
| Disclosure | Field-level encryption for PII / secrets | Phase 1 §P.1 |
| DoS | Rate limit per principal & global circuit breaker | Phase 2 §P.2 |
| EoP | Least-privilege RBAC + scoped tokens | Phase 2 §P.2 |

#### B.2 Cryptographic Suites

Mandatory: TLS 1.3 with AEAD ciphers (AES-128-GCM, AES-256-GCM,
CHACHA20-POLY1305). Forbidden: TLS 1.0, TLS 1.1, RC4, MD5, SHA-1 for
signatures, RSA below 2048 bits, ECDSA on curves smaller than P-256.

Post-quantum migration: implementations SHOULD adopt hybrid key
exchange combining a classical primitive with ML-KEM (FIPS 203) once a
profile is published; signature migration to ML-DSA (FIPS 204) is
expected within the L4 conformance window of v2.0.

#### B.3 Audit Requirements

L3 and L4 implementations MUST log: (a) every authentication decision,
(b) every authorization decision, (c) every state-changing operation,
(d) every export of data outside its sovereignty boundary. Logs are
write-once for at least 1 year and 90 days indexed for incident search.

### C. Versioning & Lifecycle

Versions follow Semantic Versioning 2.0.0 (MAJOR.MINOR.PATCH).

| Phase | Duration | Conformance Status |
|-------|---------:|-------------------|
| Draft | until ratification | Non-binding |
| Active | indefinite | Binding for new deployments |
| Maintenance | 24 months from successor's Active date | Binding for existing deployments |
| Retired | indefinite | Non-binding; conformance claims rescinded |

Deprecation MUST be announced at least one minor version before a feature
is removed in a major version.

### D. Internationalization & Accessibility

Implementations SHOULD support locale negotiation via the
`Accept-Language` header (RFC 4647). Date, time, number, and currency
formatting follow CLDR. User-facing surfaces MUST satisfy WCAG 2.1 AA at
minimum and SHOULD progress towards WCAG 2.2 AA. Right-to-left scripts
(Arabic, Hebrew, Persian, Urdu) and East-Asian wide characters MUST be
laid out correctly without line-breaking heuristics that split graphemes.

### E. Governance & IP Policy

This standard is maintained by the WIA Working Group under the WIA
governance charter. Editorial changes are merged via pull request. Normative
changes require working-group consensus and a 30-day public review.
Contributions are accepted under the Apache License 2.0 with explicit
patent grant. Members participate under the WIA Patent Policy,
which requires royalty-free licensing of any essential claim necessary
to implement a normative requirement.

### F. Normative References

The following references are normative; implementations MUST satisfy
the cited clauses:

- ISO/IEC 27001:2022 — Information security management systems
- ISO/IEC 27017:2015 — Cloud-services security controls
- ISO/IEC 27701:2019 — Privacy information management
- ISO/IEC 19790:2012 — Security requirements for cryptographic modules
- ISO 8601-1:2019 — Date and time representation
- IETF RFC 8446 — TLS 1.3
- IETF RFC 7519 — JSON Web Token
- IETF RFC 6749 — OAuth 2.0
- IETF RFC 9110 — HTTP Semantics
- IETF RFC 9112 — HTTP/1.1 message syntax
- IETF RFC 9113 — HTTP/2
- IETF RFC 9114 — HTTP/3
- IETF RFC 9000 — QUIC transport
- IETF RFC 4122 — UUID URN namespace
- IETF RFC 3339 — Date and time on the Internet
- IETF RFC 6838 — Media-type specifications and registration
- W3C TraceContext — Distributed tracing context
- W3C WCAG 2.1 — Accessibility guidelines
- FIPS PUB 197 — AES
- FIPS PUB 180-4 — SHA-2 family
- FIPS PUB 203 — ML-KEM (post-quantum KEM)
- FIPS PUB 204 — ML-DSA (post-quantum signature)

### G. Glossary

| Term | Definition |
|------|------------|
| Conformance | The state of satisfying every normative requirement at a given level |
| Implementation | A software, hardware, or composite artefact that claims conformance |
| Principal | The authenticated entity bound to a security context |
| Subject | The resource or person to which an operation applies |
| Sovereignty Boundary | The legal / regulatory perimeter outside of which data export is restricted |

---

*This Appendix is authored by the WIA Standards Working Group and is kept
in lockstep across Phases 1–4 of sewage-system so that conformance claims at any
Phase remain unambiguous.*

