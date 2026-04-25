# WIA-SOC-008 Phase 2: API Interface Specification

**Version:** 1.0.0  
**Status:** Approved  
**Last Updated:** 2025-12-26

弘益人間 (Hongik Ingan) - Benefit All Humanity

---

## 1. Overview

Phase 2 defines RESTful API endpoints, WebSocket interfaces, and GraphQL schemas for water supply system integration. All implementations MUST support the core REST API.

## 2. Base API Specification

### 2.1 Endpoint Structure

```
https://api.{utility-domain}/wia/soc-008/v1/{resource}
```

**Required Headers:**
- `Content-Type: application/json`
- `Authorization: Bearer {token}`
- `X-WIA-Version: 1.0.0`
- `X-Request-ID: {UUID}` (for tracing)

### 2.2 Authentication

**Supported Methods:**
- OAuth 2.0 (REQUIRED)
- API Keys (OPTIONAL for internal systems)
- mTLS (RECOMMENDED for machine-to-machine)

**Token Scopes:**
- `read:water-quality`
- `read:consumption`
- `read:network`
- `write:alerts`
- `admin:system`

### 2.3 Rate Limiting

| Tier | Requests/minute | Burst | Cost |
|------|----------------|-------|------|
| Public | 60 | 10 | Free |
| Registered | 600 | 100 | Free |
| Premium | 6000 | 1000 | Paid |
| Enterprise | Unlimited | Unlimited | Contract |

## 3. Core API Endpoints

### 3.1 System Information

#### GET /system/info
Get system identity and capabilities

**Response:**
```json
{
  "systemId": "WS-2025-CITY-001",
  "utility": "City Water Authority",
  "certificationLevel": "ADVANCED",
  "capabilities": ["water-quality", "leak-detection", "smart-metering"],
  "coverage": {
    "population": 500000,
    "networkLength": 1200.5,
    "zones": 15
  },
  "status": "operational"
}
```

### 3.2 Water Quality

#### GET /water-quality/current
Get current water quality readings

**Query Parameters:**
- `zoneId` (optional): Filter by zone
- `stationId` (optional): Filter by monitoring station
- `parameters` (optional): Comma-separated list (e.g., "pH,turbidity,chlorine")

**Response:**
```json
{
  "timestamp": "2025-12-26T14:32:15Z",
  "readings": [
    {
      "stationId": "SENSOR-001",
      "location": { "lat": 37.5665, "lon": 126.9780 },
      "parameters": {
        "pH": { "value": 7.3, "status": "normal" },
        "turbidity": { "value": 0.8, "status": "normal" },
        "chlorine": { "value": 0.5, "status": "normal" }
      },
      "compliance": "compliant"
    }
  ],
  "count": 1
}
```

#### GET /water-quality/history
Get historical water quality data

**Query Parameters:**
- `startDate`: ISO 8601 datetime (REQUIRED)
- `endDate`: ISO 8601 datetime (REQUIRED)
- `stationId`: Filter by station
- `parameters`: Comma-separated parameters
- `interval`: `5min|15min|1hour|1day` (default: 15min)
- `limit`: Max records (default: 1000, max: 10000)

### 3.3 Network Status

#### GET /network/status
Get real-time network status

**Response:**
```json
{
  "timestamp": "2025-12-26T14:32:15Z",
  "overall": {
    "status": "operational",
    "efficiency": 94.2,
    "waterLoss": 8.5
  },
  "zones": [
    {
      "zoneId": "ZONE-A",
      "name": "Downtown",
      "pressure": { "average": 4.2, "min": 3.8, "max": 4.6 },
      "flowRate": { "current": 2450, "average": 2300 },
      "population": 50000,
      "status": "normal"
    }
  ],
  "alerts": 3,
  "warnings": 7
}
```

### 3.4 Leak Detection

#### GET /leaks/active
Get all active leak events

**Response:**
```json
{
  "timestamp": "2025-12-26T14:32:15Z",
  "count": 3,
  "leaks": [
    {
      "eventId": "LEAK-2025-001",
      "detectionTime": "2025-12-26T12:10:00Z",
      "location": {
        "coordinates": { "lat": 37.5665, "lon": 126.9780 },
        "address": "Main St & 5th Ave",
        "accuracy": 15.5
      },
      "severity": "high",
      "estimatedLoss": { "rate": 850, "volume": 2125 },
      "status": "repairing",
      "assignedTeam": "Team-Alpha",
      "eta": "2025-12-26T16:00:00Z"
    }
  ]
}
```

#### POST /leaks/report
Report a new leak (public endpoint)

**Request Body:**
```json
{
  "reportedBy": "citizen|sensor|patrol",
  "location": {
    "coordinates": { "lat": 37.5665, "lon": 126.9780 },
    "address": "123 Main St",
    "description": "Water pooling on street"
  },
  "urgency": "low|medium|high",
  "contactInfo": {
    "phone": "+821012345678",
    "email": "reporter@example.com"
  }
}
```

**Response:**
```json
{
  "reportId": "REPORT-2025-12345",
  "status": "received",
  "estimatedResponse": "2025-12-26T16:00:00Z",
  "trackingUrl": "https://status.utility.com/reports/REPORT-2025-12345"
}
```

### 3.5 Smart Metering

#### GET /meters/{meterId}/consumption
Get consumption data for a specific meter

**Query Parameters:**
- `startDate`: ISO 8601 datetime
- `endDate`: ISO 8601 datetime
- `interval`: `hourly|daily|monthly`

**Response:**
```json
{
  "meterId": "METER-123456",
  "customerId": "CUST-789012",
  "period": {
    "start": "2025-12-01T00:00:00Z",
    "end": "2025-12-26T23:59:59Z"
  },
  "consumption": {
    "total": 45.3,
    "average": 1.74,
    "peak": 3.2,
    "unit": "m³"
  },
  "readings": [
    {
      "timestamp": "2025-12-26T00:00:00Z",
      "value": 1.8,
      "cumulative": 45.3
    }
  ],
  "billing": {
    "amount": 45.30,
    "currency": "USD",
    "dueDate": "2026-01-15"
  }
}
```

### 3.6 Alerts and Notifications

#### GET /alerts
Get system alerts

**Query Parameters:**
- `severity`: `info|warning|error|critical`
- `status`: `active|acknowledged|resolved`
- `limit`: Max records (default: 100)

**Response:**
```json
{
  "count": 2,
  "alerts": [
    {
      "alertId": "ALERT-2025-456",
      "timestamp": "2025-12-26T14:15:00Z",
      "severity": "critical",
      "type": "water-quality",
      "message": "Chlorine level below threshold in Zone C",
      "location": { "zoneId": "ZONE-C", "stationId": "SENSOR-042" },
      "status": "active",
      "actionRequired": "Increase chlorination",
      "affectedPopulation": 25000
    }
  ]
}
```

#### POST /alerts/{alertId}/acknowledge
Acknowledge an alert

**Request Body:**
```json
{
  "acknowledgedBy": "operator-id",
  "comment": "Chlorination adjustment initiated",
  "estimatedResolution": "2025-12-26T15:00:00Z"
}
```

## 4. WebSocket API

### 4.1 Connection

```
wss://api.{utility-domain}/wia/soc-008/v1/stream
```

**Authentication:** Via query parameter or upgrade header
```
wss://api.example.com/stream?token={bearer-token}
```

### 4.2 Subscription

**Client Subscribe:**
```json
{
  "action": "subscribe",
  "channels": ["water-quality", "leaks", "alerts"],
  "filters": {
    "zoneId": "ZONE-A",
    "severity": ["warning", "critical"]
  }
}
```

**Server Acknowledgment:**
```json
{
  "action": "subscribed",
  "channels": ["water-quality", "leaks", "alerts"],
  "subscriptionId": "SUB-123456"
}
```

### 4.3 Real-time Updates

**Water Quality Update:**
```json
{
  "channel": "water-quality",
  "timestamp": "2025-12-26T14:32:15Z",
  "data": {
    "stationId": "SENSOR-001",
    "parameters": {
      "pH": { "value": 7.3, "status": "normal" }
    }
  }
}
```

**Leak Alert:**
```json
{
  "channel": "leaks",
  "timestamp": "2025-12-26T14:32:15Z",
  "event": "new-leak",
  "data": {
    "eventId": "LEAK-2025-002",
    "severity": "high",
    "location": { "lat": 37.5665, "lon": 126.9780 }
  }
}
```

## 5. GraphQL Schema (Optional)

```graphql
type WaterSupplySystem {
  systemId: ID!
  utility: String!
  certificationLevel: CertificationLevel!
  coverage: Coverage!
  waterQuality: [WaterQuality!]!
  network: NetworkStatus!
  leaks: [LeakEvent!]!
  alerts: [Alert!]!
}

type WaterQuality {
  stationId: ID!
  timestamp: DateTime!
  location: Coordinates!
  pH: Parameter!
  turbidity: Parameter!
  chlorine: Parameter!
  compliance: ComplianceStatus!
}

type Parameter {
  value: Float!
  unit: String!
  status: ParameterStatus!
}

enum ParameterStatus {
  NORMAL
  WARNING
  CRITICAL
}

type Query {
  system: WaterSupplySystem!
  waterQuality(stationId: ID, zoneId: ID): [WaterQuality!]!
  leaks(status: LeakStatus): [LeakEvent!]!
  alerts(severity: AlertSeverity): [Alert!]!
}

type Subscription {
  waterQualityUpdates(stationId: ID): WaterQuality!
  newLeaks(severity: LeakSeverity): LeakEvent!
  systemAlerts(severity: AlertSeverity): Alert!
}
```

## 6. Error Handling

### 6.1 Error Response Format

```json
{
  "error": {
    "code": "INVALID_PARAMETER",
    "message": "The specified zone ID does not exist",
    "details": {
      "parameter": "zoneId",
      "value": "ZONE-X",
      "validValues": ["ZONE-A", "ZONE-B", "ZONE-C"]
    },
    "timestamp": "2025-12-26T14:32:15Z",
    "requestId": "REQ-123456"
  }
}
```

### 6.2 Standard Error Codes

| Code | HTTP Status | Description |
|------|-------------|-------------|
| INVALID_REQUEST | 400 | Malformed request |
| UNAUTHORIZED | 401 | Invalid or missing credentials |
| FORBIDDEN | 403 | Insufficient permissions |
| NOT_FOUND | 404 | Resource not found |
| RATE_LIMIT_EXCEEDED | 429 | Too many requests |
| INTERNAL_ERROR | 500 | Server error |
| SERVICE_UNAVAILABLE | 503 | Temporary outage |

## 7. Performance Requirements

| Metric | Requirement |
|--------|-------------|
| Response Time (p95) | < 200ms |
| Response Time (p99) | < 500ms |
| Availability | > 99.9% |
| Throughput | > 1000 req/sec per instance |
| WebSocket Latency | < 100ms |

## 8. Versioning and Deprecation

- API version in URL path: `/v1/`, `/v2/`
- Backward compatibility for minor versions
- Deprecation notice: 12 months minimum
- Support period: 24 months after deprecation
- Migration guide required for major versions

---

**弘益人間 · Benefit All Humanity**

© 2025 WIA / SmileStory Inc. · MIT License

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
in lockstep across Phases 1–4 of water-supply so that conformance claims at any
Phase remain unambiguous.*

