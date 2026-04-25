# WIA Cryo-Transport API Standard
## Phase 2 Specification

**Version**: 1.0.0  
**Status**: Complete  
**License**: MIT

---

## API Endpoints

### 1. Real-Time Tracking

**GET /api/v1/transports/{transportId}/status**

Response:
```json
{
  "transportId": "TR-2025-001",
  "status": "IN_TRANSIT",
  "temperature": -195.2,
  "ln2Level": 87,
  "location": {"lat": 39.8561, "lon": -104.6737},
  "eta": "2025-01-15T16:30:00Z"
}
```

### 2. Alert Notifications

**POST /api/v1/alerts**

Webhook payload for temperature alerts:
```json
{
  "alertId": "ALT-001",
  "transportId": "TR-2025-001",
  "type": "TEMPERATURE_WARNING",
  "severity": "WARNING",
  "timestamp": "2025-01-15T14:30:00Z",
  "temperature": -189.2,
  "threshold": -190.0
}
```

### 3. Route Optimization

**POST /api/v1/routes/optimize**

Request:
```json
{
  "origin": {"lat": 33.4942, "lon": -111.9261},
  "destination": {"lat": 42.5896, "lon": -82.9199},
  "transportMode": "AIR",
  "subjectType": "WHOLE_BODY",
  "constraints": {
    "maxDuration": 24,
    "backupFacilitySpacing": 200
  }
}
```

Response:
```json
{
  "routeId": "RT-2025-001",
  "distance": 2650,
  "estimatedDuration": 6.5,
  "waypoints": [...],
  "backupFacilities": [...],
  "routeScore": 0.95
}
```

---

## Authentication

All API requests require JWT authentication:
```
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
```

JWT tokens follow RFC 7519 with mandatory claims `iss`, `sub`, `exp`, `iat`, and the WIA-specific claim `wia.role` taking values `driver`, `dispatcher`, `auditor`, or `regulator`. Token signatures use Ed25519 (RFC 8032) by default; ECDSA P-256 (NIST FIPS 186-5) is acceptable for hardware-token compatibility.

OAuth 2.0 flows for third-party integrations follow RFC 6749 with PKCE (RFC 7636). Refresh tokens conform to RFC 6749 §6 and rotate on use.

Mutual-TLS client authentication (RFC 8705) is mandatory for sensor-to-server pairing where the sensor holds an X.509 certificate (RFC 5280) bound to the container identifier.

---

## Detailed API Surface

### 4. Manifest Management

**POST /api/v1/manifests** — Create transport manifest.

```json
{
  "manifest_id": "M-2026-0427-A",
  "subject_type": "WHOLE_BODY",
  "origin_facility": "WIA-FAC-AZ-001",
  "destination_facility": "WIA-FAC-MI-002",
  "container_id": "WIA-CT-1701",
  "regulatory_classification": {
    "us_dot": "UN1977",
    "iata_dgr_class": "2.2",
    "eu_adr_class": "2A"
  },
  "scheduled_pickup": "2026-04-27T08:00:00Z"
}
```

The regulatory classification block follows the canonical identifiers used by DOT 49 CFR Part 172.101 (Hazardous Materials Table), IATA Dangerous Goods Regulations Table 4.2, and ADR/RID 2025 Table A.

**GET /api/v1/manifests/{manifest_id}** — Retrieve manifest with chain-of-custody trail.

**PATCH /api/v1/manifests/{manifest_id}** — Append amendment to manifest. Implementations MUST emit a chain-of-custody event for every amendment.

### 5. Sensor Telemetry Ingestion

**POST /api/v1/transports/{transportId}/telemetry**

Sensor telemetry ingestion is the highest-volume endpoint. Implementations MUST support both single-event and batched submissions:

```json
{
  "transportId": "TR-2025-001",
  "events": [
    {
      "ts_iso8601": "2026-04-27T14:30:00Z",
      "sensor_id": "TEMP-A",
      "metric": "temperature_C",
      "value": -195.4,
      "uncertainty_C": 0.1
    },
    {
      "ts_iso8601": "2026-04-27T14:30:00Z",
      "sensor_id": "LN2-A",
      "metric": "ln2_level_pct",
      "value": 87.0,
      "uncertainty_pct": 1.5
    }
  ]
}
```

Telemetry timestamps follow ISO 8601:2019. When the sensor supports IEEE 1588 PTPv2 hardware timestamping, the receiver MUST preserve nanosecond precision in the stored record. When the sensor uses RFC 5905 NTPv4 software timestamping, the receiver MUST record the synchronisation regime in the event metadata so downstream tools can apply the appropriate uncertainty bound.

### 6. Chain-of-Custody Events

**POST /api/v1/custody/events** — Append a custody event.

The custody event schema follows the W3C PROV-O ontology mapped onto cryogenic transport semantics:

```json
{
  "event_id": "CE-2026-0427-001",
  "manifest_id": "M-2026-0427-A",
  "ts_iso8601": "2026-04-27T08:14:32Z",
  "actor": {
    "did": "did:wia:driver:DR-104",
    "credential_hash": "sha256:..."
  },
  "activity": "PICKUP",
  "location": {"lat": 33.4942, "lon": -111.9261, "elevation_m": 359},
  "container_state": {
    "temperature_C": -195.6,
    "ln2_level_pct": 95.0,
    "vacuum_torr": 7e-5
  },
  "signatures": [
    {"signer": "did:wia:driver:DR-104", "alg": "Ed25519", "sig": "base64..."},
    {"signer": "did:wia:dispatcher:DI-22", "alg": "Ed25519", "sig": "base64..."}
  ]
}
```

`activity` values are drawn from the closed enumeration `PICKUP`, `IN_TRANSIT_CHECK`, `HANDOFF`, `EMERGENCY_STOP`, `DELIVERY`, `RETURN`. Each event is countersigned by at least two roles and is appended to the immutable custody log, where the event hash is anchored to the prior event's hash, producing a tamper-evident chain.

### 7. Alert Webhook Conformance

Webhook delivery follows the conventions documented in this section. Implementations:

- Sign each delivery with HMAC-SHA-256 using the registered shared secret. The signature is carried in the `X-Wia-Signature` header.
- Include a `X-Wia-Delivery-Id` UUID for idempotent retries.
- Retry failed deliveries with exponential backoff up to a configurable maximum number of attempts.
- Record successful and failed deliveries in the audit log.

Receivers respond with HTTP 2xx within 15 seconds for success, or RFC 9457 problem-detail responses for failure.

### 8. Error Responses

All error responses follow RFC 9457 *Problem Details for HTTP APIs*:

```json
{
  "type": "https://wia-cryo.org/errors/temperature-violation",
  "title": "Temperature Violation",
  "status": 409,
  "detail": "Reported temperature -179.4°C exceeds the warning threshold -190°C.",
  "instance": "/api/v1/transports/TR-2025-001"
}
```

The Content-Type for error responses is `application/problem+json`.

### 9. Rate Limiting

Rate-limit headers follow IETF rate-limit-headers conventions:

```
RateLimit-Limit: 1000, 1000;w=60
RateLimit-Remaining: 947
RateLimit-Reset: 53
```

Throttled requests return HTTP 429 with `Retry-After` per RFC 9110.

### 10. Pagination

List endpoints use cursor-based pagination with the `Link` header per RFC 8288:

```
Link: <https://api.wia-cryo.org/api/v1/transports?cursor=...&limit=100>; rel="next"
```

---

## OpenAPI Description

The full machine-readable interface description is published as an OpenAPI 3.1.0 document at `/api/v1/openapi.json`. Schema validation uses JSON Schema draft 2020-12 as embedded in OpenAPI 3.1.

## Versioning Policy

Path versioning (`/api/v1`, `/api/v2`) follows semantic-versioning principles. Minor versions remain wire-compatible with the prior minor of the same major. Major bumps coexist with the prior major for at least 12 months before deprecation.

---

## Reference Standards Alignment

| Concern | Reference | Role |
|---------|-----------|------|
| HTTP semantics | RFC 9110 | Transport |
| HTTP/1.1 | RFC 9112 | Transport |
| HTTP/2 | RFC 9113 | Transport |
| HTTP/3 over QUIC | RFC 9114, RFC 9000 | Mobile-network resilience |
| TLS 1.3 | RFC 8446 | Encryption in transit |
| Mutual TLS | RFC 8705 | Sensor pairing |
| JWT | RFC 7519 | Bearer tokens |
| OAuth 2.0 + PKCE | RFC 6749, RFC 7636 | Federated identity |
| Time encoding | ISO 8601:2019 | Telemetry timestamps |
| Hardware time-sync | IEEE 1588-2019 PTPv2 | Sub-microsecond precision |
| Software time-sync | RFC 5905 NTPv4 | Millisecond precision |
| Provenance | W3C PROV-O | Chain-of-custody graph |
| Identity | W3C DID Core 1.0 | Decentralised actor identity |
| OpenAPI | OpenAPI Specification 3.1 | Machine-readable description |
| Errors | RFC 9457 | Problem Details for HTTP APIs |
| Web linking | RFC 8288 | Pagination |

All references conform to the WIA Citation & Veracity Policy v1.0 §2.1 ALLOW.

---

## Implementation Appendix

### A. Data Type Conventions

Numeric fields in API payloads follow IEEE 754-2019 binary64 by default. Temperature values are reported in degrees Celsius with a separate uncertainty field; LN2 levels are reported as a percentage of fill capacity from 0.0 to 100.0; vacuum readings are reported in torr.

Identifiers follow a stable string format: `{SYSTEM}-{YEAR}-{SEQUENCE}` for manifests, `{COUNTRY}-{FACILITY-CODE}` for facilities, `WIA-CT-{SEQUENCE}` for containers, and `WIA-{ROLE}-{SEQUENCE}` for personnel.

### B. Idempotency

Endpoints that create resources accept an `Idempotency-Key` request header containing a UUID. The server stores the response for at least 24 hours and returns the cached response on retry. This guards against duplicate manifest creation and duplicate custody events when the network connection is unreliable in transit.

### C. Bulk Operations

Telemetry ingestion supports bulk submission of up to 1,000 events per request. Bulk responses include per-event success status so that the client can retry only the failed records without resubmitting the full batch.

Custody-event bulk operations are not supported; each custody event is signed individually and submitted in a separate request to ensure that the cryptographic chain is established correctly.

### D. Health and Readiness

`GET /health` returns 200 when the service is operational, 503 when not. `GET /ready` returns 200 when the service is ready to accept production traffic (database connectivity, cache primed, signing key available). These endpoints follow the conventions described in the Cloud Native Computing Foundation operator guidance and are exempt from authentication.

### E. Cross-Origin Resource Sharing

CORS responses follow the W3C CORS specification (now part of the Fetch Living Standard). The reference deployment allows browser-side dispatcher dashboards to be hosted on a different origin from the API, with explicit allow-list of methods and headers per RFC 6454 (Web Origin).

### F. Request Tracing

Every request carries a W3C Trace Context `traceparent` header so that distributed traces span the dispatcher dashboard, the API tier, the telemetry ingestion pipeline, and the database. Trace identifiers are emitted in error responses to aid debugging.

---

© 2025 WIA
