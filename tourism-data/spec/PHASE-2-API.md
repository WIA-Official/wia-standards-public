# WIA-IND-017 — Phase 2: API Interface

> Tourism-data canonical Phase 2: API surface (search + reservation + analytics + crowd-density WebSocket).

# WIA-IND-017: Tourism Data Specification v1.0

> **Standard ID:** WIA-IND-017
> **Version:** 1.0.0
> **Published:** 2025-12-27
> **Status:** Active
> **Authors:** WIA Industry Standards Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Core Data Models](#2-core-data-models)
3. [Tourist Attractions](#3-tourist-attractions)
4. [Destination Information](#4-destination-information)
5. [Points of Interest](#5-points-of-interest)
6. [Visitor Analytics](#6-visitor-analytics)
7. [Crowd Density & Real-time Data](#7-crowd-density--real-time-data)
8. [Cultural Heritage](#8-cultural-heritage)
9. [Accessibility Standards](#9-accessibility-standards)
10. [Multi-language Support](#10-multi-language-support)
11. [Seasonality & Trends](#11-seasonality--trends)
12. [Local Experiences](#12-local-experiences)
13. [Safety & Health Information](#13-safety--health-information)
14. [Sustainable Tourism](#14-sustainable-tourism)
15. [Data Quality & Verification](#15-data-quality--verification)
16. [API Specifications](#16-api-specifications)
17. [Security & Privacy](#17-security--privacy)
18. [Implementation Guidelines](#18-implementation-guidelines)
19. [Use Cases](#19-use-cases)
20. [References](#20-references)

---


## 16. API Specifications

### 16.1 RESTful API Design

**Base URL:** `https://api.wiastandards.com/tourism/v1`

**Endpoints:**

```
GET /attractions
GET /attractions/{id}
GET /attractions/search
GET /destinations
GET /destinations/{id}
GET /destinations/{id}/attractions
GET /pois
GET /pois/search
GET /crowd-density/{attractionId}
GET /visitor-stats/{attractionId}
GET /heritage-sites
GET /experiences
GET /seasonality/{destinationId}
GET /safety/{destinationId}
```

### 16.2 Request/Response Format

**Request Headers:**
```
Accept: application/json
Accept-Language: en,fr;q=0.9
Authorization: Bearer {token}
X-API-Version: 1.0
```

**Response Format:**
```json
{
  "status": "success",
  "data": {},
  "meta": {
    "timestamp": "2025-12-27T14:30:00Z",
    "version": "1.0",
    "rateLimit": {
      "limit": 1000,
      "remaining": 995,
      "reset": "2025-12-27T15:00:00Z"
    }
  }
}
```

### 16.3 Error Handling

**Error Response:**
```json
{
  "status": "error",
  "error": {
    "code": "T001",
    "message": "Invalid location coordinates",
    "details": {
      "lat": "Must be between -90 and 90"
    }
  },
  "meta": {
    "timestamp": "2025-12-27T14:30:00Z"
  }
}
```

**Error Codes:**
- T001: Invalid location
- T002: Attraction not found
- T003: Destination not found
- T004: Invalid search parameters
- T005: API rate limit exceeded
- T006: Unauthorized
- T007: Invalid language code
- T008: Data quality too low
- T009: Service unavailable
- T010: Invalid date range

### 16.4 Rate Limiting

**Default Limits:**
- Free tier: 100 requests/hour
- Basic tier: 1,000 requests/hour
- Pro tier: 10,000 requests/hour
- Enterprise: Custom limits

**Rate Limit Headers:**
```
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 995
X-RateLimit-Reset: 1735311600
```

### 16.5 Pagination

```
GET /attractions?limit=20&offset=40

Response:
{
  "results": [...],
  "pagination": {
    "total": 500,
    "limit": 20,
    "offset": 40,
    "hasMore": true,
    "nextOffset": 60
  }
}
```

### 16.6 Filtering and Sorting

```
GET /attractions?category=cultural&minRating=4.0&sort=rating:desc

Supported operators:
- eq (equal)
- ne (not equal)
- gt (greater than)
- gte (greater than or equal)
- lt (less than)
- lte (less than or equal)
- in (in array)
- contains (string contains)
```

---



## 17. Security & Privacy

### 17.1 Data Privacy Compliance

**GDPR Compliance:**
- Data minimization
- Purpose limitation
- Storage limitation
- Accuracy
- Integrity and confidentiality
- Accountability

**CCPA Compliance:**
- Consumer rights to know
- Right to delete
- Right to opt-out
- Non-discrimination

### 17.2 Personal Data Handling

**Personal Data Includes:**
- Names
- Email addresses
- Phone numbers
- Location data
- Behavioral data
- Payment information

**Requirements:**
- Explicit consent
- Clear privacy policy
- Secure storage
- Right to access
- Right to deletion
- Data portability

### 17.3 API Authentication

**Methods:**
1. API Key authentication
2. OAuth 2.0
3. JWT tokens

**Security Requirements:**
- HTTPS only
- Token rotation
- IP whitelisting (optional)
- Rate limiting
- Request signing

### 17.4 Data Encryption

**In Transit:**
- TLS 1.3 minimum
- Strong cipher suites
- Certificate pinning (mobile apps)

**At Rest:**
- AES-256 encryption
- Key management system
- Regular key rotation

---




---

## A.1 Endpoint reference

```http
GET    /tourism-data/v1/attractions               # search attractions
GET    /tourism-data/v1/attractions/{id}          # fetch attraction record
POST   /tourism-data/v1/attractions               # contribute attraction
GET    /tourism-data/v1/destinations              # search destinations
GET    /tourism-data/v1/destinations/{id}/pois    # POIs within destination
GET    /tourism-data/v1/itineraries               # public itineraries
POST   /tourism-data/v1/reservations              # create timed-entry reservation
WS     /tourism-data/v1/crowd-density/stream      # live crowd-density feed
```

Every endpoint follows the discovery convention at `/.well-known/wia-tourism-data`.

## A.2 Search and filter

Attractions are searchable by destination, category, accessibility flags, opening status (open-now), language support, child-friendly flag, free-admission flag, price range, distance-from-point in km, heritage status, and free-text keyword. Cursor-based pagination via `?after=cursor&limit=N` (max 100). Search responses carry the attraction envelope plus a `match-rationale` block so SDKs can explain why each result matched.

## A.3 Reservation and timed-entry API

`POST /reservations` accepts a reservation envelope: attraction or POI reference, party-size, language preference, accessibility-aid request, time-slot identifier, and the contact envelope (email or phone for booking confirmation). Reservation tokens are short-lived, signed by the issuing partner, and bound to the requesting user's WIA-OMNI-API tenant credential. Cancellation and reschedule endpoints follow the ISO/TC 228 cancellation-window convention: no-charge until 24 h pre-arrival, attraction-defined cancellation fee thereafter.

## A.4 Crowd-density WebSocket

The crowd-density WebSocket multiplexes per-POI occupancy estimate (current-count, capacity, queue-length-minutes), the rolling-30-minute trend, and a categorical congestion level (`low`/`moderate`/`high`/`at-capacity`). Subscribers can filter by destination and by congestion threshold; the broker emits push events on threshold transitions so route-planning SDKs can re-route visitors before queues exceed acceptable bounds.

## A.5 Privacy envelope on visitor analytics

All analytics outputs respect GDPR Article 25 (data protection by design and by default). Aggregate visitor counts are exposed at minimum k=20 spatial bin granularity; individual journeys are NEVER exposed through the public API. Rate-limit envelopes apply to /attractions search and to POST /reservations: 1000 req/h unauthenticated, 5000 req/h authenticated, 10000 req/h premium tier with custom rate-limits negotiated in the partnership envelope per Phase 4 §A.2.

## A.6 Multilingual response envelope

Search responses honour the `Accept-Language` header per BCP 47, falling back through registered language-tag variants and finally to English. The response envelope carries the `Content-Language` header reflecting the actual language returned and a `lang-coverage` block enumerating the languages in which the attraction record is available so SDKs can offer in-app language-switching without a round-trip.


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/tourism-data/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-tourism-data-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/tourism-data-host:1.0.0` ships every tourism-data envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/tourism-data.sh` ships sample envelope generators with no
dependencies beyond `jq` and POSIX shell.

## Z.2 Cross-standard composition (recap)

This Phase composes with WIA-OMNI-API for credential storage,
WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 §5 for
federation handshake, WIA-INTENT for workload intent declaration,
and (where personal data is processed) WIA Secure Enclave for
sealed-data envelopes. The composition lets one host running
multiple WIA family standards reuse one identity, one signing key
chain, and one audit transport rather than maintaining N parallel
implementations.

## Z.3 Implementation runbook

A first implementation typically follows: stand up the reference
container; run the conformance suite against it end-to-end;
replace the mock backend with a real backend one endpoint at a
time; wire audit-log replication out to the operations sink;
onboard a single trusted peer for federation; expand to multiple
peers; promote to production with the warning-envelope subscription
enabled and the runbook in §Z.5 followed.

## Z.4 Backwards-compatibility promise

Within the 1.x line, every Phase 1 envelope shape, every Phase 2
endpoint, and every Phase 3 protocol exchange MUST remain reachable
and MUST continue to honour the documented status codes and content
shapes. Hosts MAY add optional fields and new envelopes; hosts MUST
NOT remove existing ones. Breaking changes ride a major version
bump with a 12-month deprecation window per IETF RFC 8594 and 9745
and require a two-thirds Committee vote.

## Z.5 Closing implementer note

This Phase fits inside the WIA Standards four-Phase architecture:
Phase 1 envelopes are the wire-format contract; Phase 2 surfaces
them through HTTPS; Phase 3 wraps them in protocol exchanges that
cross trust boundaries; Phase 4 integrates with the broader
ecosystem. Tourism-data deployments that follow this layering
inherit the per-Phase conformance gates and the cross-standard
composition behaviour described in Z.2.

弘益人間 — Benefit All Humanity.
