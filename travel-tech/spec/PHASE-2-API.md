# WIA-IND-015 — Phase 2: API Interface

> Travel-tech canonical Phase 2: API surface (availability + booking + hotel + alerts WebSocket + rate-limit).

# WIA-IND-015: Travel Tech Specification v1.0

> **Standard ID:** WIA-IND-015
> **Version:** 1.0.0
> **Published:** 2025-12-27
> **Status:** Active
> **Authors:** WIA Industry Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Flight Booking Systems](#2-flight-booking-systems)
3. [Hotel Reservation Protocols](#3-hotel-reservation-protocols)
4. [Multi-Modal Transportation](#4-multi-modal-transportation)
5. [Travel Itinerary Management](#5-travel-itinerary-management)
6. [Real-Time Travel Alerts](#6-real-time-travel-alerts)
7. [Travel Document Verification](#7-travel-document-verification)
8. [Currency Conversion and Payment](#8-currency-conversion-and-payment)
9. [Travel Insurance Integration](#9-travel-insurance-integration)
10. [Loyalty Program Management](#10-loyalty-program-management)
11. [Accessibility Accommodations](#11-accessibility-accommodations)
12. [Data Privacy and Security](#12-data-privacy-and-security)
13. [API Specifications](#13-api-specifications)
14. [Implementation Guidelines](#14-implementation-guidelines)
15. [References](#15-references)

---


## 13. API Specifications

### 13.1 RESTful API Design

#### 13.1.1 Endpoint Structure

```
API Base URL: https://api.wiastandards.com/ind-015/v1

Endpoints:
├── /flights
│   ├── GET /search
│   ├── GET /:id
│   ├── POST /book
│   └── POST /cancel
├── /hotels
│   ├── GET /search
│   ├── GET /:id
│   ├── POST /book
│   └── POST /cancel
├── /transport
│   ├── GET /search
│   └── POST /book
├── /itineraries
│   ├── GET /
│   ├── POST /
│   ├── GET /:id
│   ├── PATCH /:id
│   └── DELETE /:id
├── /documents
│   ├── POST /verify
│   └── GET /requirements
├── /currency
│   ├── POST /convert
│   └── GET /rates
├── /insurance
│   ├── GET /search
│   └── POST /purchase
└── /alerts
    ├── GET /
    └── POST /subscribe
```

### 13.2 Request/Response Format

#### 13.2.1 Standard Response Structure

```json
{
  "success": true,
  "data": {
    // Actual response data
  },
  "metadata": {
    "timestamp": "2025-06-15T10:30:00Z",
    "version": "1.0.0",
    "requestId": "req-abc123"
  },
  "pagination": {
    "page": 1,
    "pageSize": 20,
    "total": 150,
    "hasMore": true
  }
}
```

**Error Response:**
```json
{
  "success": false,
  "error": {
    "code": "INVALID_PASSPORT",
    "message": "Passport has expired",
    "details": {
      "field": "passport.expiryDate",
      "expiryDate": "2023-01-15",
      "currentDate": "2025-06-15"
    }
  }
}
```

### 13.3 Rate Limiting

```
Rate Limits:
- Free Tier: 100 requests/hour
- Basic: 1,000 requests/hour
- Pro: 10,000 requests/hour
- Enterprise: Custom limits

Headers:
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 950
X-RateLimit-Reset: 1640000000

HTTP 429 (Too Many Requests) when exceeded
```

---



## 14. Implementation Guidelines

### 14.1 Best Practices

```
Development Guidelines:
1. API Integration:
   - Use official SDKs when available
   - Implement retry logic with exponential backoff
   - Cache aggressively (respect TTL)
   - Monitor API health and latency

2. Error Handling:
   - Graceful degradation
   - User-friendly error messages
   - Detailed logging (server-side)
   - Fallback options

3. Performance:
   - Lazy loading
   - Progressive image loading
   - Minimize API calls
   - CDN for static assets
   - Database indexing

4. Testing:
   - Unit tests (80%+ coverage)
   - Integration tests
   - End-to-end tests
   - Load testing
   - Security testing
```

### 14.2 Deployment

```
Production Checklist:
☐ SSL/TLS certificates
☐ Rate limiting configured
☐ Monitoring and alerts
☐ Backup strategy
☐ CDN configured
☐ Error tracking (Sentry, etc.)
☐ Performance monitoring (APM)
☐ Security scan passed
☐ GDPR compliance verified
☐ PCI-DSS compliance (if applicable)
☐ Documentation updated
☐ API versioning strategy
```

---




---

## A.1 Endpoint reference

```http
GET    /travel-tech/v1/flights/availability      # search flight availability
POST   /travel-tech/v1/flights/price             # repriced fare quote
POST   /travel-tech/v1/bookings                  # create booking (PNR)
GET    /travel-tech/v1/bookings/{id}             # fetch PNR
POST   /travel-tech/v1/bookings/{id}/cancel      # cancel booking
GET    /travel-tech/v1/hotels/availability       # hotel availability search
POST   /travel-tech/v1/hotels/reservations       # create hotel reservation
WS     /travel-tech/v1/alerts/stream             # real-time travel alerts
GET    /travel-tech/v1/itineraries/{id}          # full multi-modal itinerary
```

Every endpoint follows the discovery convention at `/.well-known/wia-travel-tech`.

## A.2 Availability and pricing API

`GET /flights/availability` accepts origin, destination, date, passenger-mix, and cabin-preference parameters, returning the canonical flight-segment envelope per Phase 1 §A.2 with the per-fare-family availability count and the price envelope. `POST /flights/price` repriceses a candidate itinerary with the latest fare and tax components; the response carries a server-side priceQuoteToken with a 5-minute expiry that the booking endpoint validates.

## A.3 Booking lifecycle API

`POST /bookings` creates a PNR from a priceQuoteToken plus passenger details, payment envelope, ancillary selections, and SSR requests. Success returns the PNR identifier and the IATA record locator; the response also carries the issued-ticket envelope (e-ticket numbers) where instant ticketing is supported. `POST /bookings/{id}/cancel` follows the per-fare-family cancellation rules with refund computation per IATA Resolution 822 (BSP refund procedures).

## A.4 Hotel API

`GET /hotels/availability` supports geo-search (point + radius), property-name search, chain search, and POI-anchored search (within X km of an attraction). Responses carry the property envelope per Phase 1 §A.3 plus the ARI breakdown for the requested date range. `POST /hotels/reservations` creates a hotel reservation with the cancellation-policy envelope echoed back in the response so SDKs can display the binding terms before the user confirms.

## A.5 Real-time alerts WebSocket

The alerts WebSocket multiplexes flight-status updates (departed, en-route, landed, diverted, cancelled), gate changes, baggage-belt assignments, and irregular-operations (IROP) notifications. Subscribers can filter by PNR, by carrier, or by airport. The broker emits push events on threshold transitions (>=15-minute departure delay; gate-change within 60 minutes of departure; cancellation) so SDKs can push notifications to travellers within the safety-loop's hard time budget.

## A.6 Rate-limit envelope

1000 req/h unauthenticated, 5000 req/h authenticated, 10000 req/h premium tier. Search endpoints (`GET /flights/availability`) are subject to a stricter look-to-book ratio cap per IATA NDC standards: the 24-hour rolling search-to-booking ratio per credential is bounded at 500:1 unless a partnership envelope at Phase 4 §A.2 negotiates a higher limit.


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/travel-tech/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-travel-tech-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/travel-tech-host:1.0.0` ships every travel-tech envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/travel-tech.sh` ships sample envelope generators with no
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
ecosystem. Travel-tech deployments that follow this layering
inherit the per-Phase conformance gates and the cross-standard
composition behaviour described in Z.2.

弘益人間 — Benefit All Humanity.
