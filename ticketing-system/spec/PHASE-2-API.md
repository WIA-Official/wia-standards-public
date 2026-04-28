# WIA-IND-019 — Phase 2: API Interface

> Ticketing-system canonical Phase 2: API surface (reservation + purchase + dynamic pricing + WebSocket).

# WIA-IND-019: Ticketing System Standard - Complete Specification

> **Standard ID:** WIA-IND-019
> **Version:** 1.0.0
> **Status:** Active
> **Category:** IND (Industry)
> **Last Updated:** 2025-12-27
> **Authors:** WIA Industry Standards Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Scope](#2-scope)
3. [Normative References](#3-normative-references)
4. [Terms and Definitions](#4-terms-and-definitions)
5. [Ticket Data Format](#5-ticket-data-format)
6. [QR Code and Barcode Standards](#6-qr-code-and-barcode-standards)
7. [Seat Reservation System](#7-seat-reservation-system)
8. [Dynamic Pricing Algorithms](#8-dynamic-pricing-algorithms)
9. [Fraud Prevention](#9-fraud-prevention)
10. [Ticket Transfer and Resale](#10-ticket-transfer-and-resale)
11. [Multi-Venue Support](#11-multi-venue-support)
12. [Season Pass Management](#12-season-pass-management)
13. [Access Control Integration](#13-access-control-integration)
14. [API Specification](#14-api-specification)
15. [Security Requirements](#15-security-requirements)
16. [Privacy and Data Protection](#16-privacy-and-data-protection)
17. [Interoperability](#17-interoperability)
18. [Testing and Certification](#18-testing-and-certification)
19. [Appendices](#19-appendices)

---


## 7. Seat Reservation System

### 7.1 Seat Numbering

#### 7.1.1 Standard Format

Seats MUST be identified using:

```
{SECTION}-{ROW}-{SEAT}

Examples:
- A-12-15 (Section A, Row 12, Seat 15)
- ORCH-F-101 (Orchestra, Row F, Seat 101)
- GA-FLOOR-001 (General Admission Floor, Position 1)
```

#### 7.1.2 Accessible Seating

Accessible seats MUST include metadata:

```json
{
  "seat": "ACC-A-5",
  "accessible": true,
  "features": [
    "wheelchair-space",
    "companion-seat",
    "transfer-seat",
    "aisle-access"
  ],
  "width": 90, // cm
  "clearance": 120, // cm
  "sightlines": "unobstructed",
  "companion": "ACC-A-6"
}
```

### 7.2 Capacity Management

#### 7.2.1 Real-Time Inventory

The system MUST track capacity in real-time:

```typescript
interface CapacityStatus {
  total: number;
  sold: number;
  reserved: number;
  available: number;
  blocked: number; // maintenance, VIP holds
  utilization: number; // percentage (0-1)
}

// Example
{
  total: 50000,
  sold: 32450,
  reserved: 1250, // pending payment
  available: 15800,
  blocked: 500,
  utilization: 0.675 // 67.5%
}
```

#### 7.2.2 Seat Locking

During purchase, seats MUST be locked for **10 minutes**:

```typescript
const lock = await lockSeat({
  seatId: 'A-12-15',
  duration: 600, // seconds
  userId: 'USER-123',
  sessionId: 'SESSION-ABC'
});

// Auto-release after timeout
setTimeout(() => releaseSeat(lock.id), lock.duration * 1000);
```

#### 7.2.3 Overbooking Prevention

The system MUST prevent overbooking using distributed locks:

```typescript
// Atomic seat reservation
const reservation = await atomicReserve({
  seats: ['A-12-15', 'A-12-16'],
  timeout: 600,
  consistency: 'strong' // require all or none
});
```

### 7.3 Seat Maps

#### 7.3.1 SVG Format

Venue seat maps SHOULD be provided in SVG format:

```xml
<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1000 600">
  <g id="section-A">
    <rect id="seat-A-12-15"
          x="300" y="200"
          width="20" height="20"
          class="seat available"
          data-price="150.00"
          data-type="standard" />
  </g>
</svg>
```

Seat classes:
- `available`: Green (#10B981)
- `sold`: Gray (#6B7280)
- `selected`: Blue (#3B82F6)
- `blocked`: Red (#EF4444)
- `accessible`: Purple (#8B5CF6)

---



## 8. Dynamic Pricing Algorithms

### 8.1 Pricing Model

#### 8.1.1 Base Formula

The dynamic price at time `t` is calculated as:

```
P(t) = B × D(t) × T(t) × M(t) × S(t) × C(t)

Where:
- P(t) = Price at time t
- B = Base price (fixed)
- D(t) = Demand factor (0.5 - 3.0)
- T(t) = Time decay factor (0.6 - 1.5)
- M(t) = Market condition multiplier (0.8 - 1.3)
- S(t) = Supply scarcity factor (0.7 - 2.5)
- C(t) = Competition factor (0.9 - 1.1)
```

#### 8.1.2 Demand Factor D(t)

```typescript
function calculateDemandFactor(metrics: DemandMetrics): number {
  const {
    pageViews,        // Last 24h page views
    searchVolume,     // Search queries for event
    socialMentions,   // Social media mentions
    historyRate       // Similar event sell-through
  } = metrics;

  // Weighted scoring
  const score =
    (pageViews / 10000) * 0.3 +
    (searchVolume / 5000) * 0.3 +
    (socialMentions / 1000) * 0.2 +
    historyRate * 0.2;

  // Clamp to [0.5, 3.0]
  return Math.max(0.5, Math.min(3.0, score));
}
```

#### 8.1.3 Time Decay Factor T(t)

```typescript
function calculateTimeDecay(daysUntilEvent: number): number {
  if (daysUntilEvent > 90) {
    // Early bird: 40% discount
    return 0.6;
  } else if (daysUntilEvent > 60) {
    // Moderate early: 20% discount
    return 0.8;
  } else if (daysUntilEvent > 30) {
    // Normal pricing
    return 1.0;
  } else if (daysUntilEvent > 7) {
    // Rising: 10% increase
    return 1.1;
  } else if (daysUntilEvent > 1) {
    // Last week: 20% increase
    return 1.2;
  } else {
    // Last minute: 50% increase
    return 1.5;
  }
}
```

#### 8.1.4 Supply Scarcity Factor S(t)

```typescript
function calculateScarcityFactor(capacityRemaining: number): number {
  // capacityRemaining is percentage (0-1)

  if (capacityRemaining > 0.70) {
    // Plenty available: 30% discount
    return 0.7;
  } else if (capacityRemaining > 0.50) {
    // Half full: 10% discount
    return 0.9;
  } else if (capacityRemaining > 0.25) {
    // Getting scarce: normal price
    return 1.0;
  } else if (capacityRemaining > 0.10) {
    // Very limited: 50% increase
    return 1.5;
  } else {
    // Almost sold out: 150% increase
    return 2.5;
  }
}
```

#### 8.1.5 Market Condition Multiplier M(t)

```typescript
function calculateMarketMultiplier(conditions: MarketConditions): number {
  const {
    economicIndex,    // Economic indicator (0-100)
    seasonality,      // Season factor (0.8-1.2)
    competitorPrices, // Average competitor pricing
    weatherForecast   // Weather impact (outdoor events)
  } = conditions;

  let multiplier = 1.0;

  // Economic conditions
  if (economicIndex > 70) multiplier *= 1.1;
  else if (economicIndex < 40) multiplier *= 0.9;

  // Seasonality
  multiplier *= seasonality;

  // Competition (adjust to market)
  multiplier *= (competitorPrices / basePrice);

  // Weather (outdoor events only)
  if (weatherForecast === 'poor') multiplier *= 0.85;

  // Clamp to [0.8, 1.3]
  return Math.max(0.8, Math.min(1.3, multiplier));
}
```

### 8.2 Pricing Constraints

#### 8.2.1 Price Floor and Ceiling

```typescript
interface PricingConstraints {
  basePrice: number;
  minPrice: number;        // Never below this
  maxPrice: number;        // Never above this
  minPriceRatio: number;   // e.g., 0.5 (50% of base)
  maxPriceRatio: number;   // e.g., 2.5 (250% of base)
}

function applyConstraints(
  calculatedPrice: number,
  constraints: PricingConstraints
): number {
  const { basePrice, minPrice, maxPrice, minPriceRatio, maxPriceRatio } = constraints;

  const floor = Math.max(minPrice, basePrice * minPriceRatio);
  const ceiling = Math.min(maxPrice, basePrice * maxPriceRatio);

  return Math.max(floor, Math.min(ceiling, calculatedPrice));
}
```

#### 8.2.2 Price Change Rate Limiting

Prices MUST NOT change more than **10% per hour**:

```typescript
function rateLimitPriceChange(
  currentPrice: number,
  newPrice: number,
  timeDelta: number // seconds
): number {
  const maxChangePerHour = 0.10; // 10%
  const maxChange = currentPrice * maxChangePerHour * (timeDelta / 3600);

  if (newPrice > currentPrice + maxChange) {
    return currentPrice + maxChange;
  } else if (newPrice < currentPrice - maxChange) {
    return currentPrice - maxChange;
  }

  return newPrice;
}
```

### 8.3 Tiered Pricing

Different seat sections MAY have different base prices:

```typescript
const pricingTiers = {
  'VIP': { basePrice: 500, multiplier: 1.0 },
  'Premium': { basePrice: 300, multiplier: 1.0 },
  'Standard': { basePrice: 150, multiplier: 1.0 },
  'Balcony': { basePrice: 75, multiplier: 1.0 },
  'General': { basePrice: 50, multiplier: 1.0 }
};
```

---



## 14. API Specification

### 14.1 RESTful API Endpoints

```
POST   /api/v1/tickets              - Create ticket
GET    /api/v1/tickets/:id          - Get ticket details
PUT    /api/v1/tickets/:id          - Update ticket
DELETE /api/v1/tickets/:id          - Cancel ticket

POST   /api/v1/tickets/:id/validate - Validate ticket
POST   /api/v1/tickets/:id/transfer - Transfer ticket
POST   /api/v1/tickets/:id/checkin  - Check-in ticket

GET    /api/v1/events/:id           - Get event details
GET    /api/v1/events/:id/seats     - Get seat availability
POST   /api/v1/events/:id/reserve   - Reserve seats

GET    /api/v1/pricing/dynamic      - Calculate dynamic price
GET    /api/v1/pricing/forecast     - Price forecast

POST   /api/v1/resale/listings      - Create resale listing
GET    /api/v1/resale/listings/:id  - Get listing details
POST   /api/v1/resale/purchase      - Purchase resale ticket

GET    /api/v1/season-passes/:id    - Get season pass
POST   /api/v1/season-passes/:id/redeem - Redeem for event
```

### 14.2 Authentication

All API requests MUST include authentication:

```
Authorization: Bearer <JWT_TOKEN>
X-API-Key: <API_KEY>
```

### 14.3 Rate Limiting

- **Standard tier**: 100 requests/minute
- **Premium tier**: 1000 requests/minute
- **Enterprise tier**: 10000 requests/minute

---




---

## A.1 Endpoint reference

```http
GET    /tickets/v1/events                          # search events
GET    /tickets/v1/events/{id}                     # event details + seat map
GET    /tickets/v1/events/{id}/availability        # available seats
POST   /tickets/v1/reservations                    # reserve seats (timed hold)
POST   /tickets/v1/reservations/{id}/purchase      # convert reservation → ticket
GET    /tickets/v1/tickets/{id}                    # fetch ticket envelope
POST   /tickets/v1/tickets/{id}/transfer           # transfer to another holder
POST   /tickets/v1/tickets/{id}/refund             # refund per policy
WS     /tickets/v1/events/{id}/availability/stream # real-time availability
```

Every endpoint follows the discovery convention at `/.well-known/wia-ticketing-system`.

## A.2 Reservation lifecycle

`POST /reservations` places a timed hold on the requested seats (default 8 minutes). State transitions: pending → held → purchased (or → expired / cancelled). Expired holds release the seats automatically; cancellations are allowed during the held window. Concurrency: optimistic locking on the seat-id set; conflicting requests receive `409 Conflict` with the conflicting seat list.

## A.3 Purchase API and payment

`POST /reservations/{id}/purchase` accepts a tokenised payment-method handle (PCI-DSS 4.0 SAQ-A scope). The endpoint authorises the payment with the gateway, on success captures the funds, mints the ticket envelope (Phase 1 §A.1), and emits the `ticket.created` webhook. Failed authorisation rolls back the reservation; partial captures are not supported by the WIA contract but may be mediated by gateway-specific extensions.

## A.4 Dynamic-pricing API

The dynamic-pricing module is invoked transparently during the reservation step; the price returned is a binding offer for the duration of the hold. Repeated GETs against `/availability` may return different prices as demand evolves; the offered price is locked only after `POST /reservations` is called.

## A.5 Webhook events

Webhook events: `event.created`, `event.updated`, `event.cancelled`, `availability.changed`, `reservation.held`, `reservation.expired`, `ticket.created`, `ticket.transferred`, `ticket.refunded`, `ticket.scanned`. HMAC-SHA256 signing per the WIA family policy; receivers dedupe on `deliveryId`. Retry policy: 3 attempts at 1s/4s/16s.

## A.6 Rate-limit envelope

1000 req/h unauthenticated, 5000 req/h authenticated, 10000 req/h premium tier. Real-time-availability WebSocket connections count separately and are bounded at 100 simultaneous subscriptions per credential.


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/ticketing-system/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-ticketing-system-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/ticketing-system-host:1.0.0` ships every ticketing-system envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/ticketing-system.sh` ships sample envelope generators with no
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
ecosystem. Ticketing-system deployments that follow this layering
inherit the per-Phase conformance gates and the cross-standard
composition behaviour described in Z.2.

弘益人間 — Benefit All Humanity.
