# WIA-IND-016 PHASE 3 — Protocol Specification

**Standard:** WIA-IND-016
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 3 of 4)

---

// High demand periods - require longer stays
  if (demandLevel === 'very-high' && occupancy > 0.85) {
    return 3; // Minimum 3-night stay
  } else if (demandLevel === 'high' && occupancy > 0.75) {
    return 2; // Minimum 2-night stay
  }

  // Special events
  if (isSpecialEvent(date)) {
    return getEventMinimumLOS(date);
  }

  return 1; // No minimum
}
```

**Closed to Arrival (CTA)**:
```javascript
function shouldCloseTo Arrival(date, occupancy, nextDayOccupancy) {
  // Close one-night stays on peak nights to encourage longer stays
  if (occupancy > 0.90 && nextDayOccupancy < 0.60) {
    return true; // Don't want checkouts on high-demand night
  }

  return false;
}
```

#### 8.2.2 Overbooking Strategy

**Safe Overbooking Level**:
```
Optimal Overbooking = Historical No-Show Rate + Safety Buffer

Example:
No-show rate: 5%
Cancellation rate: 8%
Safety buffer: 2%

Maximum overbooking = 5% + (8% × 0.5) + 2% = 11%

For 100-room hotel:
Sellable inventory = 100 + (100 × 0.11) = 111 rooms
```

**Walk Management**:
```javascript
function handleOverbooking(arrivals, capacity) {
  const oversold = arrivals.length - capacity;

  if (oversold > 0) {
    // Prioritize who to walk (relocate to another hotel)
    const walkCandidates = arrivals
      .filter(r => !r.guest.vipStatus && !r.prePaid)
      .sort((a, b) => a.nights - b.nights); // Walk shorter stays first

    const toWalk = walkCandidates.slice(0, oversold);

    toWalk.forEach(reservation => {
      relocateGuest(reservation, {
        hotelClass: 'same-or-better',
        transportationProvided: true,
        compensationOffered: true,
        futureNightCredit: 1
      });
    });
  }
}
```

### 8.3 Forecasting

#### 8.3.1 Demand Forecasting Model

**Time Series Forecasting**:
```
Forecast = Trend + Seasonality + Events + RandomVariation

Trend: Long-term growth/decline
Seasonality: Recurring patterns (day of week, month)
Events: Conferences, holidays, special events
RandomVariation: Unexplained variance
```

**Example Calculation**:
```javascript
function forecastOccupancy(date, historicalData, events) {
  // Get historical average for this day of year
  const historicalAvg = getHistoricalAverage(date, historicalData);

  // Apply trend
  const trend = calculateTrend(historicalData);
  const trendAdjusted = historicalAvg * (1 + trend);

  // Apply events multiplier
  const eventMultiplier = getEventImpact(date, events);
  const eventAdjusted = trendAdjusted * eventMultiplier;

  // Day of week adjustment
  const dowMultiplier = getDayOfWeekMultiplier(date);
  const forecast = eventAdjusted * dowMultiplier;

  return {
    forecastedOccupancy: Math.min(forecast, 1.0),
    confidence: calculateConfidence(historicalData, events),
    upperBound: forecast * 1.1,
    lowerBound: forecast * 0.9
  };
}
```

---

## 9. Channel Manager Integration

### 9.1 OTA Connectivity

#### 9.1.1 Supported Channels

**Major OTAs**:
- Booking.com (largest global OTA)
- Expedia Group (Expedia, Hotels.com, Vrbo)
- Airbnb (vacation rentals)
- Agoda (Asia-Pacific focus)
- Trip.com (China market)

**GDS Systems**:
- Amadeus
- Sabre
- Galileo/Apollo

#### 9.1.2 Real-Time Updates

**Update Types**:

1. **Rate Update** (ARI - Availability, Rates, Inventory):
```json
{
  "updateType": "rate",
  "propertyId": "hotel-001",
  "roomType": "deluxe-king",
  "ratePlan": "BAR",
  "dateRange": {
    "start": "2025-01-15",
    "end": "2025-01-31"
  },
  "rates": [
    {
      "date": "2025-01-15",
      "rate": 250.00,
      "currency": "USD",
      "availability": 5,
      "restrictions": {
        "minLOS": 1,
        "maxLOS": 14,
        "closedToArrival": false,
        "closedToDeparture": false,
        "stopSell": false
      }
    }
  ]
}
```

2. **Inventory Update**:
```json
{
  "updateType": "inventory",
  "propertyId": "hotel-001",
  "roomType": "deluxe-king",
  "date": "2025-01-15",
  "available": 5,
  "totalInventory": 10
}
```

3. **Booking Import**:
```json
{
  "updateType": "booking",
  "source": "booking.com",
  "confirmationNumber": "BDC-789456123",
  "guest": {
    "firstName": "John",
    "lastName": "Smith",
    "email": "john@example.com"
  },
  "stayDetails": {
    "checkIn": "2025-01-15",
    "checkOut": "2025-01-17",
    "roomType": "deluxe-king",
    "adults": 2
  },
  "paymentDetails": {
    "totalAmount": 500.00,
    "currency": "USD",
    "status": "prepaid"
  }
}
```

### 9.2 Rate Parity Management

#### 9.2.1 Parity Monitoring

**Automated Rate Shopping**:
```javascript
async function monitorRateParity(property) {
  const channels = property.channels;
  const dates = getNext30Days();

  for (const date of dates) {
    const rates = {};

    for (const channel of channels) {
      const rate = await fetchRateFromChannel(channel, date);
      rates[channel.name] = rate;
    }

    // Check for parity violations
    const uniqueRates = new Set(Object.values(rates));
    if (uniqueRates.size > 1) {
      reportParityViolation({
        date: date,
        rates: rates,
        expected: property.baseRate,
        severity: calculateSeverity(rates)
      });
    }
  }
}
```

#### 9.2.2 Commission Management

**Channel Commission Rates**:
```
Booking.com: 15-25% (negotiable)
Expedia: 18-25%
Airbnb: 3% guest fee + 14-16% host fee
Agoda: 18-20%
Direct booking: 0% + credit card fees (2-3%)
```

**Net Rate Calculation**:
```javascript
function calculateNetRate(grossRate, channel) {
  const commission = channel.commissionRate;
  const creditCardFee = 0.03;

  // Net rate after commissions
  const netRate = grossRate * (1 - commission - creditCardFee);

  return {
    grossRate: grossRate,
    commission: grossRate * commission,
    creditCardFee: grossRate * creditCardFee,
    netRate: netRate,
    profitMargin: (netRate / grossRate) * 100
  };
}
```

---

## 10. Guest Services

### 10.1 Concierge Automation

#### 10.1.1 AI-Powered Recommendations

**Personalization Engine**:
```javascript
function generateRecommendations(guest, context) {
  const preferences = guest.preferences;
  const history = guest.stayHistory;

  // Analyze past behavior
  const diningPreferences = analyzeDiningHistory(history);
  const activityPreferences = analyzeActivityHistory(history);

  // Get local options
  const restaurants = getNearbyRestaurants(context.location);
  const attractions = getNearbyAttractions(context.location);

  // Score and rank
  const recommendations = {
    restaurants: scoreOptions(restaurants, diningPreferences),
    attractions: scoreOptions(attractions, activityPreferences),
    events: getUpcomingEvents(context.date, preferences)
  };

  return recommendations;
}
```

**Restaurant Recommendation**:
```json
{
  "recommendationId": "rec-rest-001",
  "name": "The Gourmet Bistro",
  "type": "restaurant",
  "cuisine": "French",
  "priceLevel": 3,
  "rating": 4.5,
  "distance": 500,
  "walkingTime": 7,
  "matchScore": 0.92,
  "matchReasons": [
    "Matches your preference for French cuisine",
    "Within preferred price range",
    "Highly rated by other guests"
  ],
  "reservationAvailable": true,
  "reservationLink": "https://...",
  "menuLink": "https://..."
}
```

#### 10.1.2 Request Management

**Service Request Types**:
- Wake-up call
- Room service
- Housekeeping
- Maintenance
- Transportation
- Restaurant reservations
- Tour bookings
- Spa appointments

**Request Processing**:
```javascript
function processServiceRequest(request) {
  const {type, guestId, roomNumber, details, urgency} = request;

  // Route to appropriate department
  const department = routeRequest(type);

  // Create work order
  const workOrder = {
    requestId: generateRequestId(),
    type: type,
    guestId: guestId,
    roomNumber: roomNumber,
    department: department,
    status: 'pending',
    priority: calculatePriority(urgency, type),
    details: details,
    createdAt: new Date()
  };

  // Assign to available staff
  const assignedStaff = assignToStaff(workOrder);

  // Notify guest
  notifyGuest(guestId, {
    message: `Your ${type} request has been received`,
    estimatedTime: estimateCompletionTime(type),
    assignedTo: assignedStaff.name
  });

  return workOrder;
}
```

### 10.2 Guest Communication

#### 10.2.1 Pre-Arrival Communication

**Timeline**:
```
T-30 days: Booking confirmation
T-14 days: Pre-arrival email (special requests)
T-7 days: Reminder + upgrade offers
T-3 days: Pre-check-in invitation
T-1 day: Arrival confirmation + directions
```

**Pre-Check-In Email**:
```html
Subject: Welcome to [Hotel Name] - Check-in Tomorrow!

Dear [Guest Name],

We're excited to welcome you tomorrow!

Reservation Details:
- Confirmation: ABC123
- Check-in: January 15, 2025 at 3:00 PM
- Check-out: January 18, 2025 at 11:00 AM
- Room: Deluxe King

Pre-Check-In:
Complete check-in online and receive your mobile key:
[Pre-Check-In Link]

Special Requests:
- High floor ✓
- King bed ✓
- Extra pillows ✓

Getting Here:
[Directions Link]
Parking: Complimentary valet

Questions? Reply to this email or call us at [Phone].

Best regards,
The [Hotel Name] Team
```

#### 10.2.2 During-Stay Communication

**Channels**:
- Mobile app notifications
- SMS
- In-room tablet
- Email
- WhatsApp/WeChat (international guests)

**Message Types**:
- Service confirmations
- Promotional offers
- Event notifications
- Feedback requests
- Emergency alerts

---

## 11. Point of Sale (POS) Integration

### 11.1 POS Systems

#### 11.1.1 Integration Points

**Restaurant POS**:
```json
{
  "transactionId": "POS-123456",
  "timestamp": "2025-01-15T19:30:00Z",
  "outlet": "Main Restaurant",
  "server": "John Doe",
  "roomCharge": {
    "roomNumber": "305",
    "guestName": "John Smith",
    "items": [
      {
        "description": "Ribeye Steak",
        "quantity": 1,
        "price": 45.00
      },
      {
        "description": "House Wine",
        "quantity": 2,
        "price": 12.00
      }
    ],
    "subtotal": 69.00,
    "tax": 6.21,
    "gratuity": 13.80,
    "total": 89.01,
    "signature": "[Guest Signature]"
  }
}
```

**Folio Integration**:
```javascript
function postChargeToFolio(roomNumber, charge) {
  const reservation = getActiveReservation(roomNumber);

  if (!reservation) {
    return {success: false, error: 'No active reservation'};
  }

  // Add charge to folio
  const folioEntry = {
    date: new Date(),
    description: `${charge.outlet} - ${charge.description}`,
    amount: charge.total,
    department: charge.outlet,
    reference: charge.transactionId
  };

  reservation.folio.push(folioEntry);

  // Update total amount
  reservation.totalAmount += charge.total;

  return {success: true, folioEntry: folioEntry};
}
```

### 11.2 Folio Management

#### 11.2.1 Folio Structure

```json
{
  "folioNumber": "F-20250115-001",
  "confirmationNumber": "ABC123",
  "guestName": "John Smith",
  "roomNumber": "305",
  "checkIn": "2025-01-15",
  "checkOut": "2025-01-18",
  "charges": [
    {
      "date": "2025-01-15",
      "description": "Room Charge - Night 1",
      "amount": 250.00,
      "department": "Rooms"
    },
    {
      "date": "2025-01-15",
      "description": "Main Restaurant",
      "amount": 89.01,
      "department": "F&B"
    },
    {
      "date": "2025-01-16",
      "description": "Mini Bar",
      "amount": 35.00,
      "department": "Mini Bar"
    }
  ],
  "payments": [
    {
      "date": "2025-01-15",
      "description": "Deposit",
      "amount": 250.00,
      "method": "Credit Card"
    }
  ],
  "balance": 124.01
}
```

#### 11.2.2 Split Folio

**Company/Personal Split**:
```javascript
function splitFolio(folio, splitRules) {
  const primaryFolio = {...folio, charges: []};
  const secondaryFolio = {...folio, charges: [], folioNumber: folio.folioNumber + '-2'};

  folio.charges.forEach(charge => {
    if (splitRules.companyDepartments.includes(charge.department)) {
      primaryFolio.charges.push(charge);
    } else {
      secondaryFolio.charges.push(charge);
    }
  });

  return {
    companyFolio: primaryFolio,
    personalFolio: secondaryFolio
  };
}
```

---


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

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-3-PROTOCOL.
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
for PHASE-3-PROTOCOL. The profile mechanism is non-normative and exists so that
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
  advertises `WIA-P3-PROTOCOL-Edge-only/2` is asserting conformance with
  the second major version of the named profile, not the second deployment
  of an unversioned profile.

The profile mechanism is intentionally lightweight; it is meant to make
real deployment shapes visible without forcing every deployment to
satisfy every normative requirement.

## Annex J — Reference Implementation Topology

The reference implementation topology described in this annex is
non-normative; it documents the deployment shape that the WIA
Standards working group used to validate the test vectors in Annex G
and is intended as a starting point, not a recommendation against
alternative topologies.

- **Single-tenant edge** — one runtime per organization, no shared
  state. Used for early-pilot deployments where conformance evidence
  is published manually. Sufficient for PHASE-3-PROTOCOL validation when the
  organization signs the manifest itself.
- **Multi-tenant gateway** — one shared runtime serves multiple
  tenants via header-based isolation. Typically backed by a
  rate-limited gateway (Envoy or NGINX) and a shared OAuth 2.1
  identity provider. The manifest is per-tenant; the runtime
  publishes a federation manifest that aggregates tenant manifests.
- **Federated mesh** — multiple runtimes peer to one another and
  publish their manifests to a directory service. Each peer signs
  its own manifest; the directory service signs the aggregated
  index. This is the topology used by cross-organization deployments
  that need to compose conformance.
- **Air-gapped batch** — no network connection between the runtime
  and the directory service. The runtime emits a signed evidence
  package on each batch and the operator transports the package via
  out-of-band channels. This is the topology used by regulators that
  prohibit live connectivity from sensitive environments.

Implementations declare their topology in the manifest (see Annex I).
A topology change MUST be reflected in a new manifest signature; the
prior topology's manifest remains valid for the deprecation window
described in Annex H to preserve audit traceability.
