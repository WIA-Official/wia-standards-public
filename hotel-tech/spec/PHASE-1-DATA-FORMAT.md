# WIA-IND-016 PHASE 1 — Data Format Specification

**Standard:** WIA-IND-016
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 1 of 4)

---

# WIA-IND-016: Hotel Tech Specification v1.0

> **Standard ID:** WIA-IND-016
> **Version:** 1.0.0
> **Published:** 2025-12-27
> **Status:** Active
> **Authors:** WIA Industry Standards Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Property Management Systems (PMS)](#2-property-management-systems-pms)
3. [Reservation Management](#3-reservation-management)
4. [Room Operations](#4-room-operations)
5. [Smart Room Technology](#5-smart-room-technology)
6. [Keyless Entry Systems](#6-keyless-entry-systems)
7. [Housekeeping Management](#7-housekeeping-management)
8. [Revenue Management](#8-revenue-management)
9. [Channel Manager Integration](#9-channel-manager-integration)
10. [Guest Services](#10-guest-services)
11. [Point of Sale (POS) Integration](#11-point-of-sale-pos-integration)
12. [Guest Feedback & Reputation](#12-guest-feedback--reputation)
13. [Security & Compliance](#13-security--compliance)
14. [API Specifications](#14-api-specifications)
15. [Data Models](#15-data-models)
16. [Implementation Guidelines](#16-implementation-guidelines)

---

## 1. Introduction

### 1.1 Purpose

This specification defines comprehensive standards for hotel technology systems, enabling seamless integration across all aspects of hotel operations from guest discovery to post-stay feedback.

### 1.2 Scope

The standard covers:
- Property Management System (PMS) core functionality
- Multi-channel reservation management
- Smart room automation and IoT integration
- Keyless entry and access control
- Housekeeping coordination and room status
- Dynamic pricing and revenue optimization
- Channel manager and OTA distribution
- Guest services and concierge automation
- Payment processing and PCI compliance
- Guest feedback and review management

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard aims to democratize access to advanced hotel technology, enabling properties of all sizes to deliver exceptional guest experiences while improving operational efficiency and sustainability.

### 1.4 Terminology

- **PMS**: Property Management System - central hotel operations platform
- **OTA**: Online Travel Agency (e.g., Booking.com, Expedia)
- **GDS**: Global Distribution System
- **ADR**: Average Daily Rate
- **RevPAR**: Revenue Per Available Room
- **Occupancy**: Percentage of rooms occupied
- **BAR**: Best Available Rate
- **LOS**: Length of Stay
- **PAX**: Passengers/Guests

---

## 2. Property Management Systems (PMS)

### 2.1 Core PMS Functions

#### 2.1.1 Front Desk Operations

The PMS SHALL provide front desk functionality including:

```
Front Desk Operations:
1. Guest check-in/check-out
2. Room assignment and management
3. Walk-in reservations
4. Room moves and changes
5. Guest profile management
6. Folio management
7. Payment processing
8. Wake-up call scheduling
```

#### 2.1.2 Reservation Management

**Reservation Creation**:
```json
{
  "confirmationNumber": "ABC123",
  "guestProfile": {
    "firstName": "John",
    "lastName": "Smith",
    "email": "john@example.com",
    "phone": "+1-555-0100",
    "address": {
      "street": "123 Main St",
      "city": "New York",
      "state": "NY",
      "postalCode": "10001",
      "country": "USA"
    }
  },
  "stayDetails": {
    "checkIn": "2025-01-15",
    "checkOut": "2025-01-18",
    "nights": 3,
    "roomType": "deluxe-king",
    "adults": 2,
    "children": 0
  },
  "rateDetails": {
    "rateCode": "BAR",
    "roomRate": 250.00,
    "totalAmount": 862.50,
    "taxesAndFees": 112.50,
    "currency": "USD"
  }
}
```

#### 2.1.3 Guest Profile Management

Guest profiles SHALL store:
- Personal information (name, contact, nationality)
- Stay history and preferences
- Loyalty program membership
- Payment methods
- Special requirements
- Communication preferences
- Marketing consent

**Profile Data Structure**:
```json
{
  "guestId": "guest-12345",
  "personalInfo": {
    "title": "Mr.",
    "firstName": "John",
    "lastName": "Smith",
    "dateOfBirth": "1985-06-15",
    "nationality": "US"
  },
  "preferences": {
    "roomLocation": "high-floor",
    "bedType": "king",
    "pillowType": "firm",
    "temperature": 21,
    "newspaper": "Wall Street Journal"
  },
  "loyaltyMembership": {
    "program": "Elite Rewards",
    "memberNumber": "ELT789456",
    "tier": "Gold",
    "points": 25000
  },
  "stayHistory": [
    {
      "propertyId": "hotel-001",
      "checkIn": "2024-12-01",
      "checkOut": "2024-12-05",
      "totalSpent": 1200.00
    }
  ]
}
```

### 2.2 PMS Integration Architecture

#### 2.2.1 System Integration Diagram

```
┌─────────────────┐
│   Web Booking   │
│     Engine      │
└────────┬────────┘
         │
         ▼
┌─────────────────┐      ┌──────────────┐
│  Central PMS    │◄────►│  Channel     │
│    (Core Hub)   │      │  Manager     │
└────────┬────────┘      └──────────────┘
         │
         ├──────────┬──────────┬──────────┬──────────┐
         ▼          ▼          ▼          ▼          ▼
    ┌────────┐ ┌────────┐ ┌────────┐ ┌────────┐ ┌────────┐
    │  Room  │ │Revenue │ │  POS   │ │  CRM   │ │  IoT   │
    │Controls│ │ Mgmt   │ │        │ │        │ │Gateway │
    └────────┘ └────────┘ └────────┘ └────────┘ └────────┘
```

#### 2.2.2 API Integration Standards

All PMS integrations SHALL support:
- RESTful API with JSON payload
- OAuth 2.0 authentication
- Webhook notifications for real-time updates
- Rate limiting (100 requests/minute)
- SSL/TLS encryption (minimum TLS 1.2)

### 2.3 Reporting and Analytics

#### 2.3.1 Standard Reports

The PMS SHALL generate:

1. **Daily Operations Report**:
   - Arrivals, departures, stayovers
   - Room status summary
   - Revenue breakdown
   - Occupancy percentage

2. **Financial Reports**:
   - Daily revenue report
   - Department breakdown
   - Payment method summary
   - Outstanding balances

3. **Forecasting Reports**:
   - 30/60/90 day occupancy forecast
   - Revenue projections
   - Pickup reports
   - Pace reports

#### 2.3.2 Key Performance Indicators

```
KPI Calculations:

Occupancy Rate = (Rooms Occupied / Total Rooms) × 100

ADR = Total Room Revenue / Rooms Sold

RevPAR = Total Room Revenue / Total Available Rooms
       = Occupancy Rate × ADR

GOPPAR = Gross Operating Profit / Total Available Rooms
```

---

## 3. Reservation Management

### 3.1 Reservation Lifecycle

#### 3.1.1 Reservation States

```
Reservation State Machine:

Inquiry → Quoted → Confirmed → Guaranteed
                      ↓
                  Modified ←→ Confirmed
                      ↓
              ┌───────┴───────┐
              ↓               ↓
         Checked-In      Cancelled
              ↓               ↓
         Checked-Out      No-Show
              ↓
         Completed
```

#### 3.1.2 Modification Rules

**Allowed Modifications**:
- Date changes (subject to availability)
- Room type upgrades/downgrades
- Guest count changes
- Rate adjustments (with authorization)
- Additional services

**Modification Constraints**:
```javascript
function canModifyReservation(reservation, modification) {
  const now = new Date();
  const checkIn = new Date(reservation.checkIn);
  const hoursUntilCheckIn = (checkIn - now) / (1000 * 60 * 60);

  if (reservation.status === 'checked-in' ||
      reservation.status === 'checked-out') {
    return false;
  }

  if (modification.type === 'dates' && hoursUntilCheckIn < 24) {
    return false; // Cannot modify dates within 24 hours
  }

  return true;
}
```

### 3.2 Rate Management

#### 3.2.1 Rate Plan Types

| Rate Code | Description | Restrictions | Cancellation |
|-----------|-------------|--------------|--------------|
| BAR | Best Available Rate | None | Flexible |
| CORP | Corporate Rate | ID required | Moderate |
| GOV | Government Rate | ID required | Flexible |
| AAA | AAA Member Rate | Membership | Flexible |
| PROMO | Promotional Rate | Advance purchase | Strict |
| PKG | Package Rate | Includes amenities | Moderate |
| GROUP | Group Rate | Min 10 rooms | Custom |
| LRA | Last Room Available | Non-refundable | None |

#### 3.2.2 Rate Calculation

**Base Rate Calculation**:
```
FinalRate = BaseRate × SeasonMultiplier × DOWMultiplier × LOSMultiplier

Where:
- BaseRate: Room type base price
- SeasonMultiplier: Seasonal adjustment (0.8 - 1.5)
- DOWMultiplier: Day of week adjustment (0.9 - 1.2)
- LOSMultiplier: Length of stay discount (0.85 - 1.0)
```

**Example**:
```
Deluxe King Room:
Base Rate: $250
High Season: × 1.3 = $325
Weekend: × 1.1 = $357.50
3+ nights: × 0.95 = $339.63

Final Rate: $340/night
```

### 3.3 Cancellation Policies

#### 3.3.1 Policy Types

**Flexible Policy**:
```json
{
  "type": "flexible",
  "freeCancellationHours": 24,
  "penalty": {
    "within24Hours": "1 night charge",
    "noShow": "full stay charge"
  }
}
```

**Moderate Policy**:
```json
{
  "type": "moderate",
  "freeCancellationDays": 7,
  "penalty": {
    "7to3Days": "50% charge",
    "within3Days": "100% charge",
    "noShow": "full stay + 1 night"
  }
}
```

**Strict/Non-Refundable**:
```json
{
  "type": "non-refundable",
  "refundAmount": 0,
  "benefits": {
    "discount": "20-30% lower rate",
    "pointsBonus": "double loyalty points"
  }
}
```

#### 3.3.2 Cancellation Processing

```javascript
function processCancellation(reservation) {
  const now = new Date();
  const checkIn = new Date(reservation.checkIn);
  const hoursUntilCheckIn = (checkIn - now) / (1000 * 60 * 60);

  let penalty = 0;
  let refund = reservation.totalAmount;

  if (reservation.cancellationPolicy.type === 'non-refundable') {
    penalty = reservation.totalAmount;
    refund = 0;
  } else if (hoursUntilCheckIn < 24) {
    penalty = reservation.roomRate; // 1 night
    refund = reservation.totalAmount - penalty;
  }

  return {
    refundAmount: refund,
    penaltyAmount: penalty,
    processingFee: 0
  };
}
```

---

## 4. Room Operations

### 4.1 Room Inventory Management

#### 4.1.1 Room Status Types

```
Room Status Workflow:

┌─────────────┐
│  Available  │◄──────┐
└──────┬──────┘       │
       │              │
       ▼              │
┌─────────────┐       │
│  Reserved   │       │
└──────┬──────┘       │
       │              │
       ▼              │
┌─────────────┐       │
│  Occupied   │       │
└──────┬──────┘       │
       │              │
       ▼              │
┌─────────────┐       │
│    Dirty    │       │
└──────┬──────┘       │
       │              │
       ▼              │
┌─────────────┐       │
│  Cleaning   │       │
└──────┬──────┘       │
       │              │
       ▼              │
┌─────────────┐       │
│    Clean    │       │
└──────┬──────┘       │
       │              │
       ▼              │
┌─────────────┐       │
│  Inspected  │───────┘
└─────────────┘
```

#### 4.1.2 Room Blocking

**Block Types**:
- **Out of Order (OOO)**: Room unusable due to maintenance
- **Out of Service (OOS)**: Room temporarily unavailable
- **House Use**: Staff or complimentary use
- **Do Not Rent**: Management restriction

**Block Management**:
```json
{
  "roomNumber": "305",
  "blockType": "out-of-order",
  "reason": "HVAC repair",
  "blockedFrom": "2025-01-15",
  "blockedUntil": "2025-01-17",
  "estimatedRevenueLoss": 500.00,
  "maintenanceTicket": "MNT-12345"
}
```

### 4.2 Room Assignment

#### 4.2.1 Auto-Assignment Algorithm

```javascript
function autoAssignRoom(reservation, availableRooms) {
  const preferences = reservation.guest.preferences;

  // Score each room
  const scoredRooms = availableRooms.map(room => {
    let score = 0;

    // Floor preference
    if (preferences.roomLocation === 'high-floor' && room.floor >= 5) {
      score += 10;
    } else if (preferences.roomLocation === 'low-floor' && room.floor <= 3) {
      score += 10;
    }

    // View preference
    if (preferences.view && room.view === preferences.view) {
      score += 15;
    }

    // Quiet area
    if (preferences.quiet && room.quietArea) {
      score += 10;
    }

    // Recent cleaning
    const hoursSinceCleaned = (Date.now() - room.lastCleaned) / (1000 * 60 * 60);
    if (hoursSinceCleaned < 2) {
      score += 5;
    }

    // VIP preference for better rooms
    if (reservation.guest.vipStatus === 'platinum') {
      score += room.floor * 2; // Higher floors for VIPs
    }

    return { room, score };
  });

  // Sort by score and return best match
  scoredRooms.sort((a, b) => b.score - a.score);
  return scoredRooms[0].room;
}
```

#### 4.2.2 Upgrade Logic

```javascript
function shouldOfferUpgrade(reservation, currentOccupancy) {
  // Upgrade criteria
  const isLoyaltyMember = reservation.guest.loyaltyMembership !== null;
  const isVIP = reservation.guest.vipStatus !== 'standard';
  const isLowOccupancy = currentOccupancy < 0.6;
  const isLongStay = reservation.nights >= 5;
  const isSpecialOccasion = reservation.specialRequests?.includes('honeymoon') ||
                            reservation.specialRequests?.includes('anniversary');

  // Upgrade priority score
  let upgradeScore = 0;
  if (isVIP) upgradeScore += 30;
  if (isLoyaltyMember) upgradeScore += 20;
  if (isLongStay) upgradeScore += 15;
  if (isSpecialOccasion) upgradeScore += 25;
  if (isLowOccupancy) upgradeScore += 10;

  // Offer upgrade if score > threshold
  return upgradeScore >= 40;
}
```

---


## Annex E — Implementation Notes for PHASE-1-DATA-FORMAT

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-1-DATA-FORMAT.

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
evidence for PHASE-1-DATA-FORMAT. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-1-data-format/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-1-DATA-FORMAT with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-1-DATA-FORMAT does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-1-DATA-FORMAT.
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
for PHASE-1-DATA-FORMAT. The profile mechanism is non-normative and exists so that
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
  advertises `WIA-P1-DATA-FORMAT-Edge-only/2` is asserting conformance with
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
  is published manually. Sufficient for PHASE-1-DATA-FORMAT validation when the
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
