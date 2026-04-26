# WIA-IND-009 PHASE 1 — Data Format Specification

**Standard:** WIA-IND-009
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 1 of 4)

---

# WIA-IND-009: Food Delivery Standard Specification v1.0

> **Standard ID:** WIA-IND-009
> **Title:** Food Delivery Standard
> **Version:** 1.0.0
> **Status:** Active
> **Category:** IND (Industry)
> **Color:** Indigo (#6366F1)
> **Authors:** WIA Food Delivery Working Group
> **Date:** 2025-01-15
> **License:** MIT

---

## Abstract

This specification defines a comprehensive standard for food delivery systems, encompassing order management, driver logistics, route optimization, temperature monitoring, quality assurance, and customer experience. The standard provides interoperability between restaurants, delivery platforms, drivers, and customers while ensuring food safety, efficiency, and reliability.

**弘益人間 (Benefit All Humanity)** - This standard aims to democratize food delivery technology, improve food accessibility, reduce waste, and ensure safe food transportation for all communities worldwide.

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Architecture](#2-architecture)
3. [Order Management](#3-order-management)
4. [Driver Management](#4-driver-management)
5. [Route Optimization](#5-route-optimization)
6. [Temperature Monitoring](#6-temperature-monitoring)
7. [Time Estimation](#7-time-estimation)
8. [Cost Calculation](#8-cost-calculation)
9. [Quality Assurance](#9-quality-assurance)
10. [Safety & Compliance](#10-safety--compliance)
11. [API Specification](#11-api-specification)
12. [Data Models](#12-data-models)
13. [Security](#13-security)
14. [Performance Requirements](#14-performance-requirements)
15. [Integration Guidelines](#15-integration-guidelines)

---

## 1. Introduction

### 1.1 Purpose

The WIA-IND-009 standard provides a unified framework for food delivery operations, enabling:
- Seamless integration between restaurants and delivery platforms
- Efficient driver assignment and routing
- Real-time order tracking and status updates
- Food safety compliance through temperature monitoring
- Performance optimization and analytics

### 1.2 Scope

This standard covers:
- **Order Lifecycle**: From placement to completion
- **Driver Operations**: Assignment, tracking, navigation
- **Route Optimization**: Single and multi-stop routing
- **Temperature Control**: Hot, cold, and frozen food handling
- **Quality Metrics**: Performance tracking and KPIs
- **Customer Experience**: Tracking, notifications, feedback

### 1.3 Terminology

| Term | Definition |
|------|------------|
| **Order** | Complete delivery request including items, locations, and requirements |
| **Driver** | Delivery personnel (employee, contractor, or gig worker) |
| **Restaurant** | Food preparation and pickup location |
| **Customer** | Delivery recipient |
| **Route** | Optimized path from pickup(s) to delivery location(s) |
| **Batch** | Multiple orders assigned to single driver trip |
| **ETA** | Estimated Time of Arrival |
| **Prep Time** | Food preparation duration at restaurant |
| **Transit Time** | Travel time from pickup to delivery |
| **Last Mile** | Final segment of delivery to customer |
| **Cold Chain** | Temperature-controlled supply chain |
| **HACCP** | Hazard Analysis Critical Control Points (food safety) |

### 1.4 Design Principles

1. **Food Safety First**: Temperature compliance and hygiene standards
2. **Driver Welfare**: Fair compensation and working conditions
3. **Customer Experience**: Transparency and reliability
4. **Efficiency**: Optimal routing and resource utilization
5. **Scalability**: Support from single restaurant to global platform
6. **Interoperability**: Standard APIs for ecosystem integration

---

## 2. Architecture

### 2.1 System Components

```
┌─────────────────────────────────────────────────────────────┐
│                    Customer Applications                     │
│              (Web, iOS, Android, Voice, Chat)               │
└────────────────────────┬────────────────────────────────────┘
                         │
┌────────────────────────▼────────────────────────────────────┐
│                   WIA-IND-009 API Gateway                    │
│  (Order Management, Tracking, Routing, Temperature)         │
└─────┬──────────┬──────────┬──────────┬──────────┬──────────┘
      │          │          │          │          │
┌─────▼────┐ ┌──▼──────┐ ┌─▼────────┐ ┌▼─────────┐ ┌▼────────┐
│ Order    │ │ Driver  │ │ Route    │ │ Temp     │ │ Payment │
│ Service  │ │ Service │ │ Optimizer│ │ Monitor  │ │ Service │
└─────┬────┘ └──┬──────┘ └─┬────────┘ └┬─────────┘ └┬────────┘
      │          │          │          │          │
┌─────▼──────────▼──────────▼──────────▼──────────▼──────────┐
│                   Data & Analytics Layer                     │
│     (PostgreSQL, Redis, TimescaleDB, Elasticsearch)         │
└────────────────────────┬────────────────────────────────────┘
                         │
┌────────────────────────▼────────────────────────────────────┐
│                  External Integrations                       │
│  (Restaurant POS, Maps, Weather, Traffic, IoT Sensors)      │
└─────────────────────────────────────────────────────────────┘
```

### 2.2 Data Flow

**Order Creation Flow:**
```
Customer → Order API → Validation → Restaurant Notification
→ Driver Assignment → Route Optimization → Driver App → Pickup
→ Transit → Delivery → Completion → Payment → Feedback
```

**Real-time Tracking Flow:**
```
Driver GPS → Location Service → Order Service → WebSocket
→ Customer App (Real-time updates every 10-30 seconds)
```

### 2.3 Technology Stack

**Required Components:**
- RESTful API with JSON payloads
- WebSocket for real-time tracking
- GPS/GNSS for location services
- PostgreSQL or compatible relational database
- Redis for caching and real-time data
- Message queue (Kafka, RabbitMQ, etc.)

**Optional Components:**
- TimescaleDB for time-series temperature data
- Elasticsearch for search and analytics
- GraphQL for flexible client queries
- IoT sensors for temperature monitoring

---

## 3. Order Management

### 3.1 Order Lifecycle States

```
PENDING → CONFIRMED → PREPARING → READY → ASSIGNED
→ PICKED_UP → IN_TRANSIT → ARRIVING → DELIVERED → COMPLETED

Alternative paths:
PENDING → REJECTED
* → CANCELLED (any state before PICKED_UP)
* → FAILED (after PICKED_UP, with refund)
```

### 3.2 Order Creation

**Minimum Required Fields:**
```json
{
  "restaurantId": "string (UUID)",
  "customerId": "string (UUID)",
  "items": [
    {
      "itemId": "string",
      "name": "string",
      "quantity": "integer (>0)",
      "price": "decimal (≥0)",
      "temperature": "hot|cold|ambient|frozen"
    }
  ],
  "pickupLocation": {
    "latitude": "decimal (-90 to 90)",
    "longitude": "decimal (-180 to 180)",
    "address": "string"
  },
  "deliveryLocation": {
    "latitude": "decimal",
    "longitude": "decimal",
    "address": "string",
    "instructions": "string (optional)"
  }
}
```

**Optional Fields:**
- `scheduledTime`: ISO 8601 timestamp for scheduled delivery
- `deliveryWindow`: Start and end time range
- `contactlessDelivery`: Boolean flag
- `utensils`: Boolean flag for including utensils
- `specialInstructions`: Free text field (max 500 chars)
- `promoCode`: Discount code
- `tip`: Pre-specified tip amount

### 3.3 Order Validation Rules

**Restaurant Validation:**
1. Restaurant must be active and accepting orders
2. Restaurant must be within service area
3. Restaurant operating hours must cover delivery time
4. Minimum order value must be met (if applicable)

**Customer Validation:**
1. Delivery address must be within service radius
2. Customer must have valid payment method
3. Customer account must be in good standing

**Item Validation:**
1. All items must be available in restaurant menu
2. Item prices must match current menu prices
3. Quantities must be within allowed limits
4. Item modifications must be valid options

**Logistics Validation:**
1. Estimated delivery time must be feasible
2. Driver availability for estimated time
3. Temperature requirements must be compatible
4. Distance must be within maximum delivery range

### 3.4 Order Assignment

**Driver Assignment Algorithm:**

```python
def assign_driver(order):
    # 1. Filter available drivers
    available_drivers = get_available_drivers(
        location=order.pickup_location,
        radius=10_km,
        vehicle_type=order.required_vehicle
    )

    # 2. Score each driver
    for driver in available_drivers:
        score = calculate_driver_score(
            distance_to_pickup=haversine(driver.location, order.pickup_location),
            driver_rating=driver.avg_rating,
            completion_rate=driver.completion_rate,
            current_batch_size=len(driver.active_orders),
            temperature_capability=driver.equipment.temperature_control
        )
        driver.assignment_score = score

    # 3. Sort by score (descending)
    drivers_ranked = sorted(available_drivers, key=lambda d: d.assignment_score, reverse=True)

    # 4. Assign to highest scored driver
    if drivers_ranked:
        return drivers_ranked[0]
    else:
        return None  # Enter queue for next available driver
```

**Scoring Factors:**
- Distance to pickup: 40% weight
- Driver rating: 25% weight
- Completion rate: 20% weight
- Current load: 10% weight
- Equipment capability: 5% weight

### 3.5 Batching Strategy

**Batching Criteria:**
```
Can batch orders A and B if:
1. |pickup_A.location - pickup_B.location| < 1 km
2. |delivery_A.location - delivery_B.location| < 2 km
3. |expected_time_A - expected_time_B| < 15 minutes
4. temperature_requirements(A) == temperature_requirements(B)
5. total_batch_size ≤ 4 orders
6. total_batch_weight ≤ driver.capacity
```

**Batch Optimization:**
```
Maximize: (Orders in batch) / (Total delivery time)
Subject to:
- Each order delivered within promised window
- Temperature maintained for all items
- Total weight ≤ vehicle capacity
- Total volume ≤ container capacity
```

---

## 4. Driver Management

### 4.1 Driver States

```
OFFLINE → ONLINE → AVAILABLE → ASSIGNED → EN_ROUTE_TO_PICKUP
→ AT_RESTAURANT → PICKING_UP → LOADED → IN_TRANSIT
→ ARRIVING → DELIVERING → COMPLETED → AVAILABLE
```

### 4.2 Driver Profile

**Required Information:**
```json
{
  "driverId": "string (UUID)",
  "name": "string",
  "phone": "string (E.164 format)",
  "email": "string",
  "vehicleType": "bike|ebike|scooter|motorcycle|car",
  "licensePlate": "string",
  "rating": "decimal (0-5)",
  "completionRate": "decimal (0-1)",
  "equipment": {
    "hotBag": "boolean",
    "coldBag": "boolean",
    "temperatureSensor": "boolean",
    "smartphoneModel": "string"
  },
  "location": {
    "latitude": "decimal",
    "longitude": "decimal",
    "accuracy": "decimal (meters)",
    "timestamp": "ISO 8601"
  },
  "status": "string (driver state)"
}
```

### 4.3 Location Tracking

**Update Frequency:**
- `AVAILABLE`: Every 60 seconds
- `EN_ROUTE_TO_PICKUP`: Every 30 seconds
- `IN_TRANSIT`: Every 10 seconds
- `ARRIVING`: Every 5 seconds

**Location Accuracy Requirements:**
- Minimum accuracy: 50 meters
- Preferred accuracy: 10 meters
- Urban areas: GPS + WiFi triangulation
- Indoor (restaurants): WiFi or Bluetooth beacons

**Privacy Considerations:**
- Location tracking only when driver is online
- Historical location data retention: 30 days
- Anonymization for analytics after 90 days

### 4.4 Driver Performance Metrics

**Core KPIs:**
```typescript
interface DriverMetrics {
  // Efficiency
  ordersPerHour: number;              // Target: 2-3
  avgDeliveryTime: number;            // Minutes
  avgDistancePerOrder: number;        // Kilometers
  utilizationRate: number;            // Active time / Online time

  // Quality
  onTimeDeliveryRate: number;         // Target: >90%
  customerRating: number;             // Target: >4.5/5
  orderAccuracy: number;              // Target: >99%
  temperatureCompliance: number;      // Target: >95%

  // Reliability
  completionRate: number;             // Target: >98%
  cancellationRate: number;           // Target: <2%
  responseTime: number;               // Seconds to accept order

  // Earnings
  totalEarnings: number;              // Currency
  avgEarningsPerHour: number;         // Currency
  avgEarningsPerDelivery: number;     // Currency
}
```

**Performance Tiers:**
```
Bronze: <100 deliveries, rating >4.0
Silver: 100-500 deliveries, rating >4.3, on-time >85%
Gold: 500-2000 deliveries, rating >4.6, on-time >90%
Platinum: >2000 deliveries, rating >4.8, on-time >95%
```

### 4.5 Driver Compensation

**Base Pay Structure:**
```
Earnings = Base Fee + Distance Pay + Time Pay + Tips + Bonuses - Fees

Base Fee = $2.50 - $5.00 per delivery
Distance Pay = $0.50 - $1.50 per km
Time Pay = $0.10 - $0.30 per minute (active time)
Peak Hour Bonus = 1.2x - 2.0x base pay
Quest Bonuses = Additional for completing X orders in timeframe
```

**Expense Considerations:**
- Fuel/electricity: ~$0.15-0.30 per km
- Vehicle maintenance: ~$0.05-0.10 per km
- Insurance: ~$100-300 per month
- Equipment: ~$50-200 one-time + $20/month replacement

---

## 5. Route Optimization

### 5.1 Single-Stop Routing

**Objective:** Minimize delivery time from pickup to dropoff

**Algorithm: Dijkstra's Shortest Path with Time Weights**

```python
def calculate_single_route(pickup, delivery, current_time):
    """
    Calculate optimal route considering:
    - Real-time traffic
    - Road restrictions
    - Weather conditions
    - Historical patterns
    """

    # Get road network graph
    graph = get_road_network(
        bounds=bounding_box(pickup, delivery)
    )

    # Apply time-dependent weights
    for edge in graph.edges:
        base_time = edge.distance / speed_limit(edge)
        traffic_factor = get_traffic_factor(edge, current_time)
        weather_factor = get_weather_factor(edge, current_time)

        edge.weight = base_time * traffic_factor * weather_factor

    # Find shortest path
    path = dijkstra(graph, pickup, delivery)

    # Calculate ETA
    total_time = sum(edge.weight for edge in path)
    eta = current_time + timedelta(seconds=total_time)

    return {
        'path': path,
        'distance': sum(edge.distance for edge in path),
        'duration': total_time,
        'eta': eta,
        'waypoints': get_turn_by_turn_directions(path)
    }
```

### 5.2 Multi-Stop Routing


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
