# WIA-AUTO-016 — Phase 2: API Interface

> Smart-logistics canonical Phase 2: API surface (fleet + REST + GraphQL + WebSocket).

# WIA-AUTO-016: Smart Logistics Specification v1.0

> **Standard ID:** WIA-AUTO-016
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Automotive & Logistics Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Warehouse Automation](#2-warehouse-automation)
3. [Fleet Management](#3-fleet-management)
4. [Route Optimization Algorithms](#4-route-optimization-algorithms)
5. [Real-time Tracking](#5-real-time-tracking)
6. [Last-mile Delivery](#6-last-mile-delivery)
7. [Inventory Management](#7-inventory-management)
8. [Data Formats](#8-data-formats)
9. [API Interface](#9-api-interface)
10. [Integration Protocols](#10-integration-protocols)
11. [References](#11-references)

---


## 3. Fleet Management

### 3.1 Vehicle Tracking

#### 3.1.1 GPS Telemetry

**Data Collection Requirements**:
- Position accuracy: ±3 meters
- Update frequency: 10-30 seconds
- Data retention: 90 days minimum
- Offline buffering: 24 hours

**Telemetry Data Points**:
```json
{
  "timestamp": "2025-12-26T10:30:00Z",
  "vehicleId": "VH-001",
  "position": {
    "lat": 37.7749,
    "lng": -122.4194,
    "altitude": 15.5,
    "accuracy": 2.3
  },
  "status": {
    "speed": 55.2,
    "heading": 270,
    "odometer": 125430.5,
    "fuelLevel": 0.65,
    "engineStatus": "running"
  },
  "driver": {
    "id": "DR-042",
    "hoursOfService": 7.5,
    "breakRequired": false
  }
}
```

#### 3.1.2 Geofencing

**Geofence Definition**:
```
Point-in-Polygon Test:
inside = false
for each edge (p1, p2) in polygon:
  if (p1.y > point.y) != (p2.y > point.y):
    if point.x < (p2.x - p1.x) × (point.y - p1.y) / (p2.y - p1.y) + p1.x:
      inside = !inside
```

**Geofence Events**:
- Entry: Vehicle enters defined zone
- Exit: Vehicle leaves defined zone
- Dwell: Vehicle remains in zone beyond threshold
- Violation: Unauthorized zone access

### 3.2 Maintenance Management

#### 3.2.1 Predictive Maintenance

**Failure Prediction Model**:
```
P(failure | X) = 1 / (1 + e^(-β₀ - Σβᵢxᵢ))
```

Where:
- `X` = [mileage, engine_hours, vibration, temperature, oil_quality]
- `βᵢ` = Learned coefficients

**Maintenance Triggers**:
- Time-based: Every N days
- Mileage-based: Every N kilometers
- Condition-based: Sensor thresholds exceeded
- Predictive: ML model predicts failure probability > 0.7

#### 3.2.2 Maintenance Scheduling

```
Minimize: C = Σ(downtime_cost(i) + maintenance_cost(i))

Subject to:
- Vehicle availability: available(v, t) for scheduled routes
- Technician capacity: assigned_jobs(tech, t) ≤ capacity(tech)
- Parts availability: parts_in_stock ≥ parts_required
- Urgency: critical_issues handled within 24h
```

### 3.3 Driver Management

#### 3.3.1 Hours of Service (HOS) Compliance

**FMCSA Rules (USA)**:
- Driving limit: 11 hours after 10 consecutive hours off duty
- On-duty limit: 14 consecutive hours
- Rest break: 30 minutes after 8 cumulative hours
- Weekly limit: 60 hours in 7 days or 70 hours in 8 days
- Restart: 34 consecutive hours off duty

**Automatic Tracking**:
```
if current_driving_hours >= 11 OR current_shift_hours >= 14:
  require_rest = true
  alert_driver("HOS limit approaching")

if continuous_driving >= 8:
  require_break = true
  minimum_break_duration = 30 minutes
```

#### 3.3.2 Driver Performance Metrics

**Safety Score**:
```
SS = w₁·(1 - accidents/miles) + w₂·(1 - violations/miles) + w₃·smooth_driving
```

Where:
- `w₁, w₂, w₃` = Weighting factors (sum to 1)
- `smooth_driving` = Score from acceleration/braking analysis

**Efficiency Score**:
```
ES = (actual_deliveries / planned_deliveries) × (1 - delay_ratio) × fuel_efficiency
```

---



## 9. API Interface

### 9.1 RESTful API Endpoints

#### 9.1.1 Shipment Management

**Create Shipment**:
```
POST /api/v1/shipments
Content-Type: application/json
Authorization: Bearer {token}

Request:
{
  "origin": {"facilityId": "WH-SF-001"},
  "destination": {
    "name": "John Doe",
    "address": "456 Residential St, Oakland, CA 94601",
    "phone": "+1-555-0123"
  },
  "packages": [
    {
      "dimensions": {"length": 30, "width": 20, "height": 15, "unit": "cm"},
      "weight": {"value": 2.5, "unit": "kg"}
    }
  ],
  "service": "standard"
}

Response (201 Created):
{
  "shipmentId": "WIA-SHIP-20250126-001",
  "trackingNumber": "1Z999AA10123456784",
  "status": "created",
  "estimatedDelivery": "2025-01-27T16:00:00Z",
  "label": "https://api.wia.com/labels/WIA-SHIP-20250126-001.pdf"
}
```

**Track Shipment**:
```
GET /api/v1/shipments/{shipmentId}/tracking
Authorization: Bearer {token}

Response (200 OK):
{
  "shipmentId": "WIA-SHIP-20250126-001",
  "status": "in_transit",
  "currentLocation": {
    "lat": 37.8044,
    "lng": -122.2712,
    "facility": "HUB-OAK-001",
    "timestamp": "2025-01-26T15:00:00Z"
  },
  "estimatedDelivery": "2025-01-27T16:00:00Z",
  "events": [...]
}
```

#### 9.1.2 Route Optimization

**Optimize Route**:
```
POST /api/v1/routes/optimize
Content-Type: application/json
Authorization: Bearer {token}

Request:
{
  "origin": {"lat": 37.7749, "lng": -122.4194},
  "destinations": [
    {"lat": 37.8044, "lng": -122.2712, "timeWindow": {"start": "10:00", "end": "12:00"}},
    {"lat": 37.6879, "lng": -122.4702, "timeWindow": {"start": "13:00", "end": "17:00"}}
  ],
  "vehicle": {
    "type": "truck",
    "capacity": 5000,
    "currentLoad": 1200
  },
  "constraints": {
    "maxDistance": 200,
    "maxDuration": 480,
    "trafficAware": true
  },
  "optimization": {
    "goal": "minimize_cost",
    "weights": {"distance": 0.4, "time": 0.4, "emissions": 0.2}
  }
}

Response (200 OK):
{
  "routeId": "RT-20250126-042",
  "optimizedRoute": {
    "stops": [
      {"sequence": 1, "location": {...}, "eta": "2025-01-26T10:15:00Z"},
      {"sequence": 2, "location": {...}, "eta": "2025-01-26T13:30:00Z"}
    ],
    "totalDistance": 45.2,
    "totalTime": 180,
    "estimatedCost": 85.50,
    "estimatedEmissions": 12.3
  },
  "alternatives": [...]
}
```

#### 9.1.3 Warehouse Operations

**Get Inventory**:
```
GET /api/v1/warehouses/{warehouseId}/inventory?sku={sku}
Authorization: Bearer {token}

Response (200 OK):
{
  "warehouseId": "WH-SF-001",
  "sku": "PROD-123",
  "description": "Widget",
  "quantity": 450,
  "location": "A-12-03-02",
  "status": "available",
  "reorderPoint": 100,
  "economicOrderQuantity": 500,
  "lastCounted": "2025-01-25T14:00:00Z",
  "nextReorder": "2025-02-05"
}
```

**Create Pick List**:
```
POST /api/v1/warehouses/{warehouseId}/pick-lists
Content-Type: application/json
Authorization: Bearer {token}

Request:
{
  "orderId": "ORD-123456",
  "priority": "standard",
  "items": [
    {"sku": "PROD-123", "quantity": 2},
    {"sku": "PROD-456", "quantity": 1}
  ]
}

Response (201 Created):
{
  "pickListId": "PL-20250126-001",
  "orderId": "ORD-123456",
  "status": "pending",
  "picks": [
    {"lineNumber": 1, "sku": "PROD-123", "location": "A-12-03-02", "quantity": 2},
    {"lineNumber": 2, "sku": "PROD-456", "location": "B-05-02-01", "quantity": 1}
  ],
  "estimatedCompletionTime": "2025-01-26T08:45:00Z"
}
```

### 9.2 WebSocket API

**Real-time Tracking**:
```
WS /api/v1/tracking/stream
Authorization: Bearer {token}

Client → Server:
{
  "action": "subscribe",
  "shipmentIds": ["WIA-SHIP-20250126-001", "WIA-SHIP-20250126-002"]
}

Server → Client (real-time updates):
{
  "type": "location_update",
  "shipmentId": "WIA-SHIP-20250126-001",
  "location": {"lat": 37.8100, "lng": -122.2700},
  "timestamp": "2025-01-26T15:05:00Z",
  "speed": 45.2,
  "heading": 270
}

{
  "type": "status_change",
  "shipmentId": "WIA-SHIP-20250126-001",
  "oldStatus": "in_transit",
  "newStatus": "out_for_delivery",
  "timestamp": "2025-01-26T16:00:00Z"
}
```

### 9.3 GraphQL API

```graphql
type Query {
  shipment(id: ID!): Shipment
  route(id: ID!): Route
  warehouse(id: ID!): Warehouse
  inventory(warehouseId: ID!, sku: String!): InventoryItem
}

type Mutation {
  createShipment(input: CreateShipmentInput!): Shipment
  optimizeRoute(input: OptimizeRouteInput!): Route
  updateInventory(input: UpdateInventoryInput!): InventoryItem
}

type Subscription {
  shipmentUpdated(id: ID!): Shipment
  routeProgress(id: ID!): RouteProgress
}

type Shipment {
  id: ID!
  trackingNumber: String!
  status: ShipmentStatus!
  origin: Location!
  destination: Location!
  packages: [Package!]!
  currentLocation: GeoLocation
  estimatedDelivery: DateTime
  events: [ShipmentEvent!]!
}

type Route {
  id: ID!
  vehicle: Vehicle!
  driver: Driver!
  stops: [Stop!]!
  metrics: RouteMetrics!
  optimization: OptimizationDetails!
}
```

**Example Query**:
```graphql
query GetShipmentDetails($id: ID!) {
  shipment(id: $id) {
    trackingNumber
    status
    currentLocation {
      lat
      lng
      timestamp
    }
    estimatedDelivery
    events {
      timestamp
      status
      location
      description
    }
  }
}
```

---




---

## A.1 Endpoint reference

```http
POST /logistics/v1/shipments                      # create shipment
GET  /logistics/v1/shipments/{id}/tracking        # track shipment
POST /logistics/v1/routes/optimize                # optimize route
GET  /logistics/v1/warehouses/{id}/inventory      # query inventory
POST /logistics/v1/warehouses/{id}/pick-lists     # create pick list
WS   /logistics/v1/tracking/stream                # real-time tracking
```

Every endpoint follows the discovery convention at `/.well-known/wia-smart-logistics`.

## A.2 Shipment lifecycle

`POST /shipments` creates the shipment in `created` state and returns a label URL. State transitions follow the canonical lifecycle: created → label_created → picked_up → in_transit → out_for_delivery → delivered (or → exception → resolved → in_transit). Every transition emits a `shipment.{state}` event on the WebSocket and queues a webhook delivery for subscribed consumers.

## A.3 Route optimization request shape

`POST /routes/optimize` accepts a vehicle profile, a list of destinations with optional time windows, and an optimization objective (`minimize_cost`, `minimize_time`, `minimize_emissions`, `balanced`). The response carries the optimized stop sequence, total distance, total time, estimated cost, estimated emissions, and up to three alternatives. Re-optimization is idempotent if the input set has not changed; otherwise a new `routeId` is issued.

## A.4 Real-time tracking WebSocket

The WebSocket carries `location_update`, `status_change`, `eta_update`, and `exception` events. Clients subscribe per `shipmentId`; the server multiplexes the underlying telemetry stream so a single client cannot exhaust the carrier-side bandwidth. Backpressure is communicated via `429 Too Many Requests` close codes and `Retry-After` hints.

## A.5 GraphQL surface

The GraphQL surface mirrors the REST endpoints but exposes them in a single relay-paginated graph. Queries support shipment ↔ route ↔ warehouse joins; mutations cover create/optimize/update; subscriptions cover shipment progress and route progress. The schema is published at `https://wiastandards.com/smart-logistics/graphql/schema.graphql` and is versioned with the standard.

## A.6 Rate-limit and quota envelope

Standard rate limits: 1000 req/h unauthenticated, 5000 req/h authenticated, 10000 req/h for premium tier, with token-bucket refill. Quota envelopes are returned in the `X-RateLimit-*` headers; integrators MUST honour the headers per IETF RFC 6585 and the Retry-After header per RFC 7231.


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/smart-logistics/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-smart-logistics-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/smart-logistics-host:1.0.0` ships every smart-logistics envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/smart-logistics.sh` ships sample envelope generators with no
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
ecosystem. Smart-logistics deployments that follow this layering
inherit the per-Phase conformance gates and the cross-standard
composition behaviour described in Z.2.

弘益人間 — Benefit All Humanity.
