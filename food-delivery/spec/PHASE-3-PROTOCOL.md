# WIA-IND-009 PHASE 3 — Protocol Specification

**Standard:** WIA-IND-009
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 3 of 4)

---

// Customer-facing
  subtotal: number;           // Food items total
  deliveryFee: number;        // Base + distance + time
  serviceFee: number;         // Platform commission
  smallOrderFee: number;      // If below minimum
  tax: number;                // Sales tax
  tip: number;                // Customer tip
  discount: number;           // Promo code discount
  total: number;              // Grand total

  // Internal breakdown
  driverPay: number;          // Amount to driver
  restaurantPayout: number;   // Amount to restaurant
  platformRevenue: number;    // Platform earnings

  // Surge
  surgeMultiplier: number;    // 1.0 - 3.0
  isSurge: boolean;
}
```

**Example Calculation:**
```
Order Subtotal: $25.00
Delivery Fee:   $4.99 (2 km, 15 min)
Service Fee:    $3.75 (15% of subtotal)
Tax:            $2.69 (9% of subtotal + fees)
Tip:            $5.00 (20%)
Discount:       -$3.00 (promo code)
----------------------------
Total:          $38.43

Breakdown:
- To Driver:    $9.99 ($4.99 delivery + $5.00 tip)
- To Restaurant: $25.00
- To Platform:  $3.44 ($3.75 service - $0.31 payment processing)
```

---

## 9. Quality Assurance

### 9.1 Quality Metrics

**Order Accuracy:**
```
Accuracy = (Correct Orders / Total Orders) × 100%

Correct Order = All items present AND correct items AND no damage

Target: >99%
```

**Customer Satisfaction:**
```
Satisfaction Score = Average(Rating_Food, Rating_Delivery, Rating_Driver)

Rating scale: 1-5 stars
Target: >4.5/5
```

**Freshness Score:**
```python
def calculate_freshness_score(order):
    """
    Freshness based on temperature compliance and time
    """

    temp_compliance = (
        time_in_safe_range / total_transit_time
    )

    time_factor = max(0, 1 - (transit_time / max_recommended_time))

    freshness = (0.6 * temp_compliance) + (0.4 * time_factor)

    return freshness * 100  # 0-100 score
```

### 9.2 Quality Control Checkpoints

**Restaurant Checkpoint:**
```
- Photo of packaged order (optional but recommended)
- Verify all items present
- Seal bag for tampering prevention
- Add temperature indicator sticker
- Timestamp of handoff
```

**Driver Checkpoint:**
```
- Confirm items match order
- Check temperature of bag
- Verify container is properly sealed
- Note any concerns or special handling
```

**Delivery Checkpoint:**
```
- Photo proof of delivery
- Customer signature (if required)
- Contactless confirmation
- Note delivery location
- Verify customer received order
```

### 9.3 Feedback System

**Rating Collection:**
```typescript
interface OrderRating {
  orderId: string;
  customerId: string;

  // Ratings (1-5 stars)
  foodQuality: number;
  foodTemperature: number;
  packaging: number;
  deliverySpeed: number;
  driverProfessionalism: number;
  communication: number;
  overallExperience: number;

  // Feedback
  comments: string;
  wouldRecommend: boolean;

  // Issues (optional)
  issues?: string[];  // ['missing_item', 'cold_food', 'late_delivery']
  photos?: string[];  // Evidence photos
}
```

**Automated Quality Flags:**
```python
def check_quality_flags(order):
    """
    Automatically flag orders with potential issues
    """

    flags = []

    # Temperature violation
    if order.temperature_violations > 0:
        flags.append('temperature_violation')

    # Excessive delay
    if order.actual_time > order.promised_time * 1.5:
        flags.append('excessive_delay')

    # Multiple re-routes
    if order.reroute_count > 3:
        flags.append('routing_issues')

    # Driver deviation
    if order.route_deviation_distance > 2_km:
        flags.append('route_deviation')

    # Low rating pattern
    if order.restaurant.recent_rating < 4.0:
        flags.append('restaurant_quality_concern')

    return flags
```

---

## 10. Safety & Compliance

### 10.1 Food Safety Standards

**HACCP Principles:**
1. **Hazard Analysis**: Identify biological, chemical, physical hazards
2. **Critical Control Points**: Temperature, time, contamination prevention
3. **Critical Limits**: Temperature thresholds, time limits
4. **Monitoring**: Continuous temperature tracking
5. **Corrective Actions**: Alerts, order rejection, refunds
6. **Verification**: Audits, sensor calibration
7. **Record-keeping**: 90-day temperature logs

**FDA Food Code Compliance:**
```
Hot Food: Maintain ≥135°F (57°C) during transport
Cold Food: Maintain ≤41°F (5°C) during transport
Time Limit: <4 hours from cooking to consumption
Contamination: Sealed containers, no bare-hand contact
```

### 10.2 Driver Safety

**Background Checks:**
- Criminal background check (state/national)
- Driving record check (MVR)
- Identity verification
- Right to work verification

**Training Requirements:**
- Food safety certification
- Defensive driving course
- Customer service training
- App and equipment usage
- Emergency procedures

**Safety Equipment:**
- Reflective vest (for bike/scooter drivers)
- Helmet (required for 2-wheeled vehicles)
- First aid kit
- Phone mount for navigation
- Flashlight for night deliveries

**Incident Reporting:**
```typescript
interface SafetyIncident {
  incidentId: string;
  driverId: string;
  orderId?: string;
  timestamp: Date;
  type: 'accident' | 'theft' | 'harassment' | 'injury' | 'other';
  severity: 'minor' | 'moderate' | 'severe' | 'critical';
  description: string;
  location: GeoLocation;
  policeReport?: string;
  witnesses?: string[];
  photos?: string[];
  status: 'reported' | 'investigating' | 'resolved' | 'closed';
}
```

### 10.3 Data Privacy

**GDPR/CCPA Compliance:**
```
Personal Data Collection:
- Name, phone, email, address (required for delivery)
- Payment information (PCI-DSS compliant)
- Location data (only when app active)
- Order history (for personalization)

User Rights:
- Right to access data
- Right to delete account and data
- Right to port data
- Right to opt-out of marketing

Data Retention:
- Active users: Indefinitely
- Inactive users (>2 years): Anonymization
- Deleted accounts: 30-day grace period, then permanent deletion
```

**Location Data Privacy:**
```
Driver Location:
- Shared with customer only for active order
- Aggregated/anonymized for analytics
- Not sold to third parties
- Retained for 30 days maximum

Customer Location:
- Encrypted at rest and in transit
- Access limited to need-to-know
- Not shared with drivers after delivery
```

### 10.4 Insurance Requirements

**Platform Insurance:**
- General liability: $1M per occurrence
- Auto liability: $1M per accident
- Cyber liability: $5M coverage
- Workers' compensation: As required by jurisdiction

**Driver Insurance:**
```
Personal Auto Policy: Minimum state requirements
Commercial Policy: $1M liability (if using personal vehicle)
Occupational Accident: $1M coverage
Uninsured Motorist: $100K/$300K
```

---

## 11. API Specification

### 11.1 RESTful Endpoints

**Base URL:** `https://api.wia-ind-009.com/v1`

**Authentication:** Bearer token (JWT)

```
POST   /orders                    Create new order
GET    /orders/:id                Get order details
GET    /orders                    List orders (with filters)
PATCH  /orders/:id                Update order
DELETE /orders/:id                Cancel order

GET    /orders/:id/tracking       Real-time tracking
GET    /orders/:id/temperature    Temperature history

POST   /drivers                   Register driver
GET    /drivers/:id               Get driver profile
PATCH  /drivers/:id               Update driver profile
GET    /drivers/:id/metrics       Driver performance metrics

POST   /routes/optimize           Optimize multi-stop route
GET    /routes/:id                Get route details

POST   /ratings                   Submit rating/feedback
GET    /ratings/:orderId          Get order ratings

GET    /analytics/dashboard       Platform analytics
GET    /analytics/reports         Generate reports
```

### 11.2 WebSocket Events

**Connection:** `wss://ws.wia-ind-009.com/tracking`

**Client → Server:**
```json
{
  "action": "subscribe",
  "orderId": "order_123",
  "userId": "user_456"
}
```

**Server → Client:**
```json
{
  "event": "location_update",
  "orderId": "order_123",
  "timestamp": "2025-01-15T14:30:00Z",
  "data": {
    "driver": {
      "location": {"lat": 37.7749, "lng": -122.4194},
      "heading": 45,
      "speed": 25
    },
    "eta": "2025-01-15T14:45:00Z",
    "distance": 2.5,
    "status": "in_transit"
  }
}
```

### 11.3 Webhook Notifications

**Webhook Events:**
```
order.created
order.confirmed
order.preparing
order.ready
order.picked_up
order.in_transit
order.delivered
order.cancelled
order.temperature_alert
order.delayed
driver.assigned
driver.arrived
```

**Webhook Payload:**
```json
{
  "event": "order.delivered",
  "timestamp": "2025-01-15T15:00:00Z",
  "data": {
    "orderId": "order_123",
    "status": "delivered",
    "deliveryTime": "2025-01-15T15:00:00Z",
    "photo": "https://cdn.example.com/proof_123.jpg"
  },
  "signature": "sha256_hmac_signature"
}
```

### 11.4 Rate Limiting

```
Authenticated Requests:
- Standard tier: 100 requests/minute
- Premium tier: 1000 requests/minute
- Enterprise tier: 10000 requests/minute

WebSocket Connections:
- Max 10 concurrent connections per user
- Max 100 subscriptions per connection

Burst Allowance:
- 2x rate for up to 10 seconds
- Then throttled to standard rate
```

---

## 12. Data Models

### 12.1 Order Entity

```typescript
interface Order {
  // Identity
  id: string;                    // UUID
  externalId?: string;           // Partner system ID

  // Parties
  restaurantId: string;
  customerId: string;
  driverId?: string;

  // Items
  items: OrderItem[];
  subtotal: number;
  tax: number;
  tip: number;
  total: number;

  // Locations
  pickupLocation: Location;
  deliveryLocation: Location;

  // Timing
  createdAt: Date;
  confirmedAt?: Date;
  preparedAt?: Date;
  pickedUpAt?: Date;
  deliveredAt?: Date;
  estimatedDelivery: Date;
  actualDelivery?: Date;

  // Status
  status: OrderStatus;
  statusHistory: StatusChange[];

  // Logistics
  route?: Route;
  temperature?: TemperatureLog[];
  distance: number;              // km
  duration: number;              // minutes

  // Preferences
  contactlessDelivery: boolean;
  specialInstructions?: string;

  // Quality
  rating?: OrderRating;
  issues?: string[];

  // Metadata
  metadata?: Record<string, any>;
}

interface OrderItem {
  id: string;
  name: string;
  quantity: number;
  price: number;
  temperature: 'hot' | 'cold' | 'ambient' | 'frozen';


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
