# WIA-IND-009 PHASE 4 — Integration Specification

**Standard:** WIA-IND-009
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 4 of 4)

---

modifiers?: string[];
  specialRequests?: string;
}

type OrderStatus =
  | 'pending'
  | 'confirmed'
  | 'preparing'
  | 'ready'
  | 'assigned'
  | 'picked_up'
  | 'in_transit'
  | 'arriving'
  | 'delivered'
  | 'completed'
  | 'cancelled'
  | 'failed';
```

### 12.2 Driver Entity

```typescript
interface Driver {
  // Identity
  id: string;
  externalId?: string;

  // Personal
  firstName: string;
  lastName: string;
  email: string;
  phone: string;
  photo?: string;

  // Vehicle
  vehicleType: VehicleType;
  licensePlate?: string;
  vehicleModel?: string;
  vehicleYear?: number;

  // Status
  status: DriverStatus;
  isOnline: boolean;
  location: Location;
  lastLocationUpdate: Date;

  // Performance
  rating: number;
  totalDeliveries: number;
  completionRate: number;
  onTimeRate: number;

  // Equipment
  equipment: {
    hasHotBag: boolean;
    hasColdBag: boolean;
    hasTemperatureSensor: boolean;
    hasInsulatedContainer: boolean;
  };

  // Capacity
  maxOrders: number;
  currentOrders: string[];       // Order IDs

  // Financials
  earnings: {
    today: number;
    week: number;
    month: number;
    allTime: number;
  };

  // Verification
  backgroundCheckStatus: 'pending' | 'approved' | 'rejected';
  licenseVerified: boolean;
  insuranceVerified: boolean;

  // Metadata
  createdAt: Date;
  lastActiveAt: Date;
  metadata?: Record<string, any>;
}

type VehicleType =
  | 'bike'
  | 'ebike'
  | 'scooter'
  | 'motorcycle'
  | 'car';

type DriverStatus =
  | 'offline'
  | 'online'
  | 'available'
  | 'assigned'
  | 'picking_up'
  | 'in_transit'
  | 'delivering';
```

### 12.3 Route Entity

```typescript
interface Route {
  id: string;
  driverId: string;
  orders: string[];              // Order IDs

  // Route details
  stops: RouteStop[];
  totalDistance: number;         // km
  totalDuration: number;         // minutes
  optimizationAlgorithm: string;

  // Waypoints
  waypoints: GeoPoint[];
  encodedPolyline?: string;      // Google polyline encoding

  // Performance
  estimatedCost: number;
  fuelConsumption?: number;
  co2Emissions?: number;

  // Status
  status: 'planned' | 'active' | 'completed' | 'cancelled';
  startedAt?: Date;
  completedAt?: Date;

  // Deviations
  deviations: RouteDeviation[];
}

interface RouteStop {
  sequence: number;
  type: 'pickup' | 'delivery';
  orderId: string;
  location: Location;
  arrivalTime: Date;
  departureTime?: Date;
  duration: number;              // minutes
  completed: boolean;
}

interface RouteDeviation {
  timestamp: Date;
  location: Location;
  reason: string;
  distanceOff: number;           // meters
  timeImpact: number;            // minutes
}
```

---

## 13. Security

### 13.1 Authentication & Authorization

**JWT Token Structure:**
```json
{
  "sub": "user_123",
  "role": "customer|driver|restaurant|admin",
  "permissions": ["order:create", "order:read", "order:update"],
  "exp": 1642262400,
  "iat": 1642176000
}
```

**Role-Based Access Control (RBAC):**
```
Customer:
- Create orders
- Track own orders
- Rate deliveries
- Manage payment methods

Driver:
- Accept orders
- Update order status
- Update location
- View assigned orders

Restaurant:
- Receive orders
- Update prep status
- Manage menu
- View analytics

Admin:
- Full access
- Platform configuration
- User management
- Analytics and reports
```

### 13.2 Data Encryption

**Encryption at Rest:**
- AES-256 for database
- Customer PII encrypted with separate keys
- Payment data: PCI-DSS Level 1 compliant

**Encryption in Transit:**
- TLS 1.3 for all API connections
- Certificate pinning for mobile apps
- WebSocket: WSS (secure WebSocket)

### 13.3 API Security

**Request Signing:**
```
X-WIA-Signature: HMAC-SHA256(secret, timestamp + method + path + body)
X-WIA-Timestamp: Unix timestamp (reject if >5 min old)
```

**Input Validation:**
```python
def validate_order_input(data):
    """
    Validate all inputs to prevent injection attacks
    """

    # Required fields
    assert 'restaurantId' in data
    assert 'customerId' in data
    assert 'items' in data and len(data['items']) > 0

    # Type checking
    assert isinstance(data['items'], list)
    assert all(isinstance(item['quantity'], int) for item in data['items'])

    # Range checking
    assert all(item['quantity'] > 0 for item in data['items'])
    assert all(item['price'] >= 0 for item in data['items'])

    # String sanitization
    for item in data['items']:
        item['name'] = sanitize_html(item['name'])
        if 'specialRequests' in item:
            item['specialRequests'] = sanitize_html(item['specialRequests'])

    # Geo validation
    assert -90 <= data['deliveryLocation']['latitude'] <= 90
    assert -180 <= data['deliveryLocation']['longitude'] <= 180

    return True
```

**Rate Limiting:**
```
Implement exponential backoff for repeated failures:
1st failure: No delay
2nd failure: 1 second
3rd failure: 2 seconds
4th failure: 4 seconds
...
Max delay: 60 seconds
```

---

## 14. Performance Requirements

### 14.1 Response Time SLAs

```
API Endpoint              | P50  | P95   | P99
--------------------------|------|-------|-------
GET /orders/:id          | 50ms | 100ms | 200ms
POST /orders             | 100ms| 300ms | 500ms
GET /orders/:id/tracking | 50ms | 100ms | 150ms
POST /routes/optimize    | 500ms| 2s    | 5s
WebSocket message        | 100ms| 200ms | 500ms
```

### 14.2 Scalability Targets

```
Concurrent Users:
- 100,000 active customers
- 10,000 active drivers
- 5,000 active restaurants

Throughput:
- 10,000 orders per hour (peak)
- 100,000 location updates per minute
- 500,000 tracking requests per minute

Database:
- 10M orders per month
- 100M location points per month
- 1B temperature readings per month
```

### 14.3 Availability

```
Service Level: 99.9% uptime (43 minutes downtime per month)

Component SLAs:
- API Gateway: 99.99%
- Database: 99.95%
- WebSocket: 99.9%
- Background jobs: 99.5%

Disaster Recovery:
- RPO (Recovery Point Objective): 1 hour
- RTO (Recovery Time Objective): 4 hours
```

---

## 15. Integration Guidelines

### 15.1 Restaurant POS Integration

**Supported Formats:**
- ODATA (Open Data Protocol)
- REST API
- Webhook callbacks
- FTP/SFTP file exchange

**Integration Flow:**
```
POS System → Order Created → WIA API → Driver Assigned
→ Driver Arrives → POS Notification → Order Ready
→ Driver Picks Up → POS Complete
```

### 15.2 Third-Party Delivery Platform Integration

**Multi-platform Aggregation:**
```
UberEats   ────┐
DoorDash   ────┤
GrubHub    ────┼──→ WIA-IND-009 Aggregator ──→ Unified Dashboard
Postmates  ────┤
Custom     ────┘
```

**Benefits:**
- Single dashboard for all orders
- Unified driver management
- Consolidated analytics
- Centralized customer communication

### 15.3 Mapping Services

**Supported Providers:**
- Google Maps Platform
- Mapbox
- HERE Maps
- OpenStreetMap (OSRM)

**Required APIs:**
- Geocoding (address → coordinates)
- Reverse geocoding (coordinates → address)
- Directions (route calculation)
- Distance Matrix (multi-point distances)
- Real-time traffic

### 15.4 Payment Processing

**Supported Gateways:**
- Stripe
- Square
- PayPal
- Braintree
- Adyen

**Payment Flow:**
```
Order Created → Payment Authorization → Order Confirmed
→ ... Delivery ... → Payment Capture → Settlement

Refund scenarios:
- Order cancelled before pickup: Full refund
- Order cancelled after pickup: Partial refund (delivery fee retained)
- Quality issue: Variable refund based on severity
```

---

## 16. Appendices

### Appendix A: Error Codes

```
1000-1099: Authentication/Authorization errors
2000-2099: Validation errors
3000-3099: Resource not found errors
4000-4099: Business logic errors
5000-5099: External service errors
6000-6099: System errors

Examples:
1001: Invalid or expired token
2001: Missing required field
2010: Invalid location coordinates
3001: Order not found
4001: Restaurant not accepting orders
4010: Driver not available
5001: Mapping service unavailable
6001: Database connection error
```

### Appendix B: Units and Conversions

```
Distance:
- SI: kilometers (km), meters (m)
- Imperial: miles (mi), feet (ft)
- Conversion: 1 mi = 1.60934 km

Temperature:
- SI: Celsius (°C)
- Imperial: Fahrenheit (°F)
- Conversion: °F = (°C × 9/5) + 32

Speed:
- SI: kilometers per hour (km/h)
- Imperial: miles per hour (mph)
- Conversion: 1 mph = 1.60934 km/h

Weight:
- SI: kilograms (kg)
- Imperial: pounds (lb)
- Conversion: 1 lb = 0.453592 kg
```

### Appendix C: References

1. FDA Food Code 2022
2. HACCP Principles & Application Guidelines
3. GDPR (General Data Protection Regulation)
4. CCPA (California Consumer Privacy Act)
5. PCI DSS v4.0
6. ISO 22000:2018 Food Safety Management
7. Google Maps Platform Documentation
8. REST API Design Best Practices (RFC 7231)

---

## 17. Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0.0 | 2025-01-15 | WIA Food Delivery WG | Initial release |

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*This specification is maintained by the WIA Food Delivery Working Group*
*© 2025 SmileStory Inc. / WIA*
*MIT License*

**For questions or contributions:**
- GitHub: https://github.com/WIA-Official/wia-standards
- Email: standards@wiastandards.com
- Web: https://wiastandards.com/ind-009


## Annex E — Implementation Notes for PHASE-4-INTEGRATION

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-4-INTEGRATION.

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
evidence for PHASE-4-INTEGRATION. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-4-integration/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-4-INTEGRATION with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-4-INTEGRATION does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-4-INTEGRATION.
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
for PHASE-4-INTEGRATION. The profile mechanism is non-normative and exists so that
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
  advertises `WIA-P4-INTEGRATION-Edge-only/2` is asserting conformance with
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
  is published manually. Sufficient for PHASE-4-INTEGRATION validation when the
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
