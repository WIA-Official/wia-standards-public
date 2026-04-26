# WIA-IND-009 PHASE 2 — API Interface Specification

**Standard:** WIA-IND-009
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 2 of 4)

---

**Problem:** Traveling Salesman Problem (TSP) with time windows

**Objective:** Minimize total route time while meeting delivery windows

**Algorithm: 2-Opt with Time Window Constraints**

```python
def optimize_multi_stop_route(orders):
    """
    Optimize route for multiple pickups and deliveries

    Constraints:
    - Each pickup must occur before its delivery
    - Delivery must occur within promised time window
    - Temperature requirements must be maintained
    - Vehicle capacity must not be exceeded
    """

    # Create stops list (pickups and deliveries)
    stops = []
    for order in orders:
        stops.append({
            'type': 'pickup',
            'order_id': order.id,
            'location': order.pickup_location,
            'time_window': order.pickup_window,
            'duration': 5  # minutes
        })
        stops.append({
            'type': 'delivery',
            'order_id': order.id,
            'location': order.delivery_location,
            'time_window': order.delivery_window,
            'duration': 3  # minutes
        })

    # Initial route using nearest neighbor
    route = nearest_neighbor_tsp(stops)

    # Validate pickup-before-delivery constraint
    route = enforce_precedence(route)

    # Optimize using 2-opt
    improved = True
    while improved:
        improved = False
        for i in range(len(route) - 1):
            for j in range(i + 2, len(route)):
                new_route = two_opt_swap(route, i, j)

                if (is_valid(new_route) and
                    route_cost(new_route) < route_cost(route)):
                    route = new_route
                    improved = True

    return route

def route_cost(route):
    """Calculate total cost (time + penalties)"""
    total_time = 0
    penalty = 0
    current_time = now()

    for i in range(len(route) - 1):
        # Travel time
        travel_time = calculate_travel_time(route[i], route[i+1])
        total_time += travel_time
        current_time += travel_time

        # Stop duration
        total_time += route[i+1]['duration']
        current_time += route[i+1]['duration']

        # Late delivery penalty
        if current_time > route[i+1]['time_window']['end']:
            penalty += 1000 * (current_time - route[i+1]['time_window']['end']).seconds

    return total_time + penalty
```

### 5.3 Dynamic Re-routing

**Triggers for Re-routing:**
1. Traffic incident on current route
2. New order added to batch
3. Restaurant delay (prep time extended)
4. Driver deviation from route
5. Road closure or weather event

**Re-routing Decision:**
```python
def should_reroute(current_route, new_conditions):
    """
    Decide if re-routing is beneficial

    Re-route if:
    - New route saves >5 minutes AND >10% time
    - Current route becomes infeasible
    - Critical alert (accident, road closure)
    """

    new_route = calculate_route(new_conditions)

    time_saved = current_route.eta - new_route.eta
    percent_saved = time_saved / current_route.duration

    if current_route.is_infeasible():
        return True, new_route

    if time_saved > 300 and percent_saved > 0.10:  # 5 min and 10%
        return True, new_route

    return False, current_route
```

### 5.4 Zone-Based Optimization

**Geofencing Strategy:**
```
City divided into zones:
- Downtown: 2km x 2km high-density
- Urban: 5km x 5km medium-density
- Suburban: 10km x 10km low-density

Zone assignment:
- Restaurants belong to zone of location
- Drivers assigned to zone(s) based on location
- Orders matched within zone first, then adjacent zones
```

**Benefits:**
- Reduced average pickup distance (30-40%)
- Better driver familiarity with area
- Improved ETA accuracy
- Easier capacity planning

---

## 6. Temperature Monitoring

### 6.1 Temperature Requirements

**Food Safety Zones:**
```
Danger Zone: 4°C - 60°C (bacteria growth)
Hot Food Safety: ≥60°C (140°F)
Cold Food Safety: ≤4°C (39°F)
Frozen Food: ≤-18°C (0°F)
Ambient: 15-25°C
```

**Maximum Transit Times:**
```
Hot Food: 45 minutes (before falling below 60°C)
Cold Food: 60 minutes (before rising above 4°C)
Frozen: 30 minutes (before thawing begins)
Ambient: 90 minutes (quality degradation)
```

### 6.2 Temperature Monitoring System

**IoT Sensor Specifications:**
```json
{
  "sensorId": "string (UUID)",
  "type": "bluetooth|wifi|cellular",
  "accuracy": "±0.5°C",
  "range": "-20°C to 100°C",
  "batteryLife": "180 days",
  "reportingInterval": "60 seconds",
  "alertThresholds": {
    "hot_min": 60,
    "cold_max": 4,
    "frozen_max": -15
  }
}
```

**Data Collection:**
```typescript
interface TemperatureReading {
  timestamp: Date;
  orderId: string;
  sensorId: string;
  temperature: number;  // Celsius
  humidity?: number;    // Percentage
  batteryLevel: number; // Percentage
  location: {
    latitude: number;
    longitude: number;
  };
}
```

**Storage:**
- Time-series database (TimescaleDB, InfluxDB)
- Retention: 90 days for compliance
- Compression: After 7 days
- Aggregation: 1-minute averages after 30 days

### 6.3 Temperature Alerts

**Alert Levels:**
```
WARNING: Temperature within 5°C of threshold for >2 minutes
CRITICAL: Temperature exceeds threshold for >5 minutes
SEVERE: Temperature in danger zone for >15 minutes
```

**Alert Actions:**
```python
def handle_temperature_alert(reading, order):
    """Process temperature alert"""

    if reading.alert_level == 'WARNING':
        # Notify driver
        send_driver_notification(order.driver_id,
            "Temperature warning: Check food container")

    elif reading.alert_level == 'CRITICAL':
        # Notify driver and support team
        send_driver_notification(order.driver_id,
            "CRITICAL: Temperature out of range")
        create_support_ticket(order.id,
            priority='high',
            reason='temperature_violation')

    elif reading.alert_level == 'SEVERE':
        # Notify all parties, flag for refund
        send_multi_channel_alert(order)
        flag_for_quality_review(order.id)
        automatic_refund(order.id,
            reason='food_safety_violation')
```

### 6.4 Insulation Requirements

**Thermal Bag Specifications:**

| Food Type | Min R-Value | Max Heat Loss | Duration |
|-----------|-------------|---------------|----------|
| Hot | R-8 | 1°C per 10 min | 45 min |
| Cold | R-10 | 1°C per 15 min | 60 min |
| Frozen | R-12 + ice packs | 2°C per 30 min | 90 min |

**Active Temperature Control:**
```
Electric Hot Bag:
- Heating element: 50-100W
- Target temp: 65-75°C
- Power source: 12V vehicle or battery pack
- Auto shutoff: At delivery

Electric Cold Bag:
- Peltier cooling: 20-40W
- Target temp: 0-4°C
- Power source: 12V vehicle or battery pack
- Backup: Phase change materials (PCM)
```

---

## 7. Time Estimation

### 7.1 Delivery Time Formula

```
Total Delivery Time = T_prep + T_assign + T_pickup + T_transit + T_dropoff

Where:
T_prep = Restaurant preparation time (historical avg + current load)
T_assign = Driver assignment time (median: 2 min)
T_pickup = Driver arrival at restaurant + waiting + loading (5-10 min)
T_transit = Route distance / avg_speed × traffic_factor
T_dropoff = Parking + walking + handoff (3-5 min)
```

### 7.2 Preparation Time Prediction

**Machine Learning Model:**
```python
def predict_prep_time(restaurant_id, order_items, current_time):
    """
    Predict food preparation time using ML model

    Features:
    - Restaurant historical avg prep time
    - Number of items in order
    - Complexity of items (simple/complex)
    - Current restaurant load (pending orders)
    - Time of day (rush hour factor)
    - Day of week
    - Special events/holidays
    """

    features = {
        'restaurant_avg': get_restaurant_avg_prep(restaurant_id),
        'item_count': len(order_items),
        'complexity_score': calculate_complexity(order_items),
        'current_load': get_pending_orders(restaurant_id),
        'hour': current_time.hour,
        'is_peak': is_peak_hour(current_time),
        'day_of_week': current_time.weekday()
    }

    # Random Forest Regression model
    predicted_time = prep_time_model.predict(features)

    # Add confidence interval
    confidence = prep_time_model.predict_confidence(features)

    return {
        'expected': predicted_time,
        'min': predicted_time * 0.8,
        'max': predicted_time * 1.3,
        'confidence': confidence
    }
```

### 7.3 Transit Time Calculation

**Speed Estimation by Vehicle Type:**
```
Bike: 15 km/h average (urban)
E-bike: 20 km/h average
Scooter: 25 km/h average
Motorcycle: 35 km/h average
Car: 30 km/h average (urban), 60 km/h (suburban)
```

**Traffic Factors:**
```python
def get_traffic_factor(route, time):
    """
    Calculate traffic multiplier for route

    Returns multiplier: 1.0 (no traffic) to 3.0 (heavy congestion)
    """

    # Historical traffic patterns
    historical = get_historical_traffic(route, time.hour, time.weekday())

    # Real-time traffic data
    realtime = get_realtime_traffic(route)

    # Weighted average (70% realtime, 30% historical)
    traffic_factor = 0.7 * realtime + 0.3 * historical

    # Weather adjustment
    weather = get_weather_conditions()
    if weather.precipitation > 0:
        traffic_factor *= (1 + 0.2 * weather.precipitation)  # Max +20%

    return min(traffic_factor, 3.0)  # Cap at 3x
```

### 7.4 ETA Updates

**Update Frequency:**
```
Initial ETA: At order confirmation
Update 1: When driver assigned (based on driver location)
Update 2: When driver picks up (based on actual route)
Update 3: Every 2 minutes during transit
Final: When driver is <2 min away
```

**ETA Accuracy Targets:**
```
Confidence Level  | Accuracy Target | Use Case
------------------|-----------------|----------
50% confidence   | ±5 minutes      | Initial estimate
80% confidence   | ±3 minutes      | Post-assignment
95% confidence   | ±1 minute       | During transit
99% confidence   | ±30 seconds     | Final approach
```

---

## 8. Cost Calculation

### 8.1 Pricing Components

**Base Delivery Fee:**
```python
def calculate_base_fee(distance_km):
    """
    Base fee increases with distance
    """
    if distance_km <= 2:
        return 2.99
    elif distance_km <= 5:
        return 3.99
    elif distance_km <= 10:
        return 5.99
    else:
        return 5.99 + (distance_km - 10) * 0.50
```

**Distance Fee:**
```
Distance Fee = distance_km × per_km_rate

per_km_rate varies by:
- Vehicle type (bike: $0.30, car: $0.50)
- Region (urban: lower, rural: higher)
- Time (peak: higher, off-peak: lower)
```

**Time Fee:**
```
Time Fee = estimated_minutes × per_minute_rate

per_minute_rate = $0.10 - $0.20
(compensates driver for time spent)
```

**Small Order Fee:**
```
if order_subtotal < minimum_order:
    small_order_fee = minimum_order - order_subtotal
else:
    small_order_fee = 0

Typical minimum: $10-15
```

**Service Fee:**
```
Service Fee = order_subtotal × service_rate

service_rate = 10-20% (platform commission)
```

**Surge Pricing:**
```python
def calculate_surge_multiplier(zone, time):
    """
    Dynamic surge pricing based on demand/supply
    """

    demand = count_active_orders(zone)
    supply = count_available_drivers(zone)

    ratio = demand / max(supply, 1)

    if ratio < 1.0:
        surge = 1.0  # No surge
    elif ratio < 2.0:
        surge = 1.0 + 0.25 * (ratio - 1.0)  # Up to 1.25x
    elif ratio < 4.0:
        surge = 1.25 + 0.375 * (ratio - 2.0)  # Up to 2.0x
    else:
        surge = min(2.0 + 0.25 * (ratio - 4.0), 3.0)  # Cap at 3.0x

    return surge
```

### 8.2 Total Cost Breakdown

```typescript
interface DeliveryCost {


## Annex E — Implementation Notes for PHASE-2-API-INTERFACE

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-2-API-INTERFACE.

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
evidence for PHASE-2-API-INTERFACE. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-2-api-interface/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-2-API-INTERFACE with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-2-API-INTERFACE does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-2-API-INTERFACE.
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
for PHASE-2-API-INTERFACE. The profile mechanism is non-normative and exists so that
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
  advertises `WIA-P2-API-INTERFACE-Edge-only/2` is asserting conformance with
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
  is published manually. Sufficient for PHASE-2-API-INTERFACE validation when the
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
