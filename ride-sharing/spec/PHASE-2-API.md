# WIA-AUTO-014 — Phase 2: API

> Service API surface: matching, route optimisation, safety features, and payment integration — each presented as a worked endpoint specification.

## 4. Driver and Rider Verification

### 4.1 Driver Verification Process

#### 4.1.1 Identity Verification

```
1. Government-issued photo ID submission
2. Selfie capture and liveness detection
3. Biometric face matching (>95% confidence)
4. Name and address verification
5. SSN/Tax ID verification (region-specific)
```

#### 4.1.2 Background Checks

**Required checks:**
- Criminal record check (7-year history)
- Sex offender registry check
- Driving record check (3-5 year history)
- Drug and alcohol screening
- Vehicle inspection and registration
- Insurance verification

**Disqualifying factors:**
- Violent crimes
- Sexual offenses
- DUI within 5 years
- Major license violations
- Suspended/revoked license
- Uninsured vehicle

#### 4.1.3 Vehicle Requirements

```
1. Age: < 10-15 years (varies by market)
2. Inspection: Pass safety inspection
3. Insurance: Commercial or rideshare coverage
4. Capacity: 4+ passenger seats
5. Condition: Clean, well-maintained
6. Features: Working AC, seatbelts, airbags
```

#### 4.1.4 Driver Training

**Mandatory modules:**
- Platform app usage
- Customer service standards
- Safety protocols
- Emergency procedures
- Route optimization
- Anti-discrimination policies
- Accessibility requirements

#### 4.1.5 Continuous Monitoring

```
1. Real-time GPS tracking
2. Driving behavior analysis (acceleration, braking, speed)
3. Rating and complaint monitoring
4. Periodic re-verification (annual)
5. Random alcohol tests (if complaints)
6. Vehicle spot inspections
```

### 4.2 Rider Verification

#### 4.2.1 Account Creation

```
1. Phone number verification (SMS code)
2. Email verification
3. Payment method validation
4. Name and profile photo
5. Optional: Government ID for premium features
```

#### 4.2.2 Trust Score

```
T_rider = w₁·R + w₂·C + w₃·H + w₄·V

Where:
- R = Average rating (0-5)
- C = Cancellation rate (0-1, inverted)
- H = Account history (months active)
- V = Verification level (0-1)
```

**Trust levels:**
- T < 0.3: Flagged account (manual review required)
- 0.3 ≤ T < 0.6: Basic access
- 0.6 ≤ T < 0.8: Standard access
- T ≥ 0.8: Premium access (priority matching)

### 4.3 Two-Way Rating System

#### 4.3.1 Driver Rating Calculation

```
R_driver = (α·R_recent + β·R_lifetime) / (α + β)

Where:
- R_recent = Average of last 50 trips
- R_lifetime = Lifetime average rating
- α = 0.7 (recent weight)
- β = 0.3 (lifetime weight)
```

**Rating categories:**
- 5 stars: Excellent (target)
- 4 stars: Good
- 3 stars: Acceptable
- 2 stars: Poor (trigger review)
- 1 star: Unacceptable (trigger investigation)

**Automatic actions:**
- Rating < 4.6: Warning notification
- Rating < 4.3: Mandatory retraining
- Rating < 4.0: Account suspension

#### 4.3.2 Rider Rating

Similar calculation for riders:
- Helps drivers make informed acceptance decisions
- Flagged riders (< 3.5) may have limited access
- Consistent poor behavior leads to ban

---


## 5. Route Optimization

### 5.1 Single Trip Routing

#### 5.1.1 Route Selection Algorithm

```
1. Generate candidate routes:
   a. Fastest route (minimize time)
   b. Shortest route (minimize distance)
   c. Eco-friendly route (minimize emissions)
   d. Scenic route (if requested)

2. For each route, calculate:
   - Total distance
   - Estimated time (with traffic)
   - Fuel/energy cost
   - Toll costs
   - Carbon emissions
   - Road quality score

3. Select optimal route based on:
   Route_score = w₁·(1/Time) + w₂·(1/Distance) + w₃·(1/Cost) + w₄·Quality
```

**Default weights:**
- `w₁ = 0.50` (Time priority)
- `w₂ = 0.25` (Distance)
- `w₃ = 0.15` (Cost)
- `w₄ = 0.10` (Quality)

#### 5.1.2 Real-Time Traffic Integration

```
T_actual = T_base × (1 + C_traffic + C_weather + C_incidents)

Where:
- T_base = Free-flow travel time
- C_traffic = Traffic congestion factor (0-2.0)
- C_weather = Weather delay factor (0-0.5)
- C_incidents = Accident/closure factor (0-1.0)
```

#### 5.1.3 Dynamic Rerouting

Trigger rerouting when:
- Traffic incident detected on route
- Estimated delay > 5 minutes
- Faster alternative discovered
- Road closure notification
- Driver deviation from route

```
Rerouting_algorithm:
1. Detect triggering condition
2. Calculate new optimal route
3. Compare time savings vs. current route
4. If savings > threshold (3 minutes):
   a. Update navigation
   b. Notify driver and rider
   c. Adjust ETA
   d. Recalculate fare if significant
```

### 5.2 Multi-Stop Optimization

For carpooling/shared rides with multiple pickups/dropoffs:

```
1. Define waypoints: W = {P₁, P₂, ..., Pₙ, D₁, D₂, ..., Dₙ}
   Where P = pickups, D = dropoffs

2. Constraints:
   - Each passenger picked up before dropped off
   - Maximum detour per passenger: 15 minutes
   - Maximum trip duration increase: 30%
   - Vehicle capacity not exceeded

3. Optimization (Traveling Salesman Problem variant):
   Minimize: Total_time = Σ(time between waypoints)

   Subject to:
   - Precedence: pickup_i before dropoff_i
   - Capacity: passengers ≤ vehicle_capacity
   - Quality: detour_i ≤ max_detour
   - Fairness: max_detour similar for all passengers

4. Use heuristic algorithms:
   - Nearest neighbor
   - Genetic algorithm
   - Simulated annealing
   - Dynamic programming (for small n)
```

### 5.3 Route Efficiency Metrics

```
E_route = (D_optimal / D_actual) × (T_optimal / T_actual) × C_carbon

Where:
- D_optimal = Straight-line distance
- D_actual = Actual route distance
- T_optimal = Estimated free-flow time
- T_actual = Actual trip time
- C_carbon = Carbon efficiency (1.0 for EV, 0.7-0.9 for ICE)
```

**Efficiency targets:**
- E_route ≥ 0.85: Excellent
- 0.70 ≤ E_route < 0.85: Good
- 0.60 ≤ E_route < 0.70: Acceptable
- E_route < 0.60: Poor (investigate)

### 5.4 Eco-Routing

For environmentally conscious routing:

```
Emissions = D × E_vehicle × C_traffic × C_speed

Where:
- D = Distance (km)
- E_vehicle = Vehicle emission factor (g CO₂/km)
- C_traffic = Congestion multiplier (1.0-1.5)
- C_speed = Speed efficiency (optimal at 50-70 km/h)

Optimize for:
- Minimize total emissions
- Avoid stop-and-go traffic
- Prefer highways at steady speeds
- Avoid steep inclines
```

---


## 6. Safety Features

### 6.1 Pre-Trip Safety

#### 6.1.1 Vehicle Safety Check

```
Daily checklist (driver confirms):
- [ ] Tires properly inflated
- [ ] Brakes functioning
- [ ] Lights operational
- [ ] Seatbelts working
- [ ] Interior clean
- [ ] Fuel/charge adequate
- [ ] Emergency kit present
```

#### 6.1.2 Driver Readiness

```
Before accepting rides:
- [ ] Well-rested (no fatigue)
- [ ] Sober (no alcohol/drugs)
- [ ] Focused (no distractions)
- [ ] Healthy (no illness impairing driving)
```

**Fatigue detection:**
```
- Maximum continuous driving: 8 hours
- Mandatory break after 4 hours: 30 minutes
- Maximum daily driving: 12 hours
- Minimum rest between days: 8 hours
```

### 6.2 During Trip Safety

#### 6.2.1 Real-Time GPS Tracking

```
1. Continuous location updates (every 5-10 seconds)
2. Route adherence monitoring
3. Speed limit compliance checking
4. Geofence violation alerts
5. Share trip with trusted contacts
```

#### 6.2.2 Ride Check-In

```
Automatic check-in prompts:
- Long stops (> 5 minutes): "Everything okay?"
- Route deviation: "Are you going the right way?"
- Extended trip: "Trip taking longer than expected?"
- Late night: "Would you like to share your trip?"
```

#### 6.2.3 Emergency Button

**Two-tier emergency system:**

**Tier 1: Discreet Help**
- Silent alert to safety team
- GPS tracking intensifies
- Audio recording starts
- Nearby authorities notified

**Tier 2: Immediate Emergency**
- Direct 911/emergency services call
- Share exact location
- Send trip details to authorities
- Alert emergency contacts
- Trigger vehicle systems (flashers, horn)

#### 6.2.4 Driving Behavior Monitoring

```
Safety_score = w₁·S + w₂·A + w₃·B + w₄·C

Where:
- S = Speed compliance (0-1)
- A = Smooth acceleration (0-1)
- B = Gentle braking (0-1)
- C = Safe cornering (0-1)
```

**Thresholds:**
- Hard braking: deceleration > 0.4g
- Harsh acceleration: acceleration > 0.3g
- Excessive speed: > 15 km/h over limit
- Sharp turn: lateral acceleration > 0.5g

**Actions:**
- Score < 0.7: Warning notification
- Score < 0.5: Mandatory safety training
- Score < 0.3: Account suspension

### 6.3 Post-Trip Safety

#### 6.3.1 Trip Completion Verification

```
1. Rider confirms safe arrival
2. GPS verifies destination reached
3. Rating and feedback collection
4. Report issues if any
5. Emergency contacts notified of safe arrival (if shared)
```

#### 6.3.2 Incident Reporting

**Report types:**
- Safety concern
- Accident/collision
- Harassment
- Lost item
- Route/pricing dispute
- Vehicle condition
- Driver/rider behavior

**Investigation process:**
```
1. Automatic report flagging
2. Safety team review (< 24 hours)
3. Contact involved parties
4. Review trip data (GPS, audio if activated)
5. Determine action:
   - Warning
   - Temporary suspension
   - Permanent ban
   - Law enforcement referral
6. Follow-up with reporter
```

### 6.4 Insurance and Liability

**Required coverage periods:**

**Period 1:** App on, waiting for ride request
- Liability: Minimum state requirements
- Collision: Optional

**Period 2:** Ride accepted, en route to pickup
- Liability: $1M per incident
- Collision: Comprehensive coverage
- Uninsured motorist: $1M

**Period 3:** Passenger in vehicle
- Liability: $1M+ per incident
- Collision: Full coverage
- Medical: $1M per person
- Uninsured motorist: $1M

---


## 7. Payment Integration

### 7.1 Payment Methods

**Supported payment types:**
- Credit/debit cards (Visa, Mastercard, Amex, etc.)
- Digital wallets (Apple Pay, Google Pay, PayPal)
- Bank accounts (direct debit)
- Prepaid accounts
- Cash (select markets)
- Corporate accounts
- Gift cards/credits

### 7.2 Payment Flow

```
1. Pre-authorization:
   - Hold estimated fare on card
   - Verify sufficient funds
   - Timeout: Release hold after 7 days if trip not completed

2. Trip completion:
   - Calculate final fare
   - Charge payment method
   - Release remaining hold
   - Generate receipt

3. Receipt details:
   - Trip date/time
   - Pickup and destination
   - Distance and duration
   - Fare breakdown
   - Driver info
   - Vehicle details
   - Trip map
```

### 7.3 Split Fare

Allow multiple riders to split payment:

```
Split_options:
1. Equal split: Total / N_riders
2. Custom split: Manual percentage allocation
3. Item split: Each pays for their segment

Process:
1. Request initiator sends split invitations
2. Recipients confirm split
3. Each payment method charged separately
4. All must approve before trip starts
```

### 7.4 Tipping

```
Tip_options = [0%, 10%, 15%, 20%, Custom]

Tip_processing:
1. Prompt after trip completion
2. Add to driver's earnings (100% to driver)
3. Separate line item on receipt
4. Can modify tip for up to 24 hours post-trip
```

### 7.5 Refunds and Disputes

**Automatic refund triggers:**
- Trip cancelled by driver after accept
- Significant delay without notification
- Route deviation causing excess charges
- Service quality issues

**Refund amounts:**
- Full refund: Trip not completed
- Partial refund: Service issues
- Fare adjustment: Pricing errors

**Dispute resolution:**
```
1. Rider submits dispute with reason
2. Automated review of trip data
3. If unresolved:
   a. Human review within 48 hours
   b. Contact both parties
   c. Make determination
   d. Process refund if approved
   e. Close dispute
4. Appeal process available
```

### 7.6 Driver Earnings

```
Driver_earnings = Base_fare × (1 - Commission) + Tips + Bonuses

Where:
- Commission: 15-30% (platform fee)
- Tips: 100% to driver
- Bonuses: Incentives and promotions
```

**Payout options:**
- Instant transfer: Available balance transferred immediately (small fee)
- Daily deposit: Automatic daily transfer (no fee)
- Weekly deposit: Weekly on set day (no fee)
- Manual transfer: On-demand by driver

---


## 9. API Interface

### 9.1 Core Endpoints

#### 9.1.1 Request Ride

```
POST /api/v1/rides/request

Request:
{
  "riderId": "rider_123",
  "pickup": {
    "location": { "lat": 37.7749, "lng": -122.4194 },
    "address": "123 Market St, San Francisco, CA"
  },
  "destination": {
    "location": { "lat": 37.8044, "lng": -122.2712 },
    "address": "456 Oakland Ave, Oakland, CA"
  },
  "passengers": 2,
  "vehicleType": "sedan"
}

Response:
{
  "tripId": "trip_x7y8z9",
  "status": "searching",
  "estimatedWait": 4,
  "fareEstimate": { "min": 18.50, "max": 23.00 }
}
```

#### 9.1.2 Accept Ride (Driver)

```
POST /api/v1/rides/{tripId}/accept

Request:
{
  "driverId": "driver_789",
  "eta": 4
}

Response:
{
  "tripId": "trip_x7y8z9",
  "status": "accepted",
  "riderInfo": {
    "name": "Jane Smith",
    "phone": "+14155551234",
    "rating": 4.9
  },
  "pickup": { ... },
  "destination": { ... }
}
```

#### 9.1.3 Update Location

```
PUT /api/v1/drivers/{driverId}/location

Request:
{
  "latitude": 37.7749,
  "longitude": -122.4194,
  "bearing": 180.5,
  "speed": 15.3,
  "timestamp": "2025-12-26T10:30:00Z"
}

Response:
{
  "success": true,
  "nearbyRequests": 3
}
```

#### 9.1.4 Complete Trip

```
POST /api/v1/rides/{tripId}/complete

Request:
{
  "driverId": "driver_789",
  "odometer": {
    "start": 45678.9,
    "end": 45694.2
  },
  "dropoffLocation": { "lat": 37.8044, "lng": -122.2712 }
}

Response:
{
  "tripId": "trip_x7y8z9",
  "status": "completed",
  "fare": {
    "total": 39.97,
    "breakdown": { ... }
  },
  "earnings": 28.78,
  "receipt": "https://receipts.example.com/trip_x7y8z9"
}
```

### 9.2 Supporting Endpoints

```
GET    /api/v1/fare/estimate          - Get fare estimate
GET    /api/v1/drivers/nearby          - Find nearby drivers
GET    /api/v1/drivers/{id}/rating     - Get driver rating
POST   /api/v1/rides/{id}/rate         - Rate completed trip
POST   /api/v1/rides/{id}/cancel       - Cancel ride
GET    /api/v1/rides/{id}/status       - Get trip status
POST   /api/v1/emergency               - Trigger emergency
GET    /api/v1/demand/heatmap          - Get demand heatmap
GET    /api/v1/surge/current           - Get current surge levels
POST   /api/v1/payment/methods         - Add payment method
GET    /api/v1/trips/history           - Get trip history
POST   /api/v1/support/report          - Report issue
```

### 9.3 WebSocket Events

Real-time updates via WebSocket:

```
// Connect
ws://api.example.com/ws/rider/{riderId}
ws://api.example.com/ws/driver/{driverId}

// Events
{
  "event": "driver_matched",
  "data": {
    "driverId": "driver_789",
    "eta": 4,
    "location": { "lat": 37.7749, "lng": -122.4194 }
  }
}

{
  "event": "driver_location_update",
  "data": {
    "location": { "lat": 37.7750, "lng": -122.4195 },
    "bearing": 180,
    "eta": 3
  }
}

{
  "event": "trip_status_change",
  "data": {
    "status": "arrived",
    "timestamp": "2025-12-26T10:37:00Z"
  }
}
```

---



## A.1 Endpoint reference

```http
POST /ridesharing/v1/ride/request    # rider requests a ride
POST /ridesharing/v1/ride/{id}/accept # driver accepts a ride
POST /ridesharing/v1/ride/{id}/cancel # rider or driver cancels
POST /ridesharing/v1/ride/{id}/start  # mark trip started
POST /ridesharing/v1/ride/{id}/end    # mark trip completed
POST /ridesharing/v1/safety/sos       # emergency button event
GET  /ridesharing/v1/driver/{id}      # driver-state query
```

Every endpoint follows the discovery convention at
`/.well-known/wia-ridesharing`.

## A.2 Matching algorithm

The matching API at `/ridesharing/v1/ride/request` returns the
matched driver in the response or a problem document explaining why
matching failed (no available drivers, geographic outside service
area, fare estimate exceeds rider's authorised payment limit). The
matching algorithm itself is a host implementation detail; the
standard requires only that the matched driver attestation chain
is verifiable by the rider before accepting.

## A.3 Route optimisation

Route optimisation runs server-side with the platform's traffic
data and returns a turn-by-turn route in the ride-acceptance
response. The route is advisory — the driver may deviate; the
deviation is logged for audit but not penalised unless the rider
files a complaint.

## A.4 Safety endpoint

The safety endpoint is the highest-stakes part of the API. Pressing
the SOS button in-app emits a signed envelope with the current ride
ID, the current location, and the rider's documented emergency
contact. The platform's safety operations centre receives the
envelope within seconds and may dispatch local emergency services.


## Z.1 Glossary

The companion glossary at `https://wiastandards.com/ride-sharing/glossary/`
expands every term used throughout this Phase. Implementers
unfamiliar with the domain should treat it as load-bearing reading.

## Z.2 Cross-standard composition

This Phase composes with: **WIA-OMNI-API** (credential storage),
**WIA-AIR-SHIELD** (runtime trust list), **WIA-SOCIAL Phase 3 §5**
(federation handshake), and **WIA-INTENT** (workload intent
declaration).

## Z.3 Conformance test suite + reference container

A black-box conformance test suite at
`https://github.com/WIA-Official/wia-ride-sharing-conformance` walks
every public endpoint and protocol exchange. The reference
container at `wia/ride-sharing-host:1.0.0` implements every Phase 2
endpoint with mock data so integrators exercise their bridge
before production. The companion CLI at `cli/ride-sharing.sh` ships
sample envelope generators (validate, info, plus phase-specific
subcommands) so an implementer can produce conformant payloads
without hand-rolling JSON.

## Z.4 Implementation runbook

A first implementation typically follows: (1) stand up reference
container, (2) run conformance suite against it, (3) replace mock
backend with real backend one endpoint at a time, (4) wire up audit
log replication, (5) onboard a single trusted peer for federation,
(6) expand to multiple peers, (7) promote to production with
warning-envelope subscription.

## Z.5 Backwards-compatibility promise + governance

Within the 1.x line every Phase 1 envelope shape, every Phase 2
endpoint, and every Phase 3 protocol exchange MUST remain reachable.
Hosts MAY add optional fields and new envelopes; hosts MUST NOT
remove existing ones. Breaking changes ride a major version bump
with a 12-month deprecation window per IETF RFC 8594 / 9745, and
require a two-thirds Committee vote.

弘益人間 — Benefit All Humanity.
