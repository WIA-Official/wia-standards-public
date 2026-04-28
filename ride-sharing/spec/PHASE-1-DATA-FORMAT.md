# WIA-AUTO-014 — Phase 1: Data Format

> Ride-sharing canonical envelopes: rider profile, driver attestation, ride request, dynamic pricing record, and the runtime conventions that fix the wire format for every protocol below.

## 1. Introduction

### 1.1 Purpose

This specification defines the technical framework and best practices for ride sharing platforms, enabling efficient, safe, and fair matching of drivers with riders while optimizing routes, pricing, and user experience.

### 1.2 Scope

The standard covers:
- Real-time matching algorithms for driver-rider pairing
- Dynamic pricing models including surge pricing
- Identity verification and background check protocols
- Route optimization and traffic-aware navigation
- Safety features and emergency protocols
- Payment processing and fare calculation
- Data privacy and security requirements

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard aims to democratize transportation, making it accessible and affordable for everyone while ensuring fair compensation for drivers, reducing traffic congestion, and minimizing environmental impact through shared mobility.

### 1.4 Terminology

- **Rider/Passenger**: Person requesting transportation
- **Driver**: Person providing transportation service
- **Trip**: Complete journey from pickup to destination
- **Match**: Pairing of driver with rider request
- **Surge**: Increased pricing during high demand periods
- **ETA**: Estimated Time of Arrival
- **Geofence**: Virtual geographic boundary
- **Heatmap**: Visual representation of demand density

---


## 2. Matching Algorithms

### 2.1 Core Matching Principles

The matching algorithm prioritizes:
1. **Efficiency**: Minimize pickup time and distance
2. **Fairness**: Equitable ride distribution among drivers
3. **Quality**: Maintain high service standards
4. **Economics**: Optimize earnings for drivers

### 2.2 Matching Score Formula

```
M(d,r) = w₁·D(d,r) + w₂·T(d,r) + w₃·R(d) + w₄·P(r) + w₅·V(d,r)
```

Where:
- `M(d,r)` = Matching score for driver `d` and rider `r` (0-1)
- `D(d,r)` = Distance factor
- `T(d,r)` = Time factor
- `R(d)` = Driver rating factor
- `P(r)` = Rider preference factor
- `V(d,r)` = Vehicle compatibility factor
- `w₁...w₅` = Weights (sum to 1.0)

**Default weights:**
- `w₁ = 0.35` (Distance)
- `w₂ = 0.25` (Time)
- `w₃ = 0.20` (Rating)
- `w₄ = 0.10` (Preference)
- `w₅ = 0.10` (Vehicle)

### 2.3 Distance Factor

```
D(d,r) = 1 - (d_pickup / d_max)
```

Where:
- `d_pickup` = Actual distance from driver to pickup location (km)
- `d_max` = Maximum acceptable pickup distance (typically 5-10 km)
- Result clamped to [0, 1]

### 2.4 Time Factor

```
T(d,r) = 1 - (t_eta / t_max)
```

Where:
- `t_eta` = Estimated time to arrival (minutes)
- `t_max` = Maximum acceptable wait time (typically 10-15 minutes)
- Result clamped to [0, 1]

### 2.5 Driver Rating Factor

```
R(d) = (rating_d - rating_min) / (rating_max - rating_min)
```

Where:
- `rating_d` = Driver's current rating (1-5)
- `rating_min` = Minimum acceptable rating (3.0)
- `rating_max` = Maximum rating (5.0)
- Drivers below `rating_min` are suspended

### 2.6 Matching Algorithm Steps

```
1. Receive ride request from rider R
2. Query available drivers within geofence radius
3. Filter drivers by:
   - Vehicle type match
   - Capacity requirements
   - Accessibility needs
   - Active status (online, not on trip)
4. For each eligible driver D:
   a. Calculate matching score M(D,R)
   b. Estimate pickup time and route
   c. Calculate preliminary fare
5. Rank drivers by matching score
6. Send request to top 3 drivers simultaneously
7. First to accept gets the trip
8. If no acceptance within timeout (30s):
   - Expand search radius
   - Increase surge multiplier
   - Repeat from step 2
```

### 2.7 Batch Matching

For carpooling and shared rides:

```
1. Collect pending ride requests in time window (30-60s)
2. Cluster requests by:
   - Geographic proximity (pickup/destination)
   - Time compatibility
   - Route overlap
3. For each cluster:
   a. Calculate optimal grouping
   b. Minimize total detour time
   c. Maximize driver utilization
4. Match groups to available drivers
5. Offer discounted fares for shared rides
```

### 2.8 Multi-Objective Optimization

```
Maximize: U = α·Revenue + β·ServiceQuality - γ·WaitTime - δ·Detour

Subject to:
- WaitTime ≤ MaxWait
- Detour ≤ MaxDetour (for carpooling)
- ServiceQuality ≥ MinQuality
- Driver earnings ≥ MinEarnings
```

Where `α, β, γ, δ` are tunable coefficients.

---


## 3. Dynamic Pricing Models

### 3.1 Base Fare Calculation

```
B = B_base + (D × R_distance) + (T × R_time)
```

Where:
- `B` = Base fare before multipliers
- `B_base` = Fixed base fee ($2-5, varies by city)
- `D` = Trip distance (km)
- `R_distance` = Rate per km ($0.75-2.00)
- `T` = Trip duration (minutes)
- `R_time` = Rate per minute ($0.15-0.50)

### 3.2 Final Price Formula

```
P = B × (1 + S) × (1 + D_coef) × (1 + T_coef) + F + C
```

Where:
- `P` = Final price
- `B` = Base fare
- `S` = Surge multiplier
- `D_coef` = Demand coefficient (0-0.5)
- `T_coef` = Time coefficient (0-0.3)
- `F` = Additional fees (tolls, airport, etc.)
- `C` = Service commission

### 3.3 Surge Pricing Model

```
S = max(0, min(S_max, k × ln(λ_demand / λ_supply)))
```

Where:
- `S` = Surge multiplier
- `S_max` = Maximum surge cap (typically 3-5x)
- `k` = Sensitivity constant (0.5-1.5)
- `λ_demand` = Request arrival rate (requests/minute)
- `λ_supply` = Available driver density (drivers/km²)

### 3.4 Demand Coefficient

```
D_coef = (N_requests - N_avg) / (N_max - N_avg)
```

Where:
- `N_requests` = Current active requests in area
- `N_avg` = Historical average for time/location
- `N_max` = Peak historical demand
- Clamped to [0, 0.5]

### 3.5 Time Coefficient

```
T_coef = P_peak + W_weather + E_event
```

Where:
- `P_peak` = Peak hours premium (0-0.15)
- `W_weather` = Weather impact (0-0.10)
- `E_event` = Special event premium (0-0.15)

**Peak Hours:**
- Morning rush: 7:00-9:30 AM → P_peak = 0.15
- Evening rush: 5:00-7:30 PM → P_peak = 0.15
- Late night: 10:00 PM-4:00 AM → P_peak = 0.20
- Off-peak: All other times → P_peak = 0.00

**Weather Impact:**
- Rain/Snow: W_weather = 0.10
- Extreme weather: W_weather = 0.15
- Clear: W_weather = 0.00

### 3.6 Fare Estimation Range

Due to uncertainty in traffic and route, provide fare range:

```
Fare_min = P × 0.85
Fare_max = P × 1.15
Fare_estimate = P
```

### 3.7 Upfront Pricing

For predictability, calculate and display fare before trip:

```
1. Calculate optimal route using current traffic
2. Estimate trip duration with buffer
3. Apply current surge multiplier
4. Lock in price (guaranteed not to increase)
5. If actual trip is significantly shorter, refund difference
```

### 3.8 Pricing Transparency

Display to user:
- Base fare breakdown
- Surge multiplier (if active)
- Additional fees
- Estimated vs. final fare
- Price comparison with alternatives

---


## 8. Data Formats

### 8.1 Location Data

```json
{
  "location": {
    "latitude": 37.7749,
    "longitude": -122.4194,
    "accuracy": 10,
    "altitude": 52.3,
    "bearing": 180.5,
    "speed": 15.3,
    "timestamp": "2025-12-26T10:30:00Z"
  },
  "address": {
    "street": "123 Market Street",
    "city": "San Francisco",
    "state": "CA",
    "postalCode": "94103",
    "country": "USA",
    "formattedAddress": "123 Market St, San Francisco, CA 94103"
  }
}
```

### 8.2 Ride Request

```json
{
  "requestId": "req_a1b2c3d4e5f6",
  "riderId": "rider_123456",
  "timestamp": "2025-12-26T10:30:00Z",
  "pickup": {
    "location": { "latitude": 37.7749, "longitude": -122.4194 },
    "address": "123 Market St, San Francisco, CA",
    "notes": "Near Starbucks entrance"
  },
  "destination": {
    "location": { "latitude": 37.8044, "longitude": -122.2712 },
    "address": "456 Oakland Ave, Oakland, CA"
  },
  "passengers": 2,
  "vehicleType": "sedan",
  "preferences": {
    "maxWaitTime": 300,
    "accessibility": ["wheelchair"],
    "petFriendly": false,
    "quiet": false,
    "temperature": 22
  },
  "scheduled": null,
  "paymentMethod": "card_ending_1234"
}
```

### 8.3 Driver Profile

```json
{
  "driverId": "driver_789012",
  "personalInfo": {
    "firstName": "John",
    "lastName": "Doe",
    "phone": "+14155551234",
    "email": "john.doe@example.com",
    "photoUrl": "https://cdn.example.com/drivers/789012.jpg"
  },
  "verification": {
    "idVerified": true,
    "backgroundCheckPassed": true,
    "backgroundCheckDate": "2025-01-15",
    "licenseNumber": "D1234567",
    "licenseExpiry": "2028-03-20",
    "licenseState": "CA"
  },
  "vehicle": {
    "make": "Toyota",
    "model": "Camry",
    "year": 2022,
    "color": "Silver",
    "licensePlate": "ABC1234",
    "capacity": 4,
    "type": "sedan",
    "features": ["ac", "bluetooth", "usb_charging"],
    "accessibility": []
  },
  "rating": {
    "average": 4.87,
    "totalTrips": 2543,
    "recentRating": 4.92,
    "last50Trips": 50
  },
  "status": {
    "online": true,
    "available": true,
    "currentTrip": null,
    "location": { "latitude": 37.7749, "longitude": -122.4194 }
  },
  "earnings": {
    "totalLifetime": 45678.90,
    "currentWeek": 1234.56,
    "pendingBalance": 234.56
  }
}
```

### 8.4 Trip Record

```json
{
  "tripId": "trip_x7y8z9",
  "riderId": "rider_123456",
  "driverId": "driver_789012",
  "status": "completed",
  "timestamps": {
    "requested": "2025-12-26T10:30:00Z",
    "accepted": "2025-12-26T10:30:15Z",
    "driverArrived": "2025-12-26T10:37:00Z",
    "pickupComplete": "2025-12-26T10:39:00Z",
    "dropoffComplete": "2025-12-26T11:04:00Z"
  },
  "locations": {
    "pickup": {
      "location": { "latitude": 37.7749, "longitude": -122.4194 },
      "address": "123 Market St, San Francisco, CA"
    },
    "destination": {
      "location": { "latitude": 37.8044, "longitude": -122.2712 },
      "address": "456 Oakland Ave, Oakland, CA"
    }
  },
  "route": {
    "distance": 15.3,
    "duration": 1500,
    "polyline": "encoded_polyline_data_here",
    "waypoints": []
  },
  "fare": {
    "currency": "USD",
    "baseFare": 3.50,
    "distanceFare": 11.48,
    "timeFare": 7.50,
    "surgeMultiplier": 1.5,
    "subtotal": 33.72,
    "fees": 1.25,
    "tip": 5.00,
    "total": 39.97,
    "driverEarnings": 28.78
  },
  "ratings": {
    "riderRating": 5,
    "driverRating": 5,
    "riderComment": "Great ride!",
    "driverComment": "Pleasant passenger"
  }
}
```

### 8.5 Fare Estimate

```json
{
  "estimateId": "est_123abc",
  "pickup": { "latitude": 37.7749, "longitude": -122.4194 },
  "destination": { "latitude": 37.8044, "longitude": -122.2712 },
  "vehicleTypes": [
    {
      "type": "economy",
      "displayName": "Economy",
      "capacity": 4,
      "eta": 4,
      "fare": {
        "minimum": 18.50,
        "maximum": 23.00,
        "estimate": 20.75,
        "currency": "USD",
        "surgeMultiplier": 1.0
      },
      "duration": 25,
      "distance": 15.3
    },
    {
      "type": "premium",
      "displayName": "Premium",
      "capacity": 4,
      "eta": 6,
      "fare": {
        "minimum": 28.00,
        "maximum": 35.00,
        "estimate": 31.50,
        "currency": "USD",
        "surgeMultiplier": 1.0
      },
      "duration": 25,
      "distance": 15.3
    }
  ],
  "timestamp": "2025-12-26T10:30:00Z",
  "expiresAt": "2025-12-26T10:35:00Z"
}
```

---



## A.1 Canonical envelope conventions

Every Phase 1 ride-sharing envelope follows the WIA family baseline:
UTF-8 JSON, RFC 8785 canonical form, Ed25519 signatures, ULID
identifiers. Geographic positions use ISO 6709 with WGS 84 datum;
distance and duration use SI units (metres, seconds).

## A.2 Rider profile envelope

```json
{
  "wia_ridesharing_version": "1.0.0",
  "type": "rider_profile",
  "rider_id": "did:wia:rider:01HX...",
  "verified_identity": true,
  "verified_payment_method": true,
  "rating_history": { "average": 4.8, "ratings_count": 142 },
  "preferences": { "wheelchair_access": false, "pet_friendly": true },
  "issued_at": "RFC 3339",
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

Rider profile fields with privacy implications (full name, phone)
are stored under WIA-OMNI-API credentials and never leave the
backend; the public-facing envelope carries only opaque references.

## A.3 Driver attestation envelope

Driver attestations carry: licence verification, vehicle inspection
state, insurance attestation, and background-check status. Each
field is signed by the verifying authority (DMV equivalent,
insurance carrier, background-check provider) so the platform can
prove the attestation chain to a regulator.

## A.4 Ride-request envelope

```json
{
  "wia_ridesharing_version": "1.0.0",
  "type": "ride_request",
  "request_id": "req_01HX...",
  "rider_id": "did:wia:rider:...",
  "pickup_iso6709": "+37.5665+126.9780/",
  "dropoff_iso6709": "+37.5172+127.0473/",
  "service_class": "economy" | "premium" | "carpool" | "wheelchair",
  "fare_estimate_minor": 12500,
  "currency": "KRW",
  "issued_at": "RFC 3339"
}
```


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
