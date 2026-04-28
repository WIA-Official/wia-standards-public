# WIA-AUTO-016 — Phase 3: Protocol

> Smart-logistics canonical Phase 3: protocols (VRP solver + tracking + last-mile).

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


## 4. Route Optimization Algorithms

### 4.1 Vehicle Routing Problem (VRP)

#### 4.1.1 Basic VRP Formulation

```
Minimize: Z = ΣΣ cᵢⱼ × xᵢⱼₖ

Subject to:
- Each customer visited exactly once: Σₖ Σⱼ xᵢⱼₖ = 1, ∀i ∈ Customers
- Flow conservation: Σᵢ xᵢⱼₖ = Σᵢ xⱼᵢₖ, ∀j, ∀k
- Vehicle capacity: Σᵢ qᵢ × Σⱼ xᵢⱼₖ ≤ Qₖ, ∀k
- Time windows: aᵢ ≤ tᵢ ≤ bᵢ, ∀i
```

Where:
- `cᵢⱼ` = Cost to travel from i to j
- `xᵢⱼₖ` = 1 if vehicle k travels from i to j, 0 otherwise
- `qᵢ` = Demand at location i
- `Qₖ` = Capacity of vehicle k
- `aᵢ, bᵢ` = Time window for location i
- `tᵢ` = Arrival time at location i

#### 4.1.2 Multi-Objective Optimization

**Cost Function**:
```
C(R) = α·D(R) + β·T(R) + γ·F(R) + δ·E(R) + ε·V(R)
```

Where:
- `D(R)` = Total distance: Σdᵢⱼ
- `T(R)` = Total time: Σ(tᵢⱼ + serviceᵢ)
- `F(R)` = Fuel cost: Σ(dᵢⱼ × fuel_rate × fuel_price)
- `E(R)` = Environmental impact: Σ(dᵢⱼ × emission_factor)
- `V(R)` = Time window violations: Σmax(0, tᵢ - bᵢ)²
- `α, β, γ, δ, ε` = Configurable weights

### 4.2 Solution Algorithms

#### 4.2.1 Genetic Algorithm

```python
def genetic_algorithm(population_size, generations):
    population = initialize_population(population_size)

    for gen in range(generations):
        # Evaluate fitness
        fitness = [evaluate(individual) for individual in population]

        # Selection (Tournament)
        parents = tournament_selection(population, fitness, size=3)

        # Crossover (Order Crossover - OX)
        offspring = []
        for i in range(0, len(parents), 2):
            child1, child2 = order_crossover(parents[i], parents[i+1])
            offspring.extend([child1, child2])

        # Mutation (Swap)
        for individual in offspring:
            if random() < mutation_rate:
                swap_mutation(individual)

        # Elitism + New generation
        population = elitism(population, fitness, elite_size) + offspring
        population = population[:population_size]

    return best_individual(population)
```

**Order Crossover (OX)**:
```
Parent 1: [A B | C D E | F G]
Parent 2: [C D | A F G | E B]

Child 1:  [F G | C D E | A B]  # Keep middle from P1, fill from P2 order
```

#### 4.2.2 Ant Colony Optimization (ACO)

```python
def ant_colony_optimization(num_ants, iterations):
    # Initialize pheromone trails
    pheromone = initialize_pheromone(num_nodes)
    best_solution = None

    for iteration in range(iterations):
        solutions = []

        for ant in range(num_ants):
            solution = construct_solution(pheromone)
            solutions.append(solution)

            if best_solution is None or cost(solution) < cost(best_solution):
                best_solution = solution

        # Update pheromones
        pheromone = evaporate(pheromone, rho=0.1)

        for solution in solutions:
            delta = Q / cost(solution)  # Q is a constant
            for edge in solution:
                pheromone[edge] += delta

    return best_solution

def construct_solution(pheromone):
    current = depot
    unvisited = set(all_customers)
    route = [current]

    while unvisited:
        # Probability of selecting next node
        probabilities = []
        for node in unvisited:
            tau = pheromone[current][node]  # Pheromone
            eta = 1.0 / distance[current][node]  # Heuristic (inverse distance)
            prob = (tau ** alpha) * (eta ** beta)
            probabilities.append((node, prob))

        # Select next node (roulette wheel)
        next_node = roulette_selection(probabilities)
        route.append(next_node)
        current = next_node
        unvisited.remove(next_node)

    return route
```

Parameters:
- `α` = Pheromone importance (typically 1.0)
- `β` = Heuristic importance (typically 2-5)
- `ρ` = Evaporation rate (typically 0.1-0.3)
- `Q` = Pheromone deposit constant

#### 4.2.3 Dynamic Route Adjustment

**Real-time Route Recalculation Triggers**:
- New urgent order received
- Traffic conditions changed significantly (>20% impact)
- Vehicle breakdown or delay
- Customer availability changed
- Weather conditions deteriorated

**Incremental Update Algorithm**:
```python
def dynamic_update(current_route, new_order):
    # Only recalculate affected portion
    insertion_point = find_best_insertion(current_route, new_order)

    # Local optimization (2-opt)
    affected_segment = current_route[insertion_point-3:insertion_point+3]
    optimized = two_opt(affected_segment)

    # Update route
    new_route = (current_route[:insertion_point-3] +
                 optimized +
                 current_route[insertion_point+3:])

    return new_route if cost(new_route) < cost(current_route) else current_route
```

### 4.3 Traffic-Aware Routing

#### 4.3.1 Traffic Prediction Model

**Time-dependent travel time**:
```
t(i, j, τ) = t₀(i, j) × [1 + f(τ, day, weather)]
```

Where:
- `t₀(i, j)` = Free-flow travel time from i to j
- `f(τ, day, weather)` = Congestion factor
- `τ` = Time of day
- `day` = Day of week

**Machine Learning Model**:
```
Predicted_Time = LSTM(historical_traffic, current_time, day_of_week, weather, events)
```

Input features:
- Historical speed data (15-minute intervals)
- Time of day (one-hot encoded)
- Day of week (one-hot encoded)
- Weather conditions (temperature, precipitation, visibility)
- Special events (sports, concerts, holidays)

#### 4.3.2 Adaptive Routing

```python
def adaptive_routing(origin, destination, departure_time):
    # Get traffic predictions for departure time
    traffic_forecast = predict_traffic(departure_time, forecast_horizon=120)

    # Calculate multiple candidate routes
    candidates = []
    for route in get_candidate_routes(origin, destination, k=5):
        estimated_time = 0
        current_time = departure_time

        for segment in route:
            travel_time = get_travel_time(segment, current_time, traffic_forecast)
            estimated_time += travel_time
            current_time += travel_time

        candidates.append({
            'route': route,
            'time': estimated_time,
            'distance': calculate_distance(route),
            'cost': calculate_cost(route, estimated_time)
        })

    # Select best route based on multi-objective criteria
    return select_best(candidates, weights={'time': 0.5, 'distance': 0.3, 'cost': 0.2})
```

---



## 5. Real-time Tracking

### 5.1 Tracking Infrastructure

#### 5.1.1 Data Collection

**Sensor Data Sources**:
- GPS: Position (lat/lng), altitude, speed, heading
- Accelerometer: Acceleration in 3 axes
- Gyroscope: Rotation rates
- Temperature: Cargo temperature (for cold chain)
- Door sensors: Open/close events
- Barcode scanner: Package scans
- Mobile app: Driver updates, photos, signatures

**Data Transmission**:
```
Protocol: MQTT (Message Queue Telemetry Transport)
QoS Level: 1 (At least once delivery)
Encryption: TLS 1.3
Payload: JSON or Protocol Buffers
Compression: gzip (optional)
```

#### 5.1.2 Event Detection

**Critical Events**:
```python
class EventDetector:
    def detect_events(self, telemetry):
        events = []

        # Harsh braking
        if telemetry.deceleration < -3.5:  # m/s²
            events.append(Event('harsh_braking', severity='warning'))

        # Speeding
        if telemetry.speed > speed_limit * 1.1:
            events.append(Event('speeding', severity='violation'))

        # Geofence violation
        if not in_authorized_zone(telemetry.position):
            events.append(Event('geofence_violation', severity='alert'))

        # Temperature excursion (cold chain)
        if telemetry.cargo_temp < min_temp or telemetry.cargo_temp > max_temp:
            events.append(Event('temperature_violation', severity='critical'))

        # Unusual stop
        if is_stopped(telemetry) and not at_delivery_location(telemetry):
            if stop_duration > 15 * 60:  # 15 minutes
                events.append(Event('unusual_stop', severity='info'))

        return events
```

### 5.2 ETA Calculation

#### 5.2.1 Basic ETA Formula

```
ETA = current_time + remaining_time

remaining_time = D/v + Σ(tᵢ) + W + B
```

Where:
- `D` = Remaining distance to destination
- `v` = Average velocity (considering traffic)
- `Σ(tᵢ)` = Sum of time at intermediate stops
- `W` = Weather delay factor
- `B` = Buffer time for uncertainties

#### 5.2.2 Advanced ETA Prediction

**Machine Learning Model**:
```
ETA = f(current_position, destination, current_time, day_of_week,
         traffic_conditions, weather, driver_history, vehicle_type,
         remaining_stops, package_count, historical_similar_routes)
```

**Model Architecture**:
- Input layer: 50+ features
- Hidden layers: 3 layers (128, 64, 32 neurons)
- Output: Predicted time to destination (minutes)
- Loss function: Mean Absolute Error (MAE)
- Training: Historical delivery data (>100k routes)

**Confidence Interval**:
```
ETA_range = [ETA - 1.96·σ, ETA + 1.96·σ]
```

Where σ is the standard deviation from model prediction uncertainty.

### 5.3 Customer Notifications

#### 5.3.1 Notification Triggers

```python
notification_rules = {
    'out_for_delivery': {
        'trigger': lambda: status == 'out_for_delivery',
        'channels': ['sms', 'email', 'app'],
        'template': 'Your package is out for delivery. ETA: {eta}'
    },
    'approaching': {
        'trigger': lambda: distance_to_customer < 5km,
        'channels': ['sms', 'app'],
        'template': 'Your delivery is 5km away. ETA: {eta}'
    },
    'nearby': {
        'trigger': lambda: eta_minutes < 10,
        'channels': ['sms', 'app'],
        'template': 'Your delivery will arrive in {eta} minutes'
    },
    'delivered': {
        'trigger': lambda: status == 'delivered',
        'channels': ['sms', 'email', 'app'],
        'template': 'Your package has been delivered. POD: {pod_url}'
    },
    'exception': {
        'trigger': lambda: status == 'exception',
        'channels': ['sms', 'email', 'app', 'call'],
        'template': 'Delivery exception: {reason}. Contact: {support}'
    }
}
```

#### 5.3.2 Delivery Preferences

```json
{
  "customerId": "CUST-001",
  "preferences": {
    "notificationChannels": ["sms", "app"],
    "notificationFrequency": "key_events_only",
    "phoneNumber": "+1-555-0123",
    "email": "customer@example.com",
    "deliveryInstructions": "Leave at front door",
    "safePlace": "Behind planter",
    "requireSignature": false,
    "deliveryTimeWindows": [
      {"start": "08:00", "end": "12:00"},
      {"start": "18:00", "end": "21:00"}
    ],
    "alternativeRecipient": {
      "name": "Neighbor Jane",
      "address": "123 Next Door St"
    }
  }
}
```

---



## 6. Last-mile Delivery

### 6.1 Delivery Density Optimization

#### 6.1.1 Clustering Algorithm

**K-means for Delivery Zones**:
```python
def cluster_deliveries(deliveries, num_clusters):
    # Initialize centroids
    centroids = random_sample(deliveries, num_clusters)

    for iteration in range(max_iterations):
        # Assign deliveries to nearest centroid
        clusters = [[] for _ in range(num_clusters)]
        for delivery in deliveries:
            nearest = argmin([distance(delivery, c) for c in centroids])
            clusters[nearest].append(delivery)

        # Update centroids
        new_centroids = []
        for cluster in clusters:
            if cluster:
                avg_lat = mean([d.lat for d in cluster])
                avg_lng = mean([d.lng for d in cluster])
                new_centroids.append(Point(avg_lat, avg_lng))
            else:
                new_centroids.append(centroids[len(new_centroids)])

        if centroids == new_centroids:
            break

        centroids = new_centroids

    return clusters
```

**Density-Based Clustering (DBSCAN)**:
```
Parameters:
- ε (epsilon): Maximum distance between points (e.g., 2 km)
- MinPts: Minimum points to form dense region (e.g., 5)

Algorithm:
1. For each point p:
   - Find all points within ε distance
   - If count ≥ MinPts, p is core point

2. Form clusters:
   - Connect core points within ε distance
   - Add border points (within ε of core, but not core themselves)
   - Mark remaining as noise/outliers
```

### 6.2 Alternative Delivery Options

#### 6.2.1 Delivery Locker Network

**Locker Assignment Algorithm**:
```python
def assign_locker(package, customer_location):
    # Find lockers within acceptable radius
    candidate_lockers = find_lockers_near(customer_location, max_distance=5km)

    # Filter by availability and size
    suitable_lockers = []
    for locker in candidate_lockers:
        if has_available_compartment(locker, package.size):
            suitable_lockers.append(locker)

    if not suitable_lockers:
        return None  # No suitable locker found

    # Score lockers
    scored = []
    for locker in suitable_lockers:
        score = (
            w1 * (1 - distance(customer, locker) / max_distance) +
            w2 * utilization_rate(locker) +
            w3 * customer_preference_score(customer, locker)
        )
        scored.append((locker, score))

    # Select best locker
    best_locker = max(scored, key=lambda x: x[1])[0]

    # Reserve compartment
    compartment = reserve_compartment(best_locker, package.size)

    return {
        'locker': best_locker,
        'compartment': compartment,
        'accessCode': generate_access_code()
    }
```

#### 6.2.2 Click-and-Collect

**Pickup Point Selection**:
```json
{
  "pickupPoints": [
    {
      "id": "PP-001",
      "name": "Main Street Retail",
      "address": "123 Main St",
      "location": {"lat": 37.7749, "lng": -122.4194},
      "hours": {
        "monday": {"open": "08:00", "close": "20:00"},
        "sunday": {"open": "10:00", "close": "18:00"}
      },
      "capacity": 500,
      "currentLoad": 245,
      "services": ["package_hold", "returns", "print_label"],
      "distance": 1.2,
      "estimatedWalkTime": 15
    }
  ]
}
```

### 6.3 Urban Delivery Challenges

#### 6.3.1 Parking and Access

**Parking Prediction Model**:
```
P(parking_available | location, time, day) =
    logistic_regression(
        historical_data,
        time_of_day,
        day_of_week,
        special_events,
        land_use_type
    )
```

**Delivery Time Buffer**:
```
delivery_time = base_service_time + parking_time + walking_time + elevator_time

where:
- base_service_time = 2-5 minutes (hand package, get signature)
- parking_time = f(parking_availability_probability)
  - High availability (>0.7): 2 min
  - Medium (0.3-0.7): 5 min
  - Low (<0.3): 10 min
- walking_time = distance_from_parking / walking_speed (1.4 m/s)
- elevator_time = floor_number * 15 seconds (if building has elevator)
```

#### 6.3.2 Multi-story Building Delivery

**Building Profile**:
```json
{
  "buildingId": "BLDG-001",
  "address": "456 High St",
  "type": "apartment",
  "floors": 25,
  "unitsPerFloor": 8,
  "accessControl": {
    "type": "intercom",
    "codes": ["*#1234", "buzzDriver"],
    "parkingAccess": "delivery_zone"
  },
  "deliveryInstructions": [
    "Park in delivery zone (rear of building)",
    "Use service elevator (code: 5678)",
    "Leave packages at unit door if no answer"
  ],
  "historicalData": {
    "avgDeliveryTime": 8.5,
    "parkingAvailability": 0.85,
    "accessIssueRate": 0.12
  }
}
```

---




---

## A.1 Vehicle Routing Problem (VRP) protocol

The protocol layer wraps the VRP solver so callers do not need to assemble the cost matrix themselves. Inputs are the vehicle fleet, the customer set with demands and time windows, and the objective weights. Outputs are the per-vehicle stop sequence, total cost decomposed by component, and an explanation block that names which constraints were binding at the optimum.

## A.2 Algorithm selector

The protocol selects between Genetic Algorithm, Ant Colony Optimization, and a 2-opt local search based on the input size and time budget: GA for ≤500 stops with ≥30 s budget, ACO for ≥500 stops, 2-opt for re-optimization of an already-optimized route. The selector is configurable per tenant; the conformance suite verifies that every algorithm produces a feasible solution within 5% of the best-found cost on the standard benchmark set.

## A.3 Real-time tracking protocol

GPS telemetry flows over MQTT QoS-1 with TLS 1.3 and gzip-compressed JSON or Protocol Buffers payloads. The broker publishes per-shipment topics that downstream consumers (operations dashboard, customer notifications, carrier-portal mirrors) subscribe to. Event detection (harsh braking, speeding, geofence violation, temperature excursion, unusual stop) runs at the broker and emits typed events alongside the raw telemetry.

## A.4 ETA prediction protocol

ETA is recomputed every 30 s during transit and on every status change. The model takes current position, destination, current time, day of week, traffic conditions, weather, driver history, vehicle type, remaining stops, and historical similar routes. The model output is a point ETA plus a 95% confidence interval; the confidence interval drives notification fan-out (no notification if the window is below the customer-configured silence threshold).

## A.5 Last-mile clustering protocol

Last-mile delivery clusters destinations using K-means or DBSCAN depending on density. Cluster centroids feed the locker-assignment, click-and-collect, and parking-prediction sub-protocols. The clustering is re-run hourly during the active delivery window so newly-added orders fold into the existing plan with minimal re-ordering of completed stops.

## A.6 Replay defence

Standard 96-bit nonce + 300-second skew window + 600-second seen-nonce cache. WebSocket subscriptions additionally require a session-bound bearer token rotated every hour.


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
