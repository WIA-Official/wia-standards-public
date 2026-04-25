# WIA-AUTO-025: Mobility-as-a-Service Specification v1.0

> **Standard ID:** WIA-AUTO-025
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Automotive & Mobility Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [MaaS Integration Levels](#2-maas-integration-levels)
3. [Multimodal Transport Integration](#3-multimodal-transport-integration)
4. [Journey Planning Algorithms](#4-journey-planning-algorithms)
5. [Unified Ticketing and Payment](#5-unified-ticketing-and-payment)
6. [Real-time Service Information](#6-real-time-service-information)
7. [Subscription Models](#7-subscription-models)
8. [Accessibility Features](#8-accessibility-features)
9. [Data Formats and Standards](#9-data-formats-and-standards)
10. [API Interface Specification](#10-api-interface-specification)
11. [Privacy and Data Protection](#11-privacy-and-data-protection)
12. [Carbon Footprint Tracking](#12-carbon-footprint-tracking)
13. [References](#13-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines a comprehensive framework for Mobility-as-a-Service (MaaS), which integrates various forms of transport services into a single, accessible mobility platform that can be accessed on-demand through digital interfaces.

### 1.2 Scope

The standard covers:
- MaaS platform architecture and integration levels
- Multimodal journey planning and optimization
- Unified ticketing and payment systems
- Real-time information and service updates
- Subscription and pricing models
- Accessibility and inclusive design
- Data formats and API specifications
- Privacy, security, and data protection

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard aims to create equitable, sustainable, and efficient urban mobility systems that serve all members of society, reduce environmental impact, and improve quality of life through accessible transportation options.

### 1.4 Terminology

- **MaaS (Mobility-as-a-Service)**: Integration of various transport modes into a unified platform
- **Multimodal Journey**: Trip involving two or more transport modes
- **First Mile / Last Mile**: Journey segments between home/destination and main transit
- **Mode**: Type of transportation (bus, bike, train, etc.)
- **Leg**: Single segment of a journey using one mode
- **Interchange**: Transfer point between different modes
- **GTFS**: General Transit Feed Specification
- **NeTEx**: Network Timetable Exchange format
- **SIRI**: Service Interface for Real-time Information
- **MDS**: Mobility Data Specification
- **TOMP**: Transport Operator Mobility-as-a-service Provider API

---

## 2. MaaS Integration Levels

### 2.1 Level Classification

MaaS systems are classified into five integration levels based on the 관련 분야 자료 framework:

#### 2.1.1 Level 0: No Integration
```
Characteristics:
- Separate services with individual apps
- No unified information or booking
- Manual journey planning required
- Multiple payment methods

User Experience:
- Install separate app for each service
- Plan journey manually across services
- Pay separately for each leg
```

#### 2.1.2 Level 1: Information Integration
```
Characteristics:
- Unified journey planning interface
- Aggregated schedules and routes
- Still requires multiple apps for booking
- No payment integration

Technical Requirements:
- API integration with all operators
- Real-time data synchronization
- Common data format (GTFS, NeTEx)
- Journey planning algorithm

User Experience:
- Single app shows all options
- Comparison of modes and costs
- Links to operator apps for booking
```

#### 2.1.3 Level 2: Booking & Payment Integration
```
Characteristics:
- Single app for planning and booking
- Unified payment interface
- One account for all services
- Integrated ticketing

Technical Requirements:
- Payment gateway integration
- Unified authentication system
- Booking API for all operators
- Digital ticket generation
- Receipt management

User Experience:
- Plan, book, and pay in one app
- Single payment method for all modes
- Digital tickets in one place
- Unified trip history
```

#### 2.1.4 Level 3: Service Integration
```
Characteristics:
- Subscription packages available
- Bundled mobility credits
- Service customization
- Loyalty programs

Technical Requirements:
- Subscription management system
- Credit allocation and tracking
- Dynamic pricing engine
- Usage analytics
- Recommendation system

User Experience:
- Monthly subscription with included trips
- Flexible mobility budget
- Personalized service packages
- Rewards for sustainable choices
```

#### 2.1.5 Level 4: Societal Integration
```
Characteristics:
- Coordinated with urban planning
- Policy integration (parking, congestion)
- Sustainability goals alignment
- Social welfare considerations

Technical Requirements:
- City infrastructure integration
- Policy enforcement systems
- Impact measurement tools
- Open data platforms
- Multi-stakeholder coordination

User Experience:
- Seamless city-wide mobility
- Incentives for green travel
- Priority lanes for shared transport
- Integrated with smart city services
```

### 2.2 Integration Level Assessment

```typescript
interface IntegrationLevel {
  level: 0 | 1 | 2 | 3 | 4;
  capabilities: {
    informationIntegration: boolean;
    bookingIntegration: boolean;
    paymentIntegration: boolean;
    subscriptionServices: boolean;
    policyCoordination: boolean;
  };
  score: number; // 0-100
}
```

---

## 3. Multimodal Transport Integration

### 3.1 Transport Mode Classification

#### 3.1.1 Public Transit
```
Fixed-Route Services:
- Bus (local, express, BRT)
- Metro / Subway
- Tram / Light Rail
- Commuter Rail
- Regional Rail
- Intercity Train

Characteristics:
- Fixed routes and schedules
- High capacity
- Lower cost per passenger
- Predictable service times
```

#### 3.1.2 Shared Mobility
```
Micromobility:
- Bike-share (docked, dockless)
- E-bike share
- E-scooter share
- Moped share

Vehicle Sharing:
- Car-share (station-based, free-floating)
- Ride-hail / Ride-share
- Taxi services
- Peer-to-peer car rental

Characteristics:
- On-demand availability
- Flexible origins/destinations
- Variable pricing
- Real-time availability tracking
```

#### 3.1.3 Active Transportation
```
Human-Powered:
- Walking
- Personal bicycle
- Personal scooter

Characteristics:
- Zero emissions
- Health benefits
- No operational cost
- Limited range and speed
```

### 3.2 Mode Integration Matrix

| From Mode | To Mode | Interchange Requirements |
|-----------|---------|--------------------------|
| Walking → Bus | Bus stop, real-time arrival info |
| Bus → Metro | Transit center, synchronized schedules |
| Metro → Bike-share | Bike station at metro exit, availability data |
| Bike-share → Train | Bike parking, docking station |
| Train → Ride-hail | Pickup zone, arrival coordination |
| E-scooter → Metro | Scooter parking zone, lock mechanism |

### 3.3 First Mile / Last Mile Solutions

```
Problem Statement:
- Average person willing to walk 400-800m to transit
- Beyond this distance, transit becomes less attractive
- Private vehicle often chosen for convenience

Solutions:
1. Micromobility Integration
   - Bike/scooter share at transit stations
   - Real-time availability display
   - Reserved vehicles for arriving passengers

2. Feeder Services
   - Shuttle buses to main transit lines
   - On-demand microtransit
   - Community ride-sharing

3. Infrastructure
   - Safe pedestrian paths
   - Covered walkways
   - Bike lanes connecting to stations
   - Weather-protected waiting areas

Integration Requirements:
- Availability data at journey planning stage
- Seamless booking across modes
- Coordinated timing (e.g., bike available when train arrives)
- Unified payment for entire journey
```

### 3.4 Interchange Optimization

```
Interchange Quality Factors:

Physical:
- Walking distance between modes
- Accessibility (elevators, ramps)
- Wayfinding signage
- Weather protection
- Safety and lighting

Temporal:
- Scheduled connection times
- Transfer time allowance
- Service frequency alignment
- Real-time delay handling

Information:
- Live arrival/departure displays
- Platform/bay announcements
- Delay notifications
- Alternative route suggestions

Optimization Formula:
InterchangeQuality = w₁(PhysicalScore) + w₂(TemporalScore) + w₃(InformationScore)

Where:
- w₁, w₂, w₃ are weights (sum to 1.0)
- Scores range from 0-100
- Minimum acceptable quality: 60
```

---

## 4. Journey Planning Algorithms

### 4.1 Multi-Objective Optimization

Journey planning in MaaS must optimize across multiple objectives:

```
Primary Objectives:
1. Travel Time (minimize)
2. Cost (minimize)
3. Carbon Emissions (minimize)
4. Comfort (maximize)
5. Reliability (maximize)

Secondary Objectives:
6. Walking Distance (minimize)
7. Number of Transfers (minimize)
8. Accessibility (maximize)
9. Safety (maximize)
10. Exercise (optional maximize)
```

### 4.2 Optimization Algorithms

#### 4.2.1 Dijkstra-Based Shortest Path

```
Algorithm: Time-Dependent Dijkstra
Purpose: Find fastest route considering schedules

Input:
- Origin (lat, lng, time)
- Destination (lat, lng)
- Departure or arrival time
- Available modes
- User preferences

Process:
1. Build multimodal graph
   - Nodes: stops, stations, bike docks, etc.
   - Edges: travel segments with time costs

2. Add temporal dimension
   - Edge costs vary by time of day
   - Scheduled services: discrete time points
   - On-demand: average wait time

3. Apply modified Dijkstra
   - Cost = travel_time + wait_time + transfer_penalty
   - Track arrival times at each node
   - Prune dominated paths

Output:
- Optimal route with time costs
- Alternative routes (Pareto-optimal set)
```

#### 4.2.2 Multi-Criteria Decision Analysis (MCDA)

```
Algorithm: Weighted Sum Method with Pareto Filtering

Step 1: Generate candidate routes
- Use multiple single-objective optimizations
- Time-optimal, cost-optimal, carbon-optimal
- K-shortest paths algorithm

Step 2: Evaluate each route
For each route r, calculate scores:
- time_score(r) = normalize(travel_time)
- cost_score(r) = normalize(total_cost)
- carbon_score(r) = normalize(co2_emissions)
- comfort_score(r) = comfort_rating(r)

Step 3: Pareto filtering
- Remove dominated routes
- Route A dominates B if better in all criteria

Step 4: Weighted scoring
For remaining routes:
score(r) = Σ[wᵢ × normalized_score_i(r)]

Where:
- wᵢ = user weight for criterion i
- Σwᵢ = 1.0
- normalized_score ∈ [0, 1]

Step 5: Rank and present
- Sort by weighted score
- Show top 3-5 options
- Display trade-offs clearly
```

#### 4.2.3 Real-Time Re-Routing

```
Trigger Conditions:
- Scheduled service delayed > 5 minutes
- Service cancelled
- Station/stop closed
- Vehicle at capacity (shared mobility)
- Traffic congestion detected

Re-routing Algorithm:
1. Detect disruption
   - Real-time data monitoring
   - Threshold-based triggers

2. Calculate impact
   - Estimated delay to destination
   - If delay < 5 minutes: notify user, no action
   - If delay > 5 minutes: calculate alternatives

3. Generate alternatives
   - Re-run journey planner from current location
   - Consider already-paid segments
   - Minimize additional cost

4. Present options
   - Continue with delay (show new ETA)
   - Switch to alternative route
   - Cancel and refund (if applicable)

5. User decision
   - Automatic switch if improvement > 15 minutes
   - Otherwise, prompt user
   - Apply changes immediately upon selection
```

### 4.3 Journey Planning API Specification

```typescript
interface JourneyPlanRequest {
  origin: Location;
  destination: Location;
  time: {
    type: 'departure' | 'arrival';
    datetime: Date;
  };
  preferences: {
    optimize: 'time' | 'cost' | 'carbon' | 'comfort' | 'balanced';
    modes: TransportMode[];
    maxWalkingDistance?: number; // meters
    maxTransfers?: number;
    accessibility?: AccessibilityRequirements;
    avoidTolls?: boolean;
    carbonPriority?: 'low' | 'medium' | 'high';
  };
  user?: {
    id: string;
    subscription?: SubscriptionPlan;
    mobilityProfile?: MobilityProfile;
  };
}

interface JourneyPlanResponse {
  journeys: Journey[];
  metadata: {
    searchTime: number; // ms
    alternativesCount: number;
    dataFreshness: Date;
  };
}

interface Journey {
  id: string;
  legs: Leg[];
  summary: {
    duration: number; // seconds
    distance: number; // meters
    cost: Money;
    carbonEmissions: number; // kg CO2
    walkingDistance: number; // meters
    transfers: number;
    departureTime: Date;
    arrivalTime: Date;
    comfortScore: number; // 0-100
    reliabilityScore: number; // 0-100
  };
  accessibility: AccessibilityInfo;
  realTimeUpdates: boolean;
}

interface Leg {
  mode: TransportMode;
  from: Stop;
  to: Stop;
  departureTime: Date;
  arrivalTime: Date;
  duration: number; // seconds
  distance: number; // meters
  operator?: Operator;
  route?: Route;
  vehicle?: Vehicle;
  booking?: BookingInfo;
  realTime: boolean;
  instructions?: string[];
  geometry?: GeoJSON.LineString;
}
```

---

## 5. Unified Ticketing and Payment

### 5.1 Digital Ticketing Architecture

```
Ticketing System Components:

1. Ticket Generation
   - QR codes (2D barcodes)
   - NFC tokens
   - Bluetooth Low Energy (BLE)
   - Account-based ticketing

2. Ticket Validation
   - Entry/exit gates
   - On-board validators
   - Conductor scanning
   - GPS-based (for micromobility)

3. Ticket Types
   - Single journey
   - Day pass
   - Multi-ride bundles
   - Monthly subscriptions
   - Annual passes
```

### 5.2 Payment Integration

#### 5.2.1 Payment Methods

```
Supported Payment Types:

1. Credit/Debit Cards
   - Visa, Mastercard, Amex
   - PCI-DSS compliance required
   - Tokenization for security

2. Mobile Wallets
   - Apple Pay
   - Google Pay
   - Samsung Pay
   - Carrier billing

3. Account Balance
   - Pre-loaded credits
   - Auto-reload options
   - Gift card redemption

4. Subscription Credits
   - Monthly allowance
   - Rollover policies
   - Bonus credits

5. External Accounts
   - PayPal
   - Venmo
   - Bank transfers (ACH/SEPA)
```

#### 5.2.2 Fare Calculation

```
Fare Calculation Logic:

Base Fare Model:
fare = base_price + (distance × distance_rate) + transfer_fees

Distance-Based:
- Common for trains, buses
- Example: $2.00 base + $0.15/km

Time-Based:
- Common for shared vehicles
- Example: $1.00 unlock + $0.25/minute

Zone-Based:
- Urban areas divided into zones
- Price increases with zone crossings
- Example: 1 zone = $2, 2 zones = $3.50, 3+ zones = $5

Flat Fare:
- Fixed price regardless of distance
- Simple for users, easier to manage
- Example: $2.75 per ride

Dynamic Pricing:
- Varies by demand, time, events
- Surge pricing for ride-hail
- Off-peak discounts for transit

Subscription Discounts:
if user.has_subscription:
    fare = fare × (1 - discount_rate)
    deduct_from_credits(fare)
else:
    charge_payment_method(fare)
```

#### 5.2.3 Payment Processing Flow

```
1. Journey Selection
   ↓
2. Fare Calculation
   - Get base fares from operators
   - Apply discounts/subscriptions
   - Calculate total cost
   ↓
3. Payment Authorization
   - Check payment method validity
   - Pre-authorize amount
   - Hold funds/credits
   ↓
4. Ticket Generation
   - Create digital ticket
   - Generate QR/NFC token
   - Send to user's app
   ↓
5. Journey Completion
   - Validate ticket usage
   - Calculate final fare (if variable)
   - Process payment
   ↓
6. Receipt & Settlement
   - Send receipt to user
   - Settle with transport operators
   - Update user's trip history
```

### 5.3 Multi-Operator Settlement

```
Settlement Process:

1. Fare Collection
   - MaaS platform collects all payments
   - Holds in escrow account

2. Revenue Allocation
   For each journey with multiple operators:

   operator_share = (operator_distance / total_distance) × total_fare

   Adjustments:
   - Platform fee (5-15% of transaction)
   - Premium for real-time data provision
   - Incentives for sustainable modes

3. Settlement Period
   - Daily settlements for large operators
   - Weekly/monthly for smaller operators
   - Automated bank transfers

4. Reconciliation
   - Match ticket validations with bookings
   - Handle disputes (no-shows, cancellations)
   - Audit trail for all transactions
```

---

## 6. Real-time Service Information

### 6.1 Data Sources

```
Real-Time Data Types:

1. Vehicle Locations
   - GPS tracking of buses, trains, trams
   - Update frequency: 10-30 seconds
   - Format: GTFS-RT Vehicle Positions

2. Service Alerts
   - Delays, cancellations, diversions
   - Station closures, track work
   - Format: GTFS-RT Service Alerts

3. Trip Updates
   - Predicted arrival/departure times
   - Stop sequence changes
   - Format: GTFS-RT Trip Updates

4. Occupancy
   - Real-time crowding levels
   - Seats/standing room available
   - Format: GTFS-RT extensions

5. Shared Mobility Availability
   - Bike/scooter locations and battery
   - Car-share availability
   - Format: GBFS, MDS
```

### 6.2 Data Aggregation and Processing

```
Real-Time Pipeline:

1. Data Ingestion
   ┌─ Operator A API ─┐
   ├─ Operator B API ─┤
   ├─ Operator C API ─┤ → Message Queue (Kafka/RabbitMQ)
   └─ Operator N API ─┘

2. Normalization
   - Convert to standard format (GTFS-RT)
   - Timestamp synchronization
   - Coordinate system alignment (WGS84)

3. Validation
   - Check data quality
   - Flag anomalies (e.g., vehicle teleporting)
   - Fill gaps with predictions

4. Fusion
   - Combine scheduled and real-time data
   - Apply prediction models
   - Calculate confidence intervals

5. Distribution
   - Push to mobile apps via WebSocket
   - Update API cache (Redis)
   - Store historical data (TimescaleDB)

Latency Target: < 3 seconds end-to-end
```

### 6.3 Predictive Analytics

```
Arrival Time Prediction:

Historical Model:
- Collect past trip data
- Extract features: time of day, day of week, weather, events
- Train regression model (Random Forest, XGBoost)

Real-Time Adjustment:
predicted_arrival = historical_baseline + current_delay + traffic_factor

where:
- historical_baseline: average trip time for this time/day
- current_delay: vehicle's current delay relative to schedule
- traffic_factor: real-time traffic data adjustment

Confidence Interval:
- 50% confidence: ±2 minutes
- 80% confidence: ±5 minutes
- 95% confidence: ±10 minutes

Occupancy Prediction:
- Boarding/alighting patterns by stop
- Event-based surges
- Seasonal variations

Model: LSTM neural network with time series data
Update: Retrain weekly with new data
```

---

## 7. Subscription Models

### 7.1 Subscription Tiers

```
Tier 1: Basic (Pay-Per-Use)
- No monthly fee
- Pay full price per trip
- No commitments
- Best for: Occasional users (<10 trips/month)

Tier 2: Light ($49/month)
- 20 trips included
- Public transit unlimited
- 50% off bike/scooter share
- Best for: Regular commuters

Tier 3: Standard ($99/month)
- 60 trips included (mixed modes)
- All public transit unlimited
- Bike/scooter included (up to 30 min/trip)
- 20% off ride-hail
- Best for: Daily users, no car

Tier 4: Premium ($199/month)
- Unlimited trips (all modes)
- Priority booking
- Carbon offset included
- 24/7 support
- Best for: Heavy users, business travelers

Tier 5: Family ($299/month)
- Unlimited for up to 4 members
- Shared credit pool
- Kid-friendly features
- Best for: Families without car
```

### 7.2 Credit-Based System

```
Mobility Credits:

Definition:
- Currency unit within MaaS platform
- 1 credit ≈ $1 equivalent value
- Used to "purchase" trips from allowance

Allocation:
Standard Tier Example:
- Base: 60 credits/month
- Bonus: 10 credits for sustainable mode usage
- Rollover: Up to 20 credits to next month
- Top-up: Purchase additional credits at $0.90/credit

Usage:
Trip Cost in Credits:
- Bus: 2 credits
- Metro: 3 credits
- Bike-share (30 min): 2 credits
- E-scooter (30 min): 3 credits
- Ride-hail (per km): 1.5 credits

Advantages:
- User controls mode mix
- Incentivizes efficient travel
- Predictable costs
- Operator neutrality
```

### 7.3 Dynamic Pricing and Incentives

```
Demand-Based Pricing:

Off-Peak Discounts:
if current_time in OFF_PEAK_HOURS:
    fare = base_fare × 0.75  # 25% discount

Peak Surcharges:
if current_time in PEAK_HOURS:
    if demand_ratio > 0.9:  # >90% capacity
        fare = base_fare × 1.25  # 25% surcharge

Sustainability Bonuses:
carbon_saved = (car_trip_carbon - chosen_trip_carbon)
bonus_credits = carbon_saved × CARBON_CREDIT_RATE

if carbon_saved > 5kg:  # Significant carbon reduction
    bonus_credits += 2  # Extra reward

Loyalty Rewards:
monthly_trips = get_user_trip_count(user, current_month)

if monthly_trips > 50:
    next_month_discount = 0.10  # 10% off
elif monthly_trips > 100:
    next_month_discount = 0.15  # 15% off
```

---

## 8. Accessibility Features

### 8.1 Universal Design Principles

```
Accessibility Requirements (following WCAG 2.1 AAA, ADA, EAA):

1. Physical Accessibility
   - Wheelchair-accessible routes
   - Step-free access
   - Elevator/ramp availability
   - Wide doors and aisles
   - Priority seating

2. Visual Accessibility
   - High-contrast UI (min 7:1 ratio)
   - Screen reader compatibility
   - Audio announcements
   - Tactile paving
   - Braille signage

3. Hearing Accessibility
   - Visual alerts for announcements
   - Vibration notifications
   - Text-based communication
   - Hearing loop systems

4. Cognitive Accessibility
   - Simple, clear instructions
   - Pictograms and icons
   - Consistent UI patterns
   - Error prevention and recovery
   - Multiple input methods
```

### 8.2 Accessible Journey Planning

```typescript
interface AccessibilityRequirements {
  mobilityAids: {
    wheelchair: boolean;
    walker: boolean;
    cane: boolean;
    serviceAnimal: boolean;
  };

  sensoryNeeds: {
    visualImpairment: 'none' | 'low-vision' | 'blind';
    hearingImpairment: 'none' | 'hard-of-hearing' | 'deaf';
    speechImpairment: boolean;
  };

  cognitiveSupport: {
    simplifiedInstructions: boolean;
    extraTime: boolean;
    companionRequired: boolean;
  };

  preferences: {
    maxWalkingDistance: number; // meters
    avoidStairs: boolean;
    requireSeating: boolean;
    preferQuietCarriage: boolean;
  };
}

// Journey filtering for accessibility
function filterAccessibleRoutes(
  routes: Journey[],
  requirements: AccessibilityRequirements
): Journey[] {
  return routes.filter(route => {
    // Check each leg for accessibility
    return route.legs.every(leg => {
      if (requirements.mobilityAids.wheelchair) {
        if (!leg.wheelchairAccessible) return false;
        if (leg.requiresStairs && requirements.preferences.avoidStairs) return false;
      }

      if (requirements.sensoryNeeds.visualImpairment === 'blind') {
        if (!leg.audioGuidanceAvailable) return false;
        if (!leg.tactilePathsAvailable) return false;
      }

      return true;
    });
  });
}
```

### 8.3 Assisted Travel Services

```
Support Services:

1. Travel Assistance
   - Pre-booking of assistance
   - Staff meet-and-greet
   - Help with boarding/alighting
   - Luggage assistance

2. Real-Time Support
   - 24/7 accessibility helpline
   - In-app chat with support
   - Video call assistance
   - Emergency contact

3. Companion Services
   - Companion travel (free or discounted)
   - Group booking for care homes
   - Chaperone matching

4. Customized Routing
   - Avoid specific barriers
   - Include rest stops
   - Extra transfer time
   - Guaranteed seating

Request Format:
{
  "assistanceType": "wheelchair",
  "noticeTime": "24hours",
  "pickupLocation": "station entrance",
  "dropoffLocation": "platform 3",
  "specialRequirements": "heavy luggage, requires ramp"
}
```

---

## 9. Data Formats and Standards

### 9.1 GTFS (General Transit Feed Specification)

```
Static GTFS:
File Structure:
- agency.txt
- stops.txt
- routes.txt
- trips.txt
- stop_times.txt
- calendar.txt
- calendar_dates.txt
- fare_attributes.txt
- fare_rules.txt
- shapes.txt

Example: stops.txt
stop_id,stop_name,stop_lat,stop_lon,wheelchair_boarding
S001,Market St & 5th Ave,37.7749,-122.4194,1
S002,Market St & 7th Ave,37.7758,-122.4189,1

GTFS-Realtime:
1. Vehicle Positions
message FeedMessage {
  FeedHeader header = 1;
  repeated FeedEntity entity = 2;
}

message VehiclePosition {
  optional TripDescriptor trip = 1;
  optional Position position = 2;
  optional uint64 timestamp = 4;
  optional VehicleDescriptor vehicle = 8;
}

2. Trip Updates
message TripUpdate {
  optional TripDescriptor trip = 1;
  repeated StopTimeUpdate stop_time_update = 2;
  optional uint64 timestamp = 4;
  optional int32 delay = 5;
}

3. Service Alerts
message Alert {
  repeated TimeRange active_period = 1;
  repeated EntitySelector informed_entity = 5;
  optional Cause cause = 6;
  optional Effect effect = 7;
  optional TranslatedString header_text = 10;
  optional TranslatedString description_text = 11;
}
```

### 9.2 NeTEx (Network Timetable Exchange)

```xml
NeTEx Structure:

<PublicationDelivery>
  <ResourceFrame>
    <!-- Organizations, operators -->
  </ResourceFrame>

  <SiteFrame>
    <!-- Stops, stations, infrastructure -->
    <stopPlaces>
      <StopPlace id="SF:StopPlace:001">
        <Name>Market Street Station</Name>
        <Centroid>
          <Location>
            <Longitude>-122.4194</Longitude>
            <Latitude>37.7749</Latitude>
          </Location>
        </Centroid>
        <AccessibilityAssessment>
          <MobilityImpairedAccess>true</MobilityImpairedAccess>
          <WheelchairAccess>true</WheelchairAccess>
        </AccessibilityAssessment>
      </StopPlace>
    </stopPlaces>
  </SiteFrame>

  <ServiceFrame>
    <!-- Routes, lines, patterns -->
  </ServiceFrame>

  <TimetableFrame>
    <!-- Schedules, journey patterns -->
  </TimetableFrame>
</PublicationDelivery>
```

### 9.3 MDS (Mobility Data Specification)

```json
{
  "version": "2.0.0",
  "data": {
    "vehicles": [
      {
        "device_id": "abc123",
        "provider_id": "provider-uuid",
        "vehicle_id": "bike-001",
        "vehicle_type": "bicycle",
        "propulsion_types": ["human"],
        "last_event_time": "2025-01-15T10:30:00Z",
        "last_event_type": "available",
        "last_event_location": {
          "type": "Point",
          "coordinates": [-122.4194, 37.7749]
        },
        "battery_pct": null,
        "current_location": {
          "type": "Point",
          "coordinates": [-122.4194, 37.7749]
        }
      }
    ]
  }
}
```

### 9.4 TOMP-API (Transport Operator Mobility-as-a-service Provider)

```
TOMP API Endpoints:

1. Planning
POST /planning/inquiries
{
  "from": {"coordinates": {"lng": -122.4194, "lat": 37.7749}},
  "to": {"coordinates": {"lng": -122.0839, "lat": 37.3861}},
  "departureTime": "2025-01-15T09:00:00Z",
  "travellers": 1
}

Response: Available options with pricing

2. Booking
POST /bookings
{
  "planningId": "plan-123",
  "customer": {"id": "user-456"},
  "extraData": {}
}

Response: Booking confirmation, ticket

3. Activation
POST /legs/{legId}/events
{
  "event": "PREPARE",
  "time": "2025-01-15T09:00:00Z"
}

4. Trip Monitoring
GET /legs/{legId}
Response: Current status, location, ETA
```

---

## 10. API Interface Specification

### 10.1 REST API Endpoints

```
Base URL: https://api.maas-platform.com/v1

Authentication:
- API Key in header: X-API-Key: {key}
- OAuth 2.0 for user actions
- JWT tokens for session management

Core Endpoints:

1. Journey Planning
POST /journeys/plan
Content-Type: application/json

Request:
{
  "origin": {"lat": 37.7749, "lng": -122.4194},
  "destination": {"lat": 37.3861, "lng": -122.0839},
  "time": {"type": "departure", "datetime": "2025-01-15T09:00:00Z"},
  "preferences": {
    "optimize": "time",
    "modes": ["bus", "train", "bike-share"],
    "maxWalkingDistance": 1000
  }
}

Response: 200 OK
{
  "journeys": [...],
  "metadata": {...}
}

2. Booking
POST /bookings
{
  "journeyId": "j-12345",
  "paymentMethod": "subscription",
  "passengers": 1
}

Response: 201 Created
{
  "bookingId": "b-67890",
  "status": "confirmed",
  "tickets": [...],
  "totalCost": {"amount": 8.50, "currency": "USD"}
}

3. Real-Time Updates
WebSocket: wss://api.maas-platform.com/v1/realtime

Subscribe to journey updates:
{
  "action": "subscribe",
  "journeyId": "j-12345"
}

Receive updates:
{
  "type": "leg_update",
  "legId": "l-001",
  "delay": 120,
  "newETA": "2025-01-15T09:12:00Z"
}

4. Mobility Options
GET /mobility/options?lat=37.7749&lng=-122.4194&radius=500&modes=bike-share,scooter

Response:
{
  "options": [
    {
      "mode": "bike-share",
      "provider": "BayWheels",
      "available": 5,
      "location": {"lat": 37.7750, "lng": -122.4190},
      "distance": 45
    }
  ]
}

5. User Subscriptions
GET /users/{userId}/subscription

Response:
{
  "tier": "standard",
  "credits": {"remaining": 35, "total": 60},
  "validUntil": "2025-02-15T00:00:00Z",
  "autoRenew": true
}

6. Trip History
GET /users/{userId}/trips?from=2025-01-01&to=2025-01-31

Response:
{
  "trips": [
    {
      "id": "t-111",
      "date": "2025-01-15",
      "journey": {...},
      "cost": 8.50,
      "carbonSaved": 2.3
    }
  ],
  "summary": {
    "totalTrips": 42,
    "totalCost": 245.00,
    "totalCarbon": 96.6
  }
}
```

### 10.2 Error Handling

```
Standard Error Response:
{
  "error": {
    "code": "INVALID_JOURNEY",
    "message": "Origin and destination are the same",
    "details": {
      "field": "destination",
      "reason": "must_differ_from_origin"
    },
    "timestamp": "2025-01-15T10:30:00Z",
    "requestId": "req-abc123"
  }
}

HTTP Status Codes:
200 OK - Success
201 Created - Resource created
400 Bad Request - Invalid input
401 Unauthorized - Missing/invalid authentication
403 Forbidden - Insufficient permissions
404 Not Found - Resource not found
409 Conflict - Resource conflict (e.g., already booked)
422 Unprocessable Entity - Validation failed
429 Too Many Requests - Rate limit exceeded
500 Internal Server Error - Server error
503 Service Unavailable - Temporary outage

Error Codes:
- INVALID_JOURNEY: Journey planning parameters invalid
- NO_ROUTES_FOUND: No routes available for criteria
- BOOKING_FAILED: Booking could not be completed
- PAYMENT_DECLINED: Payment method declined
- INSUFFICIENT_CREDITS: Not enough subscription credits
- SERVICE_UNAVAILABLE: Transport service temporarily unavailable
- REAL_TIME_DATA_STALE: Real-time data too old (>5 min)
```

---

## 11. Privacy and Data Protection

### 11.1 Data Collection and Usage

```
Data Categories:

1. Account Data
   - Name, email, phone number
   - Payment information (tokenized)
   - Date of birth (age verification)
   - Purpose: Account management, billing
   - Retention: Account lifetime + 7 years

2. Location Data
   - Trip origins and destinations
   - GPS trace during active trips
   - Purpose: Journey planning, service provision
   - Retention: 90 days (raw), 2 years (aggregated)

3. Travel History
   - Past journeys and bookings
   - Mode preferences
   - Purpose: Personalization, analytics
   - Retention: 2 years

4. Usage Analytics
   - App interactions
   - Feature usage
   - Purpose: Service improvement
   - Retention: 1 year (anonymized)

5. Device Information
   - Device type, OS version
   - App version
   - Purpose: Technical support, compatibility
   - Retention: Until app uninstalled
```

### 11.2 GDPR Compliance

```
User Rights:

1. Right to Access
   - Download all personal data
   - Format: JSON, CSV, PDF
   - Delivery: Within 30 days

2. Right to Rectification
   - Correct inaccurate data
   - Update preferences
   - Immediate effect

3. Right to Erasure ("Right to be Forgotten")
   - Delete account and associated data
   - Exceptions: Legal obligations (billing records)
   - Process: 30-day grace period, then permanent deletion

4. Right to Data Portability
   - Export data in machine-readable format
   - Transfer to another service
   - Format: JSON with schema documentation

5. Right to Restrict Processing
   - Limit data usage (e.g., marketing)
   - Maintain account but pause non-essential processing

6. Right to Object
   - Opt-out of profiling
   - Opt-out of automated decision-making
   - Opt-out of marketing communications

Implementation:
- Self-service data export in app
- Privacy settings dashboard
- Automated data deletion workflows
- Audit logs for all data access
```

### 11.3 Anonymization and Aggregation

```
Anonymization Techniques:

1. Spatial Generalization
   - Round coordinates to 100m grid
   - Remove exact addresses
   - Example:
     Original: (37.774929, -122.419416)
     Anonymized: (37.7750, -122.4190)

2. Temporal Generalization
   - Round timestamps to 15-minute intervals
   - Remove exact seconds
   - Example:
     Original: 2025-01-15 09:17:43
     Anonymized: 2025-01-15 09:15:00

3. Trajectory Truncation
   - Remove first/last 200m of trips
   - Prevents home/work identification
   - Preserves corridor-level patterns

4. K-Anonymity
   - Ensure each record matches ≥k other records
   - Typical k = 5 or 10
   - Suppress rare origin-destination pairs

5. Differential Privacy
   - Add calibrated noise to aggregate statistics
   - ε-differential privacy (ε = 0.1 - 1.0)
   - Example:
     True count: 1,247 trips
     Published: 1,251 trips (noise: +4)

Aggregation Rules:
- Minimum group size: 10 users
- Suppress metrics with <10 contributors
- Aggregate by: hour, day, week, month
- Geographic: postal code, neighborhood, city
```

### 11.4 Security Measures

```
Security Controls:

1. Encryption
   - Data at rest: AES-256
   - Data in transit: TLS 1.3
   - Payment data: PCI-DSS Level 1

2. Authentication
   - Multi-factor authentication (MFA)
   - Biometric options (fingerprint, face)
   - Session timeout: 15 minutes inactive

3. Authorization
   - Role-based access control (RBAC)
   - Principle of least privilege
   - Audit logs for admin actions

4. API Security
   - Rate limiting: 100 req/min per user
   - API key rotation: Every 90 days
   - Input validation and sanitization

5. Incident Response
   - Breach notification: Within 72 hours
   - Incident response team
   - Regular security audits (quarterly)
```

---

## 12. Carbon Footprint Tracking

### 12.1 Emission Calculation Methodology

```
Carbon Emissions Formula:

For each transport mode:
CO2 = distance × emission_factor × occupancy_factor

Emission Factors (kg CO2 per passenger-km):

Mode                    | Urban    | Intercity
------------------------|----------|----------
Walking                 | 0.000    | 0.000
Cycling (personal)      | 0.000    | 0.000
E-bike                  | 0.005    | 0.005
E-scooter               | 0.008    | 0.008
Bus (diesel)            | 0.089    | 0.068
Bus (electric)          | 0.027    | 0.027
Metro/Subway            | 0.041    | -
Tram (electric)         | 0.035    | -
Train (electric)        | 0.014    | 0.041
Train (diesel)          | -        | 0.085
Ride-hail (solo)        | 0.192    | 0.171
Ride-share (pooled)     | 0.096    | 0.086
Car-share (electric)    | 0.053    | 0.053
Car-share (gas)         | 0.171    | 0.171
Private car (avg)       | 0.192    | 0.171
Flight (economy)        | -        | 0.115

Occupancy Factors:
- Public transit: Actual occupancy / capacity
- Shared vehicles: 1 / number of passengers
- Personal vehicles: Assumed 1.5 avg

Example Calculation:
Journey: 10 km by electric bus, 50% full

CO2 = 10 km × 0.027 kg/km × (1 / 0.5)
    = 10 × 0.027 × 2
    = 0.54 kg CO2
```

### 12.2 Carbon Savings Comparison

```
Baseline Comparison:

Default baseline: Private car (gasoline)
Baseline emissions: 0.192 kg CO2/km (urban)

For each journey:
carbon_saved = (baseline_emissions - actual_emissions)

Example:
Chosen mode: Metro (10 km)
Actual CO2: 10 × 0.041 = 0.41 kg

Baseline (car): 10 × 0.192 = 1.92 kg

Carbon saved: 1.92 - 0.41 = 1.51 kg CO2

Percentage reduction: (1.51 / 1.92) × 100 = 79%

Annual Projection:
If user takes this trip daily (260 work days):
Annual savings: 1.51 × 260 = 392.6 kg CO2
Equivalent to: 180 km car trip offset
```

### 12.3 Carbon Offset Integration

```
Offset Programs:

1. Automatic Offsetting
   - Calculate carbon for each trip
   - Purchase carbon credits automatically
   - Cost: $0.01 - 0.02 per kg CO2
   - Include in subscription (Premium tier)

2. User-Initiated Offsetting
   - Monthly carbon report
   - Option to offset all trips
   - Choose offset project (renewable energy, reforestation)

3. Corporate Offsetting
   - Employer-sponsored offsets
   - Commute benefits integration
   - Tax-deductible for businesses

Offset Calculation:
monthly_carbon = sum(all_trip_emissions)
offset_cost = monthly_carbon × carbon_credit_price

Typical cost: $2-5 per month for average user

Certification:
- Verified Carbon Standard (VCS)
- Gold Standard
- Transparent reporting to user
```

---

## 13. References

### 13.1 Standards and Specifications

1. **GTFS**: General Transit Feed Specification
   - https://gtfs.org/
   - Static and Realtime specifications

2. **NeTEx**: Network Timetable Exchange
   - CEN/TS 16614 series
   - European standard for public transport data

3. **SIRI**: Service Interface for Real-time Information
   - CEN/TS 15531 series
   - Real-time public transport data exchange

4. **MDS**: Mobility Data Specification
   - Open Mobility Foundation
   - https://github.com/openmobilityfoundation/mobility-data-specification

5. **GBFS**: General Bikeshare Feed Specification
   - https://github.com/MobilityData/gbfs
   - Shared micromobility data standard

6. **TOMP-API**: Transport Operator Mobility-as-a-service Provider API
   - https://github.com/TOMP-WG/TOMP-API

### 13.2 Accessibility Standards

1. **WCAG 2.1**: Web Content Accessibility Guidelines
   - W3C Recommendation
   - Level AAA compliance target

2. **ADA**: Americans with Disabilities Act
   - US accessibility requirements
   - Public accommodation standards

3. **EAA**: European Accessibility Act
   - EU Directive 2019/882
   - Transport service accessibility

### 13.3 Privacy and Security

1. **GDPR**: General Data Protection Regulation
   - EU Regulation 2016/679
   - Data protection and privacy

2. **PCI-DSS**: Payment Card Industry Data Security Standard
   - Credit card data security
   - Level 1 compliance for large platforms

3. **ISO/IEC 27001**: Information Security Management
   - Security best practices

### 13.4 Research and Publications

1. Sochor, J., Arby, H., Karlsson, I. C. M., & Sarasini, S. (2018). "A topological approach to Mobility as a Service: A proposed tool for understanding requirements and effects, and for aiding the integration of societal goals." *Research in Transportation Business & Management*, 27, 3-14.

2. Jittrapirom, P., Caiati, V., Feneri, A. M., Ebrahimigharehbaghi, S., Alonso González, M. J., & Narayan, J. (2017). "Mobility as a Service: A Critical Review of Definitions, Assessments of Schemes, and Key Challenges." *Urban Planning*, 2(2), 13-25.

3. Hensher, D. A., Ho, C. Q., Mulley, C., Nelson, J. D., Smith, G., & Wong, Y. Z. (2020). "Understanding Mobility as a Service (MaaS): Past, Present and Future." *Elsevier*.

4. Kamargianni, M., & Matyas, M. (2017). "The Business Ecosystem of Mobility as a Service." *96th Transportation Research Board Annual Meeting*.

### 13.5 Related WIA Standards

- **WIA-INTENT**: Intent-based user interfaces
- **WIA-OMNI-API**: Universal API gateway
- **WIA-SOCIAL**: Social networking and collaboration
- **WIA-PAYMENT**: Unified payment processing
- **WIA-CARBON**: Carbon footprint tracking and offsetting
- **WIA-AUTO-001 to AUTO-024**: Automotive standards series

---

## Appendix A: Example Calculations

### A.1 Journey Planning Example

```
Scenario:
- Origin: Downtown SF (37.7749, -122.4194)
- Destination: Mountain View (37.3861, -122.0839)
- Time: Weekday, 9:00 AM departure
- User: Standard subscription, prefers fast route

Step 1: Generate Candidate Routes

Route 1: Metro + Caltrain
- Walk to metro: 0.3 km, 4 min
- Metro to Caltrain: 5 km, 12 min, $2.50
- Walk to Caltrain: 0.2 km, 3 min
- Caltrain to MV: 42 km, 48 min, $7.50
- Walk to destination: 0.4 km, 5 min
Total: 72 min, $10.00, 0.89 kg CO2

Route 2: Bus + Caltrain + Bike-share
- Walk to bus: 0.1 km, 2 min
- Bus to Caltrain: 6 km, 18 min, $2.50
- Walk to Caltrain: 0.1 km, 2 min
- Caltrain to MV: 42 km, 48 min, $7.50
- Bike-share to dest: 1.2 km, 6 min, $2.00 (included in sub)
Total: 76 min, $10.00, 0.62 kg CO2

Route 3: Ride-hail (entire trip)
- Door to door: 48 km, 65 min (traffic), $45.00
Total: 65 min, $45.00, 9.22 kg CO2

Step 2: Score Routes

Weights for "fast" preference:
- Time: 0.5
- Cost: 0.3
- Carbon: 0.2

Normalized scores (0-1, higher better):
Route 1: time=0.90, cost=0.77, carbon=0.90
Route 2: time=0.86, cost=0.77, carbon=0.93
Route 3: time=1.00, cost=0.00, carbon=0.00

Weighted scores:
Route 1: 0.5×0.90 + 0.3×0.77 + 0.2×0.90 = 0.861
Route 2: 0.5×0.86 + 0.3×0.77 + 0.2×0.93 = 0.847
Route 3: 0.5×1.00 + 0.3×0.00 + 0.2×0.00 = 0.500

Step 3: Recommend
Recommended: Route 1 (highest score)
Show Route 2 as alternative (similar cost, greener)
Show Route 3 (fastest, but expensive)
```

### A.2 Subscription Value Analysis

```
Scenario: Daily commuter, 22 work days/month

Without Subscription (Pay-per-use):
- Round trip cost: $20.00 (metro + train both ways)
- Monthly cost: $20 × 22 = $440

With Standard Subscription ($99/month):
- 60 credits included
- Round trip: 10 credits
- Credits needed: 22 × 10 = 220 credits
- Additional credits: 220 - 60 = 160 credits
- Buy at $0.90/credit: 160 × $0.90 = $144
- Total: $99 + $144 = $243

Savings: $440 - $243 = $197/month (45% savings)

With Premium Subscription ($199/month):
- Unlimited trips
- Total: $199

Savings: $440 - $199 = $241/month (55% savings)

Conclusion: Premium tier best for this user
Break-even: 10 round trips per month
```

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA-AUTO-025 Specification v1.0*
*© 2025 SmileStory Inc. / WIA*
*MIT License*
