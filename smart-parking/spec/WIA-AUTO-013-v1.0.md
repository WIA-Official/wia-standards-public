# WIA-AUTO-013: Smart Parking Specification v1.0

> **Standard ID:** WIA-AUTO-013
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Automotive Standards Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Parking Detection Technologies](#2-parking-detection-technologies)
3. [Occupancy States and Transitions](#3-occupancy-states-and-transitions)
4. [Parking Guidance Systems](#4-parking-guidance-systems)
5. [Reservation and Payment Systems](#5-reservation-and-payment-systems)
6. [Real-Time Occupancy Tracking](#6-real-time-occupancy-tracking)
7. [EV Charging Integration](#7-ev-charging-integration)
8. [Data Formats and Protocols](#8-data-formats-and-protocols)
9. [API Specification](#9-api-specification)
10. [Security and Privacy](#10-security-and-privacy)
11. [References](#11-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines a comprehensive standard for smart parking systems that enable efficient parking space detection, real-time occupancy tracking, automated guidance, reservation management, and integrated payment processing.

### 1.2 Scope

The standard covers:
- Parking detection sensor technologies and protocols
- Occupancy state management and transitions
- Smart guidance algorithms and navigation
- Reservation and payment system integration
- Electric vehicle charging coordination
- Data formats, APIs, and communication protocols
- Security, privacy, and compliance requirements

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - Smart parking technology reduces urban congestion, lowers carbon emissions from vehicles circling for parking, and improves quality of life by saving time and reducing stress for drivers worldwide.

### 1.4 Terminology

- **Parking Space**: A designated area for parking a single vehicle
- **Parking Lot**: A collection of parking spaces managed as a unit
- **Occupancy**: The state of a parking space (available, occupied, reserved, etc.)
- **Detection Confidence**: Statistical measure of sensor accuracy (0-1)
- **Dwell Time**: Duration a vehicle occupies a space
- **Turnover Rate**: Number of vehicles using a space per time period

---

## 2. Parking Detection Technologies

### 2.1 Ultrasonic Sensors

Ultrasonic sensors use sound waves to detect vehicle presence through distance measurement.

#### 2.1.1 Technical Specifications

```
Operating Frequency: 40 kHz - 200 kHz
Detection Range: 0.3m - 8m
Accuracy: ±2cm
Detection Time: <100ms
Power Consumption: 50-150 mW
Operating Temperature: -40°C to +85°C
```

#### 2.1.2 Detection Algorithm

```
1. Emit ultrasonic pulse
2. Measure time-of-flight (ToF)
3. Calculate distance: d = (v × t) / 2
   where v = speed of sound (343 m/s at 20°C)
4. Compare with threshold distance
5. If d < threshold: OCCUPIED
6. If d ≥ threshold: AVAILABLE
7. Apply confidence filter (3+ consecutive readings)
```

#### 2.1.3 Accuracy Factors

- **Weather Conditions**: Rain/snow can affect readings (-2% accuracy)
- **Temperature**: Extreme temps alter speed of sound (±1% accuracy)
- **Obstacle Interference**: Adjacent vehicles may cause reflections

**Recommended Use**: Outdoor surface parking lots, overhead mounting

### 2.2 Camera-Based Vision Systems

AI-powered camera systems use computer vision to detect parking occupancy.

#### 2.2.1 Technical Specifications

```
Camera Resolution: Minimum 1920×1080 (Full HD)
Frame Rate: 15-30 fps
Field of View: 80-120 degrees
Processing: Edge AI or cloud-based
Models: YOLO, SSD, Faster R-CNN
Accuracy: 96% (varies by lighting)
```

#### 2.2.2 Detection Pipeline

```
1. Image Acquisition
   - Capture frame from camera
   - Timestamp and metadata tagging

2. Preprocessing
   - Noise reduction (Gaussian blur)
   - Contrast enhancement
   - Geometric correction

3. Object Detection
   - Run neural network inference
   - Detect vehicles with bounding boxes
   - Classification: car, truck, motorcycle, etc.

4. Occupancy Determination
   - Map bounding boxes to parking spaces
   - Calculate overlap percentage
   - If overlap > 60%: OCCUPIED
   - If overlap < 20%: AVAILABLE
   - If 20-60%: UNCERTAIN (require validation)

5. Temporal Filtering
   - Track objects across frames
   - Eliminate false positives
   - Confidence threshold: 90%
```

#### 2.2.3 Advanced Features

- **License Plate Recognition**: Automatic vehicle identification
- **Vehicle Classification**: Size, type, color detection
- **Violation Detection**: Parking outside marked lines
- **Heat Mapping**: Visual occupancy patterns

**Recommended Use**: Multi-level parking garages, covered parking areas

### 2.3 In-Ground Magnetic Sensors

Embedded sensors detect magnetic field disturbances from vehicles.

#### 2.3.1 Technical Specifications

```
Sensor Type: Magnetometer (3-axis)
Sensitivity: 0.1 μT
Detection Threshold: 50 μT change
Response Time: <50ms
Battery Life: 5-10 years
Installation Depth: 50-100mm
Communication: LoRaWAN, NB-IoT, Zigbee
```

#### 2.3.2 Detection Method

```
1. Baseline Calibration
   - Measure ambient magnetic field
   - Store reference values [Bx, By, Bz]

2. Continuous Monitoring
   - Sample magnetic field at 10 Hz
   - Calculate magnitude: |B| = √(Bx² + By² + Bz²)

3. Change Detection
   - ΔB = |B_current| - |B_baseline|
   - If ΔB > 50 μT: Vehicle detected
   - If ΔB < 20 μT: No vehicle

4. State Transition
   - Debounce filter: 3 seconds minimum
   - Report status change to gateway
```

**Recommended Use**: Premium parking spaces, high-value areas, underground garages

### 2.4 LiDAR (Light Detection and Ranging)

LiDAR creates 3D point clouds for precise vehicle detection.

#### 2.4.1 Technical Specifications

```
Wavelength: 905nm or 1550nm
Range: 0.5m - 200m
Accuracy: ±2cm
Resolution: 0.1° - 0.2°
Scan Rate: 10-20 Hz
Points per Second: 100,000+
Weather Resistance: IP67
```

#### 2.4.2 3D Detection Algorithm

```
1. Point Cloud Acquisition
   - Capture 3D scene data
   - Filter noise and outliers

2. Ground Plane Removal
   - RANSAC algorithm
   - Identify and remove ground surface

3. Object Clustering
   - DBSCAN or Euclidean clustering
   - Group points into objects

4. Vehicle Classification
   - Height threshold: >0.5m
   - Volume estimation
   - Shape analysis

5. Occupancy Mapping
   - Map clusters to parking spaces
   - Calculate space occupancy percentage
```

**Recommended Use**: High-traffic areas, autonomous vehicle parking

### 2.5 Hybrid Multi-Sensor Fusion

Combining multiple sensor types increases accuracy and reliability.

#### 2.5.1 Fusion Architecture

```
Sensors → Data Acquisition → Preprocessing → Sensor Fusion → Decision
   ↓           ↓                 ↓               ↓            ↓
Camera    Timestamps      Normalization    Kalman Filter  State
Ultrasonic Sync           Alignment        Bayesian       Output
Magnetic                  Calibration      Neural Net
```

#### 2.5.2 Fusion Algorithm

```python
def fuse_sensors(camera, ultrasonic, magnetic):
    # Weight factors (sum = 1.0)
    w_camera = 0.4
    w_ultrasonic = 0.3
    w_magnetic = 0.3

    # Confidence-weighted fusion
    confidence = (
        w_camera * camera.confidence +
        w_ultrasonic * ultrasonic.confidence +
        w_magnetic * magnetic.confidence
    )

    # Majority voting with confidence weighting
    votes = {
        'occupied': 0,
        'available': 0
    }

    if camera.status == 'occupied':
        votes['occupied'] += w_camera * camera.confidence
    else:
        votes['available'] += w_camera * camera.confidence

    # Similar for ultrasonic and magnetic...

    # Decision
    if votes['occupied'] > votes['available']:
        return {
            'status': 'OCCUPIED',
            'confidence': votes['occupied']
        }
    else:
        return {
            'status': 'AVAILABLE',
            'confidence': votes['available']
        }
```

---

## 3. Occupancy States and Transitions

### 3.1 State Definitions

```
States:
├── AVAILABLE       : Space is free and ready for use
├── OCCUPIED        : Vehicle is parked in space
├── RESERVED        : Space is booked for future use
├── CHARGING        : EV is charging (may or may not be occupied)
├── DISABLED        : Space designated for accessibility (available or occupied)
├── MAINTENANCE     : Space temporarily unavailable
└── UNKNOWN         : Sensor malfunction or uncertain state
```

### 3.2 State Transition Diagram

```
                    ┌─────────────┐
                    │  AVAILABLE  │
                    └──────┬──────┘
                           │
        ┌──────────────────┼──────────────────┐
        │                  │                  │
    reservation       vehicle_entry      maintenance
        │                  │                  │
        ↓                  ↓                  ↓
  ┌──────────┐       ┌──────────┐      ┌──────────────┐
  │ RESERVED │       │ OCCUPIED │      │ MAINTENANCE  │
  └────┬─────┘       └────┬─────┘      └──────┬───────┘
       │                  │                    │
   vehicle_entry    vehicle_exit         repair_complete
       │                  │                    │
       ↓                  ↓                    │
  ┌──────────┐      ┌──────────┐              │
  │ OCCUPIED │      │AVAILABLE │←─────────────┘
  └──────────┘      └──────────┘
       │
   ev_charging_start
       │
       ↓
  ┌──────────┐
  │ CHARGING │
  └────┬─────┘
       │
  charging_complete
       │
       ↓
  ┌──────────┐
  │ OCCUPIED │
  └──────────┘
```

### 3.3 State Transition Rules

#### 3.3.1 AVAILABLE → OCCUPIED

```
Conditions:
- Sensor detects vehicle presence
- Detection confidence ≥ 90%
- Minimum dwell time: 10 seconds
- No active reservation conflict

Actions:
- Update occupancy status
- Record entry timestamp
- Start billing timer (if applicable)
- Send notification to monitoring system
```

#### 3.3.2 OCCUPIED → AVAILABLE

```
Conditions:
- Sensor confirms vehicle absence
- Detection confidence ≥ 90%
- Minimum vacancy time: 5 seconds
- Payment processed (if required)

Actions:
- Update occupancy status
- Record exit timestamp
- Calculate dwell time and charges
- Mark space as available
- Update guidance systems
```

#### 3.3.3 AVAILABLE → RESERVED

```
Conditions:
- Valid reservation request
- Space available at requested time
- Payment authorization successful

Actions:
- Block space for reservation period
- Generate confirmation code
- Send confirmation to user
- Set expiration timer (grace period: 15 min)
```

### 3.4 State Persistence

All state changes must be:
- **Logged**: Timestamp, previous state, new state, trigger event
- **Persistent**: Stored in database with redundancy
- **Real-Time**: Propagated to clients within 2 seconds
- **Auditable**: Complete history maintained for compliance

---

## 4. Parking Guidance Systems

### 4.1 Guidance Objectives

1. **Minimize search time**: Direct drivers to available spaces quickly
2. **Optimize traffic flow**: Reduce congestion within parking facility
3. **Balance occupancy**: Distribute vehicles evenly across facility
4. **User preferences**: Consider accessibility, EV charging, proximity

### 4.2 Guidance Algorithm

#### 4.2.1 Space Selection Criteria

```python
def calculate_space_score(space, user_location, preferences):
    """
    Calculate suitability score for a parking space (0-100)
    """
    score = 0

    # Distance factor (40% weight)
    distance = haversine_distance(user_location, space.location)
    distance_score = max(0, 100 - (distance / 10))  # Decrease by 10 per 100m
    score += 0.4 * distance_score

    # Availability confidence (30% weight)
    confidence_score = space.detection_confidence * 100
    score += 0.3 * confidence_score

    # Price factor (15% weight)
    if preferences.price_range:
        if preferences.price_range[0] <= space.hourly_rate <= preferences.price_range[1]:
            price_score = 100
        else:
            price_score = 50
        score += 0.15 * price_score

    # Size match (10% weight)
    if space.size >= preferences.vehicle_size:
        size_score = 100
    else:
        size_score = 0
    score += 0.1 * size_score

    # Special requirements (5% weight)
    special_score = 0
    if preferences.requires_ev and space.has_ev_charger:
        special_score += 50
    if preferences.requires_accessibility and space.is_disabled:
        special_score += 50
    score += 0.05 * special_score

    return score
```

#### 4.2.2 Route Planning

```
1. Identify top 5 candidate spaces
2. Calculate optimal route to each
3. Consider:
   - Current traffic conditions
   - One-way restrictions
   - Turn difficulty
   - Time of day
4. Select space with best (score + route_efficiency)
5. Provide turn-by-turn navigation
6. Monitor space availability during transit
7. Reroute if space becomes occupied
```

### 4.3 Dynamic Signage Integration

#### 4.3.1 LED Display Protocol

```
Message Format:
┌────────────────────────────┐
│  Floor 2: ↑                │
│  Available: 15             │
│  Floor 3: ↑                │
│  Available: 8              │
│  Floor 4: FULL             │
└────────────────────────────┘

Update Frequency: Every 5 seconds
Communication: MQTT, RS-485, or IP-based
```

#### 4.3.2 Space-Level Indicators

```
Red LED: OCCUPIED
Green LED: AVAILABLE
Blue LED: RESERVED
Amber LED: CHARGING
Flashing Green: ALLOCATED (guidance in progress)
```

### 4.4 Mobile App Guidance

#### 4.4.1 Navigation Features

- **AR Overlay**: Augmented reality directions to parking space
- **Floor Plans**: Interactive maps with real-time availability
- **Voice Navigation**: Turn-by-turn voice instructions
- **Space Locator**: Save location and get return directions

---

## 5. Reservation and Payment Systems

### 5.1 Reservation Workflow

#### 5.1.1 Reservation Request

```json
{
  "reservation_request": {
    "lot_id": "lot-downtown-01",
    "start_time": "2025-12-26T14:00:00Z",
    "duration": 120,
    "vehicle_info": {
      "license_plate": "ABC123",
      "type": "sedan",
      "length": 4.5,
      "requires_ev": true
    },
    "user_info": {
      "user_id": "user-12345",
      "phone": "+1-555-0100",
      "email": "user@example.com"
    },
    "preferences": {
      "proximity": "entrance",
      "covered": true
    }
  }
}
```

#### 5.1.2 Space Allocation Algorithm

```python
def allocate_space(request, available_spaces):
    """
    Allocate optimal parking space for reservation
    """
    # Filter by requirements
    eligible = [
        space for space in available_spaces
        if matches_requirements(space, request)
    ]

    # Check availability during requested timeframe
    eligible = [
        space for space in eligible
        if is_available_during(space, request.start_time, request.duration)
    ]

    # Score and rank
    scored = [
        (space, score_for_reservation(space, request))
        for space in eligible
    ]

    # Sort by score (descending)
    scored.sort(key=lambda x: x[1], reverse=True)

    if scored:
        return scored[0][0]  # Return top-scoring space
    else:
        return None  # No available space
```

#### 5.1.3 Reservation Confirmation

```json
{
  "reservation": {
    "confirmation_code": "RES-20251226-4A8F",
    "space_id": "SP-EV-042",
    "lot_id": "lot-downtown-01",
    "start_time": "2025-12-26T14:00:00Z",
    "end_time": "2025-12-26T16:00:00Z",
    "grace_period": 15,
    "estimated_cost": {
      "amount": 20.00,
      "currency": "USD",
      "breakdown": {
        "parking": 18.00,
        "ev_charging": 2.00,
        "fees": 0.00
      }
    },
    "access_code": "4582",
    "instructions": "Enter code at gate. Space located on Level 2, Section E."
  }
}
```

### 5.2 Payment Integration

#### 5.2.1 Payment Methods

```
Supported Methods:
├── Credit/Debit Cards (Stripe, Square, PayPal)
├── Mobile Wallets (Apple Pay, Google Pay, Samsung Pay)
├── License Plate Payment (automated billing)
├── QR Code Payment (scan and pay)
├── RFID/NFC (tap to pay)
└── Account Balance (prepaid accounts)
```

#### 5.2.2 Pricing Models

**Hourly Pricing**
```
Rate = base_rate × time_hours × multiplier

Multipliers:
- Peak hours (8am-6pm weekdays): 1.5×
- Off-peak: 1.0×
- Overnight (10pm-6am): 0.5×
- Weekend: 1.2×
- Special events: 2.0× - 3.0×
```

**Dynamic Pricing**
```
Rate = base_rate × demand_factor × time_factor

demand_factor = occupancy_rate × surge_multiplier
  - If occupancy < 50%: demand_factor = 1.0
  - If 50% ≤ occupancy < 80%: demand_factor = 1.2
  - If occupancy ≥ 80%: demand_factor = 1.5

time_factor = hours_parked × discount_factor
  - First hour: 1.0
  - 2-4 hours: 0.95 (5% discount)
  - 4+ hours: 0.90 (10% discount)
```

**Subscription Models**
```
- Monthly Unlimited: Fixed price, unlimited parking
- Reserved Space: Dedicated space, guaranteed availability
- Flex Pass: X hours per month, use anytime
- Business Account: Multiple vehicles, corporate billing
```

#### 5.2.3 Payment Processing Flow

```
1. Entry
   ├─→ Scan ticket/plate
   ├─→ Record entry time
   └─→ Issue temporary access token

2. Parking Duration
   ├─→ Real-time cost calculation
   ├─→ Send cost updates (optional)
   └─→ Monitor overstay

3. Pre-Exit
   ├─→ Calculate total charges
   ├─→ Present payment options
   ├─→ Process payment
   └─→ Issue exit authorization (valid 15 min)

4. Exit
   ├─→ Validate exit authorization
   ├─→ Open gate/barrier
   ├─→ Record exit time
   ├─→ Send receipt
   └─→ Close transaction
```

#### 5.2.4 License Plate Recognition (LPR) Payment

```
Flow:
1. Entry camera captures plate
2. OCR extracts plate number
3. Check for registered account
4. If registered:
   - Grant automatic entry
   - Start timer
   - Link to payment method
5. Exit camera captures plate
6. Calculate charges
7. Auto-charge payment method
8. Open exit gate
9. Send receipt via email/SMS

Accuracy Requirements:
- Plate recognition: >98%
- Character recognition: >99.5%
- Validation: Cross-check with vehicle database
```

### 5.3 Violation Management

#### 5.3.1 Overstay Detection

```
Monitoring:
- Compare actual dwell time vs. paid time
- Grace period: 15 minutes
- Warning notification at 80% of time limit
- Overstay alert at expiration + grace period

Penalties:
- First 30 min overstay: 1.5× hourly rate
- 30-60 min: 2.0× hourly rate
- 60+ min: 2.5× hourly rate + potential citation
```

#### 5.3.2 Unauthorized Parking

```
Detection:
- Reserved space occupied without reservation
- Disabled space used without permit
- EV space used by non-EV vehicle
- Parking outside marked lines

Actions:
- Capture evidence (photos, video)
- Issue warning notification
- Generate citation (if no response)
- Tow vehicle (extreme cases)
```

---

## 6. Real-Time Occupancy Tracking

### 6.1 Data Collection Architecture

```
Sensors → Edge Processing → Gateway → Cloud Platform → Client Applications
   ↓           ↓                ↓          ↓              ↓
Detect      Filter          Aggregate   Store         Display
Status      Noise           Data        Analytics     Real-time
            Debounce        Compress    Process       Updates
```

### 6.2 Data Aggregation

#### 6.2.1 Space-Level Data

```json
{
  "space": {
    "id": "SP-042",
    "lot_id": "lot-downtown-01",
    "status": "OCCUPIED",
    "confidence": 0.98,
    "last_updated": "2025-12-26T14:35:22Z",
    "vehicle": {
      "entry_time": "2025-12-26T13:15:10Z",
      "duration_minutes": 80,
      "license_plate": "ABC123",
      "type": "sedan"
    },
    "sensor": {
      "type": "camera",
      "id": "CAM-L2-03",
      "health": "normal"
    }
  }
}
```

#### 6.2.2 Lot-Level Summary

```json
{
  "lot_summary": {
    "lot_id": "lot-downtown-01",
    "timestamp": "2025-12-26T14:35:00Z",
    "total_spaces": 500,
    "available": 87,
    "occupied": 398,
    "reserved": 12,
    "maintenance": 3,
    "occupancy_rate": 0.796,
    "avg_dwell_time": 135,
    "turnover_rate": 3.2,
    "revenue_current_day": 12450.00,
    "by_floor": [
      {
        "floor": 1,
        "total": 150,
        "available": 12,
        "occupancy_rate": 0.92
      },
      {
        "floor": 2,
        "total": 175,
        "available": 35,
        "occupancy_rate": 0.80
      },
      {
        "floor": 3,
        "total": 175,
        "available": 40,
        "occupancy_rate": 0.77
      }
    ],
    "ev_charging": {
      "total_chargers": 25,
      "in_use": 18,
      "available": 7
    }
  }
}
```

### 6.3 Real-Time Updates

#### 6.3.1 WebSocket Protocol

```javascript
// Client connection
const ws = new WebSocket('wss://api.parking.example.com/v1/realtime');

ws.on('open', () => {
  // Subscribe to lot updates
  ws.send(JSON.stringify({
    action: 'subscribe',
    lot_id: 'lot-downtown-01',
    events: ['occupancy_change', 'status_update']
  }));
});

ws.on('message', (data) => {
  const event = JSON.parse(data);

  if (event.type === 'occupancy_change') {
    updateDisplay(event.lot_id, event.available, event.total);
  }

  if (event.type === 'status_update') {
    updateSpaceStatus(event.space_id, event.status);
  }
});
```

#### 6.3.2 Server-Sent Events (SSE)

```
Client: GET /v1/lots/lot-downtown-01/stream
Server:
  Content-Type: text/event-stream
  Cache-Control: no-cache
  Connection: keep-alive

event: occupancy
data: {"available": 87, "total": 500, "timestamp": "2025-12-26T14:35:00Z"}

event: space_change
data: {"space_id": "SP-042", "status": "OCCUPIED", "timestamp": "2025-12-26T14:35:22Z"}
```

### 6.4 Analytics and Reporting

#### 6.4.1 Key Metrics

```
Operational Metrics:
- Occupancy rate (current, hourly, daily, weekly)
- Average dwell time
- Turnover rate (vehicles per space per day)
- Peak occupancy times
- Space utilization efficiency

Financial Metrics:
- Revenue (hourly, daily, monthly, yearly)
- Average transaction value
- Payment method distribution
- Overstay penalties collected

User Metrics:
- Search time (before using guidance)
- User satisfaction score
- Repeat usage rate
- Mobile app adoption
```

#### 6.4.2 Predictive Analytics

```python
def predict_occupancy(lot_id, timestamp):
    """
    Predict parking occupancy using machine learning
    """
    # Features
    features = {
        'hour_of_day': timestamp.hour,
        'day_of_week': timestamp.weekday(),
        'is_holiday': is_holiday(timestamp),
        'weather': get_weather(timestamp),
        'nearby_events': get_events(lot_id, timestamp),
        'historical_avg': get_historical_avg(lot_id, timestamp)
    }

    # Model prediction (trained Random Forest or LSTM)
    predicted_occupancy = model.predict(features)

    return {
        'predicted_available': predicted_occupancy.available,
        'confidence_interval': predicted_occupancy.ci,
        'recommendation': generate_recommendation(predicted_occupancy)
    }
```

---

## 7. EV Charging Integration

### 7.1 Charging Station Types

```
Level 1 (AC):
- Power: 1.4 kW
- Voltage: 120V
- Charging Time: 40-50 hours (full charge)
- Use Case: Overnight parking

Level 2 (AC):
- Power: 7-22 kW
- Voltage: 240V
- Charging Time: 4-10 hours (full charge)
- Use Case: Workplace, shopping centers

DC Fast Charging:
- Power: 50-350 kW
- Voltage: 400-800V DC
- Charging Time: 20-60 minutes (80% charge)
- Use Case: Highway rest stops, quick stops
```

### 7.2 Connector Standards

```
Connector Types:
├── Type 1 (SAE J1772): North America, Japan (AC)
├── Type 2 (Mennekes): Europe, global (AC)
├── CCS Combo 1: North America (DC)
├── CCS Combo 2: Europe (DC)
├── CHAdeMO: Japan, some global (DC)
├── Tesla Supercharger: Tesla vehicles (DC)
└── GB/T: China (AC and DC)
```

### 7.3 Charging Session Management

#### 7.3.1 Session Initiation

```json
{
  "charging_session": {
    "session_id": "CHG-20251226-7B3D",
    "space_id": "SP-EV-042",
    "charger_id": "EVSE-L2-18",
    "vehicle": {
      "license_plate": "EV-XYZ-789",
      "battery_capacity": 75,
      "current_soc": 35,
      "target_soc": 80,
      "connector_type": "CCS2"
    },
    "start_time": "2025-12-26T14:00:00Z",
    "estimated_completion": "2025-12-26T16:30:00Z",
    "pricing": {
      "energy_rate": 0.35,
      "currency": "USD",
      "unit": "kWh",
      "parking_included": true
    }
  }
}
```

#### 7.3.2 Real-Time Monitoring

```json
{
  "charging_status": {
    "session_id": "CHG-20251226-7B3D",
    "status": "charging",
    "current_power": 11.5,
    "energy_delivered": 22.3,
    "current_soc": 52,
    "estimated_completion": "2025-12-26T16:15:00Z",
    "cost_so_far": 7.81,
    "elapsed_time": 65
  }
}
```

#### 7.3.3 Smart Charging Features

```python
def optimize_charging_schedule(vehicles, grid_capacity, electricity_prices):
    """
    Optimize EV charging schedule to minimize cost and balance load
    """
    # Sort vehicles by departure time
    vehicles.sort(key=lambda v: v.departure_time)

    schedule = []

    for vehicle in vehicles:
        # Calculate required energy
        energy_needed = (vehicle.target_soc - vehicle.current_soc) * vehicle.battery_capacity / 100

        # Find optimal charging slots
        optimal_slots = find_lowest_price_slots(
            start=vehicle.arrival_time,
            end=vehicle.departure_time,
            energy=energy_needed,
            prices=electricity_prices
        )

        # Check grid capacity constraints
        feasible_slots = filter_by_capacity(
            slots=optimal_slots,
            existing_schedule=schedule,
            grid_capacity=grid_capacity
        )

        # Assign charging schedule
        schedule.append({
            'vehicle': vehicle.id,
            'slots': feasible_slots,
            'cost': calculate_cost(feasible_slots, electricity_prices)
        })

    return schedule
```

### 7.4 Vehicle-to-Grid (V2G) Integration

```
Bidirectional Charging:
- Vehicle charges from grid during low-demand periods
- Vehicle supplies power to grid during peak demand
- Balance load and reduce grid stress
- Provide revenue opportunity for EV owners

Requirements:
- Bidirectional charger (ISO 15118)
- V2G-capable vehicle
- Grid interconnection agreement
- Real-time pricing signals
```

---

## 8. Data Formats and Protocols

### 8.1 Standard Data Formats

#### 8.1.1 Parking Space Object

```json
{
  "space": {
    "id": "SP-042",
    "lot_id": "lot-downtown-01",
    "location": {
      "lat": 37.7749,
      "lng": -122.4194,
      "floor": 2,
      "section": "E",
      "row": 4,
      "number": 2
    },
    "geometry": {
      "type": "Polygon",
      "coordinates": [
        [-122.41940, 37.77490],
        [-122.41935, 37.77490],
        [-122.41935, 37.77485],
        [-122.41940, 37.77485],
        [-122.41940, 37.77490]
      ]
    },
    "dimensions": {
      "length": 5.5,
      "width": 2.5,
      "height": 2.2,
      "unit": "meters"
    },
    "type": "standard",
    "features": {
      "covered": true,
      "ev_charger": {
        "available": true,
        "type": "Level2",
        "connector": "CCS2",
        "power": 11,
        "unit": "kW"
      },
      "accessibility": {
        "disabled": false,
        "family": false,
        "motorcycle": false
      }
    },
    "status": {
      "current": "AVAILABLE",
      "confidence": 0.98,
      "last_updated": "2025-12-26T14:35:00Z",
      "sensor": {
        "type": "camera",
        "id": "CAM-L2-03"
      }
    },
    "pricing": {
      "hourly_rate": 5.00,
      "daily_max": 40.00,
      "currency": "USD",
      "ev_charging_rate": 0.35,
      "ev_charging_unit": "kWh"
    }
  }
}
```

#### 8.1.2 Parking Lot Object

```json
{
  "lot": {
    "id": "lot-downtown-01",
    "name": "Downtown Parking Center",
    "operator": {
      "id": "op-12345",
      "name": "ParkCo Management",
      "contact": {
        "phone": "+1-555-0199",
        "email": "info@parkco.example.com",
        "website": "https://parkco.example.com"
      }
    },
    "location": {
      "address": {
        "street": "123 Main Street",
        "city": "San Francisco",
        "state": "CA",
        "zip": "94102",
        "country": "USA"
      },
      "coordinates": {
        "lat": 37.7749,
        "lng": -122.4194
      },
      "entrances": [
        {
          "id": "ENT-01",
          "type": "vehicle",
          "location": {"lat": 37.7750, "lng": -122.4195},
          "access": ["entry", "exit"]
        }
      ]
    },
    "capacity": {
      "total": 500,
      "standard": 450,
      "disabled": 25,
      "ev": 25,
      "motorcycle": 0
    },
    "features": {
      "covered": true,
      "security": {
        "cameras": true,
        "guards": true,
        "gated": true,
        "lighting": "full"
      },
      "amenities": {
        "restrooms": true,
        "elevator": true,
        "car_wash": false,
        "valet": false
      },
      "payment_methods": [
        "credit_card",
        "mobile_wallet",
        "license_plate",
        "qr_code"
      ]
    },
    "operating_hours": {
      "monday": {"open": "00:00", "close": "23:59"},
      "tuesday": {"open": "00:00", "close": "23:59"},
      "wednesday": {"open": "00:00", "close": "23:59"},
      "thursday": {"open": "00:00", "close": "23:59"},
      "friday": {"open": "00:00", "close": "23:59"},
      "saturday": {"open": "00:00", "close": "23:59"},
      "sunday": {"open": "00:00", "close": "23:59"},
      "24_7": true
    },
    "pricing": {
      "base_hourly": 5.00,
      "daily_max": 40.00,
      "currency": "USD",
      "dynamic_pricing": true
    }
  }
}
```

### 8.2 Communication Protocols

#### 8.2.1 REST API

```
Base URL: https://api.parking.example.com/v1

Authentication: Bearer token (OAuth 2.0)

Rate Limits:
- Standard: 1000 requests/hour
- Premium: 10000 requests/hour
- Real-time: Unlimited (WebSocket)

Endpoints:
├── GET    /lots                        # List parking lots
├── GET    /lots/{lot_id}               # Get lot details
├── GET    /lots/{lot_id}/spaces        # List spaces in lot
├── GET    /lots/{lot_id}/availability  # Real-time availability
├── POST   /reservations                # Create reservation
├── GET    /reservations/{id}           # Get reservation details
├── PUT    /reservations/{id}           # Update reservation
├── DELETE /reservations/{id}           # Cancel reservation
├── POST   /payments                    # Process payment
├── GET    /sessions/{id}               # Get parking session
└── GET    /analytics/occupancy         # Get occupancy analytics
```

#### 8.2.2 MQTT Topics

```
Topic Structure: parking/{region}/{lot_id}/{space_id}/{event}

Examples:
- parking/us-west/lot-001/SP-042/status
- parking/us-west/lot-001/occupancy
- parking/us-west/lot-001/SP-EV-05/charging

Message Format: JSON
QoS: 1 (at least once delivery)
Retained: Yes (for status messages)
```

#### 8.2.3 WebSocket Events

```javascript
// Event types
const events = {
  OCCUPANCY_CHANGE: 'occupancy_change',
  SPACE_STATUS: 'space_status',
  RESERVATION_UPDATE: 'reservation_update',
  CHARGING_STATUS: 'charging_status',
  PAYMENT_COMPLETE: 'payment_complete'
};

// Sample message
{
  "event": "space_status",
  "timestamp": "2025-12-26T14:35:22Z",
  "data": {
    "lot_id": "lot-downtown-01",
    "space_id": "SP-042",
    "status": "OCCUPIED",
    "confidence": 0.98,
    "previous_status": "AVAILABLE"
  }
}
```

---

## 9. API Specification

### 9.1 Find Available Spaces

#### Request

```http
GET /v1/lots/search?lat=37.7749&lng=-122.4194&radius=1000&vehicle_type=sedan&requires_ev=true
Authorization: Bearer {token}
```

#### Response

```json
{
  "status": "success",
  "results": 15,
  "spaces": [
    {
      "space_id": "SP-EV-042",
      "lot_id": "lot-downtown-01",
      "lot_name": "Downtown Parking Center",
      "location": {
        "lat": 37.7749,
        "lng": -122.4194,
        "distance": 150,
        "distance_unit": "meters"
      },
      "availability": {
        "status": "AVAILABLE",
        "confidence": 0.98,
        "last_updated": "2025-12-26T14:35:00Z"
      },
      "features": {
        "ev_charger": {
          "available": true,
          "type": "Level2",
          "power": 11,
          "connector": "CCS2"
        },
        "covered": true
      },
      "pricing": {
        "hourly_rate": 5.00,
        "ev_charging_rate": 0.35,
        "currency": "USD"
      },
      "score": 92.5
    }
  ]
}
```

### 9.2 Create Reservation

#### Request

```http
POST /v1/reservations
Authorization: Bearer {token}
Content-Type: application/json

{
  "lot_id": "lot-downtown-01",
  "space_id": "SP-EV-042",
  "start_time": "2025-12-26T15:00:00Z",
  "duration": 120,
  "vehicle": {
    "license_plate": "ABC123",
    "type": "sedan"
  },
  "user": {
    "email": "user@example.com",
    "phone": "+1-555-0100"
  },
  "payment_method": "pm_card_visa_4242"
}
```

#### Response

```json
{
  "status": "success",
  "reservation": {
    "id": "res_20251226_4A8F",
    "confirmation_code": "RES-4A8F",
    "space_id": "SP-EV-042",
    "lot_id": "lot-downtown-01",
    "start_time": "2025-12-26T15:00:00Z",
    "end_time": "2025-12-26T17:00:00Z",
    "grace_period": 15,
    "status": "confirmed",
    "cost": {
      "subtotal": 18.00,
      "tax": 1.58,
      "total": 19.58,
      "currency": "USD"
    },
    "payment": {
      "method": "card",
      "last4": "4242",
      "status": "authorized"
    },
    "access": {
      "code": "4582",
      "qr_code": "https://api.parking.example.com/qr/RES-4A8F"
    },
    "created_at": "2025-12-26T14:40:00Z"
  }
}
```

### 9.3 Get Real-Time Occupancy

#### Request

```http
GET /v1/lots/lot-downtown-01/occupancy
Authorization: Bearer {token}
```

#### Response

```json
{
  "status": "success",
  "lot_id": "lot-downtown-01",
  "timestamp": "2025-12-26T14:35:00Z",
  "occupancy": {
    "total": 500,
    "available": 87,
    "occupied": 398,
    "reserved": 12,
    "maintenance": 3,
    "rate": 0.796
  },
  "by_type": {
    "standard": {"total": 450, "available": 75},
    "disabled": {"total": 25, "available": 5},
    "ev": {"total": 25, "available": 7}
  },
  "by_floor": [
    {"floor": 1, "available": 12, "total": 150},
    {"floor": 2, "available": 35, "total": 175},
    {"floor": 3, "available": 40, "total": 175}
  ],
  "ev_charging": {
    "total_chargers": 25,
    "in_use": 18,
    "available": 7
  },
  "prediction": {
    "next_hour": 0.82,
    "trend": "increasing"
  }
}
```

---

## 10. Security and Privacy

### 10.1 Data Security

#### 10.1.1 Encryption

```
Data in Transit:
- TLS 1.3 for all API communications
- Certificate pinning for mobile apps
- Mutual TLS for server-to-server

Data at Rest:
- AES-256 encryption for database
- Encrypted backups
- Key rotation every 90 days
```

#### 10.1.2 Authentication & Authorization

```
User Authentication:
- OAuth 2.0 / OpenID Connect
- Multi-factor authentication (MFA)
- Biometric authentication (mobile apps)

API Authentication:
- API keys with rate limiting
- JWT tokens with short expiration
- Role-based access control (RBAC)

Permissions:
├── User: View availability, make reservations, payment
├── Operator: Manage lot, view analytics, set pricing
├── Admin: Full system access, user management
└── API: Read occupancy, limited write access
```

### 10.2 Privacy Protection

#### 10.2.1 Personal Data Handling

```
Data Minimization:
- Collect only necessary information
- Anonymous occupancy tracking (no PII)
- Aggregate analytics data

Data Retention:
- Transaction records: 7 years (compliance)
- Session logs: 90 days
- Video footage: 30 days (security)
- Anonymous analytics: Indefinite

User Rights (GDPR/CCPA):
- Right to access personal data
- Right to deletion (after legal retention)
- Right to data portability
- Right to opt-out of analytics
```

#### 10.2.2 License Plate Data

```
Protection Measures:
- Hash license plates for anonymity
- Limit access to authorized personnel
- Audit all access to plate data
- Automatic deletion after parking session
- Encrypted storage

Legal Compliance:
- Comply with local privacy laws
- Obtain user consent for LPR
- Provide opt-out mechanism
- Regular privacy audits
```

### 10.3 Security Best Practices

```
System Security:
├── Regular security audits
├── Penetration testing
├── Vulnerability scanning
├── Incident response plan
├── DDoS protection
└── Intrusion detection system (IDS)

IoT Security:
├── Secure boot for sensors
├── Firmware signature verification
├── Regular firmware updates
├── Network segmentation
└── Anomaly detection
```

---

## 11. References

### 11.1 Standards and Protocols

1. ISO 15118 - Vehicle-to-Grid Communication
2. OCPP (Open Charge Point Protocol) - EV charging communication
3. MQTT v5.0 - IoT messaging protocol
4. OAuth 2.0 - Authorization framework
5. GeoJSON - Geographic data format
6. ISO 8601 - Date and time format

### 11.2 Technology Specifications

1. SAE J1772 - Electric Vehicle Charging Connector (North America)
2. IEC 62196 - Plugs, socket-outlets, vehicle connectors (Europe)
3. CHAdeMO - DC fast charging standard (Japan)
4. CCS (Combined Charging System) - Universal charging standard

### 11.3 Related WIA Standards

- WIA-INTENT: Intent-based parking queries
- WIA-OMNI-API: Universal parking API gateway
- WIA-SOCIAL: Social parking sharing
- WIA-AUTO-001: Connected vehicle standards
- WIA-AUTO-002: Electric vehicle charging standards
- WIA-AUTO-003: Autonomous vehicle parking

### 11.4 Research Papers

1. "Smart Parking Systems: A Survey" (IEEE, 2023)
2. "AI-Powered Parking Occupancy Detection" (ACM, 2024)
3. "Dynamic Pricing Algorithms for Smart Parking" (Transportation Research, 2024)
4. "Electric Vehicle Grid Integration" (Energy Journal, 2024)

---

## Appendix A: Example Implementations

### A.1 Ultrasonic Sensor Integration

```python
import time
import RPi.GPIO as GPIO

class UltrasonicSensor:
    def __init__(self, trigger_pin, echo_pin, threshold_cm=250):
        self.trigger = trigger_pin
        self.echo = echo_pin
        self.threshold = threshold_cm

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.trigger, GPIO.OUT)
        GPIO.setup(self.echo, GPIO.IN)

    def measure_distance(self):
        """Measure distance in centimeters"""
        # Send 10us pulse
        GPIO.output(self.trigger, True)
        time.sleep(0.00001)
        GPIO.output(self.trigger, False)

        # Wait for echo
        while GPIO.input(self.echo) == 0:
            pulse_start = time.time()

        while GPIO.input(self.echo) == 1:
            pulse_end = time.time()

        # Calculate distance
        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17150  # Speed of sound / 2
        distance = round(distance, 2)

        return distance

    def is_occupied(self):
        """Check if parking space is occupied"""
        distances = []

        # Take 5 measurements
        for _ in range(5):
            distances.append(self.measure_distance())
            time.sleep(0.1)

        # Use median to filter noise
        median_distance = sorted(distances)[2]

        return median_distance < self.threshold

# Usage
sensor = UltrasonicSensor(trigger_pin=23, echo_pin=24, threshold_cm=200)

while True:
    if sensor.is_occupied():
        print("OCCUPIED")
    else:
        print("AVAILABLE")

    time.sleep(1)
```

### A.2 Camera-Based Detection

```python
import cv2
import numpy as np
from ultralytics import YOLO

class ParkingDetector:
    def __init__(self, model_path, parking_spaces):
        self.model = YOLO(model_path)
        self.spaces = parking_spaces  # List of polygon coordinates

    def detect_vehicles(self, frame):
        """Detect vehicles in frame"""
        results = self.model(frame, conf=0.5)
        vehicles = []

        for result in results:
            for box in result.boxes:
                if int(box.cls) in [2, 5, 7]:  # car, bus, truck
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    confidence = float(box.conf)
                    vehicles.append({
                        'bbox': [x1, y1, x2, y2],
                        'confidence': confidence
                    })

        return vehicles

    def check_occupancy(self, frame):
        """Check which parking spaces are occupied"""
        vehicles = self.detect_vehicles(frame)
        occupancy = {}

        for space_id, space_coords in self.spaces.items():
            occupied = False
            max_overlap = 0

            for vehicle in vehicles:
                overlap = self.calculate_overlap(vehicle['bbox'], space_coords)
                if overlap > 0.6:
                    occupied = True
                    max_overlap = max(max_overlap, overlap)

            occupancy[space_id] = {
                'occupied': occupied,
                'confidence': max_overlap if occupied else 1.0 - max_overlap
            }

        return occupancy

    def calculate_overlap(self, bbox, polygon):
        """Calculate overlap between bounding box and parking space polygon"""
        # Convert bbox to polygon
        bbox_poly = np.array([
            [bbox[0], bbox[1]],
            [bbox[2], bbox[1]],
            [bbox[2], bbox[3]],
            [bbox[0], bbox[3]]
        ], dtype=np.int32)

        space_poly = np.array(polygon, dtype=np.int32)

        # Calculate areas
        mask = np.zeros((1080, 1920), dtype=np.uint8)
        cv2.fillPoly(mask, [bbox_poly], 255)
        bbox_area = cv2.countNonZero(mask)

        mask = np.zeros((1080, 1920), dtype=np.uint8)
        cv2.fillPoly(mask, [space_poly], 255)
        space_area = cv2.countNonZero(mask)

        # Calculate intersection
        mask_bbox = np.zeros((1080, 1920), dtype=np.uint8)
        cv2.fillPoly(mask_bbox, [bbox_poly], 255)
        mask_space = np.zeros((1080, 1920), dtype=np.uint8)
        cv2.fillPoly(mask_space, [space_poly], 255)

        intersection = cv2.bitwise_and(mask_bbox, mask_space)
        intersection_area = cv2.countNonZero(intersection)

        # Calculate overlap ratio
        overlap = intersection_area / space_area if space_area > 0 else 0

        return overlap

# Usage
parking_spaces = {
    'SP-001': [[100, 200], [200, 200], [200, 400], [100, 400]],
    'SP-002': [[220, 200], [320, 200], [320, 400], [220, 400]],
    # ... more spaces
}

detector = ParkingDetector('yolov8n.pt', parking_spaces)
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    occupancy = detector.check_occupancy(frame)

    for space_id, status in occupancy.items():
        print(f"{space_id}: {'OCCUPIED' if status['occupied'] else 'AVAILABLE'} "
              f"(confidence: {status['confidence']:.2f})")

    cv2.imshow('Parking Detection', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
```

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA-AUTO-013 Specification v1.0*
*© 2025 SmileStory Inc. / WIA*
*MIT License*
