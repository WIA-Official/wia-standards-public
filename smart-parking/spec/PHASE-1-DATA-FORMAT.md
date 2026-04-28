# WIA-AUTO-013 — Phase 1: Data Format

> Smart-parking canonical Phase 1: data shapes (sensors + occupancy + lots + spaces).

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




---

## A.1 Sensor envelope (canonical)

Each sensor reports through a canonical envelope: sensor id, lot id, space id, technology (ultrasonic / camera / magnetic / radar / lidar / sound), reading payload, confidence (0..1), timestamp, and an optional uncertainty model. Consumers downstream (occupancy aggregator, guidance, billing) read from the same envelope so a sensor swap does not ripple through the consumer set.

## A.2 Detection-confidence policy

A detection result is committed only after three consecutive readings agree above the confidence threshold (0.85 by default). Single-reading flips are recorded but not propagated. The threshold is tunable per technology: ultrasonic 0.85, camera 0.96, magnetic 0.90, radar 0.92, lidar 0.95. Below-threshold readings are flagged for sensor maintenance work orders.

## A.3 Occupancy state machine

States: `available`, `occupied`, `reserved`, `maintenance`, `closed`. Transitions are protocol-driven (Phase 3) and audited with a 7-year retention. Reserved states carry a `reserverId` and an `expiresAt`; on expiry the state falls back to `available` automatically. Closed states are operations-driven (sweeping, surface repair) and do not fire customer-facing notifications.

## A.4 Lot / space data shape

A lot carries metadata (id, operator, geo polygon, total capacity, accessible-space count, EV-charging count) and a list of spaces. A space carries id, geo point, type (standard, accessible, EV, motorcycle, oversized), current state, last-state-change, and the sensor id(s) feeding the state. The shapes are published at `https://schemas.wia.org/smart-parking/v1.0/lot.json` and `space.json`.

## A.5 Aggregate envelope

Lot-level aggregates (total / occupied / available / reservation density) are published at 30-second cadence with a 5-second freshness SLA. The envelope carries the aggregate plus the source sensor count and the staleness percentage so consumers can reason about confidence in the headline number.


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/smart-parking/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-smart-parking-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/smart-parking-host:1.0.0` ships every smart-parking envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/smart-parking.sh` ships sample envelope generators with no
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
ecosystem. Smart-parking deployments that follow this layering
inherit the per-Phase conformance gates and the cross-standard
composition behaviour described in Z.2.

弘益人間 — Benefit All Humanity.
