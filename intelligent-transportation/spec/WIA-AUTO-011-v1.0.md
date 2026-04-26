# WIA-AUTO-011: Intelligent Transportation System Specification v1.0

> **Standard ID:** WIA-AUTO-011
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Mobility Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [ITS Architecture](#2-its-architecture)
3. [Traffic Flow Theory](#3-traffic-flow-theory)
4. [Traffic Management Systems](#4-traffic-management-systems)
5. [Smart Traffic Signals](#5-smart-traffic-signals)
6. [Congestion Prediction](#6-congestion-prediction)
7. [Emergency Vehicle Preemption](#7-emergency-vehicle-preemption)
8. [Toll Collection Systems](#8-toll-collection-systems)
9. [Traveler Information Systems](#9-traveler-information-systems)
10. [Vehicle Detection](#10-vehicle-detection)
11. [Data Formats](#11-data-formats)
12. [API Interface](#12-api-interface)
13. [Integration Protocols](#13-integration-protocols)
14. [References](#14-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines the comprehensive framework for Intelligent Transportation Systems (ITS), providing standardized methods for traffic management, signal control, congestion prediction, and traveler information services.

### 1.2 Scope

The standard covers:
- Traffic flow modeling and analysis
- Adaptive signal control algorithms
- Real-time congestion prediction
- Emergency vehicle priority systems
- Electronic toll collection
- Multi-modal traveler information
- Vehicle detection and classification
- Data integration and API interfaces

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard aims to create safer, more efficient, and environmentally sustainable transportation systems that benefit all road users, reduce congestion, and improve quality of life in urban and rural areas alike.

### 1.4 Terminology

- **ITS**: Intelligent Transportation System
- **ATMS**: Advanced Traffic Management System
- **ATIS**: Advanced Traveler Information System
- **LOS**: Level of Service (A through F)
- **V2I**: Vehicle-to-Infrastructure communication
- **TMC**: Traffic Management Center
- **DMS**: Dynamic Message Sign
- **EVP**: Emergency Vehicle Preemption

---

## 2. ITS Architecture

### 2.1 System Components

The ITS architecture consists of five primary subsystems:

```
┌─────────────────────────────────────────────────────────────┐
│                   Traffic Management Center                  │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐   │
│  │ Control  │  │ Monitor  │  │ Predict  │  │ Optimize │   │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └────┬─────┘   │
└───────┼────────────┼────────────┼────────────┼─────────────┘
        │            │            │            │
   ┌────▼────────────▼────────────▼────────────▼────────┐
   │              Communication Network                  │
   └────┬────────────┬────────────┬────────────┬─────────┘
        │            │            │            │
   ┌────▼────┐  ┌───▼────┐  ┌───▼────┐  ┌───▼────┐
   │ Signals │  │Sensors │  │  VMS   │  │Vehicles│
   └─────────┘  └────────┘  └────────┘  └────────┘
```

### 2.2 Data Flow

Data flows through the system in four stages:

1. **Collection**: Sensors gather traffic data
2. **Processing**: TMC analyzes and processes data
3. **Decision**: Algorithms determine optimal actions
4. **Implementation**: Control commands executed

### 2.3 Communication Protocols

Standard communication protocols:
- **NTCIP (National Transportation Communications for ITS Protocol)**
- **DATEX II (Data Exchange Specification)**
- **SAE J2735 (Dedicated Short Range Communications)**
- **MQTT/HTTP REST APIs** for real-time data exchange

---

## 3. Traffic Flow Theory

### 3.1 Fundamental Diagram

The fundamental relationship between traffic flow, density, and speed:

```
q = k × v
```

Where:
- `q` = Traffic flow (vehicles/hour)
- `k` = Traffic density (vehicles/km)
- `v` = Average speed (km/h)

### 3.2 Speed-Density Relationship

Greenshields Model:
```
v = v_f × (1 - k/k_j)
```

Where:
- `v_f` = Free-flow speed (km/h)
- `k_j` = Jam density (vehicles/km)

### 3.3 Flow-Density Relationship

```
q = k × v_f × (1 - k/k_j)
```

Maximum flow (capacity):
```
q_max = (v_f × k_j) / 4
```

Occurring at critical density:
```
k_c = k_j / 2
```

### 3.4 Greenberg Model

For congested conditions:
```
v = v_m × ln(k_j / k)
```

Where `v_m` is the speed at maximum flow.

### 3.5 Level of Service (LOS)

LOS classification based on density and speed:

| LOS | Density (veh/km/lane) | Speed (% of free-flow) | Description |
|-----|----------------------|------------------------|-------------|
| A | < 7 | > 95% | Free flow |
| B | 7-11 | 85-95% | Reasonably free flow |
| C | 11-18 | 70-85% | Stable flow |
| D | 18-26 | 55-70% | Approaching unstable |
| E | 26-45 | 40-55% | Unstable flow |
| F | > 45 | < 40% | Forced/breakdown flow |

---

## 4. Traffic Management Systems

### 4.1 Real-Time Traffic Monitoring

Continuous monitoring requirements:
- Update frequency: Every 30-60 seconds
- Coverage: Major arterials and freeways
- Data points: Speed, volume, occupancy, incidents

### 4.2 Incident Detection

Automatic incident detection algorithm:
```
Incident = (OCC > OCC_threshold) AND (SPEED < SPEED_threshold)
```

Where:
- `OCC` = Occupancy (% time detector occupied)
- `SPEED` = Average speed
- Thresholds vary by location and time of day

### 4.3 Ramp Metering

Ramp meter rate calculation:
```
r = min(r_max, (C - q_mainline) / n_ramps)
```

Where:
- `r` = Metering rate (veh/hour)
- `r_max` = Maximum metering rate
- `C` = Mainline capacity
- `q_mainline` = Current mainline flow
- `n_ramps` = Number of metered ramps

### 4.4 Variable Speed Limits

Dynamic speed limit determination:
```
VSL = v_f × (1 - α × (k/k_c))
```

Where:
- `VSL` = Variable speed limit
- `α` = Adjustment factor (0.2-0.4)
- `k_c` = Critical density

---

## 5. Smart Traffic Signals

### 5.1 Signal Timing Fundamentals

Basic signal timing components:
- **Cycle Length (C)**: Total time for one complete signal cycle
- **Green Time (g)**: Time allocated to green indication
- **Yellow Time (y)**: Clearance interval (typically 3-6 seconds)
- **Red Time (r)**: Time for conflicting movements
- **Lost Time (L)**: Time lost during phase changes

Cycle constraint:
```
C = Σ(g_i + y_i) + L
```

### 5.2 Webster's Method

Optimal cycle length:
```
C_opt = (1.5L + 5) / (1 - Y)
```

Where:
- `L` = Total lost time per cycle
- `Y` = Σ(y_i) = Sum of critical lane flow ratios
- `y_i = (v_i/s_i)` for approach i
- `v_i` = Flow rate on approach i
- `s_i` = Saturation flow rate

### 5.3 Green Time Allocation

Proportional allocation:
```
g_i = (C - L) × (y_i / Y)
```

### 5.4 Adaptive Signal Control

Real-time optimization based on detected traffic:

```
minimize: Σ(w_i × d_i)
subject to:
  g_i ≥ g_min,i
  g_i ≤ g_max,i
  Σg_i = C - L
```

Where:
- `w_i` = Weight for approach i
- `d_i` = Delay on approach i

### 5.5 Coordinated Signal Systems

Offset calculation for green wave:
```
offset = (distance / v_progression) mod C
```

Where:
- `distance` = Distance between intersections
- `v_progression` = Desired progression speed
- `mod C` = Modulo cycle length

### 5.6 Actuated Control

Gap-out logic:
```
IF (gap > gap_threshold) AND (time > g_min) THEN
  terminate green
ELSE IF time > g_max THEN
  force terminate
```

---

## 6. Congestion Prediction

### 6.1 Prediction Models

#### 6.1.1 Statistical Model

Auto-Regressive Integrated Moving Average (ARIMA):
```
X_t = c + Σ(φ_i × X_{t-i}) + Σ(θ_j × ε_{t-j}) + ε_t
```

Where:
- `X_t` = Traffic metric at time t
- `φ_i` = AR coefficients
- `θ_j` = MA coefficients
- `ε_t` = Error term

#### 6.1.2 Machine Learning Model

Neural network for congestion prediction:
```
Input: [time_of_day, day_of_week, weather, events, historical_flow]
Hidden Layers: 3 layers (128, 64, 32 neurons)
Output: [predicted_speed, predicted_flow, congestion_level]
```

### 6.2 Congestion Index

```
CI = ((T_actual - T_freeflow) / T_freeflow) × 100%
```

Congestion levels:
- CI < 10%: No congestion
- 10% ≤ CI < 30%: Light congestion
- 30% ≤ CI < 50%: Moderate congestion
- 50% ≤ CI < 100%: Heavy congestion
- CI ≥ 100%: Severe congestion

### 6.3 Travel Time Prediction

Kalman filter-based prediction:
```
X_t = A × X_{t-1} + B × u_t + w_t
Z_t = H × X_t + v_t
```

Where:
- `X_t` = State vector (speed, flow)
- `A` = State transition matrix
- `Z_t` = Measurement vector
- `w_t, v_t` = Process and measurement noise

### 6.4 Bottleneck Identification

Bottleneck criterion:
```
Bottleneck IF:
  (q_upstream > q_downstream) AND
  (k_downstream > k_critical) AND
  (duration > threshold)
```

---

## 7. Emergency Vehicle Preemption

### 7.1 Detection Methods

1. **GPS-based**: Vehicle broadcasts position
2. **Acoustic**: Sound detector activation
3. **Optical**: Strobe light detection
4. **Radio**: Dedicated frequency signal

### 7.2 Preemption Sequence

```
1. Detection: Emergency vehicle within range (500-1500m)
2. Priority: Calculate optimal route through intersections
3. Clearing: Clear conflicting approaches
4. Green: Provide green indication for EV approach
5. Hold: Maintain green until EV passes
6. Recovery: Return to normal operation
```

### 7.3 Priority Algorithm

Route optimization:
```
minimize: T_total = Σ(t_travel,i + t_delay,i)
subject to:
  Safety constraints
  Signal timing constraints
  Conflicting traffic clearance
```

Where:
- `t_travel,i` = Travel time on segment i
- `t_delay,i` = Delay at intersection i

### 7.4 Multi-Vehicle Coordination

When multiple emergency vehicles:
```
priority = w_type × P_type + w_response × P_response + w_distance × P_distance
```

Where:
- `P_type` = Priority by vehicle type (fire > ambulance > police)
- `P_response` = Response urgency
- `P_distance` = Distance to destination

---

## 8. Toll Collection Systems

### 8.1 Electronic Toll Collection (ETC)

Technology options:
- **RFID**: Radio frequency identification tags
- **ANPR**: Automatic Number Plate Recognition
- **GNSS**: Global Navigation Satellite System-based

### 8.2 Toll Calculation

Distance-based tolling:
```
Toll = base_fee + (distance × rate_per_km × vehicle_class_factor)
```

Time-based congestion pricing:
```
Toll = base_toll × (1 + α × CI/100)
```

Where:
- `α` = Congestion pricing factor (0.5-2.0)
- `CI` = Congestion Index

### 8.3 Dynamic Pricing

Real-time price adjustment:
```
Price_t = Price_base × (demand_t / capacity)^β
```

Where:
- `β` = Elasticity factor (1.5-2.5)
- `demand_t` = Current demand
- `capacity` = Road capacity

### 8.4 Transaction Processing

Requirements:
- Transaction time: < 100ms
- Accuracy: > 99.9%
- Account update: Real-time
- Violation detection: Immediate

---

## 9. Traveler Information Systems

### 9.1 Information Types

1. **Pre-trip**: Route planning, departure time
2. **En-route**: Real-time traffic, incidents
3. **Post-trip**: Travel time, alternative routes

### 9.2 Travel Time Estimation

Current conditions:
```
TT_current = Σ(L_i / v_i)
```

Predicted travel time:
```
TT_predicted = Σ(L_i / v_predicted,i)
```

Where:
- `L_i` = Segment length
- `v_i` = Current/predicted speed

### 9.3 Route Guidance

Shortest path with real-time updates:
```
minimize: Cost = Σ(w_time × TT_i + w_distance × d_i + w_toll × toll_i)
```

### 9.4 Dynamic Message Signs (DMS)

Message priority:
1. Emergency/Safety (highest)
2. Incidents/Road closures
3. Travel times
4. General information (lowest)

Display time:
```
display_time = max(minimum_read_time, n_words × 1.5 seconds)
```

---

## 10. Vehicle Detection

### 10.1 Detection Technologies

| Technology | Accuracy | Range | Cost | Use Case |
|------------|----------|-------|------|----------|
| Inductive Loop | 95-98% | Fixed | Low | Intersection counting |
| Radar | 90-95% | 0-250m | Medium | Speed detection |
| LiDAR | 98-99% | 0-200m | High | Classification |
| Video | 85-95% | Variable | Medium | Multi-lane monitoring |
| Acoustic | 80-90% | 0-50m | Low | Presence detection |

### 10.2 Classification Algorithm

Vehicle classification based on length:
```
Class 1 (Motorcycle): < 2.5m
Class 2 (Car): 2.5-6m
Class 3 (Van): 6-8m
Class 4 (Truck): 8-12m
Class 5 (Semi): > 12m
```

### 10.3 Occupancy Calculation

```
Occupancy = (Σ detection_time / measurement_period) × 100%
```

Relationship to density:
```
k ≈ (OCC × 100) / (L_effective × v)
```

Where `L_effective` = effective vehicle length (typically 6-8m).

### 10.4 Queue Detection

Queue present if:
```
(OCC > 20%) AND (v < 15 km/h) AND (duration > 30 sec)
```

Queue length estimation:
```
L_queue = (n_vehicles × spacing) + Σ(L_vehicle,i)
```

---

## 11. Data Formats

### 11.1 Traffic Data Message

```json
{
  "messageId": "TRF-20251226-001",
  "timestamp": "2025-12-26T10:30:00Z",
  "location": {
    "lat": 37.7749,
    "lon": -122.4194,
    "roadId": "US-101",
    "direction": "NB",
    "milepost": 123.5
  },
  "traffic": {
    "speed": 65.5,
    "volume": 1850,
    "occupancy": 18.5,
    "density": 28.2,
    "levelOfService": "C"
  },
  "weather": {
    "condition": "clear",
    "temperature": 18,
    "visibility": 10000
  }
}
```

### 11.2 Signal Timing Message

```json
{
  "intersectionId": "INT-001",
  "timestamp": "2025-12-26T10:30:00Z",
  "signalPlan": {
    "cycleLength": 120,
    "offset": 30,
    "coordination": "enabled"
  },
  "phases": [
    {
      "phaseId": 2,
      "greenTime": 45,
      "yellowTime": 4,
      "redTime": 71,
      "minGreen": 15,
      "maxGreen": 60
    }
  ]
}
```

### 11.3 Incident Message

```json
{
  "incidentId": "INC-20251226-042",
  "type": "accident",
  "severity": "major",
  "timestamp": "2025-12-26T10:25:00Z",
  "location": {
    "lat": 37.7749,
    "lon": -122.4194,
    "description": "I-80 EB at Bay Bridge Toll Plaza"
  },
  "impact": {
    "lanesBlocked": 2,
    "totalLanes": 5,
    "estimatedDuration": 45,
    "delay": 25
  },
  "responseUnits": ["FIRE-12", "CHP-45", "TOW-23"]
}
```

---

## 12. API Interface

### 12.1 Traffic Data API

#### Get Current Traffic

```
GET /api/v1/traffic/current
Parameters:
  - location: {lat, lon, radius}
  - roadId: string (optional)
  - dataType: [speed, volume, occupancy, all]

Response:
{
  "status": "success",
  "data": [TrafficDataMessage],
  "timestamp": "2025-12-26T10:30:00Z"
}
```

#### Get Historical Traffic

```
GET /api/v1/traffic/historical
Parameters:
  - location: {lat, lon, radius}
  - startTime: ISO8601
  - endTime: ISO8601
  - interval: number (minutes)

Response:
{
  "status": "success",
  "data": [TimeSeries],
  "aggregation": "5min"
}
```

### 12.2 Signal Control API

#### Get Signal Status

```
GET /api/v1/signals/{intersectionId}/status
Response:
{
  "intersectionId": "INT-001",
  "currentPhase": 2,
  "timeInPhase": 15,
  "nextPhase": 6,
  "mode": "coordinated"
}
```

#### Update Signal Timing

```
POST /api/v1/signals/{intersectionId}/timing
Body: SignalTimingMessage

Response:
{
  "status": "success",
  "applied": true,
  "timestamp": "2025-12-26T10:30:00Z"
}
```

### 12.3 Prediction API

#### Predict Congestion

```
POST /api/v1/predict/congestion
Body:
{
  "location": {lat, lon},
  "timeWindow": 60,
  "includeHistorical": true
}

Response:
{
  "predictions": [
    {
      "time": "2025-12-26T11:00:00Z",
      "speed": 45.2,
      "congestionLevel": "moderate",
      "confidence": 0.87
    }
  ]
}
```

---

## 13. Integration Protocols

### 13.1 NTCIP Compliance

The system must comply with:
- NTCIP 1201: Global Object Definitions
- NTCIP 1202: Actuated Signal Controller
- NTCIP 1203: Dynamic Message Sign
- NTCIP 1204: Environmental Sensor Station

### 13.2 V2I Communication

Message types (SAE J2735):
- BSM: Basic Safety Message
- SPaT: Signal Phase and Timing
- MAP: Intersection geometry
- TIM: Traveler Information Message
- SSM: Signal Status Message

### 13.3 Data Security

Requirements:
- Encryption: TLS 1.3 minimum
- Authentication: OAuth 2.0 or mutual TLS
- Data integrity: Digital signatures
- Privacy: PII anonymization

### 13.4 System Integration

```
┌──────────────┐     ┌──────────────┐     ┌──────────────┐
│  Field       │────▶│  TMC         │────▶│  External    │
│  Devices     │     │  (ITS Core)  │     │  Systems     │
└──────────────┘     └──────────────┘     └──────────────┘
       │                    │                     │
       ▼                    ▼                     ▼
   NTCIP/IP           REST/MQTT              DATEX II
```

---

## 14. References

### 14.1 Standards and Guidelines

1. ISO 14813: Intelligent Transport Systems Reference Model
2. IEEE 1512: Incident Management
3. NTCIP Standards: Traffic Controller and Device Communication
4. SAE J2735: Dedicated Short Range Communications Message Set
5. Highway Capacity Manual (HCM 7th Edition)

### 14.2 Traffic Flow Models

1. Greenshields, B.D. (1935). "A Study of Traffic Capacity"
2. Greenberg, H. (1959). "An Analysis of Traffic Flow"
3. Underwood, R.T. (1961). "Speed, Volume and Density Relationships"
4. Northwestern University (2019). "Traffic Flow Theory and Simulation"

### 14.3 Signal Control

1. Webster, F.V. (1958). "Traffic Signal Settings"
2. Robertson, D.I. (1969). "TRANSYT Method for Area Traffic Control"
3. Mirchandani, P. & Head, L. (2001). "Real-time Traffic-Adaptive Signal Control"

### 14.4 ITS Constants

| Constant | Symbol | Value |
|----------|--------|-------|
| Typical free-flow speed | v_f | 90-120 km/h (highway) |
| Typical jam density | k_j | 140-180 veh/km/lane |
| Saturation flow rate | s | 1800-2000 veh/h/lane |
| Average vehicle length | L_veh | 5-6 meters |
| Minimum headway | h_min | 1.5-2.0 seconds |
| Reaction time | t_r | 1.0-1.5 seconds |

### 14.5 WIA Standards

- WIA-INTENT: Intent-based navigation
- WIA-OMNI-API: Universal API gateway
- WIA-IOT: IoT sensor networks
- WIA-AI: AI-powered optimization
- WIA-VEHICLE: Connected vehicle standards

---

## Appendix A: Example Calculations

### A.1 Traffic Flow Calculation

```
Given:
- Density: k = 50 veh/km
- Speed: v = 60 km/h

Calculation:
- Flow: q = k × v = 50 × 60 = 3000 veh/h
```

### A.2 Signal Timing Optimization

```
Given:
- Approach 1: v₁ = 800 veh/h, s₁ = 1800 veh/h
- Approach 2: v₂ = 600 veh/h, s₂ = 1600 veh/h
- Lost time: L = 12 seconds

Calculation:
- y₁ = 800/1800 = 0.444
- y₂ = 600/1600 = 0.375
- Y = 0.444 + 0.375 = 0.819
- C_opt = (1.5×12 + 5)/(1-0.819) = 23/0.181 = 127 seconds
- g₁ = (127-12) × (0.444/0.819) = 62 seconds
- g₂ = (127-12) × (0.375/0.819) = 53 seconds
```

### A.3 Congestion Index

```
Given:
- Free-flow travel time: 15 minutes
- Actual travel time: 27 minutes

Calculation:
- CI = (27-15)/15 × 100% = 80%
- Classification: Heavy congestion
```

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA-AUTO-011 Specification v1.0*
*© 2025 SmileStory Inc. / WIA*
*MIT License*
