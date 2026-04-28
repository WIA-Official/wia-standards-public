# WIA-AUTO-012: Traffic Management Specification v1.0

> **Standard ID:** WIA-AUTO-012
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Automotive & Mobility Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Traffic Flow Theory](#2-traffic-flow-theory)
3. [Signal Timing Optimization](#3-signal-timing-optimization)
4. [Congestion Detection and Management](#4-congestion-detection-and-management)
5. [Incident Detection and Response](#5-incident-detection-and-response)
6. [Traffic Prediction Models](#6-traffic-prediction-models)
7. [Data Formats](#7-data-formats)
8. [API Interface](#8-api-interface)
9. [Integration Protocols](#9-integration-protocols)
10. [References](#10-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines the comprehensive framework for intelligent traffic management systems, providing mathematical models, algorithms, and interfaces for optimizing traffic flow, controlling signals, detecting incidents, and predicting traffic patterns.

### 1.2 Scope

The standard covers:
- Traffic flow dynamics and macroscopic models
- Signal timing optimization algorithms
- Congestion detection and mitigation strategies
- Incident detection and emergency response
- Predictive analytics and machine learning models
- Integration with ITS and smart city infrastructure

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard aims to create safer, more efficient, and environmentally sustainable transportation systems that serve all people, reduce travel time, improve air quality, and save lives through intelligent traffic management.

### 1.4 Terminology

- **Flow (q)**: Number of vehicles passing a point per unit time (veh/h)
- **Density (k)**: Number of vehicles per unit length of roadway (veh/km)
- **Speed (v)**: Average speed of traffic stream (km/h)
- **Capacity (c)**: Maximum sustainable flow rate (veh/h)
- **Level of Service (LOS)**: Quality measure from A (best) to F (worst)
- **Saturation Flow (s)**: Maximum flow rate during green time (veh/h)
- **Cycle Time (C)**: Total time for one complete signal sequence (s)

---

## 2. Traffic Flow Theory

### 2.1 Fundamental Traffic Flow Equation

The foundation of traffic flow analysis:

```
q = k × v
```

Where:
- `q` = Flow rate (vehicles/hour)
- `k` = Density (vehicles/km)
- `v` = Space mean speed (km/h)

This relationship holds for all traffic conditions and is the basis for macroscopic traffic flow analysis.

### 2.2 Greenshields' Model

The linear speed-density relationship:

```
v = vf × (1 - k/kj)
```

Where:
- `v` = Current speed
- `vf` = Free-flow speed (typically 85-100 km/h)
- `k` = Current density
- `kj` = Jam density (typically 150-200 veh/km)

Substituting into fundamental equation:

```
q = k × vf × (1 - k/kj)
q = vf × k - (vf/kj) × k²
```

### 2.3 Critical Density and Capacity

Maximum flow occurs at critical density:

```
kc = kj / 2
qmax = vf × kj / 4
```

Where:
- `kc` = Critical density (density at capacity)
- `qmax` = Maximum flow (capacity)

For typical highway:
- `vf` = 100 km/h
- `kj` = 180 veh/km
- `kc` = 90 veh/km
- `qmax` = 4,500 veh/h/lane

### 2.4 Three Traffic Regimes

#### 2.4.1 Free Flow (k < kc)
- Low density, high speed
- Flow increases with density
- Stable conditions
- LOS A, B, C

#### 2.4.2 Capacity (k ≈ kc)
- Critical density
- Maximum flow
- Transition point
- LOS D

#### 2.4.3 Congested Flow (k > kc)
- High density, low speed
- Flow decreases with density
- Unstable conditions
- LOS E, F

### 2.5 Fundamental Diagram

The relationship between flow, density, and speed:

```
Flow-Density Diagram:
  q |     /\
    |    /  \
    |   /    \
    |  /      \
    | /        \___
    |________________ k
      0  kc  kj

Speed-Density Diagram:
  v |  \
    |   \
    |    \
    |     \
    |      \___
    |____________ k
      0    kj

Flow-Speed Diagram:
  q |   /\
    |  /  \
    | /    \
    |/      \___
    |____________ v
      0  vc  vf
```

### 2.6 Macroscopic Traffic Flow Equation

Conservation of vehicles (continuity equation):

```
∂k/∂t + ∂q/∂x = 0
```

Where:
- `∂k/∂t` = Change in density over time
- `∂q/∂x` = Change in flow over space
- `x` = Position along roadway
- `t` = Time

### 2.7 Shockwave Analysis

Shockwave speed (boundary between traffic conditions):

```
vw = (q₂ - q₁) / (k₂ - k₁)
```

Where:
- `vw` = Shockwave speed
- `q₁, q₂` = Flow rates before and after shockwave
- `k₁, k₂` = Densities before and after shockwave

**Applications**:
- Queue formation and dissipation
- Traffic light effects
- Incident impact analysis

---

## 3. Signal Timing Optimization

### 3.1 Webster's Method

#### 3.1.1 Optimal Cycle Time

```
Co = (1.5L + 5) / (1 - Y)
```

Where:
- `Co` = Optimal cycle time (seconds)
- `L` = Total lost time per cycle (typically 3-5s per phase)
- `Y` = Sum of critical lane flow ratios

#### 3.1.2 Critical Lane Volume Ratio

```
yi = qi / si
```

Where:
- `yi` = Flow ratio for phase i
- `qi` = Flow rate for critical lane in phase i (veh/h)
- `si` = Saturation flow rate (typically 1800-2000 veh/h/lane)

#### 3.1.3 Green Time Allocation

```
gi = (Co - L) × (yi / Y)
```

Where:
- `gi` = Effective green time for phase i
- `Co` = Optimal cycle time
- `L` = Total lost time
- `yi` = Flow ratio for phase i
- `Y` = Sum of all flow ratios

### 3.2 Delay Calculation

Average delay per vehicle (Webster's formula):

```
d = C(1 - λ)² / [2(1 - λx)] + x² / [2q(1 - x)]
```

Where:
- `d` = Average delay per vehicle (seconds)
- `C` = Cycle time (seconds)
- `λ` = Green time ratio (g/C)
- `x` = Degree of saturation (q/c)
- `q` = Actual flow rate
- `c` = Capacity (saturation flow × green ratio)

Simplified approximation:

```
d ≈ C × (1 - g/C)² / [2(1 - qg/Cs)]
```

### 3.3 Level of Service (LOS)

Based on average control delay:

| LOS | Delay (s/veh) | Description |
|-----|---------------|-------------|
| A | 0 - 10 | Free flow, no delay |
| B | 10 - 20 | Stable flow, minimal delay |
| C | 20 - 35 | Stable flow, acceptable delay |
| D | 35 - 55 | Approaching unstable, tolerable delay |
| E | 55 - 80 | Unstable flow, significant delay |
| F | > 80 | Forced flow, extreme delay |

### 3.4 Capacity Analysis

Intersection capacity for approach:

```
c = s × (g/C)
```

Where:
- `c` = Capacity (veh/h)
- `s` = Saturation flow (veh/h)
- `g` = Effective green time (s)
- `C` = Cycle time (s)

Degree of saturation:

```
X = q/c
```

**Guidelines**:
- `X < 0.85`: Acceptable operation
- `0.85 ≤ X < 1.0`: Near capacity, monitor closely
- `X ≥ 1.0`: Over-saturated, improvements needed

### 3.5 Adaptive Signal Control

Real-time adjustment based on demand:

```
Cnew = Ccurrent + α × (qobserved - qexpected)
```

Where:
- `Cnew` = Updated cycle time
- `Ccurrent` = Current cycle time
- `α` = Adjustment factor (typically 0.1-0.3)
- `qobserved` = Measured flow rate
- `qexpected` = Expected flow rate

### 3.6 Coordinated Signal Systems

Offset calculation for progression:

```
t = d/v
```

Where:
- `t` = Offset time between signals
- `d` = Distance between intersections (m)
- `v` = Desired progression speed (m/s)

Bandwidth (green band width):

```
b = g - t_travel × (C/d)
```

---

## 4. Congestion Detection and Management

### 4.1 Congestion Indicators

#### 4.1.1 Speed Threshold

```
if v < 0.3 × vf then CONGESTED
```

Typical threshold: 30-40 km/h on highways

#### 4.1.2 Density Threshold

```
if k > 0.7 × kj then CONGESTED
```

#### 4.1.3 Travel Time Index

```
TTI = T_actual / T_free-flow
```

- `TTI < 1.1`: No congestion
- `1.1 ≤ TTI < 1.5`: Moderate congestion
- `TTI ≥ 1.5`: Severe congestion

### 4.2 Congestion Metrics

#### 4.2.1 Total Delay

```
D_total = ∑(T_actual - T_free-flow) × N_vehicles
```

#### 4.2.2 Vehicle-Hours of Delay (VHD)

```
VHD = ∑[(k - kc) × L × Δt]
```

Where:
- `k` = Current density
- `kc` = Critical density
- `L` = Roadway length
- `Δt` = Time interval

### 4.3 Ramp Metering

Flow control for highway on-ramps:

```
r = min(rcapacity, rhighway - qhighway)
```

Where:
- `r` = Metering rate (veh/h)
- `rcapacity` = Ramp capacity
- `rhighway` = Highway capacity
- `qhighway` = Current highway flow

ALINEA algorithm:

```
r(k+1) = r(k) + KR × (ô - o(k))
```

Where:
- `r(k)` = Metering rate at time k
- `KR` = Regulator parameter (typically 70)
- `ô` = Target occupancy (typically 0.25-0.30)
- `o(k)` = Measured occupancy

### 4.4 Variable Speed Limits

Dynamic speed adjustment:

```
v_limit = vf × min(1, √(kc/k))
```

Where speed limit decreases as density increases beyond critical.

### 4.5 Queue Length Estimation

Vertical queue:

```
Q = (q_arrival - q_departure) × t
```

Spatial queue:

```
L_queue = Q / (k_jam × n_lanes)
```

Where:
- `Q` = Number of vehicles in queue
- `L_queue` = Queue length (m)
- `k_jam` = Jam density
- `n_lanes` = Number of lanes

---

## 5. Incident Detection and Response

### 5.1 Automatic Incident Detection

#### 5.1.1 California Algorithm

```
if (|v(t) - v(t-1)| / v(t-1) > θ_speed) AND
   (k(t) / k(t-1) > θ_density) then INCIDENT
```

Typical thresholds:
- `θ_speed` = 0.25 (25% speed drop)
- `θ_density` = 1.20 (20% density increase)

#### 5.1.2 McMaster Algorithm

Uses occupancy and speed:

```
if (o(t) > o_threshold) AND (v(t) < v_threshold) then INCIDENT
```

#### 5.1.3 Machine Learning Detection

Feature vector:

```
X = [v, k, o, Δv, Δk, Δo, σ_v, σ_k]
```

Where:
- `v, k, o` = Current speed, density, occupancy
- `Δv, Δk, Δo` = Changes from previous interval
- `σ_v, σ_k` = Standard deviations (spatial)

Classification:

```
P(incident | X) = sigmoid(w^T × X + b)
```

### 5.2 Incident Impact Assessment

#### 5.2.1 Capacity Reduction

```
c_incident = c_normal × (1 - α_blocked × n_blocked / n_total)
```

Where:
- `α_blocked` = Blocking factor (0.7-1.0)
- `n_blocked` = Number of blocked lanes
- `n_total` = Total number of lanes

#### 5.2.2 Queue Growth

```
dQ/dt = q_arrival - c_incident
```

Maximum queue length:

```
Q_max = (q_arrival - c_incident) × t_clearance
```

### 5.3 Response Time Requirements

| Incident Type | Detection Time | Response Time | Clearance Time |
|---------------|----------------|---------------|----------------|
| Minor | < 2 min | < 5 min | 10-20 min |
| Moderate | < 1 min | < 3 min | 20-45 min |
| Major | < 30 sec | < 2 min | 45-90 min |
| Fatal | < 15 sec | < 1 min | 2-4 hours |

### 5.4 Emergency Vehicle Routing

Shortest time path:

```
T_total = min ∑(d_i / v_i + t_signal,i)
```

Where:
- `d_i` = Link length
- `v_i` = Expected speed on link
- `t_signal,i` = Signal delay at intersection

Signal preemption:

```
t_preempt = d_detection / v_emergency
```

Provide all-green corridor before emergency vehicle arrives.

---

## 6. Traffic Prediction Models

### 6.1 Historical Average

Simple baseline:

```
q̂(t) = (1/N) × ∑q(t - i×P)
```

Where:
- `q̂(t)` = Predicted flow at time t
- `N` = Number of historical periods
- `P` = Period length (e.g., 7 days for weekly pattern)

### 6.2 Exponential Smoothing

```
q̂(t) = α × q(t-1) + (1-α) × q̂(t-1)
```

Where:
- `α` = Smoothing factor (typically 0.1-0.3)
- Higher α gives more weight to recent observations

### 6.3 ARIMA Model

Autoregressive Integrated Moving Average:

```
q(t) = c + ∑φ_i × q(t-i) + ∑θ_j × ε(t-j) + ε(t)
```

Where:
- `φ_i` = Autoregressive coefficients
- `θ_j` = Moving average coefficients
- `ε(t)` = Error term
- `c` = Constant

### 6.4 Neural Network Prediction

Multi-layer perceptron:

```
Input: [q(t-1), q(t-2), ..., q(t-n), hour, day, weather]
Hidden: h = tanh(W1 × input + b1)
Output: q̂(t) = W2 × h + b2
```

Loss function (MSE):

```
L = (1/N) × ∑(q(t) - q̂(t))²
```

### 6.5 Kalman Filter

State prediction:

```
x̂(t|t-1) = F × x̂(t-1|t-1)
P(t|t-1) = F × P(t-1|t-1) × F^T + Q
```

Measurement update:

```
K(t) = P(t|t-1) × H^T × [H × P(t|t-1) × H^T + R]^-1
x̂(t|t) = x̂(t|t-1) + K(t) × [z(t) - H × x̂(t|t-1)]
```

Where:
- `x̂` = State estimate (speed, density, flow)
- `F` = State transition matrix
- `P` = Error covariance
- `Q` = Process noise covariance
- `R` = Measurement noise covariance
- `K` = Kalman gain
- `z` = Measurement

### 6.6 Prediction Accuracy Metrics

Mean Absolute Percentage Error (MAPE):

```
MAPE = (100/N) × ∑|q(t) - q̂(t)| / q(t)
```

Root Mean Square Error (RMSE):

```
RMSE = √[(1/N) × ∑(q(t) - q̂(t))²]
```

**Target Accuracy**:
- MAPE < 10%: Excellent
- MAPE < 20%: Good
- MAPE < 30%: Acceptable
- MAPE ≥ 30%: Poor

---

## 7. Data Formats

### 7.1 Traffic Flow Measurement

```json
{
  "timestamp": "2025-12-26T08:30:00Z",
  "location": {
    "roadId": "I-405-NB",
    "milepost": 42.3,
    "latitude": 47.6062,
    "longitude": -122.3321
  },
  "measurements": {
    "flow": 1850,
    "speed": 72,
    "density": 25.7,
    "occupancy": 0.18,
    "lanes": 4
  },
  "quality": "good",
  "source": "loop-detector"
}
```

### 7.2 Signal Timing Plan

```json
{
  "intersectionId": "INT-001",
  "planId": "PLAN-AM-PEAK",
  "cycleTime": 90,
  "offset": 15,
  "phases": [
    {
      "phaseId": 1,
      "name": "NS-Through",
      "minGreen": 10,
      "maxGreen": 45,
      "green": 35,
      "yellow": 3,
      "allRed": 2,
      "pedestrianWalk": 7,
      "pedestrianClearance": 15
    },
    {
      "phaseId": 2,
      "name": "EW-Through",
      "minGreen": 10,
      "maxGreen": 40,
      "green": 28,
      "yellow": 3,
      "allRed": 2,
      "pedestrianWalk": 7,
      "pedestrianClearance": 12
    }
  ],
  "coordination": {
    "system": "SCOOT",
    "bandwidth": 25,
    "progressionSpeed": 50
  }
}
```

### 7.3 Incident Report

```json
{
  "incidentId": "INC-2025-001234",
  "timestamp": "2025-12-26T08:45:00Z",
  "location": {
    "roadId": "I-5-SB",
    "milepost": 168.2,
    "direction": "southbound",
    "lane": 2
  },
  "type": "collision",
  "severity": "moderate",
  "lanesBlocked": 2,
  "capacityReduction": 0.50,
  "estimatedClearance": "2025-12-26T09:30:00Z",
  "impact": {
    "queueLength": 2.5,
    "delay": 850,
    "affectedVehicles": 420
  },
  "response": {
    "detectionTime": "2025-12-26T08:45:15Z",
    "responseTime": "2025-12-26T08:47:30Z",
    "dispatchedUnits": ["UNIT-42", "UNIT-73"]
  }
}
```

### 7.4 Traffic Prediction

```json
{
  "predictionId": "PRED-20251226-001",
  "timestamp": "2025-12-26T08:00:00Z",
  "location": "I-405-NB-MP42",
  "horizon": 60,
  "predictions": [
    {
      "time": "2025-12-26T09:00:00Z",
      "flow": 1920,
      "speed": 68,
      "confidence": 0.92
    },
    {
      "time": "2025-12-26T10:00:00Z",
      "flow": 1650,
      "speed": 75,
      "confidence": 0.87
    }
  ],
  "model": "lstm-neural-network",
  "accuracy": {
    "mape": 8.3,
    "rmse": 142
  }
}
```

---

## 8. API Interface

### 8.1 Calculate Traffic Flow

```typescript
interface FlowRequest {
  density: number;      // veh/km
  speed: number;        // km/h
  laneCount?: number;   // default: 1
}

interface FlowResponse {
  flow: number;         // veh/h
  capacity: number;     // veh/h
  levelOfService: 'A' | 'B' | 'C' | 'D' | 'E' | 'F';
  regime: 'free-flow' | 'capacity' | 'congested';
}
```

### 8.2 Optimize Signal Timing

```typescript
interface SignalOptimizationRequest {
  phases: Array<{
    name: string;
    volume: number;           // veh/h
    saturationFlow: number;   // veh/h
  }>;
  lostTime: number;          // seconds per phase
  targetDelay?: number;      // seconds (optional)
}

interface SignalOptimizationResponse {
  cycleTime: number;         // seconds
  greenTimes: number[];      // seconds per phase
  delays: number[];          // seconds per phase
  levelOfService: string;
  volumeToCapacity: number;
}
```

### 8.3 Detect Congestion

```typescript
interface CongestionDetectionRequest {
  speed: number;             // km/h
  density: number;           // veh/km
  freeFlowSpeed: number;     // km/h
  jamDensity?: number;       // veh/km (optional)
}

interface CongestionDetectionResponse {
  isCongested: boolean;
  severity: 'none' | 'mild' | 'moderate' | 'severe';
  travelTimeIndex: number;
  recommendedActions: string[];
}
```

### 8.4 Predict Traffic

```typescript
interface TrafficPredictionRequest {
  locationId: string;
  horizon: number;           // minutes ahead
  historicalData?: number[]; // recent flow data
  externalFactors?: {
    weather: string;
    event: string;
    dayOfWeek: string;
  };
}

interface TrafficPredictionResponse {
  predictions: Array<{
    timestamp: Date;
    flow: number;
    speed: number;
    confidence: number;       // 0-1
  }>;
  model: string;
  accuracy: {
    mape: number;
    rmse: number;
  };
}
```

---

## 9. Integration Protocols

### 9.1 NTCIP (National Transportation Communications for ITS Protocol)

Standard protocol for traffic management devices.

**NTCIP 1202** - Actuated Traffic Signal Controller (ASC)

Object definitions:
- `maxPhases`: Maximum number of phases
- `phaseGreen`: Green time for each phase
- `phasePedWalk`: Pedestrian walk time
- `coordCycleTime`: Coordinated cycle length

### 9.2 DATEX II

European standard for traffic data exchange.

**Message Types**:
- Traffic Flow Data
- Travel Time Information
- Traffic Status
- Situation (incidents, events)

### 9.3 MQTT for Real-Time Data

Topic structure:

```
wia/traffic/{region}/{road}/{milepost}/{metric}

Examples:
wia/traffic/seattle/i5-nb/168/flow
wia/traffic/seattle/i5-nb/168/speed
wia/traffic/seattle/i5-nb/168/incident
```

### 9.4 REST API

Base URL: `https://api.wia.org/traffic/v1`

Endpoints:

```
GET  /locations/{id}/flow
GET  /locations/{id}/speed
GET  /signals/{id}/timing
POST /signals/{id}/optimize
GET  /incidents
POST /incidents
GET  /predictions/{id}
```

### 9.5 WebSocket for Live Updates

```javascript
ws://stream.wia.org/traffic

// Subscribe to location
{
  "action": "subscribe",
  "topic": "traffic/seattle/i5-nb/168"
}

// Receive updates
{
  "timestamp": "2025-12-26T08:30:00Z",
  "flow": 1850,
  "speed": 72,
  "density": 25.7
}
```

---

## 10. References

### 10.1 Scientific Papers

1. Greenshields, B.D. (1935). "A Study of Traffic Capacity"
2. Webster, F.V. (1958). "Traffic Signal Settings"
3. Lighthill, M.J., Whitham, G.B. (1955). "On Kinematic Waves"
4. Daganzo, C.F. (1994). "The Cell Transmission Model"
5. Papageorgiou, M. (1991). "ALINEA: A Local Feedback Control Law"

### 10.2 Traffic Engineering Standards

| Standard | Description |
|----------|-------------|
| HCM 2010 | Highway Capacity Manual |
| MUTCD | Manual on Uniform Traffic Control Devices |
| AASHTO | Green Book - Geometric Design |
| ITE | Traffic Engineering Handbook |

### 10.3 Traffic Flow Parameters

| Parameter | Symbol | Typical Range |
|-----------|--------|---------------|
| Free-flow speed (highway) | vf | 85-120 km/h |
| Free-flow speed (urban) | vf | 50-70 km/h |
| Jam density | kj | 120-200 veh/km |
| Saturation flow | s | 1800-2100 veh/h/lane |
| Lost time per phase | L | 3-5 seconds |
| Yellow interval | Y | 3-5 seconds |
| All-red clearance | AR | 1-3 seconds |

### 10.4 WIA Standards

- WIA-INTENT: Intent-based navigation
- WIA-OMNI-API: Universal API gateway
- WIA-SOCIAL: Connected vehicle communication
- WIA-CITY: Smart city infrastructure
- WIA-ENV: Environmental monitoring

---

## Appendix A: Example Calculations

### A.1 Highway Capacity

```
Given:
- Free-flow speed: vf = 100 km/h
- Jam density: kj = 180 veh/km
- Number of lanes: 3

Calculation:
- Critical density: kc = 180/2 = 90 veh/km
- Max flow per lane: qmax = 100 × 180 / 4 = 4,500 veh/h/lane
- Total capacity: C = 4,500 × 3 = 13,500 veh/h

Result: Highway capacity is 13,500 vehicles/hour
```

### A.2 Signal Timing Optimization

```
Given:
- NS approach: 1200 veh/h, saturation 1800 veh/h
- EW approach: 900 veh/h, saturation 1800 veh/h
- Lost time: 4 seconds per phase

Calculation:
- y_NS = 1200/1800 = 0.667
- y_EW = 900/1800 = 0.500
- Y = 0.667 + 0.500 = 1.167
- Co = (1.5×8 + 5)/(1 - 1.167) = ERROR (Y > 1)

This indicates over-saturation. Need to:
1. Add lanes
2. Prohibit turns
3. Implement one-way system
```

### A.3 Queue Length at Red Light

```
Given:
- Arrival rate: 600 veh/h = 10 veh/min
- Red time: 30 seconds = 0.5 minutes
- Saturation flow: 1800 veh/h = 30 veh/min
- Green time: 30 seconds = 0.5 minutes

Calculation:
- Vehicles arriving during red: 10 × 0.5 = 5 vehicles
- Clearance rate during green: 30 veh/min
- Time to clear queue: 5/30 = 0.167 minutes = 10 seconds

Result: Queue clears in 10 seconds of green time
```

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA-AUTO-012 Specification v1.0*
*© 2025 SmileStory Inc. / WIA*
*MIT License*
