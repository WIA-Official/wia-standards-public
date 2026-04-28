# WIA-AUTO-012 — Phase 3: Protocol

> Traffic-management canonical Phase 3: protocols (signal-timing + incident-detection + congestion + demand + EV preemption).

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




---

## A.1 Signal-timing optimisation protocol

The optimisation protocol covers offline plan generation (HCM 2022 + Webster minimum-delay formula + simulation-based fine tuning in SUMO/VISSIM) and real-time adaptive control (SCATS, SCOOT, RHODES, ACS-Lite, MILP-based formulations). For each adaptive class the protocol specifies the control variables (cycle, splits, offsets), the objective (average delay, maximum queue, fuel consumption proxy, transit travel time), the constraint set (minimum/maximum greens, pedestrian-clearance, transit-priority), and the controller-update cadence (sub-second for connected-vehicle adaptive; cycle-by-cycle for plan-selection systems).

## A.2 Incident-detection protocol

Incident detection fuses detector signals (loop-occupancy spike + speed drop), camera-based computer-vision (queued vehicles, debris in lane), V2X messages (DENM emissions from involved vehicles), and citizen reports via the navigation-app feed. The fusion model emits incident hypotheses with a probabilistic score; hypotheses above the policy threshold trigger an operator-review event, with an automatic-confirmation path for high-confidence types (vehicle-fire DENM, detected-collision V2X). Confirmed incidents emit Phase 2 §A.4 push events.

## A.3 Congestion-mitigation protocol

Congestion mitigation chains: (a) information dissemination via variable-message signs and navigation-app push, (b) signal-plan adjustment via Phase 3 §A.1 adaptive control, (c) ramp-metering rate adjustment for freeway access, (d) dynamic lane-use-control sign updates, (e) operator dispatch for incident-clear teams. Each lever carries a trigger threshold expressed in the LOS framework at Phase 1 §A.2 plus a saturation-protection rule so the system avoids oscillation between adjacent control regimes.

## A.4 Demand-prediction protocol

Demand forecasting combines the LSTM/Transformer baseline (1-hour, 24-hour, 7-day horizons), the seasonality decomposition (weekly, monthly, holiday calendar), and the event-modifier table (sporting, concert, weather warning). Prediction intervals are calibrated quarterly against held-out test data per the conformance suite at `/.well-known/wia-traffic-management/forecast`. The protocol exposes the forecast at the per-segment per-horizon granularity with the 80% prediction interval; consumers can subscribe to anomaly events when realised demand departs from the prediction interval.

## A.5 Emergency-vehicle preemption protocol

Emergency-vehicle preemption follows NEMA TS 2 plus the local agency's preemption priority matrix. The protocol carries the EV identifier, the route, the requested green hold duration, and the priority class (life-threatening, urgent, non-urgent). Signal controllers honour the preemption with cycle-truncation rules that preserve pedestrian-clearance minimums; preemption events are logged for after-action review per the agency's transparency policy.

## A.6 Replay and integrity defence

Standard 96-bit nonce + 300-second skew window + 600-second seen-nonce cache for the signal-plan publication and incident-reporting control plane. V2X traffic uses the IEEE 1609.2 certificate-bound authentication with per-certificate counters; replay attempts are detected and dropped at the broker. Operator-action audit logs are hash-chained per agency so revisions to the action history can be detected during after-action reviews.


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/traffic-management/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-traffic-management-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/traffic-management-host:1.0.0` ships every traffic-management envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/traffic-management.sh` ships sample envelope generators with no
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
ecosystem. Traffic-management deployments that follow this layering
inherit the per-Phase conformance gates and the cross-standard
composition behaviour described in Z.2.

弘益人間 — Benefit All Humanity.
