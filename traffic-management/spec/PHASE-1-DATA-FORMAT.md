# WIA-AUTO-012 — Phase 1: Data Format

> Traffic-management canonical Phase 1: detector + flow-state + signal-plan + incident + V2X envelopes.

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




---

## A.1 Detector-record envelope

The Phase 1 envelope groups detector records by sensor family (inductive-loop, video-analytic, radar-Doppler, LiDAR, magnetometer, Bluetooth/Wi-Fi probe, connected-vehicle CAM/BSM) with the canonical fields: detector identifier, GeoJSON Point geometry in WGS 84, lane-association graph (per ISO 14296 lane-numbering), detector type, vendor/model, calibration-date, mean-time-between-failures (MTBF), and the per-detector noise model (count noise, classification confusion matrix). Detector data follows the reporting conventions of NTCIP 1209 (object definitions for transportation sensor systems) and IEEE 1455 (intersection-collision warning systems).

## A.2 Traffic-flow-state envelope

Flow-state records carry: timestamp, intersection or roadway segment identifier, per-lane occupancy in %, per-lane volume in vehicles per hour, mean speed in km/h, 85th-percentile speed in km/h, headway distribution parameters (mean, standard deviation, log-normal sigma), level-of-service (LOS) category A-F per HCM 2022, queue-length in metres, and the data-source provenance (which detector contributed which field). Greenshields, Greenberg, Underwood, and HCM 2022 fundamental-diagram parameters are captured as optional sub-fields for offline calibration.

## A.3 Signal-plan envelope

Signal-plan envelopes carry: plan identifier, intersection identifier, time-of-day window (RRULE), cycle length in seconds, phase ordering with green/yellow/red durations per phase, pedestrian phasing (Walk + flashing-Don't-Walk + clearance), the offset for coordination with adjacent intersections in a green-wave plan, and the policy flags (left-turn permissive/protected/protected-permissive; right-turn-on-red allowed; transit-signal priority enabled; emergency-vehicle preemption enabled). Plan envelopes are signed by the operating agency's key.

## A.4 Incident envelope

Incident envelopes carry: incident identifier, type (collision, breakdown, debris, weather-related closure, planned event), GeoJSON geometry of the affected segment(s), severity 1-5, start time, predicted end time with prediction interval, response-resource assignments (police, fire, EMS, towing), and the public-information envelope (variable-message-sign text, navigation-app push payload). Incidents cross-reference the ATIS (Advanced Traveller Information System) feed for downstream consumer integration.

## A.5 V2X-message envelope

V2X message envelopes follow ETSI ITS-G5 / IEEE 1609 (WAVE) plus the SAE J2735 message catalogue: BSM (Basic Safety Message), CAM (Cooperative Awareness Message), DENM (Decentralised Environmental Notification Message), MAP (intersection map data), SPaT (Signal Phase and Timing), TIM (Traveller Information Message). The envelope carries the certified V2X security envelope per IEEE 1609.2 with the originating station's enrolment certificate, the pseudonym certificate set, and the misbehaviour-report attachment slot.


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
