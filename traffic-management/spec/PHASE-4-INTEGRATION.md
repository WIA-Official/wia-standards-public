# WIA-AUTO-012 — Phase 4: Integration

> Traffic-management canonical Phase 4: ecosystem integration (NTCIP + SAE J2735 + IEEE 1609 + V2X + future).

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



---

## A.1 Standards cross-walk

| Concern                       | Standard                                  |
|-------------------------------|-------------------------------------------|
| Sensor object definitions     | NTCIP 1209                                |
| Traffic-controller protocol   | NTCIP 1202                                |
| Variable-message-sign protocol| NTCIP 1203                                |
| V2X message catalogue         | SAE J2735                                 |
| V2X security                  | IEEE 1609.2                               |
| ITS architecture              | ARC-IT (US) / ITS-G5 (EU)                 |
| Capacity analysis             | HCM 2022                                  |
| Adaptive control              | SCATS / SCOOT / RHODES / ACS-Lite         |
| Pedestrian crossings          | MUTCD 2023 / Vienna Convention 1968       |

This cross-walk is informative; implementers obtain authoritative copies from the issuing body.

## A.2 Connected and autonomous-vehicle integration

Connected-vehicle integration covers the BSM/CAM emission cadence (10 Hz default), the SPaT broadcast schedule (10 Hz when phase is active; reduced rate during night-flash mode), the MAP message version-control envelope (incremental updates require coordinated rollout to avoid map-version conflicts), and the misbehaviour-report relay to the security-credential management system per IEEE 1609.2.1. Autonomous-vehicle integration adds the high-definition map cross-link (intersection lane geometry; traffic-control device locations) and the operational-design-domain envelope so AVs can request stricter signal-timing predictability where their ODD requires it.

## A.3 Safety integration

Vision-Zero policies integrate via the safety-policy envelope at Phase 1 §A.3: agencies declare their pedestrian-priority weights, vulnerable-road-user (VRU) detection thresholds, and the speed-management policy (target 30 km/h or 50 km/h depending on the road class and the local design-speed envelope). Crash-data integration draws on the agency's crash-records database to identify recurrent-incident corridors and triggers the `/standards/v1/proposals` workflow for upgraded signal-control plans.

## A.4 Multi-agency federation

Multi-agency corridors require a federation envelope: each agency operates its own controllers under its own credentials but coordinates the offset and cycle parameters across boundaries via the Phase 3 §5 federation handshake from WIA-SOCIAL. Coordinated green-waves on cross-boundary corridors use a shared MILP formulation in which each agency contributes its objective weight, with conflict resolution by majority-vote among the participating agencies.

## A.5 Future directions

Active research tracks: full-corridor-scale reinforcement learning for adaptive control with safety guarantees, equity-aware optimisation that accounts for distributional impacts across neighbourhoods, V2X-enabled platoon-aware signal control that optimises for multi-vehicle progression, microgrid-integrated traffic-signal power management with ride-out for grid outages, and AI-assisted incident-detection from public-camera feeds with strict privacy guards. The standard's roadmap envelope (`POST /standards/v1/proposals`) tracks active proposals through the WIA Committee voting process per Phase 4 §Z.4.

## A.6 Reference list

- NTCIP 1202 / 1203 / 1209 — National Transportation Communications for ITS Protocol
- SAE J2735 — V2X message set dictionary
- IEEE 1609.2 / 1609.2.1 — WAVE security services and certificate management
- HCM 2022 — Highway Capacity Manual (Transportation Research Board)
- ARC-IT — Architecture Reference for Cooperative and Intelligent Transportation
- ETSI TS 102 941 — Trust and privacy management for ITS
- ETSI ITS-G5 / IEEE 1609 (WAVE) — Wireless Access in Vehicular Environments
- MUTCD 2023 — Manual on Uniform Traffic Control Devices
- Vienna Convention on Road Signs and Signals 1968
- IEEE 1455 — Intersection-collision warning systems
- ISO 14296 — Intelligent transport systems — extension of map database specifications for applications of cooperative ITS
- ISO/TS 21184 — Cooperative ITS — global transport data management
- C2C-CC Basic System Profile — Car-to-Car Communication Consortium reference profile

## A.7 Equity and Vision-Zero integration deep-dive

Vision-Zero policy adoption (Sweden 1997; New York 2014; Helsinki 2019) reframes the traffic-management problem from throughput-maximisation to harm-minimisation, with the working assumption that no fatality or serious injury is acceptable as the price of mobility. The standard's equity envelope captures the agency's commitments per neighbourhood-cohort: pedestrian-priority weight, vulnerable-road-user (VRU) protection-zone radius around schools and senior-living centres, the speed-management policy by road class, and the public-engagement schedule for plan revisions. Equity outcome metrics (per-cohort fatality and serious-injury rate normalised by population and exposure) are published at agreed cadence; significant deviations between cohorts trigger the `/standards/v1/proposals` workflow for an equity review.


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
