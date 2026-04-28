# WIA-AUTO-018 — Phase 1: Data Format

> Railway-system canonical envelopes: signalling state, train-control message, communication frame, and the runtime conventions that fix the wire format for every operational protocol below.

## 1. Introduction

### 1.1 Purpose

This specification defines the technical framework for modern railway systems, encompassing signaling, train control, communication, passenger services, and safety mechanisms to enable safe, efficient, and interoperable railway operations worldwide.

### 1.2 Scope

The standard covers:
- Signaling systems (ETCS, CBTC, PTC)
- Train control and protection
- Radio communication systems (GSM-R, LTE-R)
- Passenger information and services
- Automatic train operation
- Platform safety systems
- Railway dynamics and physics
- Safety Integrity Levels (SIL)

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard aims to create a unified railway infrastructure that connects communities, reduces environmental impact through sustainable transportation, and provides safe, accessible mobility for all people.

### 1.4 Terminology

- **ATO**: Automatic Train Operation - automated driving system
- **ATP**: Automatic Train Protection - safety system preventing unsafe movements
- **ATS**: Automatic Train Supervision - traffic management system
- **CBTC**: Communications-Based Train Control - modern signaling using continuous radio
- **ETCS**: European Train Control System - European standard for train control
- **GSM-R**: Global System for Mobile Communications - Railway - dedicated railway radio
- **PTC**: Positive Train Control - North American train protection system
- **SIL**: Safety Integrity Level - measure of safety system reliability

---


## 2. Signaling Systems

### 2.1 ETCS (European Train Control System)

ETCS is the European standard for train control and protection, part of ERTMS (European Rail Traffic Management System).

#### 2.1.1 ETCS Level 1

**Architecture:**
- Track-side Eurobalises transmit point information
- LEU (Lineside Electronic Unit) interfaces with interlocking
- DMI (Driver Machine Interface) displays information to driver
- Infill (optional) provides intermediate updates

**Operation:**
```
Movement Authority (MA) = Distance to next signal + Signal aspect
```

**Data Flow:**
```
Interlocking → LEU → Eurobalise → Train → OBU → DMI → Driver
```

**Key Features:**
- Compatible with existing signals
- Point-based transmission
- National speed limits stored in train database
- Movement authority based on signal aspects

#### 2.1.2 ETCS Level 2

**Architecture:**
- Continuous communication via GSM-R
- Radio Block Center (RBC) manages train movements
- No track-side signals (cab signaling only)
- Eurobalises only for location reference

**Operation:**
```
RBC calculates: MA = Track section + Overlap + Safety distance
```

**Communication Protocol:**
```
Train → GSM-R → RBC → Interlocking
```

**Advantages:**
- Higher capacity (reduced headway)
- Lower maintenance (no signals)
- Dynamic speed profiles
- Continuous supervision

#### 2.1.3 ETCS Level 3

**Architecture:**
- Moving block operation
- Train integrity detection via on-board systems
- Satellite positioning (GNSS) augmentation
- No track circuits required

**Capacity Improvement:**
```
C_L3 / C_L2 = (H_L2 / H_L3) ≈ 1.4 - 1.6
```

Where:
- `C` = Line capacity
- `H` = Headway
- Subscripts indicate ETCS level

**Key Innovation:**
- Real-time train position to RBC
- Dynamic safety distance
- Optimal capacity utilization

### 2.2 CBTC (Communications-Based Train Control)

CBTC is the modern signaling standard for metro and urban rail systems, enabling high-frequency operations.

#### 2.2.1 System Architecture

**Components:**
```
Train → Radio → Zone Controller → Automatic Train Supervision → Control Center
```

**Communication:**
- Continuous bidirectional radio (typically 2.4 GHz or LTE)
- Polling cycle: 100-500 ms
- Redundant radio coverage (N+1)

#### 2.2.2 Moving Block Principle

**Distance Calculation:**
```
d_safe = d_train + d_braking + d_overlay + d_uncertainty
```

Where:
- `d_train` = Distance to leading train
- `d_braking` = Braking distance of following train
- `d_overlay` = Additional safety margin
- `d_uncertainty` = Position uncertainty buffer

**Typical Values:**
- Metro (80 km/h): Headway 90-120 seconds
- Metro (60 km/h): Headway 75-90 seconds
- Frequency: 30-40 trains/hour/direction

#### 2.2.3 Grades of Automation (GoA)

**GoA 1 - Non-automated (ATP only):**
- Driver controls speed and doors
- ATP provides protection only

**GoA 2 - Semi-automated (ATO with driver):**
- ATO controls acceleration and braking
- Driver monitors and closes doors
- Driver intervention possible

**GoA 3 - Driverless (attendant on board):**
- ATO controls train completely
- Attendant handles emergencies
- Automatic door operation

**GoA 4 - Unattended (fully automated):**
- No staff on train
- Remote supervision from control center
- Automatic fault detection and response

### 2.3 PTC (Positive Train Control)

PTC is the North American standard mandated for passenger and freight railways.

#### 2.3.1 Core Functions

**Prevent:**
1. Train-to-train collisions
2. Overspeed derailments
3. Incursions into work zones
4. Movement through misaligned switches

#### 2.3.2 System Components

**On-Board:**
- GPS receiver for positioning
- Locomotive data terminal
- Brake interface unit
- Back office server connection

**Wayside:**
- Signal state monitors
- Switch position monitors
- Radio towers (220 MHz)

**Office:**
- Back office server (BOS)
- CAD (Computer-Aided Dispatch) integration
- Movement authority generation

#### 2.3.3 Braking Curves

**Service Brake Curve:**
```
v_limit(d) = √(2 × a_service × d)
```

**Penalty Brake Curve:**
```
v_penalty(d) = √(2 × a_penalty × d) + v_margin
```

Where:
- `v_limit` = Speed limit at distance d
- `a_service` = Service deceleration (0.5-0.7 m/s²)
- `a_penalty` = Penalty deceleration (0.8-1.0 m/s²)
- `v_margin` = Speed margin (typically 5-10 km/h)

---


## 3. Train Control Systems

### 3.1 Automatic Train Protection (ATP)

ATP is the safety-critical component that prevents unsafe train movements.

#### 3.1.1 Speed Supervision

**Continuous Supervision:**
```
v_actual ≤ v_permitted(location)
```

**Braking Curve Calculation:**
```
v_permitted(d) = √(v_target² + 2 × a_brake × d)
```

Where:
- `v_permitted` = Maximum permitted speed at distance d
- `v_target` = Target speed (next signal/speed limit)
- `a_brake` = Guaranteed deceleration rate
- `d` = Distance to target

#### 3.1.2 Movement Authority

**Structure:**
```
MA = {
  end_location: distance or position,
  target_speed: speed at end_location,
  speed_profiles: [(location, speed_limit)],
  gradient_profile: [(location, gradient)],
  validity: time or condition
}
```

**Enforcement:**
- Train cannot exceed MA endpoint
- ATP applies emergency brake if MA violated
- MA updated dynamically by RBC/Zone Controller

#### 3.1.3 Emergency Brake Intervention

**Trigger Conditions:**
- Speed exceeds permitted + tolerance
- MA endpoint reached without stopping
- Communication loss (timeout)
- Equipment failure detection

**Response Time:**
```
t_response ≤ 1.0 seconds (SIL 4 requirement)
```

**Brake Application:**
```
Brake command → Brake valve → Brake cylinder → Brake force
Time: < 2.5 seconds to full pressure
```

### 3.2 Automatic Train Operation (ATO)

ATO automates driving functions while ATP ensures safety.

#### 3.2.1 Speed Profile Optimization

**Objective Function:**
```
minimize: E_total = ∫(P_traction + P_comfort) dt
```

**Constraints:**
- Arrival time: t_arrival = t_scheduled ± ε
- Speed limits: v(t) ≤ v_limit(position(t))
- Jerk limits: |dv/dt²| ≤ j_max
- Braking distance: d_brake(v) ≤ d_available

**Typical Profile:**
```
1. Maximum acceleration: a = a_max
2. Constant speed: v = v_limit
3. Coasting: a = 0 (when possible)
4. Service braking: a = -a_service
```

#### 3.2.2 Passenger Comfort

**Jerk Limits (SIL 0, comfort only):**
- Longitudinal jerk: |j_long| ≤ 0.75 m/s³
- Emergency situations: |j_long| ≤ 2.0 m/s³

**Acceleration Limits:**
- Starting: a ≤ 1.0 m/s²
- Maximum service: a ≤ 1.2 m/s²
- Braking: a ≥ -0.7 m/s² (service)
- Emergency: a ≥ -1.5 m/s²

#### 3.2.3 Station Stopping

**Precision Stopping:**
```
Error tolerance: ±0.3 meters (metro), ±0.5 meters (mainline)
```

**Stopping Algorithm:**
```
1. Approach phase: Reduce to v_approach (≈ 30 km/h)
2. Precision phase: Calculate exact braking point
3. Final approach: Fine control (≈ 5 km/h)
4. Creep to marker: ±0.3 m accuracy
```

### 3.3 Automatic Train Supervision (ATS)

ATS manages traffic across the network.

#### 3.3.1 Route Setting

**Automatic Route Setting:**
```
if (train_approaching AND route_conditions_met) {
  set_route(origin, destination);
  reserve_track_sections();
  update_signals();
  send_movement_authority();
}
```

**Conflict Resolution:**
- Priority based on schedule adherence
- First-come-first-served for equal priority
- Manual override possible from control center

#### 3.3.2 Headway Management

**Target Headway:**
```
H_target = max(H_minimum, H_scheduled)
```

**Minimum Safe Headway:**
```
H_minimum = (d_braking + L_train + d_safety) / v_average
```

**Regulation:**
- Green wave progression for metro
- Automatic speed adjustment
- Skip-stop patterns for express service

---


## 9. Data Formats

### 9.1 Train Status Message

```json
{
  "message_type": "train_status",
  "version": "1.0",
  "timestamp": "2025-12-26T14:35:22.123Z",
  "train": {
    "train_id": "IC_1234",
    "line": "IC 5",
    "operator": "National Rail",
    "type": "electric_multiple_unit",
    "formation": "8_cars"
  },
  "position": {
    "latitude": 51.5074,
    "longitude": -0.1278,
    "altitude": 15.5,
    "accuracy": 2.0,
    "speed": 185.3,
    "heading": 45.2,
    "track_id": "ML1",
    "milepost": 123.45
  },
  "status": {
    "operational_mode": "ATO_supervised",
    "doors": "closed_locked",
    "brakes": "released",
    "pantograph": "raised",
    "traction": "enabled"
  },
  "movement": {
    "direction": "forward",
    "acceleration": 0.3,
    "jerk": 0.05,
    "next_station": "AIRPORT_TERM",
    "eta_seconds": 420
  },
  "safety": {
    "atp_active": true,
    "movement_authority_end": 125.2,
    "permitted_speed": 200,
    "target_speed": 160,
    "vigilance_status": "acknowledged"
  }
}
```

### 9.2 Signal Aspect Message

```json
{
  "message_type": "signal_aspect",
  "version": "1.0",
  "timestamp": "2025-12-26T14:35:22.123Z",
  "signal": {
    "signal_id": "SIG_123A",
    "location": {
      "latitude": 51.5075,
      "longitude": -0.1280,
      "milepost": 124.0
    },
    "aspect": "GREEN",
    "aspect_code": "G",
    "speed_limit": 200,
    "distance_to_next": 2500,
    "next_signal_aspect": "YELLOW"
  },
  "route": {
    "route_id": "R_45",
    "route_type": "main_line",
    "points_position": "normal",
    "overlap_distance": 200
  },
  "movement_authority": {
    "end_location": 126.5,
    "end_speed": 0,
    "profile": [
      {"location": 124.0, "speed": 200},
      {"location": 125.5, "speed": 160},
      {"location": 126.0, "speed": 80},
      {"location": 126.5, "speed": 0}
    ]
  }
}
```

### 9.3 Platform Information

```json
{
  "message_type": "platform_info",
  "version": "1.0",
  "timestamp": "2025-12-26T14:35:22.123Z",
  "station": {
    "station_id": "CENTRAL_STN",
    "station_name": "Central Station",
    "station_code": "CST"
  },
  "platform": {
    "platform_id": "3A",
    "platform_number": "3",
    "platform_section": "A",
    "length": 400,
    "height": 1100,
    "track_id": "T3"
  },
  "train": {
    "train_id": "IC_1234",
    "scheduled_arrival": "2025-12-26T14:30:00Z",
    "estimated_arrival": "2025-12-26T14:33:00Z",
    "actual_arrival": "2025-12-26T14:33:15Z",
    "scheduled_departure": "2025-12-26T14:32:00Z",
    "estimated_departure": "2025-12-26T14:35:00Z",
    "platform_alignment": "aligned",
    "stopping_position": "section_A",
    "stopping_accuracy_cm": 15
  },
  "platform_screen_doors": {
    "installed": true,
    "status": "closed_locked",
    "last_operation": "2025-12-26T14:28:45Z",
    "operational": true
  },
  "passengers": {
    "alighting": 127,
    "boarding": 89,
    "waiting_platform": 156,
    "load_factor_before": 68,
    "load_factor_after": 65
  }
}
```

---



## A.1 Canonical envelope conventions

Every Phase 1 railway envelope follows the WIA family baseline:
UTF-8 JSON, RFC 8785 canonical form, Ed25519 signatures, ULID
identifiers. Track positions use ISO 19156 spatial reference
frames; absolute time uses TAI per BIPM conventions.

## A.2 Train identity envelope

```json
{
  "wia_railway_version": "1.0.0",
  "type": "train_identity",
  "train_id": "tr_01HX...",
  "operator_id": "did:wia:railway:operator-A",
  "rolling_stock_class": "EMU-9000",
  "consist": ["car_001", "car_002", "car_003"],
  "max_speed_kph": 320,
  "etcs_level": 2,
  "issued_at": "RFC 3339",
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

## A.3 Signalling-state envelope

The signalling state carries the current authority limit (movement
authority end), the speed restrictions in effect, and the
trackside-equipment identity that issued the authority. The
envelope is signed by the trackside equipment so the on-board
system can verify provenance before accepting.

## A.4 Compatibility with ETCS / CBTC / PTC

The Phase 1 envelopes round-trip cleanly to ETCS (European Train
Control System), CBTC (Communication-Based Train Control), and PTC
(Positive Train Control) native formats. The bridge profile
documents the mapping per system.


## Z.1 Glossary

The companion glossary at `https://wiastandards.com/railway-system/glossary/`
expands every term used throughout this Phase. Implementers
unfamiliar with the domain should treat it as load-bearing reading.

## Z.2 Cross-standard composition

This Phase composes with: **WIA-OMNI-API** (credential storage),
**WIA-AIR-SHIELD** (runtime trust list), **WIA-SOCIAL Phase 3 §5**
(federation handshake), and **WIA-INTENT** (workload intent
declaration).

## Z.3 Conformance test suite + reference container

A black-box conformance test suite at
`https://github.com/WIA-Official/wia-railway-system-conformance` walks
every public endpoint and protocol exchange. The reference
container at `wia/railway-system-host:1.0.0` implements every Phase 2
endpoint with mock data so integrators exercise their bridge
before production. The companion CLI at `cli/railway-system.sh` ships
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


## Z.1 Worked deployment trace

A typical first-90-day deployment of this standard at a host
operator follows the trace below:

```
Day 0    : Reference container stood up in dev environment.
Day 1-2  : Conformance suite passes against reference container.
Day 3-7  : Backend bridge implemented for the host's primary tool.
Day 8-10 : Conformance suite passes against bridged backend.
Day 11-15: Audit log replication wired up; first envelope chain audited.
Day 16-20: First federation peer onboarded; trust list cadence verified.
Day 21-30: Federation expanded to 3-5 peers; cross-peer audit verified.
Day 31-60: Production traffic shadow-routed through new stack.
Day 61-90: Cutover from legacy to new stack; legacy retained as
           fallback for the deprecation window.
```

The 90-day timeline accommodates conformance-suite passes,
operations-team training, and the regulator-notification cadence
typical for high-stakes deployments. Lighter deployments (small
operators, prototypes) compress this to 30 days.

## Z.2 Operations runbook excerpt

Day-to-day operations focus on three signals: (a) audit-log
replication lag — alarm if either replica falls more than 60s
behind the primary; (b) trust-list freshness — alarm 7 days
before any peer's signed list expires; (c) replay-cache footprint
— alarm if cache memory exceeds 80% of the documented budget.

The runbook also covers incident response: rotating signing keys
on suspected compromise, replaying the seen-nonce cache from
persistent storage on standby failover, and re-issuing federation
handshakes when the primary controller has been offline for
longer than the seen-nonce window.
