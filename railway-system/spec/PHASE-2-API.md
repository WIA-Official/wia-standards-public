# WIA-AUTO-018 — Phase 2: API

> Operational API surface: communication systems, passenger information, automatic train operation, and platform screen door interlock — each presented as a worked endpoint specification.

## 4. Communication Systems

### 4.1 GSM-R (GSM for Railways)

GSM-R is the digital radio system for railway communication.

#### 4.1.1 Technical Specifications

**Frequency Bands:**
- Uplink: 876-880 MHz
- Downlink: 921-925 MHz
- Channel bandwidth: 200 kHz

**Coverage:**
- Track-side: 99.5% availability
- Speed: Up to 500 km/h
- Handover: < 300 ms

**Services:**
- Voice calls (drivers, dispatchers)
- ETCS data transmission
- Shunting communication
- Emergency calls

#### 4.1.2 Functional Addressing

**Numbers:**
- Train number: e.g., 0049821234567
- Functional number: e.g., 004920XXXXXX
- Emergency: Functional group call

**Priority Levels:**
```
0: Emergency call (highest)
1: High priority call
2: Normal call
3: Low priority call
```

### 4.2 LTE-R (LTE for Railways)

LTE-R is the next-generation railway communication system.

#### 4.2.1 FRMCS (Future Railway Mobile Communication System)

**Technology:**
- LTE (4G) and 5G networks
- Mission-critical push-to-talk (MCPTT)
- High-speed data (up to 300 Mbps)
- Low latency (< 50 ms)

**Applications:**
- ETCS Level 2/3 data
- CCTV streaming
- Passenger Wi-Fi
- Predictive maintenance data

#### 4.2.2 Quality of Service (QoS)

**Traffic Classes:**
```
Class 1: Safety-critical (ETCS) - Priority 1, latency < 50 ms
Class 2: Operational voice - Priority 2, latency < 100 ms
Class 3: Operational data - Priority 3, latency < 200 ms
Class 4: Passenger services - Priority 4, best effort
```

### 4.3 TETRA (Terrestrial Trunked Radio)

Used in some metro and urban rail systems.

**Features:**
- Group calls
- Direct mode operation (train-to-train)
- Fast call setup (< 300 ms)
- End-to-end encryption

---


## 5. Passenger Information Systems

### 5.1 Real-Time Information

#### 5.1.1 Data Sources

**Train Position:**
```
position = {
  train_id: string,
  latitude: number,
  longitude: number,
  accuracy: number (meters),
  speed: number (km/h),
  heading: number (degrees),
  timestamp: ISO 8601
}
```

**Schedule Data:**
```
schedule = {
  station_id: string,
  arrival_time: ISO 8601,
  departure_time: ISO 8601,
  platform: string,
  delay: number (seconds),
  status: 'on-time' | 'delayed' | 'cancelled'
}
```

#### 5.1.2 Display Systems

**Platform Displays:**
- Next train: ETA, destination, platform
- Following trains: next 2-3 departures
- Service alerts and disruptions
- Update frequency: 10-30 seconds

**In-Train Displays:**
- Next station, arrival time
- Connection information
- Route map with current position
- Station facilities

#### 5.1.3 Mobile Integration

**API Response:**
```json
{
  "station_id": "CENTRAL_STN",
  "station_name": "Central Station",
  "departures": [
    {
      "train_id": "IC_1234",
      "line": "IC 5",
      "destination": "Airport Terminal",
      "scheduled_departure": "2025-12-26T14:30:00Z",
      "estimated_departure": "2025-12-26T14:33:00Z",
      "delay_seconds": 180,
      "platform": "3A",
      "status": "delayed",
      "real_time": true
    }
  ]
}
```

### 5.2 Passenger Counting

#### 5.2.1 Technologies

- **Infrared sensors**: Door-mounted sensors
- **Weight sensors**: Platform and train floor sensors
- **Video analytics**: Camera-based counting
- **Wi-Fi/Bluetooth**: Device detection (privacy-compliant)

#### 5.2.2 Occupancy Calculation

**Load Factor:**
```
LF = (passengers_on_train / capacity_total) × 100%
```

**Categories:**
- Low: 0-40%
- Medium: 40-70%
- High: 70-90%
- Overcrowded: > 90%

**Real-Time Distribution:**
```json
{
  "train_id": "M_5678",
  "total_passengers": 856,
  "capacity": 1200,
  "load_factor": 71.3,
  "cars": [
    {"car_number": 1, "passengers": 142, "capacity": 200, "load": 71},
    {"car_number": 2, "passengers": 138, "capacity": 200, "load": 69},
    ...
  ]
}
```

---


## 6. Automatic Train Operation

### 6.1 ATO Levels

**IEC 62290 Standard:**

- **GoA 0**: Manual operation (no automation)
- **GoA 1**: ATP only (driver controls)
- **GoA 2**: ATO with driver supervision
- **GoA 3**: Driverless (train attendant present)
- **GoA 4**: Unattended (fully autonomous)

### 6.2 Autonomous Operation (GoA 4)

#### 6.2.1 System Requirements

**Safety:**
- SIL 4 for all safety functions
- Redundant sensors (N+2)
- Diverse braking systems
- Fail-safe defaults

**Perception:**
- Obstacle detection: ≥ 200 meters
- Platform edge detection
- Door obstruction detection
- Emergency stop button monitoring

**Decision Making:**
```
State Machine:
  IDLE → PREPARING → READY → DEPARTING → RUNNING → ARRIVING → DWELLING → repeat
```

**Error Handling:**
```
if (critical_failure) {
  apply_emergency_brake();
  alert_control_center();
  engage_backup_systems();
  evacuate_if_necessary();
}
```

#### 6.2.2 Door Control

**Interlocking Sequence:**
```
1. Train stopped (v = 0 ± 0.1 m/s)
2. Platform detected and aligned
3. Brakes applied and confirmed
4. Platform screen doors unlock
5. Train doors unlock
6. Simultaneous opening
7. Dwell time countdown
8. Obstacle detection clear
9. Simultaneous closing
10. Doors locked confirmation
11. Ready for departure
```

**Safety Checks:**
- Door obstruction: Close → Reopen → Close (max 3 attempts)
- Emergency door release available
- Manual override from control center

---


## 7. Platform Screen Doors

### 7.1 System Architecture

#### 7.1.1 Components

**Mechanical:**
- Sliding doors (typically 1.3-1.8 m width)
- Motor drive system (AC or DC)
- Emergency release mechanism
- Edge protection sensors

**Electrical:**
- Door control unit (DCU)
- Platform-train interface
- Emergency stop buttons
- Status indicators (LED)

**Communication:**
- Train-to-PSD data link
- CBTC/ATO integration
- Control center monitoring

#### 7.1.2 Door Alignment

**Tolerance:**
```
Lateral alignment: ±150 mm
Vertical gap: 50-75 mm
Stopping accuracy: ±300 mm (metro), ±500 mm (mainline)
```

**Detection:**
- Inductive loops in platform
- RFID tags on train
- Optical sensors

### 7.2 Safety Interlocking

#### 7.2.1 Interlock Logic

**Conditions for Opening:**
```
PSD_open = train_stopped AND position_correct AND brakes_applied AND
           train_doors_ready AND no_emergency_stop AND platform_clear
```

**Sequence Diagram:**
```
Train → "Ready to open" → PSD Controller
PSD Controller → "Unlock PSD" → PSD Doors
PSD Controller → "Unlock train doors" → Train
Train + PSD → Open simultaneously
...
Train + PSD → Close simultaneously
PSD Doors → "Locked" → PSD Controller
Train Doors → "Locked" → Train
Train → "Ready to depart" → ATO
```

#### 7.2.2 Emergency Scenarios

**Emergency Opening (evacuation):**
```
Manual release: Break glass → Pull lever → Door opens manually
Remote release: Control center command → DCU → Unlock all doors
Power failure: Battery backup → Controlled opening → Manual if needed
```

**Emergency Closing:**
- Control center can force close
- Override passenger obstruction detection
- Used during emergencies only

---


## 10. API Interface

### 10.1 RESTful API Endpoints

#### 10.1.1 Train Position

**Endpoint:**
```
GET /api/v1/trains/{train_id}/position
```

**Response:**
```json
{
  "train_id": "IC_1234",
  "position": {
    "latitude": 51.5074,
    "longitude": -0.1278,
    "speed": 185.3,
    "heading": 45.2
  },
  "status": "in_service",
  "delay_seconds": 180,
  "next_station": "AIRPORT_TERM",
  "eta": "2025-12-26T14:42:00Z"
}
```

#### 10.1.2 Station Departures

**Endpoint:**
```
GET /api/v1/stations/{station_id}/departures
```

**Query Parameters:**
- `limit`: Number of departures (default: 10)
- `time_window`: Minutes from now (default: 60)
- `line`: Filter by line

**Response:**
```json
{
  "station_id": "CENTRAL_STN",
  "station_name": "Central Station",
  "timestamp": "2025-12-26T14:35:22Z",
  "departures": [
    {
      "train_id": "IC_1234",
      "line": "IC 5",
      "destination": "Airport Terminal",
      "scheduled": "2025-12-26T14:32:00Z",
      "estimated": "2025-12-26T14:35:00Z",
      "platform": "3A",
      "status": "delayed",
      "delay_seconds": 180
    }
  ]
}
```

#### 10.1.3 Signal Status

**Endpoint:**
```
GET /api/v1/signals/{signal_id}/status
```

**Response:**
```json
{
  "signal_id": "SIG_123A",
  "aspect": "GREEN",
  "speed_limit": 200,
  "location": {
    "milepost": 124.0,
    "track": "ML1"
  },
  "next_signal": {
    "signal_id": "SIG_124B",
    "distance": 2500,
    "aspect": "YELLOW"
  },
  "updated_at": "2025-12-26T14:35:20Z"
}
```

#### 10.1.4 Calculate Braking Distance

**Endpoint:**
```
POST /api/v1/calculate/braking-distance
```

**Request:**
```json
{
  "initial_velocity": 300,
  "deceleration_rate": 0.7,
  "reaction_time": 3,
  "safety_margin": 50
}
```

**Response:**
```json
{
  "total_distance": 5260,
  "reaction_distance": 250,
  "braking_distance": 4960,
  "safety_margin": 50,
  "time_to_stop": 119
}
```

### 10.2 WebSocket API

#### 10.2.1 Real-Time Train Updates

**Connection:**
```
wss://api.railway.example.com/ws/v1/trains/{train_id}
```

**Message Format:**
```json
{
  "type": "position_update",
  "timestamp": "2025-12-26T14:35:22.123Z",
  "train_id": "IC_1234",
  "data": {
    "latitude": 51.5074,
    "longitude": -0.1278,
    "speed": 185.3,
    "heading": 45.2
  }
}
```

#### 10.2.2 Platform Events

**Connection:**
```
wss://api.railway.example.com/ws/v1/platforms/{station_id}/{platform_id}
```

**Events:**
```json
{
  "type": "train_arriving",
  "timestamp": "2025-12-26T14:35:22Z",
  "train_id": "IC_1234",
  "eta_seconds": 30
}

{
  "type": "doors_opening",
  "timestamp": "2025-12-26T14:35:55Z",
  "train_id": "IC_1234",
  "psd_status": "opening"
}

{
  "type": "train_departing",
  "timestamp": "2025-12-26T14:37:15Z",
  "train_id": "IC_1234",
  "next_station": "AIRPORT_TERM"
}
```

---



## A.1 Endpoint reference

```http
POST /railway/v1/dispatch/grant     # grant movement authority
GET  /railway/v1/train/{id}/state    # query train state
POST /railway/v1/incident/report    # report a safety incident
GET  /railway/v1/timetable/{date}    # query published timetable
POST /railway/v1/announcement/push   # passenger information system push
```

Every endpoint follows the discovery convention at
`/.well-known/wia-railway-system`.

## A.2 ATO (Automatic Train Operation) integration

Automatic Train Operation in GoA 3 / GoA 4 modes consumes the
movement-authority envelope plus the platform-screen-door interlock
state. The standard's ATO endpoint accepts both inputs and returns
a journey-profile envelope containing the speed/position trajectory
the train will execute.

## A.3 Platform Screen Door interlock

The platform-screen-door interlock is safety-critical. The endpoint
exchanges the door-state envelope between the train and the platform
control system; both sides verify the signature before opening
doors. A signature mismatch triggers a fail-safe abort.

## A.4 Bulk export

Audit trails for safety regulators (FRA in the US, ORR in the UK,
RTRI in Japan, KORAIL Safety Office in Korea) consume bulk-export
endpoints. The export includes every dispatch grant, train-state
record, and incident report in a time window with a Merkle root
commitment.


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
