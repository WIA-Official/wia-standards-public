# WIA-IND-014 — Phase 1: Data Format

> Smart-gym canonical envelopes: facility architecture, connected-equipment descriptors, member profile, and data-format shapes.

## 1. Introduction

### 1.1 Purpose

This specification defines a comprehensive technical framework for smart gym facilities that integrate connected fitness equipment, automated member management, AI-powered workout planning, facility IoT sensors, virtual training capabilities, and advanced performance analytics to create an optimal fitness environment.

### 1.2 Scope

The standard covers:
- Connected fitness equipment specifications and data protocols
- Member management systems with automated check-in and tracking
- AI-powered personalized workout program generation
- Facility IoT sensor networks for environmental and operational monitoring
- Virtual training platforms including live streaming and AR/VR
- Performance analytics and progress tracking systems
- Safety monitoring including form analysis and injury prevention
- Integration with wearable devices and mobile applications

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard aims to democratize access to professional-grade fitness technology, making high-quality training available to everyone regardless of economic status, while optimizing gym operations and creating healthier communities through intelligent automation.

### 1.4 Terminology

- **Smart Equipment**: Fitness machines with embedded IoT sensors and connectivity
- **Member Profile**: Comprehensive data including goals, restrictions, preferences, history
- **Progressive Overload**: Systematic increase in training stress for adaptation
- **Training Volume**: Total work performed (Weight × Reps × Sets)
- **1RM (One-Rep Max)**: Maximum weight lifted for one repetition
- **RPE (Rate of Perceived Exertion)**: Subjective difficulty scale (1-10)
- **MET (Metabolic Equivalent)**: Energy expenditure measure
- **VO2 Max**: Maximum oxygen uptake capacity
- **Form Analysis**: Computer vision-based movement quality assessment
- **Virtual PT**: Remote personal training via video/AR

---


## 2. Smart Gym Architecture

### 2.1 System Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                      Smart Gym Facility                         │
├─────────────────────────────────────────────────────────────────┤
│  Equipment Layer                                                 │
│  ├── Cardio Machines (Treadmills, Bikes, Rowers, Ellipticals)  │
│  ├── Strength Machines (Cables, Presses, Leg Equipment)        │
│  ├── Free Weight Area (Barbells, Dumbbells, Kettlebells)       │
│  ├── Functional Training Zone (TRX, Plyometrics, CrossFit)     │
│  └── Group Exercise Rooms (Spin, Yoga, HIIT Studios)           │
├─────────────────────────────────────────────────────────────────┤
│  IoT Sensor Network                                              │
│  ├── Equipment Sensors (Load, Position, Usage, HR Integration) │
│  ├── Environmental Sensors (Temp, Humidity, CO2, Air Quality)  │
│  ├── Occupancy Sensors (Cameras, IR, Weight Pads)              │
│  ├── Access Control (RFID, NFC, Biometric, Mobile App)         │
│  └── Safety Systems (Emergency Buttons, AED Monitors)          │
├─────────────────────────────────────────────────────────────────┤
│  Edge Computing Layer                                            │
│  ├── Local Equipment Controllers                                │
│  ├── Computer Vision Processors (Form Analysis)                │
│  ├── Real-time Analytics Engine                                │
│  └── Local Data Storage (Edge Cache)                           │
├─────────────────────────────────────────────────────────────────┤
│  Connectivity Layer                                              │
│  ├── WiFi 6 (802.11ax) - Primary network                       │
│  ├── Bluetooth 5.2 - Equipment pairing                         │
│  ├── Zigbee/Z-Wave - IoT sensors                               │
│  └── 5G/LTE - Backup & mobile apps                             │
└─────────────────────────────────────────────────────────────────┘
                              ↕
┌─────────────────────────────────────────────────────────────────┐
│                      Cloud Platform                              │
├─────────────────────────────────────────────────────────────────┤
│  Member Management                                               │
│  ├── Registration & Profiles                                    │
│  ├── Membership Plans & Billing                                │
│  ├── Check-in/out Tracking                                     │
│  └── Mobile App Backend                                        │
├─────────────────────────────────────────────────────────────────┤
│  AI/ML Services                                                  │
│  ├── Workout Plan Generator                                    │
│  ├── Form Analysis Models                                      │
│  ├── Injury Risk Prediction                                    │
│  ├── Member Retention Prediction                               │
│  └── Equipment Maintenance Forecasting                         │
├─────────────────────────────────────────────────────────────────┤
│  Data & Analytics                                                │
│  ├── Workout History Database                                  │
│  ├── Performance Metrics Warehouse                             │
│  ├── Equipment Usage Analytics                                 │
│  ├── Facility Optimization Engine                              │
│  └── Business Intelligence Dashboard                           │
├─────────────────────────────────────────────────────────────────┤
│  Virtual Training                                                │
│  ├── Live Class Streaming (WebRTC)                             │
│  ├── On-Demand Video Library                                   │
│  ├── Virtual PT Sessions                                       │
│  ├── AR/VR Workout Environments                                │
│  └── Social Features & Challenges                              │
├─────────────────────────────────────────────────────────────────┤
│  Integration Layer                                               │
│  ├── Wearable Device APIs (Fitbit, Garmin, Apple Watch)       │
│  ├── Nutrition Apps (MyFitnessPal, etc.)                      │
│  ├── Health Records (FHIR/HL7)                                │
│  └── Third-party Equipment (Peloton, Mirror, etc.)            │
└─────────────────────────────────────────────────────────────────┘
```

### 2.2 Data Flow Patterns

#### 2.2.1 Member Check-in Flow

```
1. Member → Access Control (RFID/App/Biometric)
2. Access System → Verify Membership Status
3. System → Load Member Profile & Today's Workout
4. Display → Welcome Message + Recommended Equipment
5. Analytics → Update Attendance Tracking
6. Notifications → Send to Member's App
```

#### 2.2.2 Workout Session Flow

```
1. Member → Selects Equipment
2. Equipment → Reads Member ID (NFC/QR/Manual)
3. System → Loads Member Settings & Last Workout
4. Equipment → Configures (Seat height, resistance, etc.)
5. Member → Starts Workout
6. Equipment → Streams Real-time Data
   • Speed/Resistance/Weight
   • Heart Rate (if connected)
   • Form metrics (camera)
   • Power output
7. AI → Analyzes Performance & Provides Feedback
8. System → Detects Workout Completion
9. Database → Saves Session Data
10. Member → Receives Summary & Recommendations
```

#### 2.2.3 Virtual Class Flow

```
1. Member → Books Class via App
2. System → Sends Reminder Notifications
3. Class Time → Member Joins (Mobile/Studio Screen)
4. Instructor → Streams Live Video + Audio
5. System → Overlays Member Metrics (HR, Calories)
6. Analytics → Tracks Participation & Performance
7. Post-Class → Saves Recording + Member Stats
8. System → Sends Follow-up & Next Class Suggestions
```

---


## 3. Connected Equipment Systems

### 3.1 Cardio Equipment

#### 3.1.1 Treadmill Specifications

**Hardware Requirements:**
- Motor: 2.5-4.0 HP continuous duty
- Speed Range: 0-20 km/h (0-12.5 mph)
- Incline Range: 0-15% (motorized)
- Belt Size: Minimum 140cm × 50cm
- Weight Capacity: 150 kg minimum

**IoT Sensors:**
- Speed sensor: ±0.1 km/h accuracy
- Incline sensor: ±0.5% accuracy
- Heart rate: Dual-mode (handlebar + chest strap)
- Footstrike sensors: 16-point pressure mapping
- Emergency stop: Magnetic safety key + button

**Data Collection (1 Hz minimum):**
```json
{
  "timestamp": "2025-12-27T10:30:45Z",
  "deviceId": "TREAD-001",
  "memberId": "MEM-12345",
  "speed": 10.5,
  "incline": 5.0,
  "distance": 3.25,
  "duration": 1860,
  "heartRate": 155,
  "caloriesBurned": 287,
  "stepCount": 4250,
  "avgStepLength": 0.76,
  "footstrikePattern": "midfoot",
  "cadence": 168,
  "verticalOscillation": 8.2,
  "groundContactTime": 245
}
```

**Advanced Features:**
- Auto-speed adjustment based on heart rate zones
- Virtual running trails with terrain simulation
- Form analysis via side-mounted camera
- Social running: Race against other members
- Integration with running apps (Strava, Nike Run Club)

#### 3.1.2 Exercise Bike Specifications

**Hardware Requirements:**
- Resistance: Magnetic or electronic (20+ levels)
- Flywheel: 15 kg minimum for smooth ride
- Adjustability: Seat (vertical/horizontal), handlebar height
- Pedals: SPD-compatible with toe cages
- Display: Touch screen 10"+ with Bluetooth

**IoT Sensors:**
- Cadence sensor: ±1 RPM accuracy
- Power meter: ±2% accuracy
- Heart rate: Dual-mode
- Seat pressure sensor: Comfort optimization

**Data Collection:**
```json
{
  "timestamp": "2025-12-27T10:30:45Z",
  "deviceId": "BIKE-015",
  "memberId": "MEM-12345",
  "resistance": 12,
  "cadence": 85,
  "power": 185,
  "distance": 12.5,
  "duration": 2400,
  "heartRate": 148,
  "caloriesBurned": 320,
  "ftp": 210,
  "normalizedPower": 192,
  "intensityFactor": 0.91,
  "trainingStressScore": 45
}
```

#### 3.1.3 Rowing Machine Specifications

**Hardware Requirements:**
- Resistance: Air, water, or magnetic
- Rail Length: Accommodate 220 cm height
- Monitor: PM5 or equivalent
- Footrests: Adjustable with secure straps

**Data Collection:**
```json
{
  "timestamp": "2025-12-27T10:30:45Z",
  "deviceId": "ROW-008",
  "memberId": "MEM-12345",
  "strokeRate": 28,
  "pace": "1:52.3",
  "distance": 5000,
  "duration": 1200,
  "power": 165,
  "caloriesPerHour": 720,
  "heartRate": 162,
  "splitTimes": {
    "500m": ["1:55.2", "1:52.8", "1:51.5", "1:50.9", "1:49.8"],
    "1000m": ["1:54.0", "1:52.3", "1:50.3"]
  },
  "strokeAnalysis": {
    "driveTime": 1.2,
    "recoveryTime": 1.8,
    "driveRecoveryRatio": 1.5,
    "peakForce": 520
  }
}
```

### 3.2 Strength Equipment

#### 3.2.1 Smart Weight Machines

**Cable Machine Specifications:**
- Load cell sensors: ±0.5 kg accuracy
- Position sensors: Track range of motion
- Rep counter: Automatic via load pattern
- Safety features: Overload detection

**Data Collection:**
```json
{
  "timestamp": "2025-12-27T10:30:45Z",
  "deviceId": "CABLE-003",
  "memberId": "MEM-12345",
  "exercise": "lat-pulldown",
  "weight": 70,
  "reps": 12,
  "sets": 3,
  "rangeOfMotion": {
    "min": 45,
    "max": 175,
    "avgROM": 130
  },
  "repTempo": {
    "concentric": 1.2,
    "pause": 0.5,
    "eccentric": 2.0
  },
  "timeUnderTension": 44,
  "peakForce": 85,
  "avgForce": 72,
  "powerOutput": 145,
  "formScore": 8.5,
  "restTime": 90
}
```

#### 3.2.2 Free Weight Tracking

**RFID Weight Plate System:**
- Embedded RFID chips in plates
- Detection mats under lifting areas
- Barbell sleeves with RFID readers
- Automatic exercise recognition

**Computer Vision Tracking:**
- Overhead cameras with pose estimation
- Real-time form analysis
- Rep counting and tempo tracking
- Safety monitoring (spotter alerts)

**Data Collection:**
```json
{
  "timestamp": "2025-12-27T10:30:45Z",
  "location": "FREE-ZONE-A",
  "memberId": "MEM-12345",
  "exercise": "barbell-squat",
  "detectedBy": "computer-vision",
  "weight": 100,
  "reps": 8,
  "sets": 4,
  "depth": {
    "avg": "parallel",
    "min": "parallel-plus-2cm",
    "max": "atg"
  },
  "formAnalysis": {
    "kneeValgus": "none",
    "spineNeutrality": "good",
    "barPath": "vertical",
    "hipDrive": "excellent",
    "overallScore": 9.2
  },
  "velocity": {
    "avg": 0.65,
    "peak": 1.2,
    "lossFactor": 0.15
  },
  "estimated1RM": 125,
  "relativeStrength": 1.67
}
```

---


## 10. Data Formats

### 10.1 Workout Session Schema

**Complete Workout JSON:**
```json
{
  "standardId": "WIA-IND-014",
  "version": "1.0.0",
  "sessionId": "SESSION-2025-1227-0001",
  "facilityId": "GYM-2025-001",
  "memberId": "MEM-12345",
  "timestamp": {
    "start": "2025-12-27T10:00:00Z",
    "end": "2025-12-27T11:15:00Z",
    "duration": 4500
  },
  "sessionType": "strength-training",
  "exercises": [
    {
      "exerciseId": "EX-001",
      "name": "Barbell Back Squat",
      "equipment": "barbell",
      "muscleGroups": ["quadriceps", "glutes", "hamstrings", "core"],
      "sets": [
        {
          "setNumber": 1,
          "type": "warmup",
          "weight": 60,
          "reps": 10,
          "rpe": 5,
          "restTime": 90,
          "formScore": 8.5,
          "avgVelocity": 0.75,
          "peakVelocity": 1.2,
          "rangeOfMotion": "full",
          "tempo": { "eccentric": 2.5, "pause": 1.0, "concentric": 1.0 }
        },
        {
          "setNumber": 2,
          "type": "working",
          "weight": 100,
          "reps": 8,
          "rpe": 7,
          "restTime": 180,
          "formScore": 9.2,
          "avgVelocity": 0.65,
          "peakVelocity": 1.0,
          "rangeOfMotion": "full",
          "tempo": { "eccentric": 2.0, "pause": 1.0, "concentric": 1.2 }
        }
      ],
      "totalVolume": 1400,
      "estimated1RM": 125,
      "notes": "Excellent depth and bar path"
    }
  ],
  "summary": {
    "totalExercises": 8,
    "totalSets": 24,
    "totalReps": 192,
    "totalVolume": 12500,
    "avgHeartRate": 135,
    "maxHeartRate": 172,
    "caloriesBurned": 485,
    "timeUnderTension": 1260,
    "avgRestTime": 120
  },
  "memberStats": {
    "preWorkout": {
      "weight": 75.2,
      "heartRate": 62,
      "bloodPressure": { "systolic": 118, "diastolic": 76 },
      "readinessScore": 8.5
    },
    "postWorkout": {
      "weight": 74.8,
      "heartRate": 88,
      "perceivedExertion": 8,
      "muscleSoreness": 5
    }
  },
  "achievements": [
    { "type": "personal-record", "exercise": "squat", "value": 125, "unit": "kg" },
    { "type": "volume-milestone", "description": "50,000 kg total volume this month" }
  ]
}
```

---



## A.1 Canonical envelope conventions

Every Phase 1 smart-gym envelope follows the WIA family baseline:
UTF-8 JSON, RFC 8785 canonicalisation, Ed25519 signatures, ULID
identifiers. Personal-fitness data is sensitive PII and inherits
the WIA Secure Enclave sealed-data envelope.

## A.2 Member profile envelope

```json
{
  "wia_smart_gym_version": "1.0.0",
  "type": "member_profile",
  "member_id": "did:wia:member:01HX...",
  "facility_id": "fac_01HX...",
  "membership_tier": "standard" | "premium" | "founder",
  "consent_for_telemetry": true,
  "consent_for_personalisation": true,
  "issued_at": "RFC 3339",
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

PII fields (full name, date of birth, phone, address) are stored
under WIA-OMNI-API credentials and never leave the backend. The
public envelope carries opaque references so downstream systems
operate without exposing PII.

## A.3 Connected-equipment descriptor

Each piece of connected equipment (treadmill, weight stack, rower,
indoor cycle) publishes a descriptor with manufacturer, model,
firmware version, supported telemetry channels, and calibration
state. The descriptor is signed by the facility owner so the
provenance chain is auditable for warranty and safety.

## A.4 Workout-session record

Workout sessions are recorded with member ID (with consent),
equipment ID, start/end timestamps, performance telemetry, and
the trainer ID (if present). The record is the basis for
performance analytics; access is gated on member consent.


## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/smart-gym/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-smart-gym-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/smart-gym-host:1.0.0` ships every Phase 2 endpoint with mock
data for integrators to test against. The companion CLI at
`cli/smart-gym.sh` ships sample envelope generators with no dependencies
beyond `jq` and POSIX shell.

## Z.2 Cross-standard composition (recap)

This Phase composes with WIA-OMNI-API for credential storage,
WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 §5 for
federation handshake, WIA-INTENT for workload intent declaration,
and (where personal data is processed) WIA Secure Enclave for
sealed-data envelopes. The composition lets one host running
multiple WIA family standards reuse one identity, signature, and
audit machinery rather than maintaining N parallel implementations.

## Z.3 Implementation runbook

A first implementation typically follows: stand up reference
container; run conformance suite against it; replace mock backend
with real backend one endpoint at a time; wire audit log
replication; onboard a single trusted peer for federation; expand
to multiple peers; promote to production with warning-envelope
subscription.

## Z.4 Backwards-compatibility promise

Within the 1.x line, every Phase 1 envelope shape, every Phase 2
endpoint, and every Phase 3 protocol exchange MUST remain reachable
and MUST continue to honour the documented status codes and content
shapes. Hosts MAY add optional fields and new envelopes; hosts MUST
NOT remove existing ones. Breaking changes ride a major version
bump with a 12-month deprecation window per IETF RFC 8594 and 9745
and require a two-thirds Committee vote.

## Z.5 Closing implementer note

This Phase fits inside the standards four-Phase architecture: Phase
1 envelopes are the wire-format contract; Phase 2 surfaces them
through HTTPS; Phase 3 wraps them in protocol exchanges that cross
trust boundaries; Phase 4 integrates with the broader ecosystem.

弘益人間 — Benefit All Humanity.
