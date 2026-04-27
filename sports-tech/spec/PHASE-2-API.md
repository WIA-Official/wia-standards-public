# PHASE 2 — API

> Sports-tech API surface: performance tracking endpoints,
> smart-equipment integration verbs, and the formal API
> specification that consumers code against to drive coaching
> dashboards, equipment telemetry, and broadcast feeds.

## 5. Performance Tracking

### 5.1 Sensor Requirements

#### 5.1.1 GPS Tracking

**Minimum Requirements:**
- Frequency: 10 Hz (10 samples per second)
- Horizontal accuracy: ± 2 meters (95% confidence)
- Velocity accuracy: ± 0.2 m/s
- Cold start time: < 60 seconds
- Hot start time: < 10 seconds
- Satellite systems: GPS + GLONASS + Galileo (multi-constellation)

**Recommended:**
- Frequency: 18 Hz or higher
- RTK (Real-Time Kinematic) support for sub-meter accuracy
- Assisted GPS (A-GPS) for faster acquisition

#### 5.1.2 Heart Rate Monitoring

**ECG-based (chest strap)**
- Sampling rate: 1000 Hz
- Accuracy: ± 1 bpm or 1%, whichever is greater
- RR-interval precision: ± 1 ms
- Waterproof: IPX7 minimum
- Battery life: 400+ hours

**PPG-based (optical wrist)**
- Sampling rate: 50 Hz minimum
- Accuracy: ± 5 bpm during steady-state exercise
- Motion artifact compensation: Required
- Multiple wavelength LEDs: Recommended

#### 5.1.3 Inertial Measurement Unit (IMU)

**Accelerometer**
- Range: ± 16g minimum
- Resolution: 16-bit
- Sampling rate: 100 Hz minimum (200 Hz recommended)
- Noise density: < 150 μg/√Hz

**Gyroscope**
- Range: ± 2000 °/s minimum
- Resolution: 16-bit
- Sampling rate: 100 Hz minimum
- Zero-rate offset: < 10 °/s

**Magnetometer**
- Range: ± 4900 μT
- Resolution: 16-bit
- Sampling rate: 10 Hz minimum
- Heading accuracy: ± 2°

#### 5.1.4 Power Measurement

**Cycling**
- Accuracy: ± 2%
- Cadence range: 20-220 rpm
- Power range: 0-4000 watts
- Transmission: ANT+ and Bluetooth Smart
- Update rate: 1 Hz minimum

**Running**
- Accuracy: ± 5% (estimated from biomechanics)
- Sampling rate: 1 Hz
- Calibration: Required for each athlete

### 5.2 Data Collection Protocols

#### 5.2.1 Real-time Streaming

**Transport Protocol:**
- Primary: WebSocket (for bidirectional communication)
- Fallback: MQTT (for unreliable networks)
- Binary format: Protocol Buffers or MessagePack for efficiency

**Quality of Service:**
- Critical metrics (injury alerts): QoS 2 (exactly once)
- Performance metrics: QoS 1 (at least once)
- Environmental data: QoS 0 (at most once)

**Latency Requirements:**
| Data Type | Maximum Acceptable Latency |
|-----------|----------------------------|
| Injury alerts | 50 ms |
| Real-time feedback | 100 ms |
| Performance metrics | 500 ms |
| Environmental data | 5000 ms |

**Bandwidth Optimization:**
- Use data compression (gzip, brotli)
- Implement delta encoding (send only changes)
- Adaptive sampling rates based on network conditions
- Local buffering during connectivity loss

#### 5.2.2 Post-Session Upload

**File Format:**
- Primary: WIA JSON (comprehensive data)
- Legacy support: FIT, TCX, GPX formats
- Compression: gzip required for files > 1 MB

**Upload Priority:**
1. Session summary (immediate)
2. Critical alerts and flags (immediate)
3. Performance metrics (within 5 minutes)
4. Raw sensor data (within 1 hour)
5. Video/media files (background, within 24 hours)

**Retry Logic:**
- Exponential backoff: 1s, 2s, 4s, 8s, 16s
- Maximum retries: 5 attempts
- Persistent queue: Store locally until upload succeeds

### 5.3 Metric Calculations

#### 5.3.1 Training Load

**Session RPE Method:**
```
Training Load (AU) = Duration (minutes) × RPE (1-10 scale)

Example:
60 minute run at RPE 7 = 60 × 7 = 420 AU
```

**Heart Rate Based (TRIMP):**
```
TRIMP = Duration (min) × ΔHR × 0.64 × e^(1.92 × ΔHR)

Where:
ΔHR = (Exercise HR - Resting HR) / (Max HR - Resting HR)

Example:
Duration = 60 min
Resting HR = 50 bpm
Exercise HR = 150 bpm (average)
Max HR = 190 bpm

ΔHR = (150 - 50) / (190 - 50) = 0.714
TRIMP = 60 × 0.714 × 0.64 × e^(1.92 × 0.714)
TRIMP ≈ 175 AU
```

**Power Based (TSS for cycling):**
```
TSS = (Duration (sec) × NP × IF) / (FTP × 3600) × 100

Where:
NP = Normalized Power
IF = Intensity Factor = NP / FTP
FTP = Functional Threshold Power

Example:
Duration = 3600 sec (1 hour)
NP = 220 watts
FTP = 250 watts
IF = 220 / 250 = 0.88

TSS = (3600 × 220 × 0.88) / (250 × 3600) × 100
TSS ≈ 77
```

#### 5.3.2 Acute:Chronic Workload Ratio

```
ACWR = Acute Load / Chronic Load

Acute Load = 7-day rolling average
Chronic Load = 28-day rolling average

Safe Zone: 0.8 - 1.3
Moderate Risk: 1.3 - 1.5 or 0.5 - 0.8
High Risk: > 1.5 or < 0.5

Example:
Last 7 days: 350, 420, 0, 500, 380, 450, 0
Acute Load = (350 + 420 + 0 + 500 + 380 + 450 + 0) / 7 = 300 AU

Last 28 days average = 285 AU

ACWR = 300 / 285 = 1.05 (Safe Zone ✓)
```

#### 5.3.3 VO2 Max Estimation

**Cooper Test (12-minute run):**
```
VO2max (ml/kg/min) = (Distance in meters - 504.9) / 44.73

Example:
Distance = 3000 meters
VO2max = (3000 - 504.9) / 44.73 ≈ 55.8 ml/kg/min
```

**Heart Rate Based (Firstbeat method):**
```
VO2max = 15.3 × (MaxHR / RestingHR)

Example:
MaxHR = 190 bpm
RestingHR = 50 bpm
VO2max = 15.3 × (190 / 50) ≈ 58.1 ml/kg/min
```

**Running Speed Based:**
```
VO2 (ml/kg/min) = 0.2 × Speed (m/min) + 0.9 × Speed × Grade (%) + 3.5

At maximal effort:
VO2max ≈ 0.2 × Max Speed (m/min) + 3.5

Example:
Max speed = 6:00 min/mile = 268 m/min
VO2max = 0.2 × 268 + 3.5 ≈ 57.1 ml/kg/min
```

---


## 6. Smart Equipment Integration

### 6.1 Smart Ball Specifications

#### 6.1.1 Soccer Ball

**Sensors:**
- 6-axis IMU (accelerometer + gyroscope)
- Impact sensor (force measurement)
- Bluetooth 5.0 Low Energy
- NFC for pairing and identification

**Measurements:**
- Ball speed: 0-200 km/h (± 1%)
- Spin rate: 0-4000 rpm (± 2%)
- Impact force: 0-3000 N
- Flight trajectory: 3D position tracking
- Kick type classification: instep, side-foot, chip, etc.

**Physical Requirements:**
- FIFA Quality Pro certified weight/size
- Waterproof: IP67
- Battery: 2000+ kicks, USB-C rechargeable
- Wireless charging compatible

**Data Output (JSON):**
```json
{
  "ballId": "BALL-2025-042",
  "timestamp": "2025-03-14T15:30:27.543Z",
  "eventType": "kick",
  "kinematics": {
    "speed": {
      "initial": 95.3,
      "peak": 98.7,
      "unit": "km/h"
    },
    "spin": {
      "rate": 2150,
      "axis": {"x": 0.2, "y": 0.8, "z": 0.1},
      "unit": "rpm"
    },
    "trajectory": {
      "angle": 25.3,
      "curve": "right",
      "curveMagnitude": 1.8
    }
  },
  "impact": {
    "force": 1850,
    "location": {"x": 0.3, "y": 0.5},
    "contactTime": 12,
    "unit": {
      "force": "N",
      "contactTime": "ms"
    }
  },
  "classification": {
    "kickType": "instep",
    "confidence": 0.94,
    "bodyPart": "right_foot"
  }
}
```

#### 6.1.2 Basketball

**Sensors:**
- 9-axis IMU
- Pressure sensor (inflation monitoring)
- Bluetooth 5.0
- Shot tracking algorithm

**Measurements:**
- Shot arc: 30-60 degrees optimal
- Release angle
- Spin rate: 1-3 rotations per second
- Shot make/miss detection
- Dribble count and pattern

#### 6.1.3 Golf Ball

**Sensors:**
- Micro IMU chip
- Impact sensor
- UWB (Ultra-Wideband) for precise location

**Measurements:**
- Ball speed off club: 0-300 km/h
- Launch angle: -10 to 60 degrees
- Spin rate: 0-10000 rpm
- Carry distance
- Landing angle

### 6.2 Smart Racket (Tennis)

**Sensors:**
- Handle-mounted IMU
- String bed impact sensor (piezoelectric)
- Force/torque sensors in grip

**Measurements:**
- Swing speed: 0-200 km/h
- Impact location (sweet spot analysis)
- Spin rate applied to ball
- Stroke type (forehand/backhand/serve/volley)
- Vibration dampening effectiveness

**Haptic Feedback:**
- Real-time technique correction
- Overhit warning
- Optimal timing indicator

### 6.3 Smart Shoes

**Sensors (per shoe):**
- 16 pressure sensors (insole array)
- 6-axis IMU (forefoot and heel)
- Temperature sensor
- Bluetooth 5.0

**Measurements:**
- Foot strike pattern (heel/midfoot/forefoot)
- Pronation/supination angle
- Ground contact time per foot
- Balance distribution
- Impact force (Newtons)
- Cadence asymmetry

**Applications:**
- Running form analysis
- Injury risk from improper landing
- Gait rehabilitation
- Performance optimization

---


## 11. API Specifications

### 11.1 RESTful API Endpoints

**Base URL:** `https://api.wia-sports.com/v1`

**Authentication:** Bearer token (JWT) in Authorization header

#### 11.1.1 Athlete Management

**Create Athlete Profile**
```http
POST /athletes
Content-Type: application/json
Authorization: Bearer {token}

{
  "personalInfo": { ... },
  "biometrics": { ... },
  "sportInfo": { ... }
}

Response: 201 Created
{
  "athleteId": "ATH-2025-001",
  "created": "2025-03-14T10:00:00Z"
}
```

**Get Athlete Profile**
```http
GET /athletes/{athleteId}
Authorization: Bearer {token}

Response: 200 OK
{
  "athleteProfile": { ... }
}
```

**Update Athlete Profile**
```http
PATCH /athletes/{athleteId}
Content-Type: application/json
Authorization: Bearer {token}

{
  "biometrics": {
    "weight": 76.2
  }
}

Response: 200 OK
```

**Delete Athlete Profile**
```http
DELETE /athletes/{athleteId}
Authorization: Bearer {token}

Response: 204 No Content
```

#### 11.1.2 Session Management

**Create Session**
```http
POST /athletes/{athleteId}/sessions
Content-Type: application/json
Authorization: Bearer {token}

{
  "sessionType": "training",
  "sport": "soccer",
  "timestamp": {
    "start": "2025-03-14T15:00:00Z"
  },
  "venue": { ... },
  "environment": { ... }
}

Response: 201 Created
{
  "sessionId": "SESSION-2025-03-14-001",
  "uploadUrl": "https://upload.wia-sports.com/session-data/{presigned-url}"
}
```

**Upload Session Data (Streaming)**
```http
POST /sessions/{sessionId}/data/stream
Content-Type: application/octet-stream
Authorization: Bearer {token}
Transfer-Encoding: chunked

[Binary Protocol Buffer stream]

Response: 202 Accepted
```

**Get Session Summary**
```http
GET /sessions/{sessionId}
Authorization: Bearer {token}

Response: 200 OK
{
  "session": { ... },
  "performanceMetrics": { ... },
  "skillMetrics": { ... },
  "injuryRisk": { ... }
}
```

**List Athlete Sessions**
```http
GET /athletes/{athleteId}/sessions?from=2025-03-01&to=2025-03-14&type=training
Authorization: Bearer {token}

Response: 200 OK
{
  "sessions": [
    {
      "sessionId": "...",
      "timestamp": { ... },
      "summary": { ... }
    }
  ],
  "pagination": {
    "total": 42,
    "page": 1,
    "pageSize": 20
  }
}
```

#### 11.1.3 Training Plans

**Create Training Plan**
```http
POST /athletes/{athleteId}/training-plans
Content-Type: application/json
Authorization: Bearer {token}

{
  "validFrom": "2025-03-15",
  "validTo": "2025-06-15",
  "goal": { ... },
  "mesocycles": [ ... ]
}

Response: 201 Created
{
  "planId": "PLAN-2025-001"
}
```

**Get AI-Generated Plan**
```http
POST /athletes/{athleteId}/training-plans/generate
Content-Type: application/json
Authorization: Bearer {token}

{
  "goal": "marathon_sub_3:30",
  "targetDate": "2025-10-15",
  "currentFitness": { ... },
  "constraints": { ... }
}

Response: 200 OK
{
  "trainingPlan": { ... },
  "rationale": "Based on your current VO2max of 56 ml/kg/min..."
}
```

#### 11.1.4 Injury Risk API

**Get Current Risk Assessment**
```http
GET /athletes/{athleteId}/injury-risk
Authorization: Bearer {token}

Response: 200 OK
{
  "injuryRisk": { ... }
}
```

**Get Risk Timeline**
```http
GET /athletes/{athleteId}/injury-risk/timeline?days=30
Authorization: Bearer {token}

Response: 200 OK
{
  "timeline": [
    {
      "date": "2025-03-14",
      "overallRisk": 0.23,
      "hamstring": 0.18,
      "acl": 0.12
    },
    ...
  ]
}
```

### 11.2 WebSocket API (Real-time)

**Connection:**
```
wss://stream.wia-sports.com/v1/realtime
Authorization: Bearer {token}
```

**Subscribe to Session Data**
```json
{
  "action": "subscribe",
  "channel": "session",
  "sessionId": "SESSION-2025-03-14-001",
  "dataTypes": ["heart_rate", "speed", "power"]
}
```

**Server Messages (Real-time data)**
```json
{
  "channel": "session",
  "sessionId": "SESSION-2025-03-14-001",
  "timestamp": "2025-03-14T15:30:45.123Z",
  "data": {
    "heart_rate": 165,
    "speed": 18.5,
    "power": 285
  }
}
```

**Injury Alert**
```json
{
  "channel": "alert",
  "athleteId": "ATH-2025-001",
  "timestamp": "2025-03-14T15:35:12.456Z",
  "alert": {
    "severity": "warning",
    "type": "asymmetry",
    "message": "Left-right ground contact time asymmetry increased to 8.2%",
    "recommendations": ["Monitor closely", "Consider stopping if worsens"]
  }
}
```

### 11.3 Rate Limiting

| Tier | Requests/minute | WebSocket connections |
|------|-----------------|----------------------|
| Free | 60 | 1 |
| Athlete | 300 | 3 |
| Team | 1000 | 50 |
| Enterprise | 10000 | 500 |

**Rate limit headers:**
```http
X-RateLimit-Limit: 300
X-RateLimit-Remaining: 287
X-RateLimit-Reset: 1647270000
```

**Exceeded response:**
```http
HTTP/1.1 429 Too Many Requests
Retry-After: 42

{
  "error": "rate_limit_exceeded",
  "message": "Rate limit exceeded. Try again in 42 seconds."
}
```

---


