# PHASE 3 — Protocol

> Sports-tech operational protocols: injury prevention pipeline,
> training optimization, the on-the-wire protocol definitions
> for sensor and device coordination, and the interoperability
> contract between vendor implementations.

## 7. Injury Prevention System

### 7.1 Risk Assessment Models

#### 7.1.1 Hamstring Injury Prediction

**Input Features:**
- Acute:Chronic Workload Ratio
- Sprint distance (last 7 days)
- Left-right leg strength asymmetry (%)
- Previous hamstring injury (binary)
- Fatigue score (0-100)
- Hip flexion range of motion
- Hamstring:Quadriceps strength ratio
- Age
- Recent match schedule density

**Model Architecture:**
```
Deep Neural Network:
- Input layer: 9 features (normalized)
- Hidden layer 1: 64 neurons (ReLU)
- Dropout: 0.3
- Hidden layer 2: 32 neurons (ReLU)
- Dropout: 0.2
- Hidden layer 3: 16 neurons (ReLU)
- Output layer: 1 neuron (Sigmoid)

Training:
- Dataset: 15,000 athlete-seasons
- Validation: 20% holdout
- Optimizer: Adam (lr=0.001)
- Loss: Binary cross-entropy
- Metrics: AUC-ROC, Precision, Recall

Performance:
- AUC-ROC: 0.87
- Sensitivity: 82%
- Specificity: 79%
- PPV: 23% (at 10% injury prevalence)
```

**Risk Thresholds:**
- Low risk: < 0.15 probability (Green)
- Moderate risk: 0.15-0.40 (Yellow)
- High risk: 0.40-0.70 (Orange)
- Critical risk: > 0.70 (Red)

**Recommendations by Risk Level:**
| Risk Level | Action |
|------------|--------|
| Low | Continue normal training |
| Moderate | Monitor closely, optional preventive exercises |
| High | Reduce sprint volume by 30%, add strengthening |
| Critical | Medical assessment required, modified training only |

#### 7.1.2 ACL Injury Risk (Dynamic)

**Real-time Biomechanical Analysis:**

Monitored during cutting/landing movements:
- Knee valgus angle (> 10° = risk factor)
- Hip adduction angle
- Knee flexion at initial contact (< 20° = risk)
- Ground reaction force asymmetry
- Landing stiffness (vertical vs. horizontal deceleration)

**Machine Learning Model:**
```
Random Forest Classifier:
- Trees: 100
- Max depth: 10
- Min samples split: 20
- Features: 15 kinematic/kinetic variables

Real-time Inference:
- Processing time: < 50ms per movement
- Edge computing: On-device processing
- Alert latency: < 100ms total

Alert Types:
- Visual: LED indicator on wearable
- Haptic: Vibration pulse
- Audio: Beep or voice warning (optional)
- Coach notification: Push to tablet/phone
```

### 7.2 Fatigue Monitoring

#### 7.2.1 Acute Fatigue Detection

**Physiological Markers:**
- Heart rate recovery (% decrease in 1 min post-exercise)
- HRV decline (> 10% from baseline = fatigued)
- Elevated resting heart rate (> 5 bpm from baseline)
- Reduced overnight HRV (RMSSD)

**Performance Markers:**
- Sprint speed decline (> 5% from session start)
- Power output drop (> 10% in repeated efforts)
- Increased asymmetry (> 3% change)
- Reaction time increase (> 50 ms)

**Subjective Assessment:**
- Rating of Perceived Exertion (RPE > 8/10)
- Wellness questionnaire scores
- Sleep quality (< 6 hours or < 70% quality)

**Fatigue Score Calculation:**
```
Fatigue Score (0-100) = Weighted Average:
- HRV decline: 30%
- Sleep quality: 25%
- Performance decrement: 20%
- RPE: 15%
- Subjective wellness: 10%

Thresholds:
- Fresh: 0-30
- Normal: 31-50
- Fatigued: 51-70
- Severely Fatigued: 71-100

Example:
HRV decline: 15% → Score = 60 (normalized)
Sleep: 5.5 hours → Score = 70
Performance: 7% drop → Score = 70
RPE: 8/10 → Score = 80
Wellness: Feeling okay → Score = 50

Fatigue = (60×0.3 + 70×0.25 + 70×0.2 + 80×0.15 + 50×0.1)
        = 18 + 17.5 + 14 + 12 + 5
        = 66.5 (Fatigued)

Recommendation: Reduce training load by 40% for next 24 hours
```

#### 7.2.2 Chronic Fatigue (Overtraining)

**Monitoring Window:** 4-week rolling assessment

**Red Flags (≥ 3 indicates overtraining):**
1. Consistently elevated resting HR (> 5 bpm for 7+ days)
2. Suppressed HRV (> 15% below baseline for 7+ days)
3. Performance plateau or decline (no improvement in 2 weeks)
4. Increased injury occurrence
5. Mood disturbances (irritability, depression)
6. Sleep disruption (difficulty falling/staying asleep)
7. Reduced appetite
8. Increased illness frequency

**Recovery Protocol:**
- Week 1: 50% training volume, no high-intensity
- Week 2: 60% volume, introduce moderate intensity
- Week 3: 75% volume, gradual return to normal
- Week 4: 90% volume, monitor closely
- Reassess after 4 weeks before returning to 100%

### 7.3 Movement Quality Analysis

#### 7.3.1 Asymmetry Detection

**Bilateral Metrics:**

Running:
```
Left-Right Difference (%) = |Left Value - Right Value| / Average × 100

Acceptable ranges:
- Ground contact time: < 5%
- Stride length: < 3%
- Peak force: < 10%
- Power output: < 8%

Example:
Left GCT = 240 ms
Right GCT = 255 ms
Avg = 247.5 ms

Asymmetry = |240 - 255| / 247.5 × 100 = 6.1%
Status: ALERT (exceeds 5% threshold)

Possible causes:
- Previous injury compensation
- Strength imbalance
- Leg length discrepancy
- Footwear issue

Recommendation:
- Video gait analysis
- Strength assessment (isokinetic testing)
- Physical therapy evaluation
```

**Corrective Actions:**
1. Unilateral strength training (focus on weaker side)
2. Plyometric exercises for symmetry
3. Gait retraining drills
4. Regular monitoring (weekly assessment)

#### 7.3.2 Compensatory Patterns

**Common Patterns:**

| Pattern | Description | Injury Risk |
|---------|-------------|-------------|
| Trendelenburg | Hip drop during single-leg stance | Hip/knee pain |
| Valgus Collapse | Knee inward during landing | ACL, MCL injury |
| Early Heel Rise | Premature heel lift in gait | Achilles issues |
| Over-striding | Landing ahead of center of mass | Knee, shin splints |
| Excessive Rotation | Trunk twist during movement | Lower back pain |

**Detection Method:**
- Video analysis with pose estimation AI
- IMU-based joint angle calculation
- Pressure sensor foot pattern analysis
- Real-time alerts during training

---


## 8. Training Optimization

### 8.1 Periodization Models

#### 8.1.1 Linear Periodization

**Structure:**
```
Annual Plan (Macrocycle):
├── Preparation Phase (12-16 weeks)
│   ├── General Preparation (6-8 weeks)
│   │   Focus: Base fitness, volume emphasis
│   │   Load: 60-75% intensity, high volume
│   └── Specific Preparation (6-8 weeks)
│       Focus: Sport-specific skills
│       Load: 75-85% intensity, moderate volume
│
├── Competition Phase (20-30 weeks)
│   ├── Pre-Competition (4-6 weeks)
│   │   Focus: Peak conditioning
│   │   Load: 85-95% intensity, reduced volume
│   └── Main Competition (16-24 weeks)
│       Focus: Performance maintenance
│       Load: Variable, manage fatigue
│
└── Transition Phase (4-6 weeks)
    Focus: Active recovery
    Load: 40-60% intensity, low volume
```

**Weekly Load Progression Example (12-week block):**
```
Week 1-3: Load progression 100 → 120 → 140 AU
Week 4: Recovery 80 AU
Week 5-7: Load progression 140 → 160 → 180 AU
Week 8: Recovery 100 AU
Week 9-11: Load progression 180 → 200 → 220 AU
Week 12: Taper 120 AU
```

#### 8.1.2 Block Periodization

**Consecutive Blocks:**
1. **Accumulation Block (3 weeks)**
   - High volume, moderate intensity
   - Build aerobic capacity
   - Target load: 400-500 AU/week

2. **Intensification Block (2 weeks)**
   - Moderate volume, high intensity
   - Improve lactate threshold
   - Target load: 350-450 AU/week

3. **Realization Block (1-2 weeks)**
   - Low volume, very high intensity
   - Peak for competition
   - Target load: 200-300 AU/week

**Cycle repeats every 6-7 weeks**

#### 8.1.3 Undulating Periodization

**Daily/Weekly Variation:**
```
Week Structure:
Monday: High intensity, low volume (Strength focus)
Tuesday: Low intensity, high volume (Endurance)
Wednesday: Moderate intensity, moderate volume (Skill)
Thursday: High intensity, low volume (Power)
Friday: Recovery session
Saturday: Competition or high-load training
Sunday: Rest

Rationale: Constant variation prevents adaptation plateaus
         and maintains engagement
```

### 8.2 Adaptive Training Plans

#### 8.2.1 AI-Driven Plan Adjustment

**Input Data:**
- Planned workout details
- Actual performance in recent sessions
- Current fatigue score
- Sleep quality (last 3 nights)
- Injury risk assessment
- Upcoming competition schedule
- Long-term goal timeline

**Adjustment Algorithm:**
```python
def adjust_next_workout(athlete_data):
    planned_load = get_planned_load()
    fatigue = calculate_fatigue_score(athlete_data)
    readiness = calculate_readiness(athlete_data)
    injury_risk = assess_injury_risk(athlete_data)
    acwr = calculate_acwr(athlete_data)

    adjustment_factor = 1.0

    # Fatigue adjustment
    if fatigue > 70:
        adjustment_factor *= 0.6  # Reduce by 40%
    elif fatigue > 50:
        adjustment_factor *= 0.8  # Reduce by 20%
    elif fatigue < 30 and readiness > 80:
        adjustment_factor *= 1.1  # Increase by 10%

    # Injury risk adjustment
    if injury_risk > 0.7:
        adjustment_factor *= 0.5  # Reduce by 50%
    elif injury_risk > 0.4:
        adjustment_factor *= 0.7  # Reduce by 30%

    # ACWR adjustment (target 0.8-1.3)
    if acwr > 1.5:
        adjustment_factor *= 0.6  # Significant reduction
    elif acwr < 0.5:
        adjustment_factor *= 1.2  # Increase load

    # Apply floor and ceiling
    adjustment_factor = max(0.5, min(1.3, adjustment_factor))

    adjusted_load = planned_load * adjustment_factor

    return {
        'planned': planned_load,
        'adjusted': adjusted_load,
        'factor': adjustment_factor,
        'reasons': generate_explanation(fatigue, injury_risk, acwr)
    }
```

**Example Output:**
```json
{
  "workout_adjustment": {
    "date": "2025-03-15",
    "planned": {
      "type": "interval_training",
      "duration": 60,
      "intensity": 9,
      "load": 540
    },
    "adjusted": {
      "type": "easy_run",
      "duration": 45,
      "intensity": 5,
      "load": 225
    },
    "adjustment_factor": 0.42,
    "reasons": [
      "Fatigue score 68 (Fatigued) → 40% reduction",
      "Hamstring injury risk 0.52 (High) → 30% reduction",
      "Poor sleep last 2 nights (avg 5.2 hours)",
      "ACWR 1.35 (slightly elevated) → maintain current reduction"
    ],
    "recommendation": "Focus on recovery. Easy aerobic session with emphasis on form. Monitor hamstring closely. Ensure 8+ hours sleep tonight."
  }
}
```

### 8.3 Performance Prediction

#### 8.3.1 Race Time Prediction

**Marathon Time from Training Data:**
```
Input features:
- Recent long run paces and distances
- Weekly mileage (last 12 weeks)
- VO2max estimate
- Lactate threshold pace
- Previous race performances
- Age, gender, experience

Model: Gradient Boosting Regressor
Accuracy: ± 3-5 minutes for marathon

Example:
Athlete profile:
- VO2max: 56 ml/kg/min
- Lactate threshold: 4:20 min/km
- Avg weekly mileage: 80 km
- Longest run: 35 km at 5:10 min/km
- Previous half marathon: 1:35:20

Predicted marathon time: 3:28:45 ± 4 minutes
Confidence: 87%
```

#### 8.3.2 Optimal Taper Strategy

**Taper Duration and Load Reduction:**
```
Based on:
- Event distance/duration
- Athlete training age
- Current fitness level
- Recovery capacity

Recommendations:

5K/10K:
- Taper duration: 7-10 days
- Volume reduction: 40-50%
- Maintain intensity

Half Marathon:
- Taper duration: 10-14 days
- Volume reduction: 50-60%
- Maintain some intensity

Marathon:
- Taper duration: 14-21 days (typically 14-16 days)
- Volume reduction: 60-70%
- Reduced intensity and volume

Ultra-distance:
- Taper duration: 21-28 days
- Volume reduction: 70-80%
- Focus on freshness
```

---


## 12. Protocol Definitions

### 12.1 Binary Protocol (Efficient Streaming)

**Protocol Buffers Schema:**

```protobuf
syntax = "proto3";

package wia.sports.v1;

message PerformanceDataPoint {
  int64 timestamp_ms = 1;  // Unix timestamp in milliseconds

  optional double latitude = 2;
  optional double longitude = 3;
  optional float altitude = 4;

  optional float speed = 5;  // km/h
  optional uint32 heart_rate = 6;  // bpm
  optional float power = 7;  // watts
  optional uint32 cadence = 8;  // steps/min or rpm

  optional float temperature = 9;  // Celsius
  optional uint32 battery = 10;  // 0-100%

  // Biomechanics (optional)
  optional Biomechanics biomechanics = 11;
}

message Biomechanics {
  float ground_contact_time_left = 1;  // ms
  float ground_contact_time_right = 2;  // ms
  float vertical_oscillation = 3;  // cm
  float stride_length = 4;  // meters
}

message SessionStream {
  string session_id = 1;
  repeated PerformanceDataPoint data_points = 2;
}
```

**Advantages:**
- 50-70% smaller than JSON
- Faster serialization/deserialization
- Strongly typed schema
- Forward/backward compatibility

### 12.2 MQTT Topics (IoT Devices)

**Topic Structure:**
```
wia/sports/{athleteId}/{deviceId}/{dataType}

Examples:
wia/sports/ATH-2025-001/GPS-001/location
wia/sports/ATH-2025-001/HRM-001/heart_rate
wia/sports/ATH-2025-001/BALL-042/kick_event
```

**QoS Levels:**
- QoS 0: Fire and forget (environmental data)
- QoS 1: At least once (performance metrics)
- QoS 2: Exactly once (injury alerts, critical events)

**Retained Messages:**
- Last known athlete status (online/offline)
- Latest device battery levels

### 12.3 Data Compression

**Algorithms by Use Case:**

| Use Case | Algorithm | Compression Ratio | CPU Cost |
|----------|-----------|-------------------|----------|
| Real-time streaming | LZ4 | 1.5-2x | Very low |
| Session upload | Zstandard | 2.5-4x | Low |
| Long-term archive | LZMA | 4-7x | High |
| Video | H.265/HEVC | 50-100x | High |

---


## 13. Interoperability

### 13.1 Integration with WIA Ecosystem

**WIA-MED-001 (Medical Records)**
```
Data Exchange:
- Injury history → Injury risk models
- Recovery status → Training plan adjustments
- Medical clearance → Return-to-play protocols

API Endpoint:
POST /wia-integration/medical
{
  "athleteId": "ATH-2025-001",
  "medicalData": { ... },
  "consentHash": "0x..."
}
```

**WIA-AI-008 (Machine Learning)**
```
Model Deployment:
- WIA Sports Tech provides training data
- WIA AI platform trains custom models
- Models deployed back to Sports Tech for inference

Model Registry:
- Model ID: ML-HAMSTRING-INJ-v2.3
- Accuracy: 87% AUC-ROC
- Approved for use: 2025-03-01
```

**WIA-SEC-003 (Security)**
```
Encryption Standards:
- All data encrypted per WIA-SEC-003 requirements
- Key rotation policies aligned
- Audit logs in WIA-SEC-003 format
```

### 13.2 Third-Party Integrations

**Supported Formats:**
- FIT (Garmin/Wahoo)
- TCX (Training Center XML)
- GPX (GPS Exchange)
- Strava API
- TrainingPeaks API
- Apple HealthKit
- Google Fit

**Import/Export:**
```http
POST /athletes/{athleteId}/import
Content-Type: multipart/form-data

file: session.fit

Response: 200 OK
{
  "sessionId": "SESSION-2025-03-14-042",
  "imported": {
    "distance": true,
    "heart_rate": true,
    "power": false,
    "cadence": true
  },
  "warnings": ["Power data not found in file"]
}
```

```http
GET /sessions/{sessionId}/export?format=tcx
Authorization: Bearer {token}

Response: 200 OK
Content-Type: application/vnd.garmin.tcx+xml

<?xml version="1.0"?>
<TrainingCenterDatabase>
  <Activities>
    <Activity Sport="Running">
      ...
    </Activity>
  </Activities>
</TrainingCenterDatabase>
```

---


