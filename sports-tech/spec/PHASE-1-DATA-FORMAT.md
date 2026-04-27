# PHASE 1 — Data Format

> Sports-tech canonical data models: athlete, session, event,
> sensor reading, and the architecture map that hosts the
> performance pipeline. The `WIA-IND-011` data models occupy
> the bulk of this phase because the breadth of sport, equipment,
> and physiological sensors demands a precise common schema
> before any API or protocol can be agreed.

## 1. Introduction

### 1.1 Purpose

The WIA-IND-011 standard establishes a comprehensive framework for sports technology systems, enabling:

- Unified data collection from diverse sensor platforms
- Real-time performance analysis and feedback
- AI-powered injury risk assessment and prevention
- Personalized training optimization
- Advanced broadcasting with AR/VR integration
- Cross-platform data interoperability
- Athlete-centric privacy and consent management

### 1.2 Philosophy: 弘益人間 (Benefit All Humanity)

This standard embodies the principle of "broadly benefiting humanity" by:

1. **Democratizing Access**: Making elite sports technology available to all skill levels
2. **Protecting Athletes**: Prioritizing safety and injury prevention over performance gains
3. **Empowering Individuals**: Giving athletes ownership and control of their data
4. **Advancing Science**: Creating open data formats that accelerate sports science research
5. **Promoting Fair Play**: Ensuring technology enhances rather than distorts competition
6. **Building Community**: Enabling coaches, trainers, and athletes to collaborate effectively
7. **Sustainable Innovation**: Encouraging eco-friendly equipment and event management

### 1.3 Target Audience

- Professional and amateur athletes
- Sports teams and organizations
- Coaches and training staff
- Sports medicine professionals
- Equipment manufacturers
- Broadcasting companies
- Sports science researchers
- Mobile app developers

### 1.4 Terminology

| Term | Definition |
|------|------------|
| **Athlete Profile** | Comprehensive record of biometric, performance, and injury data |
| **Session** | Discrete training or competition event with defined start/end times |
| **Metric** | Quantifiable measurement of performance or physiological state |
| **Smart Equipment** | Sports gear with embedded sensors and connectivity |
| **Training Load** | Quantified measure of physical and mental stress from training |
| **HRV** | Heart Rate Variability - variation in time between heartbeats |
| **VO2 Max** | Maximum oxygen uptake during intense exercise |
| **Biomechanics** | Study of mechanical laws relating to movement or structure |
| **AR Overlay** | Augmented reality graphics superimposed on live video |
| **Blockchain Consent** | Immutable record of data sharing permissions |

---


## 2. Scope and Objectives

### 2.1 Sports Categories Covered

This standard applies to (but is not limited to):

**Team Sports**
- Soccer/Football, Basketball, American Football, Rugby
- Hockey (Ice, Field), Volleyball, Handball
- Baseball, Cricket, Lacrosse

**Individual Sports**
- Running (Track, Marathon, Trail), Swimming, Cycling
- Tennis, Badminton, Table Tennis, Squash
- Golf, Boxing, MMA, Wrestling
- Skiing, Snowboarding, Surfing, Skateboarding

**Endurance Sports**
- Triathlon, Iron Man, Ultra-Running
- Road Cycling, Mountain Biking
- Open Water Swimming, Rowing

**Precision Sports**
- Archery, Shooting, Darts
- Gymnastics, Figure Skating, Diving

### 2.2 Technology Domains

The standard encompasses:

1. **Wearable Sensors**: GPS trackers, heart rate monitors, IMU units, smart clothing
2. **Connected Equipment**: Smart balls, rackets, shoes, bikes, goggles
3. **Fixed Infrastructure**: Court/field sensors, camera systems, timing gates
4. **Mobile Applications**: Athlete-facing apps, coaching tools, fan engagement
5. **Cloud Platforms**: Data aggregation, AI analysis, team collaboration
6. **Broadcasting Systems**: Multi-camera capture, AR/VR, live statistics

### 2.3 Objectives

**Primary Objectives**
- Define standardized data formats for all performance metrics
- Establish protocols for real-time sensor data transmission
- Create interoperable APIs for third-party integration
- Specify privacy and security requirements
- Enable cross-platform athlete data portability

**Secondary Objectives**
- Promote innovation in sports technology
- Reduce fragmentation in the sports tech ecosystem
- Lower barriers to entry for new technology providers
- Support evidence-based coaching and training
- Enhance spectator experiences through technology

---


## 3. Architecture Overview

### 3.1 System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                     ATHLETE & EQUIPMENT LAYER                    │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐       │
│  │ Wearables│  │  Smart   │  │  Mobile  │  │  Fixed   │       │
│  │  Sensors │  │Equipment │  │  Devices │  │ Sensors  │       │
│  └─────┬────┘  └─────┬────┘  └─────┬────┘  └─────┬────┘       │
└────────┼─────────────┼─────────────┼─────────────┼─────────────┘
         │             │             │             │
         └─────────────┴─────────────┴─────────────┘
                       │
         ┌─────────────▼──────────────┐
         │   DATA COLLECTION LAYER     │
         │  • BLE/ANT+ Hub             │
         │  • WiFi Gateway             │
         │  • Edge Processing          │
         │  • Data Validation          │
         └─────────────┬───────────────┘
                       │
         ┌─────────────▼──────────────┐
         │  TRANSPORT & PROTOCOL LAYER │
         │  • MQTT / WebSocket         │
         │  • REST API                 │
         │  • gRPC Streaming           │
         │  • Data Compression         │
         └─────────────┬───────────────┘
                       │
         ┌─────────────▼──────────────┐
         │    PROCESSING & AI LAYER    │
         │  • Real-time Analytics      │
         │  • ML Inference             │
         │  • Injury Risk Models       │
         │  • Performance Prediction   │
         └─────────────┬───────────────┘
                       │
         ┌─────────────▼──────────────┐
         │   APPLICATION LAYER         │
         │  • Athlete Dashboard        │
         │  • Coaching Tools           │
         │  • Medical Interface        │
         │  • Broadcasting Suite       │
         └─────────────────────────────┘
```

### 3.2 Data Flow

**Real-time Session Data Flow**

1. **Capture**: Sensors collect data at specified frequencies (1-1000 Hz)
2. **Edge Processing**: Local device performs initial filtering and validation
3. **Transmission**: Data sent via BLE/WiFi to gateway (latency < 100ms)
4. **Cloud Processing**: AI models analyze data streams in real-time
5. **Distribution**: Insights pushed to authorized stakeholders
6. **Storage**: Raw and processed data archived with encryption

**Post-Session Analysis Flow**

1. **Upload**: Complete session data uploaded to cloud platform
2. **Analysis**: Comprehensive AI processing (may take minutes to hours)
3. **Report Generation**: Automated insights and recommendations
4. **Review**: Athlete, coach, medical staff review findings
5. **Action**: Training adjustments, medical interventions, equipment changes

### 3.3 Component Responsibilities

| Component | Responsibilities | Performance Requirements |
|-----------|------------------|--------------------------|
| **Wearable Sensors** | Data collection, battery management | 8+ hours battery, IP67 waterproof |
| **Smart Equipment** | Impact/motion detection, feedback | < 10ms response time |
| **Mobile Apps** | User interface, local processing | < 2s screen load time |
| **Edge Gateway** | Protocol translation, buffering | 99.9% uptime, < 50ms latency |
| **Cloud Platform** | Storage, AI processing, APIs | 99.99% uptime, autoscaling |
| **AI Models** | Real-time inference, predictions | < 100ms inference time |

---


## 4. Data Models

### 4.1 Core Data Schema

#### 4.1.1 Athlete Profile

```json
{
  "athleteProfile": {
    "athleteId": "string (UUID)",
    "version": "string (semver)",
    "created": "ISO 8601 timestamp",
    "updated": "ISO 8601 timestamp",
    "personalInfo": {
      "firstName": "string (optional, encrypted)",
      "lastName": "string (optional, encrypted)",
      "dateOfBirth": "YYYY-MM-DD (optional, encrypted)",
      "gender": "male|female|other|prefer-not-to-say",
      "nationality": "ISO 3166-1 alpha-2 country code"
    },
    "biometrics": {
      "height": "number (cm)",
      "weight": "number (kg)",
      "bodyFatPercentage": "number (0-100)",
      "restingHeartRate": "number (bpm)",
      "maxHeartRate": "number (bpm, optional)",
      "vo2max": "number (ml/kg/min, optional)",
      "lactateThreshold": "number (bpm, optional)",
      "ftp": "number (watts, for cycling, optional)"
    },
    "sportInfo": {
      "primarySport": "string (enum from sport taxonomy)",
      "position": "string (sport-specific)",
      "experience": "beginner|intermediate|advanced|elite|professional",
      "yearsActive": "number",
      "dominantSide": "left|right|ambidextrous"
    },
    "goals": [
      {
        "goalId": "string (UUID)",
        "type": "performance|health|skill",
        "description": "string",
        "targetDate": "ISO 8601 date",
        "targetMetric": "string",
        "targetValue": "number",
        "status": "active|achieved|abandoned"
      }
    ],
    "injuryHistory": [
      {
        "injuryId": "string (UUID)",
        "date": "ISO 8601 date",
        "type": "string (ICD-11 code)",
        "area": "string (body part)",
        "severity": "minor|moderate|severe|critical",
        "recoveryDays": "number",
        "notes": "string (encrypted)"
      }
    ],
    "equipment": [
      {
        "deviceId": "string (UUID)",
        "type": "string (device taxonomy)",
        "manufacturer": "string",
        "model": "string",
        "serialNumber": "string",
        "purchaseDate": "ISO 8601 date",
        "lastCalibration": "ISO 8601 timestamp",
        "status": "active|inactive|maintenance"
      }
    ],
    "privacy": {
      "dataSharing": "private|team-only|public",
      "allowResearch": "boolean",
      "allowCommercial": "boolean",
      "retentionPeriod": "number (years)",
      "blockchainConsent": "string (transaction hash)"
    }
  }
}
```

#### 4.1.2 Session Data

```json
{
  "session": {
    "sessionId": "string (UUID)",
    "athleteId": "string (UUID)",
    "standardId": "WIA-IND-011",
    "version": "1.0.0",
    "timestamp": {
      "start": "ISO 8601 timestamp",
      "end": "ISO 8601 timestamp",
      "duration": "number (seconds)",
      "timezone": "IANA timezone string"
    },
    "sessionType": "training|match|competition|testing|recovery",
    "sport": "string (sport taxonomy)",
    "venue": {
      "venueId": "string (UUID, optional)",
      "name": "string",
      "type": "indoor|outdoor",
      "surface": "string (grass, turf, track, court, etc.)",
      "coordinates": {
        "latitude": "number",
        "longitude": "number",
        "altitude": "number (meters)"
      }
    },
    "environment": {
      "temperature": "number (Celsius)",
      "humidity": "number (0-100)",
      "pressure": "number (hPa)",
      "windSpeed": "number (km/h)",
      "windDirection": "number (degrees, 0-360)",
      "weatherCondition": "sunny|cloudy|rainy|snowy|other",
      "airQuality": "number (AQI, optional)"
    },
    "participants": [
      {
        "athleteId": "string (UUID)",
        "role": "athlete|opponent|teammate|coach|referee",
        "teamId": "string (UUID, optional)"
      }
    ],
    "devices": [
      {
        "deviceId": "string (UUID)",
        "type": "string (device taxonomy)",
        "samplingRate": "number (Hz)",
        "batteryStart": "number (0-100)",
        "batteryEnd": "number (0-100)",
        "firmwareVersion": "string (semver)",
        "dataQuality": "number (0-1, quality score)"
      }
    ],
    "metadata": {
      "importance": "low|medium|high|critical",
      "outcome": "win|loss|draw|dnf|na",
      "score": "string (sport-specific format)",
      "notes": "string (encrypted)",
      "tags": ["string"]
    }
  }
}
```

#### 4.1.3 Performance Metrics

```json
{
  "performanceMetrics": {
    "sessionId": "string (UUID)",
    "timestamp": "ISO 8601 timestamp",
    "distance": {
      "total": "number (meters)",
      "byIntensity": {
        "walking": "number (meters)",
        "jogging": "number (meters)",
        "running": "number (meters)",
        "sprinting": "number (meters)"
      },
      "byZone": {
        "zone1": "number (meters, < 50% max HR)",
        "zone2": "number (meters, 50-60% max HR)",
        "zone3": "number (meters, 60-70% max HR)",
        "zone4": "number (meters, 70-80% max HR)",
        "zone5": "number (meters, 80-90% max HR)",
        "zone6": "number (meters, > 90% max HR)"
      }
    },
    "speed": {
      "average": "number (km/h)",
      "max": "number (km/h)",
      "median": "number (km/h)",
      "p95": "number (km/h, 95th percentile)",
      "timeSeries": [
        {
          "timestamp": "ISO 8601 timestamp",
          "value": "number (km/h)"
        }
      ]
    },
    "heartRate": {
      "average": "number (bpm)",
      "max": "number (bpm)",
      "min": "number (bpm)",
      "resting": "number (bpm)",
      "recovery": "number (bpm, HR 1min post-session)",
      "hrv": {
        "rmssd": "number (ms)",
        "sdnn": "number (ms)",
        "pnn50": "number (0-100)"
      },
      "timeInZones": {
        "zone1": "number (seconds)",
        "zone2": "number (seconds)",
        "zone3": "number (seconds)",
        "zone4": "number (seconds)",
        "zone5": "number (seconds)"
      },
      "timeSeries": [
        {
          "timestamp": "ISO 8601 timestamp",
          "value": "number (bpm)",
          "rrInterval": "number (ms, optional)"
        }
      ]
    },
    "power": {
      "average": "number (watts)",
      "max": "number (watts)",
      "normalized": "number (watts)",
      "intensityFactor": "number (0-2)",
      "trainingStressScore": "number (TSS)",
      "timeSeries": [
        {
          "timestamp": "ISO 8601 timestamp",
          "value": "number (watts)"
        }
      ]
    },
    "biomechanics": {
      "cadence": {
        "average": "number (steps/min or rpm)",
        "max": "number"
      },
      "strideLength": {
        "average": "number (meters)",
        "leftRight": {
          "left": "number (meters)",
          "right": "number (meters)"
        }
      },
      "groundContactTime": {
        "average": "number (ms)",
        "leftRight": {
          "left": "number (ms)",
          "right": "number (ms)"
        }
      },
      "verticalOscillation": {
        "average": "number (cm)",
        "ratio": "number (cm/meter)"
      },
      "balance": {
        "leftRight": {
          "left": "number (percent)",
          "right": "number (percent)"
        },
        "asymmetryIndex": "number (0-1)"
      },
      "jointAngles": {
        "hip": {
          "flexion": "number (degrees)",
          "extension": "number (degrees)",
          "abduction": "number (degrees)"
        },
        "knee": {
          "flexion": "number (degrees)",
          "extension": "number (degrees)"
        },
        "ankle": {
          "dorsiFlex": "number (degrees)",
          "plantarFlex": "number (degrees)"
        }
      }
    },
    "metabolic": {
      "caloriesBurned": "number (kcal)",
      "fatBurned": "number (grams)",
      "carbsBurned": "number (grams)",
      "respiratoryRate": "number (breaths/min)",
      "coreTemperature": "number (Celsius, optional)",
      "hydrationLoss": "number (ml, estimated)",
      "sweatRate": "number (L/hour, optional)"
    },
    "neuromuscular": {
      "reactionTime": "number (ms)",
      "explosiveness": "number (0-100 score)",
      "fatigue": {
        "muscular": "number (0-100 score)",
        "central": "number (0-100 score)",
        "overall": "number (0-100 score)"
      },
      "readiness": "number (0-100 score)"
    }
  }
}
```

#### 4.1.4 Skill Metrics (Sport-Specific)

```json
{
  "skillMetrics": {
    "sessionId": "string (UUID)",
    "sport": "string (sport taxonomy)",
    "soccer": {
      "ballTouches": "number",
      "passes": {
        "total": "number",
        "successful": "number",
        "accuracy": "number (percent)",
        "avgDistance": "number (meters)",
        "byType": {
          "short": "number",
          "medium": "number",
          "long": "number"
        }
      },
      "shots": {
        "total": "number",
        "onTarget": "number",
        "goals": "number",
        "avgSpeed": "number (km/h)",
        "avgSpin": "number (rpm)",
        "positions": [
          {
            "x": "number (field coordinates)",
            "y": "number (field coordinates)",
            "result": "goal|on-target|off-target|blocked"
          }
        ]
      },
      "dribbles": {
        "attempted": "number",
        "successful": "number",
        "successRate": "number (percent)"
      },
      "tackles": {
        "attempted": "number",
        "successful": "number",
        "fouls": "number"
      },
      "aerialDuels": {
        "attempted": "number",
        "won": "number",
        "winRate": "number (percent)"
      }
    },
    "basketball": {
      "shots": {
        "freeThrows": {
          "attempted": "number",
          "made": "number",
          "percentage": "number"
        },
        "twoPoint": {
          "attempted": "number",
          "made": "number",
          "percentage": "number"
        },
        "threePoint": {
          "attempted": "number",
          "made": "number",
          "percentage": "number"
        }
      },
      "rebounds": {
        "offensive": "number",
        "defensive": "number",
        "total": "number"
      },
      "assists": "number",
      "steals": "number",
      "blocks": "number",
      "turnovers": "number"
    },
    "running": {
      "splits": [
        {
          "distance": "number (meters)",
          "time": "number (seconds)",
          "pace": "number (min/km)"
        }
      ],
      "elevation": {
        "gain": "number (meters)",
        "loss": "number (meters)",
        "maxGrade": "number (percent)"
      }
    },
    "cycling": {
      "pedalingMetrics": {
        "pedalSmoothness": "number (percent)",
        "torqueEffectiveness": "number (percent)",
        "leftRightBalance": {
          "left": "number (percent)",
          "right": "number (percent)"
        }
      },
      "climbs": [
        {
          "distance": "number (meters)",
          "elevation": "number (meters)",
          "gradient": "number (percent)",
          "category": "4|3|2|1|HC"
        }
      ]
    },
    "tennis": {
      "serves": {
        "first": {
          "attempted": "number",
          "in": "number",
          "percentage": "number",
          "avgSpeed": "number (km/h)",
          "maxSpeed": "number (km/h)"
        },
        "second": {
          "attempted": "number",
          "in": "number",
          "percentage": "number",
          "avgSpeed": "number (km/h)"
        },
        "aces": "number",
        "doubleFaults": "number"
      },
      "groundStrokes": {
        "forehand": {
          "total": "number",
          "winners": "number",
          "errors": "number",
          "avgSpeed": "number (km/h)",
          "avgSpin": "number (rpm)"
        },
        "backhand": {
          "total": "number",
          "winners": "number",
          "errors": "number",
          "avgSpeed": "number (km/h)",
          "avgSpin": "number (rpm)"
        }
      },
      "movement": {
        "courtCoverage": "number (m²)",
        "shotsWhileMoving": "number",
        "shotsStatic": "number"
      }
    }
  }
}
```

#### 4.1.5 Injury Risk Assessment

```json
{
  "injuryRisk": {
    "sessionId": "string (UUID)",
    "athleteId": "string (UUID)",
    "timestamp": "ISO 8601 timestamp",
    "overallRisk": "number (0-1 probability)",
    "riskLevel": "low|medium|high|critical",
    "areaSpecificRisk": [
      {
        "bodyPart": "string (hamstring, acl, ankle, shoulder, etc.)",
        "riskScore": "number (0-1)",
        "contributingFactors": [
          {
            "factor": "string (fatigue, asymmetry, load, movement, etc.)",
            "severity": "number (0-1)",
            "description": "string"
          }
        ]
      }
    ],
    "fatigueMetrics": {
      "acute": "number (0-1, current fatigue)",
      "chronic": "number (0-1, ongoing fatigue)",
      "acuteChronicRatio": "number (training load ratio)",
      "monotony": "number (training monotony index)",
      "strain": "number (training strain index)"
    },
    "movementQuality": {
      "overallScore": "number (0-100)",
      "asymmetryDetected": "boolean",
      "asymmetryMagnitude": "number (percent difference)",
      "compensatoryPatterns": [
        {
          "pattern": "string",
          "severity": "mild|moderate|severe"
        }
      ]
    },
    "alerts": [
      {
        "alertId": "string (UUID)",
        "timestamp": "ISO 8601 timestamp",
        "severity": "info|warning|critical",
        "type": "fatigue|asymmetry|overload|movement|equipment",
        "message": "string",
        "recommendations": ["string"]
      }
    ],
    "historicalContext": {
      "previousInjuries": "number (count in same area)",
      "daysSinceLastInjury": "number",
      "currentRecoveryStatus": "string"
    },
    "recommendations": {
      "immediate": ["string (actions to take now)"],
      "shortTerm": ["string (actions in next 48 hours)"],
      "longTerm": ["string (training adjustments)"]
    },
    "modelMetadata": {
      "modelVersion": "string (semver)",
      "modelType": "deep-learning|random-forest|ensemble",
      "confidence": "number (0-1)",
      "trainingDataSize": "number (samples)",
      "lastUpdated": "ISO 8601 timestamp"
    }
  }
}
```

#### 4.1.6 Training Plan

```json
{
  "trainingPlan": {
    "planId": "string (UUID)",
    "athleteId": "string (UUID)",
    "created": "ISO 8601 timestamp",
    "validFrom": "ISO 8601 date",
    "validTo": "ISO 8601 date",
    "goal": {
      "type": "performance|health|skill|competition",
      "description": "string",
      "targetEvent": {
        "name": "string",
        "date": "ISO 8601 date",
        "importance": "low|medium|high|critical"
      }
    },
    "seasonPhase": "off-season|pre-season|early-season|mid-season|late-season|playoffs|recovery",
    "mesocycles": [
      {
        "mesocycleId": "string (UUID)",
        "name": "string",
        "startDate": "ISO 8601 date",
        "endDate": "ISO 8601 date",
        "focus": "endurance|strength|speed|skill|taper|recovery",
        "targetLoad": "number (arbitrary units per week)",
        "microcycles": [
          {
            "microcycleId": "string (UUID)",
            "weekNumber": "number",
            "startDate": "ISO 8601 date",
            "plannedLoad": "number (arbitrary units)",
            "sessions": [
              {
                "sessionId": "string (UUID)",
                "dayOfWeek": "monday|tuesday|wednesday|thursday|friday|saturday|sunday",
                "sessionType": "training|match|recovery|rest|testing",
                "sport": "string",
                "duration": "number (minutes)",
                "intensity": "number (1-10 RPE scale)",
                "focus": "string (endurance, intervals, strength, etc.)",
                "exercises": [
                  {
                    "exerciseId": "string (UUID)",
                    "name": "string",
                    "sets": "number",
                    "reps": "number",
                    "duration": "number (seconds)",
                    "intensity": "number (1-10 RPE)",
                    "rest": "number (seconds)",
                    "notes": "string"
                  }
                ],
                "targetMetrics": {
                  "distance": "number (meters, optional)",
                  "heartRateZone": "number (1-5, optional)",
                  "power": "number (watts, optional)",
                  "pace": "number (min/km, optional)"
                },
                "notes": "string"
              }
            ]
          }
        ]
      }
    ],
    "progressTracking": {
      "completedSessions": "number",
      "totalPlannedSessions": "number",
      "adherenceRate": "number (percent)",
      "adjustments": [
        {
          "date": "ISO 8601 date",
          "reason": "string (injury, illness, fatigue, etc.)",
          "changes": "string"
        }
      ]
    },
    "coachNotes": "string (encrypted)"
  }
}
```

---


