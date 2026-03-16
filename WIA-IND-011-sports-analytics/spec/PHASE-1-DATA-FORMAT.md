# WIA-IND-011 PHASE 1: DATA FORMAT SPECIFICATION
## Sports Analytics Standard - Data Schema and Formats

**Standard:** WIA-IND-011
**Phase:** 1 of 4
**Version:** 1.0
**Status:** Active
**Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

Phase 1 of the WIA-IND-011 Sports Analytics Standard defines standardized data formats, schemas, and structures for sports performance data. This ensures interoperability, consistency, and reliability across different systems, platforms, and organizations.

**Guiding Principle (弘益人間):** Data formats should serve all stakeholders - athletes, coaches, analysts, broadcasters, and fans - by enabling seamless exchange and interpretation of sports analytics information.

---

## 2. Core Data Entities

### 2.1 Player Data Schema

```json
{
  "standard": "WIA-IND-011",
  "version": "1.0",
  "philosophy": "弘益人間",
  "entity": "player",
  "schema": {
    "playerId": "string (required, unique identifier)",
    "personalInfo": {
      "firstName": "string",
      "lastName": "string",
      "dateOfBirth": "ISO8601 date",
      "nationality": "ISO 3166-1 alpha-2",
      "height": "number (cm)",
      "weight": "number (kg)"
    },
    "sportInfo": {
      "sport": "string",
      "position": "string",
      "teamId": "string",
      "jerseyNumber": "number",
      "dominantSide": "enum [left, right, both]"
    },
    "metadata": {
      "createdAt": "ISO8601 timestamp",
      "updatedAt": "ISO8601 timestamp",
      "dataSource": "string"
    }
  }
}
```

### 2.2 Performance Metrics Schema

```json
{
  "standard": "WIA-IND-011",
  "version": "1.0",
  "philosophy": "弘益人間",
  "entity": "performance",
  "schema": {
    "performanceId": "string (required, unique)",
    "playerId": "string (required)",
    "matchId": "string (optional, if match-related)",
    "timestamp": "ISO8601",
    "physicalMetrics": {
      "totalDistance": "number (meters)",
      "highSpeedDistance": "number (meters, >19.8 km/h)",
      "sprintDistance": "number (meters, >25.2 km/h)",
      "topSpeed": "number (km/h)",
      "accelerations": "number (count)",
      "decelerations": "number (count)",
      "playerLoad": "number (arbitrary units)"
    },
    "physiologicalMetrics": {
      "avgHeartRate": "number (bpm)",
      "maxHeartRate": "number (bpm)",
      "hrZones": {
        "zone1": "number (seconds)",
        "zone2": "number (seconds)",
        "zone3": "number (seconds)",
        "zone4": "number (seconds)",
        "zone5": "number (seconds)"
      },
      "hrv": "number (ms, optional)",
      "caloriesBurned": "number (kcal, optional)"
    },
    "technicalMetrics": {
      "passes": "number",
      "passesCompleted": "number",
      "passAccuracy": "number (0-1)",
      "shots": "number",
      "shotsOnTarget": "number",
      "tackles": "number",
      "interceptions": "number",
      "dribbles": "number"
    }
  }
}
```

### 2.3 Game Event Schema

```json
{
  "standard": "WIA-IND-011",
  "version": "1.0",
  "philosophy": "弘益人間 - Every event tells a story",
  "entity": "gameEvent",
  "schema": {
    "eventId": "string (required, unique)",
    "matchId": "string (required)",
    "timestamp": "ISO8601",
    "matchClock": "number (seconds from kickoff)",
    "eventType": "enum [pass, shot, tackle, foul, corner, etc.]",
    "playerId": "string (primary actor)",
    "teamId": "string",
    "location": {
      "x": "number (0-100, pitch percentage)",
      "y": "number (0-100, pitch percentage)",
      "z": "number (optional, height in meters)"
    },
    "outcome": "enum [successful, unsuccessful, partial]",
    "relatedPlayers": [
      {
        "playerId": "string",
        "role": "enum [receiver, defender, goalkeeper]"
      }
    ],
    "contextualData": {
      "matchState": {
        "homeScore": "number",
        "awayScore": "number",
        "minute": "number"
      },
      "bodyPart": "enum [foot, head, chest, other]",
      "assist": "boolean",
      "keyPass": "boolean"
    },
    "metadata": {
      "dataQuality": "number (0-1, confidence score)",
      "dataSource": "string",
      "validator": "string (optional)"
    }
  }
}
```

### 2.4 Match Data Schema

```json
{
  "standard": "WIA-IND-011",
  "version": "1.0",
  "philosophy": "弘益人間",
  "entity": "match",
  "schema": {
    "matchId": "string (required, unique)",
    "competition": {
      "competitionId": "string",
      "season": "string (e.g., '2024-25')",
      "round": "string"
    },
    "teams": {
      "home": {
        "teamId": "string",
        "teamName": "string",
        "score": "number",
        "xG": "number (optional)"
      },
      "away": {
        "teamId": "string",
        "teamName": "string",
        "score": "number",
        "xG": "number (optional)"
      }
    },
    "matchInfo": {
      "date": "ISO8601 date",
      "kickoffTime": "ISO8601 timestamp",
      "venue": "string",
      "attendance": "number (optional)",
      "referee": "string (optional)"
    },
    "weather": {
      "temperature": "number (celsius)",
      "humidity": "number (percentage)",
      "windSpeed": "number (km/h)",
      "conditions": "string"
    },
    "status": "enum [scheduled, live, completed, postponed, cancelled]"
  }
}
```

---

## 3. Tracking Data Formats

### 3.1 Positional Tracking Data

```json
{
  "standard": "WIA-IND-011",
  "version": "1.0",
  "philosophy": "弘益人間",
  "dataType": "tracking",
  "format": "json",
  "samplingRate": "10 Hz (required minimum)",
  "accuracy": "±15 cm (required)",
  "frames": [
    {
      "timestamp": "ISO8601 with milliseconds",
      "frameNumber": "number",
      "players": [
        {
          "playerId": "string",
          "teamId": "string",
          "position": {
            "x": "number (meters from reference point)",
            "y": "number (meters from reference point)"
          },
          "velocity": {
            "magnitude": "number (m/s)",
            "direction": "number (degrees, 0-360)"
          },
          "acceleration": "number (m/s²)"
        }
      ],
      "ball": {
        "position": {
          "x": "number",
          "y": "number",
          "z": "number"
        },
        "velocity": {
          "magnitude": "number",
          "direction": "number"
        },
        "possession": "string (playerId or null)"
      }
    }
  ]
}
```

### 3.2 Wearable Sensor Data

```json
{
  "standard": "WIA-IND-011",
  "version": "1.0",
  "philosophy": "弘益人間 - Protecting athlete health through data",
  "dataType": "wearable",
  "deviceInfo": {
    "manufacturer": "string",
    "model": "string",
    "firmwareVersion": "string",
    "calibrationDate": "ISO8601 date"
  },
  "playerId": "string (required)",
  "sessionId": "string (required)",
  "samplingRates": {
    "gps": "10 Hz",
    "heartRate": "1 Hz",
    "accelerometer": "100 Hz"
  },
  "data": [
    {
      "timestamp": "ISO8601 with milliseconds",
      "gps": {
        "latitude": "number",
        "longitude": "number",
        "altitude": "number",
        "speed": "number (m/s)",
        "accuracy": "number (meters)"
      },
      "heartRate": "number (bpm)",
      "accelerometer": {
        "x": "number (g-force)",
        "y": "number (g-force)",
        "z": "number (g-force)"
      },
      "gyroscope": {
        "x": "number (degrees/second)",
        "y": "number (degrees/second)",
        "z": "number (degrees/second)"
      }
    }
  ]
}
```

---

## 4. Statistical Aggregations

### 4.1 Match Statistics Summary

```json
{
  "standard": "WIA-IND-011",
  "version": "1.0",
  "philosophy": "弘益人間",
  "matchId": "string",
  "teamStats": {
    "teamId": "string",
    "possession": "number (percentage)",
    "passes": {
      "total": "number",
      "completed": "number",
      "accuracy": "number (percentage)"
    },
    "shots": {
      "total": "number",
      "onTarget": "number",
      "xG": "number"
    },
    "distance": {
      "total": "number (km)",
      "highSpeed": "number (km)",
      "sprint": "number (km)"
    },
    "defensive": {
      "tackles": "number",
      "interceptions": "number",
      "clearances": "number"
    }
  }
}
```

---

## 5. Data Quality Standards

### 5.1 Required Accuracy Levels

| Data Type | Minimum Accuracy | Sampling Rate |
|-----------|-----------------|---------------|
| GPS Position | ±15 cm | 10 Hz |
| Heart Rate | ±2 bpm | 1 Hz |
| Event Timestamp | ±100 ms | N/A |
| Speed/Velocity | ±0.1 m/s | 10 Hz |
| Video Frames | 25 fps minimum | 25 Hz |

### 5.2 Data Validation Rules

1. **Timestamp Validity:** All timestamps must be ISO8601 format with UTC timezone
2. **Range Checks:** Physical metrics must fall within biologically plausible ranges
3. **Referential Integrity:** Foreign keys (playerId, matchId, etc.) must reference valid entities
4. **Completeness:** Required fields must not be null or empty
5. **Consistency:** Aggregate values must match sum of components

### 5.3 Missing Data Handling

```json
{
  "missingDataPolicy": {
    "philosophy": "弘益人間 - Transparency in data quality",
    "notation": {
      "missing": "null",
      "notApplicable": "N/A",
      "notYetAvailable": "pending"
    },
    "imputation": {
      "method": "must be documented in metadata",
      "allowedMethods": ["mean", "median", "forward-fill", "model-based"],
      "required": "imputation flag in metadata"
    }
  }
}
```

---

## 6. File Formats and Encoding

### 6.1 Supported Formats

- **Primary:** JSON (UTF-8 encoding required)
- **Alternative:** CSV for tabular data
- **Binary:** Protocol Buffers for high-frequency tracking data
- **Export:** XML for legacy system compatibility

### 6.2 File Naming Convention

```
WIA-IND-011_{entity}_{identifier}_{timestamp}.{format}

Examples:
WIA-IND-011_player_P123456_20250115T120000Z.json
WIA-IND-011_match_M789012_20250115T150000Z.json
WIA-IND-011_tracking_M789012_20250115T150000Z.pb
```

---

## 7. Versioning and Compatibility

### 7.1 Schema Version

All data files must include:
```json
{
  "standard": "WIA-IND-011",
  "version": "1.0",
  "schemaVersion": "1.0.0",
  "philosophy": "弘益人間"
}
```

### 7.2 Backward Compatibility

- Major version changes (e.g., 1.0 → 2.0) may break compatibility
- Minor version changes (e.g., 1.0 → 1.1) must maintain backward compatibility
- Patch version changes (e.g., 1.0.0 → 1.0.1) must be fully compatible

---

## 8. Integration Examples

### 8.1 Python Example

```python
import json
from datetime import datetime

def create_player_performance(player_id, metrics):
    """
    Create WIA-IND-011 compliant performance record
    弘益人間 - Structured data benefits everyone
    """
    return {
        "standard": "WIA-IND-011",
        "version": "1.0",
        "philosophy": "弘益人間",
        "entity": "performance",
        "performanceId": f"PERF_{datetime.utcnow().timestamp()}",
        "playerId": player_id,
        "timestamp": datetime.utcnow().isoformat() + "Z",
        "physicalMetrics": metrics
    }

# Usage
performance = create_player_performance(
    "P123456",
    {
        "totalDistance": 11250.5,
        "topSpeed": 32.1,
        "accelerations": 145
    }
)

print(json.dumps(performance, indent=2))
```

### 8.2 TypeScript Example

```typescript
interface WIAPerformanceData {
  standard: "WIA-IND-011";
  version: "1.0";
  philosophy: "弘益人間";
  entity: "performance";
  performanceId: string;
  playerId: string;
  timestamp: string;
  physicalMetrics: {
    totalDistance?: number;
    topSpeed?: number;
    accelerations?: number;
  };
}

function validateWIAData(data: any): data is WIAPerformanceData {
  return (
    data.standard === "WIA-IND-011" &&
    data.philosophy === "弘益人間" &&
    typeof data.playerId === "string"
  );
}
```

---

## 9. Compliance Checklist

- [ ] All required fields present
- [ ] ISO8601 timestamps with UTC timezone
- [ ] Unique identifiers for all entities
- [ ] Philosophy field includes "弘益人間"
- [ ] Standard field set to "WIA-IND-011"
- [ ] Version field specified
- [ ] Data quality metadata included
- [ ] Accuracy requirements met
- [ ] Proper encoding (UTF-8 for JSON)
- [ ] File naming convention followed

---

## 10. Reference Implementation

Complete reference implementations available at:
- GitHub: https://github.com/WIA-Official/wia-standards
- Documentation: https://wia.org/IND-011/phase-1

---

**Document Version:** 1.0
**Last Updated:** 2025-01-15
**Maintained by:** WIA Technical Committee
**Philosophy:** 弘益人間 - Benefit All Humanity

© 2025 SmileStory Inc. / WIA
Licensed under MIT License
