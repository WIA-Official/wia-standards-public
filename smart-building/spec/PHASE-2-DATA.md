# WIA-CITY-003: Smart Building Standard
## PHASE 2 - DATA FORMAT SPECIFICATION

**Version:** 1.0
**Status:** Active
**Category:** CITY
**Last Updated:** 2025-12-25

---

## 1. Data Model Overview

### 1.1 Core Entities

The smart building data model consists of the following core entities:

```
Building
├── Floors
│   ├── Zones
│   │   ├── Spaces
│   │   │   ├── Sensors
│   │   │   ├── Actuators
│   │   │   └── Devices
│   │   └── Equipment
│   └── Systems
└── Metadata
```

### 1.2 Data Categories

1. **Static Data**: Building structure, equipment specifications
2. **Configuration Data**: Setpoints, schedules, control logic
3. **Time-Series Data**: Sensor readings, energy consumption
4. **Event Data**: Alarms, state changes, user actions
5. **Analytics Data**: KPIs, predictions, recommendations

---

## 2. Building Data Schema

### 2.1 Building Entity

```json
{
  "id": "string (UUID)",
  "name": "string",
  "address": {
    "street": "string",
    "city": "string",
    "state": "string",
    "country": "string",
    "postalCode": "string",
    "coordinates": {
      "latitude": "number",
      "longitude": "number"
    }
  },
  "metadata": {
    "constructionYear": "integer",
    "totalFloors": "integer",
    "totalArea": "number (m²)",
    "occupancyType": "string (office|residential|retail|mixed)",
    "certifications": [
      {
        "type": "string (LEED|WELL|BREEAM)",
        "level": "string",
        "issueDate": "ISO 8601 date",
        "expiryDate": "ISO 8601 date"
      }
    ]
  },
  "timezone": "string (IANA timezone)",
  "createdAt": "ISO 8601 timestamp",
  "updatedAt": "ISO 8601 timestamp"
}
```

### 2.2 Floor Entity

```json
{
  "id": "string (UUID)",
  "buildingId": "string (UUID)",
  "floorNumber": "integer",
  "name": "string",
  "area": "number (m²)",
  "ceiling Height": "number (meters)",
  "occupancyType": "string",
  "zones": ["string (UUID)"]
}
```

### 2.3 Zone Entity

```json
{
  "id": "string (UUID)",
  "floorId": "string (UUID)",
  "name": "string",
  "type": "string (office|conference|lobby|common|technical)",
  "area": "number (m²)",
  "maxOccupancy": "integer",
  "spaces": ["string (UUID)"],
  "hvacZone": "string",
  "lightingZone": "string"
}
```

---

## 3. Sensor Data Format

### 3.1 Generic Sensor Reading

```json
{
  "sensorId": "string (UUID)",
  "timestamp": "ISO 8601 timestamp",
  "value": "number|boolean|string",
  "unit": "string",
  "quality": "string (good|uncertain|bad)",
  "metadata": {
    "location": {
      "buildingId": "string",
      "floorId": "string",
      "zoneId": "string",
      "coordinates": {
        "x": "number",
        "y": "number",
        "z": "number"
      }
    }
  }
}
```

### 3.2 Environmental Sensor Data

**Temperature Sensor:**
```json
{
  "sensorId": "TEMP-001",
  "sensorType": "temperature",
  "timestamp": "2025-12-25T10:30:00Z",
  "value": 22.5,
  "unit": "celsius",
  "quality": "good",
  "calibrationDate": "2025-01-15",
  "accuracy": "±0.5"
}
```

**Humidity Sensor:**
```json
{
  "sensorId": "HUM-001",
  "sensorType": "humidity",
  "timestamp": "2025-12-25T10:30:00Z",
  "value": 45,
  "unit": "percent",
  "quality": "good",
  "dewPoint": 10.5
}
```

**CO₂ Sensor:**
```json
{
  "sensorId": "CO2-001",
  "sensorType": "co2",
  "timestamp": "2025-12-25T10:30:00Z",
  "value": 650,
  "unit": "ppm",
  "quality": "good",
  "threshold": {
    "warning": 800,
    "critical": 1000
  }
}
```

**Air Quality Sensor:**
```json
{
  "sensorId": "AQ-001",
  "sensorType": "air-quality",
  "timestamp": "2025-12-25T10:30:00Z",
  "measurements": {
    "pm25": {
      "value": 12,
      "unit": "µg/m³"
    },
    "pm10": {
      "value": 20,
      "unit": "µg/m³"
    },
    "voc": {
      "value": 250,
      "unit": "ppb"
    },
    "tvoc": {
      "value": 300,
      "unit": "µg/m³"
    }
  },
  "aqi": 45,
  "quality": "good"
}
```

### 3.3 Occupancy Sensor Data

**PIR (Passive Infrared) Sensor:**
```json
{
  "sensorId": "OCC-001",
  "sensorType": "occupancy-pir",
  "timestamp": "2025-12-25T10:30:00Z",
  "occupied": true,
  "confidence": 0.95,
  "lastMotion": "2025-12-25T10:29:45Z",
  "timeoutPeriod": 300
}
```

**People Counter:**
```json
{
  "sensorId": "COUNT-001",
  "sensorType": "people-counter",
  "timestamp": "2025-12-25T10:30:00Z",
  "count": 12,
  "direction": {
    "in": 15,
    "out": 3
  },
  "maxCapacity": 50,
  "occupancyRate": 0.24
}
```

### 3.4 Light Level Sensor

```json
{
  "sensorId": "LIGHT-001",
  "sensorType": "illuminance",
  "timestamp": "2025-12-25T10:30:00Z",
  "value": 450,
  "unit": "lux",
  "quality": "good",
  "daylightContribution": 0.65,
  "artificialContribution": 0.35
}
```

---

## 4. Energy Data Format

### 4.1 Energy Meter Reading

```json
{
  "meterId": "string (UUID)",
  "meterType": "string (electric|gas|water|thermal)",
  "timestamp": "ISO 8601 timestamp",
  "readings": {
    "energy": {
      "value": "number",
      "unit": "kWh|m³|BTU"
    },
    "power": {
      "value": "number",
      "unit": "kW|m³/h|BTU/h"
    },
    "voltage": {
      "value": "number",
      "unit": "V"
    },
    "current": {
      "value": "number",
      "unit": "A"
    },
    "powerFactor": "number (0-1)"
  },
  "cumulative": {
    "value": "number",
    "resetDate": "ISO 8601 date"
  }
}
```

### 4.2 Energy Consumption Summary

```json
{
  "buildingId": "string (UUID)",
  "period": {
    "start": "ISO 8601 timestamp",
    "end": "ISO 8601 timestamp",
    "type": "string (hourly|daily|monthly|yearly)"
  },
  "consumption": {
    "electricity": {
      "total": "number (kWh)",
      "peak": "number (kW)",
      "average": "number (kW)",
      "cost": "number (USD)"
    },
    "gas": {
      "total": "number (m³)",
      "cost": "number (USD)"
    },
    "water": {
      "total": "number (m³)",
      "cost": "number (USD)"
    }
  },
  "breakdown": {
    "hvac": "number (%)",
    "lighting": "number (%)",
    "plugLoads": "number (%)",
    "elevators": "number (%)",
    "other": "number (%)"
  },
  "kpi": {
    "euiActual": "number (kWh/m²/year)",
    "euiTarget": "number (kWh/m²/year)",
    "variance": "number (%)"
  }
}
```

---

## 5. HVAC Data Format

### 5.1 HVAC System Status

```json
{
  "systemId": "string (UUID)",
  "systemType": "string (AHU|VAV|FCU|Chiller|Boiler)",
  "timestamp": "ISO 8601 timestamp",
  "status": "string (running|stopped|fault|maintenance)",
  "mode": "string (cooling|heating|ventilation|auto)",
  "measurements": {
    "supplyAirTemp": {
      "value": "number",
      "unit": "celsius",
      "setpoint": "number"
    },
    "returnAirTemp": {
      "value": "number",
      "unit": "celsius"
    },
    "supplyAirFlow": {
      "value": "number",
      "unit": "m³/h"
    },
    "fanSpeed": {
      "value": "number",
      "unit": "percent"
    },
    "damperPosition": {
      "value": "number",
      "unit": "percent"
    },
    "valvePosition": {
      "value": "number",
      "unit": "percent"
    }
  },
  "alarms": [
    {
      "code": "string",
      "severity": "string (info|warning|critical)",
      "message": "string",
      "timestamp": "ISO 8601 timestamp"
    }
  ]
}
```

### 5.2 Thermostat Data

```json
{
  "thermostatId": "string (UUID)",
  "zoneId": "string (UUID)",
  "timestamp": "ISO 8601 timestamp",
  "currentTemp": 22.5,
  "setpoint": {
    "heating": 20.0,
    "cooling": 24.0
  },
  "mode": "auto|heat|cool|off",
  "fanMode": "auto|on|circulate",
  "humidity": 45,
  "occupancy": true,
  "schedule": {
    "current": "occupied|unoccupied|standby",
    "nextChange": "ISO 8601 timestamp"
  },
  "override": {
    "active": false,
    "expiryTime": "ISO 8601 timestamp"
  }
}
```

---

## 6. Lighting Data Format

### 6.1 Lighting Fixture Status

```json
{
  "fixtureId": "string (UUID)",
  "zoneId": "string (UUID)",
  "timestamp": "ISO 8601 timestamp",
  "status": "on|off",
  "brightness": {
    "current": "number (0-100)",
    "target": "number (0-100)"
  },
  "colorTemp": {
    "current": "number (Kelvin)",
    "target": "number (Kelvin)"
  },
  "power": {
    "current": "number (Watts)",
    "rated": "number (Watts)"
  },
  "operatingHours": "number",
  "controlMode": "manual|auto|scheduled|daylight"
}
```

### 6.2 Lighting Scene

```json
{
  "sceneId": "string (UUID)",
  "name": "string",
  "zoneId": "string (UUID)",
  "fixtures": [
    {
      "fixtureId": "string (UUID)",
      "brightness": "number (0-100)",
      "colorTemp": "number (Kelvin)"
    }
  ],
  "triggers": [
    {
      "type": "schedule|occupancy|manual|event",
      "condition": "object"
    }
  ]
}
```

---

## 7. Event and Alarm Format

### 7.1 Generic Event

```json
{
  "eventId": "string (UUID)",
  "timestamp": "ISO 8601 timestamp",
  "eventType": "string (alarm|state-change|user-action|system)",
  "severity": "string (info|warning|error|critical)",
  "source": {
    "type": "string (sensor|system|device|user)",
    "id": "string (UUID)",
    "name": "string"
  },
  "description": "string",
  "acknowledged": {
    "status": "boolean",
    "by": "string (userId)",
    "at": "ISO 8601 timestamp"
  },
  "resolved": {
    "status": "boolean",
    "by": "string (userId)",
    "at": "ISO 8601 timestamp",
    "resolution": "string"
  }
}
```

### 7.2 Alarm Example

```json
{
  "eventId": "evt-12345",
  "timestamp": "2025-12-25T10:35:00Z",
  "eventType": "alarm",
  "severity": "critical",
  "source": {
    "type": "system",
    "id": "hvac-chiller-01",
    "name": "Main Chiller"
  },
  "alarmCode": "CHILL-001",
  "description": "Chiller low refrigerant pressure",
  "parameters": {
    "pressure": 35,
    "threshold": 50,
    "unit": "PSI"
  },
  "acknowledged": {
    "status": true,
    "by": "user-maintenance-01",
    "at": "2025-12-25T10:37:00Z"
  },
  "resolved": {
    "status": false
  }
}
```

---

## 8. Predictive Maintenance Data

### 8.1 Equipment Health Score

```json
{
  "equipmentId": "string (UUID)",
  "timestamp": "ISO 8601 timestamp",
  "healthScore": "number (0-100)",
  "confidence": "number (0-1)",
  "factors": [
    {
      "name": "vibration",
      "value": "number",
      "normal Range": "object",
      "contribution": "number (0-1)"
    },
    {
      "name": "temperature",
      "value": "number",
      "normalRange": "object",
      "contribution": "number (0-1)"
    },
    {
      "name": "operatingHours",
      "value": "number",
      "expectedLifetime": "number",
      "contribution": "number (0-1)"
    }
  ],
  "prediction": {
    "nextMaintenance": "ISO 8601 date",
    "estimatedFailure": "ISO 8601 date",
    "recommendedActions": ["string"]
  }
}
```

---

## 9. Data Quality Standards

### 9.1 Quality Indicators

- **Good**: Sensor operating within normal parameters
- **Uncertain**: Sensor reading at edge of normal range
- **Bad**: Sensor failure or reading out of range

### 9.2 Data Validation Rules

1. **Range Validation**: Values must fall within expected ranges
2. **Rate of Change**: Changes must be physically plausible
3. **Consistency**: Related sensors must show consistent readings
4. **Completeness**: Required fields must be present
5. **Timeliness**: Data must be recent (< 5 minutes old for real-time)

### 9.3 Missing Data Handling

- Use last known good value (max 1 hour)
- Interpolation for short gaps (< 5 minutes)
- Mark as "uncertain" quality
- Generate alarm if gap exceeds threshold

---

## 10. Data Aggregation

### 10.1 Time-based Aggregation

**Hourly Aggregation:**
```json
{
  "metric": "temperature",
  "period": "hourly",
  "timestamp": "2025-12-25T10:00:00Z",
  "statistics": {
    "min": 21.5,
    "max": 23.2,
    "avg": 22.4,
    "median": 22.5,
    "stdDev": 0.4
  },
  "sampleCount": 60
}
```

### 10.2 Spatial Aggregation

```json
{
  "metric": "occupancy",
  "aggregationLevel": "floor",
  "floorId": "floor-03",
  "timestamp": "2025-12-25T10:30:00Z",
  "zones": [
    {
      "zoneId": "zone-301",
      "count": 12,
      "capacity": 50
    }
  ],
  "total": {
    "count": 125,
    "capacity": 500,
    "rate": 0.25
  }
}
```

---

## 11. API Data Exchange Format

### 11.1 Request Format

```json
{
  "requestId": "string (UUID)",
  "timestamp": "ISO 8601 timestamp",
  "endpoint": "string",
  "method": "GET|POST|PUT|DELETE",
  "parameters": {
    "buildingId": "string",
    "startTime": "ISO 8601 timestamp",
    "endTime": "ISO 8601 timestamp",
    "metrics": ["temperature", "humidity", "co2"]
  },
  "authentication": {
    "token": "string (JWT)"
  }
}
```

### 11.2 Response Format

```json
{
  "requestId": "string (UUID)",
  "timestamp": "ISO 8601 timestamp",
  "status": "success|error",
  "statusCode": "integer (HTTP status code)",
  "data": "object|array",
  "pagination": {
    "page": "integer",
    "pageSize": "integer",
    "totalRecords": "integer",
    "totalPages": "integer"
  },
  "errors": [
    {
      "code": "string",
      "message": "string",
      "field": "string"
    }
  ]
}
```

---

**© 2025 SmileStory Inc. / WIA**
**弘益人間 (홍익인간) · Benefit All Humanity**
