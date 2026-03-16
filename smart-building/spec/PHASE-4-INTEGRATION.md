# WIA-CITY-003: Smart Building Standard
## PHASE 4 - INTEGRATION SPECIFICATION

**Version:** 1.0
**Status:** Active
**Category:** CITY
**Last Updated:** 2025-12-25

---

## 1. Integration Overview

### 1.1 Purpose

This specification defines how WIA-CITY-003 smart building systems integrate with:
- Building subsystems (HVAC, lighting, security, elevators)
- Enterprise systems (ERP, CMMS, BIM)
- External services (weather, grid, cloud platforms)
- Smart city infrastructure

### 1.2 Integration Architecture

```
┌─────────────────────────────────────────────────────┐
│              Cloud Services Layer                    │
│  (Analytics, AI/ML, Data Lake, API Gateway)         │
└────────────────────┬────────────────────────────────┘
                     │
┌────────────────────┴────────────────────────────────┐
│           Building Management Layer                  │
│  (BMS, HVAC, Lighting, Security, Energy Mgmt)       │
└────────────────────┬────────────────────────────────┘
                     │
┌────────────────────┴────────────────────────────────┐
│         Integration Middleware Layer                 │
│  (Protocol Gateways, Data Normalization, APIs)      │
└────────────────────┬────────────────────────────────┘
                     │
┌────────────────────┴────────────────────────────────┐
│            Field Device Layer                        │
│  (Sensors, Actuators, Controllers, Meters)          │
└─────────────────────────────────────────────────────┘
```

---

## 2. HVAC System Integration

### 2.1 HVAC Components

**Air Handling Units (AHU):**
- Supply/return fans
- Heating/cooling coils
- Dampers (mixing, outside air, return air)
- Filters
- Humidifiers

**Variable Air Volume (VAV) Boxes:**
- Damper control
- Reheat coils
- Zone temperature control

**Chillers:**
- Compressor control
- Condenser control
- Evaporator monitoring
- COP optimization

**Boilers:**
- Burner control
- Temperature control
- Safety monitoring

**Pumps:**
- Primary/secondary loops
- VFD control
- Differential pressure

### 2.2 HVAC Integration Points

**Monitoring:**
```json
{
  "ahuId": "AHU-001",
  "timestamp": "2025-12-25T10:30:00Z",
  "measurements": {
    "supplyAirTemp": {
      "value": 14.5,
      "unit": "celsius",
      "setpoint": 14.0
    },
    "returnAirTemp": {
      "value": 22.0,
      "unit": "celsius"
    },
    "mixedAirTemp": {
      "value": 18.0,
      "unit": "celsius"
    },
    "outsideAirTemp": {
      "value": 10.0,
      "unit": "celsius"
    },
    "supplyAirFlow": {
      "value": 5000,
      "unit": "m³/h"
    },
    "supplyFanSpeed": {
      "value": 75,
      "unit": "percent"
    },
    "filterDiffPressure": {
      "value": 125,
      "unit": "Pa",
      "threshold": 250
    },
    "coolingValvePosition": {
      "value": 45,
      "unit": "percent"
    },
    "heatingValvePosition": {
      "value": 0,
      "unit": "percent"
    },
    "outsideAirDamper": {
      "value": 30,
      "unit": "percent"
    }
  },
  "status": "running",
  "mode": "cooling",
  "occupancyMode": "occupied"
}
```

**Control:**
```json
{
  "command": "set-parameters",
  "targetId": "AHU-001",
  "parameters": {
    "supplyAirTempSetpoint": 14.0,
    "mode": "auto",
    "occupancySchedule": {
      "weekday": {
        "occupied": "06:00-19:00",
        "unoccupied": "19:00-06:00"
      },
      "weekend": {
        "occupied": "08:00-17:00",
        "unoccupied": "17:00-08:00"
      }
    },
    "economizer": {
      "enabled": true,
      "minOutsideAirTemp": 10,
      "maxOutsideAirTemp": 18
    }
  },
  "requestId": "cmd-12345",
  "timestamp": "2025-12-25T10:30:00Z"
}
```

### 2.3 HVAC Optimization Algorithms

**Free Cooling / Economizer:**
```python
def economizer_control(outside_temp, return_temp, setpoint):
    """
    Use outside air for free cooling when conditions are favorable
    """
    if outside_temp < return_temp and outside_temp > 5:
        # Favorable conditions for economizer
        damper_position = min(100, calculate_required_cooling(
            outside_temp, return_temp, setpoint
        ))
        cooling_valve = 0
    else:
        # Mechanical cooling required
        damper_position = minimum_ventilation_damper_position
        cooling_valve = calculate_cooling_valve_position(
            return_temp, setpoint
        )

    return {
        "outside_air_damper": damper_position,
        "cooling_valve": cooling_valve
    }
```

**Optimal Start/Stop:**
```python
def optimal_start_time(
    current_temp, target_temp, occupancy_time,
    thermal_mass, outdoor_temp
):
    """
    Calculate when to start HVAC to reach target temp by occupancy time
    """
    temp_difference = abs(target_temp - current_temp)

    # Estimate warm-up/cool-down rate (degrees per hour)
    rate = calculate_thermal_rate(
        thermal_mass, outdoor_temp, current_temp
    )

    # Calculate required time
    required_hours = temp_difference / rate

    # Add safety margin
    start_time = occupancy_time - timedelta(hours=required_hours * 1.2)

    return start_time
```

**Demand Control Ventilation:**
```python
def demand_control_ventilation(co2_level, occupancy):
    """
    Adjust ventilation based on CO2 and occupancy
    """
    if co2_level > 1000:
        # High CO2, increase ventilation
        damper_position = min(100,
            base_ventilation + (co2_level - 1000) * 0.1
        )
    elif co2_level < 600 and occupancy < 0.3:
        # Low CO2 and occupancy, reduce ventilation
        damper_position = minimum_ventilation
    else:
        # Normal operation
        damper_position = base_ventilation

    return damper_position
```

---

## 3. Lighting System Integration

### 3.1 Lighting Control Strategies

**Occupancy-Based Control:**
```json
{
  "strategy": "occupancy-based",
  "zone": "office-floor-3-zone-a",
  "configuration": {
    "occupiedBrightness": 75,
    "unoccupiedBrightness": 0,
    "partialOccupancyBrightness": 40,
    "timeout": 300,
    "fadeTime": 5
  }
}
```

**Daylight Harvesting:**
```json
{
  "strategy": "daylight-harvesting",
  "zone": "office-floor-3-zone-a",
  "configuration": {
    "targetIlluminance": 500,
    "daylightSensor": "sensor-light-001",
    "minBrightness": 20,
    "maxBrightness": 100,
    "controlLoop": {
      "algorithm": "PID",
      "kp": 0.5,
      "ki": 0.1,
      "kd": 0.05
    }
  }
}
```

**Circadian Lighting:**
```json
{
  "strategy": "circadian-rhythm",
  "zone": "office-floor-3-zone-a",
  "configuration": {
    "morning": {
      "time": "06:00",
      "colorTemp": 4000,
      "brightness": 70
    },
    "midday": {
      "time": "12:00",
      "colorTemp": 5500,
      "brightness": 100
    },
    "afternoon": {
      "time": "15:00",
      "colorTemp": 4500,
      "brightness": 80
    },
    "evening": {
      "time": "18:00",
      "colorTemp": 3000,
      "brightness": 50
    }
  }
}
```

### 3.2 Lighting Scenes

```json
{
  "sceneId": "scene-presentation",
  "name": "Presentation Mode",
  "zones": ["conference-room-01"],
  "fixtures": [
    {
      "fixtureId": "light-conf-001",
      "brightness": 30,
      "colorTemp": 3500
    },
    {
      "fixtureId": "light-conf-002",
      "brightness": 30,
      "colorTemp": 3500
    },
    {
      "fixtureId": "light-conf-projector",
      "brightness": 0
    }
  ],
  "blinds": [
    {
      "blindId": "blind-conf-001",
      "position": 100
    }
  ],
  "triggers": [
    {
      "type": "button",
      "deviceId": "switch-conf-001",
      "button": 3
    },
    {
      "type": "schedule",
      "weekday": "monday-friday",
      "time": "14:00"
    }
  ]
}
```

### 3.3 Lighting Energy Savings

```python
def calculate_lighting_savings(
    occupancy_pattern, daylight_pattern,
    manual_usage, automated_usage
):
    """
    Calculate energy savings from automated lighting control
    """
    # Baseline: Manual control (always on during business hours)
    baseline_hours = 12  # 7am-7pm
    baseline_power = 1000  # Watts
    baseline_energy = baseline_hours * baseline_power * 365 / 1000  # kWh/year

    # Automated: Occupancy + daylight harvesting
    occupied_hours = sum(occupancy_pattern)  # Actual occupancy
    daylight_reduction = sum(daylight_pattern)  # Dimming from daylight

    automated_energy = (
        occupied_hours * baseline_power * (1 - daylight_reduction)
    ) / 1000  # kWh/year

    savings = {
        "baseline_kWh": baseline_energy,
        "automated_kWh": automated_energy,
        "savings_kWh": baseline_energy - automated_energy,
        "savings_percent": (
            (baseline_energy - automated_energy) / baseline_energy * 100
        ),
        "cost_savings": (
            (baseline_energy - automated_energy) * electricity_rate
        )
    }

    return savings
```

---

## 4. Security & Access Control Integration

### 4.1 Access Control System

**Access Events:**
```json
{
  "eventId": "access-event-12345",
  "timestamp": "2025-12-25T10:30:00Z",
  "eventType": "access-granted",
  "location": {
    "buildingId": "building-001",
    "doorId": "door-entrance-main",
    "floor": 1,
    "zone": "lobby"
  },
  "credential": {
    "type": "badge",
    "id": "CARD-12345",
    "userId": "user-001",
    "userName": "John Doe"
  },
  "accessLevel": "employee",
  "verified": true,
  "method": "card-reader"
}
```

**Access Rules:**
```json
{
  "ruleId": "rule-001",
  "name": "Employee Access - Business Hours",
  "priority": 10,
  "conditions": {
    "accessLevel": ["employee", "manager", "executive"],
    "schedule": {
      "weekday": "monday-friday",
      "time": "06:00-20:00"
    },
    "doors": ["door-entrance-main", "door-parking"]
  },
  "actions": {
    "grant": true,
    "logEvent": true,
    "notifyGuard": false
  }
}
```

### 4.2 Video Surveillance Integration

**Camera Integration:**
```json
{
  "cameraId": "camera-entrance-01",
  "location": {
    "buildingId": "building-001",
    "floor": 1,
    "zone": "entrance",
    "coordinates": {
      "latitude": 37.7749,
      "longitude": -122.4194
    }
  },
  "capabilities": {
    "resolution": "1920x1080",
    "fps": 30,
    "ptz": true,
    "nightVision": true,
    "analytics": ["motion-detection", "face-recognition", "people-counting"]
  },
  "streamUrls": {
    "rtsp": "rtsp://camera-entrance-01:554/stream",
    "hls": "https://video-server/camera-entrance-01/stream.m3u8"
  },
  "recordingPolicy": {
    "continuous": true,
    "retentionDays": 30,
    "eventTriggered": true,
    "eventRetentionDays": 90
  }
}
```

**Analytics Events:**
```json
{
  "eventId": "analytics-event-67890",
  "timestamp": "2025-12-25T10:35:00Z",
  "cameraId": "camera-entrance-01",
  "eventType": "tailgating-detected",
  "confidence": 0.87,
  "metadata": {
    "authorizedPerson": "user-001",
    "unauthorizedPersons": 1,
    "videoClipUrl": "https://video-server/clips/event-67890.mp4",
    "thumbnailUrl": "https://video-server/clips/event-67890-thumb.jpg"
  },
  "actions": [
    {
      "type": "alert",
      "target": "security-guard",
      "message": "Possible tailgating at main entrance"
    },
    {
      "type": "log",
      "severity": "warning"
    }
  ]
}
```

### 4.3 Intrusion Detection Integration

```json
{
  "zoneId": "security-zone-basement",
  "armed": true,
  "armedMode": "away",
  "sensors": [
    {
      "sensorId": "motion-basement-01",
      "type": "motion-detector",
      "status": "normal",
      "batteryLevel": 85
    },
    {
      "sensorId": "door-contact-basement-02",
      "type": "door-contact",
      "status": "closed",
      "batteryLevel": 92
    },
    {
      "sensorId": "glass-break-basement-03",
      "type": "glass-break",
      "status": "normal",
      "batteryLevel": 78
    }
  ],
  "alarmTriggers": [
    {
      "triggerId": "trigger-motion",
      "condition": "motion-detected AND armed=true",
      "actions": ["sound-alarm", "notify-security", "activate-cameras"]
    }
  ]
}
```

---

## 5. Elevator System Integration

### 5.1 Elevator Control

**Elevator Status:**
```json
{
  "elevatorId": "elevator-001",
  "timestamp": "2025-12-25T10:30:00Z",
  "status": "running",
  "currentFloor": 5,
  "direction": "up",
  "doorStatus": "closed",
  "load": {
    "current": 450,
    "capacity": 1000,
    "unit": "kg",
    "percent": 45
  },
  "passengers": 6,
  "destinationFloors": [8, 12, 15],
  "mode": "normal",
  "nextMaintenance": "2025-02-15",
  "operatingHours": 12450
}
```

**Elevator Dispatch Algorithm:**
```python
def dispatch_elevator(call_floor, direction, elevator_statuses):
    """
    Dispatch the most suitable elevator for a call
    """
    suitable_elevators = []

    for elevator in elevator_statuses:
        if elevator['status'] != 'running' or elevator['mode'] == 'maintenance':
            continue

        # Calculate suitability score
        score = 0

        # Same direction and passing by
        if (elevator['direction'] == direction and
            ((direction == 'up' and elevator['currentFloor'] < call_floor) or
             (direction == 'down' and elevator['currentFloor'] > call_floor))):
            score += 100

        # Distance penalty
        distance = abs(elevator['currentFloor'] - call_floor)
        score -= distance * 2

        # Load penalty (prefer less loaded cars)
        score -= elevator['load']['percent']

        suitable_elevators.append({
            'elevator_id': elevator['elevatorId'],
            'score': score,
            'eta': calculate_eta(elevator, call_floor)
        })

    # Return best elevator
    if suitable_elevators:
        return max(suitable_elevators, key=lambda x: x['score'])
    return None
```

### 5.2 Destination Dispatch System

```json
{
  "systemType": "destination-dispatch",
  "lobbyTerminals": ["terminal-lobby-01", "terminal-lobby-02"],
  "passengerRequest": {
    "terminalId": "terminal-lobby-01",
    "destinationFloor": 12,
    "timestamp": "2025-12-25T10:30:00Z"
  },
  "dispatch": {
    "assignedElevator": "elevator-003",
    "estimatedArrival": 45,
    "instruction": "Please proceed to Elevator C"
  },
  "optimization": {
    "averageWaitTime": 28,
    "averageJourneyTime": 62,
    "energyEfficiency": 0.87
  }
}
```

---

## 6. Energy Management Integration

### 6.1 Energy Monitoring Architecture

```
┌──────────────────────────────────────────┐
│        Energy Management System          │
├──────────────────────────────────────────┤
│                                          │
│  ┌──────────────────────────────────┐   │
│  │  Energy Analytics Dashboard      │   │
│  └──────────────────────────────────┘   │
│                                          │
│  ┌──────────┐  ┌──────────┐  ┌────────┐│
│  │ Electric │  │   Gas    │  │ Water  ││
│  │  Meter   │  │  Meter   │  │ Meter  ││
│  └─────┬────┘  └─────┬────┘  └────┬───┘│
│        │             │             │    │
└────────┼─────────────┼─────────────┼────┘
         │             │             │
    ┌────┴────┐   ┌────┴────┐   ┌───┴────┐
    │ Modbus  │   │ Modbus  │   │M-Bus   │
    │  TCP    │   │   RTU   │   │        │
    └─────────┘   └─────────┘   └────────┘
```

### 6.2 Real-Time Energy Monitoring

```json
{
  "buildingId": "building-001",
  "timestamp": "2025-12-25T10:30:00Z",
  "electricity": {
    "totalPower": 245.6,
    "voltage": {
      "l1": 230.2,
      "l2": 229.8,
      "l3": 230.5
    },
    "current": {
      "l1": 142.3,
      "l2": 138.7,
      "l3": 145.2
    },
    "powerFactor": 0.92,
    "frequency": 50.02,
    "energyToday": 1847.3,
    "peakDemand": 287.4,
    "breakdown": {
      "hvac": 135.2,
      "lighting": 45.8,
      "plugLoads": 42.6,
      "elevators": 12.3,
      "other": 9.7
    }
  },
  "gas": {
    "flowRate": 12.5,
    "totalToday": 298.4,
    "pressure": 21.5
  },
  "water": {
    "flowRate": 0.85,
    "totalToday": 18.2,
    "pressure": 4.2
  }
}
```

### 6.3 Demand Response Integration

```json
{
  "eventId": "dr-event-001",
  "timestamp": "2025-12-25T14:00:00Z",
  "eventType": "peak-demand-response",
  "severity": "moderate",
  "duration": 7200,
  "targetReduction": {
    "power": 50,
    "unit": "kW",
    "percent": 20
  },
  "strategies": [
    {
      "action": "increase-temp-setpoint",
      "system": "hvac",
      "parameter": "cooling-setpoint",
      "adjustment": 2,
      "expectedSavings": 25
    },
    {
      "action": "dim-lights",
      "system": "lighting",
      "zones": ["non-critical"],
      "dimLevel": 70,
      "expectedSavings": 15
    },
    {
      "action": "defer-non-critical",
      "systems": ["water-heaters", "non-essential-equipment"],
      "expectedSavings": 10
    }
  ],
  "verification": {
    "baselinePower": 250,
    "actualPower": 198,
    "reduction": 52,
    "compliance": true
  }
}
```

---

## 7. External System Integration

### 7.1 Weather Service Integration

```json
{
  "source": "weather-service",
  "buildingLocation": {
    "latitude": 37.7749,
    "longitude": -122.4194
  },
  "current": {
    "timestamp": "2025-12-25T10:30:00Z",
    "temperature": 18.5,
    "humidity": 65,
    "pressure": 1013.2,
    "windSpeed": 12.5,
    "windDirection": 225,
    "cloudCover": 40,
    "solarRadiation": 650
  },
  "forecast": {
    "next_4_hours": {
      "temperature": 20.2,
      "precipitation": 0,
      "cloudCover": 35
    },
    "next_24_hours": {
      "high": 22.5,
      "low": 15.3,
      "precipitation": 10
    }
  },
  "integration": {
    "hvac": "Adjust economizer based on forecast",
    "lighting": "Schedule daylight harvesting",
    "irrigation": "Defer watering due to rain forecast"
  }
}
```

### 7.2 Smart Grid Integration

```json
{
  "utilityProvider": "city-power-grid",
  "timestamp": "2025-12-25T10:30:00Z",
  "pricing": {
    "current": 0.12,
    "forecast": [
      {"time": "11:00", "rate": 0.13},
      {"time": "12:00", "rate": 0.18},
      {"time": "13:00", "rate": 0.22},
      {"time": "14:00", "rate": 0.25},
      {"time": "15:00", "rate": 0.22}
    ],
    "unit": "USD/kWh"
  },
  "gridStatus": {
    "demand": "high",
    "renewablePercent": 45,
    "carbonIntensity": 380
  },
  "demandResponse": {
    "eventActive": false,
    "nextEvent": "2025-12-25T13:00:00Z"
  },
  "buildingResponse": {
    "preCondition": true,
    "shiftLoad": ["battery-charging", "water-heating"],
    "targetPeakReduction": 15
  }
}
```

### 7.3 Building Information Modeling (BIM) Integration

```json
{
  "bimModel": "building-001-revit-2024.rvt",
  "version": "2024.1",
  "lastUpdated": "2025-01-15",
  "integration": {
    "sensors": {
      "mapped": 287,
      "total": 300,
      "unmapped": 13
    },
    "equipment": {
      "mapped": 156,
      "total": 160,
      "unmapped": 4
    },
    "spaces": {
      "mapped": 450,
      "total": 450,
      "unmapped": 0
    }
  },
  "digitalTwin": {
    "enabled": true,
    "updateFrequency": "real-time",
    "features": [
      "3D-visualization",
      "energy-simulation",
      "space-utilization",
      "maintenance-planning"
    ]
  },
  "dataExchange": {
    "format": "IFC 4.0",
    "api": "https://bim-server/api/building-001",
    "authentication": "OAuth 2.0"
  }
}
```

---

## 8. Cloud Platform Integration

### 8.1 Azure IoT Integration

```json
{
  "platform": "Azure IoT Hub",
  "connectionString": "HostName=building-iot.azure-devices.net;...",
  "deviceId": "building-001",
  "authentication": "SAS-token",
  "communication": {
    "telemetry": {
      "protocol": "MQTT",
      "frequency": "1-minute",
      "topics": [
        "devices/building-001/messages/events"
      ]
    },
    "commands": {
      "protocol": "MQTT",
      "topics": [
        "devices/building-001/messages/devicebound"
      ]
    },
    "deviceTwin": {
      "enabled": true,
      "properties": {
        "buildingName": "SmartCity Tower",
        "location": "San Francisco",
        "certifications": ["LEED-Platinum", "WELL-Gold"]
      }
    }
  },
  "services": {
    "streamAnalytics": true,
    "timeSeriesInsights": true,
    "digitalTwins": true,
    "machineLearning": true
  }
}
```

### 8.2 AWS IoT Integration

```json
{
  "platform": "AWS IoT Core",
  "endpoint": "xxxxx-ats.iot.us-west-2.amazonaws.com",
  "clientId": "building-001",
  "authentication": "X.509-certificate",
  "communication": {
    "telemetry": {
      "topic": "building/building-001/telemetry",
      "qos": 1
    },
    "shadow": {
      "enabled": true,
      "topics": {
        "update": "$aws/things/building-001/shadow/update",
        "get": "$aws/things/building-001/shadow/get"
      }
    }
  },
  "services": {
    "iotAnalytics": true,
    "timestream": true,
    "sagemaker": true,
    "lambda": true
  },
  "rules": [
    {
      "name": "HighTemperatureAlert",
      "sql": "SELECT * FROM 'building/+/telemetry' WHERE temperature > 30",
      "actions": ["SNS", "Lambda"]
    }
  ]
}
```

---

## 9. API Specification

### 9.1 RESTful API Endpoints

**Building Information:**
```
GET    /api/v1/buildings
GET    /api/v1/buildings/{id}
POST   /api/v1/buildings
PUT    /api/v1/buildings/{id}
DELETE /api/v1/buildings/{id}
```

**Real-Time Data:**
```
GET    /api/v1/buildings/{id}/sensors
GET    /api/v1/buildings/{id}/sensors/{sensorId}/current
GET    /api/v1/buildings/{id}/sensors/{sensorId}/history
POST   /api/v1/buildings/{id}/sensors/{sensorId}/command
```

**Energy Management:**
```
GET    /api/v1/buildings/{id}/energy/current
GET    /api/v1/buildings/{id}/energy/history
GET    /api/v1/buildings/{id}/energy/breakdown
GET    /api/v1/buildings/{id}/energy/forecast
```

**HVAC Control:**
```
GET    /api/v1/buildings/{id}/hvac/systems
GET    /api/v1/buildings/{id}/hvac/zones
POST   /api/v1/buildings/{id}/hvac/setpoint
POST   /api/v1/buildings/{id}/hvac/mode
```

**Lighting Control:**
```
GET    /api/v1/buildings/{id}/lighting/zones
GET    /api/v1/buildings/{id}/lighting/scenes
POST   /api/v1/buildings/{id}/lighting/brightness
POST   /api/v1/buildings/{id}/lighting/scene
```

### 9.2 WebSocket API

**Real-Time Subscriptions:**
```javascript
// Connect to WebSocket
const ws = new WebSocket('wss://api.wia-city.com/v1/stream');

// Subscribe to building data
ws.send(JSON.stringify({
  action: 'subscribe',
  topic: 'building/building-001/sensors/#',
  qos: 1
}));

// Receive real-time updates
ws.onmessage = (event) => {
  const data = JSON.parse(event.data);
  console.log('Sensor update:', data);
};
```

### 9.3 GraphQL API

```graphql
type Building {
  id: ID!
  name: String!
  address: Address!
  floors: [Floor!]!
  sensors: [Sensor!]!
  energy: EnergyData!
  hvac: [HVACSystem!]!
  lighting: [LightingZone!]!
}

type Query {
  building(id: ID!): Building
  buildings: [Building!]!
  sensorData(
    buildingId: ID!,
    sensorType: String,
    startTime: DateTime,
    endTime: DateTime
  ): [SensorReading!]!
}

type Mutation {
  setHVACSetpoint(
    buildingId: ID!,
    zoneId: ID!,
    temperature: Float!
  ): HVACZone!

  setLightingBrightness(
    buildingId: ID!,
    zoneId: ID!,
    brightness: Int!
  ): LightingZone!
}

type Subscription {
  sensorUpdates(buildingId: ID!): SensorReading!
  alarmNotifications(buildingId: ID!): Alarm!
}
```

---

## 10. Testing & Validation

### 10.1 Integration Testing Checklist

**HVAC Integration:**
- [ ] Read all sensor values
- [ ] Write setpoints
- [ ] Subscribe to COV notifications
- [ ] Execute control sequences
- [ ] Verify alarm handling

**Lighting Integration:**
- [ ] Control individual fixtures
- [ ] Execute scenes
- [ ] Verify daylight harvesting
- [ ] Test occupancy sensors
- [ ] Validate energy savings

**Security Integration:**
- [ ] Access control events
- [ ] Video surveillance feeds
- [ ] Intrusion detection
- [ ] Integration with HVAC/lighting

**Energy Management:**
- [ ] Real-time meter data
- [ ] Historical data retrieval
- [ ] Demand response execution
- [ ] Energy analytics

### 10.2 Performance Testing

- Load testing: 10,000+ data points/second
- Latency testing: < 1 second end-to-end
- Failover testing: Automatic recovery
- Scalability testing: Multi-building support

---

**© 2025 SmileStory Inc. / WIA**
**弘益人間 (홍익인간) · Benefit All Humanity**
