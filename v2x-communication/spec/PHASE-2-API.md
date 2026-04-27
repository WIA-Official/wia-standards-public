# PHASE 2 — API

> V2X application surface: the four cardinal V2X modes — V2V,
> V2I, V2P, V2N — and the verbs each mode exposes to the
> in-vehicle stack and to roadside / pedestrian / network
> infrastructure.

## 5. V2V (Vehicle-to-Vehicle)

### 5.1 Communication Model

**Broadcast-based:**
- Periodic BSM/CAM transmission (10 Hz)
- No handshake or acknowledgment
- Connectionless communication
- All nearby vehicles receive

**Event-triggered:**
- Emergency brake warning
- Collision warning
- Lane change notification
- Hazardous location warning

### 5.2 Cooperative Awareness

**Position Sharing:**
```
Update Frequency:
  - Stationary: 1 Hz
  - Low speed (<50 km/h): 2 Hz
  - Medium speed (50-100 km/h): 5 Hz
  - High speed (>100 km/h): 10 Hz
  - Emergency: 20 Hz
```

**Data Elements:**
- GPS position (lat, lon, elevation)
- Speed and heading
- Acceleration (longitudinal, lateral, vertical)
- Vehicle dimensions
- Brake status
- Turn signals

### 5.3 Collision Detection

**Time-to-Collision (TTC) Calculation:**
```
TTC = (d - L₁ - L₂) / (v₁ - v₂)

Where:
  d = Distance between vehicles
  L₁, L₂ = Vehicle lengths
  v₁, v₂ = Vehicle speeds
```

**Threat Assessment:**
```
if TTC < 1.5 seconds:
    Threat Level = CRITICAL (immediate brake)
elif TTC < 2.5 seconds:
    Threat Level = HIGH (warning + prepare brake)
elif TTC < 4.0 seconds:
    Threat Level = MEDIUM (visual/audio warning)
else:
    Threat Level = LOW (monitoring)
```

### 5.4 Forward Collision Warning

**Algorithm:**
```python
def forward_collision_warning(ego_vehicle, target_vehicle):
    # Calculate relative position
    distance = calculate_distance(ego_vehicle.position, target_vehicle.position)

    # Check if target is in path
    heading_diff = abs(ego_vehicle.heading - target_vehicle.heading)
    if heading_diff > 45:  # degrees
        return None  # Not in collision path

    # Calculate relative speed
    relative_speed = ego_vehicle.speed - target_vehicle.speed

    if relative_speed <= 0:
        return None  # Target moving away or same speed

    # Calculate TTC
    ttc = (distance - ego_vehicle.length) / relative_speed

    # Determine warning level
    if ttc < 1.5:
        return {'level': 'CRITICAL', 'action': 'BRAKE', 'ttc': ttc}
    elif ttc < 2.5:
        return {'level': 'HIGH', 'action': 'WARN', 'ttc': ttc}
    elif ttc < 4.0:
        return {'level': 'MEDIUM', 'action': 'MONITOR', 'ttc': ttc}

    return None
```

### 5.5 Blind Spot Warning

**Detection Zone:**
```
Left Blind Spot:
  - Range: 0.5m - 3.0m from vehicle side
  - Length: From B-pillar to 2m behind vehicle
  - Angle: 90° ± 30°

Right Blind Spot:
  - Same as left, mirrored
```

**Warning Conditions:**
- Vehicle detected in blind spot
- Turn signal activated
- Lateral acceleration detected

---


## 6. V2I (Vehicle-to-Infrastructure)

### 6.1 Road Side Unit (RSU)

**RSU Functions:**
- Broadcast SPaT messages (traffic signals)
- Transmit MAP data (intersection geometry)
- Collect traffic data
- Relay V2V messages (range extension)
- Emergency vehicle preemption

**RSU Deployment:**
```
High Priority:
  - Signalized intersections
  - Highway on/off ramps
  - School zones
  - Work zones

Medium Priority:
  - Parking areas
  - Transit stations
  - Toll plazas

Low Priority:
  - Rural highways
  - Residential areas
```

### 6.2 Traffic Signal Priority

**SPaT Processing:**
```typescript
interface SignalPhase {
  state: 'red' | 'yellow' | 'green';
  timeRemaining: number;  // seconds
  nextPhase: 'red' | 'yellow' | 'green';
  nextPhaseTime: number;
}

function processSpat(spatMessage: SPaTMessage): SignalPhase {
  const currentTime = Date.now();
  const signalGroup = spatMessage.states.find(s => s.signalGroup === vehicleLane);

  return {
    state: signalGroup.state,
    timeRemaining: (signalGroup.timing.minEndTime - currentTime) / 1000,
    nextPhase: determineNextPhase(signalGroup),
    nextPhaseTime: signalGroup.timing.nextTime / 1000
  };
}
```

**Green Light Optimal Speed Advisory (GLOSA):**
```
If signal_state == RED and time_to_intersection < time_remaining:
    recommended_speed = calculate_optimal_speed()
    display("Slow to " + recommended_speed + " km/h to catch green")

If signal_state == GREEN and time_to_intersection > time_remaining:
    display("Speed up or prepare to stop")
```

### 6.3 Emergency Vehicle Preemption

**Preemption Request:**
```json
{
  "messageType": "SRM",  // Signal Request Message
  "requestID": 12345,
  "vehicleID": "FIRE-001",
  "vehicleType": "fire",
  "priority": "emergency",
  "route": {
    "approach": "north",
    "departure": "south"
  },
  "eta": 15,  // seconds
  "duration": 120  // requested green time
}
```

**Signal Response:**
```json
{
  "messageType": "SSM",  // Signal Status Message
  "requestID": 12345,
  "status": "granted",
  "preemptionTime": 1640000015000,
  "duration": 120
}
```

### 6.4 Work Zone Warnings

**Work Zone Data:**
```json
{
  "messageType": "DENM",
  "eventType": "roadWorks",
  "location": {
    "startPoint": {"lat": 37.7749, "lon": -122.4194},
    "endPoint": {"lat": 37.7750, "lon": -122.4180},
    "affectedLanes": [1, 2]
  },
  "workType": "maintenance",
  "laneClosures": [1],
  "speedLimit": 40,  // km/h
  "startDate": "2025-12-26T08:00:00Z",
  "endDate": "2025-12-26T17:00:00Z"
}
```

---


## 7. V2P (Vehicle-to-Pedestrian)

### 7.1 Pedestrian Detection

**Pedestrian Device Types:**
- Smartphone apps (V2P-capable)
- Wearable devices (smartwatch, fitness tracker)
- Dedicated P-ITS devices

**Message Format:**
```json
{
  "messageType": "PSM",  // Personal Safety Message
  "deviceID": "PED-123456",
  "deviceType": "smartphone",
  "position": {
    "latitude": 377749000,
    "longitude": -1224194000,
    "accuracy": 5  // meters
  },
  "speed": 1.4,  // m/s (walking speed)
  "heading": 180,
  "userType": "pedestrian",
  "cluster": {
    "clusterSize": 1,
    "clusterRadius": 0
  },
  "timestamp": 1640000000000
}
```

### 7.2 Vulnerable Road User (VRU) Safety

**Risk Assessment:**
```python
def assess_vru_risk(vehicle, pedestrian):
    # Calculate distance
    distance = calculate_distance(vehicle.position, pedestrian.position)

    # Check if pedestrian in vehicle path
    is_in_path = check_intersection(
        vehicle.position,
        vehicle.heading,
        vehicle.speed,
        pedestrian.position,
        pedestrian.heading,
        pedestrian.speed
    )

    if not is_in_path:
        return {'risk': 'NONE'}

    # Calculate time to collision
    ttc = distance / vehicle.speed

    # Risk levels
    if ttc < 2.0:
        return {
            'risk': 'CRITICAL',
            'action': 'EMERGENCY_BRAKE',
            'ttc': ttc,
            'distance': distance
        }
    elif ttc < 4.0:
        return {
            'risk': 'HIGH',
            'action': 'WARN_AND_SLOW',
            'ttc': ttc,
            'distance': distance
        }
    elif ttc < 6.0:
        return {
            'risk': 'MEDIUM',
            'action': 'MONITOR',
            'ttc': ttc,
            'distance': distance
        }

    return {'risk': 'LOW'}
```

### 7.3 Crosswalk Safety

**Crosswalk Detection System:**
```
Approach Zone: 50m before crosswalk
  - Monitor for pedestrians
  - Reduce speed to 30 km/h

Warning Zone: 20m before crosswalk
  - If pedestrian detected: SLOW to 10 km/h
  - Activate hazard lights

Critical Zone: Crosswalk area
  - If pedestrian crossing: STOP
  - Wait until clear
```

### 7.4 Cyclist Protection

**Cyclist Message (BSM-variant):**
```json
{
  "messageType": "BSM",
  "vehicleType": "cyclist",
  "id": "BIKE-001",
  "position": {
    "latitude": 377749000,
    "longitude": -1224194000
  },
  "speed": 600,  // 12 km/h
  "heading": 90,
  "deviceType": "smartphone",
  "intent": {
    "turning": false,
    "stopping": false,
    "lane_change": false
  }
}
```

---


## 8. V2N (Vehicle-to-Network)

### 8.1 Cloud Connectivity

**Cloud Services:**
- Real-time traffic management
- Predictive routing
- Fleet management
- Remote diagnostics
- OTA software updates
- HD map updates

**Communication Protocol:**
```
Transport: HTTPS / MQTT / WebSocket
Data Format: JSON / Protocol Buffers
Authentication: OAuth 2.0 / mTLS
Encryption: TLS 1.3
```

### 8.2 Traffic Management Center (TMC) Integration

**Vehicle Telemetry Upload:**
```json
{
  "messageType": "TELEMETRY",
  "vehicleID": "VEH-123456",
  "timestamp": 1640000000000,
  "position": {
    "latitude": 37.7749,
    "longitude": -122.4194,
    "heading": 270,
    "speed": 65
  },
  "roadConditions": {
    "friction": 0.8,
    "wetness": "dry",
    "visibility": "good"
  },
  "trafficConditions": {
    "density": "moderate",
    "averageSpeed": 60,
    "incidents": []
  }
}
```

**TMC Traffic Updates:**
```json
{
  "messageType": "TRAFFIC_UPDATE",
  "region": {
    "bounds": {
      "north": 37.8,
      "south": 37.7,
      "east": -122.3,
      "west": -122.5
    }
  },
  "incidents": [
    {
      "id": "INC-001",
      "type": "accident",
      "location": {"lat": 37.7749, "lon": -122.4194},
      "severity": "major",
      "lanesAffected": [1, 2],
      "estimatedClearance": "2025-12-26T14:30:00Z"
    }
  ],
  "congestion": [
    {
      "roadID": "I-280",
      "segment": "Northbound Mile 15-20",
      "speed": 25,
      "delay": 10  // minutes
    }
  ]
}
```

### 8.3 Remote Monitoring and Control

**Remote Vehicle Status:**
```json
{
  "vehicleID": "VEH-123456",
  "status": {
    "location": {"lat": 37.7749, "lon": -122.4194},
    "battery": 85,  // %
    "fuel": 45,  // liters
    "speed": 0,
    "odometer": 45682,  // km
    "engineStatus": "off",
    "doors": {
      "driverFront": "locked",
      "passengerFront": "locked",
      "driverRear": "locked",
      "passengerRear": "locked",
      "trunk": "locked"
    },
    "diagnostics": {
      "engineHealth": "good",
      "brakeHealth": "good",
      "tirePress sure": [32, 32, 32, 32]  // PSI
    }
  }
}
```

---


