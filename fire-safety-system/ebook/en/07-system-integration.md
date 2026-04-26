# Chapter 7: System Integration (Phase 4)

## Overview

Phase 4 of the WIA Standard defines comprehensive integration specifications enabling fire safety systems to coordinate with building management systems, access control, HVAC, elevator control, lighting, and emergency services. This chapter explores integration architectures, automated response actions, emergency services notification, and third-party application integration.

---

## Integration Architecture

### Holistic Building Safety System

```
Fire Safety System Integration Architecture:

┌─────────────────────────────────────────────────────────┐
│         Fire Alarm Control Panel (FACP)                 │
│              WIA Standard Interface                     │
└────┬─────────┬──────────┬──────────┬──────────┬────────┘
     │         │          │          │          │
     │         │          │          │          │
┌────▼────┐ ┌─▼──────┐ ┌─▼──────┐ ┌─▼──────┐ ┌─▼────────┐
│  HVAC   │ │ Access │ │Elevator│ │Lighting│ │Emergency │
│ Control │ │Control │ │Control │ │Control │ │Services  │
└────┬────┘ └───┬────┘ └───┬────┘ └───┬────┘ └────┬─────┘
     │          │          │          │          │
     │          │          │          │          │
 Shutdown    Unlock    Recall to   Switch to   Auto-
 air fans    all exits  ground     emergency   dispatch
                        floor      lighting    911/999

Integration Benefits:
✓ Coordinated automated response
✓ Faster evacuation (30% improvement)
✓ Enhanced firefighter access
✓ Reduced property damage
✓ Compliance with building codes
✓ Improved situational awareness
```

### Integration Protocols

The WIA Standard supports multiple integration methods:

```
Integration Protocol Stack:

┌─────────────────────────────────────────────┐
│ Application Layer                           │
│ • WIA Fire Safety API (RESTful/WebSocket)   │
├─────────────────────────────────────────────┤
│ Standard Building Protocols                 │
│ • BACnet (Building automation)              │
│ • Modbus (Industrial control)               │
│ • OPC UA (Industrial interoperability)      │
│ • MQTT (IoT messaging)                      │
│ • CoAP (Constrained devices)                │
├─────────────────────────────────────────────┤
│ Custom Integration                          │
│ • Vendor-specific APIs via adapters         │
│ • Legacy protocol gateways                  │
│ • Custom middleware development             │
└─────────────────────────────────────────────┘
```

---

## Building Management System (BMS) Integration

### HVAC Control

**On Fire Alarm Activation:**

```
HVAC Response Actions:

┌─────────────────────────────────────────────┐
│ 1. Shutdown Supply/Return Fans              │
│    • Alarm zone: Immediate shutdown         │
│    • Adjacent zones: Shutdown if directed   │
│    • Other areas: Maintain per strategy     │
│                                             │
│ 2. Close Fire/Smoke Dampers                 │
│    • All dampers per NFPA 90A               │
│    • Close within 15 seconds                │
│    • Confirm closure electrically           │
│                                             │
│ 3. Activate Smoke Exhaust                   │
│    • Designated smoke zones                 │
│    • Coordinate with stairwell pressure     │
│    • Variable speed based on conditions     │
│                                             │
│ 4. Pressurize Stairwells                    │
│    • Maintain positive pressure             │
│    • Prevent smoke infiltration             │
│    • Typical: 0.10-0.35 inches w.g.         │
│                                             │
│ 5. Pressurize Elevator Shafts               │
│    • Prevent smoke entry to elevators       │
│    • Coordinate with elevator recall        │
│                                             │
│ 6. Maintain Minimum Ventilation             │
│    • Critical areas (server rooms, etc.)    │
│    • Per code requirements                  │
│    • Balance life safety vs property        │
└─────────────────────────────────────────────┘
```

**Integration API Example:**

```http
POST /api/v1/integrations/hvac/fire-alarm-response HTTP/1.1
Host: bms.building.com
Authorization: Bearer <token>
Content-Type: application/json

{
  "alarmEventId": "9f4e2c8a-5d3b-4a7e-9c1f-8e2d4a6c3b5f",
  "location": {
    "building": "Main Tower",
    "floor": 12,
    "zone": "East Wing E12-A"
  },
  "actions": [
    {
      "system": "HVAC",
      "action": "shutdown_fans",
      "zones": ["E12-A", "E12-B", "E13-A"],
      "priority": "critical"
    },
    {
      "system": "HVAC",
      "action": "close_dampers",
      "dampers": ["FD-12-01", "FD-12-02", "FD-12-03"],
      "priority": "critical"
    },
    {
      "system": "HVAC",
      "action": "activate_smoke_exhaust",
      "zones": ["E12-A"],
      "priority": "high"
    },
    {
      "system": "HVAC",
      "action": "pressurize_stairwells",
      "stairwells": ["ST-A", "ST-B"],
      "targetPressure": 0.25,
      "priority": "high"
    }
  ]
}
```

**HVAC System Response:**

```json
{
  "responseId": "hvac-resp-1234567890",
  "timestamp": "2025-12-27T14:32:16.500Z",
  "actionsCompleted": [
    {
      "action": "shutdown_fans",
      "status": "completed",
      "completionTime": "2025-12-27T14:32:16.200Z",
      "fansShutdown": 8
    },
    {
      "action": "close_dampers",
      "status": "completed",
      "completionTime": "2025-12-27T14:32:17.100Z",
      "dampersClosed": 3,
      "confirmations": ["FD-12-01", "FD-12-02", "FD-12-03"]
    },
    {
      "action": "activate_smoke_exhaust",
      "status": "in_progress",
      "estimatedCompletion": "2025-12-27T14:32:25.000Z"
    },
    {
      "action": "pressurize_stairwells",
      "status": "in_progress",
      "currentPressure": 0.15,
      "targetPressure": 0.25
    }
  ]
}
```

---

## Access Control Integration

### Automated Egress

**On Fire Alarm Activation:**

```
Access Control Response Actions:

Priority 1: Life Safety Egress (T+0s)
┌─────────────────────────────────────────────┐
│ • Unlock all exit doors                     │
│ • Release electromagnetic locks             │
│ • Open turnstiles and gates                 │
│ • Disable card readers at exits             │
│ • Green lights on all exit points           │
└─────────────────────────────────────────────┘

Priority 2: Floor Access (T+5s)
┌─────────────────────────────────────────────┐
│ • Unlock all doors on alarm floor           │
│ • Unlock stairwell doors (re-entry)         │
│ • Unlock corridor cross-doors               │
│ • Maintain suite security (configurable)    │
└─────────────────────────────────────────────┘

Priority 3: Perimeter Security (Ongoing)
┌─────────────────────────────────────────────┐
│ • Maintain entry security                   │
│ • Log all access events                     │
│ • CCTV recording active                     │
│ • Prevent unauthorized entry                │
└─────────────────────────────────────────────┘
```

**Integration Example:**

```json
{
  "alarmEventId": "9f4e2c8a-5d3b-4a7e-9c1f-8e2d4a6c3b5f",
  "timestamp": "2025-12-27T14:32:15.000Z",
  "accessControlActions": {
    "unlockAllExits": {
      "enabled": true,
      "doors": [
        "EXIT-E-12-01",
        "EXIT-E-12-02",
        "STAIR-A-12",
        "STAIR-B-12"
      ],
      "status": "completed",
      "completionTime": "2025-12-27T14:32:15.800Z"
    },
    "releaseElectromagneticLocks": {
      "enabled": true,
      "locks": 24,
      "status": "completed",
      "completionTime": "2025-12-27T14:32:15.500Z"
    },
    "disableReaders": {
      "enabled": true,
      "readers": ["EXIT-READER-001", "EXIT-READER-002"],
      "mode": "free_egress"
    },
    "maintainPerimeter": {
      "enabled": true,
      "entryPoints": ["MAIN-LOBBY", "PARKING-ACCESS"],
      "mode": "secured"
    }
  }
}
```

---

## Elevator Control Integration

### Elevator Recall

**Fire Service Operation:**

```
Elevator Recall Sequence:

Phase 1: Initial Recall (T+0s)
┌─────────────────────────────────────────────┐
│ 1. All elevator car calls cancelled         │
│ 2. Elevators directed to recall floor       │
│    (typically ground floor)                 │
│ 3. Non-stop travel to recall floor          │
│ 4. Doors open upon arrival                  │
│ 5. Audio message: "Elevator out of service" │
└─────────────────────────────────────────────┘

Phase 2: Fire Service Mode (T+30s)
┌─────────────────────────────────────────────┐
│ 1. Elevators parked at recall floor         │
│ 2. Firefighter key switch required          │
│ 3. Manual control only                      │
│ 4. Car lighting remains on                  │
│ 5. Alarm bell silenced                      │
│ 6. Doors remain open until commanded        │
└─────────────────────────────────────────────┘

Phase 3: Firefighter Operation
┌─────────────────────────────────────────────┐
│ • Key switch activates car                  │
│ • Manual floor selection only               │
│ • Doors controlled by firefighter           │
│ • Override all safety features              │
│ • Direct communication to fire command      │
└─────────────────────────────────────────────┘

Special Cases:
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Scenario                  Action
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Smoke in shaft            Stop at nearest floor
Alarm on recall floor     Alternate recall floor
Power failure             Battery lower to ground
Occupants in car          Complete current trip first
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

**Integration Message:**

```json
{
  "alarmEventId": "9f4e2c8a-5d3b-4a7e-9c1f-8e2d4a6c3b5f",
  "timestamp": "2025-12-27T14:32:15.000Z",
  "elevatorControlActions": {
    "recallElevators": {
      "elevatorIds": ["ELEV-A", "ELEV-B", "ELEV-C", "ELEV-D"],
      "recallFloor": 1,
      "alternateRecallFloor": 2,
      "mode": "fire_service_phase_1"
    },
    "cancelAllCalls": true,
    "disableNormalOperation": true,
    "enableFirefighterService": true,
    "audioMessage": "This elevator is out of service due to fire alarm. Please use stairs.",
    "visualIndication": "flashing_red_light"
  }
}
```

---

## Emergency Lighting Integration

### Automatic Lighting Control

```
Emergency Lighting Response:

Immediate Actions (T+0s):
┌─────────────────────────────────────────────┐
│ • All emergency lighting activated          │
│ • Exit sign illumination maximized          │
│ • Path lighting to exits enabled            │
│ • Stairwell lighting to full brightness     │
│ • Disable dimming systems                   │
└─────────────────────────────────────────────┘

Enhanced Visibility (T+5s):
┌─────────────────────────────────────────────┐
│ • Alarm floor: Full lighting                │
│ • Exit paths: Maximum illumination          │
│ • Common areas: Enhanced lighting           │
│ • Non-critical areas: Maintain minimum      │
└─────────────────────────────────────────────┘

Visual Guidance:
┌─────────────────────────────────────────────┐
│ • Flashing exit signs (where permitted)     │
│ • Directional arrow indicators              │
│ • Color-coded path marking                  │
│ • Photoluminescent way-finding             │
└─────────────────────────────────────────────┘
```

---

## Emergency Services Integration

### Automated Notification

**Emergency Services API:**

```http
POST /api/v1/emergency-services/dispatch HTTP/1.1
Host: dispatch-system.city.gov
Authorization: Bearer <token>
Content-Type: application/json

{
  "emergencyType": "fire",
  "severity": "critical",
  "facility": {
    "facilityId": "BLDG-12345",
    "facilityName": "Acme Office Tower",
    "address": {
      "street": "123 Main Street",
      "city": "Anytown",
      "state": "CA",
      "postalCode": "90210",
      "country": "USA",
      "gpsCoordinates": {
        "latitude": 34.0522,
        "longitude": -118.2437,
        "accuracy": 5
      }
    },
    "facilityType": "commercial_office",
    "occupancyType": "business",
    "constructionType": "type_1A_fireproof",
    "floors": {
      "aboveGrade": 15,
      "belowGrade": 2,
      "totalFloors": 17
    },
    "squareFootage": 285000,
    "yearBuilt": 2018
  },
  "incident": {
    "detectionTime": "2025-12-27T14:32:15Z",
    "location": {
      "building": "Main Tower",
      "floor": 12,
      "zone": "East Wing E12-A",
      "room": "Conference Room 1205",
      "description": "12th floor, east wing, northeast corner"
    },
    "detectionMethod": "automatic_smoke_detector",
    "deviceType": "photoelectric_smoke_detector",
    "confirmationMethod": "multi_sensor_verified",
    "spreadPotential": "high"
  },
  "occupancy": {
    "estimatedOccupants": 450,
    "timeOfDay": "business_hours",
    "specialNeeds": [
      "Child care center (Floor 2, 15 children)",
      "Medical clinic (Floor 8, potential mobility issues)"
    ],
    "evacuationStatus": "in_progress"
  },
  "hazards": {
    "hazardousMaterials": [
      {
        "material": "Lithium-ion batteries",
        "location": "IT Equipment Room, Floor 5",
        "quantity": "500 units",
        "msdsAvailable": true
      }
    ],
    "structuralConcerns": [],
    "utilities": {
      "naturalGas": true,
      "fuelOil": false,
      "propane": false,
      "solarPanels": true
    }
  },
  "access": {
    "fireAccess": [
      "North entrance: Main Street, key box #1234",
      "East entrance: Parking garage, Level 1",
      "West entrance: Service dock"
    ],
    "keyBoxLocation": "North entrance, right of main door",
    "knoxBoxNumber": "KB-1234",
    "elevatorFireService": true,
    "standpipeLocations": ["Stairwell A", "Stairwell B"],
    "sprinklerSystemType": "wet_pipe",
    "fireDepartmentConnection": "North side, main entrance"
  },
  "resources": {
    "buildingPlans": {
      "url": "https://files.example.com/building-plans",
      "format": "PDF",
      "lastUpdated": "2025-01-15"
    },
    "videoFeeds": [
      {
        "name": "Floor 12 East Wing",
        "url": "https://cctv.example.com/live/floor12-east",
        "credentials": "Available via secure channel"
      },
      {
        "name": "Main Lobby",
        "url": "https://cctv.example.com/live/lobby"
      }
    ],
    "buildingAutomation": {
      "url": "https://bms.example.com/emergency-access",
      "capabilities": ["HVAC control", "Access control", "Elevator status"]
    }
  },
  "contacts": {
    "onSiteContacts": [
      {
        "role": "Building Manager",
        "name": "John Smith",
        "phone": "+1-555-0123",
        "mobile": "+1-555-0124",
        "availability": "on_site"
      },
      {
        "role": "Chief Engineer",
        "name": "Jane Doe",
        "phone": "+1-555-0125",
        "availability": "on_call"
      }
    ],
    "emergencyContact": {
      "company": "Acme Property Management",
      "phone": "+1-555-0100",
      "available247": true
    }
  },
  "fireProtectionSystems": {
    "fireAlarm": {
      "type": "addressable",
      "standard": "WIA_v1.0",
      "monitoring": "central_station",
      "monitoringCompany": "SecureWatch Monitoring"
    },
    "sprinklers": {
      "type": "wet_pipe",
      "coverage": "full_building",
      "waterSupply": "municipal_plus_onsite_tank"
    },
    "standpipes": {
      "type": "automatic_wet",
      "outlets": "all_floors"
    },
    "fireExtinguishers": {
      "portable": "per_code",
      "locations": "see_building_plans"
    }
  }
}
```

**Emergency Services Response:**

```json
{
  "dispatchId": "FD-2025-12-27-0156",
  "timestamp": "2025-12-27T14:32:17Z",
  "status": "dispatched",
  "response": {
    "units": [
      {
        "unitId": "ENGINE-5",
        "type": "engine",
        "personnel": 4,
        "status": "en_route",
        "eta": "2025-12-27T14:36:00Z"
      },
      {
        "unitId": "LADDER-2",
        "type": "ladder",
        "personnel": 4,
        "status": "en_route",
        "eta": "2025-12-27T14:36:30Z"
      },
      {
        "unitId": "BATTALION-1",
        "type": "battalion_chief",
        "personnel": 2,
        "status": "en_route",
        "eta": "2025-12-27T14:37:00Z"
      }
    ],
    "additionalResources": {
      "ambulance": "dispatched",
      "hazmat": "standby",
      "police": "notified"
    }
  },
  "incidentCommand": {
    "radioChannel": "FIRE-TAC-3",
    "commandPost": "North entrance",
    "staging": "Main Street, 200ft north"
  }
}
```

---

## Third-Party Application Integration

### Analytics and Monitoring

**Data Export API:**

```http
GET /api/v1/analytics/alarms/history HTTP/1.1
Host: panel.building.com
Authorization: Bearer <token>

Query Parameters:
  startDate=2025-12-01T00:00:00Z
  endDate=2025-12-27T23:59:59Z
  includeResolved=true
  format=json
```

**Response:**

```json
{
  "period": {
    "start": "2025-12-01T00:00:00Z",
    "end": "2025-12-27T23:59:59Z"
  },
  "statistics": {
    "totalAlarms": 47,
    "fireAlarms": 3,
    "falseAlarms": 38,
    "supervisory": 4,
    "trouble": 2
  },
  "alarms": [
    {
      "eventId": "...",
      "timestamp": "...",
      "type": "fire",
      "resolved": true,
      "responseTime": 180,
      "resolution": "Cooking smoke, no fire"
    }
  ],
  "trends": {
    "alarmsPerDay": 1.74,
    "falseAlarmRate": 80.85,
    "averageResponseTime": 245,
    "topAlarmZones": [
      {"zone": "Kitchen Level 1", "count": 12},
      {"zone": "Mechanical Floor 15", "count": 8}
    ]
  }
}
```

---

## Key Takeaways

1. **Coordinated integration** with building systems significantly enhances life safety and property protection.

2. **HVAC integration** controls smoke spread through strategic fan shutdown, damper closure, and pressurization.

3. **Access control integration** prioritizes life safety egress while maintaining perimeter security.

4. **Elevator recall** removes elevators from service and enables firefighter operation.

5. **Emergency services integration** provides comprehensive incident information enabling effective response.

---

## Review Questions

1. What HVAC actions are triggered on fire alarm activation?
2. How does access control balance life safety egress with perimeter security?
3. What is the sequence for elevator recall in fire service mode?
4. What information is included in emergency services notification?
5. How do third-party applications access fire alarm data?

---

## Next Steps

Chapter 8 provides implementation guidance including roadmaps, testing procedures, certification requirements, and deployment best practices.

---

© 2025 WIA Standards Committee. 弘益人間 (홍익인간) - Benefit All Humanity
