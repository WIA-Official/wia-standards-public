# WIA-CITY-007: Construction Robot Standard
## PHASE 2 - DATA FORMAT SPECIFICATION

**Version:** 1.0
**Status:** Draft
**Last Updated:** 2025-12-25
**Category:** CITY (Smart City & Urban Infrastructure)

---

## 1. Overview

Phase 2 defines standardized data formats for construction robots, ensuring interoperability across manufacturers and enabling seamless integration with site management systems, BIM software, and monitoring platforms.

### 1.1 Design Principles

**Consistency:** All robots use common data structures
**Extensibility:** Support for custom fields and future expansion
**Efficiency:** Compact representation for real-time telemetry
**Security:** Cryptographic signatures and integrity verification
**Compatibility:** JSON-LD for semantic interoperability

---

## 2. Robot Status Data Format

### 2.1 Base Status Structure

All construction robots MUST report status using this format:

```json
{
  "@context": "https://wia.org/schemas/construction-robot/v1",
  "@type": "RobotStatus",
  "robotId": "did:wia:robot:CR-BRICK-001",
  "timestamp": "2025-12-25T10:30:00Z",

  "identity": {
    "manufacturer": "RoboCon Industries",
    "model": "BrickMaster-3000",
    "serialNumber": "BM3000-2025-0142",
    "firmwareVersion": "2.5.1"
  },

  "location": {
    "type": "Point",
    "coordinates": [126.9780, 37.5665, 15.0],
    "coordinateSystem": "WGS84",
    "floor": 5,
    "zone": "A-3",
    "accuracy": 0.5
  },

  "operational": {
    "status": "active",
    "mode": "autonomous",
    "uptime": 28430,
    "lastMaintenance": "2025-12-20T08:00:00Z",
    "nextMaintenance": "2026-01-20T08:00:00Z"
  },

  "power": {
    "batteryLevel": 85,
    "batteryHealth": 98,
    "chargingStatus": "discharging",
    "estimatedRuntime": 240,
    "powerConsumption": 2.3
  },

  "health": {
    "overall": "optimal",
    "temperature": 42.5,
    "vibration": 0.05,
    "errors": [],
    "warnings": []
  },

  "performance": {
    "tasksCompleted": 240,
    "tasksRemaining": 60,
    "efficiency": 0.92,
    "accuracy": 0.98,
    "productivity": 580
  }
}
```

### 2.2 Robot Type-Specific Extensions

#### Bricklaying Robot
```json
{
  "bricklaying": {
    "bricksLaidToday": 4800,
    "bricksPerHour": 600,
    "mortarLevel": 75,
    "bondPattern": "running-bond",
    "wallHeight": 2.4,
    "wallLength": 12.5,
    "precision": 0.3
  }
}
```

#### 3D Printing Robot
```json
{
  "printing": {
    "materialType": "concrete-mix-a",
    "materialRemaining": 250,
    "layerHeight": 20,
    "printSpeed": 1.5,
    "currentLayer": 45,
    "totalLayers": 120,
    "nozzleTemperature": 25,
    "reinforcementActive": true
  }
}
```

#### Demolition Robot
```json
{
  "demolition": {
    "toolType": "hydraulic-breaker",
    "toolPressure": 180,
    "debrisRemoved": 12500,
    "safetyRadius": 5.0,
    "structuralMonitoring": true,
    "dustSuppression": "active"
  }
}
```

#### Inspection Drone
```json
{
  "flight": {
    "altitude": 25.5,
    "speed": 3.2,
    "heading": 135,
    "batteryFlightTime": 28,
    "windSpeed": 4.5,
    "gpsAccuracy": 0.8,
    "obstacleAvoidance": "active",
    "currentWaypoint": 5,
    "totalWaypoints": 12
  },
  "imaging": {
    "photosToday": 342,
    "videoRecording": true,
    "thermalImaging": false,
    "lidarActive": true,
    "storageRemaining": 128
  }
}
```

#### Autonomous Excavator
```json
{
  "excavation": {
    "bucketCapacity": 0.8,
    "materialMoved": 145,
    "targetDepth": 3.5,
    "currentDepth": 2.8,
    "gradeAccuracy": 0.01,
    "terrainStability": "stable",
    "fuelLevel": 65,
    "hydraulicPressure": 210
  }
}
```

#### Worker Exoskeleton
```json
{
  "exoskeleton": {
    "assistLevel": 75,
    "liftCapacity": 45,
    "currentLoad": 32,
    "bodyPartSupported": "back-and-shoulders",
    "wearerHeartRate": 85,
    "fatigueIndex": 0.35,
    "safetyLockEngaged": false
  }
}
```

---

## 3. Sensor Data Format

### 3.1 LiDAR Data
```json
{
  "@type": "LiDARScan",
  "timestamp": "2025-12-25T10:30:00.123Z",
  "robotId": "did:wia:robot:CR-BRICK-001",
  "sensor": {
    "id": "lidar-1",
    "model": "Velodyne VLP-16",
    "range": 100,
    "resolution": 0.01,
    "scanRate": 10
  },
  "pointCloud": {
    "format": "ply",
    "points": 65536,
    "dataUrl": "s3://scans/2025-12-25/scan-10-30-00.ply",
    "checksum": "sha256:7d8a9b..."
  },
  "detectedObjects": [
    {
      "type": "worker",
      "distance": 2.5,
      "position": [1.2, 2.3, 0.0],
      "velocity": [0.5, 0.3, 0.0],
      "confidence": 0.95
    }
  ]
}
```

### 3.2 Camera Data
```json
{
  "@type": "CameraCapture",
  "timestamp": "2025-12-25T10:30:00.456Z",
  "robotId": "did:wia:robot:DRONE-INSP-127",
  "camera": {
    "id": "cam-front",
    "resolution": "3840x2160",
    "fps": 60,
    "lens": "wide-angle",
    "exposure": "auto"
  },
  "image": {
    "format": "jpg",
    "url": "s3://images/2025-12-25/img-10-30-00.jpg",
    "thumbnailUrl": "s3://images/2025-12-25/thumb-10-30-00.jpg",
    "checksum": "sha256:4f2c1e..."
  },
  "aiAnalysis": {
    "defectsDetected": [
      {
        "type": "crack",
        "location": [1024, 768],
        "severity": "minor",
        "confidence": 0.87
      }
    ],
    "qualityScore": 0.94
  }
}
```

### 3.3 Environmental Sensors
```json
{
  "@type": "EnvironmentalData",
  "timestamp": "2025-12-25T10:30:00Z",
  "robotId": "did:wia:robot:CR-DEMO-055",
  "environment": {
    "temperature": 24.5,
    "humidity": 45,
    "airPressure": 1013.25,
    "airQuality": {
      "pm25": 12,
      "pm10": 28,
      "co2": 450,
      "voc": 120,
      "rating": "good"
    },
    "noise": {
      "level": 85,
      "frequency": "low",
      "source": "machinery"
    },
    "lighting": {
      "lux": 450,
      "spectrum": "daylight"
    }
  }
}
```

---

## 4. Work Progress Data

### 4.1 Task Execution Record
```json
{
  "@context": "https://wia.org/schemas/work-record/v1",
  "@type": "ConstructionTaskRecord",
  "recordId": "task-2025-12-25-0001",
  "timestamp": "2025-12-25T16:00:00Z",

  "robot": {
    "id": "did:wia:robot:CR-BRICK-001",
    "operator": "did:wia:worker:john-smith-742"
  },

  "project": {
    "id": "PROJ-2025-001",
    "name": "Smart City Tower",
    "contractor": "BuildTech Corporation",
    "location": "Seoul, South Korea"
  },

  "workSession": {
    "startTime": "2025-12-25T08:00:00Z",
    "endTime": "2025-12-25T16:00:00Z",
    "duration": 28800,
    "breaks": [
      { "start": "2025-12-25T12:00:00Z", "end": "2025-12-25T13:00:00Z" }
    ]
  },

  "tasks": [
    {
      "taskId": "wall-5A-section-3",
      "type": "bricklaying",
      "bimElement": "wall-5A-3",
      "status": "completed",
      "startTime": "2025-12-25T08:15:00Z",
      "endTime": "2025-12-25T11:45:00Z",
      "specifications": {
        "brickType": "red-clay-standard",
        "bondPattern": "running-bond",
        "mortarType": "type-n",
        "dimensions": {
          "length": 6.0,
          "height": 2.4,
          "thickness": 0.2
        }
      },
      "performance": {
        "bricksLaid": 2400,
        "accuracy": 0.98,
        "defects": 8,
        "rework": 1
      },
      "quality": {
        "inspectionStatus": "passed",
        "inspector": "did:wia:inspector:jane-doe-531",
        "inspectionTime": "2025-12-25T12:30:00Z",
        "deviations": [
          {
            "type": "alignment",
            "location": "row-45",
            "magnitude": 0.7,
            "acceptable": true
          }
        ]
      }
    }
  ],

  "verification": {
    "digitalSignature": "0x7d8a9b...",
    "blockchainAnchor": {
      "network": "ethereum-polygon",
      "txHash": "0x4f2c1e...",
      "blockNumber": 45892341,
      "timestamp": "2025-12-25T17:00:00Z"
    }
  }
}
```

### 4.2 Progress Summary
```json
{
  "@type": "ProgressSummary",
  "projectId": "PROJ-2025-001",
  "reportDate": "2025-12-25",

  "overall": {
    "percentComplete": 67.5,
    "daysElapsed": 145,
    "daysRemaining": 70,
    "onSchedule": true,
    "budgetStatus": "under"
  },

  "robotFleet": {
    "totalRobots": 14,
    "activeToday": 12,
    "tasksCompleted": 1247,
    "efficiency": 0.89,
    "safetyIncidents": 0
  },

  "milestones": [
    {
      "name": "Foundation Complete",
      "targetDate": "2025-11-30",
      "actualDate": "2025-11-28",
      "status": "completed-early"
    },
    {
      "name": "Floor 5 Complete",
      "targetDate": "2025-12-31",
      "status": "on-track",
      "percentComplete": 87
    }
  ]
}
```

---

## 5. BIM Integration Data

### 5.1 BIM Element Mapping
```json
{
  "@type": "BIMMapping",
  "projectId": "PROJ-2025-001",
  "bimModel": {
    "software": "Autodesk Revit",
    "version": "2025.1",
    "fileName": "smart-city-tower.rvt",
    "lastSync": "2025-12-25T09:00:00Z"
  },

  "elementMapping": [
    {
      "bimElementId": "wall-5A-3",
      "robotTask": "task-2025-12-25-0001",
      "status": "completed",
      "asBuilt": {
        "position": [126.9780, 37.5665, 15.0],
        "dimensions": {
          "length": 6.0,
          "height": 2.4,
          "thickness": 0.2
        },
        "deviationFromDesign": 0.003
      },
      "materials": {
        "bricks": 2400,
        "mortar": 120
      },
      "qualityData": {
        "strength": "verified",
        "alignment": 0.98,
        "finish": "acceptable"
      }
    }
  ]
}
```

### 5.2 As-Built Documentation
```json
{
  "@type": "AsBuiltRecord",
  "elementId": "wall-5A-3",
  "completionDate": "2025-12-25",

  "design": {
    "bimSource": "revit-element-4523",
    "specifications": {
      "length": 6.0,
      "height": 2.4,
      "thickness": 0.2
    }
  },

  "actual": {
    "measurements": {
      "length": 6.002,
      "height": 2.398,
      "thickness": 0.201
    },
    "deviation": 0.003,
    "acceptable": true
  },

  "documentation": {
    "photos": [
      "s3://asbuilt/wall-5A-3-view1.jpg",
      "s3://asbuilt/wall-5A-3-view2.jpg"
    ],
    "3dScan": "s3://asbuilt/wall-5A-3-scan.ply",
    "inspectionReport": "s3://reports/wall-5A-3-inspection.pdf"
  }
}
```

---

## 6. Safety Event Data

### 6.1 Hazard Detection Event
```json
{
  "@type": "SafetyEvent",
  "eventId": "safety-2025-12-25-0042",
  "timestamp": "2025-12-25T14:23:15Z",
  "severity": "warning",

  "robot": {
    "id": "did:wia:robot:CR-BRICK-001",
    "location": [126.9780, 37.5665, 15.0]
  },

  "hazard": {
    "type": "worker-proximity",
    "distance": 1.8,
    "safetyThreshold": 2.0,
    "detectionMethod": "lidar"
  },

  "response": {
    "action": "pause-operation",
    "responseTime": 0.15,
    "notified": [
      "did:wia:worker:john-smith-742",
      "did:wia:supervisor:sarah-jones-123"
    ]
  },

  "resolution": {
    "resolvedAt": "2025-12-25T14:24:00Z",
    "action": "worker-cleared-area",
    "resumedOperation": "2025-12-25T14:24:10Z"
  }
}
```

### 6.2 Incident Report
```json
{
  "@type": "IncidentReport",
  "incidentId": "incident-2025-12-25-0001",
  "timestamp": "2025-12-25T10:15:00Z",
  "severity": "minor",
  "status": "resolved",

  "incident": {
    "type": "equipment-malfunction",
    "description": "Bricklaying robot mortar pump failure",
    "robot": "did:wia:robot:CR-BRICK-001",
    "location": "Floor 5, Zone A-3"
  },

  "impact": {
    "workStopped": true,
    "duration": 45,
    "injuries": 0,
    "propertyDamage": "none"
  },

  "response": {
    "reportedBy": "did:wia:worker:john-smith-742",
    "respondedBy": "did:wia:technician:mike-wilson-891",
    "actions": [
      "Robot shutdown and isolation",
      "Mortar pump replacement",
      "System testing and verification"
    ]
  },

  "rootCause": {
    "analysis": "Pump seal wear from extended operation",
    "preventiveMeasures": [
      "Reduce pump maintenance interval from 500h to 400h",
      "Add seal wear monitoring to telemetry"
    ]
  }
}
```

---

## 7. Data Validation

### 7.1 JSON Schema

All data formats MUST validate against JSON Schema definitions:

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "$id": "https://wia.org/schemas/construction-robot/robot-status-v1.json",
  "title": "Construction Robot Status",
  "type": "object",
  "required": ["@context", "@type", "robotId", "timestamp", "location", "operational"],
  "properties": {
    "@context": {
      "type": "string",
      "format": "uri"
    },
    "@type": {
      "type": "string",
      "enum": ["RobotStatus"]
    },
    "robotId": {
      "type": "string",
      "pattern": "^did:wia:robot:CR-[A-Z0-9]+-[0-9]{3}$"
    },
    "timestamp": {
      "type": "string",
      "format": "date-time"
    }
  }
}
```

### 7.2 Data Integrity

**Checksums:** All binary data (images, scans) MUST include SHA-256 checksums

**Signatures:** All work records MUST be digitally signed

**Timestamps:** All events MUST use ISO 8601 format with timezone

**Validation:** Data MUST pass schema validation before transmission

---

## 8. Data Storage and Retention

### 8.1 Storage Requirements

**Real-time Telemetry:** 7 days hot storage, 90 days warm storage
**Work Records:** 7 years (legal requirement)
**Safety Events:** Permanent retention
**BIM Data:** Project lifetime + 10 years
**Images/Scans:** Project lifetime + 5 years

### 8.2 Data Privacy

**Worker Data:** Anonymize personal information in archived records
**Proprietary Data:** Contractor data remains confidential
**Public Data:** Safety statistics may be aggregated and published

---

## 9. Next Steps

**Phase 3:** Communication protocols for robot control and coordination
**Phase 4:** System integration with BIM, site management, and planning tools

---

**© 2025 SmileStory Inc. / WIA**
**弘益人間 (홍익인간) · Benefit All Humanity**
