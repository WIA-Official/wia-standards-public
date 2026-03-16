# WIA Elevator System Standard
## Phase 4: System Integration Specification v1.0

**Status:** APPROVED  
**Date:** 2025-12-26  
**弘益人間** · Benefit All Humanity

---

## 1. Introduction

Phase 4 defines integration with Building Management Systems (BMS), access control, fire alarms, cloud platforms, and third-party applications. This enables elevators to participate in the broader smart building ecosystem.

### 1.1 Scope

- BMS integration (BACnet, Modbus)
- Access control systems
- Fire alarm coordination
- Cloud platform adapters
- Energy management
- Predictive maintenance
- Third-party app ecosystem

---

## 2. BMS Integration

### 2.1 BACnet Integration

**Elevator as BACnet Device:**
```
Device Instance: 2025001
Device Name: "ELV-001"
Device Type: "Elevator Controller"
Protocol Services: ReadProperty, WriteProperty, Subscribe-COV
```

**BACnet Objects:**
- `ANALOG_VALUE` - Current floor, speed, load weight
- `BINARY_VALUE` - Door status, direction
- `MULTI_STATE_VALUE` - Elevator state (IDLE, MOVING_UP, MOVING_DOWN)
- `NOTIFICATION_CLASS` - Alarms and events

**Example BACnet Read:**
```
ReadProperty Request:
  Object: ANALOG_VALUE:1 (Current Floor)
  Property: PRESENT_VALUE

Response:
  Present Value: 5.0
```

### 2.2 Modbus Integration

**Modbus Register Map:**

| Register | Type    | Description         | Unit | Access |
|----------|---------|---------------------|------|--------|
| 40001    | Holding | Current Floor       | -    | R      |
| 40002    | Holding | Direction (0/1/2)   | -    | R      |
| 40003    | Holding | Door Status (0/1/2) | -    | R      |
| 40004    | Holding | Load Weight         | kg   | R      |
| 40005    | Holding | Target Floor        | -    | R/W    |
| 40006    | Holding | Mode (0=Auto, 1=Maint) | - | R/W    |

**Modbus Request Example:**
```
Function Code: 03 (Read Holding Registers)
Starting Address: 40001
Quantity: 6

Response:
  [5, 1, 0, 640, 10, 0]
  (Floor 5, Moving UP, Doors OPEN, 640kg load, Target floor 10, Auto mode)
```

---

## 3. Access Control Integration

### 3.1 OSDP Protocol

**Elevator as OSDP Peripheral Device:**
```json
{
  "protocol": "OSDP",
  "version": "2.2",
  "address": 1,
  "cardReaderIntegration": {
    "enabled": true,
    "location": "ELEVATOR_CAR",
    "formats": ["WIEGAND-26", "WIEGAND-37", "ABA-TRACK2"]
  }
}
```

**Badge Read Event:**
```json
{
  "event": "CARD_READ",
  "elevatorId": "ELV-001",
  "cardData": "12345678",
  "timestamp": "2025-12-26T10:30:00Z",
  "action": "REQUEST_FLOOR_ACCESS",
  "authorization": {
    "userId": "USER-001",
    "authorizedFloors": [1, 5, 10, 15],
    "accessGranted": true
  }
}
```

### 3.2 Floor Access Control

**Access rules:**
```json
{
  "userId": "USER-001",
  "buildingId": "BLD-2025",
  "accessRules": {
    "timeRestrictions": {
      "weekdays": "06:00-20:00",
      "weekends": "DENIED"
    },
    "floorAccess": [
      {"floor": 1, "access": "ALWAYS"},
      {"floor": 5, "access": "BUSINESS_HOURS"},
      {"floor": 10, "access": "ALWAYS"},
      {"floor": 15, "access": "ESCORT_REQUIRED"}
    ]
  }
}
```

---

## 4. Fire Alarm System Integration

### 4.1 Fire Alarm Protocol

**Interface:** Digital I/O + MQTT

**Fire alarm signal:**
```json
{
  "type": "FIRE_ALARM_SIGNAL",
  "buildingId": "BLD-2025",
  "timestamp": "2025-12-26T10:45:00Z",
  "alarmPanel": "PANEL-ZONE-5",
  "alarmType": "SMOKE_DETECTOR",
  "floor": 5,
  "elevatorResponse": {
    "allElevators": [
      {
        "elevatorId": "ELV-001",
        "action": "RECALL_TO_FLOOR_1",
        "status": "EXECUTING"
      },
      {
        "elevatorId": "ELV-002",
        "action": "RECALL_TO_FLOOR_1",
        "status": "EXECUTING"
      }
    ]
  }
}
```

### 4.2 Firefighter Override

```json
{
  "type": "FIREFIGHTER_CONTROL",
  "elevatorId": "ELV-001",
  "authentication": {
    "method": "DIGITAL_KEY",
    "keyId": "FD-MASTER-KEY-001"
  },
  "phase": "PHASE_2_MANUAL_CONTROL",
  "permissions": {
    "manualFloorSelection": true,
    "doorControl": true,
    "bypassSafety": ["DOOR_TIMER_ONLY"]
  }
}
```

---

## 5. Cloud Platform Integration

### 5.1 AWS IoT Core

**Thing Configuration:**
```json
{
  "thingName": "ELV-001-BLD-2025",
  "thingType": "WIA-Elevator",
  "attributes": {
    "buildingId": "BLD-2025",
    "elevatorId": "ELV-001",
    "manufacturer": "WIA-Compliant-Mfg",
    "model": "WIA-ELEV-2025",
    "installDate": "2025-01-15"
  }
}
```

**MQTT Topics:**
```
$aws/things/ELV-001-BLD-2025/shadow/update
$aws/things/ELV-001-BLD-2025/shadow/get
wia/elevator/telemetry
wia/elevator/events
```

**Device Shadow:**
```json
{
  "state": {
    "reported": {
      "currentFloor": 5,
      "direction": "UP",
      "doorStatus": "CLOSED",
      "loadWeight": 640,
      "timestamp": "2025-12-26T10:30:00Z"
    },
    "desired": {
      "targetFloor": 10
    }
  }
}
```

### 5.2 Azure IoT Hub

**Device Twin:**
```json
{
  "deviceId": "ELV-001-BLD-2025",
  "properties": {
    "desired": {
      "maintenanceMode": false,
      "telemetryInterval": 60
    },
    "reported": {
      "status": {
        "currentFloor": 5,
        "direction": "UP"
      },
      "telemetry": {
        "loadWeight": 640,
        "temperature": 22.5
      }
    }
  }
}
```

### 5.3 Google Cloud IoT Core

**Device Configuration:**
```json
{
  "id": "ELV-001-BLD-2025",
  "name": "projects/wia-elevators/locations/us-central1/registries/building-2025/devices/ELV-001",
  "credentials": [
    {
      "publicKey": {
        "format": "RSA_X509_PEM",
        "key": "-----BEGIN CERTIFICATE-----\n..."
      }
    }
  ],
  "metadata": {
    "buildingId": "BLD-2025",
    "elevatorId": "ELV-001"
  }
}
```

---

## 6. Energy Management Integration

### 6.1 Energy Dashboard Integration

**Energy API endpoint:**
```
POST /api/v1/building/energy/elevator

{
  "elevatorId": "ELV-001",
  "timestamp": "2025-12-26T10:00:00Z",
  "energyData": {
    "consumed": 42.5,
    "regenerated": 12.8,
    "net": 29.7,
    "efficiency": 30.1,
    "unit": "kWh"
  }
}
```

### 6.2 Demand Response

**Grid signal integration:**
```json
{
  "type": "DEMAND_RESPONSE_EVENT",
  "buildingId": "BLD-2025",
  "timestamp": "2025-12-26T14:00:00Z",
  "event": "PEAK_DEMAND_REDUCTION",
  "duration": 3600,
  "elevatorStrategy": {
    "reduceSpeed": 20,
    "groupDispatching": "AGGRESSIVE",
    "parkElevators": ["ELV-003", "ELV-004"],
    "estimatedReduction": "15kW"
  }
}
```

---

## 7. Predictive Maintenance Platform

### 7.1 ML Model Integration

**Training data export:**
```json
{
  "elevatorId": "ELV-001",
  "exportPeriod": {
    "start": "2025-01-01T00:00:00Z",
    "end": "2025-12-26T23:59:59Z"
  },
  "features": {
    "vibrationData": [...],
    "temperatureData": [...],
    "loadCycles": [...],
    "doorCycles": [...],
    "tripCount": 182394
  },
  "labels": {
    "brakeReplacement": "2025-06-15",
    "ropeReplacement": null,
    "motorBearingFailure": null
  }
}
```

### 7.2 Prediction Response

```json
{
  "elevatorId": "ELV-001",
  "timestamp": "2025-12-26T10:00:00Z",
  "predictions": [
    {
      "component": "BRAKE_LINING",
      "prediction": "REPLACEMENT_NEEDED",
      "confidence": 0.92,
      "estimatedDays": 18,
      "recommendedAction": "SCHEDULE_MAINTENANCE"
    },
    {
      "component": "DOOR_OPERATOR",
      "prediction": "NORMAL",
      "confidence": 0.88,
      "estimatedDays": 120
    }
  ]
}
```

---

## 8. Third-Party Application Ecosystem

### 8.1 App Marketplace

WIA-compliant elevators support third-party apps through standardized APIs:

**App categories:**
- Analytics & reporting
- Predictive maintenance
- Traffic optimization
- Mobile passenger apps
- Facility management
- Energy optimization

### 8.2 App Integration

**OAuth 2.0 app registration:**
```json
{
  "appId": "app_abc123",
  "appName": "ElevatorAnalytics Pro",
  "developer": "Analytics Co.",
  "permissions": [
    "elevator:read",
    "building:read",
    "telemetry:read"
  ],
  "webhookUrl": "https://app.analytics.com/webhooks/elevator",
  "redirectUri": "https://app.analytics.com/oauth/callback"
}
```

---

## 9. Data Privacy & Multi-Tenancy

### 9.1 Data Isolation

```json
{
  "tenantId": "TENANT-001",
  "buildingIds": ["BLD-2025", "BLD-2026"],
  "elevatorIds": ["ELV-001", "ELV-002", "ELV-003"],
  "dataAccess": {
    "telemetry": "FULL",
    "maintenance": "FULL",
    "crossTenantData": "DENIED"
  }
}
```

### 9.2 GDPR Compliance

**Data retention policy:**
```json
{
  "elevatorTelemetry": {
    "retention": "90 days",
    "aggregatedRetention": "7 years",
    "anonymization": "REQUIRED"
  },
  "accessLogs": {
    "retention": "30 days",
    "encryption": "AES-256"
  },
  "maintenanceRecords": {
    "retention": "LIFETIME"
  }
}
```

---

## 10. Compliance Checklist

Phase 4 compliance requires:
1. ✅ BACnet OR Modbus integration
2. ✅ Access control integration
3. ✅ Fire alarm coordination
4. ✅ At least one cloud platform adapter
5. ✅ Energy reporting API
6. ✅ Third-party API support
7. ✅ Data privacy compliance

---

**© 2025 WIA - World Certification Industry Association**  
**MIT License**  
**弘益人間 · Benefit All Humanity**
