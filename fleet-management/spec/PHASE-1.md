# WIA-AUTO-024: Fleet Management Standard
## PHASE 1 - Data Format Standardization

**Version:** 1.0.0  
**Status:** Active  
**Category:** Automotive / Fleet Management  
**Last Updated:** January 2025

---

## Overview

Phase 1 establishes comprehensive data format standards for fleet management systems, ensuring interoperability between different platforms, devices, and services. Standardized data formats enable seamless integration, data exchange, and analytics across the entire fleet management ecosystem.

**弘益人間 (Benefit All Humanity)** - Universal data standards promote collaboration, reduce integration costs, and enable innovation across the fleet management industry.

---

## 1. Vehicle Telemetry Data Format

### 1.1 Position Update Schema

```json
{
  "$schema": "https://wiastandards.com/schemas/fleet-management/position-v1.json",
  "vehicleId": "string (required)",
  "timestamp": "ISO 8601 datetime (required)",
  "location": {
    "latitude": "number (required, -90 to 90)",
    "longitude": "number (required, -180 to 180)",
    "altitude": "number (optional, meters)",
    "accuracy": "number (optional, meters)",
    "heading": "number (optional, 0-360 degrees)"
  },
  "speed": {
    "value": "number (required)",
    "unit": "string (km/h, mph, m/s)"
  },
  "gpsStatus": {
    "satelliteCount": "integer (optional)",
    "signalQuality": "string (poor, fair, good, excellent)",
    "fixType": "string (2D, 3D, DGPS)"
  }
}
```

### 1.2 Vehicle Diagnostics Schema

```json
{
  "$schema": "https://wiastandards.com/schemas/fleet-management/diagnostics-v1.json",
  "vehicleId": "string (required)",
  "timestamp": "ISO 8601 datetime (required)",
  "engine": {
    "rpm": "integer (revolutions per minute)",
    "temperature": "number (celsius)",
    "load": "number (percentage 0-100)",
    "runtime": "integer (hours)"
  },
  "fuel": {
    "level": "number (percentage 0-100)",
    "consumption": "number (L/100km or MPG)",
    "pressure": "number (kPa)",
    "type": "string (gasoline, diesel, electric, hybrid)"
  },
  "battery": {
    "voltage": "number (volts)",
    "current": "number (amperes)",
    "stateOfCharge": "number (percentage 0-100)"
  },
  "odometer": {
    "value": "number",
    "unit": "string (km, miles)"
  },
  "diagnostics": {
    "troubleCodes": ["array of DTC codes"],
    "status": "string (normal, warning, critical)",
    "checkEngine": "boolean",
    "lastServiceDate": "ISO 8601 date"
  }
}
```

### 1.3 Driving Event Schema

```json
{
  "$schema": "https://wiastandards.com/schemas/fleet-management/event-v1.json",
  "vehicleId": "string (required)",
  "driverId": "string (optional)",
  "timestamp": "ISO 8601 datetime (required)",
  "eventType": "string (required)",
  "severity": "string (low, medium, high, critical)",
  "location": {
    "latitude": "number",
    "longitude": "number"
  },
  "telemetry": {
    "speed": "number (km/h)",
    "acceleration": "number (m/s²)",
    "deceleration": "number (m/s²)",
    "lateralG": "number (g-force)",
    "duration": "number (seconds)"
  },
  "metadata": {
    "description": "string",
    "tags": ["array of strings"],
    "videoUrl": "string (optional)"
  }
}
```

**Standard Event Types:**
- `harsh_braking` - Deceleration > 0.4g
- `harsh_acceleration` - Acceleration > 0.35g
- `sharp_turn` - Lateral force > 0.4g
- `speeding` - Speed exceeds limit by threshold
- `idling` - Engine running while stationary > threshold
- `impact` - Collision or impact detected
- `emergency_braking` - ABS activation or extreme deceleration
- `seatbelt_violation` - Seatbelt not fastened while moving

---

## 2. Sensor Data Formats

### 2.1 Temperature Sensor Data

```json
{
  "$schema": "https://wiastandards.com/schemas/fleet-management/temperature-v1.json",
  "vehicleId": "string (required)",
  "timestamp": "ISO 8601 datetime (required)",
  "sensors": [
    {
      "sensorId": "string",
      "location": "string (cargo_zone_1, refrigeration_unit, etc.)",
      "temperature": {
        "value": "number",
        "unit": "string (celsius, fahrenheit)"
      },
      "setpoint": "number (target temperature)",
      "status": "string (normal, warning, alarm)"
    }
  ],
  "alerts": [
    {
      "alertId": "string",
      "type": "string (high_temp, low_temp, sensor_failure)",
      "timestamp": "ISO 8601 datetime",
      "acknowledged": "boolean"
    }
  ]
}
```

### 2.2 Door/Cargo Sensor Data

```json
{
  "$schema": "https://wiastandards.com/schemas/fleet-management/door-v1.json",
  "vehicleId": "string (required)",
  "timestamp": "ISO 8601 datetime (required)",
  "doors": [
    {
      "doorId": "string",
      "location": "string (driver, passenger, cargo_rear, cargo_side)",
      "status": "string (open, closed)",
      "lockStatus": "string (locked, unlocked)",
      "openDuration": "number (seconds since opened)"
    }
  ],
  "events": [
    {
      "doorId": "string",
      "action": "string (opened, closed, locked, unlocked)",
      "timestamp": "ISO 8601 datetime",
      "location": {
        "latitude": "number",
        "longitude": "number"
      }
    }
  ]
}
```

---

## 3. Driver Data Formats

### 3.1 Driver Profile Schema

```json
{
  "$schema": "https://wiastandards.com/schemas/fleet-management/driver-v1.json",
  "driverId": "string (required)",
  "personalInfo": {
    "firstName": "string",
    "lastName": "string",
    "email": "string",
    "phone": "string",
    "employeeId": "string"
  },
  "license": {
    "number": "string",
    "class": "string (A, B, C, CDL, etc.)",
    "issueDate": "ISO 8601 date",
    "expiryDate": "ISO 8601 date",
    "endorsements": ["array of strings"]
  },
  "certifications": [
    {
      "type": "string (hazmat, forklift, etc.)",
      "number": "string",
      "expiryDate": "ISO 8601 date"
    }
  ],
  "safetyScore": {
    "overall": "number (0-100)",
    "components": {
      "acceleration": "number (0-100)",
      "braking": "number (0-100)",
      "cornering": "number (0-100)",
      "speeding": "number (0-100)",
      "distraction": "number (0-100)"
    },
    "lastUpdated": "ISO 8601 datetime"
  }
}
```

### 3.2 Driver Hours of Service

```json
{
  "$schema": "https://wiastandards.com/schemas/fleet-management/hos-v1.json",
  "driverId": "string (required)",
  "date": "ISO 8601 date (required)",
  "dutyStatus": [
    {
      "status": "string (off_duty, sleeper, driving, on_duty)",
      "startTime": "ISO 8601 datetime",
      "endTime": "ISO 8601 datetime",
      "location": "string",
      "odometer": "number"
    }
  ],
  "summary": {
    "drivingHours": "number",
    "onDutyHours": "number",
    "remainingDriving": "number",
    "remainingOnDuty": "number",
    "violations": ["array of violation objects"]
  }
}
```

---

## 4. Route and Trip Data

### 4.1 Route Definition Schema

```json
{
  "$schema": "https://wiastandards.com/schemas/fleet-management/route-v1.json",
  "routeId": "string (required)",
  "vehicleId": "string (required)",
  "driverId": "string (optional)",
  "plannedDate": "ISO 8601 date",
  "status": "string (planned, in_progress, completed, cancelled)",
  "waypoints": [
    {
      "waypointId": "string",
      "sequence": "integer",
      "type": "string (pickup, delivery, depot, waypoint)",
      "location": {
        "latitude": "number",
        "longitude": "number",
        "address": "string"
      },
      "timeWindow": {
        "start": "ISO 8601 datetime",
        "end": "ISO 8601 datetime"
      },
      "serviceTime": "number (minutes)",
      "cargo": {
        "action": "string (load, unload)",
        "weight": "number (kg)",
        "volume": "number (cubic meters)",
        "units": "integer"
      }
    }
  ],
  "optimization": {
    "algorithm": "string",
    "objectives": ["array of strings"],
    "constraints": ["array of constraint objects"]
  }
}
```

### 4.2 Trip Record Schema

```json
{
  "$schema": "https://wiastandards.com/schemas/fleet-management/trip-v1.json",
  "tripId": "string (required)",
  "vehicleId": "string (required)",
  "driverId": "string (required)",
  "routeId": "string (optional)",
  "startTime": "ISO 8601 datetime",
  "endTime": "ISO 8601 datetime",
  "startLocation": {
    "latitude": "number",
    "longitude": "number",
    "odometer": "number"
  },
  "endLocation": {
    "latitude": "number",
    "longitude": "number",
    "odometer": "number"
  },
  "summary": {
    "distance": "number (km)",
    "duration": "number (minutes)",
    "fuelConsumed": "number (liters)",
    "avgSpeed": "number (km/h)",
    "maxSpeed": "number (km/h)",
    "idleTime": "number (minutes)",
    "stopCount": "integer"
  },
  "events": [
    {
      "eventType": "string",
      "timestamp": "ISO 8601 datetime",
      "severity": "string"
    }
  ]
}
```

---

## 5. Maintenance Data Formats

### 5.1 Maintenance Record Schema

```json
{
  "$schema": "https://wiastandards.com/schemas/fleet-management/maintenance-v1.json",
  "maintenanceId": "string (required)",
  "vehicleId": "string (required)",
  "type": "string (preventive, corrective, predictive)",
  "category": "string (oil_change, tire_rotation, brake_service, etc.)",
  "scheduledDate": "ISO 8601 date",
  "completedDate": "ISO 8601 date",
  "status": "string (scheduled, in_progress, completed, cancelled)",
  "odometer": "number (at time of service)",
  "workOrders": [
    {
      "workOrderId": "string",
      "description": "string",
      "technician": "string",
      "parts": [
        {
          "partNumber": "string",
          "description": "string",
          "quantity": "integer",
          "cost": "number"
        }
      ],
      "labor": {
        "hours": "number",
        "rate": "number",
        "cost": "number"
      }
    }
  ],
  "totalCost": "number",
  "nextServiceDue": {
    "date": "ISO 8601 date",
    "odometer": "number"
  }
}
```

---

## 6. Fleet Analytics Data

### 6.1 Fleet Performance Metrics

```json
{
  "$schema": "https://wiastandards.com/schemas/fleet-management/metrics-v1.json",
  "fleetId": "string (required)",
  "period": {
    "start": "ISO 8601 datetime",
    "end": "ISO 8601 datetime"
  },
  "efficiency": {
    "fleetEfficiencyScore": "number (0-100)",
    "avgUtilization": "number (percentage)",
    "avgFuelEconomy": "number (km/L or MPG)",
    "costPerKm": "number"
  },
  "safety": {
    "totalEvents": "integer",
    "accidentRate": "number (per million km)",
    "avgDriverScore": "number (0-100)",
    "violationCount": "integer"
  },
  "operations": {
    "totalDistance": "number (km)",
    "totalTrips": "integer",
    "avgTripDistance": "number (km)",
    "onTimeDeliveryRate": "number (percentage)",
    "vehicleDowntime": "number (hours)"
  },
  "costs": {
    "totalOperatingCost": "number",
    "fuelCost": "number",
    "maintenanceCost": "number",
    "laborCost": "number"
  }
}
```

---

## 7. API Response Formats

### 7.1 Success Response

```json
{
  "status": "success",
  "statusCode": 200,
  "timestamp": "ISO 8601 datetime",
  "data": {
    // Response payload
  },
  "metadata": {
    "requestId": "string",
    "responseTime": "number (milliseconds)"
  }
}
```

### 7.2 Error Response

```json
{
  "status": "error",
  "statusCode": "integer (400-599)",
  "timestamp": "ISO 8601 datetime",
  "error": {
    "code": "string (error code)",
    "message": "string (human-readable message)",
    "details": ["array of specific error details"]
  },
  "metadata": {
    "requestId": "string",
    "documentation": "URL to error documentation"
  }
}
```

---

## 8. Data Quality Requirements

### 8.1 Validation Rules

- All required fields must be present and non-null
- Timestamps must be in ISO 8601 format with timezone
- Numeric values must be within specified ranges
- Enumerations must match defined values
- GPS coordinates must be valid (lat: -90 to 90, lon: -180 to 180)
- Vehicle IDs must be unique within a fleet
- Driver IDs must be unique within an organization

### 8.2 Data Retention

- Real-time telemetry: 90 days detailed, 2 years aggregated
- Trip records: 5 years
- Maintenance records: Vehicle lifetime + 3 years
- Driver records: Employment period + 7 years
- Compliance data: Per regulatory requirements (typically 3-7 years)

---

## 9. Data Exchange Protocols

### 9.1 Supported Formats

- **JSON**: Primary format for REST APIs
- **Protocol Buffers**: High-performance binary format for high-volume data
- **CSV**: Batch data import/export
- **XML**: Legacy system compatibility

### 9.2 Compression

- GZIP compression recommended for payloads > 1KB
- Brotli compression supported for modern clients

### 9.3 Encoding

- UTF-8 encoding for all text data
- Base64 encoding for binary data in JSON

---

## 10. Implementation Guidelines

### 10.1 Backward Compatibility

- Minor version updates must maintain backward compatibility
- Deprecated fields must be supported for minimum 12 months
- API version in URL path: `/api/v1/...`

### 10.2 Extensibility

- Custom fields allowed in `extensions` object
- Vendor-specific data prefixed with vendor identifier
- Schema versioning for future enhancements

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA - World Certification Industry Association*  
*© 2025 SmileStory Inc. / WIA*  
*MIT License*
