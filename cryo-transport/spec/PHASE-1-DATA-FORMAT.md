# WIA Cryo-Transport Data Format Standard
## Phase 1 Specification

---

**Version**: 1.0.0
**Status**: Complete
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #06B6D4 (Cyan)

---

## Table of Contents

1. [Overview](#overview)
2. [Transport Manifest Schema](#transport-manifest-schema)
3. [Container Monitoring Data](#container-monitoring-data)
4. [Chain of Custody Records](#chain-of-custody-records)
5. [GPS Telemetry Format](#gps-telemetry-format)
6. [Incident Report Templates](#incident-report-templates)
7. [Quality Metrics](#quality-metrics)

---

## Overview

### 1.1 Purpose

The WIA Cryo-Transport Data Format Standard (Phase 1) defines JSON-based schemas for all data generated, transmitted, and stored during cryogenic transport operations.

**Core Objectives**:
- Standardize transport data across all providers
- Enable real-time monitoring and alerting
- Support regulatory compliance and audit
- Facilitate inter-facility data exchange

### 1.2 Design Principles

1. **JSON-First**: All data in JSON format for universal compatibility
2. **Self-Describing**: Schemas include metadata and validation rules
3. **Extensible**: Custom fields supported via extension mechanism
4. **Versioned**: Schema version tracking for backward compatibility
5. **Secure**: Encryption and signature support built-in

---

## Transport Manifest Schema

### 2.1 Core Manifest Structure

```json
{
  "$schema": "https://wia.org/schemas/cryo-transport/manifest/v1.0",
  "manifestId": "MNF-2025-001-AB",
  "manifestVersion": "1.0.0",
  "createdTimestamp": "2025-01-15T08:00:00Z",
  "status": "IN_TRANSIT",
  
  "transport": {
    "transportId": "TR-2025-001-AB",
    "transportMode": "AIR",
    "priority": "STANDARD",
    "estimatedDuration": 6.5,
    "durationUnit": "hours"
  },
  
  "subject": {
    "subjectId": "SUB-2025-045",
    "subjectType": "WHOLE_BODY",
    "species": "HOMO_SAPIENS",
    "preservationMethod": "VITRIFICATION",
    "preservationDate": "2025-01-10T15:30:00Z"
  },
  
  "origin": {
    "facilityId": "FAC-ALCOR-001",
    "facilityName": "Alcor Life Extension Foundation",
    "address": {
      "street": "7895 E Acoma Dr",
      "city": "Scottsdale",
      "state": "AZ",
      "postalCode": "85260",
      "country": "USA"
    },
    "contact": {
      "name": "Dr. Sarah Chen",
      "phone": "+1-480-905-1906",
      "email": "transport@alcor.org"
    },
    "releaseTimestamp": "2025-01-15T10:00:00Z",
    "releaseAuthorization": {
      "officerName": "Dr. Sarah Chen",
      "officerId": "HC-12345",
      "signature": "DIGITAL_SIG_BASE64_...",
      "timestamp": "2025-01-15T09:55:00Z"
    }
  },
  
  "destination": {
    "facilityId": "FAC-CI-001",
    "facilityName": "Cryonics Institute",
    "address": {
      "street": "24355 Sorrentino Ct",
      "city": "Clinton Township",
      "state": "MI",
      "postalCode": "48035",
      "country": "USA"
    },
    "contact": {
      "name": "Dr. James Wilson",
      "phone": "+1-586-791-5961",
      "email": "receiving@cryonics.org"
    },
    "expectedArrival": "2025-01-15T16:30:00Z"
  },
  
  "container": {
    "containerId": "TC-2025-001",
    "containerType": "LARGE_DEWAR",
    "manufacturer": "Taylor-Wharton",
    "model": "HC-350",
    "capacity": 350,
    "capacityUnit": "liters",
    "lastInspectionDate": "2025-01-01",
    "certificationExpiry": "2026-01-01"
  },
  
  "route": {
    "routeId": "RT-2025-001",
    "waypoints": [
      {
        "sequence": 1,
        "location": "Origin Facility",
        "latitude": 33.4942,
        "longitude": -111.9261,
        "type": "DEPARTURE",
        "eta": "2025-01-15T10:00:00Z"
      },
      {
        "sequence": 2,
        "location": "Phoenix Sky Harbor Airport",
        "latitude": 33.4342,
        "longitude": -112.0080,
        "type": "CHECKPOINT",
        "eta": "2025-01-15T11:00:00Z"
      },
      {
        "sequence": 3,
        "location": "Detroit Metropolitan Airport",
        "latitude": 42.2124,
        "longitude": -83.3534,
        "type": "CHECKPOINT",
        "eta": "2025-01-15T15:30:00Z"
      },
      {
        "sequence": 4,
        "location": "Destination Facility",
        "latitude": 42.5896,
        "longitude": -82.9199,
        "type": "ARRIVAL",
        "eta": "2025-01-15T16:30:00Z"
      }
    ],
    "backupFacilities": [
      {
        "facilityId": "BACKUP-001",
        "name": "Emergency Facility Alpha",
        "distance": 45,
        "distanceUnit": "km",
        "phone": "+1-505-555-0100"
      }
    ]
  },
  
  "personnel": {
    "coordinator": {
      "name": "Alex Martinez",
      "certificationId": "WIA-COORD-789",
      "phone": "+1-800-CRYO-911",
      "email": "coordinator@wiatransport.com"
    },
    "driver": {
      "name": "Mike Rodriguez",
      "licenseNumber": "CDL-AZ-98765",
      "phone": "+1-555-0123",
      "certifications": ["HAZMAT", "CRYO-TRANS"]
    }
  },
  
  "regulatory": {
    "hazmatClass": "2.2",
    "unNumber": "UN1977",
    "properShippingName": "Nitrogen, refrigerated liquid",
    "permits": [
      {
        "type": "DOT_HAZMAT",
        "number": "DOT-2025-12345",
        "expiry": "2025-12-31"
      }
    ]
  },
  
  "signatures": [
    {
      "role": "ORIGIN_OFFICER",
      "name": "Dr. Sarah Chen",
      "timestamp": "2025-01-15T09:55:00Z",
      "signature": "DIGITAL_SIG_...",
      "signatureAlgorithm": "RSA-SHA256"
    }
  ]
}
```

### 2.2 Field Specifications

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `manifestId` | string | Yes | Unique manifest identifier |
| `transportId` | string | Yes | Unique transport identifier |
| `subjectId` | string | Yes | Subject identification |
| `status` | enum | Yes | PLANNING, IN_TRANSIT, ARRIVED, COMPLETED |
| `transportMode` | enum | Yes | GROUND, AIR, HELICOPTER |

---

## Container Monitoring Data

### 3.1 Real-Time Sensor Data

```json
{
  "$schema": "https://wia.org/schemas/cryo-transport/monitoring/v1.0",
  "monitoringId": "MON-TR-2025-001-001",
  "transportId": "TR-2025-001-AB",
  "containerId": "TC-2025-001",
  "timestamp": "2025-01-15T13:45:30Z",
  
  "temperature": {
    "sensors": [
      {
        "sensorId": "TEMP-01",
        "location": "SUBJECT_AREA",
        "value": -195.2,
        "unit": "C",
        "status": "NORMAL",
        "accuracy": 0.5
      },
      {
        "sensorId": "TEMP-02",
        "location": "LN2_BULK",
        "value": -196.0,
        "unit": "C",
        "status": "NORMAL",
        "accuracy": 0.5
      },
      {
        "sensorId": "TEMP-03",
        "location": "VAPOR_SPACE",
        "value": -192.5,
        "unit": "C",
        "status": "NORMAL",
        "accuracy": 0.5
      }
    ],
    "mean": -194.57,
    "stdDev": 1.53,
    "min": -196.0,
    "max": -192.5
  },
  
  "ln2Level": {
    "sensorId": "LEVEL-01",
    "method": "CAPACITANCE",
    "percentage": 87,
    "volumeLiters": 304.5,
    "status": "NORMAL",
    "estimatedHoldTime": "12.5 days",
    "evaporationRate": "0.45 L/day"
  },
  
  "location": {
    "primary": {
      "latitude": 39.8561,
      "longitude": -104.6737,
      "altitude": 1655,
      "accuracy": 8,
      "satelliteCount": 14,
      "status": "EXCELLENT"
    },
    "backup": {
      "status": "STANDBY"
    }
  },
  
  "shock": {
    "sensors": [
      {
        "sensorId": "SHOCK-01",
        "location": "BASE",
        "currentG": 0.3,
        "peakG_1hour": 1.2,
        "peakG_24hour": 2.1,
        "status": "NORMAL"
      },
      {
        "sensorId": "SHOCK-02",
        "location": "TOP",
        "currentG": 0.4,
        "peakG_1hour": 1.5,
        "peakG_24hour": 2.3,
        "status": "NORMAL"
      }
    ]
  },
  
  "batteryLevel": 87,
  "transmissionMode": "CELLULAR",
  "signalStrength": -65,
  "signalStrengthUnit": "dBm",
  
  "alerts": [],
  "overallStatus": "ALL_SYSTEMS_NORMAL"
}
```

---

## Chain of Custody Records

### 4.1 Custody Transfer Schema

```json
{
  "$schema": "https://wia.org/schemas/cryo-transport/custody/v1.0",
  "custodyId": "COC-TR-2025-001",
  "transportId": "TR-2025-001-AB",
  "subjectId": "SUB-2025-045",
  
  "chainOfCustody": [
    {
      "event": "ORIGIN_RELEASE",
      "timestamp": "2025-01-15T10:00:00Z",
      "facility": "FAC-ALCOR-001",
      "handler": {
        "name": "Dr. Sarah Chen",
        "id": "HC-12345",
        "role": "RELEASE_OFFICER"
      },
      "verification": {
        "method": "DIGITAL_SIGNATURE",
        "signature": "SIG_BASE64_...",
        "algorithm": "RSA-SHA256"
      },
      "conditions": {
        "temperature": -195.5,
        "ln2Level": 95,
        "containerSealed": true
      },
      "location": {
        "latitude": 33.4942,
        "longitude": -111.9261
      }
    },
    {
      "event": "TRANSPORT_ACCEPT",
      "timestamp": "2025-01-15T10:05:00Z",
      "handler": {
        "name": "Mike Rodriguez",
        "id": "CDL-AZ-98765",
        "role": "TRANSPORT_DRIVER"
      },
      "verification": {
        "method": "DIGITAL_SIGNATURE",
        "signature": "SIG_BASE64_...",
        "algorithm": "RSA-SHA256",
        "biometric": "FINGERPRINT_HASH_..."
      },
      "conditions": {
        "temperature": -195.3,
        "ln2Level": 95,
        "containerIntegrity": "VERIFIED"
      },
      "location": {
        "latitude": 33.4942,
        "longitude": -111.9261
      }
    }
  ],
  
  "documentation": {
    "transportManifest": "ATTACHED",
    "hazmatDocuments": "ATTACHED",
    "permits": "ATTACHED",
    "insurance": "VERIFIED"
  }
}
```

---

## GPS Telemetry Format

### 5.1 Location Update Schema

```json
{
  "$schema": "https://wia.org/schemas/cryo-transport/gps/v1.0",
  "telemetryId": "GPS-TR-2025-001-12345",
  "transportId": "TR-2025-001-AB",
  "timestamp": "2025-01-15T13:45:00Z",
  
  "position": {
    "latitude": 39.7392,
    "longitude": -104.9903,
    "altitude": 1609,
    "accuracy": {
      "horizontal": 8,
      "vertical": 15,
      "unit": "meters"
    }
  },
  
  "motion": {
    "speed": 850,
    "speedUnit": "km/h",
    "heading": 95,
    "headingUnit": "degrees"
  },
  
  "satellite": {
    "count": 14,
    "systems": ["GPS", "GLONASS", "GALILEO"],
    "pdop": 1.2,
    "signalQuality": "EXCELLENT"
  },
  
  "route": {
    "distanceTraveled": 1800,
    "distanceRemaining": 850,
    "unit": "km",
    "eta": "2025-01-15T16:30:00Z",
    "onRoute": true,
    "deviationDistance": 0.5,
    "deviationUnit": "km"
  }
}
```

---

## Incident Report Templates

### 6.1 Incident Report Schema

```json
{
  "$schema": "https://wia.org/schemas/cryo-transport/incident/v1.0",
  "incidentId": "INC-TR-2025-001-001",
  "transportId": "TR-2025-001-AB",
  "classification": "LEVEL_2_MODERATE",
  "status": "RESOLVED",
  
  "detection": {
    "timestamp": "2025-01-15T14:30:00Z",
    "method": "AUTOMATED_ALERT",
    "alertType": "TEMPERATURE_WARNING",
    "detectedBy": "MONITORING_SYSTEM"
  },
  
  "event": {
    "type": "TEMPERATURE_ELEVATION",
    "description": "Temperature sensor 1 exceeded -190°C threshold",
    "temperature": {
      "peak": -189.2,
      "duration": "45 minutes",
      "sensors": ["TEMP-01"]
    },
    "location": {
      "latitude": 39.8561,
      "longitude": -104.6737,
      "description": "En route, over Colorado"
    }
  },
  
  "response": {
    "assessmentTime": "2025-01-15T14:35:00Z",
    "classification": "LEVEL_2_MODERATE",
    "actions": [
      {
        "timestamp": "2025-01-15T14:36:00Z",
        "action": "NOTIFY_ALL_PARTIES",
        "personnel": ["coordinator", "origin", "destination"]
      },
      {
        "timestamp": "2025-01-15T14:40:00Z",
        "action": "ASSESS_LN2_LEVEL",
        "result": "LN2 level adequate at 82%"
      },
      {
        "timestamp": "2025-01-15T14:45:00Z",
        "action": "MONITOR_TREND",
        "result": "Temperature stabilizing"
      }
    ],
    "resolution": {
      "timestamp": "2025-01-15T15:15:00Z",
      "outcome": "Temperature returned to normal range",
      "finalTemperature": -194.8,
      "transportContinued": true
    }
  },
  
  "rootCause": {
    "primaryCause": "Brief external heat exposure during airport handling",
    "contributingFactors": [
      "High ambient temperature (35°C)",
      "Direct sunlight on container"
    ],
    "preventable": true,
    "correctiveActions": [
      "Improved shade covering for ground transport segments",
      "Enhanced communication with airport handlers"
    ]
  }
}
```

---

## Quality Metrics

### 7.1 Transport Quality Score (TQS)

```json
{
  "$schema": "https://wia.org/schemas/cryo-transport/quality/v1.0",
  "transportId": "TR-2025-001-AB",
  "calculationTimestamp": "2025-01-15T17:00:00Z",
  
  "tqs": {
    "overall": 0.949,
    "grade": "A",
    "components": {
      "temperatureCompliance": {
        "score": 0.9861,
        "weight": 0.40,
        "weighted": 0.3944,
        "details": {
          "totalSamples": 1440,
          "optimalRange": 1420,
          "acceptableRange": 18,
          "warningRange": 2,
          "criticalRange": 0,
          "percentOptimal": 98.61
        }
      },
      "routeAdherence": {
        "score": 0.98,
        "weight": 0.25,
        "weighted": 0.245,
        "details": {
          "onRoutePercentage": 98.0,
          "maxDeviation": 2.5,
          "maxDeviationUnit": "km"
        }
      },
      "timeCompliance": {
        "score": 1.0,
        "weight": 0.20,
        "weighted": 0.20,
        "details": {
          "plannedDuration": 6.5,
          "actualDuration": 6.25,
          "variancePercentage": -3.8
        }
      },
      "documentationQuality": {
        "score": 1.0,
        "weight": 0.15,
        "weighted": 0.15,
        "details": {
          "completeness": 100,
          "timeliness": 100,
          "accuracy": 100
        }
      }
    }
  },
  
  "recommendations": [
    "Excellent transport - suitable as training reference",
    "Minor temperature elevation managed effectively",
    "Continue current protocols"
  ]
}
```

---

© 2025 WIA - World Certification Industry Association  
弘益人間 (Hongik Ingan) - Benefit All Humanity
