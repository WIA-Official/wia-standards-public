# WIA-CRYO-FACILITY: PHASE 2 - API Interface Specification

**Version:** 1.0.0
**Status:** Draft
**Date:** 2025-12-18
**Category:** Cryonics Facility Operations
**Color Code:** #06B6D4 (Cyan)

---

## 1. Introduction

### 1.1 Purpose
This specification defines RESTful API interfaces for cryonics facility management systems, enabling standardized communication for facility operations, monitoring, staff management, and emergency response.

### 1.2 API Architecture
The API follows REST principles with:
- JSON request/response format
- HTTPS-only communication
- OAuth 2.0 authentication
- Rate limiting and throttling
- Versioned endpoints
- Comprehensive error handling

### 1.3 Base URL Structure
```
https://api.cryo-facility.wia.org/v1/{resource}
```

### 1.4 Authentication
All API requests require Bearer token authentication:
```
Authorization: Bearer {access_token}
```

---

## 2. Facility Management APIs

### 2.1 GET /facilities

Retrieve list of registered cryonics facilities.

**Request:**
```http
GET /v1/facilities?status=operational&limit=50&offset=0
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
```

**Query Parameters:**

| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| status | string | No | Filter by operational status |
| country | string | No | Filter by country code |
| certified | boolean | No | Filter by certification status |
| limit | integer | No | Number of results (default: 50, max: 100) |
| offset | integer | No | Pagination offset (default: 0) |

**Response (200 OK):**
```json
{
  "status": "success",
  "timestamp": "2025-12-18T14:22:00Z",
  "data": {
    "facilities": [
      {
        "facilityId": "CRYO-FAC-A7B3C9D2",
        "facilityName": "Phoenix Cryonics Research Facility",
        "location": {
          "city": "Scottsdale",
          "state": "Arizona",
          "country": "United States"
        },
        "operationalStatus": "operational",
        "capacity": {
          "totalCapacity": 200,
          "currentOccupancy": 145,
          "availableSlots": 55
        },
        "certification": {
          "status": "active",
          "expiryDate": "2027-01-15T00:00:00Z"
        }
      }
    ],
    "pagination": {
      "total": 127,
      "limit": 50,
      "offset": 0,
      "hasMore": true
    }
  }
}
```

**Error Response (401 Unauthorized):**
```json
{
  "status": "error",
  "timestamp": "2025-12-18T14:22:00Z",
  "error": {
    "code": "AUTH_TOKEN_INVALID",
    "message": "The provided authentication token is invalid or expired",
    "details": "Token expired at 2025-12-18T12:00:00Z"
  }
}
```

### 2.2 POST /facilities

Register a new cryonics facility.

**Request:**
```http
POST /v1/facilities
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
Content-Type: application/json

{
  "facilityName": "New Hope Cryonics Center",
  "location": {
    "address": {
      "street": "1200 Innovation Drive",
      "city": "Austin",
      "state": "Texas",
      "country": "United States",
      "postalCode": "78701"
    },
    "coordinates": {
      "latitude": 30.2672,
      "longitude": -97.7431,
      "altitude": 149
    },
    "timezone": "America/Chicago"
  },
  "capacity": {
    "totalCapacity": 150,
    "capacityByType": {
      "wholebody": 100,
      "neuro": 40,
      "research": 10
    }
  },
  "contactInfo": {
    "primaryContact": {
      "name": "Dr. Robert Chen",
      "role": "Facility Director",
      "email": "r.chen@newhope-cryo.org",
      "phone": "+1-512-555-0200"
    },
    "emergencyContact": {
      "name": "Emergency Operations",
      "phone": "+1-512-555-0911",
      "email": "emergency@newhope-cryo.org",
      "available24x7": true
    }
  }
}
```

**Response (201 Created):**
```json
{
  "status": "success",
  "timestamp": "2025-12-18T14:22:00Z",
  "data": {
    "facilityId": "CRYO-FAC-B8C4D1E3",
    "facilityName": "New Hope Cryonics Center",
    "operationalStatus": "offline",
    "message": "Facility registered successfully. Certification process initiated.",
    "nextSteps": [
      "Complete certification application",
      "Schedule facility inspection",
      "Submit staff qualification records"
    ]
  }
}
```

### 2.3 GET /facilities/{facilityId}

Retrieve detailed information for a specific facility.

**Request:**
```http
GET /v1/facilities/CRYO-FAC-A7B3C9D2
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
```

**Response (200 OK):**
```json
{
  "status": "success",
  "timestamp": "2025-12-18T14:22:00Z",
  "data": {
    "facilityId": "CRYO-FAC-A7B3C9D2",
    "facilityName": "Phoenix Cryonics Research Facility",
    "certification": {
      "certificationId": "CERT-2025-001",
      "issuingBody": "International Cryonics Standards Board",
      "issueDate": "2025-01-15T00:00:00Z",
      "expiryDate": "2027-01-15T00:00:00Z",
      "status": "active",
      "accreditations": [
        {
          "accreditationName": "ISO 9001:2015 Quality Management",
          "accreditationDate": "2025-02-01T00:00:00Z",
          "validUntil": "2028-02-01T00:00:00Z"
        }
      ]
    },
    "location": {
      "address": {
        "street": "4500 N Scottsdale Road",
        "city": "Scottsdale",
        "state": "Arizona",
        "country": "United States",
        "postalCode": "85251"
      },
      "coordinates": {
        "latitude": 33.4942,
        "longitude": -111.9261,
        "altitude": 393
      },
      "timezone": "America/Phoenix"
    },
    "capacity": {
      "totalCapacity": 200,
      "currentOccupancy": 145,
      "availableSlots": 55,
      "capacityByType": {
        "wholebody": 120,
        "neuro": 60,
        "research": 20
      }
    },
    "operationalStatus": "operational",
    "contactInfo": {
      "primaryContact": {
        "name": "Dr. Sarah Mitchell",
        "role": "Facility Director",
        "email": "s.mitchell@phoenix-cryo.org",
        "phone": "+1-480-555-0100"
      },
      "emergencyContact": {
        "name": "Operations Center",
        "phone": "+1-480-555-0911",
        "email": "emergency@phoenix-cryo.org",
        "available24x7": true
      }
    },
    "metadata": {
      "createdAt": "2025-01-15T10:30:00Z",
      "updatedAt": "2025-12-18T14:22:00Z",
      "version": "1.0.0"
    }
  }
}
```

### 2.4 PATCH /facilities/{facilityId}

Update facility information.

**Request:**
```http
PATCH /v1/facilities/CRYO-FAC-A7B3C9D2
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
Content-Type: application/json

{
  "operationalStatus": "maintenance",
  "contactInfo": {
    "primaryContact": {
      "phone": "+1-480-555-0101"
    }
  }
}
```

**Response (200 OK):**
```json
{
  "status": "success",
  "timestamp": "2025-12-18T14:25:00Z",
  "data": {
    "facilityId": "CRYO-FAC-A7B3C9D2",
    "updated": true,
    "changes": [
      "operationalStatus: operational -> maintenance",
      "contactInfo.primaryContact.phone updated"
    ]
  }
}
```

---

## 3. Dewar Management APIs

### 3.1 GET /facilities/{facilityId}/dewars

List all dewars in a facility.

**Request:**
```http
GET /v1/facilities/CRYO-FAC-A7B3C9D2/dewars?status=active
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
```

**Query Parameters:**

| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| status | string | No | Filter by dewar status |
| type | string | No | Filter by dewar type |
| lowNitrogen | boolean | No | Show only dewars below refill threshold |

**Response (200 OK):**
```json
{
  "status": "success",
  "timestamp": "2025-12-18T14:22:00Z",
  "data": {
    "dewars": [
      {
        "dewarId": "DEWAR-BF01XL2025",
        "dewarType": "bigfoot",
        "currentStatus": "active",
        "liquidNitrogenLevel": {
          "currentLevel": 72.5,
          "criticalLevel": 30.0,
          "status": "normal"
        },
        "temperature": {
          "currentTemp": 77.2,
          "criticalTemp": 85.0,
          "status": "normal"
        },
        "capacity": {
          "patientCapacity": 8,
          "currentOccupancy": 6
        }
      }
    ],
    "summary": {
      "totalDewars": 12,
      "activeCount": 10,
      "maintenanceCount": 1,
      "alarmCount": 0
    }
  }
}
```

### 3.2 POST /facilities/{facilityId}/dewars

Register a new dewar unit.

**Request:**
```http
POST /v1/facilities/CRYO-FAC-A7B3C9D2/dewars
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
Content-Type: application/json

{
  "dewarType": "bigfoot",
  "manufacturerInfo": {
    "manufacturer": "Custom Biogenic Systems",
    "model": "Bigfoot XL-1000",
    "serialNumber": "CBS-BF-2025-0234",
    "manufactureDate": "2025-08-15",
    "warrantyExpiry": "2030-08-15"
  },
  "capacity": {
    "volumeLiters": 1000,
    "patientCapacity": 8
  },
  "liquidNitrogenLevel": {
    "criticalLevel": 30.0
  },
  "temperature": {
    "criticalTemp": 85.0
  }
}
```

**Response (201 Created):**
```json
{
  "status": "success",
  "timestamp": "2025-12-18T14:22:00Z",
  "data": {
    "dewarId": "DEWAR-BF03XL2025",
    "facilityId": "CRYO-FAC-A7B3C9D2",
    "currentStatus": "standby",
    "message": "Dewar registered successfully. Awaiting initial nitrogen fill and sensor calibration."
  }
}
```

### 3.3 GET /dewars/{dewarId}/monitoring

Get real-time monitoring data for a specific dewar.

**Request:**
```http
GET /v1/dewars/DEWAR-BF01XL2025/monitoring
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
```

**Response (200 OK):**
```json
{
  "status": "success",
  "timestamp": "2025-12-18T14:22:00Z",
  "data": {
    "dewarId": "DEWAR-BF01XL2025",
    "currentStatus": "active",
    "liquidNitrogenLevel": {
      "currentLevel": 72.5,
      "criticalLevel": 30.0,
      "lastRefill": "2025-12-16T08:30:00Z",
      "nextScheduledRefill": "2025-12-23T08:00:00Z",
      "averageConsumptionRate": 6.8,
      "daysUntilRefill": 6.8,
      "status": "normal"
    },
    "temperature": {
      "currentTemp": 77.2,
      "criticalTemp": 85.0,
      "status": "normal",
      "sensorReadings": [
        {
          "sensorId": "TEMP-SENSOR-001",
          "location": "upper_chamber",
          "currentReading": 77.1,
          "lastCalibration": "2025-11-01T10:00:00Z",
          "status": "operational"
        },
        {
          "sensorId": "TEMP-SENSOR-002",
          "location": "middle_chamber",
          "currentReading": 77.2,
          "lastCalibration": "2025-11-01T10:15:00Z",
          "status": "operational"
        },
        {
          "sensorId": "TEMP-SENSOR-003",
          "location": "lower_chamber",
          "currentReading": 77.3,
          "lastCalibration": "2025-11-01T10:30:00Z",
          "status": "operational"
        }
      ]
    },
    "pressure": {
      "currentPressure": 0.12,
      "normalRange": {
        "min": 0.05,
        "max": 0.25
      },
      "status": "normal"
    },
    "activeAlerts": []
  }
}
```

### 3.4 POST /dewars/{dewarId}/refill

Record a liquid nitrogen refill operation.

**Request:**
```http
POST /v1/dewars/DEWAR-BF01XL2025/refill
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
Content-Type: application/json

{
  "refillDate": "2025-12-18T08:30:00Z",
  "volumeAdded": 280,
  "performedBy": "STAFF-TC0012ABC",
  "supplier": "AirLiquide",
  "batchNumber": "LN2-2025-12-18-001",
  "notes": "Routine scheduled refill. All systems normal."
}
```

**Response (200 OK):**
```json
{
  "status": "success",
  "timestamp": "2025-12-18T14:22:00Z",
  "data": {
    "refillId": "REFILL-2025-1234",
    "dewarId": "DEWAR-BF01XL2025",
    "newLevel": 98.5,
    "nextScheduledRefill": "2025-12-25T08:00:00Z",
    "recorded": true
  }
}
```

---

## 4. Environmental Monitoring APIs

### 4.1 GET /facilities/{facilityId}/environmental

Get current environmental conditions for a facility.

**Request:**
```http
GET /v1/facilities/CRYO-FAC-A7B3C9D2/environmental
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
```

**Response (200 OK):**
```json
{
  "status": "success",
  "timestamp": "2025-12-18T14:22:00Z",
  "data": {
    "monitoringId": "ENV-MON-2025121814",
    "facilityId": "CRYO-FAC-A7B3C9D2",
    "timestamp": "2025-12-18T14:22:00Z",
    "sensorData": {
      "temperature": {
        "ambient": 22.5,
        "status": "normal",
        "dewars": [
          {
            "dewarId": "DEWAR-BF01XL2025",
            "temperature": 77.2,
            "status": "normal"
          }
        ]
      },
      "pressure": {
        "atmospheric": 101.2,
        "status": "normal"
      },
      "humidity": {
        "relativeHumidity": 35.2,
        "status": "normal"
      },
      "oxygenLevel": {
        "percentage": 20.8,
        "status": "normal"
      }
    },
    "status": "normal",
    "alerts": []
  }
}
```

### 4.2 GET /facilities/{facilityId}/environmental/history

Retrieve historical environmental data.

**Request:**
```http
GET /v1/facilities/CRYO-FAC-A7B3C9D2/environmental/history?from=2025-12-01T00:00:00Z&to=2025-12-18T23:59:59Z&interval=1h
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
```

**Query Parameters:**

| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| from | datetime | Yes | Start date/time (ISO 8601) |
| to | datetime | Yes | End date/time (ISO 8601) |
| interval | string | No | Data aggregation interval (1m, 5m, 15m, 1h, 1d) |
| metrics | string | No | Comma-separated metrics to include |

**Response (200 OK):**
```json
{
  "status": "success",
  "timestamp": "2025-12-18T14:22:00Z",
  "data": {
    "facilityId": "CRYO-FAC-A7B3C9D2",
    "period": {
      "from": "2025-12-01T00:00:00Z",
      "to": "2025-12-18T23:59:59Z",
      "interval": "1h"
    },
    "dataPoints": 432,
    "readings": [
      {
        "timestamp": "2025-12-01T00:00:00Z",
        "temperature": {
          "ambient": 22.3,
          "dewarAverage": 77.4
        },
        "humidity": 36.1,
        "oxygenLevel": 20.9
      },
      {
        "timestamp": "2025-12-01T01:00:00Z",
        "temperature": {
          "ambient": 22.1,
          "dewarAverage": 77.3
        },
        "humidity": 35.8,
        "oxygenLevel": 20.9
      }
    ],
    "summary": {
      "temperature": {
        "min": 21.5,
        "max": 23.2,
        "average": 22.4
      },
      "alertsTriggered": 0
    }
  }
}
```

### 4.3 POST /facilities/{facilityId}/environmental/alerts

Create a custom environmental alert rule.

**Request:**
```http
POST /v1/facilities/CRYO-FAC-A7B3C9D2/environmental/alerts
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
Content-Type: application/json

{
  "alertName": "High Storage Room Temperature",
  "metric": "temperature.ambient",
  "condition": "greater_than",
  "threshold": 25.0,
  "severity": "warning",
  "notificationChannels": ["email", "sms", "dashboard"],
  "recipients": [
    "s.mitchell@phoenix-cryo.org",
    "ops@phoenix-cryo.org"
  ]
}
```

**Response (201 Created):**
```json
{
  "status": "success",
  "timestamp": "2025-12-18T14:22:00Z",
  "data": {
    "alertRuleId": "ALERT-RULE-0123",
    "facilityId": "CRYO-FAC-A7B3C9D2",
    "alertName": "High Storage Room Temperature",
    "status": "active",
    "message": "Alert rule created successfully and is now monitoring."
  }
}
```

---

## 5. Staff Management APIs

### 5.1 GET /facilities/{facilityId}/staff

List all staff members at a facility.

**Request:**
```http
GET /v1/facilities/CRYO-FAC-A7B3C9D2/staff?status=active&role=cryonics_technician
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
```

**Query Parameters:**

| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| status | string | No | Filter by employment status |
| role | string | No | Filter by role |
| certificationExpiring | integer | No | Show staff with certs expiring in N days |

**Response (200 OK):**
```json
{
  "status": "success",
  "timestamp": "2025-12-18T14:22:00Z",
  "data": {
    "staff": [
      {
        "staffId": "STAFF-TC0012ABC",
        "personalInfo": {
          "firstName": "James",
          "lastName": "Rodriguez",
          "email": "j.rodriguez@phoenix-cryo.org"
        },
        "employmentStatus": {
          "status": "active",
          "role": "Senior Cryonics Technician",
          "department": "Operations",
          "startDate": "2019-06-01"
        },
        "qualifications": {
          "certificationsCount": 2,
          "expiringCertifications": 1
        }
      }
    ],
    "summary": {
      "totalStaff": 45,
      "activeStaff": 42,
      "onLeave": 2,
      "suspended": 1
    }
  }
}
```

### 5.2 POST /facilities/{facilityId}/staff

Add a new staff member.

**Request:**
```http
POST /v1/facilities/CRYO-FAC-A7B3C9D2/staff
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
Content-Type: application/json

{
  "personalInfo": {
    "firstName": "Emily",
    "lastName": "Watson",
    "email": "e.watson@phoenix-cryo.org",
    "phone": "+1-480-555-0245"
  },
  "employmentStatus": {
    "role": "Cryonics Technician",
    "department": "Operations",
    "startDate": "2025-12-18",
    "accessLevel": "basic"
  },
  "qualifications": {
    "education": [
      {
        "degree": "Bachelor of Science in Biology",
        "institution": "University of Arizona",
        "completionDate": "2023-05-15",
        "verified": true
      }
    ],
    "certifications": [
      {
        "certificationName": "Certified Cryonics Technician",
        "issuingOrganization": "American Cryonics Society",
        "issueDate": "2025-10-10",
        "expiryDate": "2032-10-10",
        "certificationNumber": "CCT-2025-0789",
        "status": "valid"
      }
    ]
  }
}
```

**Response (201 Created):**
```json
{
  "status": "success",
  "timestamp": "2025-12-18T14:22:00Z",
  "data": {
    "staffId": "STAFF-TC0034DEF",
    "facilityId": "CRYO-FAC-A7B3C9D2",
    "message": "Staff member added successfully. Access credentials pending."
  }
}
```

### 5.3 POST /staff/{staffId}/training

Record completion of a training course.

**Request:**
```http
POST /v1/staff/STAFF-TC0012ABC/training
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
Content-Type: application/json

{
  "trainingName": "Advanced Dewar Maintenance 2025",
  "trainingType": "advanced",
  "completionDate": "2025-12-15T16:30:00Z",
  "expiryDate": "2027-12-15T16:30:00Z",
  "score": 92,
  "instructor": "Dr. Emily Chen",
  "certificateIssued": true
}
```

**Response (201 Created):**
```json
{
  "status": "success",
  "timestamp": "2025-12-18T14:22:00Z",
  "data": {
    "trainingId": "TRAIN-2025-0234",
    "staffId": "STAFF-TC0012ABC",
    "recorded": true,
    "certificateUrl": "https://certificates.cryo-facility.wia.org/TRAIN-2025-0234.pdf"
  }
}
```

---

## 6. Emergency Protocol APIs

### 6.1 GET /facilities/{facilityId}/emergency-protocols

List all emergency protocols for a facility.

**Request:**
```http
GET /v1/facilities/CRYO-FAC-A7B3C9D2/emergency-protocols
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
```

**Response (200 OK):**
```json
{
  "status": "success",
  "timestamp": "2025-12-18T14:22:00Z",
  "data": {
    "protocols": [
      {
        "protocolId": "EMERG-PROTO-001",
        "protocolName": "Dewar Temperature Critical Alert",
        "severity": "critical",
        "lastDrillDate": "2025-09-15T10:00:00Z",
        "nextScheduledDrill": "2026-03-15T10:00:00Z",
        "status": "active"
      },
      {
        "protocolId": "EMERG-PROTO-002",
        "protocolName": "Power Outage Response",
        "severity": "critical",
        "lastDrillDate": "2025-10-20T14:00:00Z",
        "nextScheduledDrill": "2026-04-20T14:00:00Z",
        "status": "active"
      },
      {
        "protocolId": "EMERG-PROTO-003",
        "protocolName": "Nitrogen Supply Disruption",
        "severity": "critical",
        "lastDrillDate": "2025-11-10T09:00:00Z",
        "nextScheduledDrill": "2026-05-10T09:00:00Z",
        "status": "active"
      }
    ]
  }
}
```

### 6.2 POST /facilities/{facilityId}/emergency/activate

Activate an emergency protocol.

**Request:**
```http
POST /v1/facilities/CRYO-FAC-A7B3C9D2/emergency/activate
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
Content-Type: application/json

{
  "protocolId": "EMERG-PROTO-001",
  "triggerReason": "Dewar DEWAR-BF02XL2025 temperature exceeded critical threshold",
  "triggeredBy": "STAFF-TC0012ABC",
  "affectedAssets": ["DEWAR-BF02XL2025"],
  "severity": "critical"
}
```

**Response (200 OK):**
```json
{
  "status": "success",
  "timestamp": "2025-12-18T14:22:00Z",
  "data": {
    "incidentId": "INCIDENT-2025-0089",
    "protocolId": "EMERG-PROTO-001",
    "status": "activated",
    "responseTeam": [
      {
        "role": "Emergency Coordinator",
        "name": "Dr. Sarah Mitchell",
        "notified": true,
        "notificationTime": "2025-12-18T14:22:01Z"
      },
      {
        "role": "Senior Cryonics Technician",
        "name": "James Rodriguez",
        "notified": true,
        "notificationTime": "2025-12-18T14:22:02Z"
      }
    ],
    "responseSteps": [
      {
        "step": 1,
        "action": "immediate_alert",
        "status": "completed",
        "completedAt": "2025-12-18T14:22:05Z"
      },
      {
        "step": 2,
        "action": "initiate_backup_cooling",
        "status": "in_progress",
        "assignedTo": "STAFF-TC0012ABC"
      }
    ],
    "message": "Emergency protocol activated. Response team notified."
  }
}
```

### 6.3 POST /incidents/{incidentId}/update

Update the status of an active incident.

**Request:**
```http
POST /v1/incidents/INCIDENT-2025-0089/update
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
Content-Type: application/json

{
  "stepCompleted": 2,
  "actionTaken": "Backup nitrogen supply activated. Dewar temperature stabilizing at 78K.",
  "updatedBy": "STAFF-TC0012ABC",
  "timestamp": "2025-12-18T14:27:00Z"
}
```

**Response (200 OK):**
```json
{
  "status": "success",
  "timestamp": "2025-12-18T14:27:00Z",
  "data": {
    "incidentId": "INCIDENT-2025-0089",
    "currentStep": 3,
    "nextAction": "assess_patient_risk",
    "incidentStatus": "ongoing",
    "timeElapsed": "5 minutes"
  }
}
```

---

## 7. Compliance and Reporting APIs

### 7.1 GET /facilities/{facilityId}/compliance/status

Get compliance status for a facility.

**Request:**
```http
GET /v1/facilities/CRYO-FAC-A7B3C9D2/compliance/status
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
```

**Response (200 OK):**
```json
{
  "status": "success",
  "timestamp": "2025-12-18T14:22:00Z",
  "data": {
    "facilityId": "CRYO-FAC-A7B3C9D2",
    "overallStatus": "compliant",
    "lastAudit": "2025-09-15T10:00:00Z",
    "nextAudit": "2026-03-15T10:00:00Z",
    "complianceAreas": {
      "certification": {
        "status": "compliant",
        "validUntil": "2027-01-15T00:00:00Z",
        "issues": []
      },
      "staffQualifications": {
        "status": "warning",
        "issues": [
          "3 staff members have certifications expiring within 60 days"
        ]
      },
      "maintenanceSchedule": {
        "status": "compliant",
        "overdueItems": 0,
        "upcomingItems": 5
      },
      "emergencyProtocols": {
        "status": "compliant",
        "lastDrillDate": "2025-11-10T09:00:00Z",
        "nextDrillDate": "2026-02-10T09:00:00Z"
      }
    },
    "recommendations": [
      "Schedule certification renewal training for 3 staff members",
      "Update emergency contact list to include new staff"
    ]
  }
}
```

### 7.2 GET /facilities/{facilityId}/reports/monthly

Generate monthly operational report.

**Request:**
```http
GET /v1/facilities/CRYO-FAC-A7B3C9D2/reports/monthly?month=2025-12
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
```

**Response (200 OK):**
```json
{
  "status": "success",
  "timestamp": "2025-12-18T14:22:00Z",
  "data": {
    "facilityId": "CRYO-FAC-A7B3C9D2",
    "reportPeriod": {
      "month": "2025-12",
      "from": "2025-12-01T00:00:00Z",
      "to": "2025-12-18T23:59:59Z"
    },
    "operationalSummary": {
      "uptimePercentage": 99.97,
      "incidentsReported": 2,
      "criticalIncidents": 0,
      "maintenanceHours": 24,
      "staffHours": 3240
    },
    "dewarPerformance": {
      "totalDewars": 12,
      "averageTemperature": 77.3,
      "nitrogenRefills": 48,
      "totalNitrogenConsumed": 3420,
      "alarmsTriggered": 3,
      "criticalAlarms": 0
    },
    "capacityMetrics": {
      "startingOccupancy": 142,
      "endingOccupancy": 145,
      "newAdmissions": 3,
      "transfers": 0,
      "currentCapacityUtilization": 72.5
    },
    "complianceMetrics": {
      "staffTrainingCompleted": 15,
      "maintenanceTasksCompleted": 42,
      "auditsPassed": 1,
      "regulatoryViolations": 0
    },
    "reportUrl": "https://reports.cryo-facility.wia.org/CRYO-FAC-A7B3C9D2/2025-12.pdf"
  }
}
```

---

## 8. API Response Codes

### 8.1 Success Codes

| Code | Message | Description |
|------|---------|-------------|
| 200 | OK | Request successful |
| 201 | Created | Resource created successfully |
| 202 | Accepted | Request accepted, processing asynchronously |
| 204 | No Content | Request successful, no content to return |

### 8.2 Client Error Codes

| Code | Message | Description |
|------|---------|-------------|
| 400 | Bad Request | Invalid request format or parameters |
| 401 | Unauthorized | Authentication required or failed |
| 403 | Forbidden | Authenticated but not authorized |
| 404 | Not Found | Resource not found |
| 409 | Conflict | Resource conflict (duplicate, state violation) |
| 422 | Unprocessable Entity | Validation error |
| 429 | Too Many Requests | Rate limit exceeded |

### 8.3 Server Error Codes

| Code | Message | Description |
|------|---------|-------------|
| 500 | Internal Server Error | Unexpected server error |
| 502 | Bad Gateway | Upstream service error |
| 503 | Service Unavailable | Service temporarily unavailable |
| 504 | Gateway Timeout | Upstream service timeout |

---

## 9. Rate Limiting

### 9.1 Rate Limit Headers

All API responses include rate limiting headers:

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 987
X-RateLimit-Reset: 1703088000
```

### 9.2 Rate Limit Tiers

| Tier | Requests/Hour | Burst Limit |
|------|---------------|-------------|
| Basic | 1,000 | 50 |
| Professional | 5,000 | 200 |
| Enterprise | 20,000 | 1,000 |
| Emergency | Unlimited | Unlimited |

---

## 10. Webhooks

### 10.1 Webhook Events

| Event | Description | Payload |
|-------|-------------|---------|
| dewar.alert.critical | Critical dewar alert triggered | DewarAlert |
| facility.status.changed | Facility status changed | FacilityStatus |
| staff.certification.expiring | Staff certification expiring soon | StaffCertification |
| emergency.activated | Emergency protocol activated | EmergencyIncident |
| maintenance.overdue | Maintenance task overdue | MaintenanceTask |

### 10.2 Webhook Registration Example

**Request:**
```http
POST /v1/webhooks
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
Content-Type: application/json

{
  "url": "https://myapp.example.com/webhooks/cryo-facility",
  "events": [
    "dewar.alert.critical",
    "emergency.activated"
  ],
  "secret": "whsec_7f8d9e0a1b2c3d4e5f6g7h8i9j0k1l2m"
}
```

**Response (201 Created):**
```json
{
  "status": "success",
  "timestamp": "2025-12-18T14:22:00Z",
  "data": {
    "webhookId": "WEBHOOK-A1B2C3D4",
    "url": "https://myapp.example.com/webhooks/cryo-facility",
    "events": [
      "dewar.alert.critical",
      "emergency.activated"
    ],
    "status": "active",
    "createdAt": "2025-12-18T14:22:00Z"
  }
}
```

### 10.3 Webhook Payload Example

```json
{
  "webhookId": "WEBHOOK-A1B2C3D4",
  "eventType": "dewar.alert.critical",
  "eventId": "EVENT-2025-1234",
  "timestamp": "2025-12-18T14:22:00Z",
  "data": {
    "facilityId": "CRYO-FAC-A7B3C9D2",
    "dewarId": "DEWAR-BF02XL2025",
    "alertType": "temperature_critical",
    "severity": "critical",
    "currentTemperature": 86.5,
    "criticalThreshold": 85.0,
    "message": "Dewar temperature exceeded critical threshold",
    "actionRequired": true
  },
  "signature": "sha256=7f8d9e0a1b2c3d4e5f6g7h8i9j0k1l2m..."
}
```

---

## 11. API Client Examples

### 11.1 Python Client Example

```python
import requests
import json
from datetime import datetime

class CryoFacilityAPI:
    def __init__(self, api_key, base_url="https://api.cryo-facility.wia.org/v1"):
        self.api_key = api_key
        self.base_url = base_url
        self.headers = {
            "Authorization": f"Bearer {api_key}",
            "Content-Type": "application/json"
        }

    def get_facility(self, facility_id):
        """Get facility details"""
        response = requests.get(
            f"{self.base_url}/facilities/{facility_id}",
            headers=self.headers
        )
        response.raise_for_status()
        return response.json()

    def list_dewars(self, facility_id, status=None):
        """List dewars in facility"""
        params = {"status": status} if status else {}
        response = requests.get(
            f"{self.base_url}/facilities/{facility_id}/dewars",
            headers=self.headers,
            params=params
        )
        response.raise_for_status()
        return response.json()

    def get_environmental_data(self, facility_id):
        """Get current environmental conditions"""
        response = requests.get(
            f"{self.base_url}/facilities/{facility_id}/environmental",
            headers=self.headers
        )
        response.raise_for_status()
        return response.json()

    def activate_emergency(self, facility_id, protocol_id, reason, triggered_by):
        """Activate emergency protocol"""
        data = {
            "protocolId": protocol_id,
            "triggerReason": reason,
            "triggeredBy": triggered_by,
            "severity": "critical"
        }
        response = requests.post(
            f"{self.base_url}/facilities/{facility_id}/emergency/activate",
            headers=self.headers,
            json=data
        )
        response.raise_for_status()
        return response.json()

# Usage example
api = CryoFacilityAPI("your_api_key_here")

# Get facility information
facility = api.get_facility("CRYO-FAC-A7B3C9D2")
print(f"Facility: {facility['data']['facilityName']}")
print(f"Occupancy: {facility['data']['capacity']['currentOccupancy']}/{facility['data']['capacity']['totalCapacity']}")

# List active dewars
dewars = api.list_dewars("CRYO-FAC-A7B3C9D2", status="active")
print(f"Active dewars: {len(dewars['data']['dewars'])}")

# Check environmental conditions
env = api.get_environmental_data("CRYO-FAC-A7B3C9D2")
print(f"Current temperature: {env['data']['sensorData']['temperature']['ambient']}°C")
```

### 11.2 JavaScript/Node.js Client Example

```javascript
const axios = require('axios');

class CryoFacilityAPI {
  constructor(apiKey, baseURL = 'https://api.cryo-facility.wia.org/v1') {
    this.apiKey = apiKey;
    this.baseURL = baseURL;
    this.client = axios.create({
      baseURL: this.baseURL,
      headers: {
        'Authorization': `Bearer ${apiKey}`,
        'Content-Type': 'application/json'
      }
    });
  }

  async getFacility(facilityId) {
    const response = await this.client.get(`/facilities/${facilityId}`);
    return response.data;
  }

  async listDewars(facilityId, options = {}) {
    const response = await this.client.get(`/facilities/${facilityId}/dewars`, {
      params: options
    });
    return response.data;
  }

  async getDewarMonitoring(dewarId) {
    const response = await this.client.get(`/dewars/${dewarId}/monitoring`);
    return response.data;
  }

  async recordRefill(dewarId, refillData) {
    const response = await this.client.post(`/dewars/${dewarId}/refill`, refillData);
    return response.data;
  }

  async getEnvironmentalData(facilityId) {
    const response = await this.client.get(`/facilities/${facilityId}/environmental`);
    return response.data;
  }

  async activateEmergency(facilityId, emergencyData) {
    const response = await this.client.post(
      `/facilities/${facilityId}/emergency/activate`,
      emergencyData
    );
    return response.data;
  }
}

// Usage example
const api = new CryoFacilityAPI('your_api_key_here');

async function monitorFacility() {
  try {
    // Get facility info
    const facility = await api.getFacility('CRYO-FAC-A7B3C9D2');
    console.log(`Facility: ${facility.data.facilityName}`);

    // List dewars
    const dewars = await api.listDewars('CRYO-FAC-A7B3C9D2', { status: 'active' });
    console.log(`Active dewars: ${dewars.data.dewars.length}`);

    // Check each dewar
    for (const dewar of dewars.data.dewars) {
      const monitoring = await api.getDewarMonitoring(dewar.dewarId);
      console.log(`${dewar.dewarId}: ${monitoring.data.temperature.currentTemp}K, ${monitoring.data.liquidNitrogenLevel.currentLevel}%`);

      // Check if refill needed
      if (monitoring.data.liquidNitrogenLevel.currentLevel < monitoring.data.liquidNitrogenLevel.criticalLevel) {
        console.warn(`WARNING: ${dewar.dewarId} needs refill!`);
      }
    }

    // Get environmental data
    const env = await api.getEnvironmentalData('CRYO-FAC-A7B3C9D2');
    console.log(`Environment: ${env.data.sensorData.temperature.ambient}°C, ${env.data.sensorData.humidity.relativeHumidity}% RH`);

  } catch (error) {
    console.error('Error:', error.response?.data || error.message);
  }
}

// Run monitoring
monitorFacility();
```

---

## 12. Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-12-18 | Initial API specification |

---

## 13. References

- REST API Design Best Practices
- OAuth 2.0 RFC 6749
- JSON Schema Specification
- OpenAPI 3.0 Specification

---

**弘益人間 (홍익인간)** - Benefit All Humanity
© 2025 WIA
MIT License
