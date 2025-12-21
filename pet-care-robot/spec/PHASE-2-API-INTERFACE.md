# Phase 2: Pet Care Robot API Interface Specification

## WIA-PET-CARE-ROBOT API Standard

**Version**: 1.0.0
**Date**: 2025-12-18
**Status**: Draft
**Standard ID**: WIA-PET-CARE-ROBOT-PHASE2-001
**Primary Color**: #F59E0B (Amber)

---

## 1. Overview

### 1.1 Purpose

This specification defines the RESTful API interfaces for pet care robot systems, enabling standardized communication between robots, cloud services, mobile applications, and smart home platforms. The API supports real-time control, scheduling, monitoring, and integration across the pet care ecosystem.

**API Design Principles**:
- RESTful architecture with JSON payloads
- OAuth 2.0 authentication and authorization
- Rate limiting and quota management
- Webhook support for real-time events
- Versioned endpoints for backward compatibility
- Comprehensive error handling
- HATEOAS support for discoverability

### 1.2 Base URL Structure

```
Production:  https://api.petcare.wia.org/v1
Staging:     https://api-staging.petcare.wia.org/v1
Development: https://api-dev.petcare.wia.org/v1
```

### 1.3 API Endpoints Overview

| Category | Endpoints | Description |
|----------|-----------|-------------|
| **Robot Management** | 5 endpoints | Device registration, status, configuration |
| **Feeding Operations** | 6 endpoints | Schedule, dispense, history, nutrition |
| **Play & Exercise** | 5 endpoints | Sessions, control, analytics |
| **Health Monitoring** | 4 endpoints | Observations, alerts, trends |
| **Pet Profiles** | 5 endpoints | Pet management, identification |
| **Scheduling** | 4 endpoints | Automated routines, calendars |
| **Smart Home** | 3 endpoints | Integration, voice commands |
| **Media** | 3 endpoints | Camera, recordings, streaming |
| **Analytics** | 4 endpoints | Reports, insights, predictions |
| **Webhooks** | 3 endpoints | Event subscriptions |

---

## 2. Authentication & Authorization

### 2.1 OAuth 2.0 Flow

```http
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials
&client_id=YOUR_CLIENT_ID
&client_secret=YOUR_CLIENT_SECRET
&scope=robot:read robot:write pet:read pet:write
```

**Response:**
```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "robot:read robot:write pet:read pet:write",
  "refresh_token": "def50200..."
}
```

### 2.2 API Key Authentication

```http
GET /api/v1/robots
Authorization: Bearer YOUR_ACCESS_TOKEN
X-API-Key: your-api-key-here
```

### 2.3 Permission Scopes

| Scope | Description | Operations |
|-------|-------------|------------|
| `robot:read` | View robot status and data | GET operations |
| `robot:write` | Control robot operations | POST, PUT, PATCH |
| `robot:admin` | Full robot management | All operations including DELETE |
| `pet:read` | View pet profiles and data | GET pet information |
| `pet:write` | Manage pet profiles | Create/update pets |
| `feeding:execute` | Trigger feeding operations | Manual feeding |
| `play:execute` | Control play sessions | Start/stop play |
| `health:read` | Access health data | View health observations |
| `analytics:read` | Access reports and insights | Analytics endpoints |
| `webhook:manage` | Manage webhook subscriptions | Webhook operations |

---

## 3. Robot Management Endpoints

### 3.1 List Robots

```http
GET /api/v1/robots
```

**Query Parameters:**
| Parameter | Type | Description | Required |
|-----------|------|-------------|----------|
| `status` | string | Filter by operational status | No |
| `deviceType` | string | Filter by device type | No |
| `location` | string | Filter by location/room | No |
| `page` | integer | Page number (default: 1) | No |
| `limit` | integer | Items per page (default: 20, max: 100) | No |

**Response (200 OK):**
```json
{
  "data": [
    {
      "robotId": "PCR-ABC123456789",
      "deviceType": "multi_function",
      "manufacturer": {
        "name": "PetTech Pro",
        "model": "AutoCare 3000",
        "serialNumber": "SN-2025-001234"
      },
      "status": {
        "operational": "online",
        "batteryLevel": 85,
        "foodLevel": 65,
        "waterLevel": 80
      },
      "location": {
        "room": "Living Room",
        "zone": "Pet Area"
      },
      "capabilities": {
        "feeding": true,
        "watering": true,
        "play": true,
        "monitoring": true
      },
      "lastSeen": "2025-12-18T10:30:00Z"
    }
  ],
  "pagination": {
    "page": 1,
    "limit": 20,
    "total": 3,
    "pages": 1
  },
  "links": {
    "self": "/api/v1/robots?page=1",
    "first": "/api/v1/robots?page=1",
    "last": "/api/v1/robots?page=1"
  }
}
```

### 3.2 Get Robot Details

```http
GET /api/v1/robots/{robotId}
```

**Response (200 OK):**
```json
{
  "robotId": "PCR-ABC123456789",
  "deviceType": "multi_function",
  "manufacturer": {
    "name": "PetTech Pro",
    "model": "AutoCare 3000",
    "serialNumber": "SN-2025-001234",
    "firmwareVersion": "2.5.1",
    "hardwareRevision": "Rev C",
    "manufactureDate": "2025-06-15"
  },
  "capabilities": {
    "feeding": {
      "enabled": true,
      "portionControl": true,
      "scheduledFeeding": true,
      "maxPortionSize": 200,
      "containerCapacity": 5000,
      "foodTypes": ["dry_kibble", "treats"]
    },
    "watering": {
      "enabled": true,
      "waterFiltered": true,
      "waterCirculation": true,
      "capacityLiters": 2.5
    },
    "play": {
      "enabled": true,
      "interactiveToys": true,
      "laserPointer": true,
      "motionTracking": true,
      "maxPlayDuration": 30
    },
    "monitoring": {
      "camera": true,
      "nightVision": true,
      "twoWayAudio": true,
      "petRecognition": true,
      "weightScale": true
    }
  },
  "sensors": [
    {
      "sensorId": "SENS-001",
      "sensorType": "camera",
      "status": "active",
      "lastCalibration": "2025-12-15T08:00:00Z"
    },
    {
      "sensorId": "SENS-002",
      "sensorType": "weight_scale",
      "status": "active",
      "lastCalibration": "2025-12-10T12:00:00Z"
    }
  ],
  "status": {
    "operational": "online",
    "batteryLevel": 85,
    "charging": false,
    "foodLevel": 65,
    "waterLevel": 80,
    "errors": []
  },
  "location": {
    "room": "Living Room",
    "zone": "Pet Area",
    "fixedPosition": true
  },
  "connectivity": {
    "wifi": {
      "connected": true,
      "signalStrength": -45,
      "ssid": "HomeNetwork"
    },
    "cloud": {
      "connected": true,
      "lastSync": "2025-12-18T10:29:55Z"
    }
  },
  "createdAt": "2025-08-01T09:00:00Z",
  "updatedAt": "2025-12-18T10:30:00Z"
}
```

### 3.3 Register New Robot

```http
POST /api/v1/robots
Content-Type: application/json
```

**Request Body:**
```json
{
  "deviceType": "multi_function",
  "serialNumber": "SN-2025-001234",
  "location": {
    "room": "Living Room",
    "zone": "Pet Area"
  },
  "setupCode": "SETUP-123456"
}
```

**Response (201 Created):**
```json
{
  "robotId": "PCR-ABC123456789",
  "deviceType": "multi_function",
  "registrationStatus": "completed",
  "setupInstructions": {
    "step": 1,
    "message": "Robot registered successfully. Connect to WiFi network.",
    "nextSteps": [
      "Configure WiFi settings",
      "Calibrate sensors",
      "Register pets"
    ]
  },
  "links": {
    "self": "/api/v1/robots/PCR-ABC123456789",
    "configure": "/api/v1/robots/PCR-ABC123456789/configure",
    "status": "/api/v1/robots/PCR-ABC123456789/status"
  }
}
```

### 3.4 Update Robot Configuration

```http
PATCH /api/v1/robots/{robotId}
Content-Type: application/json
```

**Request Body:**
```json
{
  "location": {
    "room": "Kitchen",
    "zone": "Feeding Area"
  },
  "settings": {
    "quietMode": true,
    "nightVisionEnabled": true,
    "alertVolume": 5
  }
}
```

**Response (200 OK):**
```json
{
  "robotId": "PCR-ABC123456789",
  "updatedFields": ["location", "settings"],
  "message": "Robot configuration updated successfully",
  "updatedAt": "2025-12-18T10:35:00Z"
}
```

### 3.5 Get Robot Status

```http
GET /api/v1/robots/{robotId}/status
```

**Response (200 OK):**
```json
{
  "robotId": "PCR-ABC123456789",
  "operational": "online",
  "batteryLevel": 85,
  "charging": false,
  "supplies": {
    "foodLevel": 65,
    "foodLevelGrams": 3250,
    "foodCapacity": 5000,
    "waterLevel": 80,
    "waterLevelMl": 2000,
    "waterCapacity": 2500
  },
  "alerts": [
    {
      "alertId": "ALERT-001",
      "type": "low_food_warning",
      "severity": "warning",
      "message": "Food level below 70%. Consider refilling soon.",
      "timestamp": "2025-12-18T09:00:00Z"
    }
  ],
  "uptime": 432000,
  "lastMaintenance": "2025-12-10T14:00:00Z",
  "nextMaintenanceDue": "2026-01-10T14:00:00Z",
  "timestamp": "2025-12-18T10:30:00Z"
}
```

---

## 4. Feeding Operations Endpoints

### 4.1 Schedule Feeding

```http
POST /api/v1/feeding/schedule
Content-Type: application/json
```

**Request Body:**
```json
{
  "robotId": "PCR-ABC123456789",
  "petId": "PET-DOG001",
  "schedule": {
    "name": "Morning Breakfast",
    "time": "07:00:00",
    "timezone": "America/New_York",
    "recurring": true,
    "daysOfWeek": ["MON", "TUE", "WED", "THU", "FRI", "SAT", "SUN"]
  },
  "portion": {
    "size": 150,
    "unit": "grams",
    "foodType": "dry_kibble"
  },
  "enabled": true
}
```

**Response (201 Created):**
```json
{
  "scheduleId": "SCHED-FEED-001",
  "robotId": "PCR-ABC123456789",
  "petId": "PET-DOG001",
  "schedule": {
    "name": "Morning Breakfast",
    "time": "07:00:00",
    "timezone": "America/New_York",
    "recurring": true,
    "daysOfWeek": ["MON", "TUE", "WED", "THU", "FRI", "SAT", "SUN"],
    "nextExecution": "2025-12-19T07:00:00-05:00"
  },
  "portion": {
    "size": 150,
    "unit": "grams",
    "foodType": "dry_kibble"
  },
  "enabled": true,
  "createdAt": "2025-12-18T10:40:00Z"
}
```

### 4.2 Manual Feed Dispense

```http
POST /api/v1/feeding/dispense
Content-Type: application/json
```

**Request Body:**
```json
{
  "robotId": "PCR-ABC123456789",
  "petId": "PET-DOG001",
  "portionSize": 100,
  "foodType": "treats",
  "reason": "manual_reward"
}
```

**Response (200 OK):**
```json
{
  "eventId": "FEED-20251218-ABC123",
  "robotId": "PCR-ABC123456789",
  "petId": "PET-DOG001",
  "timestamp": "2025-12-18T10:45:00Z",
  "dispensed": true,
  "portionSize": 100,
  "foodType": "treats",
  "remainingFood": 3150,
  "message": "Food dispensed successfully"
}
```

### 4.3 Get Feeding History

```http
GET /api/v1/feeding/history
```

**Query Parameters:**
| Parameter | Type | Description |
|-----------|------|-------------|
| `robotId` | string | Filter by robot |
| `petId` | string | Filter by pet |
| `startDate` | date | Start date (ISO 8601) |
| `endDate` | date | End date (ISO 8601) |
| `feedingType` | string | Filter by type (scheduled/manual/automated) |
| `page` | integer | Page number |
| `limit` | integer | Items per page |

**Response (200 OK):**
```json
{
  "data": [
    {
      "eventId": "FEED-20251218-001",
      "robotId": "PCR-ABC123456789",
      "petId": "PET-DOG001",
      "timestamp": "2025-12-18T07:00:00Z",
      "feedingType": "scheduled",
      "foodType": "dry_kibble",
      "portionSize": 150,
      "dispensed": true,
      "consumed": {
        "detected": true,
        "estimatedAmount": 148,
        "duration": 180
      },
      "schedule": {
        "scheduleId": "SCHED-FEED-001",
        "onTime": true
      }
    },
    {
      "eventId": "FEED-20251218-002",
      "robotId": "PCR-ABC123456789",
      "petId": "PET-DOG001",
      "timestamp": "2025-12-18T12:00:00Z",
      "feedingType": "scheduled",
      "foodType": "dry_kibble",
      "portionSize": 100,
      "dispensed": true,
      "consumed": {
        "detected": true,
        "estimatedAmount": 95,
        "duration": 145
      }
    }
  ],
  "summary": {
    "totalFeedings": 2,
    "totalDispensed": 250,
    "totalConsumed": 243,
    "averageConsumptionRate": 97.2
  },
  "pagination": {
    "page": 1,
    "limit": 20,
    "total": 2
  }
}
```

### 4.4 Get Feeding Schedules

```http
GET /api/v1/feeding/schedules
```

**Response (200 OK):**
```json
{
  "data": [
    {
      "scheduleId": "SCHED-FEED-001",
      "name": "Morning Breakfast",
      "robotId": "PCR-ABC123456789",
      "petId": "PET-DOG001",
      "time": "07:00:00",
      "timezone": "America/New_York",
      "recurring": true,
      "daysOfWeek": ["MON", "TUE", "WED", "THU", "FRI", "SAT", "SUN"],
      "portion": {
        "size": 150,
        "foodType": "dry_kibble"
      },
      "enabled": true,
      "nextExecution": "2025-12-19T07:00:00-05:00",
      "lastExecution": "2025-12-18T07:00:00-05:00",
      "executionCount": 45
    },
    {
      "scheduleId": "SCHED-FEED-002",
      "name": "Evening Dinner",
      "robotId": "PCR-ABC123456789",
      "petId": "PET-DOG001",
      "time": "18:00:00",
      "timezone": "America/New_York",
      "recurring": true,
      "daysOfWeek": ["MON", "TUE", "WED", "THU", "FRI", "SAT", "SUN"],
      "portion": {
        "size": 150,
        "foodType": "dry_kibble"
      },
      "enabled": true,
      "nextExecution": "2025-12-18T18:00:00-05:00"
    }
  ]
}
```

### 4.5 Update Feeding Schedule

```http
PATCH /api/v1/feeding/schedules/{scheduleId}
Content-Type: application/json
```

**Request Body:**
```json
{
  "portion": {
    "size": 175
  },
  "enabled": true
}
```

**Response (200 OK):**
```json
{
  "scheduleId": "SCHED-FEED-001",
  "message": "Schedule updated successfully",
  "updatedFields": ["portion"],
  "nextExecution": "2025-12-19T07:00:00-05:00"
}
```

### 4.6 Delete Feeding Schedule

```http
DELETE /api/v1/feeding/schedules/{scheduleId}
```

**Response (200 OK):**
```json
{
  "scheduleId": "SCHED-FEED-001",
  "message": "Feeding schedule deleted successfully",
  "deletedAt": "2025-12-18T10:50:00Z"
}
```

---

## 5. Play & Exercise Endpoints

### 5.1 Start Play Session

```http
POST /api/v1/play/sessions
Content-Type: application/json
```

**Request Body:**
```json
{
  "robotId": "PCR-ABC123456789",
  "petId": "PET-DOG001",
  "playType": "laser_chase",
  "duration": 900,
  "intensity": "adaptive",
  "settings": {
    "speed": "medium",
    "pattern": "random",
    "restIntervals": true
  }
}
```

**Response (201 Created):**
```json
{
  "sessionId": "PLAY-20251218-001",
  "robotId": "PCR-ABC123456789",
  "petId": "PET-DOG001",
  "playType": "laser_chase",
  "startTime": "2025-12-18T11:00:00Z",
  "estimatedEndTime": "2025-12-18T11:15:00Z",
  "status": "active",
  "intensity": "adaptive",
  "links": {
    "self": "/api/v1/play/sessions/PLAY-20251218-001",
    "stop": "/api/v1/play/sessions/PLAY-20251218-001/stop",
    "status": "/api/v1/play/sessions/PLAY-20251218-001/status"
  }
}
```

### 5.2 Get Play Session Status

```http
GET /api/v1/play/sessions/{sessionId}
```

**Response (200 OK):**
```json
{
  "sessionId": "PLAY-20251218-001",
  "robotId": "PCR-ABC123456789",
  "petId": "PET-DOG001",
  "playType": "laser_chase",
  "startTime": "2025-12-18T11:00:00Z",
  "currentTime": "2025-12-18T11:07:30Z",
  "elapsedTime": 450,
  "remainingTime": 450,
  "status": "active",
  "intensity": "medium",
  "engagement": {
    "level": 85,
    "interactions": 42,
    "attentionSpan": 320
  },
  "activity": {
    "movementDetected": true,
    "distanceTraveled": 145.5,
    "averageSpeed": 0.32,
    "caloriesBurned": 18
  },
  "adjustments": [
    {
      "timestamp": "2025-12-18T11:05:00Z",
      "adjustment": "Increased speed based on high engagement",
      "previousIntensity": "low",
      "newIntensity": "medium"
    }
  ]
}
```

### 5.3 Stop Play Session

```http
POST /api/v1/play/sessions/{sessionId}/stop
Content-Type: application/json
```

**Request Body:**
```json
{
  "reason": "manual_stop"
}
```

**Response (200 OK):**
```json
{
  "sessionId": "PLAY-20251218-001",
  "status": "completed",
  "endTime": "2025-12-18T11:07:45Z",
  "totalDuration": 465,
  "terminationReason": "manual_stop",
  "summary": {
    "engagement": 85,
    "interactions": 42,
    "distanceTraveled": 145.5,
    "caloriesBurned": 18
  }
}
```

### 5.4 Get Play History

```http
GET /api/v1/play/history
```

**Query Parameters:**
| Parameter | Type | Description |
|-----------|------|-------------|
| `petId` | string | Filter by pet |
| `robotId` | string | Filter by robot |
| `playType` | string | Filter by play type |
| `startDate` | date | Start date |
| `endDate` | date | End date |

**Response (200 OK):**
```json
{
  "data": [
    {
      "sessionId": "PLAY-20251218-001",
      "petId": "PET-DOG001",
      "playType": "laser_chase",
      "startTime": "2025-12-18T11:00:00Z",
      "endTime": "2025-12-18T11:15:00Z",
      "duration": 900,
      "engagement": 85,
      "caloriesBurned": 32,
      "distanceTraveled": 280.5
    },
    {
      "sessionId": "PLAY-20251217-003",
      "petId": "PET-DOG001",
      "playType": "ball_fetch",
      "startTime": "2025-12-17T15:30:00Z",
      "endTime": "2025-12-17T15:50:00Z",
      "duration": 1200,
      "engagement": 92,
      "caloriesBurned": 68,
      "distanceTraveled": 520.3
    }
  ],
  "summary": {
    "totalSessions": 2,
    "totalPlayTime": 2100,
    "averageEngagement": 88.5,
    "totalCaloriesBurned": 100,
    "totalDistance": 800.8
  }
}
```

### 5.5 Schedule Play Session

```http
POST /api/v1/play/schedule
Content-Type: application/json
```

**Request Body:**
```json
{
  "robotId": "PCR-ABC123456789",
  "petId": "PET-DOG001",
  "schedule": {
    "name": "Afternoon Play Time",
    "time": "15:00:00",
    "timezone": "America/New_York",
    "recurring": true,
    "daysOfWeek": ["MON", "WED", "FRI"]
  },
  "playType": "laser_chase",
  "duration": 600,
  "intensity": "adaptive"
}
```

**Response (201 Created):**
```json
{
  "scheduleId": "SCHED-PLAY-001",
  "robotId": "PCR-ABC123456789",
  "petId": "PET-DOG001",
  "schedule": {
    "name": "Afternoon Play Time",
    "nextExecution": "2025-12-19T15:00:00-05:00"
  },
  "enabled": true,
  "createdAt": "2025-12-18T11:20:00Z"
}
```

---

## 6. Health Monitoring Endpoints

### 6.1 Record Health Observation

```http
POST /api/v1/health/observations
Content-Type: application/json
```

**Request Body:**
```json
{
  "robotId": "PCR-ABC123456789",
  "petId": "PET-DOG001",
  "observationType": "weight_measurement",
  "timestamp": "2025-12-18T12:00:00Z",
  "weight": {
    "value": 22.5,
    "unit": "kg"
  },
  "notes": "Regular weekly weigh-in"
}
```

**Response (201 Created):**
```json
{
  "observationId": "OBS-20251218-001",
  "robotId": "PCR-ABC123456789",
  "petId": "PET-DOG001",
  "observationType": "weight_measurement",
  "timestamp": "2025-12-18T12:00:00Z",
  "weight": {
    "value": 22.5,
    "unit": "kg",
    "trend": "stable",
    "changePercent": 0.2
  },
  "alerts": [],
  "createdAt": "2025-12-18T12:00:15Z"
}
```

### 6.2 Get Health Observations

```http
GET /api/v1/health/observations
```

**Query Parameters:**
| Parameter | Type | Description |
|-----------|------|-------------|
| `petId` | string | Filter by pet (required) |
| `observationType` | string | Filter by observation type |
| `startDate` | date | Start date |
| `endDate` | date | End date |

**Response (200 OK):**
```json
{
  "data": [
    {
      "observationId": "OBS-20251218-001",
      "petId": "PET-DOG001",
      "observationType": "weight_measurement",
      "timestamp": "2025-12-18T12:00:00Z",
      "weight": {
        "value": 22.5,
        "unit": "kg",
        "trend": "stable"
      }
    },
    {
      "observationId": "OBS-20251218-002",
      "petId": "PET-DOG001",
      "observationType": "activity_level",
      "timestamp": "2025-12-18T10:00:00Z",
      "activityLevel": {
        "level": "normal",
        "duration": 180,
        "intensityScore": 72
      }
    }
  ],
  "trends": {
    "weight": {
      "current": 22.5,
      "weekAgo": 22.4,
      "change": "+0.1 kg",
      "trend": "stable"
    },
    "activity": {
      "averageDailyMinutes": 165,
      "trend": "increasing"
    }
  }
}
```

### 6.3 Get Health Alerts

```http
GET /api/v1/health/alerts
```

**Query Parameters:**
| Parameter | Type | Description |
|-----------|------|-------------|
| `petId` | string | Filter by pet |
| `severity` | string | Filter by severity (info/low/medium/high/urgent) |
| `acknowledged` | boolean | Filter by acknowledgment status |

**Response (200 OK):**
```json
{
  "data": [
    {
      "alertId": "ALERT-HEALTH-001",
      "petId": "PET-DOG001",
      "alertType": "eating_change",
      "severity": "medium",
      "message": "Food consumption below normal for 2 consecutive meals",
      "timestamp": "2025-12-18T12:30:00Z",
      "recommendation": "Monitor for additional symptoms. Contact vet if continues beyond 24 hours.",
      "relatedObservations": [
        "OBS-20251218-003",
        "OBS-20251218-004"
      ],
      "acknowledged": false
    }
  ],
  "summary": {
    "total": 1,
    "byeSeverity": {
      "urgent": 0,
      "high": 0,
      "medium": 1,
      "low": 0,
      "info": 0
    }
  }
}
```

### 6.4 Acknowledge Health Alert

```http
POST /api/v1/health/alerts/{alertId}/acknowledge
Content-Type: application/json
```

**Request Body:**
```json
{
  "acknowledgedBy": "owner",
  "notes": "Monitoring closely. Will contact vet if no improvement by tomorrow.",
  "actionTaken": "Offered different food options"
}
```

**Response (200 OK):**
```json
{
  "alertId": "ALERT-HEALTH-001",
  "acknowledged": true,
  "acknowledgedAt": "2025-12-18T13:00:00Z",
  "acknowledgedBy": "owner",
  "notes": "Monitoring closely. Will contact vet if no improvement by tomorrow."
}
```

---

## 7. Pet Profile Endpoints

### 7.1 Create Pet Profile

```http
POST /api/v1/pets
Content-Type: application/json
```

**Request Body:**
```json
{
  "name": "Max",
  "species": "dog",
  "breed": "Golden Retriever",
  "birthDate": "2020-03-15",
  "gender": "male",
  "neutered": true,
  "identification": {
    "microchipId": "982000123456789",
    "rfidTag": "RFID-MAX-001"
  },
  "dietaryRequirements": {
    "foodType": ["dry_kibble"],
    "portionSize": 400,
    "feedingFrequency": 2,
    "allergies": ["chicken"],
    "restrictions": []
  },
  "preferences": {
    "favoritePlayType": ["ball_fetch", "laser_chase"],
    "playIntensity": "high",
    "treatPreference": ["peanut_butter", "dental_chews"]
  }
}
```

**Response (201 Created):**
```json
{
  "petId": "PET-DOG001",
  "name": "Max",
  "species": "dog",
  "breed": "Golden Retriever",
  "age": {
    "years": 5,
    "months": 9
  },
  "identification": {
    "microchipId": "982000123456789",
    "rfidTag": "RFID-MAX-001",
    "facialRecognition": {
      "enabled": false,
      "modelId": null
    }
  },
  "createdAt": "2025-12-18T13:10:00Z",
  "links": {
    "self": "/api/v1/pets/PET-DOG001",
    "health": "/api/v1/pets/PET-DOG001/health",
    "feeding": "/api/v1/pets/PET-DOG001/feeding"
  }
}
```

### 7.2 Get Pet Profile

```http
GET /api/v1/pets/{petId}
```

**Response (200 OK):**
```json
{
  "petId": "PET-DOG001",
  "name": "Max",
  "species": "dog",
  "breed": "Golden Retriever",
  "birthDate": "2020-03-15",
  "age": {
    "years": 5,
    "months": 9,
    "days": 3
  },
  "gender": "male",
  "neutered": true,
  "identification": {
    "microchipId": "982000123456789",
    "rfidTag": "RFID-MAX-001",
    "facialRecognition": {
      "enabled": true,
      "modelId": "FR-MODEL-001",
      "confidence": 95
    }
  },
  "dietaryRequirements": {
    "foodType": ["dry_kibble"],
    "portionSize": 400,
    "feedingFrequency": 2,
    "allergies": ["chicken"],
    "restrictions": []
  },
  "preferences": {
    "favoritePlayType": ["ball_fetch", "laser_chase"],
    "playIntensity": "high",
    "treatPreference": ["peanut_butter", "dental_chews"]
  },
  "statistics": {
    "currentWeight": 22.5,
    "averageDailyActivity": 165,
    "totalFeedingsThisWeek": 14,
    "totalPlayTimeThisWeek": 420
  },
  "createdAt": "2025-08-01T10:00:00Z",
  "updatedAt": "2025-12-18T13:10:00Z"
}
```

### 7.3 List Pets

```http
GET /api/v1/pets
```

**Response (200 OK):**
```json
{
  "data": [
    {
      "petId": "PET-DOG001",
      "name": "Max",
      "species": "dog",
      "breed": "Golden Retriever",
      "age": {
        "years": 5,
        "months": 9
      },
      "currentWeight": 22.5,
      "lastSeen": "2025-12-18T12:00:00Z"
    },
    {
      "petId": "PET-CAT001",
      "name": "Luna",
      "species": "cat",
      "breed": "Domestic Shorthair",
      "age": {
        "years": 3,
        "months": 2
      },
      "currentWeight": 4.8,
      "lastSeen": "2025-12-18T11:30:00Z"
    }
  ],
  "total": 2
}
```

### 7.4 Update Pet Profile

```http
PATCH /api/v1/pets/{petId}
Content-Type: application/json
```

**Request Body:**
```json
{
  "dietaryRequirements": {
    "portionSize": 350,
    "allergies": ["chicken", "corn"]
  }
}
```

**Response (200 OK):**
```json
{
  "petId": "PET-DOG001",
  "message": "Pet profile updated successfully",
  "updatedFields": ["dietaryRequirements"],
  "updatedAt": "2025-12-18T13:20:00Z"
}
```

### 7.5 Delete Pet Profile

```http
DELETE /api/v1/pets/{petId}
```

**Response (200 OK):**
```json
{
  "petId": "PET-DOG001",
  "message": "Pet profile deleted successfully",
  "dataRetention": {
    "healthRecords": "retained for 7 years",
    "feedingHistory": "retained for 1 year",
    "playHistory": "deleted immediately"
  },
  "deletedAt": "2025-12-18T13:25:00Z"
}
```

---

## 8. Scheduling Endpoints

### 8.1 Create Routine

```http
POST /api/v1/schedules/routines
Content-Type: application/json
```

**Request Body:**
```json
{
  "name": "Morning Care Routine",
  "enabled": true,
  "trigger": {
    "type": "time",
    "time": "07:00:00",
    "timezone": "America/New_York",
    "daysOfWeek": ["MON", "TUE", "WED", "THU", "FRI"]
  },
  "actions": [
    {
      "order": 1,
      "type": "feeding",
      "robotId": "PCR-ABC123456789",
      "petId": "PET-DOG001",
      "parameters": {
        "portionSize": 150,
        "foodType": "dry_kibble"
      }
    },
    {
      "order": 2,
      "type": "play_session",
      "robotId": "PCR-ABC123456789",
      "petId": "PET-DOG001",
      "parameters": {
        "playType": "laser_chase",
        "duration": 300,
        "intensity": "medium"
      },
      "delay": 1800
    }
  ]
}
```

**Response (201 Created):**
```json
{
  "routineId": "ROUTINE-001",
  "name": "Morning Care Routine",
  "enabled": true,
  "nextExecution": "2025-12-19T07:00:00-05:00",
  "createdAt": "2025-12-18T13:30:00Z"
}
```

### 8.2 List Routines

```http
GET /api/v1/schedules/routines
```

**Response (200 OK):**
```json
{
  "data": [
    {
      "routineId": "ROUTINE-001",
      "name": "Morning Care Routine",
      "enabled": true,
      "nextExecution": "2025-12-19T07:00:00-05:00",
      "lastExecution": "2025-12-18T07:00:00-05:00",
      "executionCount": 25,
      "successRate": 100
    }
  ]
}
```

### 8.3 Get Calendar View

```http
GET /api/v1/schedules/calendar
```

**Query Parameters:**
| Parameter | Type | Description |
|-----------|------|-------------|
| `startDate` | date | Start date (required) |
| `endDate` | date | End date (required) |
| `petId` | string | Filter by pet |
| `robotId` | string | Filter by robot |

**Response (200 OK):**
```json
{
  "startDate": "2025-12-18",
  "endDate": "2025-12-24",
  "events": [
    {
      "eventId": "CAL-001",
      "date": "2025-12-18",
      "time": "07:00:00",
      "type": "feeding",
      "description": "Morning Breakfast - Max",
      "robotId": "PCR-ABC123456789",
      "petId": "PET-DOG001",
      "status": "completed"
    },
    {
      "eventId": "CAL-002",
      "date": "2025-12-18",
      "time": "15:00:00",
      "type": "play_session",
      "description": "Afternoon Play Time - Max",
      "robotId": "PCR-ABC123456789",
      "petId": "PET-DOG001",
      "status": "scheduled"
    }
  ],
  "summary": {
    "totalEvents": 42,
    "byType": {
      "feeding": 28,
      "play_session": 12,
      "health_check": 2
    }
  }
}
```

### 8.4 Update Routine

```http
PATCH /api/v1/schedules/routines/{routineId}
Content-Type: application/json
```

**Request Body:**
```json
{
  "enabled": false
}
```

**Response (200 OK):**
```json
{
  "routineId": "ROUTINE-001",
  "message": "Routine updated successfully",
  "enabled": false,
  "updatedAt": "2025-12-18T13:40:00Z"
}
```

---

## 9. Smart Home Integration Endpoints

### 9.1 Get Voice Assistant Commands

```http
GET /api/v1/integrations/voice/commands
```

**Response (200 OK):**
```json
{
  "platform": "alexa",
  "commands": [
    {
      "command": "Alexa, feed my pet",
      "action": "manual_feeding",
      "description": "Dispense regular portion of food"
    },
    {
      "command": "Alexa, start play time",
      "action": "start_play_session",
      "description": "Begin interactive play session"
    },
    {
      "command": "Alexa, ask PetCare about Max",
      "action": "pet_status",
      "description": "Get pet status and recent activity"
    },
    {
      "command": "Alexa, what's the food level?",
      "action": "supply_status",
      "description": "Check food and water levels"
    }
  ]
}
```

### 9.2 Configure Smart Home Integration

```http
POST /api/v1/integrations/smarthome
Content-Type: application/json
```

**Request Body:**
```json
{
  "platform": "google_home",
  "accountLinking": {
    "authorizationCode": "AUTH-CODE-123"
  },
  "devices": [
    {
      "robotId": "PCR-ABC123456789",
      "roomName": "Living Room",
      "customName": "Pet Feeder"
    }
  ],
  "enabledFeatures": [
    "voice_feeding",
    "voice_play",
    "status_queries",
    "routines"
  ]
}
```

**Response (201 Created):**
```json
{
  "integrationId": "INT-GH-001",
  "platform": "google_home",
  "status": "active",
  "linkedDevices": 1,
  "setupComplete": true,
  "message": "Google Home integration configured successfully"
}
```

### 9.3 Get Integration Status

```http
GET /api/v1/integrations
```

**Response (200 OK):**
```json
{
  "data": [
    {
      "integrationId": "INT-ALEXA-001",
      "platform": "alexa",
      "status": "active",
      "linkedDevices": 1,
      "lastSync": "2025-12-18T13:00:00Z",
      "features": {
        "voice_control": true,
        "routines": true,
        "notifications": true
      }
    },
    {
      "integrationId": "INT-GH-001",
      "platform": "google_home",
      "status": "active",
      "linkedDevices": 1,
      "lastSync": "2025-12-18T12:45:00Z",
      "features": {
        "voice_control": true,
        "routines": true,
        "notifications": false
      }
    }
  ]
}
```

---

## 10. Media Endpoints

### 10.1 Get Live Stream

```http
GET /api/v1/media/stream/{robotId}
```

**Response (200 OK):**
```json
{
  "robotId": "PCR-ABC123456789",
  "streamUrl": "wss://stream.petcare.wia.org/live/PCR-ABC123456789",
  "protocol": "WebRTC",
  "resolution": "1080p",
  "frameRate": 30,
  "bitrate": 2500,
  "audio": true,
  "expiresAt": "2025-12-18T15:00:00Z",
  "token": "STREAM-TOKEN-XYZ"
}
```

### 10.2 List Recordings

```http
GET /api/v1/media/recordings
```

**Query Parameters:**
| Parameter | Type | Description |
|-----------|------|-------------|
| `robotId` | string | Filter by robot |
| `startDate` | date | Start date |
| `endDate` | date | End date |
| `eventType` | string | Filter by event type |

**Response (200 OK):**
```json
{
  "data": [
    {
      "recordingId": "REC-001",
      "robotId": "PCR-ABC123456789",
      "timestamp": "2025-12-18T10:00:00Z",
      "duration": 180,
      "eventType": "motion_detected",
      "thumbnailUrl": "https://cdn.petcare.wia.org/thumbnails/REC-001.jpg",
      "videoUrl": "https://cdn.petcare.wia.org/videos/REC-001.mp4",
      "fileSize": 45678900
    }
  ],
  "pagination": {
    "page": 1,
    "limit": 20,
    "total": 5
  }
}
```

### 10.3 Capture Snapshot

```http
POST /api/v1/media/snapshot
Content-Type: application/json
```

**Request Body:**
```json
{
  "robotId": "PCR-ABC123456789",
  "quality": "high"
}
```

**Response (200 OK):**
```json
{
  "snapshotId": "SNAP-001",
  "robotId": "PCR-ABC123456789",
  "timestamp": "2025-12-18T14:00:00Z",
  "imageUrl": "https://cdn.petcare.wia.org/snapshots/SNAP-001.jpg",
  "resolution": "1920x1080",
  "fileSize": 2456789
}
```

---

## 11. Analytics Endpoints

### 11.1 Get Activity Report

```http
GET /api/v1/analytics/activity
```

**Query Parameters:**
| Parameter | Type | Description |
|-----------|------|-------------|
| `petId` | string | Pet ID (required) |
| `period` | string | Period (day/week/month) |
| `startDate` | date | Start date |
| `endDate` | date | End date |

**Response (200 OK):**
```json
{
  "petId": "PET-DOG001",
  "period": "week",
  "startDate": "2025-12-11",
  "endDate": "2025-12-18",
  "feeding": {
    "totalMeals": 14,
    "totalFood": 2100,
    "averagePerMeal": 150,
    "missedMeals": 0,
    "onTimeRate": 100
  },
  "play": {
    "totalSessions": 5,
    "totalMinutes": 75,
    "averageEngagement": 87,
    "caloriesBurned": 245,
    "distanceTraveled": 1450
  },
  "health": {
    "weightChange": "+0.1 kg",
    "activityLevel": "normal",
    "alerts": 0
  },
  "recommendations": [
    "Activity levels are excellent - maintain current routine",
    "Consider increasing play variety with different toy types"
  ]
}
```

### 11.2 Get Nutrition Report

```http
GET /api/v1/analytics/nutrition
```

**Response (200 OK):**
```json
{
  "petId": "PET-DOG001",
  "period": "week",
  "totalCalories": 8400,
  "dailyAverage": 1200,
  "recommendedDaily": 1150,
  "variance": "+4.3%",
  "macronutrients": {
    "protein": 2100,
    "fat": 840,
    "carbohydrates": 4200,
    "fiber": 420
  },
  "trends": {
    "consumption": "stable",
    "appetite": "normal",
    "weightImpact": "minimal_gain"
  },
  "recommendations": [
    "Reduce daily portion by 25g to maintain ideal weight",
    "Consider adding more fiber-rich vegetables"
  ]
}
```

### 11.3 Get Behavioral Insights

```http
GET /api/v1/analytics/behavior
```

**Response (200 OK):**
```json
{
  "petId": "PET-DOG001",
  "period": "month",
  "patterns": [
    {
      "pattern": "high_energy_morning",
      "description": "Consistently shows higher activity between 6-9 AM",
      "confidence": 92,
      "recommendation": "Schedule play sessions in the morning for optimal engagement"
    },
    {
      "pattern": "food_preference",
      "description": "Faster eating speed with dry kibble vs wet food",
      "confidence": 85,
      "recommendation": "Consider slow-feed bowl for dry kibble"
    }
  ],
  "anomalies": [
    {
      "date": "2025-12-15",
      "anomaly": "reduced_activity",
      "severity": "low",
      "description": "Activity 30% below average",
      "possibleCauses": ["weather", "temporary fatigue"],
      "resolved": true
    }
  ]
}
```

### 11.4 Get Predictive Analytics

```http
GET /api/v1/analytics/predictions
```

**Response (200 OK):**
```json
{
  "petId": "PET-DOG001",
  "predictions": [
    {
      "type": "supply_depletion",
      "item": "food",
      "estimatedDepletion": "2025-12-22T14:00:00Z",
      "confidence": 88,
      "recommendation": "Restock by December 21"
    },
    {
      "type": "activity_trend",
      "metric": "play_engagement",
      "trend": "increasing",
      "estimatedValue": 92,
      "timeframe": "next_week",
      "confidence": 76
    },
    {
      "type": "health_alert",
      "risk": "weight_gain",
      "probability": 0.15,
      "timeframe": "30_days",
      "preventionSteps": [
        "Reduce portion size by 10%",
        "Increase play sessions to 3x weekly"
      ]
    }
  ]
}
```

---

## 12. Webhook Endpoints

### 12.1 Create Webhook Subscription

```http
POST /api/v1/webhooks
Content-Type: application/json
```

**Request Body:**
```json
{
  "url": "https://your-server.com/webhooks/petcare",
  "events": [
    "feeding.completed",
    "play.started",
    "play.completed",
    "health.alert.created",
    "robot.status.changed",
    "supply.low"
  ],
  "secret": "your-webhook-secret",
  "enabled": true
}
```

**Response (201 Created):**
```json
{
  "webhookId": "WH-001",
  "url": "https://your-server.com/webhooks/petcare",
  "events": [
    "feeding.completed",
    "play.started",
    "play.completed",
    "health.alert.created",
    "robot.status.changed",
    "supply.low"
  ],
  "secret": "your-webhook-secret",
  "enabled": true,
  "createdAt": "2025-12-18T14:30:00Z",
  "verificationStatus": "pending"
}
```

### 12.2 List Webhooks

```http
GET /api/v1/webhooks
```

**Response (200 OK):**
```json
{
  "data": [
    {
      "webhookId": "WH-001",
      "url": "https://your-server.com/webhooks/petcare",
      "events": ["feeding.completed", "play.started"],
      "enabled": true,
      "verificationStatus": "verified",
      "lastDelivery": "2025-12-18T14:25:00Z",
      "deliveryStats": {
        "successful": 245,
        "failed": 2,
        "successRate": 99.2
      }
    }
  ]
}
```

### 12.3 Delete Webhook

```http
DELETE /api/v1/webhooks/{webhookId}
```

**Response (200 OK):**
```json
{
  "webhookId": "WH-001",
  "message": "Webhook deleted successfully",
  "deletedAt": "2025-12-18T14:35:00Z"
}
```

---

## 13. Error Handling

### 13.1 Error Response Format

```json
{
  "error": {
    "code": "ROBOT_NOT_FOUND",
    "message": "Robot with ID 'PCR-INVALID123' not found",
    "statusCode": 404,
    "timestamp": "2025-12-18T14:40:00Z",
    "requestId": "req-abc123",
    "details": {
      "robotId": "PCR-INVALID123",
      "suggestion": "Verify robot ID format: PCR-[A-Z0-9]{12}"
    }
  }
}
```

### 13.2 Error Codes

| HTTP Status | Error Code | Description | Resolution |
|-------------|-----------|-------------|------------|
| 400 | `INVALID_REQUEST` | Malformed request data | Check request format |
| 400 | `VALIDATION_ERROR` | Field validation failed | Review field requirements |
| 401 | `UNAUTHORIZED` | Missing or invalid authentication | Provide valid credentials |
| 403 | `FORBIDDEN` | Insufficient permissions | Check scope permissions |
| 404 | `ROBOT_NOT_FOUND` | Robot does not exist | Verify robot ID |
| 404 | `PET_NOT_FOUND` | Pet profile not found | Verify pet ID |
| 409 | `ROBOT_BUSY` | Robot currently executing operation | Wait or cancel current operation |
| 409 | `SCHEDULE_CONFLICT` | Time slot already scheduled | Choose different time |
| 422 | `SUPPLY_EMPTY` | Food or water depleted | Refill supplies |
| 429 | `RATE_LIMIT_EXCEEDED` | Too many requests | Reduce request frequency |
| 500 | `INTERNAL_ERROR` | Server error | Retry or contact support |
| 503 | `SERVICE_UNAVAILABLE` | Service temporarily down | Retry later |

---

## 14. Rate Limiting

### 14.1 Rate Limit Headers

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 995
X-RateLimit-Reset: 1703001600
```

### 14.2 Rate Limit Tiers

| Tier | Requests/Hour | Burst | Use Case |
|------|---------------|-------|----------|
| **Free** | 100 | 10 | Personal use |
| **Basic** | 1,000 | 50 | Small households |
| **Pro** | 10,000 | 200 | Multi-device homes |
| **Enterprise** | 100,000 | 1,000 | Commercial applications |

---

## 15. Webhook Event Payloads

### 15.1 Feeding Completed Event

```json
{
  "event": "feeding.completed",
  "timestamp": "2025-12-18T15:00:00Z",
  "data": {
    "eventId": "FEED-20251218-001",
    "robotId": "PCR-ABC123456789",
    "petId": "PET-DOG001",
    "feedingType": "scheduled",
    "portionSize": 150,
    "dispensed": true,
    "consumed": {
      "detected": true,
      "estimatedAmount": 148
    }
  }
}
```

### 15.2 Health Alert Event

```json
{
  "event": "health.alert.created",
  "timestamp": "2025-12-18T15:05:00Z",
  "data": {
    "alertId": "ALERT-HEALTH-002",
    "petId": "PET-DOG001",
    "alertType": "weight_change",
    "severity": "medium",
    "message": "Weight increased by 5% in the past week",
    "recommendation": "Review feeding portions and activity levels"
  }
}
```

### 15.3 Supply Low Event

```json
{
  "event": "supply.low",
  "timestamp": "2025-12-18T15:10:00Z",
  "data": {
    "robotId": "PCR-ABC123456789",
    "supplyType": "food",
    "currentLevel": 18,
    "threshold": 20,
    "estimatedDepletion": "2025-12-19T18:00:00Z",
    "message": "Food supply below 20% - refill recommended"
  }
}
```

---

**弘益人間 (홍익인간)** - Benefit All Humanity
© 2025 WIA
MIT License
