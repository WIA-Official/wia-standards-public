# WIA-AGRI-030: Food Crisis Response
## PHASE 2 - API INTERFACE SPECIFICATION

**Version:** 1.0
**Status:** ✅ Complete
**Last Updated:** 2025-12-26

---

## 1. Overview

This phase defines RESTful API interfaces for the Food Crisis Response system, enabling real-time crisis reporting, resource coordination, and emergency response management.

### 1.1 Base URL

```
Production: https://api.wia-agri.org/v1/crisis
Staging: https://staging-api.wia-agri.org/v1/crisis
```

### 1.2 Authentication

All API requests require authentication using API keys or OAuth 2.0:

```http
Authorization: Bearer YOUR_API_KEY
```

---

## 2. Crisis Management APIs

### 2.1 Report Crisis Event

Report a new food crisis event or update an existing one.

**Endpoint:** `POST /crisis/report`

**Request Body:**
```json
{
  "type": "drought",
  "severity": 3,
  "region": {
    "country": "Somalia",
    "province": "Bay",
    "coordinates": { "lat": 2.6857, "lng": 44.0153 }
  },
  "population": {
    "total": 800000,
    "affected": 650000,
    "children": 280000
  },
  "triggers": [
    {
      "type": "rainfall-deficit",
      "value": -75,
      "unit": "percent"
    }
  ],
  "reportedBy": "FAO-Somalia",
  "contactEmail": "crisis@fao-somalia.org"
}
```

**Response (201 Created):**
```json
{
  "success": true,
  "crisisId": "CRISIS-2025-SO-001",
  "status": "RECEIVED",
  "message": "Crisis report successfully logged",
  "assessment": {
    "initiated": true,
    "estimatedCompletion": "2025-01-17T12:00:00Z"
  },
  "notifications": {
    "sent": 14,
    "stakeholders": ["WFP", "UNICEF", "Government-Somalia", "Red-Cross"]
  },
  "nextSteps": [
    "Automated severity analysis in progress",
    "Field assessment team will be deployed within 24 hours",
    "Resource allocation calculation started"
  ]
}
```

### 2.2 Get Crisis Details

Retrieve detailed information about a specific crisis.

**Endpoint:** `GET /crisis/{crisisId}`

**Response (200 OK):**
```json
{
  "crisisId": "CRISIS-2025-SO-001",
  "type": "drought",
  "severity": 3,
  "status": "active",
  "region": {
    "country": "Somalia",
    "province": "Bay",
    "coordinates": { "lat": 2.6857, "lng": 44.0153 },
    "affectedArea": 12500
  },
  "population": {
    "total": 800000,
    "affected": 650000,
    "displaced": 85000,
    "children": 280000
  },
  "timeline": {
    "detectedAt": "2025-01-15T08:30:00Z",
    "reportedAt": "2025-01-15T09:45:00Z",
    "lastUpdated": "2025-01-16T14:20:00Z",
    "estimatedDuration": 180
  },
  "currentPhase": {
    "ipc": 4,
    "description": "Emergency"
  },
  "responseStatus": {
    "assessmentComplete": true,
    "resourcesAllocated": 65,
    "distributionActive": true
  }
}
```

### 2.3 List Active Crises

Retrieve all active crises, optionally filtered by region or severity.

**Endpoint:** `GET /crisis/list`

**Query Parameters:**
- `status` (optional): active|monitoring|resolved
- `severity` (optional): 1|2|3
- `country` (optional): ISO country code
- `limit` (default: 50, max: 100)
- `offset` (default: 0)

**Response (200 OK):**
```json
{
  "total": 12,
  "limit": 50,
  "offset": 0,
  "crises": [
    {
      "crisisId": "CRISIS-2025-SO-001",
      "type": "drought",
      "severity": 3,
      "country": "Somalia",
      "affectedPopulation": 650000,
      "status": "active",
      "reportedAt": "2025-01-15T09:45:00Z"
    },
    {
      "crisisId": "CRISIS-2025-YE-002",
      "type": "conflict",
      "severity": 3,
      "country": "Yemen",
      "affectedPopulation": 2400000,
      "status": "active",
      "reportedAt": "2025-01-10T14:20:00Z"
    }
  ]
}
```

### 2.4 Update Crisis Status

Update the status or details of an existing crisis.

**Endpoint:** `PATCH /crisis/{crisisId}`

**Request Body:**
```json
{
  "status": "monitoring",
  "severity": 2,
  "population": {
    "affected": 450000
  },
  "notes": "Situation improving due to emergency aid delivery"
}
```

**Response (200 OK):**
```json
{
  "success": true,
  "crisisId": "CRISIS-2025-SO-001",
  "updated": ["status", "severity", "population"],
  "timestamp": "2025-01-20T10:30:00Z"
}
```

---

## 3. Assessment APIs

### 3.1 Submit Food Security Assessment

Submit a comprehensive food security assessment.

**Endpoint:** `POST /assessment/submit`

**Request Body:**
```json
{
  "crisisId": "CRISIS-2025-SO-001",
  "ipcPhase": {
    "current": 4,
    "projected": 3,
    "projectionDate": "2025-04-01"
  },
  "indicators": {
    "foodConsumptionScore": {
      "poor": 52,
      "borderline": 31,
      "acceptable": 17
    },
    "malnutrition": {
      "gam": 16.8,
      "sam": 3.9
    }
  },
  "needs": {
    "food": {
      "cereals": 12000,
      "pulses": 2400,
      "oil": 600
    }
  },
  "assessmentMethod": "household-survey",
  "sampleSize": 1800,
  "assessor": "WFP-Assessment-Team-3"
}
```

**Response (201 Created):**
```json
{
  "success": true,
  "assessmentId": "FSA-2025-SO-001",
  "crisisId": "CRISIS-2025-SO-001",
  "status": "VALIDATED",
  "resourceAllocation": {
    "triggered": true,
    "priority": "critical",
    "estimatedCost": 8500000
  }
}
```

### 3.2 Get Assessment Details

**Endpoint:** `GET /assessment/{assessmentId}`

**Response (200 OK):**
```json
{
  "assessmentId": "FSA-2025-SO-001",
  "crisisId": "CRISIS-2025-SO-001",
  "timestamp": "2025-01-16T12:00:00Z",
  "ipcPhase": {
    "current": 4,
    "projected": 3
  },
  "indicators": { ... },
  "needs": { ... },
  "confidence": 0.93
}
```

---

## 4. Resource Management APIs

### 4.1 Check Resource Availability

Query available resources at specific locations.

**Endpoint:** `GET /resources/availability`

**Query Parameters:**
- `location` (required): facility ID or coordinates
- `category` (optional): food|nutrition|water|medical
- `radius` (optional): search radius in km

**Response (200 OK):**
```json
{
  "query": {
    "location": "WFP-NAIROBI-01",
    "radius": 500
  },
  "facilities": [
    {
      "facilityId": "WFP-NAIROBI-01",
      "location": "Nairobi, Kenya",
      "distance": 0,
      "items": [
        {
          "category": "food",
          "type": "wheat-flour",
          "available": 8500,
          "unit": "metric-tons"
        },
        {
          "category": "nutrition",
          "type": "RUTF",
          "available": 450000,
          "unit": "sachets"
        }
      ]
    },
    {
      "facilityId": "WFP-ADDIS-01",
      "location": "Addis Ababa, Ethiopia",
      "distance": 1100,
      "items": [ ... ]
    }
  ]
}
```

### 4.2 Reserve Resources

Reserve resources for a specific crisis response.

**Endpoint:** `POST /resources/reserve`

**Request Body:**
```json
{
  "crisisId": "CRISIS-2025-SO-001",
  "facilityId": "WFP-NAIROBI-01",
  "items": [
    {
      "type": "wheat-flour",
      "quantity": 2000,
      "unit": "metric-tons"
    },
    {
      "type": "RUTF",
      "quantity": 150000,
      "unit": "sachets"
    }
  ],
  "reservedBy": "WFP-Response-Coordinator",
  "validUntil": "2025-02-15T23:59:59Z"
}
```

**Response (201 Created):**
```json
{
  "success": true,
  "reservationId": "RES-2025-001",
  "status": "CONFIRMED",
  "items": [
    {
      "type": "wheat-flour",
      "quantity": 2000,
      "reserved": true
    },
    {
      "type": "RUTF",
      "quantity": 150000,
      "reserved": true
    }
  ],
  "validUntil": "2025-02-15T23:59:59Z",
  "releaseInstructions": "Contact logistics@wfp.org to initiate shipment"
}
```

---

## 5. Distribution and Tracking APIs

### 5.1 Create Shipment

Create a new aid shipment.

**Endpoint:** `POST /distribution/shipment`

**Request Body:**
```json
{
  "crisisId": "CRISIS-2025-SO-001",
  "priority": "critical",
  "origin": {
    "facilityId": "WFP-NAIROBI-01"
  },
  "destination": {
    "distributionPointId": "DP-SO-023",
    "coordinates": { "lat": 2.6857, "lng": 44.0153 },
    "beneficiaries": 25000
  },
  "cargo": [
    {
      "type": "wheat-flour",
      "quantity": 500,
      "unit": "metric-tons"
    }
  ],
  "scheduledDeparture": "2025-01-18T06:00:00Z"
}
```

**Response (201 Created):**
```json
{
  "success": true,
  "shipmentId": "SHIP-2025-045",
  "status": "PLANNED",
  "eta": "2025-01-21T14:00:00Z",
  "trackingUrl": "https://track.wia-agri.org/SHIP-2025-045",
  "blockchainHash": "0x8b4f2a..."
}
```

### 5.2 Track Shipment

Get real-time tracking information for a shipment.

**Endpoint:** `GET /distribution/track/{shipmentId}`

**Response (200 OK):**
```json
{
  "shipmentId": "SHIP-2025-045",
  "status": "in-transit",
  "progress": 58,
  "currentLocation": {
    "lat": 1.4528,
    "lng": 42.8453,
    "name": "Near Beledweyne, Somalia"
  },
  "destination": {
    "distributionPointId": "DP-SO-023",
    "coordinates": { "lat": 2.6857, "lng": 44.0153 }
  },
  "timeline": {
    "departed": "2025-01-18T07:15:00Z",
    "eta": "2025-01-21T14:00:00Z",
    "estimatedDelay": 0
  },
  "lastUpdate": "2025-01-20T09:30:00Z",
  "verification": {
    "blockchainHash": "0x8b4f2a...",
    "verified": true
  }
}
```

### 5.3 Confirm Delivery

Confirm receipt of aid shipment at distribution point.

**Endpoint:** `POST /distribution/confirm/{shipmentId}`

**Request Body:**
```json
{
  "receivedAt": "2025-01-21T13:45:00Z",
  "receivedBy": "DP-Manager-SO-023",
  "condition": "good",
  "discrepancies": [],
  "beneficiariesServed": 24800,
  "signature": "DIGITAL-SIGNATURE-HASH",
  "photos": [
    "https://storage.wia-agri.org/deliveries/SHIP-2025-045-01.jpg"
  ]
}
```

**Response (200 OK):**
```json
{
  "success": true,
  "shipmentId": "SHIP-2025-045",
  "status": "DELIVERED",
  "blockchainHash": "0x9c5e3b...",
  "certificateUrl": "https://cert.wia-agri.org/delivery/SHIP-2025-045"
}
```

---

## 6. Alert and Notification APIs

### 6.1 Subscribe to Alerts

Subscribe to crisis alerts for specific regions or criteria.

**Endpoint:** `POST /alerts/subscribe`

**Request Body:**
```json
{
  "organizationId": "RED-CROSS-GLOBAL",
  "filters": {
    "countries": ["Somalia", "Ethiopia", "Kenya"],
    "severityMin": 2,
    "types": ["drought", "flood"]
  },
  "channels": [
    {
      "type": "webhook",
      "url": "https://api.redcross.org/wia/crisis-webhook"
    },
    {
      "type": "email",
      "address": "emergency@redcross.org"
    }
  ]
}
```

**Response (201 Created):**
```json
{
  "success": true,
  "subscriptionId": "SUB-2025-001",
  "status": "ACTIVE",
  "filters": { ... },
  "testAlertSent": true
}
```

### 6.2 Issue Early Warning

Issue an early warning alert (authorized organizations only).

**Endpoint:** `POST /alerts/warning`

**Request Body:**
```json
{
  "type": "food-crisis-warning",
  "severity": "warning",
  "region": {
    "countries": ["Somalia", "Kenya"]
  },
  "validUntil": "2025-04-30T23:59:59Z",
  "triggers": [
    {
      "indicator": "rainfall-forecast",
      "value": -60,
      "confidence": 0.89
    }
  ],
  "predictions": {
    "affectedPopulation": 3200000,
    "ipcPhase": 3,
    "timeline": "2025-03-01 to 2025-06-30"
  },
  "recommendations": [
    "Preposition food stocks",
    "Activate emergency procurement"
  ]
}
```

**Response (201 Created):**
```json
{
  "success": true,
  "alertId": "ALERT-2025-EA-005",
  "issued": "2025-01-15T00:00:00Z",
  "notificationsSent": 47,
  "publicUrl": "https://alerts.wia-agri.org/ALERT-2025-EA-005"
}
```

---

## 7. Analytics and Reporting APIs

### 7.1 Get Crisis Statistics

**Endpoint:** `GET /analytics/statistics`

**Query Parameters:**
- `startDate`, `endDate`: Date range
- `region`: Filter by region
- `groupBy`: day|week|month

**Response (200 OK):**
```json
{
  "period": {
    "start": "2025-01-01",
    "end": "2025-12-31"
  },
  "totalCrises": 48,
  "activeCrises": 12,
  "resolvedCrises": 36,
  "affectedPopulation": 28500000,
  "resourcesDistributed": {
    "food": 485000,
    "water": 125000000,
    "medical": 2400000
  },
  "costUSD": 1250000000,
  "byType": {
    "drought": 18,
    "conflict": 12,
    "flood": 10,
    "economic": 5,
    "pest": 2,
    "supply": 1
  }
}
```

### 7.2 Generate Crisis Report

**Endpoint:** `POST /analytics/report`

**Request Body:**
```json
{
  "crisisId": "CRISIS-2025-SO-001",
  "format": "pdf|json|xlsx",
  "sections": ["overview", "assessment", "response", "distribution", "outcomes"]
}
```

**Response (200 OK):**
```json
{
  "success": true,
  "reportId": "REPORT-2025-001",
  "downloadUrl": "https://reports.wia-agri.org/REPORT-2025-001.pdf",
  "expiresAt": "2025-02-01T00:00:00Z"
}
```

---

## 8. Error Handling

### 8.1 Error Response Format

```json
{
  "success": false,
  "error": {
    "code": "RESOURCE_NOT_FOUND",
    "message": "Crisis with ID CRISIS-2025-XX-999 not found",
    "details": {
      "crisisId": "CRISIS-2025-XX-999"
    }
  }
}
```

### 8.2 HTTP Status Codes

- `200 OK`: Successful request
- `201 Created`: Resource successfully created
- `400 Bad Request`: Invalid request parameters
- `401 Unauthorized`: Missing or invalid authentication
- `403 Forbidden`: Insufficient permissions
- `404 Not Found`: Resource not found
- `429 Too Many Requests`: Rate limit exceeded
- `500 Internal Server Error`: Server error
- `503 Service Unavailable`: Service temporarily unavailable

### 8.3 Common Error Codes

- `INVALID_REQUEST`: Request validation failed
- `AUTHENTICATION_REQUIRED`: Missing authentication
- `PERMISSION_DENIED`: Insufficient permissions
- `RESOURCE_NOT_FOUND`: Requested resource not found
- `RESOURCE_CONFLICT`: Resource already exists
- `RATE_LIMIT_EXCEEDED`: Too many requests
- `VALIDATION_ERROR`: Data validation failed

---

## 9. Rate Limiting

- Default: 1000 requests per hour per API key
- Critical endpoints (/crisis/report, /alerts/warning): 100 requests per hour
- Tracking endpoints: 5000 requests per hour
- Headers:
  - `X-RateLimit-Limit`: Maximum requests per window
  - `X-RateLimit-Remaining`: Remaining requests
  - `X-RateLimit-Reset`: Timestamp when limit resets

---

## 10. Webhooks

Subscribe to real-time events via webhooks:

**Supported Events:**
- `crisis.created`
- `crisis.updated`
- `crisis.resolved`
- `assessment.completed`
- `shipment.departed`
- `shipment.delivered`
- `alert.issued`

**Webhook Payload:**
```json
{
  "eventId": "evt_2025_001",
  "type": "crisis.created",
  "timestamp": "2025-01-15T09:45:00Z",
  "data": {
    "crisisId": "CRISIS-2025-SO-001",
    ...
  }
}
```

---

**Next Phase**: [PHASE-3-PROTOCOL.md](PHASE-3-PROTOCOL.md)

---

弘益人間 · Benefit All Humanity
© 2025 WIA Standards - MIT License
