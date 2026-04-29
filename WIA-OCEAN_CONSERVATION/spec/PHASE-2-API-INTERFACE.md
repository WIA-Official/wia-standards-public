# WIA-OCEAN_CONSERVATION: Phase 2 - API Interface Specification
## Version 1.0 | 弘益人間 (Benefit All Humanity)

---

## 1. Overview

This specification defines the REST API interfaces for the WIA-OCEAN_CONSERVATION standard, enabling species tracking, ecosystem monitoring, MPA management, illegal fishing detection, pollution tracking, and conservation alert systems.

**Base URL**: `https://api.wia.org/ocean-conservation/v1`

**Authentication**: OAuth 2.0 / API Key

---

## 2. Species Tracking APIs

### 2.1 Submit Species Observation

**Endpoint**: `POST /species/observations`

**Description**: Submit a new marine species observation record.

**Request Body**:
```json
{
  "speciesCode": "IUCN-Chelonia-mydas",
  "commonName": "Green Sea Turtle",
  "scientificName": "Chelonia mydas",
  "location": {
    "latitude": -23.5505,
    "longitude": -46.6333,
    "depth": 15.5
  },
  "observationType": "VISUAL",
  "count": 1,
  "behavior": "FEEDING",
  "timestamp": "2026-01-12T14:30:00Z",
  "observer": {
    "observerId": "uuid-v4",
    "type": "RESEARCHER"
  },
  "media": [{
    "type": "PHOTO",
    "url": "https://storage.example.com/photo.jpg"
  }]
}
```

**Response**: `201 Created`
```json
{
  "observationId": "obs-uuid-12345",
  "status": "PENDING_VERIFICATION",
  "message": "Observation submitted successfully",
  "verificationEstimate": "24 hours"
}
```

### 2.2 Get Species Observations

**Endpoint**: `GET /species/observations`

**Query Parameters**:
- `speciesCode` (optional): Filter by species
- `startDate` (optional): ISO 8601 date
- `endDate` (optional): ISO 8601 date
- `bbox` (optional): Bounding box (minLon,minLat,maxLon,maxLat)
- `verified` (optional): true/false
- `limit` (optional): Max 1000, default 100
- `offset` (optional): Pagination offset

**Response**: `200 OK`
```json
{
  "total": 1523,
  "limit": 100,
  "offset": 0,
  "observations": [
    {
      "observationId": "obs-uuid-12345",
      "speciesCode": "IUCN-Chelonia-mydas",
      "commonName": "Green Sea Turtle",
      "location": {"latitude": -23.5505, "longitude": -46.6333},
      "timestamp": "2026-01-12T14:30:00Z",
      "verified": true,
      "confidence": 0.95
    }
  ]
}
```

### 2.3 Get Species Population Trends

**Endpoint**: `GET /species/{speciesCode}/population`

**Path Parameters**:
- `speciesCode`: IUCN species identifier

**Query Parameters**:
- `region`: Geographic region
- `startYear`: Beginning year
- `endYear`: End year

**Response**: `200 OK`
```json
{
  "speciesCode": "IUCN-Chelonia-mydas",
  "region": "CARIBBEAN_SEA",
  "currentPopulation": 25000,
  "trend": "INCREASING",
  "trendRate": 0.05,
  "yearlyData": [
    {
      "year": 2024,
      "population": 23500,
      "confidence": 0.90
    },
    {
      "year": 2025,
      "population": 24200,
      "confidence": 0.92
    }
  ],
  "threats": ["BYCATCH", "HABITAT_LOSS"],
  "conservationActions": ["NEST_PROTECTION", "BYCATCH_REDUCTION"]
}
```

---

## 3. Ecosystem Monitoring APIs

### 3.1 Submit Coral Reef Assessment

**Endpoint**: `POST /ecosystems/coral-reefs/assessments`

**Request Body**:
```json
{
  "reefId": "reef-uuid",
  "assessmentDate": "2026-01-12T00:00:00Z",
  "coralCoverPercentage": 45.2,
  "bleachingLevel": "MILD",
  "bleachedPercentage": 12.5,
  "waterQuality": {
    "temperature": 28.5,
    "pH": 8.15,
    "dissolvedOxygen": 6.8
  },
  "threats": ["OCEAN_WARMING", "ACIDIFICATION"]
}
```

**Response**: `201 Created`
```json
{
  "assessmentId": "assess-uuid-789",
  "reefId": "reef-uuid",
  "status": "RECORDED",
  "healthScore": 72.5,
  "recommendations": [
    "Increase monitoring frequency",
    "Consider restoration intervention"
  ]
}
```

### 3.2 Get Reef Health Status

**Endpoint**: `GET /ecosystems/coral-reefs/{reefId}/health`

**Response**: `200 OK`
```json
{
  "reefId": "reef-uuid",
  "reefName": "Great Barrier Reef - Section A12",
  "currentHealth": {
    "overallScore": 72.5,
    "coralCover": 45.2,
    "bleaching": "MILD",
    "diversity": 2.3,
    "trend": "STABLE"
  },
  "historicalTrend": [
    {"date": "2024-01-01", "score": 68.0},
    {"date": "2025-01-01", "score": 70.5},
    {"date": "2026-01-01", "score": 72.5}
  ],
  "alerts": [
    {
      "type": "TEMPERATURE_RISE",
      "severity": "MEDIUM",
      "message": "Water temperature 1.5°C above seasonal average"
    }
  ]
}
```

### 3.3 Ocean Acidification Monitoring

**Endpoint**: `GET /ecosystems/acidification`

**Query Parameters**:
- `bbox`: Bounding box
- `startDate`: ISO 8601
- `endDate`: ISO 8601
- `depth`: Depth range (min-max)

**Response**: `200 OK`
```json
{
  "measurements": [
    {
      "measurementId": "acid-uuid-456",
      "location": {"latitude": 35.6762, "longitude": 139.6503, "depth": 50},
      "timestamp": "2026-01-12T12:00:00Z",
      "pH": 8.05,
      "pCO2": 450,
      "aragoniteSaturation": 2.5,
      "trend": {
        "pHChangePerDecade": -0.018,
        "projectedPH2050": 7.95
      }
    }
  ],
  "regionalSummary": {
    "averagePH": 8.08,
    "trend": "DECREASING",
    "criticalAreas": 3
  }
}
```

---

## 4. Marine Protected Area APIs

### 4.1 Get MPA Information

**Endpoint**: `GET /mpas/{mpaId}`

**Response**: `200 OK`
```json
{
  "mpaId": "mpa-uuid",
  "name": "Papahānaumokuākea Marine National Monument",
  "area": 1508870,
  "protectionLevel": "NO_TAKE",
  "regulations": [{
    "type": "FISHING_BAN",
    "description": "All extractive activities prohibited"
  }],
  "biodiversity": {
    "speciesCount": 7000,
    "endemicSpecies": 1400,
    "threatenedSpecies": 23
  },
  "complianceRate": 0.98
}
```

### 4.2 Check MPA Compliance

**Endpoint**: `POST /mpas/{mpaId}/check-compliance`

**Request Body**:
```json
{
  "vesselId": "IMO-1234567",
  "location": {
    "latitude": 25.8,
    "longitude": -170.5
  },
  "activity": "FISHING",
  "timestamp": "2026-01-12T10:00:00Z"
}
```

**Response**: `200 OK`
```json
{
  "compliant": false,
  "violation": {
    "type": "MPA_INTRUSION",
    "severity": "CRITICAL",
    "regulation": "No-take zone violation",
    "penalty": "Fine up to $100,000"
  },
  "enforcementNotified": true
}
```

---

## 5. Illegal Fishing Detection APIs

### 5.1 Report Suspicious Activity

**Endpoint**: `POST /enforcement/suspicious-activity`

**Request Body**:
```json
{
  "timestamp": "2026-01-12T03:45:00Z",
  "detectionMethod": "SATELLITE",
  "vessel": {
    "mmsi": "123456789",
    "name": "Suspicious Vessel"
  },
  "location": {
    "latitude": -10.5678,
    "longitude": 142.3456
  },
  "violation": {
    "type": "MPA_INTRUSION",
    "evidence": ["SATELLITE_IMAGE"],
    "confidence": 0.92
  }
}
```

**Response**: `201 Created`
```json
{
  "detectionId": "detect-uuid-123",
  "status": "UNDER_INVESTIGATION",
  "caseNumber": "IUU-2026-00142",
  "assignedTo": "Coast Guard District 7",
  "priority": "HIGH"
}
```

### 5.2 Track Vessel Activity

**Endpoint**: `GET /enforcement/vessels/{vesselId}/track`

**Query Parameters**:
- `startDate`: ISO 8601
- `endDate`: ISO 8601
- `includeAISGaps`: true/false

**Response**: `200 OK`
```json
{
  "vesselId": "IMO-1234567",
  "trackingData": [
    {
      "timestamp": "2026-01-12T00:00:00Z",
      "location": {"latitude": -10.5, "longitude": 142.3},
      "speed": 12.5,
      "heading": 180
    }
  ],
  "suspiciousPatterns": [
    {
      "pattern": "AIS_GAP",
      "duration": 48,
      "location": "Within MPA boundary",
      "riskScore": 0.85
    }
  ],
  "mpaIntrusions": 2,
  "violations": 1
}
```

---

## 6. Pollution Tracking APIs

### 6.1 Report Pollution Event

**Endpoint**: `POST /pollution/events`

**Request Body**:
```json
{
  "eventType": "PLASTIC_DEBRIS",
  "severity": "MAJOR",
  "location": {
    "latitude": 29.9511,
    "longitude": -90.0715
  },
  "pollutantDetails": {
    "type": "MICROPLASTICS",
    "concentration": 150000,
    "unit": "particles_per_cubic_meter"
  },
  "source": {
    "type": "LAND_BASED",
    "identified": true
  },
  "reportedBy": "citizen-scientist-uuid"
}
```

**Response**: `201 Created`
```json
{
  "pollutionEventId": "pollution-uuid-456",
  "status": "RESPONSE_INITIATED",
  "cleanupTeamAssigned": "Gulf Coast Response Team",
  "estimatedResponseTime": "4 hours"
}
```

### 6.2 Get Pollution Hotspots

**Endpoint**: `GET /pollution/hotspots`

**Query Parameters**:
- `type`: Pollution type
- `severity`: Minimum severity
- `region`: Geographic region

**Response**: `200 OK`
```json
{
  "hotspots": [
    {
      "location": {"latitude": 29.95, "longitude": -90.07},
      "pollutionType": "PLASTIC_DEBRIS",
      "concentration": "HIGH",
      "affectedArea": 250,
      "activeCleanup": true,
      "wildlifeImpact": "SEVERE"
    }
  ],
  "totalHotspots": 47,
  "criticalHotspots": 8
}
```

---

## 7. Conservation Alert APIs

### 7.1 Subscribe to Alerts

**Endpoint**: `POST /alerts/subscriptions`

**Request Body**:
```json
{
  "subscriberId": "user-uuid",
  "alertTypes": ["BLEACHING_EVENT", "ILLEGAL_FISHING", "POLLUTION"],
  "regions": ["CARIBBEAN_SEA", "GREAT_BARRIER_REEF"],
  "severity": ["CRITICAL", "HIGH"],
  "deliveryMethod": "EMAIL|SMS|WEBHOOK",
  "webhookUrl": "https://example.com/webhook"
}
```

**Response**: `201 Created`
```json
{
  "subscriptionId": "sub-uuid-789",
  "status": "ACTIVE",
  "message": "Alert subscription created successfully"
}
```

### 7.2 Get Active Alerts

**Endpoint**: `GET /alerts/active`

**Query Parameters**:
- `region`: Geographic region
- `severity`: Minimum severity
- `type`: Alert type

**Response**: `200 OK`
```json
{
  "activeAlerts": [
    {
      "alertId": "alert-uuid-999",
      "type": "BLEACHING_EVENT",
      "severity": "CRITICAL",
      "region": "GREAT_BARRIER_REEF",
      "description": "Mass coral bleaching event detected",
      "affectedArea": 12000,
      "issuedAt": "2026-01-10T00:00:00Z",
      "actionRequired": "Increase monitoring, deploy response teams",
      "updates": [
        {
          "timestamp": "2026-01-11T12:00:00Z",
          "message": "Bleaching spreading to adjacent sectors"
        }
      ]
    }
  ],
  "totalAlerts": 15,
  "criticalAlerts": 3
}
```

---

## 8. Analytics and Reporting APIs

### 8.1 Generate Conservation Report

**Endpoint**: `POST /reports/generate`

**Request Body**:
```json
{
  "reportType": "SPECIES_POPULATION|ECOSYSTEM_HEALTH|MPA_EFFECTIVENESS",
  "region": "MEDITERRANEAN_SEA",
  "period": {
    "start": "2025-01-01T00:00:00Z",
    "end": "2025-12-31T23:59:59Z"
  },
  "includeMetrics": ["BIODIVERSITY", "THREATS", "CONSERVATION_ACTIONS"],
  "format": "PDF|HTML|JSON"
}
```

**Response**: `202 Accepted`
```json
{
  "reportId": "report-uuid-111",
  "status": "GENERATING",
  "estimatedCompletion": "2026-01-12T15:30:00Z",
  "downloadUrl": null
}
```

### 8.2 Get Report Status

**Endpoint**: `GET /reports/{reportId}`

**Response**: `200 OK`
```json
{
  "reportId": "report-uuid-111",
  "status": "COMPLETED",
  "generatedAt": "2026-01-12T15:25:00Z",
  "downloadUrl": "https://reports.wia.org/conservation/report-111.pdf",
  "expiresAt": "2026-01-19T15:25:00Z"
}
```

---

## 9. Rate Limiting and Quotas

- **Free Tier**: 1,000 requests/day
- **Research Tier**: 100,000 requests/day
- **Enterprise Tier**: Unlimited (custom SLA)

**Rate Limit Headers**:
```
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 856
X-RateLimit-Reset: 1705104000
```

---

## 10. Error Handling

**Standard Error Response**:
```json
{
  "error": {
    "code": "INVALID_SPECIES_CODE",
    "message": "The provided species code is not recognized",
    "details": {
      "field": "speciesCode",
      "value": "INVALID-CODE"
    },
    "timestamp": "2026-01-12T14:30:00Z",
    "requestId": "req-uuid-456"
  }
}
```

**HTTP Status Codes**:
- `200 OK`: Success
- `201 Created`: Resource created
- `400 Bad Request`: Invalid input
- `401 Unauthorized`: Authentication required
- `403 Forbidden`: Insufficient permissions
- `404 Not Found`: Resource not found
- `429 Too Many Requests`: Rate limit exceeded
- `500 Internal Server Error`: Server error

---

© 2025 SmileStory Inc. / WIA
弘益人間 · Protecting Oceans for All Humanity
