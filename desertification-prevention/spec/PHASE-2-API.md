# WIA-ENE-057: Desertification Prevention
## PHASE 2 - API Specification

**Version:** 1.0.0
**Status:** Standard
**Last Updated:** 2025-12-25
**Base URL:** `https://api.wia-ene-057.org/v1`

---

## Overview

This document defines the RESTful API for the WIA-ENE-057 Desertification Prevention standard, enabling real-time land degradation monitoring, restoration planning, and integration with global conservation systems.

**Philosophy:** 弘益人間 (홍익인간) - Benefit All Humanity

---

## 1. Authentication

### 1.1 API Key Authentication

```http
GET /api/v1/monitoring/vegetation
Authorization: Bearer YOUR_API_KEY
```

### 1.2 OAuth 2.0 Flow

```http
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials
&client_id=YOUR_CLIENT_ID
&client_secret=YOUR_CLIENT_SECRET
&scope=monitoring:read restoration:write
```

**Response:**
```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "monitoring:read restoration:write"
}
```

---

## 2. Vegetation Monitoring API

### 2.1 Submit Vegetation Data

**Endpoint:** `POST /api/v1/monitoring/vegetation`

**Description:** Submit vegetation index measurements (NDVI, EVI, LAI) for a location.

**Request:**
```http
POST /api/v1/monitoring/vegetation
Content-Type: application/json
Authorization: Bearer YOUR_API_KEY

{
  "locationId": "SAHEL-REGION-001",
  "timestamp": "2025-12-25T10:00:00Z",
  "sensorData": {
    "ndvi": 0.35,
    "evi": 0.28,
    "lai": 1.2
  },
  "source": "MODIS",
  "resolution": "250m",
  "quality": {
    "cloudCover": 5,
    "confidence": 0.92
  }
}
```

**Response:** `201 Created`
```json
{
  "status": "success",
  "data": {
    "recordId": "VEG-20251225-001",
    "locationId": "SAHEL-REGION-001",
    "timestamp": "2025-12-25T10:00:00Z",
    "analysis": {
      "healthStatus": "degraded",
      "trendDirection": "declining",
      "alertLevel": "medium",
      "confidence": 0.87
    },
    "recommendations": [
      "Monitor closely for continued decline",
      "Consider intervention if trend persists"
    ]
  },
  "links": {
    "self": "/api/v1/monitoring/vegetation/VEG-20251225-001",
    "location": "/api/v1/locations/SAHEL-REGION-001",
    "trend": "/api/v1/monitoring/vegetation/SAHEL-REGION-001/trend"
  }
}
```

### 2.2 Get Vegetation Trend

**Endpoint:** `GET /api/v1/monitoring/vegetation/{locationId}/trend`

**Parameters:**
- `period` (optional): `7d`, `30d`, `90d`, `1y`, `5y` (default: `30d`)
- `metric` (optional): `ndvi`, `evi`, `lai`, `all` (default: `all`)

**Request:**
```http
GET /api/v1/monitoring/vegetation/SAHEL-REGION-001/trend?period=90d&metric=ndvi
Authorization: Bearer YOUR_API_KEY
```

**Response:** `200 OK`
```json
{
  "locationId": "SAHEL-REGION-001",
  "metric": "ndvi",
  "period": "90d",
  "data": [
    {
      "date": "2025-10-01",
      "value": 0.45,
      "quality": "high"
    },
    {
      "date": "2025-11-01",
      "value": 0.40,
      "quality": "high"
    },
    {
      "date": "2025-12-01",
      "value": 0.35,
      "quality": "high"
    }
  ],
  "statistics": {
    "mean": 0.40,
    "min": 0.35,
    "max": 0.45,
    "trend": "declining",
    "changeRate": -0.033,
    "significance": 0.95
  }
}
```

---

## 3. Soil Monitoring API

### 3.1 Submit Soil Data

**Endpoint:** `POST /api/v1/monitoring/soil`

**Request:**
```http
POST /api/v1/monitoring/soil
Content-Type: application/json
Authorization: Bearer YOUR_API_KEY

{
  "locationId": "SAHEL-REGION-001",
  "timestamp": "2025-12-25T10:00:00Z",
  "moisture": {
    "surface": 12.5,
    "profile": [
      {"depth": "0-10cm", "moisture": 15.2},
      {"depth": "10-30cm", "moisture": 12.8},
      {"depth": "30-60cm", "moisture": 10.1}
    ]
  },
  "chemistry": {
    "ph": 7.2,
    "organicMatter": 1.8,
    "nitrogen": 0.08,
    "phosphorus": 12.5,
    "potassium": 145.2
  },
  "degradation": {
    "erosionRisk": "high",
    "compaction": "moderate",
    "salinization": "low"
  }
}
```

**Response:** `201 Created`
```json
{
  "status": "success",
  "data": {
    "recordId": "SOIL-20251225-001",
    "locationId": "SAHEL-REGION-001",
    "analysis": {
      "soilHealth": 45.2,
      "healthCategory": "poor",
      "criticalIssues": [
        "Low organic matter content",
        "High erosion risk"
      ],
      "recommendations": [
        "Increase organic matter through composting",
        "Implement erosion control measures"
      ]
    }
  }
}
```

---

## 4. Desertification Risk Assessment API

### 4.1 Calculate Risk Score

**Endpoint:** `GET /api/v1/assessment/desertification/{locationId}`

**Request:**
```http
GET /api/v1/assessment/desertification/SAHEL-REGION-001
Authorization: Bearer YOUR_API_KEY
```

**Response:** `200 OK`
```json
{
  "locationId": "SAHEL-REGION-001",
  "assessmentDate": "2025-12-25T10:00:00Z",
  "riskScore": 68.5,
  "riskCategory": "high-risk",
  "factors": {
    "vegetation": {
      "score": 35.0,
      "weight": 0.30,
      "status": "degraded",
      "metrics": {
        "ndvi": 0.35,
        "cover": 45.2
      }
    },
    "soil": {
      "score": 72.0,
      "weight": 0.25,
      "status": "severely-degraded",
      "metrics": {
        "moisture": 12.5,
        "organicMatter": 1.8,
        "erosionRisk": "high"
      }
    },
    "climate": {
      "score": 85.0,
      "weight": 0.25,
      "status": "adverse",
      "metrics": {
        "rainfall": 325.4,
        "deviation": -23.6,
        "droughtIndex": -1.8
      }
    },
    "humanActivity": {
      "score": 90.0,
      "weight": 0.20,
      "status": "high-pressure",
      "metrics": {
        "grazingIntensity": "high",
        "populationPressure": "high"
      }
    }
  },
  "projections": {
    "1year": {
      "riskScore": 72.3,
      "category": "high-risk"
    },
    "5year": {
      "riskScore": 81.5,
      "category": "critical"
    }
  },
  "recommendations": [
    {
      "priority": "critical",
      "action": "Implement immediate soil conservation measures",
      "impact": "high",
      "timeline": "immediate"
    },
    {
      "priority": "high",
      "action": "Establish green belt with drought-resistant species",
      "impact": "high",
      "timeline": "1-2 years"
    },
    {
      "priority": "medium",
      "action": "Reduce grazing intensity by 40%",
      "impact": "medium",
      "timeline": "6 months"
    }
  ]
}
```

### 4.2 Batch Risk Assessment

**Endpoint:** `POST /api/v1/assessment/desertification/batch`

**Request:**
```http
POST /api/v1/assessment/desertification/batch
Content-Type: application/json
Authorization: Bearer YOUR_API_KEY

{
  "locationIds": [
    "SAHEL-REGION-001",
    "SAHEL-REGION-002",
    "SAHEL-REGION-003"
  ],
  "options": {
    "includeProjections": true,
    "projectionsYears": [1, 5, 10]
  }
}
```

**Response:** `200 OK`
```json
{
  "assessmentId": "BATCH-20251225-001",
  "results": [
    {
      "locationId": "SAHEL-REGION-001",
      "riskScore": 68.5,
      "category": "high-risk"
    },
    {
      "locationId": "SAHEL-REGION-002",
      "riskScore": 52.3,
      "category": "medium-risk"
    },
    {
      "locationId": "SAHEL-REGION-003",
      "riskScore": 82.1,
      "category": "critical"
    }
  ],
  "summary": {
    "totalLocations": 3,
    "averageRisk": 67.6,
    "highRiskCount": 2,
    "criticalCount": 1
  }
}
```

---

## 5. Restoration Planning API

### 5.1 Create Restoration Plan

**Endpoint:** `POST /api/v1/restoration/plan`

**Request:**
```http
POST /api/v1/restoration/plan
Content-Type: application/json
Authorization: Bearer YOUR_API_KEY

{
  "locationId": "SAHEL-REGION-001",
  "projectName": "Sahel Green Belt Initiative",
  "organization": "Great Green Wall Foundation",
  "area": {
    "size": 1000,
    "unit": "hectares"
  },
  "objectives": [
    "soil-stabilization",
    "vegetation-recovery",
    "biodiversity-enhancement"
  ],
  "constraints": {
    "budget": 500000,
    "timeline": "5-years",
    "waterAvailability": "limited"
  },
  "preferences": {
    "nativeSpecies": true,
    "communityInvolvement": true,
    "carbonSequestration": true
  }
}
```

**Response:** `201 Created`
```json
{
  "status": "success",
  "data": {
    "planId": "REST-PLAN-20251225-001",
    "projectName": "Sahel Green Belt Initiative",
    "strategy": {
      "phase1": {
        "name": "Soil preparation and water conservation",
        "duration": "12 months",
        "activities": [
          "Half-moon technique implementation",
          "Stone bund construction",
          "Rainwater harvesting structures"
        ],
        "cost": 100000
      },
      "phase2": {
        "name": "Native species planting",
        "duration": "24 months",
        "activities": [
          "Nursery establishment",
          "Tree planting (50,000 seedlings)",
          "Initial maintenance"
        ],
        "species": [
          {
            "name": "Acacia senegal",
            "quantity": 30000,
            "purpose": "nitrogen-fixation, gum-arabic"
          },
          {
            "name": "Balanites aegyptiaca",
            "quantity": 20000,
            "purpose": "food, soil-stabilization"
          }
        ],
        "cost": 250000
      },
      "phase3": {
        "name": "Monitoring and maintenance",
        "duration": "24 months",
        "activities": [
          "Regular monitoring",
          "Adaptive management",
          "Community capacity building"
        ],
        "cost": 150000
      }
    },
    "expectedOutcomes": {
      "environmental": {
        "vegetationCover": "+35%",
        "soilQuality": "+50%",
        "carbonSequestration": "2500 tons CO2",
        "biodiversity": "+30 species"
      },
      "social": {
        "beneficiaries": 2500,
        "jobsCreated": 150,
        "incomeIncrease": "+20%"
      },
      "economic": {
        "roi": "185%",
        "paybackPeriod": "8 years",
        "annualValue": "125,000 USD"
      }
    },
    "riskMitigation": [
      {
        "risk": "drought-during-planting",
        "probability": "medium",
        "mitigation": "Water harvesting, drought-resistant species"
      },
      {
        "risk": "livestock-damage",
        "probability": "high",
        "mitigation": "Community agreements, protective fencing"
      }
    ],
    "monitoringPlan": {
      "frequency": "quarterly",
      "indicators": [
        "vegetation-cover",
        "soil-moisture",
        "tree-survival-rate",
        "community-satisfaction"
      ]
    }
  },
  "links": {
    "self": "/api/v1/restoration/plan/REST-PLAN-20251225-001",
    "location": "/api/v1/locations/SAHEL-REGION-001",
    "progress": "/api/v1/restoration/plan/REST-PLAN-20251225-001/progress"
  }
}
```

### 5.2 Update Restoration Progress

**Endpoint:** `PUT /api/v1/restoration/plan/{planId}/progress`

**Request:**
```http
PUT /api/v1/restoration/plan/REST-PLAN-20251225-001/progress
Content-Type: application/json
Authorization: Bearer YOUR_API_KEY

{
  "date": "2025-12-25",
  "phase": "phase2",
  "completionPercentage": 65.0,
  "activities": [
    {
      "name": "Tree planting",
      "status": "completed",
      "treesPlanted": 50000,
      "survivalRate": 82.5
    },
    {
      "name": "Soil conservation",
      "status": "in-progress",
      "halfMoonsCreated": 3500,
      "targetHalfMoons": 5000
    }
  ],
  "outcomes": {
    "vegetationCoverChange": 28.5,
    "soilMoistureImprovement": 15.2,
    "carbonSequestered": 1250
  },
  "challenges": [
    {
      "description": "Lower than expected rainfall",
      "impact": "medium",
      "mitigation": "Increased irrigation from harvested water"
    }
  ]
}
```

**Response:** `200 OK`
```json
{
  "status": "success",
  "data": {
    "planId": "REST-PLAN-20251225-001",
    "overallProgress": 65.0,
    "currentPhase": "phase2",
    "onTrack": true,
    "nextMilestone": {
      "name": "Complete soil conservation measures",
      "dueDate": "2026-03-15",
      "daysRemaining": 80
    }
  }
}
```

---

## 6. Location Management API

### 6.1 Register Location

**Endpoint:** `POST /api/v1/locations`

**Request:**
```http
POST /api/v1/locations
Content-Type: application/json
Authorization: Bearer YOUR_API_KEY

{
  "locationId": "SAHEL-REGION-001",
  "name": "Sahel Test Region",
  "coordinates": {
    "latitude": 14.5,
    "longitude": -4.0,
    "datum": "WGS84"
  },
  "area": {
    "size": 10000,
    "unit": "hectares"
  },
  "administrativeLevel": {
    "country": "Mali",
    "region": "Kayes",
    "district": "Nioro"
  },
  "baseline": {
    "vegetationCover": 45.2,
    "soilHealth": 38.5,
    "desertificationRisk": 68.5,
    "baselineYear": 2025
  }
}
```

**Response:** `201 Created`
```json
{
  "status": "success",
  "data": {
    "locationId": "SAHEL-REGION-001",
    "registrationDate": "2025-12-25T10:00:00Z",
    "status": "active"
  },
  "links": {
    "self": "/api/v1/locations/SAHEL-REGION-001",
    "monitoring": "/api/v1/monitoring/vegetation/SAHEL-REGION-001",
    "assessment": "/api/v1/assessment/desertification/SAHEL-REGION-001"
  }
}
```

### 6.2 Get Location Details

**Endpoint:** `GET /api/v1/locations/{locationId}`

**Response:** `200 OK`
```json
{
  "locationId": "SAHEL-REGION-001",
  "name": "Sahel Test Region",
  "coordinates": {
    "latitude": 14.5,
    "longitude": -4.0
  },
  "area": {
    "size": 10000,
    "unit": "hectares"
  },
  "currentStatus": {
    "vegetationCover": 45.2,
    "soilHealth": 38.5,
    "desertificationRisk": 68.5,
    "lastAssessment": "2025-12-25T10:00:00Z"
  },
  "activeProjects": [
    {
      "projectId": "REST-2025-SAHEL-001",
      "name": "Sahel Green Belt Initiative",
      "status": "in-progress",
      "progress": 65.0
    }
  ],
  "statistics": {
    "monitoringRecords": 156,
    "assessments": 12,
    "alerts": 8
  }
}
```

---

## 7. Alert & Notification API

### 7.1 Subscribe to Alerts

**Endpoint:** `POST /api/v1/alerts/subscribe`

**Request:**
```http
POST /api/v1/alerts/subscribe
Content-Type: application/json
Authorization: Bearer YOUR_API_KEY

{
  "locationIds": ["SAHEL-REGION-001"],
  "alertTypes": [
    "rapid-vegetation-decline",
    "critical-soil-moisture",
    "drought-alert",
    "erosion-risk"
  ],
  "severity": ["high", "critical"],
  "deliveryMethod": {
    "webhook": "https://your-domain.com/webhooks/desertification",
    "email": "alerts@your-organization.org"
  }
}
```

**Response:** `201 Created`
```json
{
  "status": "success",
  "data": {
    "subscriptionId": "SUB-20251225-001",
    "status": "active",
    "monitoredLocations": 1,
    "alertTypes": 4
  }
}
```

### 7.2 Webhook Payload Example

```json
{
  "alertId": "ALERT-20251225-001",
  "subscriptionId": "SUB-20251225-001",
  "timestamp": "2025-12-25T10:30:00Z",
  "severity": "high",
  "type": "rapid-vegetation-decline",
  "location": {
    "locationId": "SAHEL-REGION-001",
    "name": "Sahel Test Region",
    "coordinates": {
      "latitude": 14.5,
      "longitude": -4.0
    }
  },
  "details": {
    "metric": "ndvi",
    "currentValue": 0.35,
    "previousValue": 0.45,
    "change": -0.10,
    "changePeriod": "90 days",
    "threshold": -0.05
  },
  "message": "NDVI decreased by 22% over 90 days, exceeding alert threshold",
  "recommendations": [
    "Investigate cause of vegetation decline",
    "Consider immediate intervention",
    "Review grazing or land use patterns"
  ],
  "links": {
    "location": "https://api.wia-ene-057.org/v1/locations/SAHEL-REGION-001",
    "assessment": "https://api.wia-ene-057.org/v1/assessment/desertification/SAHEL-REGION-001"
  }
}
```

---

## 8. Reporting API

### 8.1 Generate UNCCD LDN Report

**Endpoint:** `POST /api/v1/reports/unccd-ldn`

**Request:**
```http
POST /api/v1/reports/unccd-ldn
Content-Type: application/json
Authorization: Bearer YOUR_API_KEY

{
  "locationIds": ["SAHEL-REGION-001"],
  "reportingPeriod": {
    "start": "2025-01-01",
    "end": "2025-12-31"
  },
  "reportType": "annual",
  "format": "json",
  "includeProjections": true
}
```

**Response:** `200 OK`
```json
{
  "reportId": "UNCCD-LDN-2025-001",
  "reportingPeriod": "2025",
  "locations": [
    {
      "locationId": "SAHEL-REGION-001",
      "baseline": {
        "year": 2015,
        "landCover": { ... },
        "productivity": { ... },
        "carbonStocks": { ... }
      },
      "current": {
        "year": 2025,
        "landCover": { ... },
        "productivity": { ... },
        "carbonStocks": { ... }
      },
      "ldnStatus": {
        "achieved": false,
        "progress": 45.2,
        "neutralityGap": -54.8
      }
    }
  ],
  "downloadUrl": "https://api.wia-ene-057.org/v1/reports/UNCCD-LDN-2025-001/download",
  "expiresAt": "2026-01-25T10:00:00Z"
}
```

---

## 9. WebSocket Streaming API

### 9.1 Real-time Monitoring Stream

**Endpoint:** `wss://api.wia-ene-057.org/v1/stream`

**Connection:**
```javascript
const ws = new WebSocket('wss://api.wia-ene-057.org/v1/stream');

// Authenticate
ws.send(JSON.stringify({
  type: 'auth',
  token: 'YOUR_API_KEY'
}));

// Subscribe to location
ws.send(JSON.stringify({
  type: 'subscribe',
  locationIds: ['SAHEL-REGION-001'],
  dataTypes: ['vegetation', 'soil', 'alerts']
}));

// Receive updates
ws.onmessage = (event) => {
  const data = JSON.parse(event.data);
  console.log('Update:', data);
};
```

**Stream Message Format:**
```json
{
  "type": "vegetation-update",
  "locationId": "SAHEL-REGION-001",
  "timestamp": "2025-12-25T10:00:00Z",
  "data": {
    "ndvi": 0.35,
    "change": -0.02,
    "trend": "declining"
  }
}
```

---

## 10. Error Handling

### 10.1 Error Response Format

```json
{
  "error": {
    "code": "INVALID_LOCATION",
    "message": "Location ID 'INVALID-001' not found",
    "details": {
      "locationId": "INVALID-001",
      "suggestion": "Register location first using POST /api/v1/locations"
    },
    "timestamp": "2025-12-25T10:00:00Z",
    "requestId": "req-abc123"
  }
}
```

### 10.2 HTTP Status Codes

| Code | Description |
|------|-------------|
| 200 | OK - Request successful |
| 201 | Created - Resource created |
| 400 | Bad Request - Invalid input |
| 401 | Unauthorized - Invalid or missing authentication |
| 403 | Forbidden - Insufficient permissions |
| 404 | Not Found - Resource not found |
| 429 | Too Many Requests - Rate limit exceeded |
| 500 | Internal Server Error |

---

## 11. Rate Limiting

- **Standard Tier:** 1,000 requests/hour
- **Professional Tier:** 10,000 requests/hour
- **Enterprise Tier:** Unlimited

**Rate Limit Headers:**
```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 847
X-RateLimit-Reset: 2025-12-25T11:00:00Z
```

---

## 12. Pagination

**Request:**
```http
GET /api/v1/monitoring/vegetation?page=2&limit=50
```

**Response:**
```json
{
  "data": [ ... ],
  "pagination": {
    "page": 2,
    "limit": 50,
    "totalPages": 10,
    "totalRecords": 487,
    "hasNext": true,
    "hasPrevious": true
  },
  "links": {
    "first": "/api/v1/monitoring/vegetation?page=1&limit=50",
    "previous": "/api/v1/monitoring/vegetation?page=1&limit=50",
    "next": "/api/v1/monitoring/vegetation?page=3&limit=50",
    "last": "/api/v1/monitoring/vegetation?page=10&limit=50"
  }
}
```

---

**Document Control:**
- **Author:** WIA Standards Committee
- **Review Date:** 2025-12-25
- **Next Review:** 2026-06-25
- **Philosophy:** 弘익人間 (홍익인간) - Benefit All Humanity

© 2025 SmileStory Inc. / WIA
