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


## Annex E — Implementation Notes for PHASE-2-API

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-2-API.

- **Operational scope** — implementations SHOULD declare their operational
  scope (single-tenant, multi-tenant, federated) in the OpenAPI document so
  that downstream auditors can score the deployment against the correct
  conformance tier in Annex A.
- **Schema evolution** — additive changes (new optional fields, new error
  codes) are non-breaking; renaming or removing fields, even in error
  payloads, MUST trigger a minor version bump.
- **Audit retention** — a 7-year retention window is sufficient to satisfy
  ISO/IEC 17065:2012 audit expectations in most jurisdictions; some
  regulators require longer retention, in which case the deployment policy
  MUST extend the retention window rather than relying on this PHASE's
  defaults.
- **Time synchronization** — sub-second deadlines depend on synchronized
  clocks. NTPv4 with stratum-2 servers is sufficient for most deadlines
  expressed in this PHASE; PTP is recommended for sites that require
  deterministic interlocks.
- **Error budget reporting** — implementations SHOULD publish a monthly
  error-budget summary (latency p95, error rate, violation hours) in the
  format defined by the WIA reporting profile to facilitate cross-vendor
  comparison without exposing tenant-specific data.

These notes are not requirements; they are a reference for field teams
mapping their existing operations onto WIA conformance.

## Annex F — Adoption Roadmap

The adoption roadmap for this PHASE document is non-normative and is intended to set expectations for early implementers about the relative stability of each section.

- **Stable** (sections marked normative with `MUST` / `MUST NOT`) — semantic versioning applies; breaking changes require a major version bump and at minimum 90 days of overlap with the prior major version on all WIA-published reference implementations.
- **Provisional** (sections in this Annex and Annex D) — items are tracked openly and may be promoted to normative status without a major version bump if community feedback supports promotion.
- **Reference** (test vectors, simulator behaviour, the reference TypeScript SDK) — versioned independently of this document so that mistakes in reference material can be corrected without amending the published PHASE document.

Implementers SHOULD subscribe to the WIA Standards GitHub release notifications to track promotions between these tiers. Comments on the roadmap are accepted via the GitHub issues tracker on the WIA-Official organization.

The roadmap is reviewed at every minor version of this PHASE document, and the review outcomes are recorded in the version-history table at the start of the document.

## Annex G — Test Vectors and Conformance Evidence

This annex describes how implementations capture and publish conformance
evidence for PHASE-2-API. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-2-api/`. Implementations claiming
  conformance MUST run all vectors in CI and publish the resulting
  pass/fail matrix in their compliance package.
- **Evidence package** — the compliance package is a tarball containing
  the SBOM (CycloneDX 1.5 or SPDX 2.3), the OpenAPI document, the test
  vector matrix, and a signed manifest. Signatures use Sigstore (DSSE
  envelope, Rekor transparency log entry) so that downstream consumers
  can verify provenance without trusting a private CA.
- **Quarterly recheck** — implementations re-publish the evidence package
  every quarter even if no source change occurred, so that consumers can
  detect environmental drift (compiler updates, dependency updates, OS
  updates) without polling vendor changelogs.
- **Cross-vendor crosswalk** — the WIA Standards working group maintains a
  crosswalk that maps each vector to the equivalent assertion in adjacent
  industry programs (where one exists), so an implementer that already
  certifies under one program can show conformance to PHASE-2-API with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-2-API does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-2-API.
It is non-normative; the rules below describe the policy that the WIA
Standards working group commits to when amending this PHASE document.

- **Semantic versioning** — major / minor / patch components follow
  Semantic Versioning 2.0.0 (https://semver.org/spec/v2.0.0.html).
  Major bump indicates a backwards-incompatible change to a normative
  requirement; minor bump indicates new normative requirements that do
  not break existing implementations; patch bump indicates editorial
  changes only (clarifications, typo fixes, formatting).
- **Deprecation window** — when a normative requirement is removed or
  altered in a backwards-incompatible way, the prior major version is
  maintained in parallel for at least 180 days. During the parallel
  window, both major versions are marked Stable in the WIA Standards
  registry and either may be cited as "WIA-conformant".
- **Sunset notification** — deprecated major versions enter a 12-month
  sunset window during which the WIA registry marks the version as
  Deprecated. The deprecation entry includes a migration note pointing
  to the replacement requirement(s) and an explanation of why the
  change was made.
- **Editorial errata** — patch-level errata are issued without a
  deprecation window because they do not change normative behaviour.
  Errata are tracked in a public errata register and each entry is
  signed by the WIA Standards working group chair.
- **Implementation changelog mapping** — implementations SHOULD publish
  a changelog mapping each PHASE version they support to the specific
  build, container digest, or SDK version that satisfies the version.
  This allows downstream auditors to verify version conformance without
  re-running the entire test matrix on every release.

The policy is reviewed at the same cadence as the PHASE document and
any changes to the policy itself are tracked in the version-history
table at the start of the document.

## Annex I — Interoperability Profiles

This annex describes how implementations declare interoperability profiles
for PHASE-2-API. The profile mechanism is non-normative and exists so that
deployments of varying scope (single tenant, regional cluster, federated
network) can advertise the subset of normative requirements they satisfy
without misrepresenting partial conformance as full conformance.

- **Profile manifest** — every implementation publishes a profile manifest
  in JSON. The manifest enumerates the normative requirement IDs from this
  PHASE that are satisfied (`status: "supported"`), partially satisfied
  (`status: "partial"`, with a reason field), or excluded
  (`status: "excluded"`, with a justification). The manifest is signed
  using the same Sigstore key used for the SBOM in Annex G.
- **Federation profile** — federated deployments publish an aggregated
  manifest summarizing the union and intersection of member-implementation
  profiles. The aggregated manifest is consumed by directory services so
  that callers can route a request to the least common denominator profile
  required for an interaction.
- **Backwards-profile compatibility** — when a deployment migrates from one
  profile to a wider profile, the prior profile manifest remains valid and
  signed for the deprecation window defined in Annex H. This preserves
  audit traceability for auditors evaluating long-term interoperability.
- **Profile registry** — the WIA Standards working group maintains a
  public registry of named profiles. Common deployment shapes (e.g.,
  "Edge-only", "Federated-with-replay") are added to the registry by
  consensus. Registry entries are immutable; new shapes are added under
  new names rather than amending existing entries.
- **Profile versioning** — profile names are versioned with the same
  Semantic Versioning rules described in Annex H. A deployment that
  advertises `WIA-P2-API-Edge-only/2` is asserting conformance with
  the second major version of the named profile, not the second deployment
  of an unversioned profile.

The profile mechanism is intentionally lightweight; it is meant to make
real deployment shapes visible without forcing every deployment to
satisfy every normative requirement.
