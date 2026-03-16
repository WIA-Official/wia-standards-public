# WIA-ENE-058: Soil Restoration - Phase 2 API Specification

**Standard ID:** WIA-ENE-058
**Category:** Energy & Environment (ENE)
**Version:** 1.0.0
**Status:** Active
**Last Updated:** 2025-12-25

---

## Overview

Phase 2 defines the RESTful API for soil restoration monitoring, health assessment, restoration planning, and carbon sequestration tracking.

**Base URL:** `https://api.wia.earth/v1/soil`

---

## 1. Authentication

### 1.1 API Key Authentication

```http
Authorization: Bearer YOUR_API_KEY
```

### 1.2 OAuth 2.0

```http
Authorization: Bearer ACCESS_TOKEN
```

### 1.3 Rate Limits

- **Free Tier:** 100 requests/hour
- **Basic:** 1,000 requests/hour
- **Professional:** 10,000 requests/hour
- **Enterprise:** Unlimited

---

## 2. Soil Monitoring Endpoints

### 2.1 Record Soil Sample

**POST** `/monitor`

Submit soil sample data for analysis.

**Request:**
```json
{
  "plotId": "PLOT-FARM-A-001",
  "location": {
    "latitude": 37.7749,
    "longitude": -122.4194,
    "elevation": 52.0
  },
  "sampleDate": "2025-12-25T10:30:00Z",
  "soilProperties": {
    "ph": 6.5,
    "organicMatter": 3.5,
    "texture": "sandy_loam",
    "bulkDensity": 1.35,
    "moisture": 22.0
  },
  "nutrients": {
    "nitrogen": 25,
    "phosphorus": 15,
    "potassium": 120
  },
  "carbonMetrics": {
    "totalCarbon": 2.03,
    "organicCarbon": 1.75
  }
}
```

**Response:** `201 Created`
```json
{
  "sampleId": "SOIL-2025-001-ABC",
  "plotId": "PLOT-FARM-A-001",
  "status": "received",
  "timestamp": "2025-12-25T10:30:00Z",
  "analysisUrl": "/samples/SOIL-2025-001-ABC",
  "estimatedProcessingTime": "2 hours"
}
```

### 2.2 Get Sample Details

**GET** `/samples/{sampleId}`

Retrieve details of a specific soil sample.

**Response:** `200 OK`
```json
{
  "sampleId": "SOIL-2025-001-ABC",
  "plotId": "PLOT-FARM-A-001",
  "status": "analyzed",
  "submittedAt": "2025-12-25T10:30:00Z",
  "analyzedAt": "2025-12-25T12:30:00Z",
  "data": {
    "soilProperties": { },
    "nutrients": { },
    "carbonMetrics": { }
  },
  "healthScore": 75.5,
  "recommendations": [
    "Maintain current organic matter levels",
    "Consider cover cropping to improve soil structure"
  ]
}
```

### 2.3 List Samples

**GET** `/samples?plotId={plotId}&startDate={date}&endDate={date}`

Retrieve list of samples with optional filters.

**Query Parameters:**
- `plotId` (optional): Filter by plot ID
- `startDate` (optional): ISO 8601 date
- `endDate` (optional): ISO 8601 date
- `limit` (optional): Max results (default: 50)
- `offset` (optional): Pagination offset

**Response:** `200 OK`
```json
{
  "total": 125,
  "limit": 50,
  "offset": 0,
  "samples": [
    {
      "sampleId": "SOIL-2025-001-ABC",
      "plotId": "PLOT-FARM-A-001",
      "sampleDate": "2025-12-25T10:30:00Z",
      "healthScore": 75.5,
      "status": "analyzed"
    }
  ]
}
```

---

## 3. Soil Health Endpoints

### 3.1 Get Health Assessment

**GET** `/health/{plotId}`

Get comprehensive soil health assessment for a plot.

**Response:** `200 OK`
```json
{
  "plotId": "PLOT-FARM-A-001",
  "assessmentDate": "2025-12-25T10:30:00Z",
  "overallScore": 75.5,
  "grade": "Good",
  "trend": {
    "direction": "improving",
    "changeRate": "+2.5 points/year"
  },
  "components": {
    "physical": {
      "score": 80,
      "status": "Good"
    },
    "chemical": {
      "score": 72,
      "status": "Fair"
    },
    "biological": {
      "score": 74,
      "status": "Good"
    }
  },
  "degradationRisk": {
    "level": "Low",
    "factors": [
      {
        "factor": "erosion",
        "risk": "Low",
        "score": 85
      }
    ]
  },
  "historicalData": [
    {
      "date": "2024-12-25",
      "score": 70.5
    },
    {
      "date": "2025-06-25",
      "score": 73.0
    },
    {
      "date": "2025-12-25",
      "score": 75.5
    }
  ]
}
```

### 3.2 Calculate Health Index

**POST** `/health/calculate`

Calculate soil health index from provided data.

**Request:**
```json
{
  "soilData": {
    "ph": 6.5,
    "organicMatter": 3.5,
    "nutrients": {
      "nitrogen": 25,
      "phosphorus": 15,
      "potassium": 120
    },
    "bulkDensity": 1.35,
    "microbialBiomass": 450
  }
}
```

**Response:** `200 OK`
```json
{
  "healthIndex": 75.5,
  "grade": "Good",
  "componentScores": {
    "ph": 90,
    "organicMatter": 70,
    "nutrients": 65,
    "physical": 80,
    "biological": 75
  },
  "recommendations": [
    "Maintain current practices",
    "Monitor nutrient levels quarterly"
  ]
}
```

---

## 4. Restoration Planning Endpoints

### 4.1 Create Restoration Plan

**POST** `/restore`

Create a soil restoration plan.

**Request:**
```json
{
  "plotId": "PLOT-FARM-A-001",
  "currentHealth": 65.0,
  "targetHealth": 85.0,
  "timeframe": "18_months",
  "constraints": {
    "budget": 5000,
    "currency": "USD",
    "availableEquipment": ["tractor", "spreader"],
    "seasonalWindow": ["spring", "fall"]
  },
  "preferences": {
    "organic": true,
    "minimumTillage": true,
    "coverCropping": true
  }
}
```

**Response:** `201 Created`
```json
{
  "planId": "RESTORE-2025-001",
  "plotId": "PLOT-FARM-A-001",
  "status": "active",
  "createdDate": "2025-12-25T10:30:00Z",
  "currentHealth": 65.0,
  "targetHealth": 85.0,
  "estimatedDuration": {
    "months": 18,
    "completionDate": "2027-06-25"
  },
  "practices": [
    {
      "practiceId": "COVER-CROP-001",
      "name": "Cover Cropping",
      "schedule": "Fall 2025, Fall 2026",
      "expectedImpact": "+0.5% organic matter/year",
      "estimatedCost": 500
    },
    {
      "practiceId": "COMPOST-001",
      "name": "Composting",
      "schedule": "Spring 2026, Spring 2027",
      "expectedImpact": "+1.0% organic matter/year",
      "estimatedCost": 2000
    }
  ],
  "milestones": [
    {
      "phase": 1,
      "targetDate": "2026-06-25",
      "targetScore": 70,
      "practices": ["COVER-CROP-001"]
    }
  ],
  "totalEstimatedCost": 4500,
  "roi": {
    "yieldIncrease": "15-20%",
    "carbonCredits": "2.5 tons CO2e",
    "waterRetention": "+30%"
  }
}
```

### 4.2 Get Restoration Plan

**GET** `/restore/{planId}`

Retrieve restoration plan details.

**Response:** `200 OK`
```json
{
  "planId": "RESTORE-2025-001",
  "plotId": "PLOT-FARM-A-001",
  "status": "in_progress",
  "progress": 45.5,
  "currentHealth": 72.5,
  "targetHealth": 85.0,
  "upcomingActions": [
    {
      "action": "Apply compost",
      "dueDate": "2026-04-01",
      "status": "pending"
    }
  ]
}
```

### 4.3 Update Restoration Progress

**PUT** `/restore/{planId}/progress`

Update progress on restoration plan.

**Request:**
```json
{
  "completedAction": "COVER-CROP-001",
  "completionDate": "2025-10-15",
  "actualCost": 520,
  "notes": "Used winter rye and crimson clover mix",
  "newHealthScore": 68.5
}
```

**Response:** `200 OK`
```json
{
  "planId": "RESTORE-2025-001",
  "status": "updated",
  "progress": 33.3,
  "nextAction": {
    "practiceId": "COMPOST-001",
    "scheduledDate": "2026-04-01"
  }
}
```

---

## 5. Carbon Sequestration Endpoints

### 5.1 Get Carbon Levels

**GET** `/carbon/{plotId}`

Retrieve carbon sequestration data for a plot.

**Response:** `200 OK`
```json
{
  "plotId": "PLOT-FARM-A-001",
  "areaHectares": 10.5,
  "currentCarbon": {
    "totalCarbon": 2.03,
    "organicCarbon": 1.75,
    "inorganicCarbon": 0.28,
    "unit": "tons/hectare"
  },
  "sequestrationRate": {
    "annual": 0.5,
    "unit": "tons/hectare/year"
  },
  "totalSequestered": {
    "since": "2020-01-01",
    "amount": 26.25,
    "unit": "tons CO2e"
  },
  "baseline": {
    "date": "2020-01-01",
    "organicCarbon": 1.25
  },
  "projections": {
    "2026": 2.25,
    "2027": 2.75,
    "2028": 3.25
  },
  "carbonCredits": {
    "verified": 20.0,
    "pending": 6.25,
    "projected": 52.5,
    "unit": "tons CO2e"
  }
}
```

### 5.2 Calculate Carbon Credits

**POST** `/carbon/credits`

Calculate eligible carbon credits.

**Request:**
```json
{
  "plotId": "PLOT-FARM-A-001",
  "baselineDate": "2020-01-01",
  "currentDate": "2025-12-25",
  "baselineCarbon": 1.25,
  "currentCarbon": 1.75,
  "areaHectares": 10.5,
  "methodology": "soil_organic_carbon",
  "verificationLevel": "third_party"
}
```

**Response:** `200 OK`
```json
{
  "eligibleCredits": {
    "amount": 20.0,
    "unit": "tons CO2e",
    "monetaryValue": {
      "estimated": 500,
      "currency": "USD",
      "pricePerTon": 25
    }
  },
  "calculation": {
    "carbonIncrease": 0.5,
    "areaHectares": 10.5,
    "totalSequestered": 5.25,
    "co2Equivalent": 20.0,
    "conversionFactor": 3.67
  },
  "verification": {
    "required": true,
    "estimatedCost": 200,
    "certificationBody": "WIA-Certified Auditor",
    "timeline": "4-6 weeks"
  },
  "validUntil": "2026-12-25"
}
```

---

## 6. Analytics Endpoints

### 6.1 Get Regional Statistics

**GET** `/analytics/regional`

Get aggregated soil health statistics for a region.

**Query Parameters:**
- `bounds`: Geographic bounding box
- `startDate`, `endDate`: Date range

**Response:** `200 OK`
```json
{
  "region": {
    "north": 38.0,
    "south": 37.0,
    "east": -122.0,
    "west": -123.0
  },
  "timeRange": {
    "start": "2025-01-01",
    "end": "2025-12-25"
  },
  "statistics": {
    "totalPlots": 1250,
    "averageHealth": 72.5,
    "healthDistribution": {
      "excellent": 125,
      "good": 625,
      "fair": 375,
      "poor": 100,
      "critical": 25
    },
    "averageOrganicMatter": 3.2,
    "totalCarbonSequestered": 1575.0,
    "degradationRisk": {
      "low": 875,
      "moderate": 250,
      "high": 100,
      "critical": 25
    }
  }
}
```

### 6.2 Generate Trend Report

**POST** `/analytics/trends`

Generate trend analysis report.

**Request:**
```json
{
  "plotIds": ["PLOT-001", "PLOT-002"],
  "metrics": ["health_score", "organic_matter", "carbon"],
  "timeRange": {
    "start": "2020-01-01",
    "end": "2025-12-25"
  },
  "interval": "quarterly"
}
```

**Response:** `200 OK`
```json
{
  "reportId": "TREND-2025-001",
  "generatedAt": "2025-12-25T10:30:00Z",
  "summary": {
    "overallTrend": "improving",
    "averageImprovement": "+12.5%",
    "bestPerformer": "PLOT-001"
  },
  "trends": [
    {
      "plotId": "PLOT-001",
      "metric": "health_score",
      "data": [
        { "date": "2020-Q1", "value": 60 },
        { "date": "2025-Q4", "value": 78 }
      ],
      "trend": "improving",
      "changeRate": "+3.0 points/year"
    }
  ],
  "downloadUrl": "/reports/TREND-2025-001.pdf"
}
```

---

## 7. Error Responses

### 7.1 Error Format

```json
{
  "error": {
    "code": "INVALID_SAMPLE_DATA",
    "message": "pH value out of valid range (0-14)",
    "details": {
      "field": "soilProperties.ph",
      "value": 15.5,
      "constraint": "max: 14"
    },
    "timestamp": "2025-12-25T10:30:00Z",
    "requestId": "req_abc123"
  }
}
```

### 7.2 Common Error Codes

- `400` - Bad Request
  - `INVALID_SAMPLE_DATA`
  - `MISSING_REQUIRED_FIELD`
  - `INVALID_DATE_RANGE`
- `401` - Unauthorized
  - `INVALID_API_KEY`
  - `EXPIRED_TOKEN`
- `403` - Forbidden
  - `INSUFFICIENT_PERMISSIONS`
  - `QUOTA_EXCEEDED`
- `404` - Not Found
  - `PLOT_NOT_FOUND`
  - `SAMPLE_NOT_FOUND`
  - `PLAN_NOT_FOUND`
- `429` - Too Many Requests
  - `RATE_LIMIT_EXCEEDED`
- `500` - Internal Server Error
  - `ANALYSIS_FAILED`
  - `DATABASE_ERROR`

---

## 8. Webhooks

### 8.1 Register Webhook

**POST** `/webhooks`

```json
{
  "url": "https://your-app.com/webhook",
  "events": [
    "sample.analyzed",
    "health.degradation_warning",
    "restoration.milestone_reached",
    "carbon.credits_verified"
  ],
  "secret": "your_webhook_secret"
}
```

### 8.2 Webhook Events

**Sample Analyzed:**
```json
{
  "event": "sample.analyzed",
  "timestamp": "2025-12-25T12:30:00Z",
  "data": {
    "sampleId": "SOIL-2025-001-ABC",
    "plotId": "PLOT-FARM-A-001",
    "healthScore": 75.5
  }
}
```

---

## 9. SDK Examples

### 9.1 JavaScript/TypeScript

```typescript
import { SoilRestorationAPI } from '@wia/soil-restoration';

const api = new SoilRestorationAPI({
  apiKey: process.env.WIA_API_KEY
});

// Record soil sample
const sample = await api.monitor({
  plotId: 'PLOT-FARM-A-001',
  location: { latitude: 37.7749, longitude: -122.4194 },
  soilProperties: { ph: 6.5, organicMatter: 3.5 }
});

// Get health assessment
const health = await api.getHealth('PLOT-FARM-A-001');
console.log('Health Score:', health.overallScore);

// Create restoration plan
const plan = await api.createRestorationPlan({
  plotId: 'PLOT-FARM-A-001',
  currentHealth: 65,
  targetHealth: 85
});
```

### 9.2 Python

```python
from wia_soil_restoration import SoilRestorationAPI

api = SoilRestorationAPI(api_key=os.environ['WIA_API_KEY'])

# Record soil sample
sample = api.monitor(
    plot_id='PLOT-FARM-A-001',
    location={'latitude': 37.7749, 'longitude': -122.4194},
    soil_properties={'ph': 6.5, 'organic_matter': 3.5}
)

# Get carbon levels
carbon = api.get_carbon('PLOT-FARM-A-001')
print(f"Carbon Sequestered: {carbon['totalSequestered']['amount']} tons")
```

---

**Philosophy:** 弘益人間 (홍익인간) - Benefit All Humanity

© 2025 SmileStory Inc. / WIA
