# WIA-ENE-060: Wetland Conservation
## PHASE 2 - API SPECIFICATION

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-12-25

---

## Overview

This specification defines the RESTful API for wetland conservation data management, monitoring, and reporting. The API enables real-time data submission, retrieval, and analysis of wetland health metrics.

## Base URL

```
Production: https://api.wia.org/v1/wetland-conservation
Sandbox: https://sandbox-api.wia.org/v1/wetland-conservation
```

## Authentication

All API requests require authentication using API keys or OAuth 2.0 tokens.

```http
Authorization: Bearer {access_token}
X-API-Key: {api_key}
```

## Core Endpoints

### 1. Wetland Management

#### List All Wetlands

```http
GET /wetlands
```

**Query Parameters:**
- `country` (optional): ISO 3166-1 alpha-2 country code
- `ramsar` (optional): boolean - filter Ramsar sites
- `type` (optional): wetland type filter
- `page` (optional): page number (default: 1)
- `limit` (optional): items per page (default: 20, max: 100)

**Response (200 OK):**
```json
{
  "success": true,
  "data": {
    "wetlands": [
      {
        "id": "WL-OKAV-2025",
        "name": "Okavango Delta",
        "type": "riverine",
        "country": "BW",
        "area_hectares": 1500000,
        "ramsar_site": true,
        "health_score": 92,
        "last_monitored": "2025-12-25T10:00:00Z"
      }
    ],
    "pagination": {
      "page": 1,
      "limit": 20,
      "total": 245,
      "pages": 13
    }
  }
}
```

#### Get Wetland Details

```http
GET /wetlands/{id}
```

**Response (200 OK):**
```json
{
  "success": true,
  "data": {
    "wetland": {
      "id": "WL-OKAV-2025",
      "name": "Okavango Delta",
      "type": "riverine",
      "ramsar_site": true,
      "ramsar_id": "RS1234",
      "location": {
        "latitude": -19.2833,
        "longitude": 22.7333,
        "country": "BW",
        "region": "Northwest District"
      },
      "area_hectares": 1500000,
      "health_score": 92,
      "last_monitored": "2025-12-25T10:00:00Z"
    }
  }
}
```

#### Register New Wetland

```http
POST /wetlands
```

**Request Body:**
```json
{
  "name": "Everglades National Park",
  "type": "coastal",
  "location": {
    "latitude": 25.3197,
    "longitude": -80.9284,
    "country": "US",
    "region": "Florida"
  },
  "area_hectares": 610685,
  "ramsar_site": true,
  "established_date": "1987-06-04"
}
```

**Response (201 Created):**
```json
{
  "success": true,
  "data": {
    "wetland": {
      "id": "WL-EVER-2025",
      "name": "Everglades National Park",
      "created_at": "2025-12-25T12:00:00Z"
    }
  }
}
```

#### Update Wetland Information

```http
PUT /wetlands/{id}
```

**Request Body:**
```json
{
  "area_hectares": 610700,
  "protection_status": "unesco"
}
```

**Response (200 OK):**
```json
{
  "success": true,
  "data": {
    "wetland": {
      "id": "WL-EVER-2025",
      "updated_at": "2025-12-25T12:30:00Z"
    }
  }
}
```

### 2. Monitoring Data Submission

#### Submit Water Quality Data

```http
POST /wetlands/{id}/monitoring/water-quality
```

**Request Body:**
```json
{
  "timestamp": "2025-12-25T10:00:00Z",
  "ph": 7.2,
  "dissolved_oxygen_mg_l": 8.5,
  "temperature_celsius": 24.5,
  "turbidity_ntu": 12,
  "nutrients": {
    "nitrate_mg_l": 0.8,
    "phosphate_mg_l": 0.15,
    "ammonia_mg_l": 0.05
  },
  "sampling_location": {
    "latitude": -19.2833,
    "longitude": 22.7333
  },
  "sampler_id": "SAMPLER-001"
}
```

**Response (201 Created):**
```json
{
  "success": true,
  "data": {
    "monitoring_id": "MON-WQ-20251225-001",
    "quality_index": 88,
    "status": "healthy",
    "alerts": []
  }
}
```

#### Submit Biodiversity Survey

```http
POST /wetlands/{id}/monitoring/biodiversity
```

**Request Body:**
```json
{
  "survey_date": "2025-12-20",
  "surveyor_id": "BIO-SURVEYOR-001",
  "birds": {
    "species_count": 444,
    "individual_count": 200000,
    "endangered_species": [
      {
        "scientific_name": "Grus carunculata",
        "common_name": "Wattled Crane",
        "iucn_status": "VU",
        "population_estimate": 250
      }
    ]
  },
  "amphibians": {
    "species_count": 33,
    "individual_count": 150000
  },
  "fish": {
    "species_count": 71,
    "native_species_ratio": 0.97
  }
}
```

**Response (201 Created):**
```json
{
  "success": true,
  "data": {
    "survey_id": "SUR-BIO-20251220-001",
    "biodiversity_index": 94,
    "trend": "stable",
    "notable_observations": [
      "Wattled Crane population stable"
    ]
  }
}
```

#### Submit Vegetation Assessment

```http
POST /wetlands/{id}/monitoring/vegetation
```

**Request Body:**
```json
{
  "assessment_date": "2025-12-15",
  "coverage_percent": 85,
  "native_species_ratio": 0.98,
  "dominant_species": [
    {
      "scientific_name": "Cyperus papyrus",
      "common_name": "Papyrus",
      "coverage_percent": 35
    }
  ],
  "invasive_species": []
}
```

**Response (201 Created):**
```json
{
  "success": true,
  "data": {
    "assessment_id": "ASS-VEG-20251215-001",
    "health_index": 92,
    "recommendations": [
      "Continue monitoring for invasive species"
    ]
  }
}
```

### 3. Health & Analytics

#### Get Ecosystem Health Score

```http
GET /wetlands/{id}/health
```

**Query Parameters:**
- `period` (optional): time period (day, week, month, year)
- `include_trends` (optional): boolean

**Response (200 OK):**
```json
{
  "success": true,
  "data": {
    "wetland_id": "WL-OKAV-2025",
    "overall_health_score": 92,
    "last_calculated": "2025-12-25T12:00:00Z",
    "components": {
      "water_quality": {
        "score": 88,
        "status": "excellent",
        "trend": "stable"
      },
      "biodiversity": {
        "score": 94,
        "status": "excellent",
        "trend": "improving"
      },
      "vegetation": {
        "score": 92,
        "status": "excellent",
        "trend": "stable"
      },
      "hydrology": {
        "score": 90,
        "status": "excellent",
        "trend": "stable"
      }
    },
    "threats": {
      "overall_level": "low",
      "active_threats": 2
    },
    "recommendations": [
      "Continue regular monitoring",
      "Maintain current conservation efforts"
    ]
  }
}
```

#### Get Ecosystem Services Valuation

```http
GET /wetlands/{id}/ecosystem-services
```

**Response (200 OK):**
```json
{
  "success": true,
  "data": {
    "wetland_id": "WL-OKAV-2025",
    "calculated_date": "2025-12-25",
    "services": {
      "water_purification": {
        "capacity_liters_per_day": 150000000,
        "annual_value_usd": 45000000
      },
      "flood_mitigation": {
        "storage_capacity_m3": 18000000000,
        "annual_value_usd": 125000000
      },
      "carbon_sequestration": {
        "rate_tons_per_year": 3750000,
        "annual_value_usd": 187500000
      },
      "habitat_provision": {
        "species_supported": 4500,
        "annual_value_usd": 95000000
      },
      "recreation": {
        "annual_visitors": 85000,
        "economic_value_usd": 178000000
      }
    },
    "total_annual_value_usd": 630500000
  }
}
```

#### Get Historical Trends

```http
GET /wetlands/{id}/trends
```

**Query Parameters:**
- `metric`: water_quality | biodiversity | vegetation | health_score
- `start_date`: ISO 8601 date
- `end_date`: ISO 8601 date
- `interval`: day | week | month | year

**Response (200 OK):**
```json
{
  "success": true,
  "data": {
    "wetland_id": "WL-OKAV-2025",
    "metric": "health_score",
    "interval": "month",
    "data_points": [
      {
        "date": "2025-01-01",
        "value": 89
      },
      {
        "date": "2025-02-01",
        "value": 90
      },
      {
        "date": "2025-03-01",
        "value": 91
      }
    ],
    "trend": "improving",
    "change_percent": 3.37
  }
}
```

### 4. Species Tracking

#### List Species in Wetland

```http
GET /wetlands/{id}/species
```

**Query Parameters:**
- `category`: birds | amphibians | fish | plants | invertebrates
- `iucn_status` (optional): CR | EN | VU | NT | LC
- `endemic` (optional): boolean

**Response (200 OK):**
```json
{
  "success": true,
  "data": {
    "wetland_id": "WL-OKAV-2025",
    "category": "birds",
    "species": [
      {
        "scientific_name": "Grus carunculata",
        "common_name": "Wattled Crane",
        "iucn_status": "VU",
        "population_estimate": 250,
        "breeding_status": "resident",
        "last_observed": "2025-12-20"
      }
    ],
    "total_count": 444
  }
}
```

#### Report Species Observation

```http
POST /wetlands/{id}/species/observations
```

**Request Body:**
```json
{
  "observation_date": "2025-12-25T08:30:00Z",
  "observer_id": "OBS-001",
  "species": {
    "scientific_name": "Grus carunculata",
    "common_name": "Wattled Crane"
  },
  "count": 12,
  "location": {
    "latitude": -19.2833,
    "longitude": 22.7333
  },
  "behavior": "feeding",
  "habitat": "shallow water",
  "photos": [
    "https://storage.wia.org/obs/photo1.jpg"
  ]
}
```

**Response (201 Created):**
```json
{
  "success": true,
  "data": {
    "observation_id": "OBS-20251225-001",
    "verified": false,
    "contribution_points": 10
  }
}
```

### 5. Threat Assessment

#### Submit Threat Report

```http
POST /wetlands/{id}/threats
```

**Request Body:**
```json
{
  "assessment_date": "2025-12-25",
  "assessor_id": "ASSESS-001",
  "pollution": {
    "severity": "moderate",
    "sources": ["agricultural runoff", "urban wastewater"],
    "trend": "worsening"
  },
  "habitat_loss": {
    "severity": "low",
    "causes": ["development"],
    "area_affected_hectares": 25
  },
  "evidence": [
    {
      "type": "photo",
      "url": "https://storage.wia.org/threats/photo1.jpg",
      "description": "Algal bloom in northern sector"
    }
  ]
}
```

**Response (201 Created):**
```json
{
  "success": true,
  "data": {
    "threat_id": "THR-20251225-001",
    "severity": "moderate",
    "alert_sent": true,
    "recommended_actions": [
      "Increase water quality monitoring",
      "Investigate pollution sources",
      "Contact local authorities"
    ]
  }
}
```

### 6. Conservation Projects

#### List Conservation Projects

```http
GET /wetlands/{id}/projects
```

**Response (200 OK):**
```json
{
  "success": true,
  "data": {
    "wetland_id": "WL-OKAV-2025",
    "projects": [
      {
        "id": "PROJ-001",
        "name": "Delta Monitoring Network",
        "type": "monitoring",
        "status": "active",
        "start_date": "2020-01-01",
        "end_date": "2030-12-31",
        "budget_usd": 5000000,
        "lead_organization": "Okavango Research Institute"
      }
    ],
    "total_count": 5
  }
}
```

#### Create Conservation Project

```http
POST /wetlands/{id}/projects
```

**Request Body:**
```json
{
  "name": "Invasive Species Management",
  "type": "restoration",
  "description": "Remove invasive plant species and restore native vegetation",
  "start_date": "2026-01-01",
  "end_date": "2028-12-31",
  "budget_usd": 500000,
  "lead_organization": "Local Conservation NGO",
  "objectives": [
    "Remove 95% of invasive species",
    "Restore 200 hectares of native vegetation",
    "Monitor recovery for 2 years"
  ]
}
```

**Response (201 Created):**
```json
{
  "success": true,
  "data": {
    "project_id": "PROJ-006",
    "status": "planned",
    "created_at": "2025-12-25T14:00:00Z"
  }
}
```

### 7. Reporting & Certificates

#### Generate Wetland Report

```http
POST /wetlands/{id}/reports
```

**Request Body:**
```json
{
  "report_type": "annual_assessment",
  "year": 2025,
  "format": "pdf",
  "include_sections": [
    "executive_summary",
    "water_quality",
    "biodiversity",
    "threats",
    "recommendations"
  ],
  "language": "en"
}
```

**Response (200 OK):**
```json
{
  "success": true,
  "data": {
    "report_id": "REP-20251225-001",
    "download_url": "https://storage.wia.org/reports/WL-OKAV-2025-2025.pdf",
    "expires_at": "2026-01-25T00:00:00Z"
  }
}
```

#### Issue Conservation Certificate

```http
POST /wetlands/{id}/certificates
```

**Request Body:**
```json
{
  "organization_name": "Conservation International",
  "achievement_type": "monitoring",
  "period": {
    "start": "2025-01-01",
    "end": "2025-12-31"
  },
  "metrics": {
    "monitoring_compliance": 100,
    "data_quality_score": 95,
    "impact_score": 92
  }
}
```

**Response (201 Created):**
```json
{
  "success": true,
  "data": {
    "certificate_id": "CERT-20251225-001",
    "verifiable_credential": {
      "@context": [
        "https://www.w3.org/2018/credentials/v1",
        "https://wia.org/credentials/wetland/v1"
      ],
      "type": ["VerifiableCredential", "WetlandConservationCertificate"],
      "issuer": "did:wia:ene-060",
      "credentialSubject": {
        "id": "did:wia:org:conservation-international",
        "achievement": "monitoring",
        "impact_score": 92
      }
    },
    "qr_code_url": "https://storage.wia.org/qr/CERT-20251225-001.png"
  }
}
```

## Error Responses

### Standard Error Format

```json
{
  "success": false,
  "error": {
    "code": "WETLAND_NOT_FOUND",
    "message": "Wetland with ID WL-XXXX-0000 not found",
    "details": {}
  }
}
```

### Error Codes

- `400` - Bad Request
- `401` - Unauthorized
- `403` - Forbidden
- `404` - Not Found
- `422` - Validation Error
- `429` - Rate Limit Exceeded
- `500` - Internal Server Error

## Rate Limiting

- Free tier: 100 requests/hour
- Standard tier: 1,000 requests/hour
- Premium tier: 10,000 requests/hour
- Research tier: Unlimited

## Webhooks

Subscribe to real-time events:

```http
POST /webhooks
```

**Events:**
- `wetland.created`
- `monitoring.submitted`
- `health.alert`
- `species.endangered_observed`
- `threat.reported`
- `project.completed`

---

**Standard:** WIA-ENE-060
**Category:** Energy & Environment
**Philosophy:** 弘益人間 (홍익인간) - Benefit All Humanity

© 2025 SmileStory Inc. / WIA
