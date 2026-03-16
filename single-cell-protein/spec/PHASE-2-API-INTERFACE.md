# WIA-AGRI-034: Phase 2 - API Interface Specification

**Standard ID:** WIA-AGRI-034
**Title:** Single Cell Protein Production API Interface
**Version:** 1.0
**Status:** Active
**Last Updated:** 2025-01-15

---

## 1. Overview

This specification defines the RESTful API interface for WIA-AGRI-034 Single Cell Protein production systems, enabling programmatic access to batch management, real-time monitoring, quality analysis, and optimization services.

### 1.1 API Design Principles

- **RESTful**: Resource-oriented architecture following REST best practices
- **Versioned**: API version in URL path for backward compatibility
- **Secure**: OAuth 2.0 / API key authentication with HTTPS required
- **Paginated**: Large result sets use cursor-based pagination
- **Rate-limited**: Fair usage policies to ensure system stability

### 1.2 Base URL

```
Production: https://api.wia.org/agri-034/v1
Staging: https://api-staging.wia.org/agri-034/v1
```

---

## 2. Authentication

### 2.1 API Key Authentication

```http
GET /api/v1/batches HTTP/1.1
Host: api.wia.org
Authorization: Bearer YOUR_API_KEY
Content-Type: application/json
```

### 2.2 OAuth 2.0 Flow

```http
POST /oauth/token HTTP/1.1
Host: api.wia.org
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials
&client_id=YOUR_CLIENT_ID
&client_secret=YOUR_CLIENT_SECRET
&scope=batches:read batches:write quality:read
```

**Response:**
```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "batches:read batches:write quality:read"
}
```

---

## 3. Core API Endpoints

### 3.1 Batch Management

#### 3.1.1 List Batches

```http
GET /api/v1/batches
```

**Query Parameters:**
- `status` (optional): Filter by status (active, completed, failed)
- `facility_id` (optional): Filter by facility
- `strain_type` (optional): Filter by microbial strain type
- `start_date` (optional): ISO8601 date
- `end_date` (optional): ISO8601 date
- `limit` (optional): Results per page (default: 50, max: 100)
- `cursor` (optional): Pagination cursor

**Response 200:**
```json
{
  "data": [
    {
      "batch_id": "BATCH-20250115-001",
      "status": "active",
      "facility": {
        "id": "FAC-001",
        "name": "SCP Production Center Alpha"
      },
      "strain": {
        "id": "STRAIN-001",
        "strain_name": "Methylophilus methylotrophus WIA-M1",
        "taxonomy": {
          "genus": "Methylophilus",
          "species": "methylotrophus"
        }
      },
      "start_time": "2025-01-15T08:00:00Z",
      "current_metrics": {
        "biomass_g_L": 38.7,
        "protein_yield_g_L": 20.1,
        "duration_hours": 28.5
      },
      "created_at": "2025-01-15T08:00:00Z",
      "updated_at": "2025-01-15T12:30:00Z"
    }
  ],
  "pagination": {
    "next_cursor": "eyJpZCI6MTIzNDU2fQ==",
    "has_more": true,
    "total_count": 247
  }
}
```

#### 3.1.2 Create Batch

```http
POST /api/v1/batches
```

**Request Body:**
```json
{
  "facility_id": "FAC-001",
  "strain_id": "STRAIN-001",
  "reactor_type": "stirred_tank",
  "working_volume_L": 1000,
  "substrate": {
    "primary_source": "methanol",
    "concentration_g_L": 50
  },
  "process_parameters": {
    "temperature": {
      "setpoint_C": 37
    },
    "pH": {
      "setpoint": 6.5,
      "control_agent_acid": "H2SO4",
      "control_agent_base": "NaOH"
    },
    "dissolved_oxygen": {
      "setpoint_percent": 40
    },
    "agitation": {
      "speed_rpm": 300
    }
  },
  "nutrient_feed": {
    "nitrogen_source": "ammonium_sulfate",
    "nitrogen_g_L": 2.5,
    "phosphorus_mg_L": 150,
    "potassium_mg_L": 200
  },
  "metadata": {
    "operator": "John Doe",
    "purpose": "production",
    "notes": "Standard production batch"
  }
}
```

**Response 201:**
```json
{
  "batch_id": "BATCH-20250115-002",
  "status": "initializing",
  "message": "Batch created successfully",
  "created_at": "2025-01-15T14:00:00Z",
  "estimated_duration_hours": 48,
  "links": {
    "self": "/api/v1/batches/BATCH-20250115-002",
    "monitor": "/api/v1/batches/BATCH-20250115-002/stream",
    "control": "/api/v1/batches/BATCH-20250115-002/control"
  }
}
```

#### 3.1.3 Get Batch Details

```http
GET /api/v1/batches/{batch_id}
```

**Response 200:**
```json
{
  "batch_id": "BATCH-20250115-001",
  "version": "1.0",
  "protocol": "WIA-AGRI-034",
  "status": "active",
  "facility": { /* ... */ },
  "strain": { /* ... */ },
  "fermentation": { /* ... */ },
  "current_sensor_data": { /* ... */ },
  "production_metrics": { /* ... */ },
  "timeline": [
    {
      "timestamp": "2025-01-15T08:00:00Z",
      "event": "batch_started",
      "details": "Inoculation completed"
    },
    {
      "timestamp": "2025-01-15T12:00:00Z",
      "event": "exponential_phase_reached",
      "details": "Cell density: 25 g/L"
    }
  ],
  "created_at": "2025-01-15T08:00:00Z",
  "updated_at": "2025-01-15T14:30:00Z"
}
```

#### 3.1.4 Update Batch Parameters

```http
PATCH /api/v1/batches/{batch_id}
```

**Request Body:**
```json
{
  "process_parameters": {
    "temperature": {
      "setpoint_C": 38
    },
    "dissolved_oxygen": {
      "setpoint_percent": 45
    }
  },
  "notes": "Increased temperature and DO based on optimization recommendation"
}
```

**Response 200:**
```json
{
  "batch_id": "BATCH-20250115-001",
  "status": "active",
  "message": "Parameters updated successfully",
  "updated_parameters": {
    "temperature": {
      "old_setpoint_C": 37,
      "new_setpoint_C": 38
    },
    "dissolved_oxygen": {
      "old_setpoint_percent": 40,
      "new_setpoint_percent": 45
    }
  },
  "updated_at": "2025-01-15T14:35:00Z"
}
```

#### 3.1.5 Complete Batch

```http
POST /api/v1/batches/{batch_id}/complete
```

**Request Body:**
```json
{
  "completion_reason": "normal | early_termination | contamination | equipment_failure",
  "final_notes": "Batch completed successfully with excellent yield",
  "trigger_quality_analysis": true
}
```

**Response 200:**
```json
{
  "batch_id": "BATCH-20250115-001",
  "status": "completed",
  "completion_time": "2025-01-17T12:00:00Z",
  "duration_hours": 52,
  "final_metrics": {
    "biomass_yield_g_L": 45.2,
    "protein_yield_g_L": 23.6,
    "substrate_conversion_percent": 78.5
  },
  "quality_analysis_id": "QA-20250117-001"
}
```

---

### 3.2 Real-time Monitoring

#### 3.2.1 Get Current Sensor Data

```http
GET /api/v1/batches/{batch_id}/sensors
```

**Response 200:**
```json
{
  "batch_id": "BATCH-20250115-001",
  "timestamp": "2025-01-15T14:40:00Z",
  "measurements": [
    {
      "sensor_id": "TEMP-001",
      "parameter": "temperature",
      "value": 37.2,
      "unit": "C",
      "setpoint": 37.0,
      "deviation": 0.2,
      "quality": "good",
      "last_calibration": "2025-01-10T10:00:00Z"
    },
    {
      "sensor_id": "PH-001",
      "parameter": "pH",
      "value": 6.48,
      "unit": "pH",
      "setpoint": 6.50,
      "deviation": -0.02,
      "quality": "good",
      "last_calibration": "2025-01-12T09:00:00Z"
    },
    {
      "sensor_id": "DO-001",
      "parameter": "dissolved_oxygen",
      "value": 42.5,
      "unit": "percent",
      "setpoint": 40.0,
      "deviation": 2.5,
      "quality": "good",
      "last_calibration": "2025-01-13T14:00:00Z"
    }
  ],
  "biomass": {
    "optical_density_600nm": 4.25,
    "dry_cell_weight_g_L": 38.7,
    "estimated_protein_g_L": 20.1
  }
}
```

#### 3.2.2 WebSocket Stream (Real-time)

```javascript
// WebSocket connection for real-time streaming
ws://api.wia.org/agri-034/v1/batches/{batch_id}/stream

// Connection request
{
  "action": "subscribe",
  "batch_id": "BATCH-20250115-001",
  "parameters": ["temperature", "pH", "dissolved_oxygen", "biomass"],
  "interval_seconds": 5
}

// Streaming data (every 5 seconds)
{
  "timestamp": "2025-01-15T14:40:05Z",
  "batch_id": "BATCH-20250115-001",
  "data": {
    "temperature_C": 37.2,
    "pH": 6.48,
    "dissolved_oxygen_percent": 42.5,
    "biomass_g_L": 38.7
  }
}
```

#### 3.2.3 Get Historical Data

```http
GET /api/v1/batches/{batch_id}/history
```

**Query Parameters:**
- `parameter`: Specific parameter (temperature, pH, biomass, etc.)
- `start_time`: ISO8601 datetime
- `end_time`: ISO8601 datetime
- `interval`: Data aggregation interval (1min, 5min, 1hour)
- `limit`: Maximum data points (default: 1000, max: 10000)

**Response 200:**
```json
{
  "batch_id": "BATCH-20250115-001",
  "parameter": "temperature",
  "unit": "C",
  "interval": "5min",
  "data_points": [
    {
      "timestamp": "2025-01-15T08:00:00Z",
      "value": 37.0,
      "min": 36.8,
      "max": 37.2,
      "avg": 37.0,
      "stddev": 0.1
    },
    {
      "timestamp": "2025-01-15T08:05:00Z",
      "value": 37.1,
      "min": 36.9,
      "max": 37.3,
      "avg": 37.1,
      "stddev": 0.1
    }
  ],
  "total_points": 576,
  "statistics": {
    "overall_min": 36.5,
    "overall_max": 37.8,
    "overall_avg": 37.1,
    "overall_stddev": 0.3
  }
}
```

---

### 3.3 Quality Analysis

#### 3.3.1 Submit Quality Sample

```http
POST /api/v1/quality/samples
```

**Request Body:**
```json
{
  "batch_id": "BATCH-20250115-001",
  "sample_time": "2025-01-17T12:00:00Z",
  "sample_volume_L": 1.0,
  "laboratory": "Quality Lab A",
  "tests_requested": [
    "protein_content",
    "amino_acid_profile",
    "digestibility",
    "safety_testing"
  ],
  "priority": "standard | urgent",
  "notes": "Final batch sample for quality certification"
}
```

**Response 201:**
```json
{
  "sample_id": "SAMPLE-20250117-001",
  "quality_analysis_id": "QA-20250117-001",
  "status": "pending",
  "estimated_completion": "2025-01-18T12:00:00Z",
  "tracking_url": "/api/v1/quality/QA-20250117-001"
}
```

#### 3.3.2 Get Quality Analysis Results

```http
GET /api/v1/quality/{analysis_id}
```

**Response 200:**
```json
{
  "quality_analysis_id": "QA-20250117-001",
  "batch_id": "BATCH-20250115-001",
  "sample_id": "SAMPLE-20250117-001",
  "analysis_date": "2025-01-18T10:00:00Z",
  "status": "completed",
  "laboratory": "Quality Lab A",
  "accreditation": "ISO17025",
  "composition": {
    "protein_content": {
      "value_percent": 52.3,
      "method": "Kjeldahl",
      "nitrogen_to_protein_factor": 6.25
    },
    "moisture_percent": 8.2,
    "ash_percent": 7.5,
    "fat_percent": 3.1
  },
  "amino_acid_profile": {
    "method": "HPLC",
    "essential_amino_acids": {
      "lysine_g_100g": 6.8,
      "methionine_g_100g": 2.1,
      "threonine_g_100g": 4.5,
      "tryptophan_g_100g": 1.3,
      "leucine_g_100g": 7.2,
      "isoleucine_g_100g": 4.8,
      "valine_g_100g": 5.1,
      "phenylalanine_g_100g": 4.3,
      "histidine_g_100g": 2.4
    },
    "amino_acid_score": 0.94,
    "limiting_amino_acid": "methionine"
  },
  "digestibility": {
    "in_vitro_percent": 87.5,
    "pdcaas": 0.82
  },
  "safety_testing": {
    "microbial_contamination": {
      "total_plate_count_cfu_g": 150,
      "salmonella": "not_detected",
      "e_coli": "not_detected"
    },
    "heavy_metals": {
      "lead_mg_kg": 0.05,
      "cadmium_mg_kg": 0.02,
      "mercury_mg_kg": 0.01,
      "arsenic_mg_kg": 0.03
    },
    "all_tests_passed": true
  },
  "overall_grade": "A",
  "certification_status": ["non_gmo", "vegan"],
  "pdf_report_url": "/api/v1/quality/QA-20250117-001/report.pdf"
}
```

---

### 3.4 Analytics & Optimization

#### 3.4.1 Get Optimization Recommendations

```http
POST /api/v1/analytics/optimize
```

**Request Body:**
```json
{
  "batch_id": "BATCH-20250115-001",
  "optimization_goals": [
    {
      "metric": "protein_yield",
      "target": "maximize",
      "weight": 0.5
    },
    {
      "metric": "production_cost",
      "target": "minimize",
      "weight": 0.3
    },
    {
      "metric": "environmental_impact",
      "target": "minimize",
      "weight": 0.2
    }
  ],
  "constraints": {
    "temperature_C": {
      "min": 35,
      "max": 40
    },
    "pH": {
      "min": 6.0,
      "max": 7.0
    }
  }
}
```

**Response 200:**
```json
{
  "batch_id": "BATCH-20250115-001",
  "optimization_id": "OPT-20250115-001",
  "timestamp": "2025-01-15T15:00:00Z",
  "model_version": "2.1.0",
  "confidence_score": 0.92,
  "recommendations": [
    {
      "parameter": "temperature",
      "current_value": 37.0,
      "optimal_value": 38.2,
      "unit": "C",
      "expected_improvement": {
        "protein_yield_increase_percent": 8.3,
        "cost_impact_usd_kg": -0.12
      },
      "implementation_priority": "high",
      "notes": "Increase temperature gradually over 2 hours"
    },
    {
      "parameter": "dissolved_oxygen",
      "current_value": 40,
      "optimal_value": 45,
      "unit": "percent",
      "expected_improvement": {
        "growth_rate_increase_percent": 5.2,
        "energy_cost_increase_usd_kg": 0.05
      },
      "implementation_priority": "medium",
      "notes": "Increase air flow rate by 12%"
    },
    {
      "parameter": "nutrient_feed_rate",
      "current_value": 100,
      "optimal_value": 115,
      "unit": "g/h",
      "expected_improvement": {
        "protein_content_increase_percent": 12.1,
        "substrate_cost_increase_usd_kg": 0.08
      },
      "implementation_priority": "high",
      "notes": "Implement fed-batch strategy"
    }
  ],
  "predicted_outcomes": {
    "protein_yield_g_L": 26.8,
    "improvement_percent": 33.2,
    "production_cost_usd_kg": 2.87,
    "cost_reduction_percent": 8.5,
    "carbon_footprint_kg_co2eq": 1.92,
    "emission_reduction_percent": 12.3
  }
}
```

#### 3.4.2 Generate Production Report

```http
GET /api/v1/analytics/reports/{batch_id}
```

**Query Parameters:**
- `format`: pdf | json | csv
- `include`: sections to include (summary, details, charts, recommendations)

**Response 200 (JSON format):**
```json
{
  "report_id": "RPT-20250118-001",
  "batch_id": "BATCH-20250115-001",
  "generated_at": "2025-01-18T14:00:00Z",
  "summary": {
    "batch_duration_hours": 52,
    "final_biomass_g_L": 45.2,
    "protein_yield_g_L": 23.6,
    "substrate_conversion_percent": 78.5,
    "quality_grade": "A",
    "production_cost_usd_kg": 2.95
  },
  "performance_vs_targets": {
    "yield_target_achievement_percent": 118,
    "quality_target_achievement_percent": 112,
    "cost_target_achievement_percent": 95
  },
  "recommendations_for_next_batch": [
    "Maintain optimized temperature profile",
    "Consider increasing working volume to 1200L",
    "Implement automated pH control"
  ],
  "pdf_url": "/api/v1/analytics/reports/RPT-20250118-001.pdf"
}
```

---

## 4. Error Handling

### 4.1 Error Response Format

```json
{
  "error": {
    "code": "BATCH_NOT_FOUND",
    "message": "Batch with ID 'BATCH-99999-001' does not exist",
    "status": 404,
    "timestamp": "2025-01-15T15:30:00Z",
    "request_id": "req_abc123xyz",
    "documentation_url": "https://docs.wia.org/agri-034/errors#BATCH_NOT_FOUND"
  }
}
```

### 4.2 Common Error Codes

| Status | Code | Description |
|--------|------|-------------|
| 400 | INVALID_REQUEST | Request validation failed |
| 401 | UNAUTHORIZED | Authentication required |
| 403 | FORBIDDEN | Insufficient permissions |
| 404 | RESOURCE_NOT_FOUND | Requested resource does not exist |
| 409 | CONFLICT | Resource state conflict |
| 422 | VALIDATION_ERROR | Data validation failed |
| 429 | RATE_LIMIT_EXCEEDED | Too many requests |
| 500 | INTERNAL_ERROR | Server error |
| 503 | SERVICE_UNAVAILABLE | Service temporarily unavailable |

---

## 5. Rate Limiting

### 5.1 Rate Limit Headers

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 987
X-RateLimit-Reset: 1642253400
```

### 5.2 Rate Limits by Tier

| Tier | Requests/Hour | Burst |
|------|---------------|-------|
| Free | 100 | 20 |
| Standard | 1,000 | 50 |
| Professional | 10,000 | 200 |
| Enterprise | Custom | Custom |

---

## 6. Webhooks

### 6.1 Configure Webhook

```http
POST /api/v1/webhooks
```

**Request Body:**
```json
{
  "url": "https://your-server.com/wia-webhook",
  "events": [
    "batch.created",
    "batch.completed",
    "batch.failed",
    "quality.analysis.completed",
    "alert.threshold.exceeded"
  ],
  "secret": "your-webhook-secret",
  "active": true
}
```

### 6.2 Webhook Payload Example

```json
{
  "event": "batch.completed",
  "timestamp": "2025-01-17T12:00:00Z",
  "data": {
    "batch_id": "BATCH-20250115-001",
    "status": "completed",
    "final_metrics": {
      "protein_yield_g_L": 23.6,
      "quality_grade": "A"
    }
  },
  "signature": "sha256=abc123..."
}
```

---

## 7. SDK Examples

### 7.1 TypeScript/JavaScript

```typescript
import { WIAAGRI034Client } from '@wia/agri-034';

const client = new WIAAGRI034Client({
  apiKey: process.env.WIA_API_KEY
});

// Create batch
const batch = await client.batches.create({
  facility_id: 'FAC-001',
  strain_id: 'STRAIN-001',
  working_volume_L: 1000
});

// Monitor real-time
client.batches.stream(batch.id, (data) => {
  console.log('Current protein yield:', data.protein_yield_g_L);
});

// Get optimization
const optimization = await client.analytics.optimize(batch.id);
console.log('Recommendations:', optimization.recommendations);
```

### 7.2 Python

```python
from wia_agri_034 import Client

client = Client(api_key=os.environ['WIA_API_KEY'])

# Create batch
batch = client.batches.create(
    facility_id='FAC-001',
    strain_id='STRAIN-001',
    working_volume_L=1000
)

# Get quality results
quality = client.quality.get(analysis_id='QA-20250117-001')
print(f"Protein content: {quality.composition.protein_content.value_percent}%")
```

---

**弘益人間 (홍익인간) - Benefit All Humanity**

© 2025 SmileStory Inc. / WIA
