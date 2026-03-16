# WIA-AGRI-013: Pest Detection Standard
## Phase 2: API Interface Specification

### 2.1 Overview

Phase 2 defines the RESTful API interfaces for pest detection systems, enabling communication between mobile apps, AI detection engines, monitoring networks, and agricultural management platforms.

**Duration**: 4-6 months
**Key Outcome**: Production-ready API specification and reference implementation

### 2.2 API Architecture

#### 2.2.1 Base URL Structure

```
Production: https://api.wia-pest-detection.org/v1
Staging: https://staging-api.wia-pest-detection.org/v1
Development: http://localhost:8080/v1
```

#### 2.2.2 Authentication

All API requests MUST include authentication via Bearer token:

```http
Authorization: Bearer <JWT_TOKEN>
```

JWT token payload:
```json
{
  "sub": "user_id",
  "role": "enum [farmer, inspector, researcher, extension_officer, admin]",
  "organization": "string",
  "region": "string",
  "exp": "unix_timestamp",
  "iat": "unix_timestamp"
}
```

### 2.3 Core API Endpoints

#### 2.3.1 Pest Detection Submission

**POST /detections**

Submit a new pest detection record.

Request:
```json
{
  "detection_method": "image_ai",
  "location": {
    "field_id": "field-12345",
    "coordinates": {
      "latitude": 35.8219,
      "longitude": 127.1489
    }
  },
  "pest_identification": {
    "pest_type": "insect",
    "species_id": "Nilaparvata_lugens",
    "confidence_score": 0.92
  },
  "severity_assessment": {
    "level": "high",
    "infestation_percent": 65.0,
    "population_density": 45
  },
  "crop_information": {
    "crop_type": "rice",
    "variety": "Koshihikari",
    "growth_stage": "tillering"
  },
  "images": [
    {
      "image_url": "string (S3 URL)",
      "capture_timestamp": "2025-06-15T14:30:00Z"
    }
  ]
}
```

Response (201 Created):
```json
{
  "detection_id": "det-7f8a9b0c",
  "status": "submitted",
  "timestamp": "2025-06-15T14:32:15Z",
  "verification_required": true,
  "risk_level": "high",
  "recommended_action": "immediate_treatment",
  "next_steps": [
    "Alert sent to local extension service",
    "Expert verification requested",
    "Neighboring farmers notified"
  ]
}
```

#### 2.3.2 Image Upload for AI Analysis

**POST /analyze/image**

Upload image for AI-powered pest detection.

Request (multipart/form-data):
```
image: binary (max 20MB)
location: {"latitude": 35.8219, "longitude": 127.1489}
crop_type: "rice"
metadata: {
  "camera_model": "iPhone 14 Pro",
  "capture_date": "2025-06-15T14:30:00Z"
}
```

Response (200 OK):
```json
{
  "analysis_id": "ana-9d2e1f3a",
  "processing_time_ms": 842,
  "detections": [
    {
      "pest_id": "Nilaparvata_lugens",
      "common_name": {
        "en": "Rice Planthopper",
        "ko": "벼멸구"
      },
      "confidence": 0.94,
      "bounding_box": {
        "x": 120,
        "y": 85,
        "width": 45,
        "height": 38
      },
      "severity": "high",
      "count_estimate": 12
    }
  ],
  "overall_assessment": {
    "total_pests_detected": 12,
    "highest_confidence": 0.94,
    "risk_level": "high",
    "treatment_recommended": true
  },
  "image_quality": {
    "resolution": "4032x3024",
    "clarity_score": 0.88,
    "lighting_score": 0.91,
    "usable": true
  }
}
```

#### 2.3.3 Risk Assessment

**POST /risk/calculate**

Calculate pest risk based on environmental and crop conditions.

Request:
```json
{
  "location_id": "field-12345",
  "pest_species": "Nilaparvata_lugens",
  "crop_type": "rice",
  "growth_stage": "tillering",
  "current_density": 35,
  "weather_forecast": {
    "temperature_avg_c": 27,
    "humidity_avg_percent": 78,
    "rainfall_expected_mm": 15
  },
  "days_since_last_treatment": 21,
  "nearby_outbreaks": true
}
```

Response (200 OK):
```json
{
  "risk_score": 78,
  "risk_level": "high",
  "outbreak_probability": 0.72,
  "economic_threshold": {
    "current_density": 35,
    "threshold": 20,
    "exceeded": true,
    "days_to_threshold": -7
  },
  "recommendations": {
    "action": "immediate_treatment",
    "urgency": "24-48 hours",
    "ipm_strategy": {
      "biological_control": "Limited effectiveness at current density",
      "chemical_control": "Recommended - see treatment options",
      "cultural_practices": "Monitor adjacent fields, adjust water management"
    }
  },
  "forecast": {
    "7_day_risk_trend": "increasing",
    "peak_expected": "day 3-4",
    "factors": [
      "High temperature favorable for reproduction",
      "High humidity increases survival",
      "Nearby outbreaks indicate regional pressure"
    ]
  }
}
```

#### 2.3.4 Treatment Recommendations

**GET /treatments/recommend**

Get IPM treatment recommendations for detected pest.

Query Parameters:
- `pest_id`: string (required)
- `severity`: enum [low, medium, high, critical] (required)
- `crop_type`: string (required)
- `organic`: boolean (optional, default: false)
- `region`: string (optional)

Response (200 OK):
```json
{
  "pest_id": "Nilaparvata_lugens",
  "pest_name": {
    "en": "Rice Planthopper",
    "ko": "벼멸구"
  },
  "severity": "high",
  "ipm_recommendations": {
    "immediate_actions": [
      "Deploy pheromone traps for monitoring",
      "Scout fields every 2-3 days",
      "Alert neighboring farmers"
    ],
    "biological_control": {
      "recommended": false,
      "reason": "Population exceeds biological control effectiveness threshold",
      "alternatives": [
        {
          "method": "Predatory spiders (Lycosa pseudoannulata)",
          "effectiveness": "Limited at current density",
          "cost_per_ha": "$50-80"
        }
      ]
    },
    "chemical_control": {
      "recommended": true,
      "options": [
        {
          "product": "Imidacloprid 20% SC",
          "registration_number": "KR-농약-2021-0534",
          "application_rate": "1.0 L/ha",
          "water_volume": "100-150 L/ha",
          "phi_days": 14,
          "efficacy_rating": 0.92,
          "resistance_risk": "medium",
          "environmental_impact": "moderate",
          "cost_per_ha": "$25-35"
        },
        {
          "product": "Buprofezin 25% SC",
          "registration_number": "KR-농약-2020-0891",
          "application_rate": "0.8 kg/ha",
          "water_volume": "100-150 L/ha",
          "phi_days": 21,
          "efficacy_rating": 0.88,
          "resistance_risk": "low",
          "environmental_impact": "low",
          "cost_per_ha": "$30-40"
        }
      ],
      "application_timing": "Early morning or late evening, avoid rain within 6 hours",
      "spray_guidelines": {
        "nozzle_type": "Hollow cone",
        "pressure_bar": "2.5-3.5",
        "droplet_size": "Medium (200-300 μm)"
      }
    },
    "cultural_practices": {
      "water_management": "Drain fields for 2-3 days to reduce habitat",
      "fertilizer_adjustment": "Reduce nitrogen to slow pest reproduction",
      "crop_rotation": "Consider different variety next season",
      "sanitation": "Remove weeds that serve as alternate hosts"
    }
  },
  "resistance_management": {
    "current_resistance_status": "Moderate resistance reported in region",
    "rotation_plan": [
      "Use Imidacloprid (Group 4A) for first application",
      "Rotate to Buprofezin (Group 16) after 21 days if needed",
      "Avoid consecutive applications of same mode of action"
    ]
  },
  "cost_benefit_analysis": {
    "estimated_yield_loss_untreated": "35-50%",
    "estimated_yield_loss_treated": "5-10%",
    "treatment_cost_per_ha": "$25-40",
    "expected_yield_protection_value": "$800-1200"
  }
}
```

#### 2.3.5 Alert Subscription

**POST /alerts/subscribe**

Subscribe to pest alerts for specific region or pest types.

Request:
```json
{
  "user_id": "user-abc123",
  "subscription_type": "regional",
  "filters": {
    "regions": ["Jeonbuk", "Jeonnam"],
    "pest_types": ["insect", "fungal"],
    "severity_min": "medium",
    "crop_types": ["rice", "corn"]
  },
  "notification_channels": [
    {
      "type": "sms",
      "contact": "+82-10-1234-5678",
      "priority": ["high", "critical"]
    },
    {
      "type": "email",
      "contact": "farmer@example.com",
      "priority": ["low", "medium", "high", "critical"]
    },
    {
      "type": "app_push",
      "device_id": "device-xyz789",
      "priority": ["medium", "high", "critical"]
    }
  ]
}
```

Response (201 Created):
```json
{
  "subscription_id": "sub-f4e7d8c2",
  "status": "active",
  "created_at": "2025-06-15T15:00:00Z",
  "estimated_alerts_per_month": 8,
  "coverage": {
    "regions": 2,
    "monitored_area_ha": 125000,
    "active_monitoring_stations": 47
  }
}
```

#### 2.3.6 Historical Data Query

**GET /detections/history**

Query historical pest detection records.

Query Parameters:
- `field_id`: string (optional)
- `pest_species`: string (optional)
- `start_date`: ISO 8601 date (required)
- `end_date`: ISO 8601 date (required)
- `severity_min`: enum (optional)
- `limit`: integer (default: 50, max: 200)
- `offset`: integer (default: 0)

Response (200 OK):
```json
{
  "total_records": 237,
  "returned": 50,
  "offset": 0,
  "detections": [
    {
      "detection_id": "det-a1b2c3",
      "timestamp": "2025-05-20T10:15:00Z",
      "pest_species": "Nilaparvata_lugens",
      "severity": "medium",
      "location": {
        "field_id": "field-12345",
        "region": "Jeonbuk"
      },
      "population_density": 18,
      "treatment_applied": true,
      "outcome": "controlled"
    }
  ],
  "summary_statistics": {
    "most_common_pest": "Nilaparvata_lugens",
    "average_severity": "medium",
    "treatment_success_rate": 0.87,
    "total_affected_area_ha": 1250
  }
}
```

### 2.4 Extension Service Integration

#### 2.4.1 RDA Pest Monitoring System Integration

**POST /integrate/rda**

Submit data to Korean Rural Development Administration (RDA) pest monitoring system (농촌진흥청 병해충예찰정보시스템).

Request:
```json
{
  "detection_id": "det-7f8a9b0c",
  "rda_region_code": "JB-01",
  "reporting_organization": "Jeonbuk Agricultural Extension",
  "detection_summary": {
    "pest_korean_name": "벼멸구",
    "발생면적_ha": 25.5,
    "발생정도": "다발생",
    "예찰일자": "2025-06-15"
  }
}
```

Response (200 OK):
```json
{
  "rda_submission_id": "RDA-2025-0615-001",
  "status": "submitted",
  "rda_confirmation": "접수완료",
  "follow_up_actions": [
    "지역 방제 권고 발령",
    "인근 지역 예찰 강화"
  ]
}
```

### 2.5 Real-time Data Streaming

#### 2.5.1 WebSocket Connection

```
wss://stream.wia-pest-detection.org/v1/live
```

Subscribe to real-time pest detection events:

```json
{
  "action": "subscribe",
  "channels": ["detections", "alerts"],
  "filters": {
    "region": "Jeonbuk",
    "severity_min": "high"
  }
}
```

Incoming messages:
```json
{
  "event_type": "new_detection",
  "timestamp": "2025-06-15T15:30:45Z",
  "data": {
    "detection_id": "det-x9y8z7",
    "pest_species": "Nilaparvata_lugens",
    "severity": "critical",
    "location": {"latitude": 35.8219, "longitude": 127.1489}
  }
}
```

### 2.6 Error Handling

#### 2.6.1 Error Response Format

```json
{
  "error": {
    "code": "PEST_NOT_FOUND",
    "message": "The specified pest species was not found in the database",
    "details": "Species ID 'unknown_pest' is not recognized",
    "timestamp": "2025-06-15T15:35:00Z",
    "request_id": "req-abc123"
  }
}
```

#### 2.6.2 Common Error Codes

- `AUTH_REQUIRED` (401): Missing or invalid authentication
- `FORBIDDEN` (403): Insufficient permissions
- `NOT_FOUND` (404): Resource not found
- `VALIDATION_ERROR` (400): Invalid request data
- `RATE_LIMIT_EXCEEDED` (429): Too many requests
- `IMAGE_TOO_LARGE` (413): Image exceeds size limit
- `PROCESSING_ERROR` (500): Server-side processing error
- `AI_MODEL_UNAVAILABLE` (503): AI detection service unavailable

### 2.7 Rate Limiting

- Free tier: 100 requests/hour
- Basic tier: 1,000 requests/hour
- Professional tier: 10,000 requests/hour
- Enterprise tier: Unlimited (contact sales)

Headers returned:
```
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 847
X-RateLimit-Reset: 1623852000
```

### 2.8 API Versioning

- Current version: v1
- Version included in URL path
- Deprecated versions supported for 12 months
- Breaking changes require new major version

### 2.9 SDK Availability

Official SDKs:
- Python: `pip install wia-pest-detection`
- JavaScript/Node: `npm install @wia/pest-detection`
- Java: Maven/Gradle artifact
- Swift (iOS): CocoaPods/SPM
- Kotlin (Android): Maven

### 2.10 Testing Environment

Sandbox API available at:
```
https://sandbox-api.wia-pest-detection.org/v1
```

Features:
- Test data generation
- No rate limits
- Mock AI responses
- No real alerts sent
- Data reset daily
