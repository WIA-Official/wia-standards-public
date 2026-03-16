# WIA-PLASTIC-ENZYME Phase 2: API Interface Specification

> **Version:** 1.0.0
> **Status:** Official
> **Last Updated:** 2025-01-01
> **Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

Phase 2 of the WIA-PLASTIC-ENZYME standard defines the API interfaces that enable software systems to interact with enzymatic plastic degradation services. The architecture follows REST principles with JSON payloads.

### 1.1 Design Principles

- **RESTful:** Resource-oriented URLs with standard HTTP methods
- **JSON-first:** All requests and responses in JSON format
- **Versioned:** API versioning in URL path (/v1/, /v2/)
- **Authenticated:** API key or OAuth 2.0 authentication
- **Rate-limited:** Configurable rate limits per client

### 1.2 Base URL

```
Production: https://api.wia.live/plastic-enzyme/v1
Sandbox:    https://sandbox.wia.live/plastic-enzyme/v1
```

---

## 2. Authentication

### 2.1 API Key Authentication

```http
GET /api/v1/enzyme/library
Authorization: Bearer wia_key_xxxxxxxxxxxxx
```

### 2.2 OAuth 2.0

```http
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials
&client_id=your_client_id
&client_secret=your_client_secret
&scope=enzyme:read process:write
```

### 2.3 Available Scopes

| Scope | Description |
|-------|-------------|
| `enzyme:read` | Read enzyme library and profiles |
| `enzyme:write` | Create and update enzyme profiles |
| `process:read` | Read degradation process data |
| `process:write` | Create and update process records |
| `quality:read` | Access quality metrics |
| `quality:certify` | Issue quality certifications |

---

## 3. API Endpoints

### 3.1 Plastic Identification

```yaml
POST /api/v1/plastic/identify
```

**Description:** Identify plastic type from spectroscopic or image data

**Request Body:**
```json
{
  "method": "ftir_spectrum",
  "data": {
    "wavenumbers": [4000, 3999, 3998],
    "absorbance": [0.02, 0.021, 0.019]
  }
}
```

**Response:**
```json
{
  "plastic_type": "PET",
  "confidence": 0.98,
  "crystallinity_estimate": 28.5,
  "contaminants_detected": ["PP traces"],
  "recommended_pretreatment": "mechanical_grinding"
}
```

### 3.2 Enzyme Matching

```yaml
GET /api/v1/enzyme/match/{plastic_type}
```

**Parameters:**
| Name | Type | Description |
|------|------|-------------|
| plastic_type | path | PET, PBAT, PLA, PCL, PHA |
| temperature | query | Operating temperature (°C) |
| ph | query | Operating pH |
| target_efficiency | query | Target degradation % |

**Response:**
```json
{
  "recommendations": [
    {
      "enzyme_id": "urn:wia:enzyme:turbopetase-v2",
      "name": "TurboPETase-v2",
      "match_score": 98.5,
      "predicted_efficiency": 97.2,
      "optimal_concentration_mg_g": 2.5,
      "estimated_time_hours": 48
    }
  ],
  "cocktail_suggestion": {
    "primary": "urn:wia:enzyme:turbopetase-v2",
    "secondary": "urn:wia:enzyme:mhetase-std",
    "ratio": "2:1",
    "synergy_bonus": "+12%"
  }
}
```

### 3.3 Enzyme Library

```yaml
GET /api/v1/enzyme/library
```

**Query Parameters:**
| Name | Type | Description |
|------|------|-------------|
| classification | string | Filter by enzyme type |
| min_temperature | number | Minimum optimal temperature |
| max_temperature | number | Maximum optimal temperature |
| engineered | boolean | Filter engineered enzymes |
| page | integer | Page number |
| per_page | integer | Results per page (max 100) |

**Response:**
```json
{
  "total": 47,
  "page": 1,
  "per_page": 20,
  "enzymes": [
    {
      "enzyme_id": "urn:wia:enzyme:turbopetase-v2",
      "name": "TurboPETase-v2",
      "classification": "PETase",
      "source_organism": "Engineered E. coli",
      "optimal_temperature": 55,
      "kcat_km": 68700,
      "reference": "doi:10.1038/s41467-024-45662-9"
    }
  ]
}
```

### 3.4 Degradation Optimization

```yaml
POST /api/v1/degradation/optimize
```

**Request:**
```json
{
  "plastic": {
    "type": "PET",
    "weight_kg": 1000,
    "crystallinity_percent": 30,
    "form": "flakes"
  },
  "constraints": {
    "max_temperature_c": 60,
    "max_time_hours": 72,
    "target_efficiency": 95
  },
  "optimization_goal": "cost"
}
```

**Response:**
```json
{
  "optimal_conditions": {
    "temperature_c": 55,
    "ph": 8.0,
    "enzyme_loading_mg_g": 2.5,
    "substrate_loading_percent": 12
  },
  "enzyme_cocktail": [
    {"enzyme_id": "urn:wia:enzyme:turbopetase-v2", "ratio": 0.7},
    {"enzyme_id": "urn:wia:enzyme:mhetase-std", "ratio": 0.3}
  ],
  "predicted_outcomes": {
    "efficiency_percent": 97.5,
    "time_hours": 48,
    "tpa_yield_kg": 820,
    "cost_per_kg": 0.45
  }
}
```

### 3.5 Degradation Prediction

```yaml
POST /api/v1/degradation/predict
```

**Request:**
```json
{
  "plastic": {"type": "PET", "weight_kg": 100, "crystallinity_percent": 25},
  "enzyme": {"enzyme_id": "urn:wia:enzyme:turbopetase-v2", "concentration_mg_g": 2.0},
  "conditions": {"temperature_c": 55, "ph": 8.0, "duration_hours": 48}
}
```

**Response:**
```json
{
  "predictions": {
    "degradation_percent": {"mean": 95.5, "std": 2.1, "ci_95": [91.3, 99.7]},
    "tpa_yield_kg": {"mean": 81.2, "std": 1.8},
    "eg_yield_kg": {"mean": 27.8, "std": 0.6}
  },
  "time_course": [
    {"hour": 0, "degradation": 0},
    {"hour": 12, "degradation": 45.2},
    {"hour": 24, "degradation": 78.5},
    {"hour": 48, "degradation": 95.5}
  ]
}
```

---

## 4. CRUD Operations

### 4.1 Enzyme Profiles

| Method | Endpoint | Description |
|--------|----------|-------------|
| GET | /api/v1/enzymes | List all enzymes |
| GET | /api/v1/enzymes/{id} | Get enzyme by ID |
| POST | /api/v1/enzymes | Create enzyme profile |
| PUT | /api/v1/enzymes/{id} | Update enzyme profile |
| DELETE | /api/v1/enzymes/{id} | Delete enzyme profile |

### 4.2 Processes

| Method | Endpoint | Description |
|--------|----------|-------------|
| GET | /api/v1/processes | List all processes |
| GET | /api/v1/processes/{id} | Get process by ID |
| POST | /api/v1/processes | Create process record |
| PUT | /api/v1/processes/{id} | Update process record |
| POST | /api/v1/processes/{id}/output | Record process output |

---

## 5. Webhooks

### 5.1 Webhook Registration

```http
POST /api/v1/webhooks
Content-Type: application/json

{
  "url": "https://your-system.com/wia-events",
  "events": ["process.completed", "quality.certified"],
  "secret": "your-webhook-secret"
}
```

### 5.2 Webhook Events

| Event | Description |
|-------|-------------|
| `process.started` | Degradation process initiated |
| `process.completed` | Degradation process finished |
| `quality.tested` | Quality testing completed |
| `quality.certified` | WIA certification issued |

### 5.3 Webhook Payload

```json
{
  "event": "process.completed",
  "timestamp": "2025-01-15T10:30:00Z",
  "data": {
    "process_id": "urn:wia:process:2025-001-abc",
    "status": "completed",
    "efficiency": 97.5
  },
  "signature": "sha256=xxxxxxxx"
}
```

---

## 6. Rate Limits

| Tier | Rate Limit | Monthly Quota |
|------|------------|---------------|
| Free | 60 requests/minute | 10,000 requests |
| Professional | 600 requests/minute | 500,000 requests |
| Enterprise | Unlimited | Unlimited |

**Rate Limit Headers:**
```http
X-RateLimit-Limit: 60
X-RateLimit-Remaining: 45
X-RateLimit-Reset: 1704067200
```

---

## 7. Error Handling

### 7.1 Error Response Format

```json
{
  "error": {
    "code": "ENZYME_NOT_FOUND",
    "message": "The specified enzyme ID does not exist",
    "details": {
      "enzyme_id": "urn:wia:enzyme:invalid-id"
    },
    "request_id": "req_abc123",
    "documentation_url": "https://docs.wia.live/errors/ENZYME_NOT_FOUND"
  }
}
```

### 7.2 Error Codes

| Code | HTTP Status | Description |
|------|-------------|-------------|
| INVALID_REQUEST | 400 | Malformed request body |
| UNAUTHORIZED | 401 | Missing or invalid authentication |
| FORBIDDEN | 403 | Insufficient permissions |
| NOT_FOUND | 404 | Resource not found |
| RATE_LIMITED | 429 | Rate limit exceeded |
| INTERNAL_ERROR | 500 | Server error |

---

## 8. SDK Support

### 8.1 Official SDKs

| Language | Package | Installation |
|----------|---------|--------------|
| TypeScript | @wia/plastic-enzyme-sdk | npm install @wia/plastic-enzyme-sdk |
| Python | wia-plastic-enzyme | pip install wia-plastic-enzyme |
| Rust | wia-plastic-enzyme | cargo add wia-plastic-enzyme |
| Go | wia-plastic-enzyme | go get github.com/wia/plastic-enzyme-go |

---

## 9. Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-01-01 | Initial release |

---

**弘益人間 (Benefit All Humanity)**

© 2025 WIA - World Certification Industry Association
