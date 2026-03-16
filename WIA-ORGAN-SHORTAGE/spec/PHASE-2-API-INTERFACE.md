# WIA-ORGAN-SHORTAGE - Phase 2: API Interface

> **Version:** 1.0.0
> **Status:** Complete
> **弘益人間 (Hongik Ingan)** - Benefit All Humanity

## 1. Overview

This document defines the RESTful API interface for the WIA-ORGAN-SHORTAGE standard, enabling secure data exchange between transplant systems, research platforms, and organ procurement organizations.

## 2. Base URL

```
Production: https://api.organ-shortage.wia.org/v1
Sandbox:    https://sandbox.organ-shortage.wia.org/v1
```

## 3. Authentication

### 3.1 OAuth 2.0 Bearer Token

```http
Authorization: Bearer <access_token>
```

### 3.2 API Key (Development)

```http
X-WIA-API-Key: <api_key>
```

## 4. API Endpoints

### 4.1 Waitlist Management

#### GET /waitlist/status
Get current waitlist statistics.

**Response:**
```json
{
  "total_waiting": 103245,
  "by_organ": {
    "kidney": 89432,
    "liver": 10234,
    "heart": 3456,
    "lung": 1023,
    "pancreas": 876,
    "intestine": 224
  },
  "daily_deaths": 17,
  "timestamp": "2025-01-05T00:00:00Z"
}
```

#### GET /waitlist/patients
List patients on waitlist (paginated).

**Query Parameters:**
| Parameter | Type | Description |
|-----------|------|-------------|
| organ | string | Filter by organ type |
| urgency | string | Filter by urgency level |
| region | string | Filter by geographic region |
| limit | integer | Results per page (default: 20) |
| offset | integer | Pagination offset |

**Response:**
```json
{
  "data": [
    {
      "id": "patient_123",
      "organ_needed": "kidney",
      "urgency": "high",
      "blood_type": "O",
      "waiting_days": 847,
      "pra_percent": 65
    }
  ],
  "pagination": {
    "total": 89432,
    "limit": 20,
    "offset": 0,
    "has_more": true
  }
}
```

#### POST /waitlist/patients
Register a new patient on the waitlist.

**Request:**
```json
{
  "organ_needed": "kidney",
  "urgency": "high",
  "immunological": {
    "blood_type": "O",
    "hla_typing": {},
    "pra_percent": 65
  },
  "listing_center": "center_id"
}
```

### 4.2 Xenotransplantation

#### POST /xenotransplant/eligibility
Evaluate xenotransplantation eligibility.

**Request:**
```json
{
  "patient_id": "patient_123",
  "organ_needed": "kidney",
  "pra_percent": 65,
  "waiting_years": 3,
  "contraindications": []
}
```

**Response:**
```json
{
  "eligible": true,
  "score": 85,
  "reasons": [
    "High PRA (>50%) - difficult to match with human donor",
    "Extended waitlist time (>2 years)"
  ],
  "available_trials": [
    {
      "id": "NCT06488183",
      "sponsor": "eGenesis",
      "organ": "kidney",
      "phase": "Phase 1",
      "status": "Recruiting"
    }
  ],
  "recommendations": [
    "Consider enrollment in eGenesis kidney trial",
    "Continue on standard waitlist"
  ]
}
```

#### GET /xenotransplant/trials
List available xenotransplantation clinical trials.

**Response:**
```json
{
  "trials": [
    {
      "id": "NCT06488183",
      "title": "Gene-Edited Pig Kidney Transplantation",
      "sponsor": "eGenesis",
      "organ": "kidney",
      "phase": "Phase 1",
      "status": "Recruiting",
      "locations": ["Massachusetts General Hospital", "NYU Langone"],
      "genetic_modifications": 69,
      "enrollment_criteria": {
        "age_min": 18,
        "age_max": 70,
        "pra_min": 50
      }
    }
  ]
}
```

### 4.3 Bioprinting

#### POST /bioprint/order
Submit a bioprinted organ order (research).

**Request:**
```json
{
  "patient_id": "patient_123",
  "organ_type": "kidney",
  "cell_source": "autologous",
  "priority": "standard",
  "specifications": {
    "size_ml": 150,
    "vascularization_required": true
  }
}
```

**Response:**
```json
{
  "order_id": "bioprint_456",
  "status": "queued",
  "estimated_production_days": 90,
  "estimated_maturation_days": 30,
  "facility": "Organovo Research Center",
  "tracking_url": "https://track.wia.org/bioprint_456"
}
```

#### GET /bioprint/status/{order_id}
Check bioprinting order status.

**Response:**
```json
{
  "order_id": "bioprint_456",
  "status": "printing",
  "progress_percent": 45,
  "current_phase": "scaffold_construction",
  "quality_metrics": {
    "cell_viability": 0.94,
    "structural_integrity": 0.87,
    "vascular_network": "in_progress"
  },
  "estimated_completion": "2025-04-15T00:00:00Z"
}
```

### 4.4 Matching Algorithm

#### POST /match
Run optimal matching algorithm.

**Request:**
```json
{
  "patient_id": "patient_123",
  "include_traditional": true,
  "include_xenotransplant": true,
  "include_bioprinted": false,
  "max_results": 10
}
```

**Response:**
```json
{
  "matches": [
    {
      "type": "deceased_donor",
      "donor_id": "donor_789",
      "match_score": 92,
      "blood_compatible": true,
      "hla_matches": 5,
      "crossmatch": "negative",
      "distance_km": 150,
      "cold_ischemia_hours": 4
    },
    {
      "type": "xenotransplant",
      "source": "eGenesis",
      "match_score": 78,
      "trial_id": "NCT06488183",
      "availability": "immediate"
    }
  ],
  "algorithm_version": "2.1.0",
  "computed_at": "2025-01-05T10:30:00Z"
}
```

### 4.5 Registry Integration

#### GET /registry/unos/sync
Sync with UNOS/OPTN registry.

#### GET /registry/eurotransplant/sync
Sync with Eurotransplant registry.

## 5. Error Codes

| Code | HTTP Status | Description |
|------|-------------|-------------|
| INVALID_PATIENT | 400 | Invalid patient identifier |
| BLOOD_INCOMPATIBLE | 400 | Blood type incompatibility |
| AUTH_REQUIRED | 401 | Authentication required |
| ACCESS_DENIED | 403 | Insufficient permissions |
| PATIENT_NOT_FOUND | 404 | Patient not found |
| TRIAL_FULL | 409 | Clinical trial enrollment full |
| RATE_LIMITED | 429 | Too many requests |
| SERVER_ERROR | 500 | Internal server error |

## 6. Rate Limiting

| Tier | Requests/Hour | Burst |
|------|---------------|-------|
| Free | 100 | 10 |
| Standard | 1,000 | 50 |
| Enterprise | 10,000 | 200 |

## 7. Webhooks

### 7.1 Event Types

| Event | Description |
|-------|-------------|
| `organ.available` | New organ available for matching |
| `match.found` | Potential match identified |
| `trial.opening` | New clinical trial recruiting |
| `bioprint.complete` | Bioprinted organ ready |

### 7.2 Webhook Payload

```json
{
  "id": "evt_123",
  "type": "organ.available",
  "timestamp": "2025-01-05T10:30:00Z",
  "data": {
    "organ_type": "kidney",
    "donor_id": "donor_789",
    "region": "Region 5",
    "expires_at": "2025-01-05T14:30:00Z"
  },
  "signature": "sha256=..."
}
```

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 MIT License*
