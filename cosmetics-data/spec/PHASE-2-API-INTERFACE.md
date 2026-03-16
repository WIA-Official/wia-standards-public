# WIA-IND-005: Phase 2 - API Interface Specification

**Version:** 1.0
**Status:** Active
**Philosophy:** 弘益人間 - Benefit All Humanity

## Overview

Phase 2 defines RESTful API interfaces for accessing cosmetic ingredient databases, conducting safety analyses, checking regulatory compliance, and matching products to consumer needs. This specification enables programmatic access to WIA-IND-005 data and services.

## Base API Architecture

### Endpoint Structure

```
https://api.wia-standards.org/v1/cosmetics/{resource}
```

### Authentication

#### API Key Authentication

```http
Authorization: Bearer {api_key}
X-WIA-Client-ID: {client_id}
```

#### OAuth 2.0 (Optional for consumer applications)

```http
Authorization: Bearer {oauth_token}
```

### Rate Limiting

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 999
X-RateLimit-Reset: 1640000000
```

**Limits:**
- **Free Tier:** 100 requests/hour
- **Basic Tier:** 1,000 requests/hour
- **Professional Tier:** 10,000 requests/hour
- **Enterprise Tier:** Unlimited (custom SLA)

**Philosophy Note:** Basic ingredient safety lookups are unlimited for consumer protection apps, embodying 弘益人間.

## Core API Endpoints

### 1. Ingredient Database API

#### Get Ingredient by INCI Name

```http
GET /v1/cosmetics/ingredients/search?inci_name={name}
```

**Example Request:**
```bash
curl -X GET "https://api.wia-standards.org/v1/cosmetics/ingredients/search?inci_name=Glycerin" \
  -H "Authorization: Bearer YOUR_API_KEY"
```

**Example Response:**
```json
{
  "status": "success",
  "data": {
    "inci_name": "Glycerin",
    "cas_number": "56-81-5",
    "safety": {
      "ewg_score": 1,
      "ewg_rating": "Low Hazard"
    },
    "philosophy": "弘益人間"
  },
  "metadata": {
    "request_id": "req_abc123",
    "timestamp": "2025-01-15T10:30:00Z"
  }
}
```

#### Get Ingredient by CAS Number

```http
GET /v1/cosmetics/ingredients/cas/{cas_number}
```

#### Search Ingredients

```http
POST /v1/cosmetics/ingredients/search
Content-Type: application/json

{
  "query": "moisturizer",
  "filters": {
    "function": "Humectant",
    "max_ewg_score": 3,
    "allergen_free": true,
    "vegan": true
  },
  "page": 1,
  "limit": 50
}
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "total": 145,
    "page": 1,
    "limit": 50,
    "results": [
      {
        "inci_name": "Glycerin",
        "function": "Humectant",
        "safety": {...}
      }
    ]
  },
  "pagination": {
    "next": "/v1/cosmetics/ingredients/search?page=2",
    "prev": null
  },
  "philosophy": "弘益人間 - Accessible knowledge for all"
}
```

### 2. Safety Analysis API

#### Analyze Product Safety

```http
POST /v1/cosmetics/safety/analyze
Content-Type: application/json

{
  "product_type": "leave_on_moisturizer",
  "ingredients": [
    {
      "inci": "Aqua",
      "percentage": 65.0
    },
    {
      "inci": "Glycerin",
      "percentage": 12.0
    },
    {
      "inci": "Niacinamide",
      "percentage": 5.0
    }
  ],
  "target_markets": ["US", "EU", "CN"]
}
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "overall_safety_score": 8.7,
    "ingredient_analysis": [
      {
        "inci": "Aqua",
        "safety_score": 10,
        "concerns": []
      },
      {
        "inci": "Glycerin",
        "safety_score": 10,
        "concerns": []
      },
      {
        "inci": "Niacinamide",
        "safety_score": 9,
        "concerns": []
      }
    ],
    "allergen_assessment": {
      "eu_26_allergens": [],
      "risk_level": "Low"
    },
    "recommendations": [
      "All ingredients approved for target markets",
      "No known allergens present",
      "Formulation considered safe for intended use"
    ]
  },
  "philosophy": "弘益人間 - Safety analysis for global benefit"
}
```

#### Check Allergen Compatibility

```http
POST /v1/cosmetics/safety/allergen-check
Content-Type: application/json

{
  "ingredients": ["Limonene", "Linalool", "Aqua"],
  "user_allergens": ["Limonene"],
  "product_type": "leave_on"
}
```

### 3. Regulatory Compliance API

#### Check Regulatory Compliance

```http
POST /v1/cosmetics/compliance/check
Content-Type: application/json

{
  "product": {
    "ingredients": [...],
    "product_type": "Moisturizer",
    "claims": ["Hypoallergenic", "Fragrance-free"]
  },
  "target_markets": ["US", "EU", "JP", "CN", "KR"]
}
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "overall_compliant": true,
    "market_compliance": {
      "US": {
        "compliant": true,
        "warnings": [],
        "required_actions": ["VCRP registration recommended"]
      },
      "EU": {
        "compliant": true,
        "warnings": [],
        "required_actions": ["PIF documentation", "CPNP notification"]
      },
      "CN": {
        "compliant": false,
        "violations": [
          {
            "ingredient": "Phenoxyethanol",
            "issue": "Requires animal testing for imported products",
            "regulation": "NMPA Regulation 2021"
          }
        ],
        "required_actions": ["Product registration", "Safety assessment"]
      }
    }
  },
  "philosophy": "弘益人間 - Simplified compliance for all manufacturers"
}
```

#### Get Regulatory Updates

```http
GET /v1/cosmetics/compliance/updates?market={ISO_CODE}&since={date}
```

### 4. Product Matching API

#### Find Matching Products

```http
POST /v1/cosmetics/products/match
Content-Type: application/json

{
  "user_profile": {
    "skin_type": "Dry",
    "concerns": ["Hydration", "Anti-aging"],
    "allergens": [],
    "preferences": {
      "vegan": true,
      "cruelty_free": true,
      "fragrance_free": true
    }
  },
  "product_category": "Moisturizer"
}
```

**Response:**
```json
{
  "status": "success",
  "data": {
    "matches": [
      {
        "product_id": "PROD-12345",
        "name": "Hydrating Cream",
        "match_score": 95,
        "reasons": [
          "Formulated for dry skin",
          "Contains hyaluronic acid for hydration",
          "Vegan and cruelty-free certified",
          "Fragrance-free formulation"
        ],
        "key_ingredients": ["Hyaluronic Acid", "Ceramides", "Glycerin"]
      }
    ],
    "total_matches": 23
  },
  "philosophy": "弘益人間 - Personalized recommendations for everyone"
}
```

### 5. Batch Operations API

#### Batch Ingredient Lookup

```http
POST /v1/cosmetics/batch/ingredients
Content-Type: application/json

{
  "inci_names": [
    "Aqua",
    "Glycerin",
    "Niacinamide",
    "Sodium Hyaluronate"
  ],
  "fields": ["inci_name", "safety", "regulatory"]
}
```

## Webhooks

### Register Webhook

```http
POST /v1/cosmetics/webhooks
Content-Type: application/json

{
  "url": "https://yourdomain.com/webhooks/cosmetics",
  "events": [
    "ingredient.updated",
    "regulation.changed",
    "safety.alert"
  ],
  "secret": "your_webhook_secret"
}
```

### Webhook Events

#### Ingredient Updated

```json
{
  "event": "ingredient.updated",
  "timestamp": "2025-01-15T10:30:00Z",
  "data": {
    "inci_name": "Methylisothiazolinone",
    "changes": {
      "eu_max_concentration": {
        "old": "0.01%",
        "new": "0.0015%"
      }
    }
  },
  "philosophy": "弘益人間"
}
```

## Error Handling

### Error Response Format

```json
{
  "status": "error",
  "error": {
    "code": "INGREDIENT_NOT_FOUND",
    "message": "Ingredient with INCI name 'InvalidName' not found",
    "details": {
      "inci_name": "InvalidName",
      "suggestions": ["ValidName1", "ValidName2"]
    }
  },
  "metadata": {
    "request_id": "req_xyz789",
    "timestamp": "2025-01-15T10:30:00Z"
  }
}
```

### Common Error Codes

| Code | HTTP Status | Description |
|------|-------------|-------------|
| INVALID_API_KEY | 401 | API key is invalid or expired |
| RATE_LIMIT_EXCEEDED | 429 | Rate limit exceeded |
| INGREDIENT_NOT_FOUND | 404 | Ingredient not found in database |
| INVALID_REQUEST | 400 | Request body validation failed |
| COMPLIANCE_CHECK_FAILED | 422 | Compliance check could not be completed |
| INTERNAL_ERROR | 500 | Internal server error |

## SDK Support

### Official SDKs

#### TypeScript/JavaScript

```typescript
import { WIACosmeticsSDK } from '@wia/cosmetics-ind-005';

const client = new WIACosmeticsSDK({
  apiKey: 'your_api_key',
  version: '1.0'
});

// Search ingredient
const ingredient = await client.ingredients.search('Glycerin');

// Safety analysis
const safety = await client.safety.analyze({
  ingredients: [
    { inci: 'Aqua', percentage: 65.0 },
    { inci: 'Glycerin', percentage: 12.0 }
  ]
});
```

#### Python

```python
from wia_cosmetics import CosmeticsClient

client = CosmeticsClient(api_key='your_api_key')

# Search ingredient
ingredient = client.ingredients.search('Glycerin')

# Compliance check
compliance = client.compliance.check(
    ingredients=[...],
    target_markets=['US', 'EU']
)
```

## Philosophy Statement

**弘益人間 (Benefit All Humanity):** These APIs are designed with accessibility in mind. Basic safety lookups are provided free of charge to consumer protection applications, ensuring that ingredient safety information is available to all, regardless of economic resources. Enterprise features subsidize free access, creating an ecosystem that benefits humanity broadly.

---

**Document Version:** 1.0
**Last Updated:** 2025-01-15
**Status:** Active
**Philosophy:** 弘益人間 - Benefit All Humanity
