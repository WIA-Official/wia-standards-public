# WIA-IND-004 Phase 2: API Interface Standard
## Beauty Technology API Specifications

**Version:** 1.0.0
**Status:** Active
**Philosophy:** 弘益人間 (Benefit All Humanity)

---

## Overview

Phase 2 defines RESTful and GraphQL API interfaces for beauty technology systems. These APIs enable programmatic access to skin analysis, recommendations, product catalogs, and device synchronization.

## Core API Principles

1. **RESTful Design**: Resource-oriented URLs, standard HTTP methods
2. **Stateless**: Each request contains all necessary information
3. **Versioned**: API version in URL path (e.g., `/v1/`)
4. **Authenticated**: OAuth 2.0 for all protected endpoints
5. **Rate Limited**: Prevent abuse and ensure fair usage
6. **JSON**: All requests and responses in JSON format
7. **HTTPS Only**: TLS 1.3 minimum

---

## Authentication

### OAuth 2.0 Flow

All protected endpoints require OAuth 2.0 Bearer tokens:

```http
Authorization: Bearer <access_token>
```

### Token Endpoint

```
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials
&client_id=<your_client_id>
&client_secret=<your_client_secret>
```

**Response:**
```json
{
  "access_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "read write"
}
```

---

## 1. Skin Analysis API

### Analyze Skin (Image Upload)

**Endpoint:** `POST /v1/skin-analysis`

**Description:** Upload image for AI-powered skin analysis

**Headers:**
```
Authorization: Bearer <token>
Content-Type: multipart/form-data
```

**Request Body:**
```
image: <file> (JPEG/PNG, max 10MB)
userId: <string> (optional)
language: <string> (ISO 639-1, default: en)
```

**Response:** HTTP 200
```json
{
  "standard": "WIA-IND-004",
  "version": "1.0.0",
  "type": "SkinAnalysisResult",
  "timestamp": "2025-12-27T10:30:00Z",
  "data": {
    "analysisId": "550e8400-e29b-41d4-a716-446655440000",
    "userId": "660e8400-e29b-41d4-a716-446655440001",
    "metrics": { /* See Phase 1 schema */ },
    "overallScore": 72,
    "confidence": 0.94,
    "recommendations": [/* ... */]
  },
  "philosophy": "弘益人間"
}
```

**Error Responses:**
- 400: Invalid image format
- 401: Unauthorized (invalid/expired token)
- 413: Image too large
- 422: Unable to detect face in image
- 429: Rate limit exceeded

---

### Get Analysis History

**Endpoint:** `GET /v1/skin-analysis/history`

**Parameters:**
- `userId` (required): User ID
- `limit` (optional, default 10): Number of results
- `offset` (optional, default 0): Pagination offset
- `startDate` (optional): ISO 8601 date
- `endDate` (optional): ISO 8601 date

**Response:** HTTP 200
```json
{
  "total": 25,
  "limit": 10,
  "offset": 0,
  "results": [
    { /* SkinAnalysisResult */ },
    { /* SkinAnalysisResult */ }
  ]
}
```

---

## 2. Product Catalog API

### Search Products

**Endpoint:** `GET /v1/products/search`

**Parameters:**
- `q` (optional): Search query
- `category` (optional): Product category
- `skinType` (optional): Filter by suitable skin type
- `concern` (optional): Filter by concern addressed
- `minPrice` (optional): Minimum price
- `maxPrice` (optional): Maximum price
- `vegan` (optional): Boolean filter
- `crueltyFree` (optional): Boolean filter
- `limit` (optional, default 20)
- `offset` (optional, default 0)
- `sort` (optional): `price-asc|price-desc|rating|popular`

**Response:** HTTP 200
```json
{
  "total": 150,
  "limit": 20,
  "offset": 0,
  "products": [
    { /* BeautyProduct schema */ }
  ]
}
```

---

### Get Product Details

**Endpoint:** `GET /v1/products/{productId}`

**Response:** HTTP 200
```json
{
  "standard": "WIA-IND-004",
  "version": "1.0.0",
  "type": "BeautyProduct",
  /* Full product data */
}
```

---

### Get Product Ingredients

**Endpoint:** `GET /v1/products/{productId}/ingredients`

**Response:** HTTP 200
```json
{
  "productId": "BT-001",
  "ingredients": [
    {
      "inciName": "Hyaluronic Acid",
      "concentration": 2.0,
      "purpose": "humectant",
      "benefits": ["hydration", "plumping"],
      "concerns": ["dry-skin", "dehydration"],
      "allergen": false
    }
  ]
}
```

---

## 3. Recommendation API

### Get Personalized Recommendations

**Endpoint:** `POST /v1/recommendations`

**Request Body:**
```json
{
  "userId": "string",
  "context": {
    "trigger": "skin-analysis|browse|cart",
    "budget": 100.00,
    "urgency": "immediate"
  },
  "preferences": {
    "productTypes": ["serum", "moisturizer"],
    "values": ["vegan", "cruelty-free"]
  }
}
```

**Response:** HTTP 200
```json
{
  "standard": "WIA-IND-004",
  "type": "ProductRecommendation",
  "data": {
    "recommendationId": "rec-123",
    "recommendations": [/* ... */],
    "routineSuggestion": {/* ... */}
  }
}
```

---

## 4. Device Sync API

### Register Device

**Endpoint:** `POST /v1/devices/register`

**Request Body:**
```json
{
  "deviceType": "cleansing-brush|led-mask|skin-analyzer|...",
  "serialNumber": "string",
  "firmwareVersion": "string",
  "userId": "string"
}
```

**Response:** HTTP 201
```json
{
  "deviceId": "dev-550e8400",
  "registeredAt": "2025-12-27T10:30:00Z",
  "status": "active"
}
```

---

### Sync Device Data

**Endpoint:** `POST /v1/devices/{deviceId}/sync`

**Request Body:**
```json
{
  "timestamp": "2025-12-27T10:30:00Z",
  "usageRecords": [
    {
      "startTime": "2025-12-27T07:00:00Z",
      "duration": 60,
      "mode": "sensitive",
      "settings": {
        "speed": 75,
        "intensity": 80
      }
    }
  ],
  "measurements": {
    "batteryLevel": 85,
    "usageCount": 42,
    "lastCleaned": "2025-12-26T18:00:00Z"
  }
}
```

**Response:** HTTP 200
```json
{
  "synced": true,
  "recordsProcessed": 1,
  "nextSync": "2025-12-28T10:30:00Z"
}
```

---

## 5. User Profile API

### Get User Profile

**Endpoint:** `GET /v1/users/{userId}/profile`

**Response:** HTTP 200
```json
{
  "standard": "WIA-IND-004",
  "type": "UserProfile",
  "data": { /* Full profile */ }
}
```

---

### Update User Profile

**Endpoint:** `PATCH /v1/users/{userId}/profile`

**Request Body:** (partial update)
```json
{
  "skinProfile": {
    "concerns": [
      {"type": "acne", "severity": "moderate", "priority": 1}
    ]
  },
  "preferences": {
    "budgetRange": "premium"
  }
}
```

**Response:** HTTP 200
```json
{
  "updated": true,
  "profile": { /* Updated profile */ }
}
```

---

## 6. Treatment Tracking API

### Create Treatment

**Endpoint:** `POST /v1/treatments`

**Request Body:**
```json
{
  "userId": "string",
  "name": "8-Week Hydration Program",
  "startDate": "2025-12-27",
  "targetConcerns": ["dehydration", "fine-lines"],
  "products": [
    {"productId": "BT-001", "usage": "twice-daily"}
  ]
}
```

**Response:** HTTP 201
```json
{
  "treatmentId": "trt-550e8400",
  "status": "active",
  "createdAt": "2025-12-27T10:30:00Z"
}
```

---

### Log Treatment Progress

**Endpoint:** `POST /v1/treatments/{treatmentId}/checkpoints`

**Request Body:**
```json
{
  "date": "2026-01-10",
  "analysisId": "analysis-abc123",
  "notes": "Noticeable improvement in hydration"
}
```

---

## Rate Limiting

All APIs implement rate limiting:

**Headers:**
```
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 999
X-RateLimit-Reset: 1640608800
```

**Limits:**
- Free tier: 100 requests/hour
- Basic tier: 1,000 requests/hour
- Premium tier: 10,000 requests/hour
- Enterprise: Custom limits

**429 Response:**
```json
{
  "error": "rate_limit_exceeded",
  "message": "Rate limit exceeded. Try again in 3600 seconds.",
  "retryAfter": 3600
}
```

---

## Error Handling

### Standard Error Response

```json
{
  "error": {
    "code": "string",
    "message": "string",
    "details": "string (optional)",
    "timestamp": "ISO 8601"
  }
}
```

### HTTP Status Codes

- 200: Success
- 201: Created
- 400: Bad Request
- 401: Unauthorized
- 403: Forbidden
- 404: Not Found
- 422: Unprocessable Entity
- 429: Too Many Requests
- 500: Internal Server Error
- 503: Service Unavailable

---

## Webhooks

### Register Webhook

**Endpoint:** `POST /v1/webhooks`

**Request Body:**
```json
{
  "url": "https://your-app.com/webhook",
  "events": ["skin.analysis.completed", "product.updated", "device.synced"],
  "secret": "webhook_secret_key"
}
```

### Webhook Payload

```json
{
  "event": "skin.analysis.completed",
  "timestamp": "2025-12-27T10:30:00Z",
  "data": { /* Event-specific data */ },
  "signature": "sha256=..." 
}
```

---

## GraphQL API

### Endpoint

`POST /graphql`

### Example Query

```graphql
query GetUserWithAnalysis($userId: ID!) {
  user(id: $userId) {
    profile {
      skinProfile {
        type
        concerns {
          type
          severity
        }
      }
    }
    skinAnalyses(limit: 5) {
      analysisId
      timestamp
      overallScore
      metrics {
        wrinkles {
          score
        }
      }
    }
    recommendations {
      products {
        name
        matchScore
      }
    }
  }
}
```

---

## SDK Examples

### JavaScript/TypeScript

```javascript
import { WIABeautyClient } from '@wia/beauty-sdk';

const client = new WIABeautyClient({
  apiKey: 'your_api_key',
  version: 'v1'
});

// Analyze skin
const analysis = await client.skinAnalysis.create({
  image: imageFile,
  userId: 'user-123'
});

// Get recommendations
const recs = await client.recommendations.get({
  userId: 'user-123',
  context: { budget: 100 }
});
```

### Python

```python
from wia_beauty import WIABeautyClient

client = WIABeautyClient(api_key='your_api_key')

# Analyze skin
analysis = client.skin_analysis.create(
    image=image_file,
    user_id='user-123'
)

# Get recommendations
recs = client.recommendations.get(
    user_id='user-123',
    context={'budget': 100}
)
```

---

## Compliance

Phase 2 compliant implementations must:
1. Implement all core endpoints
2. Use OAuth 2.0 authentication
3. Support rate limiting
4. Return Phase 1 compliant data schemas
5. Use HTTPS/TLS 1.3
6. Implement proper error handling
7. Pass WIA API certification tests

---

**Maintained by:** WIA (World Certification Industry Association)
**Last Updated:** 2025-12-27

弘益人間 · Benefit All Humanity
