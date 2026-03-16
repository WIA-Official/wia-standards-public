# WIA-IND-001: Phase 2 - API Interface Specification
**Version:** 1.0
**Status:** Official
**Philosophy:** 弘益人間 (Benefit All Humanity)

## 1. Overview

This document specifies RESTful API endpoints and GraphQL interfaces for the WIA-IND-001 Fashion Technology Standard. All implementations MUST provide these interfaces to enable interoperability between measurement systems, recommendation engines, virtual fitting applications, and e-commerce platforms.

## 2. API Architecture

### 2.1 Base URL Structure
```
https://api.{provider}.com/wia/ind-001/v1/
```

### 2.2 Authentication
- **Method:** OAuth 2.0 (Authorization Code Grant)
- **Token Type:** Bearer JWT
- **Scopes:** `measurement:read`, `measurement:write`, `recommendation:read`, `fitting:read`, `profile:read`, `profile:write`

### 2.3 Request/Response Format
- **Content-Type:** `application/json`
- **Character Encoding:** UTF-8
- **Date Format:** ISO 8601
- **Error Format:** RFC 7807 (Problem Details)

### 2.4 Rate Limiting
- **Default:** 1000 requests/hour per API key
- **Headers:** `X-RateLimit-Limit`, `X-RateLimit-Remaining`, `X-RateLimit-Reset`
- **429 Response:** Include `Retry-After` header

## 3. Measurement API

### 3.1 Create Measurement
```http
POST /measurements
Content-Type: application/json
Authorization: Bearer {token}

{
  "subject": {
    "id": "user-12345",
    "birthDate": "1990-05-15",
    "gender": "female"
  },
  "measurements": {
    "height": { "value": 165.5, "unit": "cm", "accuracy": 0.5, "method": "smartphone-scan" },
    "weight": { "value": 58.2, "unit": "kg", "accuracy": 0.5, "method": "digital-scale" },
    "chest": { "value": 88.0, "unit": "cm", "accuracy": 0.5, "method": "smartphone-scan" },
    "waist": { "value": 68.5, "unit": "cm", "accuracy": 0.5, "method": "smartphone-scan" },
    "hip": { "value": 94.0, "unit": "cm", "accuracy": 0.5, "method": "smartphone-scan" }
  }
}

Response 201 Created:
{
  "id": "meas-abc123",
  "timestamp": "2025-01-15T10:30:00Z",
  "url": "/measurements/meas-abc123",
  "philosophy": "弘益人間"
}
```

### 3.2 Get Measurement
```http
GET /measurements/{id}
Authorization: Bearer {token}

Response 200 OK:
{
  "@context": "https://wia.org/standards/IND-001/v1",
  "@type": "BodyMeasurement",
  "id": "meas-abc123",
  "...": "complete measurement object"
}
```

### 3.3 Update Measurement
```http
PUT /measurements/{id}
PATCH /measurements/{id}  // Partial update
Authorization: Bearer {token}

PATCH Example:
{
  "measurements": {
    "weight": { "value": 58.5, "unit": "kg", "accuracy": 0.5, "method": "digital-scale" }
  }
}

Response 200 OK:
{
  "id": "meas-abc123",
  "updated": "2025-01-16T14:20:00Z"
}
```

### 3.4 Delete Measurement
```http
DELETE /measurements/{id}
Authorization: Bearer {token}

Response 204 No Content
```

### 3.5 List User Measurements
```http
GET /users/{userId}/measurements?limit=10&offset=0&sort=-timestamp
Authorization: Bearer {token}

Response 200 OK:
{
  "measurements": [ array of measurement objects ],
  "total": 25,
  "limit": 10,
  "offset": 0,
  "philosophy": "弘益人間"
}
```

## 4. Size Recommendation API

### 4.1 Get Size Recommendation
```http
POST /recommendations/size
Authorization: Bearer {token}

{
  "userId": "user-12345",
  "productId": "prod-67890",
  "measurementId": "meas-abc123",  // Optional if userId provided
  "preferredFit": "regular"  // Optional
}

Response 200 OK:
{
  "productId": "prod-67890",
  "recommendation": {
    "primarySize": {
      "label": "M",
      "confidence": 0.92,
      "fit": "perfect",
      "reasoning": "Based on chest 88cm, waist 68.5cm, hip 94cm"
    },
    "alternatives": [
      {
        "label": "L",
        "confidence": 0.65,
        "fit": "slightly-loose",
        "reasoning": "Consider if you prefer looser fit"
      },
      {
        "label": "S",
        "confidence": 0.28,
        "fit": "tight",
        "reasoning": "May fit if fabric is very stretchy"
      }
    ],
    "fitAnalysis": {
      "overall": "good",
      "chest": "perfect",
      "waist": "perfect",
      "hip": "good",
      "length": "good"
    },
    "measurements": {
      "recommendedFor": {
        "chest": { "min": 86, "max": 92 },
        "waist": { "min": 66, "max": 72 },
        "hip": { "min": 92, "max": 98 }
      }
    }
  },
  "metadata": {
    "algorithm": "WIA-IND-001-ML-v1.2",
    "timestamp": "2025-01-15T10:35:00Z",
    "philosophy": "弘益人間"
  }
}
```

### 4.2 Batch Size Recommendations
```http
POST /recommendations/size/batch
Authorization: Bearer {token}

{
  "userId": "user-12345",
  "products": ["prod-67890", "prod-67891", "prod-67892"]
}

Response 200 OK:
{
  "recommendations": [
    { "productId": "prod-67890", "recommendation": {...} },
    { "productId": "prod-67891", "recommendation": {...} },
    { "productId": "prod-67892", "recommendation": {...} }
  ]
}
```

### 4.3 Size Chart Lookup
```http
GET /size-charts?brand={brand}&category={category}&gender={gender}
Authorization: Bearer {token}

Response 200 OK:
{
  "@type": "SizeChart",
  "brand": "Example Brand",
  "category": "shirts",
  "gender": "female",
  "chart": [ ... ]
}
```

## 5. Virtual Fitting API

### 5.1 Create Fitting Session
```http
POST /virtual-fitting/sessions
Authorization: Bearer {token}

{
  "userId": "user-12345",
  "productId": "prod-67890",
  "size": "M",
  "avatarProfileId": "avatar-001",  // Optional, create from measurements
  "renderQuality": "high",  // low|medium|high|ultra
  "viewAngles": ["front", "side", "back", "360"]
}

Response 201 Created:
{
  "sessionId": "session-xyz789",
  "status": "initializing",
  "estimatedReadyTime": "2025-01-15T10:36:00Z",
  "expiresAt": "2025-01-15T11:00:00Z"
}
```

### 5.2 Get Fitting Session Status
```http
GET /virtual-fitting/sessions/{sessionId}
Authorization: Bearer {token}

Response 200 OK:
{
  "sessionId": "session-xyz789",
  "status": "ready",
  "renders": {
    "front": "https://cdn.example.com/renders/session-xyz789/front.png",
    "side": "https://cdn.example.com/renders/session-xyz789/side.png",
    "back": "https://cdn.example.com/renders/session-xyz789/back.png",
    "360": "https://cdn.example.com/renders/session-xyz789/360.mp4"
  },
  "interactive": "https://fitting.example.com/view/session-xyz789",
  "fitAnalysis": {
    "overall": "good",
    "shoulders": "perfect",
    "chest": "slightly-tight",
    "waist": "good",
    "length": "perfect"
  },
  "philosophy": "弘益人間"
}
```

### 5.3 Update Fitting Parameters
```http
PATCH /virtual-fitting/sessions/{sessionId}
Authorization: Bearer {token}

{
  "size": "L",  // Change size
  "pose": "walking"  // neutral|walking|arms-raised
}

Response 200 OK:
{
  "sessionId": "session-xyz789",
  "status": "rendering",
  "estimatedReadyTime": "2025-01-15T10:38:00Z"
}
```

## 6. Product API

### 6.1 Get Product Details
```http
GET /products/{productId}
Authorization: Bearer {token}

Response 200 OK:
{
  "@type": "GarmentSpecification",
  "id": "prod-67890",
  "product": { ... },
  "sizing": { ... },
  "materials": { ... },
  "3dModel": { ... },
  "sustainability": { ... }
}
```

### 6.2 Search Products
```http
GET /products?category=shirts&brand=ExampleBrand&gender=female&minPrice=20&maxPrice=100
Authorization: Bearer {token}

Response 200 OK:
{
  "products": [ array of product objects ],
  "total": 156,
  "facets": {
    "categories": { "shirts": 156, "dresses": 89 },
    "brands": { "ExampleBrand": 156 },
    "priceRanges": { ... }
  }
}
```

## 7. User Profile API

### 7.1 Get User Profile
```http
GET /users/{userId}/profile
Authorization: Bearer {token}

Response 200 OK:
{
  "userId": "user-12345",
  "defaultMeasurement": "meas-abc123",
  "fitPreferences": { ... },
  "purchaseHistory": [ ... ],
  "preferredBrands": [ ... ]
}
```

### 7.2 Update Fit Preferences
```http
PATCH /users/{userId}/fit-preferences
Authorization: Bearer {token}

{
  "global": { "fit": "regular" },
  "byCategory": {
    "shirts": { "fit": "slim" },
    "pants": { "fit": "relaxed" }
  }
}

Response 200 OK
```

## 8. Webhook API

### 8.1 Register Webhook
```http
POST /webhooks
Authorization: Bearer {token}

{
  "url": "https://your-app.com/webhooks/wia",
  "events": ["measurement.created", "recommendation.generated", "fitting.ready"],
  "secret": "your-webhook-secret"
}

Response 201 Created:
{
  "webhookId": "webhook-123",
  "url": "https://your-app.com/webhooks/wia",
  "events": [ ... ],
  "active": true
}
```

### 8.2 Webhook Payload Example
```http
POST https://your-app.com/webhooks/wia
X-WIA-Signature: sha256=...
X-WIA-Event: measurement.created

{
  "event": "measurement.created",
  "timestamp": "2025-01-15T10:30:00Z",
  "data": {
    "measurementId": "meas-abc123",
    "userId": "user-12345"
  }
}
```

## 9. GraphQL API

### 9.1 Endpoint
```
POST /graphql
Content-Type: application/json
Authorization: Bearer {token}
```

### 9.2 Schema Excerpt
```graphql
type Query {
  user(id: ID!): User
  measurement(id: ID!): Measurement
  product(id: ID!): Product
  sizeRecommendation(userId: ID!, productId: ID!): SizeRecommendation
}

type Mutation {
  createMeasurement(input: MeasurementInput!): Measurement
  updateFitPreferences(userId: ID!, preferences: FitPreferencesInput!): User
  createFittingSession(input: FittingSessionInput!): FittingSession
}

type User {
  id: ID!
  measurements: [Measurement!]!
  fitPreferences: FitPreferences
  purchaseHistory: [Purchase!]!
}

type Measurement {
  id: ID!
  timestamp: DateTime!
  height: MeasurementValue!
  weight: MeasurementValue!
  chest: MeasurementValue!
  waist: MeasurementValue!
  hip: MeasurementValue!
  derived: DerivedMetrics
}

type SizeRecommendation {
  primarySize: SizeOption!
  alternatives: [SizeOption!]!
  confidence: Float!
  fitAnalysis: FitAnalysis!
}
```

### 9.3 Example Query
```graphql
query GetUserWithRecommendation($userId: ID!, $productId: ID!) {
  user(id: $userId) {
    id
    measurements {
      id
      height { value unit }
      chest { value unit }
      waist { value unit }
      hip { value unit }
    }
    fitPreferences {
      global { fit }
    }
  }
  product(id: $productId) {
    id
    name
    brand
    sizing {
      sizes {
        label
        measurements { chest { min max } }
      }
    }
  }
  sizeRecommendation(userId: $userId, productId: $productId) {
    primarySize { label confidence fit reasoning }
    alternatives { label confidence fit }
  }
}
```

## 10. Error Handling

### 10.1 Error Response Format (RFC 7807)
```json
{
  "type": "https://wia.org/errors/invalid-measurement",
  "title": "Invalid Measurement Data",
  "status": 400,
  "detail": "Height value 300cm exceeds maximum allowed value of 250cm",
  "instance": "/measurements/meas-abc123",
  "invalidParams": [
    {
      "name": "measurements.height.value",
      "reason": "Value must be between 100 and 250 cm"
    }
  ]
}
```

### 10.2 Common Error Codes
- **400 Bad Request:** Invalid input data
- **401 Unauthorized:** Missing or invalid authentication
- **403 Forbidden:** Insufficient permissions
- **404 Not Found:** Resource does not exist
- **409 Conflict:** Resource conflict (e.g., duplicate)
- **429 Too Many Requests:** Rate limit exceeded
- **500 Internal Server Error:** Server error
- **503 Service Unavailable:** Temporary unavailability

## 11. Versioning

- **URL Versioning:** `/v1/`, `/v2/`
- **Header Versioning:** `Accept: application/vnd.wia.ind-001.v1+json`
- **Deprecation:** 12-month notice via `Sunset` header
- **Backward Compatibility:** Minor versions maintain compatibility

---

**Copyright © 2025 SmileStory Inc. / WIA**
**License:** CC BY 4.0
**弘益人間 - Benefit All Humanity**
