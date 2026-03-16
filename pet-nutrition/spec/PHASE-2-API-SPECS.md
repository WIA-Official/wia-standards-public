# WIA-PET-009: Phase 2 - API Specifications

**Version:** 1.0 | **Status:** Final | **Last Updated:** 2025-12-25

## REST API Endpoints

### Authentication
```
POST /api/v1/auth/token
Content-Type: application/json

{
  "grant_type": "client_credentials",
  "client_id": "string",
  "client_secret": "string",
  "scope": "pet.read pet.write nutrition.read"
}
```

### Pet Management
```
GET    /api/v1/pets/{petId}
POST   /api/v1/pets
PUT    /api/v1/pets/{petId}
DELETE /api/v1/pets/{petId}
GET    /api/v1/pets?ownerId={ownerId}&species={species}
```

### Nutrition Requirements
```
GET    /api/v1/pets/{petId}/nutrition/requirements
PUT    /api/v1/pets/{petId}/nutrition/requirements
POST   /api/v1/nutrition/calculate
```

### Diet Plans
```
GET    /api/v1/pets/{petId}/diet-plans
POST   /api/v1/pets/{petId}/diet-plans
GET    /api/v1/diet-plans/{planId}
PUT    /api/v1/diet-plans/{planId}
DELETE /api/v1/diet-plans/{planId}
```

### Feeding Logs
```
GET    /api/v1/pets/{petId}/feeding-logs?startDate={date}&endDate={date}
POST   /api/v1/feeding-logs
GET    /api/v1/feeding-logs/{logId}
```

### Allergy Management
```
GET    /api/v1/pets/{petId}/allergies
POST   /api/v1/pets/{petId}/allergies
DELETE /api/v1/pets/{petId}/allergies/{allergyId}
```

### Food Products
```
GET    /api/v1/products
GET    /api/v1/products/{productId}
POST   /api/v1/products/search
GET    /api/v1/products/{productId}/allergen-check?petId={petId}
```

### Weight Tracking
```
GET    /api/v1/pets/{petId}/weight-history
POST   /api/v1/pets/{petId}/weight-measurements
```

## HTTP Status Codes

| Code | Meaning | Usage |
|------|---------|-------|
| 200 | OK | Successful GET, PUT |
| 201 | Created | Successful POST |
| 204 | No Content | Successful DELETE |
| 400 | Bad Request | Invalid JSON, missing required fields |
| 401 | Unauthorized | Missing/invalid token |
| 403 | Forbidden | Insufficient permissions |
| 404 | Not Found | Resource doesn't exist |
| 409 | Conflict | Duplicate resource |
| 422 | Validation Error | Invalid data values |
| 429 | Rate Limit Exceeded | Too many requests |
| 500 | Internal Server Error | Server error |
| 503 | Service Unavailable | Maintenance/downtime |

## Rate Limiting

**Standard Tier:** 1,000 requests/hour per API key  
**Premium Tier:** 10,000 requests/hour per API key  
**Enterprise Tier:** Custom limits

Rate limit headers:
```
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 847
X-RateLimit-Reset: 1640000000
```

## Pagination

For list endpoints:
```
GET /api/v1/pets?page=2&limit=50

Response:
{
  "data": [...],
  "pagination": {
    "page": 2,
    "limit": 50,
    "total": 247,
    "totalPages": 5
  }
}
```

## Error Response Format

```json
{
  "error": {
    "code": "VALIDATION_ERROR",
    "message": "Pet profile validation failed",
    "details": [
      {
        "field": "weight.current",
        "issue": "Must be positive number",
        "value": -5.2
      }
    ],
    "timestamp": "2025-12-25T14:30:00Z",
    "requestId": "req-12345-abcdef"
  }
}
```

## Versioning

URL-based: `/api/v1/`, `/api/v2/`  
Support N-1 version for 24 months

---

**弘益人間 · Benefit All Humanity**  
© 2025 WIA | WIA-PET-009 v1.0 Phase 2
