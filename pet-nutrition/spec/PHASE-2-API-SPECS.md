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

## Resource Path Inventory

| Resource | Method | Path | Notes |
|----------|--------|------|-------|
| Pet | GET | `/api/v1/pets/{petId}` | Single pet record |
| Pet | POST | `/api/v1/pets` | Create new pet |
| Pet | PATCH | `/api/v1/pets/{petId}` | Update fields |
| Pet | DELETE | `/api/v1/pets/{petId}` | Soft delete |
| Pet List | GET | `/api/v1/pets` | Cursor pagination |
| Nutrition | POST | `/api/v1/nutrition/calculate` | Returns DER, macros |
| Plan | POST | `/api/v1/plans` | Generate 7- or 30-day plan |
| Plan | GET | `/api/v1/plans/{planId}` | Read plan |
| Recipe | GET | `/api/v1/recipes` | List recipes |
| Recipe | GET | `/api/v1/recipes/{recipeId}` | Single recipe |
| Webhook | POST | `/api/v1/webhooks` | Register callback |
| Webhook | DELETE | `/api/v1/webhooks/{id}` | Remove callback |
| Audit | GET | `/api/v1/audit` | Tenant audit log |
| Health | GET | `/api/v1/health` | Liveness probe |

All endpoints require Bearer authentication except `/api/v1/health`.

## OpenAPI Schema Highlights

```yaml
PetProfile:
  type: object
  required: [petId, species, breed, age, weight, activityLevel]
  properties:
    petId:    { type: string, pattern: "^pet-[A-Za-z0-9]{6,32}$" }
    species:  { type: string, enum: [dog, cat, rabbit, ferret, guinea-pig] }
    breed:    { type: string }
    age:
      type: object
      required: [years, months]
      properties:
        years:  { type: integer, minimum: 0, maximum: 30 }
        months: { type: integer, minimum: 0, maximum: 11 }
    weight:
      type: object
      required: [current, units]
      properties:
        current: { type: number, minimum: 0 }
        target:  { type: number, minimum: 0 }
        units:   { type: string, enum: [kg, lb] }
    activityLevel:
      type: string
      enum: [sedentary, light, moderate, active, working]
    healthFlags:
      type: array
      items: { type: string }
```

## Idempotency

`POST /api/v1/pets`, `POST /api/v1/feeding-plans`, and similar create endpoints MUST honour the header `Idempotency-Key`. The platform stores the original response for at least 24 hours and replays it on retry. Keys are scoped per (tenant, route).

## Rate Limits

| Tier | Requests/min | Burst |
|------|--------------|-------|
| Free | 60 | 20 |
| Pro | 600 | 200 |
| Clinic | 6000 | 2000 |

429 responses MUST include `Retry-After`. Burst budgets reset every 60 seconds via a token-bucket algorithm.

## Pagination

All list endpoints MUST use opaque cursors:

```
GET /api/v1/pets?limit=50&cursor=eyJ0IjoxNzM1MTI4MDAwfQ==

200 OK
{
  "data": [...],
  "nextCursor": "eyJ0IjoxNzM1MTI5MDAwfQ==",
  "hasMore": true
}
```

`nextCursor` MUST be omitted when no further data exists. Clients MUST treat the cursor as opaque and never parse it.

## Error Catalog

| HTTP | Type slug | When |
|------|-----------|------|
| 400 | `validation-error` | JSON Schema mismatch, invalid units, negative weight |
| 401 | `unauthenticated` | Missing or invalid Bearer token |
| 403 | `forbidden` | Token scope insufficient for resource |
| 404 | `pet-not-found`, `plan-not-found` | Resource missing |
| 409 | `pet-already-exists`, `idempotency-conflict` | Duplicate creation, mismatching idempotency key |
| 422 | `nutrition-infeasible` | No recipe satisfies the constraints |
| 429 | `rate-limited` | Tier quota exceeded |
| 500 | `internal-error` | Unhandled, with correlation ID for support |
| 503 | `dependency-unavailable` | Upstream nutrient registry temporarily down |

All error responses MUST be `application/problem+json` (RFC 9457):

```json
{
  "type": "https://standards.wia.example/pet-nutrition/errors/nutrition-infeasible",
  "title": "Nutrition plan infeasible",
  "status": 422,
  "detail": "No commercial recipe meets all constraints (low-fat, grain-free, < 1500 kcal/day).",
  "instance": "/api/v1/plans"
}
```

## Webhook Notifications

Clinics MAY register a callback URL per pet/event combination:

```
POST /api/v1/webhooks
{
  "url": "https://clinic.example/wia-cb",
  "secret": "<base64 random ≥ 32 bytes>",
  "events": ["pet.weight.alert", "pet.feeding.skipped", "pet.health.followup"]
}
```

Each delivery MUST be signed with `WIA-Signature: t=<unix>,v1=<HMAC-SHA-256(secret, "{t}.{body}")>` and treated as idempotent on `WIA-Event-Id`.

## SDK Coverage

| Language | Package | Min runtime |
|----------|---------|-------------|
| TypeScript / Node | `@wia/pet-009-sdk` | Node 18 |
| Python | `wia-pet-009` | Python 3.10 |
| Go | `github.com/wia-official/pet009-go` | Go 1.21 |
| Swift | `WiaPet009` (SwiftPM) | iOS 16 |
| Kotlin | `org.wia.pet009` | Android 24 |

All SDKs MUST emit OpenTelemetry spans and respect the platform-level `Idempotency-Key` and `Retry-After` semantics without manual user intervention.

## Authentication & Scopes

OAuth 2.0 client credentials and OIDC user flows are both supported. Standard scopes:

| Scope | Allows |
|-------|--------|
| `pet.read` | Read pet records |
| `pet.write` | Create / update pet records |
| `pet.delete` | Soft-delete pet records |
| `nutrition.read` | Calculate nutritional requirements |
| `plans.write` | Generate feeding plans |
| `recipes.read` | Read the recipe catalogue |
| `webhooks.write` | Register webhooks |
| `audit.read` | Read tenant audit log |

Tokens MUST be JWT with `kid` set; the JWKS endpoint MUST be discoverable through `/.well-known/openid-configuration`.

## Bulk Operations

Clinics commonly onboard hundreds of patients at once. The API supports bulk endpoints with at-least-once delivery semantics:

```
POST /api/v1/pets:bulk
Content-Type: application/json
Idempotency-Key: 01HG…

{
  "operations": [
    {"op": "create", "pet": { ...PetProfile1 }},
    {"op": "update", "petId": "pet-abc123", "patch": {"weight": {"current": 27.4, "units": "kg"}}}
  ]
}
```

Responses are per-operation:

```json
{
  "results": [
    {"index": 0, "status": "created", "petId": "pet-xyz999"},
    {"index": 1, "status": "updated", "petId": "pet-abc123"}
  ]
}
```

Bulk batches MUST contain ≤ 1,000 operations and ≤ 5 MB payload. Mixed-success batches return HTTP 207 Multi-Status; entirely successful batches return 200; entirely failed batches return 4xx with the first error promoted to the top-level body.

## CSV Import

For onboarding from non-API systems the platform exposes a streaming CSV import:

```
POST /api/v1/pets:import
Content-Type: text/csv

petId,species,breed,birthDate,weightKg,activityLevel
pet-abc,dog,Beagle,2020-04-15,12.4,moderate
pet-def,cat,Domestic Shorthair,2018-09-01,4.8,light
```

The platform responds with a job ID; clients poll `/api/v1/imports/{jobId}` until status `completed`. Failed rows are returned as a downloadable CSV with the original row plus a `wia_error` column.

## API Versioning Policy

- New fields MAY be added without bumping the version.
- Renaming or removing fields requires a new MAJOR version.
- The platform supports the prior MAJOR for ≥ 24 months after a new MAJOR ships.
- Deprecation announcements appear in the `Sunset` and `Deprecation` headers per RFC 9745 (Sunset HTTP Header) and RFC 8594 (Sunset HTTP Header — Sunset).

## Multi-Tenancy

Every API call resolves a tenant from the bearer token. Cross-tenant access is forbidden; even superadmins MUST impersonate explicitly via `X-WIA-Impersonate-Tenant` and impersonation events MUST appear in the tenant audit log.

## Streaming Endpoints

For dashboards that need near-real-time updates the platform exposes Server-Sent Events:

```
GET /api/v1/streams/feeding-events?since=2026-04-26T00:00:00Z
Accept: text/event-stream

event: pet.feeding.started
data: {"petId":"pet-abc","plannedKcal":520,"timestamp":"2026-04-26T13:01:00Z"}

event: pet.feeding.completed
data: {"petId":"pet-abc","actualKcal":520,"timestamp":"2026-04-26T13:08:00Z"}

event: pet.feeding.skipped
data: {"petId":"pet-abc","reason":"owner-cancel","timestamp":"2026-04-26T18:01:00Z"}
```

Streams MUST emit a heartbeat comment line every 30 seconds and resume from the `Last-Event-ID` if a client reconnects within 5 minutes.

## Long-Running Operations

Some operations (bulk imports, multi-pet plan recalculation, large recipe migrations) can take minutes. They follow the long-running operation pattern:

```
POST /api/v1/plans/recalculate
Accept: application/json

202 Accepted
Location: /api/v1/operations/op-12345
{
  "operationId": "op-12345",
  "status": "running",
  "createdAt": "2026-04-26T13:00:00Z",
  "estimatedSeconds": 120
}
```

```
GET /api/v1/operations/op-12345
{
  "operationId": "op-12345",
  "status": "completed",
  "result": { "plansAffected": 1284, "errors": 0 }
}
```

Operations entries MUST be retained for ≥ 7 days so clients can poll lazily.

## Sandbox Environment

The platform MUST provide a no-cost sandbox at `https://sandbox-api.wia.org/pet-nutrition/v1`. Sandbox traffic MUST NOT be billed, MUST be reset every 24 hours, and MUST NOT participate in cross-tenant analytics.

---

**弘益人間 · Benefit All Humanity**  
© 2025 WIA | WIA-PET-009 v1.0 Phase 2
