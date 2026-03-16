# WIA-DATA-006: Data Governance - PHASE 2: API

**Version:** 1.0
**Status:** DRAFT
**Last Updated:** 2025-01-15

---

## Overview

Phase 2 defines the RESTful API specification for accessing and managing data governance artifacts. This enables programmatic interaction with governance metadata, policies, and workflows.

## API Principles

- **RESTful:** Follow REST architectural constraints
- **Versioned:** Support multiple API versions
- **Secure:** OAuth 2.0 / JWT authentication
- **Documented:** OpenAPI 3.0 specification
- **Consistent:** Uniform error handling and responses

## Base URL

```
https://api.governance.{organization}.com/v1
```

## Authentication

### OAuth 2.0 Flow

```http
POST /oauth/token
Content-Type: application/json

{
  "grant_type": "client_credentials",
  "client_id": "your-client-id",
  "client_secret": "your-client-secret",
  "scope": "governance:read governance:write"
}
```

**Response:**

```json
{
  "access_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "governance:read governance:write"
}
```

### Authorization Header

```http
Authorization: Bearer {access_token}
```

## API Endpoints

### 1. Data Assets API

#### List Data Assets

```http
GET /assets?domain={domain}&type={type}&limit={limit}&offset={offset}
```

**Query Parameters:**
- `domain` (optional): Filter by data domain
- `type` (optional): Filter by asset type
- `limit` (optional): Results per page (default: 50, max: 100)
- `offset` (optional): Pagination offset (default: 0)

**Response:**

```json
{
  "assets": [
    {
      "assetId": "uuid",
      "assetName": "customer_data",
      "assetType": "table",
      "domain": "customer",
      "owner": {
        "email": "owner@example.com",
        "name": "Jane Doe"
      }
    }
  ],
  "pagination": {
    "total": 150,
    "limit": 50,
    "offset": 0,
    "hasMore": true
  }
}
```

#### Get Asset Details

```http
GET /assets/{assetId}
```

**Response:** Full asset metadata (see Phase 1 format)

#### Create Data Asset

```http
POST /assets
Content-Type: application/json

{
  "assetName": "new_asset",
  "assetType": "table",
  "domain": "finance",
  "owner": {
    "email": "owner@example.com",
    "name": "John Smith"
  },
  "classification": "confidential",
  "description": "Financial transaction data"
}
```

**Response:**

```json
{
  "assetId": "uuid",
  "status": "created",
  "message": "Asset successfully created"
}
```

#### Update Data Asset

```http
PATCH /assets/{assetId}
Content-Type: application/json

{
  "description": "Updated description",
  "stewards": [
    {
      "email": "steward@example.com",
      "name": "Alice Johnson",
      "role": "business"
    }
  ]
}
```

#### Delete Data Asset

```http
DELETE /assets/{assetId}
```

**Response:**

```json
{
  "status": "deleted",
  "message": "Asset successfully deleted"
}
```

### 2. Policies API

#### List Policies

```http
GET /policies?type={type}&status={status}
```

**Query Parameters:**
- `type`: Filter by policy type (quality, security, privacy, retention, access)
- `status`: Filter by status (draft, active, deprecated)

#### Get Policy Details

```http
GET /policies/{policyId}
```

#### Create Policy

```http
POST /policies
Content-Type: application/json

{
  "policyName": "Data Retention Policy",
  "policyType": "retention",
  "statement": "All customer data must be retained for 7 years",
  "requirements": [
    {
      "requirementId": "RET-001",
      "description": "Retain transaction data for 7 years",
      "mandatory": true
    }
  ]
}
```

#### Approve Policy

```http
POST /policies/{policyId}/approve
Content-Type: application/json

{
  "approver": {
    "email": "approver@example.com",
    "name": "Chief Data Officer"
  },
  "comments": "Approved for implementation"
}
```

### 3. Business Glossary API

#### Search Terms

```http
GET /glossary/terms?q={query}&domain={domain}
```

**Query Parameters:**
- `q`: Search query (name, definition, synonyms)
- `domain`: Filter by data domain

#### Get Term

```http
GET /glossary/terms/{termId}
```

#### Create Term

```http
POST /glossary/terms
Content-Type: application/json

{
  "termName": "Customer Lifetime Value",
  "definition": "Total revenue expected from a customer",
  "domain": "marketing",
  "synonyms": ["CLV", "LTV"],
  "owner": {
    "email": "owner@example.com",
    "name": "Marketing Lead"
  }
}
```

### 4. Access Requests API

#### Submit Access Request

```http
POST /access-requests
Content-Type: application/json

{
  "assetId": "uuid",
  "accessLevel": "read",
  "justification": "Need data for quarterly analysis",
  "duration": "P90D"
}
```

**Response:**

```json
{
  "requestId": "uuid",
  "status": "pending",
  "workflow": [
    {
      "step": 1,
      "approver": "data.owner@example.com",
      "status": "pending"
    }
  ]
}
```

#### Get Request Status

```http
GET /access-requests/{requestId}
```

#### Approve/Reject Request

```http
POST /access-requests/{requestId}/decision
Content-Type: application/json

{
  "decision": "approved",
  "comments": "Access granted for analysis purposes"
}
```

### 5. Data Quality API

#### Get Quality Metrics

```http
GET /quality/assets/{assetId}
```

**Response:**

```json
{
  "assetId": "uuid",
  "qualityScore": 85,
  "dimensions": {
    "completeness": 90,
    "accuracy": 88,
    "consistency": 82,
    "timeliness": 80
  },
  "issues": [
    {
      "issueId": "uuid",
      "severity": "medium",
      "description": "10% of email addresses are invalid",
      "affectedRecords": 1500
    }
  ],
  "lastAssessed": "2025-01-15T10:30:00Z"
}
```

#### Create Quality Rule

```http
POST /quality/rules
Content-Type: application/json

{
  "ruleName": "Email Validation",
  "ruleType": "validity",
  "assetId": "uuid",
  "field": "customer_email",
  "expression": "REGEXP_LIKE(customer_email, '^[A-Za-z0-9._%+-]+@[A-Za-z0-9.-]+\\.[A-Z]{2,}$')",
  "threshold": {
    "operator": "gte",
    "value": 95
  },
  "severity": "high"
}
```

#### Execute Quality Check

```http
POST /quality/rules/{ruleId}/execute
```

### 6. Lineage API

#### Get Asset Lineage

```http
GET /lineage/assets/{assetId}?direction={upstream|downstream|both}&depth={depth}
```

**Query Parameters:**
- `direction`: Lineage direction (upstream, downstream, both)
- `depth`: How many levels to traverse (default: 3, max: 10)

**Response:**

```json
{
  "assetId": "uuid",
  "lineage": {
    "upstream": [
      {
        "assetId": "uuid",
        "assetName": "source_table",
        "transformation": "SELECT * FROM source WHERE status='active'"
      }
    ],
    "downstream": [
      {
        "assetId": "uuid",
        "assetName": "analytics_view",
        "transformation": "Aggregation by customer_id"
      }
    ]
  }
}
```

## Error Handling

### Standard Error Response

```json
{
  "error": {
    "code": "ERROR_CODE",
    "message": "Human-readable error message",
    "details": [
      {
        "field": "assetName",
        "issue": "Asset name already exists"
      }
    ],
    "timestamp": "2025-01-15T10:30:00Z",
    "requestId": "uuid"
  }
}
```

### Error Codes

- `400` - Bad Request (invalid input)
- `401` - Unauthorized (missing or invalid token)
- `403` - Forbidden (insufficient permissions)
- `404` - Not Found (resource doesn't exist)
- `409` - Conflict (resource already exists)
- `422` - Unprocessable Entity (validation failed)
- `429` - Too Many Requests (rate limit exceeded)
- `500` - Internal Server Error
- `503` - Service Unavailable

## Rate Limiting

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 999
X-RateLimit-Reset: 1642262400
```

- **Limit:** 1000 requests per hour per client
- **Burst:** 50 requests per minute
- **Retry-After:** Included in 429 response (seconds)

## Webhooks

### Subscribe to Events

```http
POST /webhooks
Content-Type: application/json

{
  "url": "https://your-app.com/webhook",
  "events": [
    "asset.created",
    "asset.updated",
    "policy.approved",
    "access.granted"
  ],
  "secret": "webhook-signing-secret"
}
```

### Event Payload

```json
{
  "eventId": "uuid",
  "eventType": "asset.created",
  "timestamp": "2025-01-15T10:30:00Z",
  "data": {
    "assetId": "uuid",
    "assetName": "new_asset"
  }
}
```

## SDK Support

Official SDKs available for:
- **Python:** `pip install wia-governance`
- **JavaScript/TypeScript:** `npm install @wia/governance`
- **Java:** Maven/Gradle dependency
- **Go:** `go get github.com/wia/governance-go`

### Example (Python)

```python
from wia_governance import GovernanceClient

client = GovernanceClient(
    api_url="https://api.governance.example.com/v1",
    client_id="your-client-id",
    client_secret="your-client-secret"
)

# List assets
assets = client.assets.list(domain="customer", limit=10)

# Get asset details
asset = client.assets.get("asset-uuid")

# Create asset
new_asset = client.assets.create(
    name="new_dataset",
    type="table",
    domain="finance",
    owner="owner@example.com"
)
```

## Testing

### Test Environment

```
https://api.governance.test.{organization}.com/v1
```

### Mock Data

Test data available at `/test/fixtures`

## Next Steps

After implementing Phase 2 APIs:

1. **Test** all API endpoints
2. **Document** custom endpoints
3. **Deploy** to staging environment
4. **Proceed** to Phase 3: Protocol implementation

---

**弘益人間 (홍익인간) · Benefit All Humanity**

© 2025 SmileStory Inc. / WIA
