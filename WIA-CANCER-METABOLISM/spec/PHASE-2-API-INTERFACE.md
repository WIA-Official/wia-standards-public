# WIA-CANCER-METABOLISM Phase 2: API Interface Specification

**Version**: 1.0
**Status**: Draft
**Last Updated**: 2025-12-29

## Overview

The WIA-CANCER-METABOLISM API Interface specification defines RESTful web services for accessing, submitting, and analyzing cancer metabolism data. This API enables seamless integration between clinical systems, research platforms, and analytical tools while maintaining security, privacy, and data integrity.

### API Design Principles

1. **RESTful Architecture**: Resource-oriented design with standard HTTP methods
2. **HATEOAS**: Hypermedia-driven navigation between resources
3. **Stateless**: Each request contains all necessary information
4. **Cacheable**: Appropriate cache headers for performance optimization
5. **Versioned**: Explicit API versioning for backward compatibility
6. **Secure**: OAuth 2.0 authentication and encryption by default

## Base URL

```
Production: https://api.wia.org/cancer-metabolism/v1
Staging: https://api-staging.wia.org/cancer-metabolism/v1
Development: https://api-dev.wia.org/cancer-metabolism/v1
```

## Authentication

### OAuth 2.0

The API uses OAuth 2.0 with the Client Credentials flow for machine-to-machine authentication and Authorization Code flow for user-based access.

#### Client Credentials Flow (M2M)

```http
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials
&client_id=YOUR_CLIENT_ID
&client_secret=YOUR_CLIENT_SECRET
&scope=read:profiles write:profiles read:biomarkers
```

**Response**:
```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "read:profiles write:profiles read:biomarkers"
}
```

#### Authorization Code Flow (User-based)

```
Step 1: Authorization Request
GET /oauth/authorize?response_type=code
  &client_id=YOUR_CLIENT_ID
  &redirect_uri=https://yourapp.com/callback
  &scope=read:profiles write:profiles
  &state=RANDOM_STATE_STRING

Step 2: Token Exchange
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=authorization_code
&code=AUTHORIZATION_CODE
&client_id=YOUR_CLIENT_ID
&client_secret=YOUR_CLIENT_SECRET
&redirect_uri=https://yourapp.com/callback
```

### API Keys (Legacy)

For backward compatibility, API keys are supported but OAuth 2.0 is recommended.

```http
GET /api/v1/biomarkers
X-API-Key: your-api-key-here
```

### Scopes

| Scope | Description |
|-------|-------------|
| `read:profiles` | Read metabolic profiles |
| `write:profiles` | Create and update metabolic profiles |
| `delete:profiles` | Delete metabolic profiles |
| `read:biomarkers` | Read biomarker definitions |
| `write:biomarkers` | Create and update biomarkers |
| `read:metabolites` | Read metabolite data |
| `write:metabolites` | Create and update metabolites |
| `read:pathways` | Read pathway information |
| `write:pathways` | Create and update pathways |
| `read:patients` | Read patient-associated data |
| `write:patients` | Update patient-associated data |
| `admin` | Full administrative access |

## RESTful API Endpoints

### 1. Metabolic Profiles

#### GET /profiles

Retrieve a list of metabolic profiles with filtering and pagination.

**Query Parameters**:
- `patientId` (string, optional): Filter by patient identifier
- `cancerType` (string, optional): Filter by ICD-O-3 code
- `dateFrom` (ISO 8601, optional): Start date for collection date range
- `dateTo` (ISO 8601, optional): End date for collection date range
- `sampleType` (enum, optional): Filter by tissue type
- `institution` (string, optional): Filter by institution
- `page` (integer, default: 1): Page number
- `pageSize` (integer, default: 20, max: 100): Results per page
- `sort` (string, default: "-collectionDate"): Sort field (prefix with - for descending)

**Request Example**:
```http
GET /profiles?cancerType=C50.9&dateFrom=2025-01-01&page=1&pageSize=20
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
Accept: application/json
```

**Response (200 OK)**:
```json
{
  "data": [
    {
      "profileId": "a1b2c3d4-e5f6-4a7b-8c9d-0e1f2a3b4c5d",
      "patientId": "PATIENT-BC-00123",
      "sampleId": "SAMPLE-2025-001234",
      "collectionDate": "2025-03-15T09:30:00Z",
      "cancerType": "C50.9",
      "cancerStage": "T2N1M0",
      "sampleType": {
        "tissue": "tumor",
        "source": "breast-upper-outer-quadrant"
      },
      "metaboliteCount": 147,
      "pathwayCount": 12,
      "biomarkerCount": 3,
      "_links": {
        "self": { "href": "/profiles/a1b2c3d4-e5f6-4a7b-8c9d-0e1f2a3b4c5d" },
        "metabolites": { "href": "/profiles/a1b2c3d4-e5f6-4a7b-8c9d-0e1f2a3b4c5d/metabolites" },
        "pathways": { "href": "/profiles/a1b2c3d4-e5f6-4a7b-8c9d-0e1f2a3b4c5d/pathways" }
      }
    }
  ],
  "pagination": {
    "page": 1,
    "pageSize": 20,
    "totalPages": 15,
    "totalRecords": 287
  },
  "_links": {
    "self": { "href": "/profiles?page=1&pageSize=20" },
    "next": { "href": "/profiles?page=2&pageSize=20" },
    "last": { "href": "/profiles?page=15&pageSize=20" }
  }
}
```

#### GET /profiles/{profileId}

Retrieve a complete metabolic profile by ID.

**Path Parameters**:
- `profileId` (UUID): Unique profile identifier

**Request Example**:
```http
GET /profiles/a1b2c3d4-e5f6-4a7b-8c9d-0e1f2a3b4c5d
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
Accept: application/json
```

**Response (200 OK)**: Full MetabolicProfile object (see Phase 1 spec)

#### POST /profiles

Create a new metabolic profile.

**Required Scope**: `write:profiles`

**Request Example**:
```http
POST /profiles
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
Content-Type: application/json

{
  "patientId": "PATIENT-BC-00125",
  "sampleId": "SAMPLE-2025-001250",
  "collectionDate": "2025-03-20T10:15:00Z",
  "cancerType": "C50.9",
  "cancerStage": "T1N0M0",
  "sampleType": {
    "tissue": "tumor",
    "source": "breast-lower-inner-quadrant",
    "quality": {
      "cellularity": 78,
      "viability": 88,
      "necrosisLevel": 8
    }
  },
  "metabolites": [
    {
      "metaboliteId": "HMDB0000190",
      "name": "Lactate",
      "concentration": {
        "value": 12.4,
        "unit": "mM",
        "method": "LC-MS/MS"
      },
      "confidence": 0.96
    }
  ],
  "metadata": {
    "institution": "City Cancer Institute",
    "platform": "Waters-Xevo-TQ-S",
    "protocol": "SOP-METABOLISM-2025-v2.1"
  }
}
```

**Response (201 Created)**:
```json
{
  "profileId": "b2c3d4e5-f6a7-4b8c-9d0e-1f2a3b4c5d6e",
  "message": "Metabolic profile created successfully",
  "_links": {
    "self": { "href": "/profiles/b2c3d4e5-f6a7-4b8c-9d0e-1f2a3b4c5d6e" }
  }
}
```

#### PUT /profiles/{profileId}

Update an existing metabolic profile.

**Required Scope**: `write:profiles`

**Request**: Same structure as POST

**Response (200 OK)**: Updated profile object

#### PATCH /profiles/{profileId}

Partially update a metabolic profile (add metabolites, update metadata, etc.).

**Required Scope**: `write:profiles`

**Request Example**:
```http
PATCH /profiles/a1b2c3d4-e5f6-4a7b-8c9d-0e1f2a3b4c5d
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
Content-Type: application/json

{
  "op": "add",
  "path": "/metabolites/-",
  "value": {
    "metaboliteId": "HMDB0000158",
    "name": "Glutamine",
    "concentration": {
      "value": 9.2,
      "unit": "mM",
      "method": "LC-MS/MS"
    },
    "confidence": 0.95
  }
}
```

**Response (200 OK)**: Updated profile

#### DELETE /profiles/{profileId}

Delete a metabolic profile (soft delete with retention policy).

**Required Scope**: `delete:profiles`

**Response (204 No Content)**

### 2. Biomarkers

#### GET /biomarkers

List all cancer metabolism biomarkers.

**Query Parameters**:
- `cancerType` (string): Filter by cancer type
- `category` (enum): Filter by diagnostic/prognostic/predictive/monitoring
- `fdaApproved` (boolean): Filter by FDA approval status
- `minSensitivity` (number): Minimum sensitivity threshold
- `minSpecificity` (number): Minimum specificity threshold

**Request Example**:
```http
GET /biomarkers?cancerType=C25&category=diagnostic&minSensitivity=80
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
```

**Response (200 OK)**:
```json
{
  "data": [
    {
      "biomarkerId": "f6e5d4c3-b2a1-4d5e-6f7a-8b9c0d1e2f3a",
      "name": "Warburg Effect Index",
      "type": "ratio",
      "category": "diagnostic",
      "cancerTypes": ["C25.0", "C25.1"],
      "clinicalUtility": {
        "sensitivity": 87.5,
        "specificity": 82.3,
        "auc": 0.89
      },
      "_links": {
        "self": { "href": "/biomarkers/f6e5d4c3-b2a1-4d5e-6f7a-8b9c0d1e2f3a" }
      }
    }
  ],
  "pagination": {
    "page": 1,
    "pageSize": 20,
    "totalRecords": 45
  }
}
```

#### GET /biomarkers/{biomarkerId}

Retrieve detailed biomarker information.

**Response (200 OK)**: Full Biomarker object (see Phase 1 spec)

#### POST /biomarkers

Create a new biomarker definition.

**Required Scope**: `write:biomarkers`

**Request**: Biomarker object (see Phase 1 spec)

**Response (201 Created)**: Created biomarker with ID

#### GET /biomarkers/{biomarkerId}/evaluate

Evaluate a biomarker against a metabolic profile.

**Query Parameters**:
- `profileId` (UUID, required): Profile to evaluate

**Response (200 OK)**:
```json
{
  "biomarkerId": "f6e5d4c3-b2a1-4d5e-6f7a-8b9c0d1e2f3a",
  "profileId": "a1b2c3d4-e5f6-4a7b-8c9d-0e1f2a3b4c5d",
  "value": 6.87,
  "unit": "ratio",
  "interpretation": "elevated",
  "clinicalSignificance": "Indicates enhanced glycolytic activity, suggestive of malignancy",
  "confidence": 0.94,
  "referenceRange": {
    "healthy": { "min": 2.0, "max": 4.5 },
    "cancerous": { "min": 8.5, "max": 25.0 }
  },
  "percentile": 65,
  "evaluatedAt": "2025-03-20T14:30:00Z"
}
```

### 3. Metabolites

#### GET /metabolites

Search and retrieve metabolite information.

**Query Parameters**:
- `query` (string): Search term (name, HMDB ID, KEGG ID)
- `class` (string): Filter by chemical class
- `pathway` (string): Filter by pathway involvement
- `cancerRelevance` (boolean): Only cancer-relevant metabolites

**Request Example**:
```http
GET /metabolites?query=lactate&cancerRelevance=true
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
```

**Response (200 OK)**:
```json
{
  "data": [
    {
      "metaboliteId": "HMDB0000190",
      "name": {
        "common": "Lactate",
        "iupac": "2-Hydroxypropanoic acid"
      },
      "externalIds": {
        "hmdb": "HMDB0000190",
        "kegg": "C00186",
        "chebi": "CHEBI:422"
      },
      "structure": {
        "molecularFormula": "C3H6O3",
        "molecularWeight": 90.08
      },
      "cancerMetabolism": {
        "alterationPattern": "elevated",
        "therapeuticRelevance": "Target for lactate dehydrogenase inhibitors"
      },
      "_links": {
        "self": { "href": "/metabolites/HMDB0000190" },
        "pathways": { "href": "/metabolites/HMDB0000190/pathways" }
      }
    }
  ]
}
```

#### GET /metabolites/{metaboliteId}

Retrieve detailed metabolite information.

**Response (200 OK)**: Full Metabolite object (see Phase 1 spec)

#### GET /metabolites/{metaboliteId}/pathways

Get all pathways involving this metabolite.

**Response (200 OK)**: Array of MetabolicPathway references

#### GET /metabolites/{metaboliteId}/cancer-context

Retrieve cancer-specific metabolite alterations across cancer types.

**Response (200 OK)**:
```json
{
  "metaboliteId": "HMDB0000190",
  "name": "Lactate",
  "cancerAlterations": [
    {
      "cancerType": "C50.9",
      "cancerName": "Breast Cancer",
      "alterationDirection": "up",
      "averageFoldChange": 3.2,
      "pValue": 0.00001,
      "studies": 47,
      "clinicalImplications": "Associated with poor prognosis and therapy resistance"
    },
    {
      "cancerType": "C61",
      "cancerName": "Prostate Cancer",
      "alterationDirection": "up",
      "averageFoldChange": 2.8,
      "pValue": 0.0001,
      "studies": 32,
      "clinicalImplications": "Marker of aggressive disease"
    }
  ]
}
```

### 4. Pathways

#### GET /pathways

List metabolic pathways with cancer relevance.

**Query Parameters**:
- `category` (enum): Filter by pathway category (glycolysis, TCA, etc.)
- `cancerType` (string): Filter by cancer relevance
- `therapeuticTarget` (boolean): Only druggable pathways
- `hallmark` (string): Filter by cancer hallmark

**Response (200 OK)**: Array of pathway summaries

#### GET /pathways/{pathwayId}

Retrieve detailed pathway information.

**Response (200 OK)**: Full MetabolicPathway object (see Phase 1 spec)

#### GET /pathways/{pathwayId}/enrichment

Perform pathway enrichment analysis on a metabolic profile.

**Query Parameters**:
- `profileId` (UUID, required): Profile to analyze

**Response (200 OK)**:
```json
{
  "pathwayId": "hsa00010",
  "profileId": "a1b2c3d4-e5f6-4a7b-8c9d-0e1f2a3b4c5d",
  "enrichmentScore": 0.78,
  "pValue": 0.0001,
  "fdr": 0.002,
  "method": "GSEA",
  "alteredMetabolites": 23,
  "totalMetabolites": 68,
  "coverage": 33.8,
  "direction": "upregulated",
  "leadingEdgeMetabolites": [
    "HMDB0000190",
    "HMDB0000122",
    "HMDB0000158"
  ]
}
```

### 5. Patient Data

#### GET /patients/{patientId}/profiles

Retrieve all metabolic profiles for a patient (longitudinal data).

**Required Scope**: `read:patients`

**Response (200 OK)**:
```json
{
  "patientId": "PATIENT-BC-00123",
  "profiles": [
    {
      "profileId": "...",
      "collectionDate": "2025-01-15T09:00:00Z",
      "timepoint": "baseline",
      "treatmentStatus": "pre-treatment"
    },
    {
      "profileId": "...",
      "collectionDate": "2025-03-15T09:00:00Z",
      "timepoint": "cycle-3",
      "treatmentStatus": "on-treatment"
    }
  ],
  "timeline": {
    "firstCollection": "2025-01-15T09:00:00Z",
    "lastCollection": "2025-03-15T09:00:00Z",
    "totalProfiles": 2
  }
}
```

#### GET /patients/{patientId}/trends

Analyze metabolic trends over time for a patient.

**Query Parameters**:
- `metaboliteId` (string, optional): Specific metabolite to track
- `pathwayId` (string, optional): Specific pathway to track
- `biomarkerId` (string, optional): Specific biomarker to track

**Response (200 OK)**:
```json
{
  "patientId": "PATIENT-BC-00123",
  "metric": "biomarker",
  "biomarkerId": "f6e5d4c3-b2a1-4d5e-6f7a-8b9c0d1e2f3a",
  "timeSeries": [
    {
      "date": "2025-01-15T09:00:00Z",
      "value": 8.2,
      "interpretation": "elevated"
    },
    {
      "date": "2025-03-15T09:00:00Z",
      "value": 5.1,
      "interpretation": "normalizing"
    }
  ],
  "trend": "decreasing",
  "changePercentage": -37.8,
  "clinicalSignificance": "Positive response to therapy"
}
```

## HTTP Response Codes

| Code | Description | Usage |
|------|-------------|-------|
| 200 | OK | Successful GET, PUT, PATCH |
| 201 | Created | Successful POST |
| 204 | No Content | Successful DELETE |
| 400 | Bad Request | Invalid request syntax or validation failure |
| 401 | Unauthorized | Missing or invalid authentication |
| 403 | Forbidden | Insufficient permissions (scope) |
| 404 | Not Found | Resource does not exist |
| 409 | Conflict | Resource already exists or state conflict |
| 422 | Unprocessable Entity | Validation error on request body |
| 429 | Too Many Requests | Rate limit exceeded |
| 500 | Internal Server Error | Server-side error |
| 503 | Service Unavailable | Temporary service outage |

## Error Response Format

All errors follow a consistent structure:

```json
{
  "error": {
    "code": "VALIDATION_ERROR",
    "message": "The request body contains invalid data",
    "details": [
      {
        "field": "metabolites[0].concentration.value",
        "issue": "Must be a positive number",
        "rejectedValue": -5.2
      }
    ],
    "requestId": "req_a1b2c3d4e5f6",
    "timestamp": "2025-03-20T14:30:00Z",
    "documentation": "https://docs.wia.org/cancer-metabolism/errors#VALIDATION_ERROR"
  }
}
```

### Error Codes

| Code | HTTP Status | Description |
|------|-------------|-------------|
| `INVALID_REQUEST` | 400 | Malformed request syntax |
| `VALIDATION_ERROR` | 422 | Request body validation failed |
| `UNAUTHORIZED` | 401 | Authentication required |
| `FORBIDDEN` | 403 | Insufficient permissions |
| `NOT_FOUND` | 404 | Resource not found |
| `CONFLICT` | 409 | Resource conflict |
| `RATE_LIMIT_EXCEEDED` | 429 | Too many requests |
| `INTERNAL_ERROR` | 500 | Internal server error |
| `SERVICE_UNAVAILABLE` | 503 | Temporary outage |

## Rate Limiting

Rate limits protect API stability and ensure fair usage.

### Limits by Tier

| Tier | Requests/Hour | Requests/Day | Burst |
|------|---------------|--------------|-------|
| Free | 1,000 | 10,000 | 20/min |
| Research | 10,000 | 100,000 | 100/min |
| Clinical | 50,000 | 500,000 | 500/min |
| Enterprise | Custom | Custom | Custom |

### Rate Limit Headers

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 847
X-RateLimit-Reset: 1711891200
X-RateLimit-Window: 3600
```

### Rate Limit Exceeded Response

```json
{
  "error": {
    "code": "RATE_LIMIT_EXCEEDED",
    "message": "Rate limit exceeded. Please retry after 300 seconds.",
    "retryAfter": 300,
    "limit": 1000,
    "window": 3600
  }
}
```

## Pagination

All list endpoints support cursor-based and offset-based pagination.

### Offset-Based (Default)

```http
GET /profiles?page=2&pageSize=50
```

**Response Headers**:
```http
X-Total-Count: 287
Link: </profiles?page=1&pageSize=50>; rel="first",
      </profiles?page=3&pageSize=50>; rel="next",
      </profiles?page=6&pageSize=50>; rel="last"
```

### Cursor-Based (Recommended for large datasets)

```http
GET /profiles?cursor=eyJpZCI6ImExYjJjM2Q0In0&limit=50
```

**Response**:
```json
{
  "data": [...],
  "pagination": {
    "nextCursor": "eyJpZCI6ImU1ZjZhN2I4In0",
    "hasMore": true
  }
}
```

## Filtering and Sorting

### Filtering Operators

- `eq`: Equal (default)
- `ne`: Not equal
- `gt`: Greater than
- `gte`: Greater than or equal
- `lt`: Less than
- `lte`: Less than or equal
- `in`: In array
- `contains`: String contains

**Example**:
```http
GET /profiles?cancerStage[in]=T2N1M0,T3N1M0&collectionDate[gte]=2025-01-01
```

### Sorting

Use the `sort` parameter with field names. Prefix with `-` for descending order.

```http
GET /profiles?sort=-collectionDate,cancerType
```

## SDK Support

Official SDKs are available for multiple languages:

### TypeScript/JavaScript

```bash
npm install @wia/cancer-metabolism
```

```typescript
import { WIACancerMetabolism } from '@wia/cancer-metabolism';

const client = new WIACancerMetabolism({
  apiKey: process.env.WIA_API_KEY,
  baseUrl: 'https://api.wia.org/cancer-metabolism/v1'
});

// Get metabolic profile
const profile = await client.profiles.get('a1b2c3d4-e5f6-4a7b-8c9d-0e1f2a3b4c5d');

// Create new profile
const newProfile = await client.profiles.create({
  patientId: 'PATIENT-001',
  sampleId: 'SAMPLE-2025-001',
  collectionDate: new Date().toISOString(),
  cancerType: 'C50.9',
  metabolites: [...]
});

// Search biomarkers
const biomarkers = await client.biomarkers.list({
  cancerType: 'C50',
  minSensitivity: 80
});
```

### Python

```bash
pip install wia-cancer-metabolism
```

```python
from wia_cancer_metabolism import WIACancerMetabolism

client = WIACancerMetabolism(
    api_key=os.environ['WIA_API_KEY'],
    base_url='https://api.wia.org/cancer-metabolism/v1'
)

# Get metabolic profile
profile = client.profiles.get('a1b2c3d4-e5f6-4a7b-8c9d-0e1f2a3b4c5d')

# Evaluate biomarker
evaluation = client.biomarkers.evaluate(
    biomarker_id='f6e5d4c3-b2a1-4d5e-6f7a-8b9c0d1e2f3a',
    profile_id='a1b2c3d4-e5f6-4a7b-8c9d-0e1f2a3b4c5d'
)
```

### R

```r
install.packages("wiacancermetabolism")

library(wiacancermetabolism)

client <- wia_client(
  api_key = Sys.getenv("WIA_API_KEY"),
  base_url = "https://api.wia.org/cancer-metabolism/v1"
)

# Get profiles
profiles <- wia_profiles_list(client, cancer_type = "C50.9")

# Pathway enrichment
enrichment <- wia_pathway_enrichment(
  client,
  pathway_id = "hsa00010",
  profile_id = "a1b2c3d4-e5f6-4a7b-8c9d-0e1f2a3b4c5d"
)
```

## Webhooks

Subscribe to real-time events via webhooks.

### Event Types

- `profile.created`: New metabolic profile created
- `profile.updated`: Profile data updated
- `profile.deleted`: Profile removed
- `biomarker.evaluated`: Biomarker evaluation completed
- `analysis.completed`: Batch analysis finished

### Webhook Registration

```http
POST /webhooks
Content-Type: application/json
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...

{
  "url": "https://yourapp.com/webhooks/wia-cancer-metabolism",
  "events": ["profile.created", "biomarker.evaluated"],
  "secret": "your-webhook-secret",
  "active": true
}
```

### Webhook Payload

```json
{
  "id": "evt_1234567890",
  "type": "profile.created",
  "data": {
    "profileId": "a1b2c3d4-e5f6-4a7b-8c9d-0e1f2a3b4c5d",
    "patientId": "PATIENT-BC-00125",
    "cancerType": "C50.9"
  },
  "timestamp": "2025-03-20T14:30:00Z",
  "signature": "sha256=..."
}
```

### Signature Verification

```javascript
const crypto = require('crypto');

function verifyWebhook(payload, signature, secret) {
  const hmac = crypto.createHmac('sha256', secret);
  const digest = 'sha256=' + hmac.update(payload).digest('hex');
  return crypto.timingSafeEqual(Buffer.from(signature), Buffer.from(digest));
}
```

## Batch Operations

Process multiple operations in a single request.

### Batch Request

```http
POST /batch
Content-Type: application/json
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...

{
  "operations": [
    {
      "id": "op1",
      "method": "GET",
      "path": "/profiles/a1b2c3d4-e5f6-4a7b-8c9d-0e1f2a3b4c5d"
    },
    {
      "id": "op2",
      "method": "GET",
      "path": "/biomarkers/f6e5d4c3-b2a1-4d5e-6f7a-8b9c0d1e2f3a"
    },
    {
      "id": "op3",
      "method": "POST",
      "path": "/biomarkers/f6e5d4c3-b2a1-4d5e-6f7a-8b9c0d1e2f3a/evaluate",
      "body": {
        "profileId": "a1b2c3d4-e5f6-4a7b-8c9d-0e1f2a3b4c5d"
      }
    }
  ]
}
```

### Batch Response

```json
{
  "results": [
    {
      "id": "op1",
      "status": 200,
      "body": { /* profile data */ }
    },
    {
      "id": "op2",
      "status": 200,
      "body": { /* biomarker data */ }
    },
    {
      "id": "op3",
      "status": 200,
      "body": { /* evaluation result */ }
    }
  ]
}
```

---
弘益人間 (홍익인간) - Benefit All Humanity
© 2025 WIA Standards | MIT License
