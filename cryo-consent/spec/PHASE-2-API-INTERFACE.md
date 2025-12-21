# WIA Cryo-Consent API Interface Standard
## Phase 2 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #06B6D4 (Cyan)

---

## Table of Contents

1. [Overview](#overview)
2. [API Architecture](#api-architecture)
3. [Authentication & Authorization](#authentication--authorization)
4. [Endpoints](#endpoints)
5. [Request/Response Formats](#requestresponse-formats)
6. [Error Handling](#error-handling)
7. [Rate Limiting](#rate-limiting)
8. [Code Examples](#code-examples)
9. [Version History](#version-history)

---

## Overview

### 1.1 Purpose

The WIA Cryo-Consent API Interface Standard defines RESTful API endpoints for managing legal consent records throughout their lifecycle. This standard enables secure, compliant, and interoperable consent management across healthcare facilities, legal systems, and cryonics organizations.

**Core Objectives**:
- Provide standardized REST API for consent lifecycle management
- Enable secure consent creation, modification, and revocation
- Support multi-jurisdiction consent validation and compliance
- Facilitate proxy and guardian consent delegation workflows
- Ensure cryptographic verification and audit trail integrity
- Integrate with legal, medical, and blockchain systems

### 1.2 Design Principles

1. **RESTful Architecture**: Standard HTTP methods and status codes
2. **Security First**: OAuth 2.0, JWT tokens, and end-to-end encryption
3. **Idempotency**: Safe retry mechanisms for critical operations
4. **Versioning**: API version negotiation via headers
5. **Pagination**: Cursor-based pagination for large result sets
6. **Filtering**: Rich query parameters for data retrieval

### 1.3 API Versioning

| Version | Base URL | Status |
|---------|----------|--------|
| v1 | `https://api.wia.live/cryo-consent/v1` | Current |

---

## API Architecture

### 2.1 Architecture Overview

```
┌─────────────┐
│   Clients   │
│ (Web, CLI)  │
└──────┬──────┘
       │
       │ HTTPS/REST
       │
┌──────▼──────────────────────────────┐
│     API Gateway                      │
│  - Authentication                    │
│  - Rate Limiting                     │
│  - Request Validation               │
└──────┬──────────────────────────────┘
       │
       │
┌──────▼──────────────────────────────┐
│  Consent Management Service          │
│  - Consent CRUD Operations          │
│  - Validation & Verification        │
│  - Event Publishing                 │
└──────┬──────────────────────────────┘
       │
       │
┌──────▼──────────────────────────────┐
│   Data Layer                         │
│  - PostgreSQL (Primary)             │
│  - Blockchain (Audit)               │
│  - Document Storage                 │
└─────────────────────────────────────┘
```

### 2.2 Service Capabilities

| Service | Description | SLA |
|---------|-------------|-----|
| Consent Management | CRUD operations for consent records | 99.9% |
| Validation Service | Legal and business rule validation | 99.95% |
| Notification Service | Real-time consent change notifications | 99.5% |
| Audit Service | Immutable audit trail management | 99.99% |
| Integration Service | Third-party system integration | 99.5% |

### 2.3 Data Flow

| Operation | Flow | Validation Points |
|-----------|------|-------------------|
| Create Consent | Client → API → Validation → Storage → Blockchain | Identity, Legal, Business Rules |
| Modify Consent | Client → API → Verification → New Version → Notification | Authority, Versioning, Legal |
| Revoke Consent | Client → API → Finality Check → Archive → Notification | Authority, Irreversibility |
| Query Consent | Client → API → Authorization → Fetch → Response | Access Rights, Privacy |

---

## Authentication & Authorization

### 3.1 Authentication Methods

| Method | Use Case | Description |
|--------|----------|-------------|
| OAuth 2.0 | Web Applications | Standard OAuth 2.0 flow with PKCE |
| JWT Bearer Token | API Clients | Short-lived access tokens |
| API Key | Service-to-Service | Long-lived credentials for trusted services |
| mTLS | High Security | Mutual TLS for critical operations |

### 3.2 Authorization Scopes

| Scope | Description | Access Level |
|-------|-------------|--------------|
| `consent:read` | Read consent records | View consent details |
| `consent:write` | Create and modify consent | Full CRUD except delete |
| `consent:revoke` | Revoke consent | Permanent revocation |
| `consent:admin` | Administrative operations | All operations including audit |
| `proxy:manage` | Manage proxy delegations | Add/remove proxies |
| `legal:verify` | Legal verification | Notarization and witnessing |

### 3.3 Authentication Headers

```http
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
X-API-Key: wia_live_sk_1234567890abcdef
X-Request-ID: uuid-v4-string
X-API-Version: 1.0.0
```

---

## Endpoints

### 4.1 Consent Management Endpoints

#### 4.1.1 Create Consent Record

```http
POST /api/v1/consent
```

Creates a new consent record with full legal validation.

**Request Body:**
```json
{
  "grantor": {
    "id": "PERSON-001",
    "identityVerification": {
      "method": "government_id",
      "documentId": "DL-CA-D1234567"
    }
  },
  "consent": {
    "scope": {
      "preservation": {
        "authorized": true,
        "types": ["whole_body"]
      },
      "research": {
        "authorized": true,
        "categories": ["medical", "scientific"]
      },
      "revival": {
        "authorized": true,
        "minimumViabilityThreshold": 0.7
      }
    },
    "jurisdiction": ["US-CA"]
  },
  "legal": {
    "witnesses": [
      {
        "name": "Jane Doe",
        "signature": "0x1234abcd..."
      }
    ],
    "jurisdiction": "US-CA"
  }
}
```

**Response (201 Created):**
```json
{
  "consentId": "550e8400-e29b-41d4-a716-446655440001",
  "status": "active",
  "version": 1,
  "createdAt": "2025-01-15T10:30:00Z",
  "effectiveDate": "2025-01-15T00:00:00Z",
  "meta": {
    "hash": "sha256:a5b9c3d4e5f6...",
    "signature": "0xabcdef123456...",
    "blockchainAnchor": "ethereum:0x1234567890abcdef"
  }
}
```

**Validation Rules:**
- Grantor must be verified
- At least one witness or notarization required
- Jurisdiction-specific rules applied
- All required scope fields present

---

#### 4.1.2 Get Consent Record

```http
GET /api/v1/consent/{consentId}
```

Retrieves a specific consent record by ID.

**Path Parameters:**
- `consentId` (string, required): UUID of the consent record

**Query Parameters:**
- `version` (integer, optional): Specific version number (default: latest)
- `include` (string, optional): Include related data (`proxies`, `history`, `audit`)

**Response (200 OK):**
```json
{
  "consentId": "550e8400-e29b-41d4-a716-446655440001",
  "version": 1,
  "messageType": "consent_record",
  "timestamp": {
    "created": "2025-01-15T10:30:00Z",
    "modified": "2025-01-15T10:30:00Z",
    "effectiveDate": "2025-01-15T00:00:00Z"
  },
  "grantor": {
    "id": "PERSON-001"
  },
  "consent": {
    "status": "active",
    "scope": { /* ... */ }
  },
  "legal": { /* ... */ },
  "meta": { /* ... */ }
}
```

---

#### 4.1.3 Update Consent Record

```http
PUT /api/v1/consent/{consentId}
```

Creates a new version of an existing consent record. Original consent is preserved for audit trail.

**Request Body:**
```json
{
  "modifications": {
    "consent": {
      "scope": {
        "research": {
          "authorized": true,
          "categories": ["medical", "scientific", "commercial"]
        }
      }
    }
  },
  "verification": {
    "method": "video",
    "verifierId": "LAWYER-001"
  },
  "legal": {
    "witnesses": [
      {
        "name": "Attorney Sarah Lee",
        "signature": "0x9999aaaa..."
      }
    ]
  }
}
```

**Response (200 OK):**
```json
{
  "consentId": "550e8400-e29b-41d4-a716-446655440001",
  "version": 2,
  "previousVersion": 1,
  "status": "active",
  "modifiedAt": "2025-02-20T14:00:00Z",
  "meta": {
    "hash": "sha256:b6c8d5e7f8...",
    "previousHash": "sha256:a5b9c3d4e5f6..."
  }
}
```

---

#### 4.1.4 Revoke Consent

```http
POST /api/v1/consent/{consentId}/revoke
```

Permanently revokes consent. This operation is irreversible.

**Request Body:**
```json
{
  "reason": "Changed decision - no longer wish to be preserved",
  "verification": {
    "method": "notary",
    "verifierId": "NOTARY-CA-12345"
  },
  "legal": {
    "witnesses": [
      {
        "name": "Michael Chen",
        "signature": "0xbbbbcccc..."
      }
    ],
    "notarization": {
      "notaryId": "NOTARY-CA-12345",
      "sealNumber": "SEAL-2025-045"
    }
  }
}
```

**Response (200 OK):**
```json
{
  "consentId": "550e8400-e29b-41d4-a716-446655440001",
  "status": "revoked",
  "revokedAt": "2025-03-10T09:00:00Z",
  "version": 3,
  "finalHash": "sha256:c7d9e6f8g9..."
}
```

---

#### 4.1.5 List Consent Records

```http
GET /api/v1/consent
```

Retrieves a paginated list of consent records with filtering.

**Query Parameters:**
- `grantorId` (string, optional): Filter by grantor
- `status` (string, optional): Filter by status (`active`, `revoked`, etc.)
- `jurisdiction` (string, optional): Filter by jurisdiction
- `fromDate` (string, optional): ISO 8601 date
- `toDate` (string, optional): ISO 8601 date
- `limit` (integer, optional): Page size (default: 50, max: 100)
- `cursor` (string, optional): Pagination cursor

**Response (200 OK):**
```json
{
  "data": [
    {
      "consentId": "550e8400-e29b-41d4-a716-446655440001",
      "status": "active",
      "createdAt": "2025-01-15T10:30:00Z",
      "grantor": { "id": "PERSON-001" }
    }
  ],
  "pagination": {
    "nextCursor": "eyJpZCI6MTIzfQ==",
    "hasMore": true,
    "total": 150
  }
}
```

---

### 4.2 Proxy Management Endpoints

#### 4.2.1 Add Proxy

```http
POST /api/v1/consent/{consentId}/proxy
```

Delegates consent management authority to a proxy.

**Request Body:**
```json
{
  "proxy": {
    "id": "PROXY-001",
    "type": "healthcare_proxy",
    "authorizedActions": ["modify_consent", "authorize_revival"],
    "effectiveDate": "2025-01-15T00:00:00Z"
  },
  "verification": {
    "method": "notary",
    "verifierId": "NOTARY-001"
  }
}
```

**Response (201 Created):**
```json
{
  "proxyId": "PROXY-001",
  "consentId": "550e8400-e29b-41d4-a716-446655440001",
  "status": "active",
  "effectiveDate": "2025-01-15T00:00:00Z",
  "authorizedActions": ["modify_consent", "authorize_revival"]
}
```

---

#### 4.2.2 List Proxies

```http
GET /api/v1/consent/{consentId}/proxies
```

Retrieves all proxies for a consent record.

**Response (200 OK):**
```json
{
  "proxies": [
    {
      "proxyId": "PROXY-001",
      "type": "healthcare_proxy",
      "status": "active",
      "authorizedActions": ["modify_consent", "authorize_revival"],
      "effectiveDate": "2025-01-15T00:00:00Z"
    }
  ]
}
```

---

#### 4.2.3 Revoke Proxy

```http
DELETE /api/v1/consent/{consentId}/proxy/{proxyId}
```

Removes proxy authorization.

**Response (200 OK):**
```json
{
  "proxyId": "PROXY-001",
  "status": "revoked",
  "revokedAt": "2025-04-01T12:00:00Z"
}
```

---

### 4.3 Validation & Verification Endpoints

#### 4.3.1 Validate Consent

```http
POST /api/v1/consent/validate
```

Validates consent data against legal and business rules without creating a record.

**Request Body:**
```json
{
  "consent": {
    /* Full consent object */
  },
  "jurisdiction": "US-CA"
}
```

**Response (200 OK):**
```json
{
  "valid": true,
  "warnings": [
    "No notarization provided - required in California for whole_body"
  ],
  "errors": [],
  "compliance": {
    "US-CA": "valid",
    "GDPR": "compliant",
    "HIPAA": "compliant"
  }
}
```

---

#### 4.3.2 Verify Consent Signature

```http
POST /api/v1/consent/{consentId}/verify
```

Verifies cryptographic signatures and blockchain anchors.

**Response (200 OK):**
```json
{
  "consentId": "550e8400-e29b-41d4-a716-446655440001",
  "verified": true,
  "checks": {
    "hashIntegrity": true,
    "signatureValid": true,
    "blockchainAnchored": true,
    "witnessSignatures": true
  },
  "blockchainProof": {
    "network": "ethereum",
    "transactionHash": "0x1234567890abcdef",
    "blockNumber": 18234567,
    "timestamp": "2025-01-15T10:35:00Z"
  }
}
```

---

#### 4.3.3 Check Consent Status

```http
GET /api/v1/consent/{consentId}/status
```

Quick status check without full record retrieval.

**Response (200 OK):**
```json
{
  "consentId": "550e8400-e29b-41d4-a716-446655440001",
  "status": "active",
  "effectiveDate": "2025-01-15T00:00:00Z",
  "expirationDate": null,
  "version": 2,
  "lastModified": "2025-02-20T14:00:00Z"
}
```

---

### 4.4 History & Audit Endpoints

#### 4.4.1 Get Consent History

```http
GET /api/v1/consent/{consentId}/history
```

Retrieves complete version history.

**Response (200 OK):**
```json
{
  "consentId": "550e8400-e29b-41d4-a716-446655440001",
  "versions": [
    {
      "version": 1,
      "timestamp": "2025-01-15T10:30:00Z",
      "messageType": "consent_record",
      "status": "superseded",
      "hash": "sha256:a5b9c3d4e5f6..."
    },
    {
      "version": 2,
      "timestamp": "2025-02-20T14:00:00Z",
      "messageType": "consent_modification",
      "status": "active",
      "hash": "sha256:b6c8d5e7f8...",
      "previousHash": "sha256:a5b9c3d4e5f6..."
    }
  ]
}
```

---

#### 4.4.2 Get Audit Trail

```http
GET /api/v1/consent/{consentId}/audit
```

Retrieves complete audit log.

**Response (200 OK):**
```json
{
  "consentId": "550e8400-e29b-41d4-a716-446655440001",
  "events": [
    {
      "eventId": "EVT-001",
      "timestamp": "2025-01-15T10:30:00Z",
      "eventType": "consent_created",
      "actor": "PERSON-001",
      "details": {
        "version": 1,
        "status": "active"
      }
    },
    {
      "eventId": "EVT-002",
      "timestamp": "2025-02-20T14:00:00Z",
      "eventType": "consent_modified",
      "actor": "PERSON-001",
      "details": {
        "version": 2,
        "modifications": ["research.categories"]
      }
    }
  ]
}
```

---

### 4.5 Jurisdiction & Compliance Endpoints

#### 4.5.1 Get Jurisdiction Requirements

```http
GET /api/v1/jurisdiction/{jurisdictionCode}
```

Retrieves legal requirements for a specific jurisdiction.

**Response (200 OK):**
```json
{
  "jurisdiction": "US-CA",
  "name": "California, United States",
  "requirements": {
    "minimumAge": 18,
    "witnessRequired": true,
    "minimumWitnesses": 2,
    "notarizationRequired": true,
    "waitingPeriod": null,
    "mentalCapacityAssessment": "recommended"
  },
  "governingLaws": [
    "California Health and Safety Code Section 7100-7117",
    "Uniform Anatomical Gift Act"
  ],
  "compliance": ["HIPAA", "California Consumer Privacy Act"]
}
```

---

#### 4.5.2 Check Multi-Jurisdiction Compliance

```http
POST /api/v1/consent/compliance-check
```

Validates consent against multiple jurisdictions.

**Request Body:**
```json
{
  "consentId": "550e8400-e29b-41d4-a716-446655440001",
  "jurisdictions": ["US-CA", "US-NY", "EU-DE"]
}
```

**Response (200 OK):**
```json
{
  "compliant": false,
  "jurisdictions": {
    "US-CA": {
      "compliant": true,
      "issues": []
    },
    "US-NY": {
      "compliant": true,
      "issues": []
    },
    "EU-DE": {
      "compliant": false,
      "issues": [
        "Missing GDPR data protection impact assessment",
        "No EU data representative designated"
      ]
    }
  }
}
```

---

## Request/Response Formats

### 5.1 Standard Response Structure

| Field | Type | Description |
|-------|------|-------------|
| `data` | object/array | Response payload |
| `meta` | object | Metadata (pagination, timestamps) |
| `errors` | array | Error details (if applicable) |

### 5.2 Error Response Structure

```json
{
  "error": {
    "code": "VALIDATION_ERROR",
    "message": "Consent validation failed",
    "details": [
      {
        "field": "legal.witnesses",
        "issue": "Minimum 2 witnesses required in US-NY"
      }
    ],
    "requestId": "req_1234567890",
    "timestamp": "2025-01-15T10:30:00Z"
  }
}
```

### 5.3 HTTP Status Codes

| Code | Meaning | Usage |
|------|---------|-------|
| 200 | OK | Successful GET, PUT, POST (modification) |
| 201 | Created | Successful resource creation |
| 400 | Bad Request | Invalid request format or parameters |
| 401 | Unauthorized | Missing or invalid authentication |
| 403 | Forbidden | Insufficient permissions |
| 404 | Not Found | Resource does not exist |
| 409 | Conflict | Resource state conflict |
| 422 | Unprocessable Entity | Valid format but business logic failure |
| 429 | Too Many Requests | Rate limit exceeded |
| 500 | Internal Server Error | Server-side error |
| 503 | Service Unavailable | Temporary service disruption |

---

## Error Handling

### 6.1 Error Categories

| Category | HTTP Code | Description |
|----------|-----------|-------------|
| Validation Error | 400, 422 | Data validation failure |
| Authentication Error | 401 | Invalid credentials |
| Authorization Error | 403 | Insufficient permissions |
| Not Found Error | 404 | Resource not found |
| Conflict Error | 409 | State conflict (e.g., already revoked) |
| Rate Limit Error | 429 | Too many requests |
| Server Error | 500, 503 | Internal system error |

### 6.2 Error Codes

| Code | Description | Resolution |
|------|-------------|------------|
| `INVALID_CONSENT_DATA` | Consent data format invalid | Check JSON schema |
| `INSUFFICIENT_ATTESTATION` | Missing witnesses/notarization | Add required attestations |
| `INVALID_JURISDICTION` | Jurisdiction not supported | Use valid jurisdiction code |
| `CONSENT_ALREADY_REVOKED` | Cannot modify revoked consent | Create new consent |
| `UNAUTHORIZED_PROXY` | Proxy lacks authorization | Verify proxy permissions |
| `SIGNATURE_VERIFICATION_FAILED` | Invalid cryptographic signature | Re-sign with valid key |
| `BLOCKCHAIN_ANCHOR_FAILED` | Blockchain anchoring error | Retry operation |

---

## Rate Limiting

### 7.1 Rate Limit Tiers

| Tier | Requests/Hour | Burst |
|------|---------------|-------|
| Free | 100 | 10 |
| Standard | 1,000 | 50 |
| Professional | 10,000 | 200 |
| Enterprise | Custom | Custom |

### 7.2 Rate Limit Headers

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 999
X-RateLimit-Reset: 1642262400
Retry-After: 3600
```

---

## Code Examples

### 8.1 TypeScript - Create Consent

```typescript
import axios from 'axios';

interface ConsentRequest {
  grantor: {
    id: string;
    identityVerification: {
      method: string;
      documentId: string;
    };
  };
  consent: {
    scope: {
      preservation: {
        authorized: boolean;
        types: string[];
      };
      research: {
        authorized: boolean;
        categories: string[];
      };
      revival: {
        authorized: boolean;
        minimumViabilityThreshold: number;
      };
    };
    jurisdiction: string[];
  };
  legal: {
    witnesses: Array<{
      name: string;
      signature: string;
    }>;
    jurisdiction: string;
  };
}

async function createConsent(
  apiKey: string,
  consentData: ConsentRequest
): Promise<any> {
  try {
    const response = await axios.post(
      'https://api.wia.live/cryo-consent/v1/consent',
      consentData,
      {
        headers: {
          'Authorization': `Bearer ${apiKey}`,
          'Content-Type': 'application/json',
          'X-API-Version': '1.0.0'
        }
      }
    );

    console.log('Consent created:', response.data.consentId);
    console.log('Blockchain anchor:', response.data.meta.blockchainAnchor);

    return response.data;
  } catch (error) {
    if (axios.isAxiosError(error)) {
      console.error('API Error:', error.response?.data);
      throw new Error(`Failed to create consent: ${error.response?.data.error.message}`);
    }
    throw error;
  }
}

// Usage
const consentData: ConsentRequest = {
  grantor: {
    id: 'PERSON-001',
    identityVerification: {
      method: 'government_id',
      documentId: 'DL-CA-D1234567'
    }
  },
  consent: {
    scope: {
      preservation: {
        authorized: true,
        types: ['whole_body']
      },
      research: {
        authorized: true,
        categories: ['medical', 'scientific']
      },
      revival: {
        authorized: true,
        minimumViabilityThreshold: 0.7
      }
    },
    jurisdiction: ['US-CA']
  },
  legal: {
    witnesses: [
      {
        name: 'Jane Doe',
        signature: '0x1234abcd...'
      }
    ],
    jurisdiction: 'US-CA'
  }
};

createConsent('wia_live_sk_1234567890', consentData);
```

### 8.2 Python - Get Consent

```python
import requests
from typing import Dict, Optional
from datetime import datetime

class ConsentClient:
    def __init__(self, api_key: str, base_url: str = "https://api.wia.live/cryo-consent/v1"):
        self.api_key = api_key
        self.base_url = base_url
        self.session = requests.Session()
        self.session.headers.update({
            'Authorization': f'Bearer {api_key}',
            'Content-Type': 'application/json',
            'X-API-Version': '1.0.0'
        })

    def get_consent(
        self,
        consent_id: str,
        version: Optional[int] = None,
        include: Optional[str] = None
    ) -> Dict:
        """
        Retrieve a consent record by ID.

        Args:
            consent_id: UUID of the consent record
            version: Specific version number (default: latest)
            include: Include related data (proxies, history, audit)

        Returns:
            Dict containing consent record
        """
        url = f"{self.base_url}/consent/{consent_id}"
        params = {}

        if version:
            params['version'] = version
        if include:
            params['include'] = include

        try:
            response = self.session.get(url, params=params)
            response.raise_for_status()

            data = response.json()
            print(f"Retrieved consent: {data['consentId']}")
            print(f"Status: {data['consent']['status']}")
            print(f"Version: {data['version']}")

            return data

        except requests.exceptions.HTTPError as e:
            error_data = e.response.json()
            print(f"Error: {error_data['error']['message']}")
            raise

    def revoke_consent(
        self,
        consent_id: str,
        reason: str,
        verification: Dict
    ) -> Dict:
        """
        Revoke a consent record.

        Args:
            consent_id: UUID of the consent record
            reason: Reason for revocation
            verification: Verification details

        Returns:
            Dict containing revocation confirmation
        """
        url = f"{self.base_url}/consent/{consent_id}/revoke"
        payload = {
            'reason': reason,
            'verification': verification,
            'legal': {
                'witnesses': [
                    {
                        'name': 'Michael Chen',
                        'signature': '0xbbbbcccc...'
                    }
                ]
            }
        }

        try:
            response = self.session.post(url, json=payload)
            response.raise_for_status()

            data = response.json()
            print(f"Consent revoked: {data['consentId']}")
            print(f"Revoked at: {data['revokedAt']}")

            return data

        except requests.exceptions.HTTPError as e:
            error_data = e.response.json()
            print(f"Error: {error_data['error']['message']}")
            raise

# Usage
client = ConsentClient('wia_live_sk_1234567890')

# Get consent with history
consent = client.get_consent(
    '550e8400-e29b-41d4-a716-446655440001',
    include='history,proxies'
)

# Revoke consent
revocation = client.revoke_consent(
    '550e8400-e29b-41d4-a716-446655440001',
    reason='Changed decision',
    verification={
        'method': 'notary',
        'verifierId': 'NOTARY-CA-12345'
    }
)
```

### 8.3 TypeScript - Validate Before Create

```typescript
async function validateAndCreateConsent(
  apiKey: string,
  consentData: ConsentRequest
): Promise<any> {
  // Step 1: Validate
  const validationResponse = await axios.post(
    'https://api.wia.live/cryo-consent/v1/consent/validate',
    {
      consent: consentData,
      jurisdiction: consentData.consent.jurisdiction[0]
    },
    {
      headers: {
        'Authorization': `Bearer ${apiKey}`,
        'Content-Type': 'application/json'
      }
    }
  );

  if (!validationResponse.data.valid) {
    console.error('Validation errors:', validationResponse.data.errors);
    throw new Error('Consent validation failed');
  }

  if (validationResponse.data.warnings.length > 0) {
    console.warn('Warnings:', validationResponse.data.warnings);
  }

  // Step 2: Create
  return await createConsent(apiKey, consentData);
}
```

### 8.4 Python - Batch Query

```python
def list_active_consents(
    client: ConsentClient,
    jurisdiction: str = 'US-CA'
) -> list:
    """
    List all active consents for a jurisdiction.
    """
    url = f"{client.base_url}/consent"
    all_consents = []
    cursor = None

    while True:
        params = {
            'status': 'active',
            'jurisdiction': jurisdiction,
            'limit': 100
        }
        if cursor:
            params['cursor'] = cursor

        response = client.session.get(url, params=params)
        response.raise_for_status()

        data = response.json()
        all_consents.extend(data['data'])

        if not data['pagination']['hasMore']:
            break

        cursor = data['pagination']['nextCursor']

    print(f"Retrieved {len(all_consents)} active consents")
    return all_consents
```

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-01 | Initial release |

---

## Appendix A: Related Standards

| Standard | Relationship |
|----------|--------------|
| WIA Cryo-Consent Phase 1 | Data format specification |
| WIA Cryo-Consent Phase 3 | Real-time notification protocol |
| OAuth 2.0 RFC 6749 | Authentication framework |
| OpenAPI 3.0 | API documentation standard |

---

<div align="center">

**WIA Cryo-Consent API Interface Standard v1.0.0**

**弘益人間 (홍익인간)** - Benefit All Humanity

---

**© 2025 WIA Standards Committee**

**MIT License**

</div>
