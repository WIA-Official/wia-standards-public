## Phase 2 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-12-18
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #64748B (Slate)

---

## Table of Contents

1. [Overview](#overview)
2. [Authentication & Authorization](#authentication--authorization)
3. [API Endpoints](#api-endpoints)
4. [Request/Response Formats](#requestresponse-formats)
5. [Error Handling](#error-handling)
6. [Rate Limiting](#rate-limiting)
7. [Code Examples](#code-examples)
8. [Security](#security)
9. [Version History](#version-history)

---

## Overview

### 1.1 Purpose

The WIA Digital Erasure API Interface Standard defines RESTful API endpoints, request/response formats, authentication mechanisms, and integration patterns for digital erasure services, enabling standardized interaction between executors, platforms, and verification services.

**Core Objectives**:
- Standardize API interfaces across all digital erasure operations
- Provide secure authentication for executor verification
- Enable real-time deletion status monitoring
- Support batch operations for multi-platform erasure
- Facilitate compliance reporting and audit trail access

### 1.2 API Architecture

| Component | Description |
|-----------|-------------|
| **Erasure Request API** | Submit and manage deletion requests |
| **Account Discovery API** | Scan and inventory digital footprints |
| **Verification API** | Validate deletion completion |
| **Compliance API** | Generate GDPR/legal compliance reports |
| **Webhook API** | Real-time status notifications |
| **Platform Integration API** | Interface with service providers |

### 1.3 Base URL Structure

```
Production:  https://api.wia.live/digital-erasure/v1
Staging:     https://api-staging.wia.live/digital-erasure/v1
Sandbox:     https://api-sandbox.wia.live/digital-erasure/v1
```

---

## Authentication & Authorization

### 2.1 Authentication Methods

| Method | Use Case | Security Level |
|--------|----------|----------------|
| **OAuth 2.0** | Executor authentication | High |
| **API Key** | Service-to-service | Medium |
| **JWT** | Session management | High |
| **mTLS** | Platform integration | Very High |
| **Digital Certificate** | Legal document verification | Very High |

### 2.2 Authentication Flow

#### OAuth 2.0 Flow

```http
POST /auth/token
Content-Type: application/json

{
  "grant_type": "authorization_code",
  "client_id": "executor-app-001",
  "client_secret": "secret_key",
  "code": "auth_code_from_login",
  "redirect_uri": "https://executor-app.com/callback"
}
```

**Response:**

```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "refresh_token": "refresh_token_string",
  "scope": "erasure:create erasure:read verification:read"
}
```

### 2.3 Authorization Scopes

| Scope | Description | Operations |
|-------|-------------|------------|
| `erasure:create` | Create erasure requests | POST /erasure-requests |
| `erasure:read` | Read erasure status | GET /erasure-requests |
| `erasure:update` | Update requests | PATCH /erasure-requests |
| `inventory:read` | View digital footprint | GET /inventory |
| `inventory:scan` | Scan for accounts | POST /inventory/scan |
| `verification:read` | View verification proofs | GET /verifications |
| `compliance:read` | Access compliance reports | GET /compliance/reports |
| `admin:all` | Full administrative access | All operations |

### 2.4 API Key Management

```http
GET /auth/api-keys
Authorization: Bearer {access_token}
```

**Response:**

```json
{
  "api_keys": [
    {
      "key_id": "key_001",
      "name": "Production Key",
      "key_prefix": "wia_live_",
      "created": "2025-12-18T10:00:00Z",
      "last_used": "2025-12-18T15:30:00Z",
      "scopes": ["erasure:create", "erasure:read"],
      "rate_limit": 1000,
      "active": true
    }
  ]
}
```

---

## API Endpoints

### 3.1 Erasure Request Endpoints

#### 3.1.1 Create Erasure Request

```http
POST /erasure-requests
Authorization: Bearer {access_token}
Content-Type: application/json
```

**Request Body:**

```json
{
  "decedent": {
    "deathCertificateId": "DC-KR-2025-001",
    "dateOfDeath": "2025-12-01T00:00:00Z"
  },
  "executor": {
    "name": "Jane Doe",
    "email": "executor@example.com",
    "authenticationDocument": "base64_encoded_document"
  },
  "requestType": "complete_erasure",
  "legalBasis": "gdpr_article_17",
  "targetAccounts": [
    {
      "platform": "facebook",
      "accountIdentifier": "user@example.com",
      "requestedAction": "permanent_deletion"
    },
    {
      "platform": "google",
      "accountIdentifier": "user@gmail.com",
      "requestedAction": "permanent_deletion",
      "downloadDataFirst": true
    }
  ],
  "deletionMethod": {
    "algorithm": "crypto_shred",
    "passes": 7,
    "verificationRequired": true
  }
}
```

**Response (201 Created):**

```json
{
  "requestId": "ER-2025-12-18-001",
  "status": "authentication_pending",
  "created": "2025-12-18T11:00:00Z",
  "estimatedCompletion": "2026-01-18T00:00:00Z",
  "accountsQueued": 2,
  "nextSteps": [
    "Verify executor authentication documents",
    "Await 30-day grace period",
    "Begin deletion process"
  ],
  "links": {
    "self": "/erasure-requests/ER-2025-12-18-001",
    "status": "/erasure-requests/ER-2025-12-18-001/status",
    "cancel": "/erasure-requests/ER-2025-12-18-001/cancel"
  }
}
```

#### 3.1.2 Get Erasure Request Status

```http
GET /erasure-requests/{requestId}
Authorization: Bearer {access_token}
```

**Response (200 OK):**

```json
{
  "requestId": "ER-2025-12-18-001",
  "status": "in_progress",
  "created": "2025-12-18T11:00:00Z",
  "lastUpdated": "2025-12-20T15:30:00Z",
  "completionPercentage": 45.0,
  "accountsTotal": 2,
  "accountsCompleted": 0,
  "accountsInProgress": 1,
  "accountsPending": 1,
  "accountsFailed": 0,
  "accounts": [
    {
      "accountId": "ACC-001",
      "platform": "facebook",
      "status": "in_progress",
      "deletionStarted": "2025-12-20T14:00:00Z",
      "estimatedCompletion": "2025-12-20T16:00:00Z"
    },
    {
      "accountId": "ACC-002",
      "platform": "google",
      "status": "pending",
      "scheduledStart": "2025-12-20T16:00:00Z"
    }
  ],
  "timeline": [
    {
      "timestamp": "2025-12-18T11:00:00Z",
      "event": "request_created",
      "description": "Erasure request submitted"
    },
    {
      "timestamp": "2025-12-18T14:00:00Z",
      "event": "executor_verified",
      "description": "Executor authentication completed"
    },
    {
      "timestamp": "2025-12-20T14:00:00Z",
      "event": "deletion_started",
      "description": "Facebook account deletion initiated"
    }
  ]
}
```

#### 3.1.3 List Erasure Requests

```http
GET /erasure-requests?status={status}&page={page}&limit={limit}
Authorization: Bearer {access_token}
```

**Query Parameters:**

| Parameter | Type | Required | Description | Default |
|-----------|------|----------|-------------|---------|
| `status` | string | No | Filter by status | all |
| `page` | integer | No | Page number | 1 |
| `limit` | integer | No | Results per page | 20 |
| `sort` | string | No | Sort field | created |
| `order` | string | No | Sort order (asc/desc) | desc |

**Response (200 OK):**

```json
{
  "requests": [
    {
      "requestId": "ER-2025-12-18-001",
      "status": "in_progress",
      "created": "2025-12-18T11:00:00Z",
      "completionPercentage": 45.0
    }
  ],
  "pagination": {
    "page": 1,
    "limit": 20,
    "total": 1,
    "totalPages": 1
  }
}
```

#### 3.1.4 Cancel Erasure Request

```http
DELETE /erasure-requests/{requestId}
Authorization: Bearer {access_token}
Content-Type: application/json
```

**Request Body:**

```json
{
  "reason": "Family request to preserve account",
  "authorizedBy": "Jane Doe"
}
```

**Response (200 OK):**

```json
{
  "requestId": "ER-2025-12-18-001",
  "status": "cancelled",
  "cancelledAt": "2025-12-19T10:00:00Z",
  "reason": "Family request to preserve account",
  "accountsReverted": 0,
  "accountsPreserved": 2
}
```

### 3.2 Account Discovery Endpoints

#### 3.2.1 Scan Digital Footprint

```http
POST /inventory/scan
Authorization: Bearer {access_token}
Content-Type: application/json
```

**Request Body:**

```json
{
  "decedent": {
    "email": "user@example.com",
    "alternateEmails": ["user2@example.com", "user3@example.com"],
    "phone": "+821012345678",
    "fullName": "John Doe",
    "dateOfBirth": "1980-01-15"
  },
  "scanOptions": {
    "deepScan": true,
    "includeInactive": true,
    "platformCategories": ["social_media", "email_messaging", "cloud_storage", "financial_services"],
    "timeout": 300
  }
}
```

**Response (202 Accepted):**

```json
{
  "scanId": "SCAN-2025-12-18-001",
  "status": "in_progress",
  "started": "2025-12-18T10:00:00Z",
  "estimatedCompletion": "2025-12-18T10:05:00Z",
  "links": {
    "status": "/inventory/scan/SCAN-2025-12-18-001",
    "results": "/inventory/scan/SCAN-2025-12-18-001/results"
  }
}
```

#### 3.2.2 Get Scan Results

```http
GET /inventory/scan/{scanId}/results
Authorization: Bearer {access_token}
```

**Response (200 OK):**

```json
{
  "scanId": "SCAN-2025-12-18-001",
  "status": "completed",
  "started": "2025-12-18T10:00:00Z",
  "completed": "2025-12-18T10:04:32Z",
  "totalAccounts": 47,
  "accountsFound": [
    {
      "platform": "facebook",
      "platformType": "social_media",
      "accountIdentifier": "user@example.com",
      "confidence": 0.98,
      "lastActivity": "2025-11-30T15:22:00Z",
      "dataVolume": {
        "estimatedSizeGB": 12.5
      }
    },
    {
      "platform": "google",
      "platformType": "email_messaging",
      "accountIdentifier": "user@gmail.com",
      "confidence": 1.0,
      "services": ["gmail", "drive", "photos", "youtube"],
      "dataVolume": {
        "estimatedSizeGB": 189.3
      }
    }
  ],
  "categorySummary": {
    "social_media": 8,
    "email_messaging": 5,
    "cloud_storage": 6,
    "financial_services": 12,
    "subscriptions": 10,
    "professional_networks": 4,
    "other": 2
  }
}
```

#### 3.2.3 Get Account Inventory

```http
GET /inventory
Authorization: Bearer {access_token}
```

**Response (200 OK):**

```json
{
  "totalAccounts": 47,
  "lastUpdated": "2025-12-18T10:04:32Z",
  "accounts": [
    {
      "accountId": "ACC-001",
      "platform": "facebook",
      "platformType": "social_media",
      "status": "active",
      "lastActivity": "2025-11-30T15:22:00Z"
    }
  ]
}
```

### 3.3 Verification Endpoints

#### 3.3.1 Get Verification Proof

```http
GET /verifications/{accountId}
Authorization: Bearer {access_token}
```

**Response (200 OK):**

```json
{
  "accountId": "ACC-001",
  "platform": "facebook",
  "verificationStatus": "confirmed_deleted",
  "deletionTimestamp": "2025-12-20T15:00:00Z",
  "verificationMethod": "cryptographic_hash",
  "preDeleteionHash": "sha256:a1b2c3d4e5f6a7b8c9d0e1f2...",
  "postDeletionHash": "sha256:0000000000000000000000000...",
  "platformConfirmation": {
    "confirmationId": "FB-DEL-2025-12-20-001",
    "deletionReceipt": "https://facebook.com/deletion/receipt/ABC123",
    "permanent": true
  },
  "certificate": {
    "url": "/verifications/ACC-001/certificate",
    "format": "pdf",
    "signed": true,
    "signature": "digital-signature-hash"
  }
}
```

#### 3.3.2 Download Verification Certificate

```http
GET /verifications/{accountId}/certificate
Authorization: Bearer {access_token}
Accept: application/pdf
```

**Response (200 OK):**

```
Content-Type: application/pdf
Content-Disposition: attachment; filename="verification-ACC-001.pdf"

[PDF binary data]
```

#### 3.3.3 Batch Verify Deletions

```http
POST /verifications/batch
Authorization: Bearer {access_token}
Content-Type: application/json
```

**Request Body:**

```json
{
  "accountIds": ["ACC-001", "ACC-002", "ACC-003"]
}
```

**Response (200 OK):**

```json
{
  "verifications": [
    {
      "accountId": "ACC-001",
      "status": "confirmed_deleted",
      "verified": true
    },
    {
      "accountId": "ACC-002",
      "status": "in_progress",
      "verified": false
    },
    {
      "accountId": "ACC-003",
      "status": "confirmed_deleted",
      "verified": true
    }
  ],
  "summary": {
    "total": 3,
    "verified": 2,
    "pending": 1,
    "failed": 0
  }
}
```

### 3.4 Compliance Endpoints

#### 3.4.1 Generate Compliance Report

```http
POST /compliance/reports
Authorization: Bearer {access_token}
Content-Type: application/json
```

**Request Body:**

```json
{
  "requestId": "ER-2025-12-18-001",
  "reportType": "gdpr_article_17",
  "includeAuditTrail": true,
  "includeVerifications": true,
  "format": "pdf"
}
```

**Response (202 Accepted):**

```json
{
  "reportId": "REPORT-2025-12-20-001",
  "status": "generating",
  "estimatedCompletion": "2025-12-20T16:05:00Z",
  "links": {
    "status": "/compliance/reports/REPORT-2025-12-20-001",
    "download": "/compliance/reports/REPORT-2025-12-20-001/download"
  }
}
```

#### 3.4.2 Download Compliance Report

```http
GET /compliance/reports/{reportId}/download
Authorization: Bearer {access_token}
```

**Response (200 OK):**

```json
{
  "reportId": "REPORT-2025-12-20-001",
  "reportType": "gdpr_article_17",
  "generated": "2025-12-20T16:03:45Z",
  "requestId": "ER-2025-12-18-001",
  "summary": {
    "accountsProcessed": 47,
    "accountsDeleted": 45,
    "accountsFailed": 2,
    "gdprCompliant": true,
    "allVerified": true
  },
  "downloadUrl": "https://api.wia.live/reports/REPORT-2025-12-20-001.pdf",
  "expiresAt": "2025-12-27T16:03:45Z"
}
```

#### 3.4.3 Get Audit Trail

```http
GET /compliance/audit-trail/{requestId}
Authorization: Bearer {access_token}
```

**Response (200 OK):**

```json
{
  "requestId": "ER-2025-12-18-001",
  "auditTrail": [
    {
      "timestamp": "2025-12-18T11:00:00Z",
      "action": "request_created",
      "actor": "EXEC-2025-001",
      "actorType": "executor",
      "details": {
        "accountsRequested": 47
      },
      "ipAddress": "192.168.1.1",
      "hash": "sha256:audit-001..."
    },
    {
      "timestamp": "2025-12-18T14:00:00Z",
      "action": "executor_verified",
      "actor": "WIA-VERIFICATION-SERVICE",
      "actorType": "system",
      "details": {
        "verificationMethod": "government_id",
        "documentId": "GOVT-ID-001"
      },
      "hash": "sha256:audit-002..."
    },
    {
      "timestamp": "2025-12-20T15:00:00Z",
      "action": "account_deleted",
      "actor": "facebook",
      "actorType": "platform",
      "details": {
        "accountId": "ACC-001",
        "confirmationId": "FB-DEL-2025-12-20-001"
      },
      "hash": "sha256:audit-003...",
      "previousHash": "sha256:audit-002..."
    }
  ],
  "chainValid": true,
  "totalEvents": 127
}
```

### 3.5 Webhook Endpoints

#### 3.5.1 Register Webhook

```http
POST /webhooks
Authorization: Bearer {access_token}
Content-Type: application/json
```

**Request Body:**

```json
{
  "url": "https://executor-app.com/webhooks/erasure",
  "events": [
    "erasure.request.created",
    "erasure.request.completed",
    "account.deletion.started",
    "account.deletion.completed",
    "account.deletion.failed",
    "verification.completed"
  ],
  "secret": "webhook_secret_key",
  "active": true
}
```

**Response (201 Created):**

```json
{
  "webhookId": "WEBHOOK-001",
  "url": "https://executor-app.com/webhooks/erasure",
  "events": ["erasure.request.created", "erasure.request.completed"],
  "created": "2025-12-18T11:30:00Z",
  "active": true,
  "secret": "webhook_secret_key"
}
```

#### 3.5.2 Webhook Payload Example

```http
POST https://executor-app.com/webhooks/erasure
Content-Type: application/json
X-WIA-Signature: sha256=signature_hash
X-WIA-Event: account.deletion.completed
```

**Webhook Payload:**

```json
{
  "eventId": "EVENT-2025-12-20-001",
  "eventType": "account.deletion.completed",
  "timestamp": "2025-12-20T15:00:00Z",
  "data": {
    "requestId": "ER-2025-12-18-001",
    "accountId": "ACC-001",
    "platform": "facebook",
    "deletionTimestamp": "2025-12-20T15:00:00Z",
    "verificationStatus": "confirmed_deleted"
  }
}
```

### 3.6 Platform Integration Endpoints

#### 3.6.1 Submit Platform Deletion Request

```http
POST /platforms/{platform}/delete
Authorization: Bearer {access_token}
Content-Type: application/json
```

**Request Body:**

```json
{
  "accountIdentifier": "user@example.com",
  "deathCertificate": "base64_encoded_certificate",
  "executorAuthorization": "base64_encoded_authorization",
  "requestedAction": "permanent_deletion",
  "dataRetention": "none"
}
```

**Response (202 Accepted):**

```json
{
  "platformRequestId": "FB-REQ-2025-12-18-001",
  "platform": "facebook",
  "status": "submitted",
  "estimatedProcessingTime": "14-30 days",
  "trackingUrl": "https://facebook.com/memorialization/request/ABC123"
}
```

#### 3.6.2 Get Platform Deletion Status

```http
GET /platforms/{platform}/delete/{platformRequestId}
Authorization: Bearer {access_token}
```

**Response (200 OK):**

```json
{
  "platformRequestId": "FB-REQ-2025-12-18-001",
  "platform": "facebook",
  "status": "completed",
  "submittedAt": "2025-12-18T11:05:00Z",
  "completedAt": "2025-12-20T15:00:00Z",
  "confirmationId": "FB-DEL-2025-12-20-001",
  "permanent": true
}
```

---

## Request/Response Formats

### 4.1 Standard Headers

| Header | Required | Description |
|--------|----------|-------------|
| `Authorization` | Yes | Bearer token or API key |
| `Content-Type` | Yes | application/json |
| `X-Request-ID` | No | Unique request identifier |
| `X-Idempotency-Key` | No | Idempotency key for POST requests |
| `User-Agent` | Yes | Client application identifier |

### 4.2 Error Response Format

```json
{
  "error": {
    "code": "ERR_UNVERIFIED_EXECUTOR",
    "message": "Executor must be verified before processing",
    "details": {
      "field": "executor.verified",
      "reason": "Verification required for deletion requests"
    },
    "timestamp": "2025-12-18T11:00:00Z",
    "requestId": "REQ-550e8400"
  }
}
```

### 4.3 Pagination Format

```json
{
  "data": [...],
  "pagination": {
    "page": 1,
    "limit": 20,
    "total": 100,
    "totalPages": 5,
    "hasNext": true,
    "hasPrevious": false,
    "links": {
      "first": "/erasure-requests?page=1&limit=20",
      "last": "/erasure-requests?page=5&limit=20",
      "next": "/erasure-requests?page=2&limit=20",
      "previous": null
    }
  }
}
```

---

## Error Handling

### 5.1 HTTP Status Codes

| Status Code | Description | Use Case |
|-------------|-------------|----------|
| 200 OK | Success | Successful GET, PATCH, DELETE |
| 201 Created | Resource created | Successful POST |
| 202 Accepted | Processing | Async operation started |
| 204 No Content | Success, no data | Successful DELETE |
| 400 Bad Request | Invalid request | Validation errors |
| 401 Unauthorized | Not authenticated | Missing/invalid token |
| 403 Forbidden | Not authorized | Insufficient permissions |
| 404 Not Found | Resource not found | Invalid resource ID |
| 409 Conflict | Resource conflict | Duplicate request |
| 422 Unprocessable Entity | Validation failed | Business logic error |
| 429 Too Many Requests | Rate limit exceeded | Too many requests |
| 500 Internal Server Error | Server error | Unexpected error |
| 503 Service Unavailable | Service down | Maintenance |

### 5.2 Error Codes

| Code | HTTP Status | Description |
|------|-------------|-------------|
| `ERR_INVALID_TOKEN` | 401 | Invalid authentication token |
| `ERR_UNVERIFIED_EXECUTOR` | 403 | Executor not verified |
| `ERR_INSUFFICIENT_PERMISSIONS` | 403 | Missing required scopes |
| `ERR_REQUEST_NOT_FOUND` | 404 | Erasure request not found |
| `ERR_INVALID_DATE_SEQUENCE` | 422 | Invalid date order |
| `ERR_PLATFORM_TIMEOUT` | 503 | Platform API timeout |
| `ERR_RATE_LIMIT_EXCEEDED` | 429 | Rate limit exceeded |

---

## Rate Limiting

### 6.1 Rate Limit Rules

| Endpoint Category | Rate Limit | Window |
|-------------------|------------|--------|
| Authentication | 10 requests | 1 minute |
| Erasure Requests | 100 requests | 1 hour |
| Account Discovery | 50 requests | 1 hour |
| Verification | 200 requests | 1 hour |
| Webhooks | 1000 requests | 1 hour |

### 6.2 Rate Limit Headers

```http
HTTP/1.1 200 OK
X-RateLimit-Limit: 100
X-RateLimit-Remaining: 95
X-RateLimit-Reset: 1703001600
X-RateLimit-Window: 3600
```

### 6.3 Rate Limit Exceeded Response

```json
{
  "error": {
    "code": "ERR_RATE_LIMIT_EXCEEDED",
    "message": "Rate limit exceeded",
    "details": {
      "limit": 100,
      "window": "1 hour",
      "resetAt": "2025-12-18T12:00:00Z"
    }
  }
}
```

---


## Annex E — Implementation Notes for PHASE-2-API-INTERFACE

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-2-API-INTERFACE.

- **Operational scope** — implementations SHOULD declare their operational
  scope (single-tenant, multi-tenant, federated) in the OpenAPI document so
  that downstream auditors can score the deployment against the correct
  conformance tier in Annex A.
- **Schema evolution** — additive changes (new optional fields, new error
  codes) are non-breaking; renaming or removing fields, even in error
  payloads, MUST trigger a minor version bump.
- **Audit retention** — a 7-year retention window is sufficient to satisfy
  ISO/IEC 17065:2012 audit expectations in most jurisdictions; some
  regulators require longer retention, in which case the deployment policy
  MUST extend the retention window rather than relying on this PHASE's
  defaults.
- **Time synchronization** — sub-second deadlines depend on synchronized
  clocks. NTPv4 with stratum-2 servers is sufficient for most deadlines
  expressed in this PHASE; PTP is recommended for sites that require
  deterministic interlocks.
- **Error budget reporting** — implementations SHOULD publish a monthly
  error-budget summary (latency p95, error rate, violation hours) in the
  format defined by the WIA reporting profile to facilitate cross-vendor
  comparison without exposing tenant-specific data.

These notes are not requirements; they are a reference for field teams
mapping their existing operations onto WIA conformance.

## Annex F — Adoption Roadmap

The adoption roadmap for this PHASE document is non-normative and is intended to set expectations for early implementers about the relative stability of each section.

- **Stable** (sections marked normative with `MUST` / `MUST NOT`) — semantic versioning applies; breaking changes require a major version bump and at minimum 90 days of overlap with the prior major version on all WIA-published reference implementations.
- **Provisional** (sections in this Annex and Annex D) — items are tracked openly and may be promoted to normative status without a major version bump if community feedback supports promotion.
- **Reference** (test vectors, simulator behaviour, the reference TypeScript SDK) — versioned independently of this document so that mistakes in reference material can be corrected without amending the published PHASE document.

Implementers SHOULD subscribe to the WIA Standards GitHub release notifications to track promotions between these tiers. Comments on the roadmap are accepted via the GitHub issues tracker on the WIA-Official organization.

The roadmap is reviewed at every minor version of this PHASE document, and the review outcomes are recorded in the version-history table at the start of the document.

## Annex G — Test Vectors and Conformance Evidence

This annex describes how implementations capture and publish conformance
evidence for PHASE-2-API-INTERFACE. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-2-api-interface/`. Implementations claiming
  conformance MUST run all vectors in CI and publish the resulting
  pass/fail matrix in their compliance package.
- **Evidence package** — the compliance package is a tarball containing
  the SBOM (CycloneDX 1.5 or SPDX 2.3), the OpenAPI document, the test
  vector matrix, and a signed manifest. Signatures use Sigstore (DSSE
  envelope, Rekor transparency log entry) so that downstream consumers
  can verify provenance without trusting a private CA.
- **Quarterly recheck** — implementations re-publish the evidence package
  every quarter even if no source change occurred, so that consumers can
  detect environmental drift (compiler updates, dependency updates, OS
  updates) without polling vendor changelogs.
- **Cross-vendor crosswalk** — the WIA Standards working group maintains a
  crosswalk that maps each vector to the equivalent assertion in adjacent
  industry programs (where one exists), so an implementer that already
  certifies under one program can show conformance to PHASE-2-API-INTERFACE with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-2-API-INTERFACE does not require bespoke
auditor tooling.
