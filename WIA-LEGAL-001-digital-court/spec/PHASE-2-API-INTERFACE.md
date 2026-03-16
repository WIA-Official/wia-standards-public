# WIA-LEGAL-001: Digital Court - Phase 2: API Interface

**Version:** 1.0
**Status:** Complete
**Last Updated:** 2025-12-27

## Overview

Phase 2 defines the REST API and GraphQL interfaces for digital court systems, enabling electronic filing, case management, hearing scheduling, and secure communication.

## 1. API Endpoints

### 1.1 Base URL

```
Production: https://api.digitalcourt.wia.org/v1
Staging: https://api-staging.digitalcourt.wia.org/v1
```

### 1.2 Authentication

```http
POST /auth/login
POST /auth/refresh
POST /auth/logout
```

### 1.3 Case Management

```http
GET    /cases                    # List cases
POST   /cases                    # Create new case
GET    /cases/{caseId}           # Get case details
PUT    /cases/{caseId}           # Update case
DELETE /cases/{caseId}           # Archive case
GET    /cases/{caseId}/timeline  # Get case timeline
GET    /cases/{caseId}/parties   # Get all parties
POST   /cases/{caseId}/parties   # Add party
```

### 1.4 Document Filing

```http
GET    /documents                # List documents
POST   /documents                # File new document
GET    /documents/{docId}        # Get document
PUT    /documents/{docId}        # Update document
DELETE /documents/{docId}        # Remove document
GET    /documents/{docId}/versions  # Get document versions
POST   /documents/{docId}/sign   # Digital signature
```

### 1.5 Hearing Management

```http
GET    /hearings                 # List hearings
POST   /hearings                 # Schedule hearing
GET    /hearings/{hearingId}     # Get hearing details
PUT    /hearings/{hearingId}     # Update hearing
DELETE /hearings/{hearingId}     # Cancel hearing
POST   /hearings/{hearingId}/join  # Join virtual hearing
GET    /hearings/{hearingId}/recording  # Get recording
```

### 1.6 Evidence Management

```http
GET    /evidence                 # List evidence
POST   /evidence                 # Submit evidence
GET    /evidence/{evidenceId}    # Get evidence
PUT    /evidence/{evidenceId}    # Update evidence
POST   /evidence/{evidenceId}/verify  # Verify authenticity
GET    /evidence/{evidenceId}/chain   # Get chain of custody
```

## 2. Request/Response Format

### 2.1 Create Case

**Request:**

```http
POST /cases
Content-Type: application/json
Authorization: Bearer {token}

{
  "caseNumber": "2025-CV-001234",
  "caseType": "civil",
  "court": {
    "courtId": "court-nysd-001",
    "jurisdiction": "US-NY"
  },
  "parties": {
    "plaintiffs": [{
      "name": "John Doe",
      "did": "did:wia:legal:johndoe123",
      "email": "john@example.com"
    }],
    "defendants": [{
      "name": "Acme Corp",
      "did": "did:wia:legal:acmecorp",
      "email": "legal@acme.com"
    }]
  },
  "description": "Contract dispute case"
}
```

**Response:**

```http
HTTP/1.1 201 Created
Content-Type: application/json

{
  "success": true,
  "data": {
    "caseId": "550e8400-e29b-41d4-a716-446655440000",
    "caseNumber": "2025-CV-001234",
    "status": "filed",
    "filingDate": "2025-12-27T10:30:00Z",
    "nextSteps": [
      "Await defendant response (30 days)",
      "Preliminary hearing scheduled"
    ]
  },
  "timestamp": "2025-12-27T10:30:00Z"
}
```

### 2.2 File Document

**Request:**

```http
POST /documents
Content-Type: multipart/form-data
Authorization: Bearer {token}

{
  "caseId": "550e8400-e29b-41d4-a716-446655440000",
  "type": "motion",
  "title": "Motion to Dismiss",
  "filedBy": "party-002",
  "confidentiality": "public",
  "file": <binary>
}
```

**Response:**

```http
HTTP/1.1 201 Created
Content-Type: application/json

{
  "success": true,
  "data": {
    "documentId": "doc-123456",
    "caseId": "550e8400-e29b-41d4-a716-446655440000",
    "title": "Motion to Dismiss",
    "hash": "sha256:a3f5b8c9d2e1f0...",
    "filedDate": "2025-12-27T11:00:00Z",
    "status": "accepted",
    "serveRequired": true,
    "serveDeadline": "2025-12-30T23:59:59Z"
  },
  "timestamp": "2025-12-27T11:00:00Z"
}
```

### 2.3 Schedule Virtual Hearing

**Request:**

```http
POST /hearings
Content-Type: application/json
Authorization: Bearer {token}

{
  "caseId": "550e8400-e29b-41d4-a716-446655440000",
  "type": "preliminary",
  "scheduledDate": "2026-01-15T14:00:00Z",
  "duration": 60,
  "mode": "virtual",
  "participants": [
    {
      "partyId": "party-001",
      "role": "plaintiff"
    },
    {
      "partyId": "party-002",
      "role": "defendant"
    }
  ],
  "recording": true
}
```

**Response:**

```http
HTTP/1.1 201 Created
Content-Type: application/json

{
  "success": true,
  "data": {
    "hearingId": "hearing-789",
    "caseId": "550e8400-e29b-41d4-a716-446655440000",
    "scheduledDate": "2026-01-15T14:00:00Z",
    "virtualRoomUrl": "https://court.wia.org/room/hearing-789",
    "accessCode": "COURT-2025-789",
    "calendar": {
      "icsUrl": "https://court.wia.org/calendar/hearing-789.ics",
      "googleCalendar": "https://calendar.google.com/event?...",
      "outlookCalendar": "https://outlook.office365.com/calendar?..."
    },
    "notifications": {
      "emailSent": true,
      "smsSent": true,
      "reminder24h": true,
      "reminder1h": true
    }
  },
  "timestamp": "2025-12-27T11:30:00Z"
}
```

## 3. Authentication & Authorization

### 3.1 OAuth 2.0 Flow

```http
POST /auth/login
Content-Type: application/json

{
  "username": "attorney@example.com",
  "password": "secure_password",
  "mfa_code": "123456"
}
```

**Response:**

```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "refresh_token": "refresh_token_here",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "case:read case:write document:file hearing:schedule"
}
```

### 3.2 Authorization Scopes

| Scope | Description |
|-------|-------------|
| `case:read` | Read case information |
| `case:write` | Create and update cases |
| `document:file` | File documents |
| `document:read` | Access documents |
| `hearing:schedule` | Schedule hearings |
| `hearing:join` | Join virtual hearings |
| `evidence:submit` | Submit evidence |
| `admin:manage` | Administrative access |

### 3.3 Role-Based Access Control

**Roles:**
- **Judge**: Full access to assigned cases
- **Attorney**: Access to cases where representing a party
- **Party**: Access to own cases (limited)
- **Clerk**: Administrative case management
- **Public**: Public records only

## 4. Error Codes

### 4.1 HTTP Status Codes

| Code | Meaning | Description |
|------|---------|-------------|
| 200 | OK | Successful request |
| 201 | Created | Resource created |
| 204 | No Content | Successful deletion |
| 400 | Bad Request | Invalid request format |
| 401 | Unauthorized | Authentication required |
| 403 | Forbidden | Insufficient permissions |
| 404 | Not Found | Resource not found |
| 409 | Conflict | Resource conflict |
| 422 | Unprocessable Entity | Validation error |
| 429 | Too Many Requests | Rate limit exceeded |
| 500 | Internal Server Error | Server error |
| 503 | Service Unavailable | Temporary unavailability |

### 4.2 Error Response Format

```json
{
  "success": false,
  "error": {
    "code": "VALIDATION_ERROR",
    "message": "Invalid case number format",
    "details": {
      "field": "caseNumber",
      "expected": "YYYY-TT-NNNNNN",
      "received": "invalid-format"
    },
    "timestamp": "2025-12-27T12:00:00Z",
    "requestId": "req-abc123"
  }
}
```

## 5. Rate Limiting

### 5.1 Rate Limits

| Endpoint Category | Rate Limit | Window |
|-------------------|------------|--------|
| Authentication | 10 requests | 1 minute |
| Case Read | 100 requests | 1 minute |
| Case Write | 20 requests | 1 minute |
| Document Filing | 50 requests | 1 hour |
| Hearings | 30 requests | 1 minute |

### 5.2 Rate Limit Headers

```http
X-RateLimit-Limit: 100
X-RateLimit-Remaining: 95
X-RateLimit-Reset: 1640606400
```

## 6. Webhooks

### 6.1 Event Types

```
case.created
case.updated
case.closed
document.filed
document.served
hearing.scheduled
hearing.completed
evidence.submitted
order.issued
```

### 6.2 Webhook Payload

```json
{
  "event": "document.filed",
  "eventId": "evt-123456",
  "timestamp": "2025-12-27T13:00:00Z",
  "data": {
    "caseId": "550e8400-e29b-41d4-a716-446655440000",
    "documentId": "doc-123456",
    "type": "motion",
    "filedBy": "party-002"
  }
}
```

## 7. GraphQL API (Alternative)

### 7.1 Schema

```graphql
type Query {
  case(id: ID!): Case
  cases(filter: CaseFilter, limit: Int, offset: Int): [Case]
  document(id: ID!): Document
  hearing(id: ID!): Hearing
  evidence(id: ID!): Evidence
}

type Mutation {
  createCase(input: CaseInput!): Case
  updateCase(id: ID!, input: CaseInput!): Case
  fileDocument(input: DocumentInput!): Document
  scheduleHearing(input: HearingInput!): Hearing
  submitEvidence(input: EvidenceInput!): Evidence
}

type Subscription {
  caseUpdated(caseId: ID!): Case
  documentFiled(caseId: ID!): Document
  hearingScheduled(caseId: ID!): Hearing
}
```

### 7.2 Example Query

```graphql
query GetCase {
  case(id: "550e8400-e29b-41d4-a716-446655440000") {
    caseNumber
    caseType
    status
    parties {
      plaintiffs {
        name
        did
      }
      defendants {
        name
        did
      }
    }
    documents {
      id
      title
      filedDate
    }
    hearings {
      id
      scheduledDate
      type
    }
  }
}
```

## 8. SDK Examples

### 8.1 JavaScript/TypeScript

```typescript
import { DigitalCourtClient } from '@wia/legal-001';

const client = new DigitalCourtClient({
  apiKey: 'your-api-key',
  environment: 'production'
});

// Create a new case
const newCase = await client.cases.create({
  caseNumber: '2025-CV-001234',
  caseType: 'civil',
  parties: {
    plaintiffs: [{ name: 'John Doe' }],
    defendants: [{ name: 'Acme Corp' }]
  }
});

// File a document
const document = await client.documents.file({
  caseId: newCase.caseId,
  type: 'motion',
  title: 'Motion to Dismiss',
  file: fileBuffer
});

// Schedule a hearing
const hearing = await client.hearings.schedule({
  caseId: newCase.caseId,
  type: 'preliminary',
  scheduledDate: '2026-01-15T14:00:00Z',
  mode: 'virtual'
});
```

---

**弘益人間 (Hongik Ingan)** - *Benefit All Humanity*

© 2025 WIA - World Certification Industry Association | MIT License
