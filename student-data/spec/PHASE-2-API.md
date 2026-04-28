# WIA-EDU-010 — Phase 2: API Interface

> Student-data canonical Phase 2: REST surface + auth + webhooks + batch.

# WIA-EDU-010 Student Data Standard v1.1

## Phase 2: API Interface & Integration

**Status:** ✅ Complete
**Version:** 1.1.0
**Date:** 2025-01-15
**Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

Phase 2 extends v1.0 with comprehensive API specifications, authentication mechanisms, and integration protocols. This phase enables interoperability between student information systems and third-party applications.

## 2. Changes from v1.0

- Added RESTful API specification
- Defined authentication and authorization
- Added webhook support for real-time events
- Specified rate limiting and pagination
- Added batch operations
- Defined error handling standards

## 3. RESTful API Specification

### 3.1 Base URL Structure

```
https://api.{institution}.edu/wia/v1/
```

### 3.2 Resource Endpoints

#### Student Resources

```
GET    /students                    # List students (paginated)
POST   /students                    # Create student
GET    /students/{studentId}        # Get student profile
PUT    /students/{studentId}        # Update student profile
PATCH  /students/{studentId}        # Partial update
DELETE /students/{studentId}        # Soft delete student
```

#### Academic Records

```
GET    /students/{studentId}/records              # Get all academic records
POST   /students/{studentId}/records              # Add new record
GET    /students/{studentId}/records/{recordId}   # Get specific record
PUT    /students/{studentId}/records/{recordId}   # Update record
DELETE /students/{studentId}/records/{recordId}   # Delete record

GET    /students/{studentId}/transcripts          # Generate transcript
POST   /students/{studentId}/transcripts/request  # Request official transcript
```

#### Attendance

```
GET    /students/{studentId}/attendance           # Get attendance records
POST   /students/{studentId}/attendance           # Record attendance
GET    /students/{studentId}/attendance/summary   # Get attendance summary
```

#### Privacy & Consent

```
GET    /students/{studentId}/privacy              # Get privacy settings
PUT    /students/{studentId}/privacy              # Update privacy settings
GET    /students/{studentId}/consent              # Get consent log
POST   /students/{studentId}/consent              # Record new consent
DELETE /students/{studentId}/consent/{consentId}  # Withdraw consent
```

#### Data Export (GDPR)

```
POST   /students/{studentId}/export               # Request data export
GET    /students/{studentId}/export/{exportId}    # Check export status
GET    /students/{studentId}/export/{exportId}/download  # Download export
```

## 4. Authentication

### 4.1 Supported Methods

**OAuth 2.0** (REQUIRED)
- Authorization Code Flow for user applications
- Client Credentials Flow for server-to-server
- Token expiration: 1 hour (access), 30 days (refresh)

**OpenID Connect** (RECOMMENDED)
- SSO integration
- Student/staff authentication

**API Keys** (OPTIONAL)
- For internal system integration
- Must be rotated every 90 days

### 4.2 Authorization Header

```http
Authorization: Bearer {access_token}
```

### 4.3 Scopes

```
students:read          - Read student profiles
students:write         - Create/update students
records:read           - Read academic records
records:write          - Create/update records
attendance:read        - Read attendance data
attendance:write       - Record attendance
privacy:read           - Read privacy settings
privacy:write          - Update privacy settings
export:request         - Request data exports
```

## 5. Request/Response Format

### 5.1 Content Types

**Supported:**
- `application/json` (default)
- `application/xml` (legacy support)

**Headers:**
```http
Content-Type: application/json
Accept: application/json
```

### 5.2 Standard Response Format

**Success Response:**
```json
{
  "success": true,
  "data": { },
  "metadata": {
    "requestId": "req_abc123",
    "timestamp": "2025-01-15T10:30:00Z",
    "version": "1.1.0"
  }
}
```

**Error Response:**
```json
{
  "success": false,
  "error": {
    "code": "STUDENT_NOT_FOUND",
    "message": "Student with ID '2024-CS-001' not found",
    "details": {
      "studentId": "2024-CS-001"
    }
  },
  "metadata": {
    "requestId": "req_abc123",
    "timestamp": "2025-01-15T10:30:00Z",
    "version": "1.1.0"
  }
}
```

## 6. Error Handling

### 6.1 HTTP Status Codes

```
200 OK                  - Successful GET, PUT, PATCH
201 Created             - Successful POST
204 No Content          - Successful DELETE
400 Bad Request         - Invalid request data
401 Unauthorized        - Missing/invalid authentication
403 Forbidden           - Insufficient permissions
404 Not Found           - Resource not found
409 Conflict            - Duplicate/conflicting data
422 Unprocessable       - Validation error
429 Too Many Requests   - Rate limit exceeded
500 Internal Error      - Server error
503 Service Unavailable - Maintenance mode
```

### 6.2 Error Codes

```
INVALID_REQUEST         - Malformed request
AUTHENTICATION_REQUIRED - No auth token provided
INVALID_TOKEN           - Token expired or invalid
INSUFFICIENT_SCOPE      - Missing required scope
STUDENT_NOT_FOUND       - Student ID not found
RECORD_NOT_FOUND        - Academic record not found
VALIDATION_ERROR        - Data validation failed
DUPLICATE_STUDENT       - Student already exists
RATE_LIMIT_EXCEEDED     - Too many requests
FERPA_VIOLATION         - Privacy rule violation
GDPR_VIOLATION          - GDPR compliance issue
```

## 7. Pagination

### 7.1 Request Parameters

```
?page=1              # Page number (1-based)
&limit=50            # Items per page (max 100)
&sort=lastName       # Sort field
&order=asc           # Sort order (asc/desc)
```

### 7.2 Response Format

```json
{
  "success": true,
  "data": [ ],
  "pagination": {
    "page": 1,
    "limit": 50,
    "total": 10000,
    "totalPages": 200,
    "hasNext": true,
    "hasPrevious": false,
    "links": {
      "self": "/students?page=1&limit=50",
      "next": "/students?page=2&limit=50",
      "first": "/students?page=1&limit=50",
      "last": "/students?page=200&limit=50"
    }
  }
}
```

## 8. Rate Limiting

### 8.1 Limits

- **Standard:** 1000 requests per hour
- **Authenticated:** 5000 requests per hour
- **Premium:** 10000 requests per hour

### 8.2 Headers

```http
X-RateLimit-Limit: 5000
X-RateLimit-Remaining: 4999
X-RateLimit-Reset: 1642780800
```

## 9. Webhooks

### 9.1 Supported Events

```
student.created         - New student enrolled
student.updated         - Student profile updated
student.deleted         - Student withdrawn/deleted
record.added            - New academic record
record.updated          - Grade/record modified
attendance.recorded     - Attendance recorded
privacy.updated         - Privacy settings changed
consent.granted         - New consent given
consent.withdrawn       - Consent withdrawn
export.completed        - Data export ready
```

### 9.2 Webhook Payload

```json
{
  "event": "student.created",
  "timestamp": "2025-01-15T10:30:00Z",
  "data": {
    "studentId": "2024-CS-001",
    "changes": { }
  },
  "metadata": {
    "webhookId": "wh_abc123",
    "deliveryId": "del_xyz789"
  }
}
```

### 9.3 Webhook Security

- HMAC-SHA256 signature verification
- TLS 1.3 required
- Retry policy: 3 attempts with exponential backoff

## 10. Batch Operations

### 10.1 Batch Create

```http
POST /students/batch

{
  "students": [
    { },
    { }
  ]
}
```

### 10.2 Batch Response

```json
{
  "success": true,
  "results": [
    {"success": true, "studentId": "2024-CS-001"},
    {"success": false, "error": "VALIDATION_ERROR"}
  ],
  "summary": {
    "total": 100,
    "successful": 98,
    "failed": 2
  }
}
```

## 11. Filtering and Search

### 11.1 Query Parameters

```
?firstName=John
&lastName=Smith
&enrollmentStatus=active
&program=Computer Science
&enrollmentDate[gte]=2024-01-01
&enrollmentDate[lte]=2024-12-31
```

### 11.2 Full-Text Search

```
?q=John Smith Computer Science
```

## 12. Versioning

### 12.1 API Version

Include in URL path: `/wia/v1/`

### 12.2 Schema Version

Include in headers:
```http
X-WIA-Schema-Version: 1.1.0
```

---

**Philosophy:** 弘益人間 (Benefit All Humanity)

*Enabling seamless integration while protecting student privacy and data rights.*

---

## A.1 Endpoint reference

```http
GET    /students/v1/students                  # list (paginated)
POST   /students/v1/students                  # create
GET    /students/v1/students/{id}             # fetch profile
PATCH  /students/v1/students/{id}             # partial update
GET    /students/v1/students/{id}/records     # academic records
POST   /students/v1/students/{id}/records     # add record
GET    /students/v1/students/{id}/attendance  # attendance
GET    /students/v1/students/{id}/privacy     # privacy
POST   /students/v1/students/{id}/consent     # consent grant
POST   /students/v1/students/{id}/export      # data export request
```

Every endpoint follows the discovery convention at `/.well-known/wia-student-data`.

## A.2 Authentication and scopes

OAuth 2.0 + OpenID Connect authentication. Scopes: `students:read`, `students:write`, `records:read`, `records:write`, `attendance:read`, `attendance:write`, `privacy:read`, `privacy:write`, `export:request`. Access tokens are 1 hour; refresh tokens are 30 days. API keys are an optional secondary path for internal system-to-system traffic and MUST be rotated every 90 days.

## A.3 Webhook events

Webhooks fire on `student.created`, `student.updated`, `student.deleted`, `record.added`, `record.updated`, `attendance.recorded`, `privacy.updated`, `consent.granted`, `consent.withdrawn`, `export.completed`. Delivery is at-least-once with HMAC-SHA256 signing; receivers dedupe on `deliveryId`. Retry policy: 3 attempts at 1s/4s/16s; the failed delivery enters the dead-letter queue and operations is paged.

## A.4 Pagination and filtering

Cursor-based pagination via `?after=cursor&limit=N` (max 100). Filtering uses the `?field=value` and `?field[op]=value` syntax (`gte`, `lte`, `eq`, `in`); search uses `?q=string` against the searchable index (firstName, lastName, email). Sort via `?sort=field&order=asc|desc`.

## A.5 Batch operations

`POST /students/batch`, `POST /records/batch`, `POST /attendance/batch` accept up to 100 entries per request. The response carries per-entry success/failure status; partial-success is the normal case so clients MUST inspect the per-entry status rather than assuming whole-batch atomicity.

## A.6 Rate-limit envelope

1000 req/h unauthenticated, 5000 req/h authenticated, 10000 req/h premium tier. Token-bucket refill; the response advertises the current state in `X-RateLimit-Limit`, `X-RateLimit-Remaining`, and `X-RateLimit-Reset` per IETF RFC 6585.


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/student-data/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-student-data-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/student-data-host:1.0.0` ships every student-data envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/student-data.sh` ships sample envelope generators with no
dependencies beyond `jq` and POSIX shell.

## Z.2 Cross-standard composition (recap)

This Phase composes with WIA-OMNI-API for credential storage,
WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 §5 for
federation handshake, WIA-INTENT for workload intent declaration,
and (where personal data is processed) WIA Secure Enclave for
sealed-data envelopes. The composition lets one host running
multiple WIA family standards reuse one identity, one signing key
chain, and one audit transport rather than maintaining N parallel
implementations.

## Z.3 Implementation runbook

A first implementation typically follows: stand up the reference
container; run the conformance suite against it end-to-end;
replace the mock backend with a real backend one endpoint at a
time; wire audit-log replication out to the operations sink;
onboard a single trusted peer for federation; expand to multiple
peers; promote to production with the warning-envelope subscription
enabled and the runbook in §Z.5 followed.

## Z.4 Backwards-compatibility promise

Within the 1.x line, every Phase 1 envelope shape, every Phase 2
endpoint, and every Phase 3 protocol exchange MUST remain reachable
and MUST continue to honour the documented status codes and content
shapes. Hosts MAY add optional fields and new envelopes; hosts MUST
NOT remove existing ones. Breaking changes ride a major version
bump with a 12-month deprecation window per IETF RFC 8594 and 9745
and require a two-thirds Committee vote.

## Z.5 Closing implementer note

This Phase fits inside the WIA Standards four-Phase architecture:
Phase 1 envelopes are the wire-format contract; Phase 2 surfaces
them through HTTPS; Phase 3 wraps them in protocol exchanges that
cross trust boundaries; Phase 4 integrates with the broader
ecosystem. Student-data deployments that follow this layering
inherit the per-Phase conformance gates and the cross-standard
composition behaviour described in Z.2.

弘益人間 — Benefit All Humanity.
