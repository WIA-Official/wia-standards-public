# WIA-EDU-007: Educational Robot Standard
## Phase 2: API Interface Specification

**Version:** 1.1.0
**Status:** ✅ Complete
**Last Updated:** 2025-12-25
**Category:** Education/Culture (EDU)

---

## 1. Overview

### 1.1 Purpose
This specification defines the RESTful API interface for educational robots, enabling standardized communication between robots, learning management systems, educational applications, and the WIA ecosystem.

### 1.2 Philosophy
**弘益人間 (Benefit All Humanity)** - By standardizing APIs, we enable seamless integration of educational robots across platforms, making quality education accessible to all.

### 1.3 API Design Principles
- **RESTful architecture**: HTTP methods for CRUD operations
- **JSON-LD responses**: Semantic interoperability
- **OAuth 2.0 + DID**: Secure authentication
- **Rate limiting**: Fair resource allocation
- **Versioning**: Backward compatibility

---

## 2. Base URL & Versioning

### 2.1 Base URL
```
https://api.wiastandards.com/edu-robot/v1
```

### 2.2 Version Header
```http
WIA-API-Version: 1.1.0
```

### 2.3 Content Type
```http
Content-Type: application/ld+json
```

---

## 3. Authentication

### 3.1 OAuth 2.0 + DID
All API requests require authentication using OAuth 2.0 with Decentralized Identifiers (DIDs).

#### Authorization Header
```http
Authorization: Bearer <access_token>
```

#### Token Endpoint
```http
POST /auth/token
Content-Type: application/json

{
  "grant_type": "client_credentials",
  "client_id": "did:wia:robot:EDU-ROBOT-2025-001",
  "client_secret": "<secret>",
  "scope": "robot.read robot.write student.read"
}
```

#### Response
```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "robot.read robot.write student.read"
}
```

### 3.2 Scopes

| Scope | Description | Access Level |
|-------|-------------|--------------|
| `robot.read` | Read robot profile & capabilities | Read |
| `robot.write` | Update robot configuration | Write |
| `student.read` | View student progress (anonymized) | Read |
| `student.write` | Record learning sessions | Write |
| `curriculum.read` | Access lesson plans | Read |
| `assessment.write` | Submit assessments | Write |
| `admin` | Full administrative access | Admin |

---

## 4. Core Endpoints

### 4.1 Robot Profile Management

#### GET /robots/{robotId}
Retrieve robot profile and capabilities.

**Request:**
```http
GET /robots/EDU-ROBOT-2025-001
Authorization: Bearer <token>
```

**Response:** `200 OK`
```json
{
  "@context": "https://wiastandards.com/contexts/educational-robot/v1",
  "@type": "EducationalRobotProfile",
  "robotId": "EDU-ROBOT-2025-001",
  "robotType": "stem-education",
  "specifications": {
    "ageGroup": "7-12",
    "subjectAreas": ["mathematics", "science", "programming"],
    "interactionModes": ["voice", "touch", "visual"],
    "languages": ["en", "es", "zh", "ko"]
  },
  "capabilities": {
    "teaching": ["adaptive-instruction", "personalized-feedback"],
    "interaction": ["natural-language-processing", "emotion-detection"],
    "assessment": ["real-time-quizzing", "progress-tracking"]
  },
  "status": "active",
  "lastSeen": "2025-12-25T10:30:00Z"
}
```

#### PUT /robots/{robotId}
Update robot configuration.

**Request:**
```http
PUT /robots/EDU-ROBOT-2025-001
Authorization: Bearer <token>
Content-Type: application/json

{
  "status": "active",
  "location": {
    "school": "Lincoln Elementary",
    "classroom": "Room 204"
  },
  "assignedTeacher": "did:wia:teacher:12345"
}
```

**Response:** `200 OK`

#### DELETE /robots/{robotId}
Deactivate robot (soft delete).

**Response:** `204 No Content`

---

### 4.2 Learning Session Management

#### POST /sessions
Create a new learning session.

**Request:**
```http
POST /sessions
Authorization: Bearer <token>
Content-Type: application/json

{
  "robotId": "EDU-ROBOT-2025-001",
  "studentId": "anonymous-hash-12345",
  "sessionType": "practice",
  "curriculum": {
    "subjectArea": "mathematics",
    "lessonId": "MATH-GR3-L12",
    "learningObjectives": ["Multiply single-digit numbers"],
    "difficulty": "intermediate"
  }
}
```

**Response:** `201 Created`
```json
{
  "sessionId": "SESSION-2025-12345",
  "robotId": "EDU-ROBOT-2025-001",
  "studentId": "anonymous-hash-12345",
  "startTime": "2025-12-25T10:00:00Z",
  "status": "active",
  "sessionUrl": "/sessions/SESSION-2025-12345"
}
```

#### GET /sessions/{sessionId}
Retrieve session details.

**Response:** `200 OK`
```json
{
  "@context": "https://wiastandards.com/contexts/learning-session/v1",
  "sessionId": "SESSION-2025-12345",
  "robotId": "EDU-ROBOT-2025-001",
  "studentId": "anonymous-hash-12345",
  "sessionType": "practice",
  "startTime": "2025-12-25T10:00:00Z",
  "endTime": "2025-12-25T10:25:00Z",
  "duration": 25,
  "interaction": {
    "engagementScore": 87,
    "emotionalState": ["curious", "focused"],
    "questionsAsked": 8,
    "responsiveness": 0.92
  },
  "assessment": {
    "preTestScore": 65,
    "postTestScore": 82,
    "improvementRate": 26.2,
    "masteryLevel": "intermediate"
  }
}
```

#### PATCH /sessions/{sessionId}
Update session (e.g., record real-time engagement).

**Request:**
```http
PATCH /sessions/SESSION-2025-12345
Content-Type: application/json

{
  "interaction": {
    "engagementScore": 90,
    "emotionalState": ["excited", "confident"]
  }
}
```

**Response:** `200 OK`

#### DELETE /sessions/{sessionId}
End session.

**Response:** `204 No Content`

---

### 4.3 Student Progress Tracking

#### GET /students/{studentId}/progress
Retrieve student progress across all sessions.

**Query Parameters:**
- `subjectArea` (optional): Filter by subject
- `startDate` (optional): ISO 8601 date
- `endDate` (optional): ISO 8601 date

**Request:**
```http
GET /students/anonymous-hash-12345/progress?subjectArea=mathematics&startDate=2025-12-01
Authorization: Bearer <token>
```

**Response:** `200 OK`
```json
{
  "@context": "https://wiastandards.com/contexts/student-progress/v1",
  "studentId": "anonymous-hash-12345",
  "subjectArea": "mathematics",
  "progressPeriod": {
    "startDate": "2025-12-01T00:00:00Z",
    "endDate": "2025-12-25T23:59:59Z"
  },
  "metrics": {
    "sessionsCompleted": 18,
    "totalLearningTime": 450,
    "averageEngagement": 85,
    "skillsMastered": [
      "single-digit-multiplication",
      "double-digit-addition",
      "pattern-recognition"
    ],
    "challengeAreas": ["division-with-remainders"]
  },
  "achievements": [
    {
      "achievementId": "ACH-MATH-MULTIPLICATION",
      "type": "skill-mastery",
      "dateEarned": "2025-12-15T14:30:00Z",
      "level": "gold"
    }
  ]
}
```

#### POST /students/{studentId}/achievements
Award an achievement.

**Request:**
```http
POST /students/anonymous-hash-12345/achievements
Content-Type: application/json

{
  "achievementId": "ACH-MATH-EXPERT",
  "type": "course-completion",
  "level": "platinum",
  "verifiableCredential": true
}
```

**Response:** `201 Created`

---

### 4.4 Curriculum & Lesson Plans

#### GET /curriculum/{subjectArea}
List available lesson plans for a subject.

**Request:**
```http
GET /curriculum/mathematics?grade=3
Authorization: Bearer <token>
```

**Response:** `200 OK`
```json
{
  "@context": "https://wiastandards.com/contexts/curriculum/v1",
  "subjectArea": "mathematics",
  "grade": "3",
  "lessons": [
    {
      "lessonId": "MATH-GR3-L01",
      "title": "Introduction to Multiplication",
      "duration": 30,
      "difficulty": "beginner",
      "bloomLevel": "BL2"
    },
    {
      "lessonId": "MATH-GR3-L02",
      "title": "Multiplication Tables 1-5",
      "duration": 25,
      "difficulty": "beginner",
      "bloomLevel": "BL3"
    }
  ]
}
```

#### GET /curriculum/{subjectArea}/{lessonId}
Retrieve detailed lesson plan.

**Response:** `200 OK`
```json
{
  "@context": "https://wiastandards.com/contexts/curriculum/v1",
  "lessonId": "MATH-GR3-L01",
  "curriculum": {
    "framework": "Common Core State Standards",
    "subject": "mathematics",
    "grade": "3",
    "standards": ["CCSS.MATH.CONTENT.3.OA.A.1"]
  },
  "learningObjectives": [
    {
      "objectiveId": "OBJ-001",
      "description": "Interpret products of whole numbers",
      "bloomLevel": "BL2",
      "measurable": true
    }
  ],
  "activities": [
    {
      "activityId": "ACT-001",
      "type": "interactive-demo",
      "duration": 10,
      "materials": ["virtual-manipulatives"],
      "instructions": "Show 3 groups of 4 objects...",
      "assessment": "Can student identify total?"
    }
  ]
}
```

---

### 4.5 Real-Time Interaction

#### POST /interactions
Send student input to robot.

**Request:**
```http
POST /interactions
Content-Type: application/json

{
  "sessionId": "SESSION-2025-12345",
  "studentInput": {
    "type": "voice",
    "content": "What is 7 times 8?",
    "timestamp": "2025-12-25T10:15:30Z"
  }
}
```

**Response:** `200 OK`
```json
{
  "interactionId": "INT-67890",
  "robotResponse": {
    "type": "voice",
    "content": "Great question! Let's think about it together. 7 times 8 means 7 groups of 8. Can you use the visual aids on the screen?",
    "emotion": "encouraging",
    "gestureHint": "point-to-screen"
  },
  "timestamp": "2025-12-25T10:15:32Z"
}
```

---

## 5. Error Codes

### 5.1 Standard HTTP Status Codes

| Code | Status | Description |
|------|--------|-------------|
| `200` | OK | Successful request |
| `201` | Created | Resource created successfully |
| `204` | No Content | Successful deletion |
| `400` | Bad Request | Invalid request format |
| `401` | Unauthorized | Missing or invalid authentication |
| `403` | Forbidden | Insufficient permissions |
| `404` | Not Found | Resource not found |
| `429` | Too Many Requests | Rate limit exceeded |
| `500` | Internal Server Error | Server error |

### 5.2 Error Response Format

```json
{
  "@context": "https://wiastandards.com/contexts/error/v1",
  "@type": "ErrorResponse",
  "error": {
    "code": "STUDENT_NOT_FOUND",
    "message": "Student with ID 'anonymous-hash-12345' not found",
    "details": "Ensure student ID is correct and student has active consent",
    "timestamp": "2025-12-25T10:00:00Z",
    "requestId": "req-abc123"
  }
}
```

### 5.3 Custom Error Codes

| Code | HTTP Status | Description |
|------|-------------|-------------|
| `ROBOT_NOT_FOUND` | 404 | Robot ID does not exist |
| `SESSION_EXPIRED` | 410 | Learning session has expired |
| `CONSENT_REQUIRED` | 403 | Parental consent not on file |
| `AGE_MISMATCH` | 400 | Student age doesn't match robot certification |
| `CURRICULUM_UNAVAILABLE` | 404 | Requested lesson plan not available |
| `RATE_LIMIT_EXCEEDED` | 429 | Too many requests |

---

## 6. Rate Limiting

### 6.1 Rate Limits

| Tier | Requests/Minute | Requests/Day |
|------|-----------------|--------------|
| **Free** | 60 | 10,000 |
| **Education** | 600 | 100,000 |
| **Enterprise** | 6,000 | 1,000,000 |

### 6.2 Rate Limit Headers

```http
X-RateLimit-Limit: 600
X-RateLimit-Remaining: 587
X-RateLimit-Reset: 1735134000
```

### 6.3 Rate Limit Exceeded Response

```json
{
  "error": {
    "code": "RATE_LIMIT_EXCEEDED",
    "message": "Rate limit of 600 requests/minute exceeded",
    "retryAfter": 45,
    "timestamp": "2025-12-25T10:00:00Z"
  }
}
```

---

## 7. Pagination

### 7.1 Query Parameters

```http
GET /sessions?page=2&limit=50&sort=startTime&order=desc
```

| Parameter | Default | Description |
|-----------|---------|-------------|
| `page` | 1 | Page number |
| `limit` | 20 | Items per page (max 100) |
| `sort` | `timestamp` | Sort field |
| `order` | `asc` | Sort order (`asc` or `desc`) |

### 7.2 Pagination Response

```json
{
  "@context": "https://wiastandards.com/contexts/pagination/v1",
  "data": [...],
  "pagination": {
    "page": 2,
    "limit": 50,
    "totalPages": 5,
    "totalItems": 234,
    "hasNext": true,
    "hasPrevious": true,
    "nextPage": "/sessions?page=3&limit=50",
    "previousPage": "/sessions?page=1&limit=50"
  }
}
```

---

## 8. Webhooks

### 8.1 Webhook Registration

```http
POST /webhooks
Content-Type: application/json

{
  "url": "https://school.example.com/webhook/edu-robot",
  "events": ["session.completed", "achievement.earned", "alert.safety"],
  "secret": "webhook-secret-key"
}
```

### 8.2 Webhook Events

| Event | Trigger | Payload |
|-------|---------|---------|
| `session.started` | New session begins | Session object |
| `session.completed` | Session ends | Session + assessment |
| `achievement.earned` | Student earns achievement | Achievement object |
| `alert.safety` | Safety concern detected | Alert details |
| `consent.expired` | Parental consent expired | Student ID |

### 8.3 Webhook Payload

```json
{
  "@context": "https://wiastandards.com/contexts/webhook/v1",
  "@type": "WebhookEvent",
  "eventId": "evt-12345",
  "eventType": "achievement.earned",
  "timestamp": "2025-12-25T10:30:00Z",
  "data": {
    "studentId": "anonymous-hash-12345",
    "achievementId": "ACH-MATH-EXPERT",
    "type": "skill-mastery",
    "level": "gold"
  },
  "signature": "sha256=abc123..."
}
```

---

## 9. SDK Support

### 9.1 Official SDKs

| Language | Repository | Version |
|----------|-----------|---------|
| **TypeScript** | `@wia/edu-robot` | 1.1.0 |
| **Python** | `wia-edu-robot` | 1.1.0 |
| **Java** | `com.wia.edurobots` | 1.1.0 |
| **C#** | `WIA.EduRobots` | 1.1.0 |

### 9.2 TypeScript Example

```typescript
import { WIAEduRobot } from '@wia/edu-robot';

const client = new WIAEduRobot({
  apiKey: 'your-api-key',
  robotId: 'EDU-ROBOT-2025-001'
});

const session = await client.sessions.create({
  studentId: 'anonymous-hash-12345',
  sessionType: 'practice',
  curriculum: {
    subjectArea: 'mathematics',
    lessonId: 'MATH-GR3-L12'
  }
});

console.log('Session started:', session.sessionId);
```

---

## 10. Testing & Sandbox

### 10.1 Sandbox Environment

```
https://sandbox.api.wiastandards.com/edu-robot/v1
```

### 10.2 Test Credentials

```
Robot ID: EDU-ROBOT-TEST-001
API Key: test_key_abc123xyz789
```

### 10.3 Postman Collection

Download: `https://wiastandards.com/postman/edu-robot-v1.json`

---

## 11. Conformance

A system is conformant with WIA-EDU-007 Phase 2 if it:

✅ Implements all core endpoints
✅ Uses OAuth 2.0 + DID for authentication
✅ Returns JSON-LD responses
✅ Implements rate limiting
✅ Supports pagination
✅ Provides webhook notifications
✅ Returns standard error codes
✅ Implements COPPA/GDPR compliance checks

---

**弘益人間 (Benefit All Humanity)**

*WIA - World Certification Industry Association*
*© 2025 MIT License*


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

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-2-API-INTERFACE.
It is non-normative; the rules below describe the policy that the WIA
Standards working group commits to when amending this PHASE document.

- **Semantic versioning** — major / minor / patch components follow
  Semantic Versioning 2.0.0 (https://semver.org/spec/v2.0.0.html).
  Major bump indicates a backwards-incompatible change to a normative
  requirement; minor bump indicates new normative requirements that do
  not break existing implementations; patch bump indicates editorial
  changes only (clarifications, typo fixes, formatting).
- **Deprecation window** — when a normative requirement is removed or
  altered in a backwards-incompatible way, the prior major version is
  maintained in parallel for at least 180 days. During the parallel
  window, both major versions are marked Stable in the WIA Standards
  registry and either may be cited as "WIA-conformant".
- **Sunset notification** — deprecated major versions enter a 12-month
  sunset window during which the WIA registry marks the version as
  Deprecated. The deprecation entry includes a migration note pointing
  to the replacement requirement(s) and an explanation of why the
  change was made.
- **Editorial errata** — patch-level errata are issued without a
  deprecation window because they do not change normative behaviour.
  Errata are tracked in a public errata register and each entry is
  signed by the WIA Standards working group chair.
- **Implementation changelog mapping** — implementations SHOULD publish
  a changelog mapping each PHASE version they support to the specific
  build, container digest, or SDK version that satisfies the version.
  This allows downstream auditors to verify version conformance without
  re-running the entire test matrix on every release.

The policy is reviewed at the same cadence as the PHASE document and
any changes to the policy itself are tracked in the version-history
table at the start of the document.

## Annex I — Interoperability Profiles

This annex describes how implementations declare interoperability profiles
for PHASE-2-API-INTERFACE. The profile mechanism is non-normative and exists so that
deployments of varying scope (single tenant, regional cluster, federated
network) can advertise the subset of normative requirements they satisfy
without misrepresenting partial conformance as full conformance.

- **Profile manifest** — every implementation publishes a profile manifest
  in JSON. The manifest enumerates the normative requirement IDs from this
  PHASE that are satisfied (`status: "supported"`), partially satisfied
  (`status: "partial"`, with a reason field), or excluded
  (`status: "excluded"`, with a justification). The manifest is signed
  using the same Sigstore key used for the SBOM in Annex G.
- **Federation profile** — federated deployments publish an aggregated
  manifest summarizing the union and intersection of member-implementation
  profiles. The aggregated manifest is consumed by directory services so
  that callers can route a request to the least common denominator profile
  required for an interaction.
- **Backwards-profile compatibility** — when a deployment migrates from one
  profile to a wider profile, the prior profile manifest remains valid and
  signed for the deprecation window defined in Annex H. This preserves
  audit traceability for auditors evaluating long-term interoperability.
- **Profile registry** — the WIA Standards working group maintains a
  public registry of named profiles. Common deployment shapes (e.g.,
  "Edge-only", "Federated-with-replay") are added to the registry by
  consensus. Registry entries are immutable; new shapes are added under
  new names rather than amending existing entries.
- **Profile versioning** — profile names are versioned with the same
  Semantic Versioning rules described in Annex H. A deployment that
  advertises `WIA-P2-API-INTERFACE-Edge-only/2` is asserting conformance with
  the second major version of the named profile, not the second deployment
  of an unversioned profile.

The profile mechanism is intentionally lightweight; it is meant to make
real deployment shapes visible without forcing every deployment to
satisfy every normative requirement.

## Annex J — Reference Implementation Topology

The reference implementation topology described in this annex is
non-normative; it documents the deployment shape that the WIA
Standards working group used to validate the test vectors in Annex G
and is intended as a starting point, not a recommendation against
alternative topologies.

- **Single-tenant edge** — one runtime per organization, no shared
  state. Used for early-pilot deployments where conformance evidence
  is published manually. Sufficient for PHASE-2-API-INTERFACE validation when the
  organization signs the manifest itself.
- **Multi-tenant gateway** — one shared runtime serves multiple
  tenants via header-based isolation. Typically backed by a
  rate-limited gateway (Envoy or NGINX) and a shared OAuth 2.1
  identity provider. The manifest is per-tenant; the runtime
  publishes a federation manifest that aggregates tenant manifests.
- **Federated mesh** — multiple runtimes peer to one another and
  publish their manifests to a directory service. Each peer signs
  its own manifest; the directory service signs the aggregated
  index. This is the topology used by cross-organization deployments
  that need to compose conformance.
- **Air-gapped batch** — no network connection between the runtime
  and the directory service. The runtime emits a signed evidence
  package on each batch and the operator transports the package via
  out-of-band channels. This is the topology used by regulators that
  prohibit live connectivity from sensitive environments.

Implementations declare their topology in the manifest (see Annex I).
A topology change MUST be reflected in a new manifest signature; the
prior topology's manifest remains valid for the deprecation window
described in Annex H to preserve audit traceability.
