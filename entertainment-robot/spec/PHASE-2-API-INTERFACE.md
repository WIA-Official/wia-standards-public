# WIA-EDU-025: API Reference

**Version:** 1.0.0
**Last Updated:** 2025-12-26

---

## Base URL

```
Production: https://api.wiastandards.com/entertainment-robot/v1
Sandbox: https://sandbox-api.wiastandards.com/entertainment-robot/v1
```

---

## Authentication

All API requests require authentication using OAuth 2.0 or DID (Decentralized Identifier).

### OAuth 2.0 Flow

```http
POST /oauth/token
Content-Type: application/json

{
  "grant_type": "client_credentials",
  "client_id": "your-client-id",
  "client_secret": "your-client-secret"
}
```

Response:
```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600
}
```

---

## REST API Endpoints

### Stories

#### GET /stories
List available interactive stories.

**Query Parameters:**
- `genre` (string): Filter by genre
- `ageRange` (string): Filter by target age range (e.g., "7-12")
- `language` (string): ISO 639-1 language code
- `page` (integer): Page number for pagination
- `limit` (integer): Results per page (max 100)

**Response:**
```json
{
  "stories": [
    {
      "storyId": "STORY-2025-001",
      "title": "The Magic Forest Adventure",
      "genre": "fantasy",
      "targetAgeRange": "7-12",
      "duration": 1200,
      "thumbnailUrl": "https://...",
      "rating": 4.8,
      "learningObjectives": ["critical-thinking", "decision-making"]
    }
  ],
  "pagination": {
    "page": 1,
    "limit": 20,
    "totalPages": 5,
    "totalItems": 95
  }
}
```

#### GET /stories/{storyId}
Get complete story data.

**Response:**
```json
{
  "@context": "https://wiastandards.com/contexts/entertainment-robot/v1",
  "@type": "InteractiveStory",
  "storyId": "STORY-2025-001",
  "title": "The Magic Forest Adventure",
  "chapters": [...],
  "characters": [...],
  "learningObjectives": [...]
}
```

#### POST /stories/{storyId}/sessions
Start a new story session.

**Request:**
```json
{
  "userId": "anonymous-hash-12345",
  "robotId": "ENT-ROBOT-001",
  "sessionMetadata": {
    "location": "home",
    "parentalSupervision": true
  }
}
```

**Response:**
```json
{
  "sessionId": "SESSION-2025-001",
  "startTime": "2025-12-26T10:00:00Z",
  "websocketUrl": "wss://ws.wiastandards.com/sessions/SESSION-2025-001",
  "expiresIn": 7200
}
```

### Performances

#### GET /performances
List available performance programs.

#### GET /performances/{performanceId}
Get complete performance program.

#### POST /performances/{performanceId}/schedule
Schedule a performance.

### Therapeutic Sessions

#### GET /therapeutic-protocols
List validated therapeutic protocols.

**Requires:** Professional credentials verification

#### POST /therapeutic-sessions
Create therapeutic session.

**Requires:** Professional supervision authorization

### Emotions

#### POST /emotions/detect
Process emotion detection data.

**Request:**
```json
{
  "sessionId": "SESSION-2025-001",
  "timestamp": "2025-12-26T10:30:00Z",
  "multimodalData": {
    "facial": {...},
    "voice": {...},
    "gesture": {...}
  }
}
```

**Response:**
```json
{
  "emotionalState": {
    "primary": "happy",
    "confidence": 0.87,
    "secondary": ["excited"],
    "engagementLevel": 0.85
  },
  "recommendedResponse": {
    "strategy": "encourage-continue",
    "tone": "enthusiastic-supportive"
  }
}
```

---

## WebSocket Protocol

### Connection

```javascript
const ws = new WebSocket('wss://ws.wiastandards.com/sessions/SESSION-2025-001');
ws.onopen = () => {
  ws.send(JSON.stringify({
    type: 'auth',
    token: 'your-access-token'
  }));
};
```

### Message Types

#### Story Progression
```json
{
  "type": "story-beat",
  "beatId": "B01",
  "text": "Once upon a time...",
  "characterId": "narrator",
  "emotionalTone": "mysterious"
}
```

#### User Choice
```json
{
  "type": "user-choice",
  "choiceId": "C01",
  "selectedOption": "O01",
  "timestamp": "2025-12-26T10:35:00Z"
}
```

#### Emotion Update
```json
{
  "type": "emotion-update",
  "emotionalState": {
    "primary": "happy",
    "confidence": 0.87
  }
}
```

#### Robot Action
```json
{
  "type": "robot-action",
  "actionType": "movement",
  "choreography": "celebratory-dance",
  "duration": 5
}
```

---

## Rate Limiting

- **Standard Tier:** 1000 requests/hour
- **Professional Tier:** 10,000 requests/hour
- **Enterprise Tier:** Custom limits

Rate limit headers:
```
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 847
X-RateLimit-Reset: 1640534400
```

---

## Error Codes

| Code | Meaning |
|------|---------|
| 400 | Bad Request - Invalid parameters |
| 401 | Unauthorized - Invalid or missing authentication |
| 403 | Forbidden - Insufficient permissions |
| 404 | Not Found - Resource doesn't exist |
| 429 | Too Many Requests - Rate limit exceeded |
| 500 | Internal Server Error |
| 503 | Service Unavailable |

Error response format:
```json
{
  "error": {
    "code": "INVALID_AGE_RANGE",
    "message": "Age range must be in format 'X-Y'",
    "details": "Received: '7-'",
    "timestamp": "2025-12-26T10:40:00Z",
    "requestId": "req-12345"
  }
}
```

---

## SDK Example

### TypeScript/JavaScript

```typescript
import { WIAEntertainmentRobot } from '@wia/entertainment-robot';

const client = new WIAEntertainmentRobot({
  apiKey: 'your-api-key',
  robotId: 'ENT-ROBOT-001'
});

// Start a story session
const session = await client.stories.startSession('STORY-2025-001', {
  userId: 'anonymous-hash-12345',
  parentalSupervision: true
});

// Listen for story events
session.on('storyBeat', (beat) => {
  console.log(`Narrator: ${beat.text}`);
});

// Make a choice
await session.makeChoice('C01', 'O01');

// Track emotions
session.on('emotionDetected', (emotion) => {
  console.log(`Child is feeling: ${emotion.primary}`);
});
```

---

**© 2025 WIA - World Certification Industry Association**


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
