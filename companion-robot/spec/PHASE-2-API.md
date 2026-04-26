# Phase 2: API Interface Specification - WIA-ROB-013

## WIA-ROB-013 Companion Robot API Standard

**Version**: 1.0.0  
**Date**: 2025-01-15  
**Status**: Active  
**Standard ID**: WIA-ROB-013-PHASE2-001

---

## 1. API Overview

This phase defines standard APIs for companion robot systems enabling interoperability, integration, and cross-platform compatibility. All WIA-ROB-013 compliant systems must implement these core endpoints.

### 1.1 Base Requirements

- RESTful design principles
- JSON request/response format
- HTTP/2 or HTTP/3 support
- TLS 1.3+ for all connections
- JWT-based authentication
- Rate limiting and quota management
- Comprehensive error handling
- API versioning support

### 1.2 Base URL Structure

```
https://api.companion.example.com/v1/{resource}
```

---

## 2. Authentication

### 2.1 JWT Token Structure

```typescript
interface JWTPayload {
  sub: string;           // User ID
  cid: string;           // Companion ID
  iat: number;           // Issued at
  exp: number;           // Expiration
  scope: string[];       // Permissions
}
```

### 2.2 Authentication Endpoints

**Login**
```
POST /v1/auth/login
Content-Type: application/json

{
  "email": "user@example.com",
  "password": "secure_password"
}

Response: 200 OK
{
  "token": "eyJhbGc...",
  "refreshToken": "eyJhbGc...",
  "expiresIn": 3600
}
```

---

## 3. Session Management

### 3.1 Create Session

```
POST /v1/sessions
Authorization: Bearer {token}
Content-Type: application/json

{
  "companionId": "uuid",
  "preferences": {
    "language": "en",
    "notificationsEnabled": true
  }
}

Response: 201 Created
{
  "sessionId": "uuid",
  "companionId": "uuid",
  "createdAt": "2025-01-15T10:00:00Z",
  "expiresAt": "2025-01-15T18:00:00Z"
}
```

### 3.2 Get Session

```
GET /v1/sessions/{sessionId}
Authorization: Bearer {token}

Response: 200 OK
{
  "sessionId": "uuid",
  "status": "active",
  "messageCount": 42,
  "duration": 3600
}
```

---

## 4. Messaging API

### 4.1 Send Message

```
POST /v1/sessions/{sessionId}/messages
Authorization: Bearer {token}
Content-Type: application/json

{
  "text": "I'm feeling overwhelmed today",
  "language": "en",
  "context": {
    "location": "home",
    "activity": "relaxing",
    "timeOfDay": "evening"
  }
}

Response: 200 OK
{
  "messageId": "uuid",
  "companionResponse": {
    "text": "I hear that you're feeling overwhelmed...",
    "emotionalTone": {
      "valence": 0.4,
      "arousal": 0.3,
      "dominance": 0.2
    },
    "suggestions": [
      "Would you like to talk about what's overwhelming you?",
      "Sometimes a short walk can help clear your mind.",
      "Let's try a brief breathing exercise together."
    ]
  },
  "processingTime": 850
}
```

### 4.2 Get Conversation History

```
GET /v1/sessions/{sessionId}/messages?limit=50&offset=0
Authorization: Bearer {token}

Response: 200 OK
{
  "messages": [...],
  "pagination": {
    "limit": 50,
    "offset": 0,
    "total": 150
  }
}
```

---

## 5. Emotion Analysis API

### 5.1 Analyze Emotion

```
POST /v1/emotions/analyze
Authorization: Bearer {token}
Content-Type: application/json

{
  "text": "I can't believe how happy I am!",
  "voice": "base64_encoded_audio",
  "language": "en"
}

Response: 200 OK
{
  "primaryEmotion": "joy",
  "confidence": 0.92,
  "secondaryEmotions": [
    {"emotion": "excitement", "probability": 0.78},
    {"emotion": "contentment", "probability": 0.45}
  ],
  "dimensions": {
    "valence": 0.85,
    "arousal": 0.72,
    "dominance": 0.55
  },
  "sources": {
    "text": {"emotion": "joy", "confidence": 0.88},
    "voice": {"emotion": "joy", "confidence": 0.96}
  }
}
```

---

## 6. Personality Management API

### 6.1 Get Companion Personality

```
GET /v1/companions/{companionId}/personality
Authorization: Bearer {token}

Response: 200 OK
{
  "personalityTraits": {
    "openness": 75,
    "conscientiousness": 80,
    "extraversion": 65,
    "agreeableness": 90,
    "neuroticism": 25,
    "playfulness": 60,
    "formality": 40,
    "proactivity": 70
  },
  "communicationStyle": "warm and supportive",
  "lastUpdated": "2025-01-15T10:00:00Z"
}
```

### 6.2 Update Personality Traits

```
PATCH /v1/companions/{companionId}/personality
Authorization: Bearer {token}
Content-Type: application/json

{
  "personalityTraits": {
    "extraversion": 70,
    "playfulness": 65
  }
}

Response: 200 OK
{
  "updated": true,
  "personalityTraits": {...}
}
```

---

## 7. Memory and Context API

### 7.1 Store Memory

```
POST /v1/memories
Authorization: Bearer {token}
Content-Type: application/json

{
  "type": "preference",
  "content": "User prefers morning check-ins",
  "importance": 0.8
}

Response: 201 Created
{
  "memoryId": "uuid",
  "stored": true
}
```

### 7.2 Query Memories

```
GET /v1/memories?type=preference&limit=10
Authorization: Bearer {token}

Response: 200 OK
{
  "memories": [...]
}
```

---

## 8. WebSocket API

### 8.1 Connection

```
wss://api.companion.example.com/v1/stream?sessionId={sessionId}&token={jwt}
```

### 8.2 Message Types

```typescript
// User message
{
  "type": "user_message",
  "payload": {
    "text": "Hello!",
    "language": "en"
  },
  "timestamp": "2025-01-15T10:00:00Z"
}

// Companion response
{
  "type": "companion_response",
  "payload": {
    "text": "Hi! How are you feeling today?",
    "emotionalTone": {...}
  },
  "timestamp": "2025-01-15T10:00:01Z"
}

// Emotion update
{
  "type": "emotion_update",
  "payload": {
    "primaryEmotion": "joy",
    "confidence": 0.85
  },
  "timestamp": "2025-01-15T10:00:02Z"
}
```

---

## 9. Error Handling

### 9.1 Standard Error Format

```json
{
  "error": {
    "code": "INVALID_REQUEST",
    "message": "Missing required field: text",
    "details": {
      "field": "text",
      "requirement": "must be non-empty string"
    },
    "timestamp": "2025-01-15T10:00:00Z"
  }
}
```

### 9.2 Error Codes

- `INVALID_REQUEST` (400)
- `UNAUTHORIZED` (401)
- `FORBIDDEN` (403)
- `NOT_FOUND` (404)
- `RATE_LIMIT_EXCEEDED` (429)
- `INTERNAL_ERROR` (500)
- `SERVICE_UNAVAILABLE` (503)

---

## 10. Rate Limiting

```
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 999
X-RateLimit-Reset: 1705320000
```

Limits:
- Free tier: 1,000 requests/hour
- Standard tier: 10,000 requests/hour
- Premium tier: 100,000 requests/hour

---

**WIA-ROB-013 PHASE 2 - API Interface Specification**  
© 2025 World Certification Industry Association  
弘益人間 · Benefit All Humanity


## Annex E — Implementation Notes for PHASE-2-API

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-2-API.

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
evidence for PHASE-2-API. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-2-api/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-2-API with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-2-API does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-2-API.
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
for PHASE-2-API. The profile mechanism is non-normative and exists so that
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
  advertises `WIA-P2-API-Edge-only/2` is asserting conformance with
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
  is published manually. Sufficient for PHASE-2-API validation when the
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
