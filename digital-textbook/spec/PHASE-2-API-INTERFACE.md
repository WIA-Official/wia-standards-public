# WIA-EDU-008 Digital Textbook Standard v1.1

## Phase 2: API Interface & Integration

**Status:** ✅ Complete
**Version:** 1.1.0
**Date:** 2025-02-15
**Philosophy:** 弘益人間 (Benefit All Humanity)
**Builds on:** v1.0 (Phase 1)

---

## 1. Overview

Phase 2 extends the WIA Digital Textbook Standard with comprehensive API specifications for content delivery, user data management, and learning management system integration. This phase enables seamless integration across platforms and ecosystems.

## 2. RESTful API Specification

### 2.1 Base URL Structure

```
https://api.{provider}.com/wia/v1/
```

### 2.2 Core Endpoints

#### Textbook Discovery and Metadata

```
GET    /textbooks                    # List available textbooks
GET    /textbooks/{id}               # Get textbook metadata
GET    /textbooks/{id}/toc           # Get table of contents
GET    /textbooks/{id}/chapters      # List chapters
GET    /textbooks/{id}/chapters/{num} # Get chapter content
```

#### Content Delivery

```
GET    /textbooks/{id}/download      # Download full EPUB
GET    /textbooks/{id}/cover         # Get cover image
GET    /textbooks/{id}/sample        # Get sample chapters
```

#### User Data Management

```
GET    /users/{userId}/library       # User's textbook library
POST   /users/{userId}/library       # Add textbook to library
DELETE /users/{userId}/library/{id}  # Remove from library
```

#### Annotations

```
GET    /annotations                  # List user annotations
POST   /annotations                  # Create annotation
GET    /annotations/{id}             # Get specific annotation
PUT    /annotations/{id}             # Update annotation
DELETE /annotations/{id}             # Delete annotation
```

#### Reading Progress

```
GET    /progress/{textbookId}        # Get reading progress
POST   /progress/{textbookId}        # Update reading progress
```

#### Analytics

```
GET    /analytics/reading            # Reading analytics
GET    /analytics/engagement         # Engagement metrics
GET    /analytics/assessment         # Assessment results
```

### 2.3 Request/Response Format

All requests and responses use JSON with UTF-8 encoding.

**Standard Headers:**
```
Content-Type: application/json
Accept: application/json
Authorization: Bearer {access_token}
X-WIA-Standard: WIA-EDU-008
X-WIA-Version: 1.1
```

### 2.4 Example: Get Textbook Metadata

**Request:**
```http
GET /textbooks/978-3-16-148410-0 HTTP/1.1
Host: api.textbooks.com
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
Accept: application/json
```

**Response:**
```json
{
  "id": "978-3-16-148410-0",
  "wiaId": "WIA-EDU-008-TB-00123456",
  "title": "Introduction to Physics",
  "subtitle": "Mechanics and Thermodynamics",
  "authors": [
    {
      "name": "Dr. Jane Smith",
      "orcid": "0000-0002-1825-0097"
    },
    {
      "name": "Dr. John Doe",
      "orcid": "0000-0001-5678-1234"
    }
  ],
  "publisher": {
    "name": "Academic Press",
    "wiaId": "WIA-PUB-00789"
  },
  "edition": "3rd Edition",
  "publicationDate": "2025-01-15",
  "language": ["en", "es"],
  "isbn": "978-3-16-148410-0",
  "format": "EPUB3",
  "fileSize": 127834289,
  "pageCount": 456,
  "chapterCount": 15,
  "wiaCompliant": true,
  "wcagLevel": "AA",
  "subjects": ["Physics", "Mechanics", "Thermodynamics"],
  "educationLevel": "Grade 11-12",
  "features": {
    "hasVideo": true,
    "hasAudio": true,
    "hasInteractive": true,
    "hasMathML": true,
    "hasAssessments": true
  },
  "links": {
    "download": "/textbooks/978-3-16-148410-0/download",
    "cover": "/textbooks/978-3-16-148410-0/cover",
    "sample": "/textbooks/978-3-16-148410-0/sample",
    "toc": "/textbooks/978-3-16-148410-0/toc"
  }
}
```

## 3. Authentication & Authorization

### 3.1 OAuth 2.0 Support (Required)

Implement OAuth 2.0 Authorization Code Flow with PKCE.

**Authorization Endpoint:**
```
GET https://auth.provider.com/oauth/authorize?
  response_type=code&
  client_id=YOUR_CLIENT_ID&
  redirect_uri=https://yourapp.com/callback&
  scope=read:textbooks write:annotations read:progress&
  state=xyz123&
  code_challenge=CHALLENGE&
  code_challenge_method=S256
```

**Token Endpoint:**
```http
POST /oauth/token HTTP/1.1
Content-Type: application/x-www-form-urlencoded

grant_type=authorization_code&
code=AUTH_CODE&
client_id=YOUR_CLIENT_ID&
redirect_uri=https://yourapp.com/callback&
code_verifier=VERIFIER
```

**Token Response:**
```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "refresh_token": "def50200...",
  "scope": "read:textbooks write:annotations read:progress"
}
```

### 3.2 OpenID Connect Support (Optional)

For user identity management, support OpenID Connect.

### 3.3 LTI 1.3 Integration (Required for LMS)

Implement IMS Global LTI 1.3 for learning management system integration.

**LTI Launch:**
- Deep linking for content selection
- Grade passback via Assignment and Grade Services
- Names and Role Provisioning Services

## 4. Annotation API Specification

### 4.1 Create Annotation

**Request:**
```http
POST /annotations HTTP/1.1
Content-Type: application/json
Authorization: Bearer {token}

{
  "textbookId": "978-3-16-148410-0",
  "chapterId": "chapter-05",
  "type": "highlight",
  "selector": {
    "type": "TextPositionSelector",
    "start": 1234,
    "end": 1456
  },
  "color": "#FFFF00",
  "note": "Important formula for exam",
  "tags": ["exam", "formulas", "chapter5"],
  "visibility": "private"
}
```

**Response:**
```json
{
  "id": "anno-12345678",
  "created": "2025-12-25T10:30:00Z",
  "modified": "2025-12-25T10:30:00Z",
  "syncStatus": "synced",
  "links": {
    "self": "/annotations/anno-12345678"
  }
}
```

### 4.2 List Annotations

```http
GET /annotations?textbookId=978-3-16-148410-0&chapter=chapter-05 HTTP/1.1
```

**Response:**
```json
{
  "total": 23,
  "page": 1,
  "perPage": 20,
  "annotations": [
    {
      "id": "anno-12345678",
      "type": "highlight",
      "color": "#FFFF00",
      "note": "Important formula for exam",
      "created": "2025-12-25T10:30:00Z",
      "selector": {...}
    }
  ]
}
```

## 5. Learning Analytics API

### 5.1 xAPI Statement Endpoint

```http
POST /xapi/statements HTTP/1.1
Content-Type: application/json
X-Experience-API-Version: 1.0.3

{
  "actor": {
    "objectType": "Agent",
    "name": "Alice Johnson",
    "mbox": "mailto:alice@example.com"
  },
  "verb": {
    "id": "http://adlnet.gov/expapi/verbs/completed",
    "display": {"en-US": "completed"}
  },
  "object": {
    "objectType": "Activity",
    "id": "https://textbooks.com/physics/chapter-05",
    "definition": {
      "name": {"en-US": "Chapter 5: Newton's Laws"}
    }
  },
  "result": {
    "completion": true,
    "success": true,
    "score": {"scaled": 0.92}
  }
}
```

## 6. LMS Integration

### 6.1 Grade Passback

```http
POST /lms/grades HTTP/1.1
Content-Type: application/json

{
  "lmsType": "canvas",
  "courseId": "PHYS101",
  "assignmentId": "quiz-ch5",
  "userId": "user123",
  "score": 0.92,
  "maxScore": 1.0,
  "timestamp": "2025-12-25T11:45:00Z",
  "comment": "Excellent understanding of Newton's Laws"
}
```

## 7. Error Handling

### 7.1 Standard Error Response

```json
{
  "error": {
    "code": "RESOURCE_NOT_FOUND",
    "message": "Textbook with ID 978-0-00-000000-0 not found",
    "details": {
      "requestId": "req-abc123",
      "timestamp": "2025-12-25T10:30:00Z"
    }
  }
}
```

### 7.2 HTTP Status Codes

- 200: Success
- 201: Created
- 204: No Content
- 400: Bad Request
- 401: Unauthorized
- 403: Forbidden
- 404: Not Found
- 429: Rate Limit Exceeded
- 500: Internal Server Error

## 8. Rate Limiting

```http
HTTP/1.1 200 OK
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 987
X-RateLimit-Reset: 1735132800
```

## 9. API Versioning

APIs MUST support versioning via URL path:
```
/v1/textbooks
/v2/textbooks
```

Backward compatibility MUST be maintained for at least 24 months.

---

**Philosophy:** 弘益人間 · Benefit All Humanity

© 2025 WIA - World Certification Industry Association


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
