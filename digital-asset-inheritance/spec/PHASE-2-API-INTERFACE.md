# WIA-LEG-006: Digital Asset Inheritance Standard
## Version 1.1 - Phase 2: API Interface Specification

**Status:** Final
**Published:** 2025-02-01
**Authors:** WIA Technical Committee on Digital Legacy
**License:** MIT License
**Requires:** v1.0 (Phase 1 - Data Format)

---

## 1. Introduction

### 1.1 Purpose
Phase 2 defines RESTful APIs enabling programmatic interaction with digital asset inheritance systems built on Phase 1 data formats.

### 1.2 Scope
- HTTP REST API endpoints
- Authentication and authorization
- Request/response formats
- Webhook event system
- Error handling
- Rate limiting

---

## 2. API Architecture

### 2.1 Base URL Structure
```
https://api.{provider}.com/wia/leg-006/v1/{resource}
```

### 2.2 Authentication
- **Method:** OAuth 2.0 with JWT tokens
- **Token Lifetime:** 1 hour (access), 30 days (refresh)
- **Scopes:**
  - `plan:read` - Read inheritance plans
  - `plan:write` - Create/update plans
  - `plan:execute` - Trigger inheritance execution
  - `asset:discover` - Asset discovery operations
  - `beneficiary:verify` - Beneficiary verification

### 2.3 Additional Security
- Multi-Factor Authentication for sensitive operations
- Biometric verification for plan modifications
- Hardware security keys for emergency access

---

## 3. Core Endpoints

### 3.1 Plan Management

#### POST /plans
Create new inheritance plan
- **Auth:** OAuth 2.0 + MFA
- **Request:** Phase 1 inheritance plan JSON
- **Response:** `planId`, `version`, `createdAt`, `backupLocations`

#### GET /plans/{planId}
Retrieve inheritance plan
- **Auth:** OAuth 2.0
- **Query Params:**
  - `version` (optional): Specific version
  - `include` (optional): Sections to include
  - `decrypt` (optional): Decrypt sensitive fields
- **Response:** Complete plan data

#### PUT /plans/{planId}
Update existing plan
- **Auth:** OAuth 2.0 + MFA + Biometric
- **Request:** Change set with current version
- **Response:** New version number, change summary

#### DELETE /plans/{planId}
Revoke inheritance plan
- **Auth:** OAuth 2.0 + MFA + Confirmation phrase
- **Request:** Revocation reason
- **Response:** Revocation confirmation

### 3.2 Asset Discovery

#### POST /assets/discover
Scan for digital assets
- **Auth:** OAuth 2.0
- **Request:**
  ```json
  {
    "owner": "did:wia:owner:...",
    "searchCriteria": {
      "walletAddresses": [],
      "blockchains": [],
      "platforms": [],
      "timeRange": {}
    },
    "includeHistory": boolean,
    "estimateValues": boolean
  }
  ```
- **Response:** Discovered assets array, total estimated value

### 3.3 Beneficiary Management

#### POST /beneficiaries/verify
Verify beneficiary identity and rights
- **Auth:** OAuth 2.0
- **Request:** Beneficiary DID, credentials, verification level
- **Response:** Verification status, entitlements, next steps

### 3.4 Trigger Management

#### POST /triggers/checkin
Owner check-in to reset dead man's switch
- **Auth:** OAuth 2.0 + Biometric
- **Request:** Plan ID, authentication data, location
- **Response:** Confirmation, next check-in due date

#### POST /triggers/execute
Trigger inheritance execution
- **Auth:** OAuth 2.0 + Multi-party signatures
- **Request:** Plan ID, trigger reason, documentation, executor signatures
- **Response:** Execution ID, validation results, timeline

---

## 4. Webhook System

### 4.1 Event Types
- `plan.created` - New plan created
- `plan.updated` - Plan modified
- `plan.revoked` - Plan revoked
- `trigger.warning` - Inactivity warning
- `trigger.activated` - Inheritance triggered
- `distribution.initiated` - Asset distribution started
- `distribution.completed` - Distribution finished
- `dispute.filed` - Inheritance dispute filed

### 4.2 Webhook Configuration

#### POST /webhooks
Register webhook endpoint
- **Request:**
  ```json
  {
    "url": "https://...",
    "events": ["trigger.warning", "distribution.completed"],
    "authentication": {
      "method": "hmac-sha256",
      "secret": "..."
    },
    "retryPolicy": {
      "maxAttempts": 5,
      "backoff": "exponential"
    }
  }
  ```

### 4.3 Webhook Payload
```json
{
  "event": "trigger.activated",
  "timestamp": "2025-01-15T10:30:00Z",
  "planId": "...",
  "data": {},
  "signature": "HMAC-SHA256 signature"
}
```

---

## 5. Error Handling

### 5.1 Error Response Format
```json
{
  "error": {
    "code": "ERROR_CODE",
    "message": "Human-readable message",
    "details": {},
    "timestamp": "ISO 8601",
    "requestId": "UUID",
    "documentation": "https://docs.wiastandards.com/..."
  }
}
```

### 5.2 Standard Error Codes
- `400 INVALID_REQUEST` - Malformed request
- `401 UNAUTHORIZED` - Missing/invalid authentication
- `403 INSUFFICIENT_AUTH` - Additional verification required
- `404 NOT_FOUND` - Resource doesn't exist
- `409 CONFLICT` - Version conflict
- `422 VALIDATION_ERROR` - Data validation failed
- `429 RATE_LIMIT` - Too many requests
- `500 INTERNAL_ERROR` - Server error

---

## 6. Rate Limiting

### 6.1 Rate Limits by Endpoint
- Plan creation: 10 per hour
- Plan updates: 100 per hour
- Check-ins: 1000 per hour
- Asset discovery: 10 per day
- Webhook registration: 100 per day

### 6.2 Rate Limit Headers
```
X-RateLimit-Limit: 100
X-RateLimit-Remaining: 95
X-RateLimit-Reset: 1640995200
```

---

## 7. Versioning

### 7.1 API Version Strategy
- Version in URL path (`/v1/`, `/v2/`)
- Breaking changes require new major version
- Backward compatible changes increment minor version
- Old versions supported for minimum 24 months

### 7.2 Deprecation Policy
- 12 months advance notice for deprecations
- Deprecated endpoints return `Sunset` header
- Migration guides provided

---

## 8. Security Requirements

### 8.1 Transport Security
- TLS 1.3 or higher required
- Certificate pinning recommended
- HSTS headers mandatory

### 8.2 Request Signing
- All mutating operations require request signatures
- Signature algorithm: Ed25519 or ECDSA
- Signature includes timestamp, prevents replay attacks

### 8.3 Data Privacy
- GDPR compliance: Right to erasure after distribution
- Data minimization: Collect only necessary information
- Encryption at rest and in transit

---

## 9. Compliance Features

### 9.1 Audit Logging
- All API calls logged with timestamps
- Immutable audit trail
- Accessible via `/audit-log` endpoint

### 9.2 Legal Requirements
- Support for jurisdictional data localization
- Ability to generate compliance reports
- Integration with tax reporting systems

---

## 10. Client Libraries

### 10.1 Official SDKs
- JavaScript/TypeScript (Browser + Node.js)
- Python 3.8+
- Java 11+
- Swift (iOS)
- Kotlin (Android)

### 10.2 SDK Features
- Automatic authentication token refresh
- Retry logic with exponential backoff
- Type-safe request/response models
- Built-in validation

---

## 11. Testing

### 11.1 Sandbox Environment
- URL: `https://api-sandbox.wiastandards.com/wia/leg-006/v1/`
- Test credentials provided
- Synthetic blockchain environments
- No real asset transfers

### 11.2 API Testing Tools
- Postman collection available
- OpenAPI 3.0 specification provided
- Automated integration tests in GitHub

---

## 12. References

- OAuth 2.0: RFC 6749
- JSON Web Tokens: RFC 7519
- OpenAPI Specification: https://spec.openapis.org/oas/v3.0.0
- RESTful API Design: https://restfulapi.net/

---

**弘益人間 · Benefit All Humanity**

© 2025 World Certification Industry Association
Licensed under MIT License


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
