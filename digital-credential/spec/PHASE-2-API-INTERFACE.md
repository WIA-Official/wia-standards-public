# WIA-EDU-011 Digital Credential Standard v1.1

## Phase 2: API Interface & Integration

**Status:** ✅ Complete
**Version:** 1.1.0
**Date:** 2025-01-15
**Philosophy:** 弘익人間 (Benefit All Humanity)

---

## 1. Overview

Phase 2 extends Phase 1 by defining RESTful API specifications for credential issuance, verification, revocation, and wallet integration. This enables interoperability between credential issuers, holders, and verifiers.

## 2. Scope

Phase 2 covers:
- RESTful API endpoints
- Authentication and authorization
- Credential issuance protocol
- Verification protocol
- Revocation mechanisms
- Wallet integration
- Error handling and status codes

## 3. API Architecture

### 3.1 Base URL Structure

```
https://api.{institution}.edu/wia/v1/credentials
```

### 3.2 Authentication

**OAuth 2.0 + OpenID Connect:**
- Authorization Code Flow for user authentication
- Client Credentials Flow for service-to-service
- JWT access tokens with 1-hour expiration

## 4. Credential Issuance API

### 4.1 Issue Credential

**Endpoint:** `POST /credentials/issue`

**Request:**
```json
{
  "credentialType": "BachelorDegree",
  "recipientDID": "did:wia:student:alice",
  "recipientEmail": "alice@example.com",
  "achievement": {
    "degree": "Bachelor of Science",
    "field": "Computer Science",
    "gpa": "3.85",
    "graduationDate": "2024-06-15"
  },
  "deliveryMethod": ["wallet", "email"]
}
```

**Response (201 Created):**
```json
{
  "credentialId": "did:wia:edu:mit:degree:12345",
  "credential": { /* full VC JSON-LD */ },
  "blockchainTxHash": "0x...",
  "qrCode": "data:image/png;base64,...",
  "downloadUrl": "https://credentials.mit.edu/download/12345"
}
```

## 5. Credential Verification API

### 5.1 Verify Credential

**Endpoint:** `POST /credentials/verify`

**Request:**
```json
{
  "credential": { /* VC JSON-LD */ },
  "verificationOptions": {
    "checkRevocation": true,
    "checkExpiration": true,
    "trustRegistry": ["wia-global", "us-chea"]
  }
}
```

**Response (200 OK):**
```json
{
  "verified": true,
  "checks": [
    {"type": "signature", "status": "passed"},
    {"type": "revocation", "status": "passed"},
    {"type": "expiration", "status": "passed"},
    {"type": "trustRegistry", "status": "passed"}
  ],
  "issuer": {
    "did": "did:wia:edu:mit",
    "name": "MIT",
    "verified": true
  },
  "credentialSubject": {
    "achievement": "Bachelor of Science in Computer Science"
  }
}
```

## 6. Credential Revocation API

### 6.1 Revoke Credential

**Endpoint:** `POST /credentials/revoke`

**Request:**
```json
{
  "credentialId": "did:wia:edu:mit:degree:12345",
  "reason": "Credential issued in error",
  "revocationDate": "2024-12-01T00:00:00Z"
}
```

**Response (200 OK):**
```json
{
  "credentialId": "did:wia:edu:mit:degree:12345",
  "revoked": true,
  "revocationDate": "2024-12-01T00:00:00Z",
  "blockchainTxHash": "0x..."
}
```

### 6.2 Check Revocation Status

**Endpoint:** `GET /credentials/{id}/status`

**Response:**
```json
{
  "credentialId": "did:wia:edu:mit:degree:12345",
  "status": "active",
  "issuanceDate": "2024-06-15T00:00:00Z",
  "lastChecked": "2025-01-15T10:30:00Z"
}
```

## 7. Wallet Integration APIs

### 7.1 Credential Presentation Request

**Endpoint:** `POST /presentations/request`

**Request:**
```json
{
  "verifier": "did:wia:employer:acme",
  "credentialTypes": ["BachelorDegree", "MasterDegree"],
  "selectiveDisclosure": {
    "required": ["degree", "field", "graduationDate"],
    "optional": ["gpa", "honors"]
  }
}
```

### 7.2 Submit Presentation

**Endpoint:** `POST /presentations/submit`

**Request:**
```json
{
  "presentation": { /* VP JSON-LD */ },
  "challenge": "abc123...",
  "domain": "employer.acme.com"
}
```

## 8. Batch Operations

### 8.1 Batch Issuance

**Endpoint:** `POST /credentials/batch/issue`

**Request:**
```json
{
  "credentials": [
    { /* credential 1 data */ },
    { /* credential 2 data */ },
    { /* ... */ }
  ],
  "blockchainBatch": true
}
```

## 9. Error Handling

### 9.1 Error Response Format

```json
{
  "error": {
    "code": "INVALID_SIGNATURE",
    "message": "Cryptographic signature verification failed",
    "details": {
      "verificationMethod": "did:wia:edu:mit#key-1",
      "algorithm": "Ed25519Signature2020"
    }
  }
}
```

### 9.2 HTTP Status Codes

- `200 OK` - Successful operation
- `201 Created` - Credential issued
- `400 Bad Request` - Invalid request data
- `401 Unauthorized` - Authentication required
- `403 Forbidden` - Insufficient permissions
- `404 Not Found` - Credential not found
- `409 Conflict` - Credential already exists
- `422 Unprocessable Entity` - Validation failed
- `500 Internal Server Error` - Server error

## 10. Rate Limiting

**Default Limits:**
- Issuance: 100 requests/hour
- Verification: 1000 requests/hour
- Revocation: 50 requests/hour

**Headers:**
```
X-RateLimit-Limit: 100
X-RateLimit-Remaining: 95
X-RateLimit-Reset: 1640995200
```

## 11. Webhooks

### 11.1 Credential Status Changes

**Event:** `credential.revoked`

**Payload:**
```json
{
  "event": "credential.revoked",
  "credentialId": "did:wia:edu:mit:degree:12345",
  "timestamp": "2024-12-01T00:00:00Z",
  "reason": "Credential issued in error"
}
```

## 12. API Security

### 12.1 HTTPS Only

- All API endpoints MUST use HTTPS/TLS 1.3
- HTTP connections MUST redirect to HTTPS

### 12.2 API Key Management

- Rotate API keys every 90 days
- Store keys in secure key management systems
- Never log API keys

## 13. Compliance

Phase 2 APIs MUST comply with:
- ✅ OpenAPI 3.1 specification
- ✅ OAuth 2.0 / OpenID Connect
- ✅ RESTful API best practices
- ✅ GDPR data protection requirements
- ✅ W3C VC HTTP API specification

---

**Philosophy:** 弘益人間 (Benefit All Humanity)

*WIA-EDU-011 Digital Credential Standard*

© 2025 MIT License


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
