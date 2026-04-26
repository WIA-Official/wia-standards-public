# WIA-MED-028 API Specification

**Version:** 1.0.0
**Base URL:** `https://api.wia-health.org/v1`
**Protocol:** REST + EDI X12
**Authentication:** OAuth 2.0 + Client Credentials

---

## Authentication

### OAuth 2.0 Token Endpoint

```http
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials
&client_id=YOUR_CLIENT_ID
&client_secret=YOUR_CLIENT_SECRET
&scope=claims eligibility prior_auth
```

**Response:**
```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIs...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "claims eligibility prior_auth"
}
```

### Using Access Token

```http
Authorization: Bearer eyJhbGciOiJSUzI1NiIs...
```

---

## API Endpoints

### 1. Eligibility Verification (270/271)

#### Check Eligibility

```http
POST /eligibility/check
Content-Type: application/json
Authorization: Bearer {token}

{
  "subscriber": {
    "memberId": "ABC123456",
    "firstName": "John",
    "lastName": "Smith",
    "dateOfBirth": "1980-05-15"
  },
  "provider": {
    "npi": "1234567890",
    "taxId": "12-3456789"
  },
  "serviceDate": "2025-01-26",
  "serviceType": "30"
}
```

**Response (200 OK):**
```json
{
  "transactionId": "ELG-20250126-001",
  "eligibilityStatus": "active",
  "subscriber": {
    "memberId": "ABC123456",
    "name": "John Smith",
    "dateOfBirth": "1980-05-15"
  },
  "coverage": {
    "planName": "Blue Cross PPO",
    "effectiveDate": "2025-01-01",
    "terminationDate": "2025-12-31",
    "inNetwork": true
  },
  "benefits": {
    "copay": {
      "office": 25.00,
      "specialist": 50.00,
      "emergency": 150.00
    },
    "deductible": {
      "individual": 1500.00,
      "family": 3000.00,
      "remaining": 1440.00
    },
    "coinsurance": {
      "inNetwork": "80-20",
      "outOfNetwork": "60-40"
    },
    "outOfPocketMax": {
      "individual": 5000.00,
      "family": 10000.00,
      "remaining": 4898.00
    }
  },
  "priorAuthRequired": {
    "mri": true,
    "ct": true,
    "surgery": true
  }
}
```

---

### 2. Claim Submission (837)

#### Submit Professional Claim

```http
POST /claims/submit
Content-Type: application/json
Authorization: Bearer {token}

{
  "claimType": "professional",
  "claimId": "CLM-2025-001",
  "billingProvider": {
    "npi": "1234567890",
    "name": "City Medical Clinic",
    "taxId": "12-3456789",
    "address": {
      "street": "123 Main St",
      "city": "Anytown",
      "state": "CA",
      "zip": "90210"
    }
  },
  "subscriber": {
    "memberId": "ABC123456",
    "firstName": "John",
    "lastName": "Smith",
    "dateOfBirth": "1980-05-15",
    "relationship": "self"
  },
  "payer": {
    "payerId": "12345",
    "name": "Blue Cross Blue Shield"
  },
  "serviceLines": [
    {
      "lineNumber": 1,
      "serviceDate": "2025-01-15",
      "placeOfService": "11",
      "procedureCode": "99213",
      "diagnosisPointers": ["A"],
      "chargedAmount": 150.00,
      "units": 1
    },
    {
      "lineNumber": 2,
      "serviceDate": "2025-01-15",
      "placeOfService": "11",
      "procedureCode": "73030",
      "diagnosisPointers": ["A"],
      "chargedAmount": 80.00,
      "units": 1
    }
  ],
  "diagnoses": [
    {
      "sequence": "A",
      "code": "M25.561",
      "description": "Pain in right knee"
    }
  ],
  "totalCharged": 230.00
}
```

**Response (201 Created):**
```json
{
  "claimId": "CLM-2025-001",
  "transactionId": "837-20250126-001",
  "status": "accepted",
  "acceptedAt": "2025-01-26T14:30:00Z",
  "clearinghouse": {
    "id": "CLH-001",
    "name": "WIA Clearinghouse"
  },
  "estimatedProcessing": "7-14 days",
  "trackingUrl": "https://api.wia-health.org/v1/claims/CLM-2025-001/status"
}
```

---

### 3. Claim Status (276/277)

#### Check Claim Status

```http
GET /claims/{claimId}/status
Authorization: Bearer {token}
```

**Response (200 OK):**
```json
{
  "claimId": "CLM-2025-001",
  "status": "paid",
  "statusCategory": "finalized",
  "statusDate": "2025-01-20",
  "paidAmount": 78.00,
  "paidDate": "2025-01-20",
  "paymentMethod": "ACH",
  "checkNumber": "12345678",
  "remittanceAdvice": {
    "eraId": "ERA-20250120-001",
    "url": "https://api.wia-health.org/v1/remittance/ERA-20250120-001"
  }
}
```

---

### 4. Prior Authorization (278)

#### Request Prior Authorization

```http
POST /prior-auth/request
Content-Type: application/json
Authorization: Bearer {token}

{
  "requestType": "initial",
  "subscriber": {
    "memberId": "ABC123456",
    "firstName": "John",
    "lastName": "Smith",
    "dateOfBirth": "1980-05-15"
  },
  "provider": {
    "npi": "1234567890",
    "name": "Dr. Jane Doe"
  },
  "requestedService": {
    "procedureCode": "70553",
    "description": "MRI Brain with contrast",
    "quantity": 1,
    "dateOfService": "2025-02-15"
  },
  "diagnosis": [
    {
      "code": "G43.909",
      "description": "Migraine, unspecified"
    }
  ],
  "clinicalInformation": {
    "symptoms": "Severe headaches for 3 months",
    "previousTreatments": ["Pain medication", "Physical therapy"],
    "urgency": "routine"
  },
  "attachments": [
    {
      "type": "clinical_notes",
      "url": "https://provider.example.com/documents/note-123"
    }
  ]
}


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
