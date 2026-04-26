# WIA-EDU-023: API Reference
## Cultural Heritage Digitization Standard

**Version:** 1.0.0
**Last Updated:** 2025-01-26

---

## 1. Overview

This document specifies the RESTful API for cultural heritage digitization systems complying with WIA-EDU-023.

### Base URL
```
https://api.example.com/wia/v1/heritage
```

### Authentication
```http
Authorization: Bearer <access_token>
```

---

## 2. Core Resources

### 2.1 Artifacts

#### Get Artifact
```http
GET /artifacts/{id}
```

**Response:**
```json
{
  "id": "artifact-001",
  "wiaId": "WIA-EDU-023-000001",
  "title": "Tang Dynasty Vase",
  "culture": "Tang Dynasty China",
  "period": "618-907 CE",
  "materials": ["Glazed ceramic"],
  "dimensions": {"height": 30, "width": 20, "depth": 20, "unit": "cm"},
  "location": {"current": "Xi'an Museum", "origin": "Xi'an, Shaanxi"},
  "3dModels": {
    "glb": "https://cdn.example.com/models/artifact-001.glb",
    "usdz": "https://cdn.example.com/models/artifact-001.usdz"
  },
  "metadata": {
    "dublinCore": {...},
    "cidocCrm": {...},
    "technical": {...}
  }
}
```

#### List Artifacts
```http
GET /artifacts?culture=Tang&limit=20&offset=0
```

#### Create Artifact
```http
POST /artifacts
Content-Type: application/json

{
  "title": "Ancient Scroll",
  "culture": "Song Dynasty",
  ...
}
```

### 2.2 3D Models

#### Upload 3D Model
```http
POST /artifacts/{id}/models
Content-Type: multipart/form-data

file: artifact.glb
format: glTF-2.0
resolution: high
```

#### Get Model Metadata
```http
GET /artifacts/{id}/models/{modelId}
```

### 2.3 Virtual Tours

#### Create Tour
```http
POST /tours

{
  "name": "Ancient Egypt Gallery",
  "type": "vr",
  "artifacts": ["artifact-001", "artifact-002"],
  "scenes": [...]
}
```

---

## 3. Metadata Operations

### 3.1 Dublin Core

```http
GET /artifacts/{id}/metadata/dublincore
PUT /artifacts/{id}/metadata/dublincore
```

### 3.2 CIDOC-CRM

```http
GET /artifacts/{id}/metadata/cidoc
```

---

## 4. Search & Discovery

```http
GET /search?q=pottery&culture=Roman&period=100BCE-400CE
```

---

## 5. Analytics

```http
GET /artifacts/{id}/analytics
```

---

© 2025 WIA - MIT License


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
