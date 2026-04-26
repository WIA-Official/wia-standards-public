# WIA-EDU-008 Digital Textbook Standard v2.0

## Phase 4: WIA Ecosystem Integration

**Status:** ✅ Complete
**Version:** 2.0.0
**Date:** 2025-04-15
**Philosophy:** 弘익人間 (Benefit All Humanity)
**Builds on:** v1.0 (Phase 1), v1.1 (Phase 2), v1.2 (Phase 3)

---

## 1. Overview

Phase 4 integrates digital textbooks into the broader WIA ecosystem, enabling:
- Cross-standard interoperability
- Verifiable credentials for completion
- Global registry integration
- Certification and quality assurance
- International harmonization

## 2. WIA Registry Integration

### 2.1 Registration Requirement

All WIA-compliant textbooks MUST be registered in the global WIA Registry.

### 2.2 Registration API

**Submit for Registration:**
```http
POST https://registry.wia.org/api/v1/standards/EDU-008/textbooks HTTP/1.1
Content-Type: application/json
Authorization: Bearer {publisher_token}

{
  "isbn": "978-3-16-148410-0",
  "title": "Introduction to Physics",
  "publisher": {
    "name": "Academic Press",
    "wiaPublisherId": "WIA-PUB-00789"
  },
  "contentUrl": "https://textbooks.academic.com/physics-101.epub",
  "metadataUrl": "https://api.academic.com/textbooks/978-3-16-148410-0",
  "sampleUrl": "https://samples.academic.com/physics-101/",
  "certificationDocuments": [
    "https://certs.academic.com/physics-101/epubcheck.pdf",
    "https://certs.academic.com/physics-101/accessibility.pdf"
  ]
}
```

**Response:**
```json
{
  "wiaId": "WIA-EDU-008-TB-00123456",
  "status": "pending_review",
  "submittedAt": "2025-12-25T10:00:00Z",
  "estimatedReviewTime": "5-7 business days",
  "trackingUrl": "https://registry.wia.org/submissions/sub-789"
}
```

### 2.3 Registry Entry Format

```json
{
  "wiaId": "WIA-EDU-008-TB-00123456",
  "standard": "WIA-EDU-008",
  "version": "2.0",
  "isbn": "978-3-16-148410-0",
  "title": "Introduction to Physics",
  "subtitle": "Mechanics and Thermodynamics",
  "publisher": {
    "name": "Academic Press",
    "wiaId": "WIA-PUB-00789",
    "verified": true,
    "country": "US"
  },
  "certificationStatus": "certified",
  "certificationDate": "2025-01-15",
  "certificationExpiry": "2028-01-15",
  "accessibilityLevel": "WCAG-AA",
  "languages": ["en", "es", "fr"],
  "subjects": ["Physics", "Science", "STEM"],
  "educationLevel": ["Grade 11", "Grade 12", "University Freshman"],
  "formats": ["EPUB3"],
  "pricing": {
    "currency": "USD",
    "student": 49.99,
    "institutional": 29.99,
    "licensing": "perpetual"
  },
  "links": {
    "website": "https://textbooks.academic.com/physics-101",
    "sample": "https://samples.wia.org/physics-101/",
    "metadata": "https://api.academic.com/textbooks/978-3-16-148410-0"
  },
  "qualityMetrics": {
    "reviewScore": 4.7,
    "reviewCount": 234,
    "adoptionCount": 1247,
    "studentCount": 45623
  }
}
```

## 3. Verifiable Credentials Integration

### 3.1 Course Completion Credentials

Issue W3C Verifiable Credentials upon textbook/course completion.

**Credential Format:**
```json
{
  "@context": [
    "https://www.w3.org/2018/credentials/v1",
    "https://wia.org/credentials/education/v1"
  ],
  "type": ["VerifiableCredential", "CourseCompletionCredential"],
  "issuer": {
    "id": "did:wia:publisher:academic-press",
    "name": "Academic Press",
    "wiaId": "WIA-PUB-00789"
  },
  "issuanceDate": "2025-12-25T12:00:00Z",
  "expirationDate": null,
  "credentialSubject": {
    "id": "did:wia:student:user123",
    "name": "Alice Johnson",
    "studentId": "STU-2025-12345",
    "completedCourse": {
      "name": "Introduction to Physics",
      "isbn": "978-3-16-148410-0",
      "wiaId": "WIA-EDU-008-TB-00123456",
      "publisher": "Academic Press",
      "completionDate": "2025-12-25",
      "grade": "A",
      "gradeScale": "A-F",
      "percentageScore": 92,
      "credits": 3.0,
      "duration": "PT120H"
    },
    "achievements": {
      "chaptersCompleted": 15,
      "quizAverage": 0.92,
      "readingCompleteness": 1.0,
      "annotationsCreated": 147,
      "interactiveElementsCompleted": 23
    }
  },
  "proof": {
    "type": "Ed25519Signature2020",
    "created": "2025-12-25T12:00:00Z",
    "verificationMethod": "did:wia:publisher:academic-press#key-1",
    "proofPurpose": "assertionMethod",
    "proofValue": "z3FXQsWzK2v8fZp9jKLM5NoPqR7sTuV3wXy..."
  }
}
```

### 3.2 QR Code for Credentials

Generate standard QR codes for credential verification:

```json
{
  "type": "WIA-VC-QR",
  "version": "1.0",
  "credentialUrl": "https://verify.wia.org/vc/WIA-VC-123456",
  "qrContent": "https://verify.wia.org/vc/WIA-VC-123456"
}
```

## 4. Learning Records Store (LRS) Integration

### 4.1 xAPI Statement Generation

All learning activities MUST generate xAPI statements compatible with any LRS.

**Example: Chapter Completion**
```json
{
  "actor": {
    "objectType": "Agent",
    "name": "Alice Johnson",
    "account": {
      "homePage": "https://textbooks.academic.com",
      "name": "user123"
    },
    "mbox": "mailto:alice@example.com"
  },
  "verb": {
    "id": "http://adlnet.gov/expapi/verbs/completed",
    "display": {"en-US": "completed"}
  },
  "object": {
    "objectType": "Activity",
    "id": "https://textbooks.academic.com/978-3-16-148410-0/chapter-05",
    "definition": {
      "name": {"en-US": "Chapter 5: Newton's Laws of Motion"},
      "description": {"en-US": "Laws of motion and their applications"},
      "type": "http://adlnet.gov/expapi/activities/lesson",
      "extensions": {
        "https://wia.org/extensions/textbookId": "978-3-16-148410-0",
        "https://wia.org/extensions/wiaId": "WIA-EDU-008-TB-00123456"
      }
    }
  },
  "result": {
    "completion": true,
    "success": true,
    "score": {
      "scaled": 0.92,
      "raw": 92,
      "min": 0,
      "max": 100
    },
    "duration": "PT45M30S"
  },
  "context": {
    "contextActivities": {
      "parent": [{
        "id": "https://textbooks.academic.com/978-3-16-148410-0",
        "objectType": "Activity",
        "definition": {
          "name": {"en-US": "Introduction to Physics"}
        }
      }],
      "category": [{
        "id": "https://wia.org/standards/EDU-008",
        "objectType": "Activity",
        "definition": {
          "name": {"en-US": "WIA Digital Textbook Standard"}
        }
      }]
    },
    "platform": "WIA-EDU-008 Compliant Reader v2.0",
    "language": "en-US"
  },
  "timestamp": "2025-12-25T10:30:00Z"
}
```

## 5. Cross-Standard Integration

### 5.1 Integration Points

| WIA Standard | Integration Purpose | Data Exchange |
|--------------|---------------------|---------------|
| WIA-ID-001 | Digital Identity | Student authentication |
| WIA-EDU-001 | Digital Credentials | Completion certificates |
| WIA-EDU-005 | Learning Records | xAPI statements |
| WIA-ACCESS-001 | Accessibility | Assistive tech compatibility |
| WIA-DATA-001 | Data Privacy | Consent & data protection |
| WIA-AI-001 | AI Integration | Personalized learning |

### 5.2 Example: Integration with WIA-ID-001

Student authentication using WIA Digital Identity:

```json
{
  "authMethod": "WIA-ID-001",
  "did": "did:wia:student:user123",
  "credential": {
    "type": "StudentIdentityCredential",
    "institution": "State University",
    "studentId": "STU-2025-12345",
    "verified": true
  }
}
```

## 6. Global Discovery API

### 6.1 Search Textbooks

```http
GET https://registry.wia.org/api/v1/search?
  q=physics&
  standard=WIA-EDU-008&
  level=high-school&
  language=en&
  accessibility=AA&
  price_max=50&
  sort=rating

Response:
{
  "total": 47,
  "page": 1,
  "perPage": 20,
  "results": [
    {
      "wiaId": "WIA-EDU-008-TB-00123456",
      "title": "Introduction to Physics",
      "publisher": "Academic Press",
      "price": 49.99,
      "rating": 4.7,
      "reviews": 234,
      "adoptions": 1247,
      "accessibilityLevel": "WCAG-AA",
      "languages": ["en", "es", "fr"],
      "sampleUrl": "https://samples.wia.org/physics-101/"
    }
  ]
}
```

## 7. Quality Assurance Framework

### 7.1 Quality Metrics

Textbooks in the registry are rated on multiple dimensions:

```json
{
  "qualityMetrics": {
    "overall": 4.7,
    "contentAccuracy": {
      "score": 4.8,
      "reviewerCount": 12,
      "lastReviewed": "2025-11-15"
    },
    "pedagogicalQuality": {
      "score": 4.7,
      "educatorReviews": 89
    },
    "technicalQuality": {
      "score": 4.9,
      "automated": true,
      "lastTested": "2025-12-01"
    },
    "accessibility": {
      "wcagLevel": "AA",
      "score": 4.6,
      "issuesFound": 0
    },
    "studentEngagement": {
      "completionRate": 0.87,
      "averageTimeSpent": "PT45H30M",
      "annotationsPerStudent": 47
    }
  }
}
```

### 7.2 Peer Review Process

1. Submit textbook for certification
2. Automated technical validation
3. Accessibility testing
4. Expert content review (3+ reviewers)
5. Educator field testing (optional, 30+ days)
6. Certification decision
7. Ongoing monitoring and re-certification (every 3 years)

## 8. Open Educational Resources (OER) Support

### 8.1 Creative Commons Licensing

```xml
<metadata>
  <dc:rights>
    This work is licensed under a Creative Commons
    Attribution-ShareAlike 4.0 International License.
  </dc:rights>
  <meta property="cc:license">
    http://creativecommons.org/licenses/by-sa/4.0/
  </meta>
  <meta property="cc:attributionName">Dr. Jane Smith</meta>
  <meta property="cc:attributionURL">
    https://example.com/author/jsmith
  </meta>
  <meta property="wia:oer">true</meta>
  <meta property="wia:oerRepository">
    https://oer.wia.org/physics-101
  </meta>
</metadata>
```

## 9. Certification Process

### 9.1 Certification Levels

| Level | Requirements | Validity | Fee |
|-------|--------------|----------|-----|
| Phase 1 | EPUB3 + accessibility | 1 year | $500 |
| Phases 1-2 | + API compliance | 2 years | $1,500 |
| Phases 1-3 | + Synchronization | 3 years | $3,000 |
| Full (1-4) | Complete WIA integration | 3 years | $5,000 |

### 9.2 Certification Badge

Certified textbooks receive a digital badge:

```json
{
  "badgeType": "WIA-EDU-008-Certified",
  "level": "Full",
  "phases": [1, 2, 3, 4],
  "certificationDate": "2025-01-15",
  "expiryDate": "2028-01-15",
  "badgeImageUrl": "https://badges.wia.org/EDU-008-full.svg",
  "verificationUrl": "https://verify.wia.org/cert/WIA-EDU-008-TB-00123456"
}
```

## 10. International Harmonization

### 10.1 Compatible Standards

WIA-EDU-008 aligns with:

- **ISO/IEC TS 30135:** Digital textbook modularization
- **IMS Global LTI 1.3:** LMS integration
- **IMS Global OneRoster:** Roster management
- **IMS Global Caliper:** Analytics
- **IEEE LOM:** Learning object metadata
- **SCORM 2004 / xAPI:** Content packaging and tracking
- **Dublin Core:** Metadata standards
- **W3C EPUB3:** Content format
- **W3C Verifiable Credentials:** Digital credentials

### 10.2 Mapping to National Standards

Provide mappings to national curriculum standards:

```json
{
  "curriculumMappings": [
    {
      "country": "US",
      "framework": "Next Generation Science Standards (NGSS)",
      "standardIds": ["HS-PS2-1", "HS-PS2-2", "HS-PS2-3"]
    },
    {
      "country": "UK",
      "framework": "A-Level Physics",
      "modules": ["Module 4: Mechanics"]
    }
  ]
}
```

---

**Philosophy:** 弘益人間 · Benefit All Humanity

© 2025 WIA - World Certification Industry Association

**This completes all four phases of the WIA-EDU-008 Digital Textbook Standard.**


## Annex E — Implementation Notes for PHASE-4-INTEGRATION

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-4-INTEGRATION.

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
evidence for PHASE-4-INTEGRATION. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-4-integration/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-4-INTEGRATION with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-4-INTEGRATION does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-4-INTEGRATION.
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
for PHASE-4-INTEGRATION. The profile mechanism is non-normative and exists so that
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
  advertises `WIA-P4-INTEGRATION-Edge-only/2` is asserting conformance with
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
  is published manually. Sufficient for PHASE-4-INTEGRATION validation when the
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
