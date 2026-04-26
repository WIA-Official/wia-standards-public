# WIA-EDU-007: Educational Robot Standard
## Phase 1: Data Format Specification

**Version:** 1.0.0
**Status:** ✅ Complete
**Last Updated:** 2025-12-25
**Category:** Education/Culture (EDU)

---

## 1. Overview

### 1.1 Purpose
The WIA-EDU-007 Educational Robot Standard defines a unified data format for educational robots across teaching assistance, STEM education, social interaction, programming education, and language learning applications. This standard enables interoperability between different robot platforms, educational systems, and learning management platforms.

### 1.2 Scope
- Robot profile and capability declaration
- Student interaction data
- Learning progress tracking
- Safety and compliance metadata
- Curriculum integration data

### 1.3 Philosophy
**弘益人間 (Benefit All Humanity)** - This standard aims to democratize educational robotics, making quality robot-assisted education accessible to all students regardless of location or socioeconomic background.

---

## 2. Core Data Schema

### 2.1 Educational Robot Profile

```json
{
  "@context": "https://wiastandards.com/contexts/educational-robot/v1",
  "@type": "EducationalRobotProfile",
  "robotId": "string (required)",
  "robotType": "enum (required)",
  "manufacturer": {
    "name": "string",
    "did": "string",
    "certification": "string"
  },
  "specifications": {
    "ageGroup": "string",
    "subjectAreas": ["string"],
    "interactionModes": ["string"],
    "languages": ["string"]
  },
  "capabilities": {
    "teaching": ["string"],
    "interaction": ["string"],
    "assessment": ["string"],
    "adaptation": ["string"]
  },
  "safetyCompliance": {
    "certifications": ["string"],
    "childSafe": "boolean",
    "dataPrivacy": "string",
    "supervisionRequired": "boolean"
  },
  "version": "string",
  "timestamp": "ISO8601"
}
```

### 2.2 Field Definitions

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `robotId` | string | Yes | Unique identifier (e.g., `EDU-ROBOT-2025-001`) |
| `robotType` | enum | Yes | `teaching-assistant`, `stem-education`, `social-companion`, `programming-tutor`, `language-learning` |
| `ageGroup` | string | Yes | Target age range (e.g., `3-6`, `7-12`, `13-18`, `18+`) |
| `subjectAreas` | array | Yes | Supported subjects: `mathematics`, `science`, `programming`, `language`, `robotics`, `social-skills` |
| `interactionModes` | array | Yes | `voice`, `touch`, `gesture`, `visual`, `multimodal` |
| `languages` | array | Yes | Supported languages (ISO 639-1 codes) |
| `childSafe` | boolean | Yes | Child safety certification status |
| `dataPrivacy` | string | Yes | Compliance standard: `COPPA`, `GDPR`, `FERPA` |

---

## 3. Learning Session Data Format

### 3.1 Session Record

```json
{
  "@context": "https://wiastandards.com/contexts/learning-session/v1",
  "@type": "LearningSession",
  "sessionId": "string (required)",
  "robotId": "string (required)",
  "studentId": "string (required)",
  "sessionType": "enum (required)",
  "startTime": "ISO8601 (required)",
  "endTime": "ISO8601",
  "duration": "integer (minutes)",
  "curriculum": {
    "subjectArea": "string",
    "lessonId": "string",
    "learningObjectives": ["string"],
    "difficulty": "enum"
  },
  "interaction": {
    "engagementScore": "number (0-100)",
    "emotionalState": ["string"],
    "questionsAsked": "integer",
    "responsiveness": "number (0-1)"
  },
  "assessment": {
    "preTestScore": "number",
    "postTestScore": "number",
    "improvementRate": "number",
    "masteryLevel": "enum"
  },
  "adaptations": {
    "difficultyAdjustments": ["object"],
    "paceModifications": ["object"],
    "modalityShifts": ["object"]
  },
  "timestamp": "ISO8601"
}
```

### 3.2 Session Types

| Type | Description | Typical Duration |
|------|-------------|------------------|
| `lecture` | Concept introduction | 10-20 min |
| `practice` | Hands-on exercises | 15-30 min |
| `quiz` | Knowledge assessment | 5-15 min |
| `project` | Project-based learning | 30-60 min |
| `social` | Social skills development | 10-30 min |
| `remedial` | Targeted skill improvement | 15-25 min |

---

## 4. Student Progress Data

### 4.1 Progress Tracking Format

```json
{
  "@context": "https://wiastandards.com/contexts/student-progress/v1",
  "@type": "StudentProgressRecord",
  "studentId": "string (required)",
  "robotId": "string (required)",
  "subjectArea": "string (required)",
  "progressPeriod": {
    "startDate": "ISO8601",
    "endDate": "ISO8601"
  },
  "metrics": {
    "sessionsCompleted": "integer",
    "totalLearningTime": "integer (minutes)",
    "averageEngagement": "number (0-100)",
    "skillsMastered": ["string"],
    "challengeAreas": ["string"]
  },
  "achievements": [
    {
      "achievementId": "string",
      "type": "string",
      "dateEarned": "ISO8601",
      "level": "enum"
    }
  ],
  "adaptivePath": {
    "currentLevel": "string",
    "recommendedNext": ["string"],
    "learningStyle": "string",
    "pacePreference": "string"
  },
  "timestamp": "ISO8601"
}
```

### 4.2 Mastery Levels

| Level | Score Range | Description |
|-------|-------------|-------------|
| `novice` | 0-39 | Initial exposure |
| `beginner` | 40-59 | Basic understanding |
| `intermediate` | 60-79 | Competent application |
| `advanced` | 80-89 | Proficient mastery |
| `expert` | 90-100 | Expert-level mastery |

---

## 5. Safety & Compliance Metadata

### 5.1 Safety Record Format

```json
{
  "@context": "https://wiastandards.com/contexts/safety/v1",
  "@type": "SafetyComplianceRecord",
  "robotId": "string (required)",
  "certifications": [
    {
      "type": "string",
      "issuingBody": "string",
      "certificationId": "string",
      "issuedDate": "ISO8601",
      "expiryDate": "ISO8601",
      "status": "enum"
    }
  ],
  "safetyFeatures": {
    "emergencyStop": "boolean",
    "physicalSafety": {
      "softMaterials": "boolean",
      "roundedEdges": "boolean",
      "lowSpeed": "boolean",
      "collisionAvoidance": "boolean"
    },
    "dataSafety": {
      "encryption": "string",
      "anonymization": "boolean",
      "parentalControls": "boolean",
      "dataRetentionDays": "integer"
    }
  },
  "supervisionMode": {
    "required": "boolean",
    "minimumAge": "integer",
    "teacherPresenceRequired": "boolean"
  },
  "timestamp": "ISO8601"
}
```

### 5.2 Regulatory Compliance

| Regulation | Region | Requirement |
|------------|--------|-------------|
| **COPPA** | USA | Parental consent for children <13 |
| **GDPR** | EU | Data protection & right to erasure |
| **FERPA** | USA | Educational record privacy |
| **APPI** | Japan | Personal information protection |
| **PIPL** | China | Personal information protection law |

---

## 6. Curriculum Integration Data

### 6.1 Lesson Plan Format

```json
{
  "@context": "https://wiastandards.com/contexts/curriculum/v1",
  "@type": "RobotLessonPlan",
  "lessonId": "string (required)",
  "robotId": "string (required)",
  "curriculum": {
    "framework": "string",
    "subject": "string",
    "grade": "string",
    "standards": ["string"]
  },
  "learningObjectives": [
    {
      "objectiveId": "string",
      "description": "string",
      "bloomLevel": "enum",
      "measurable": "boolean"
    }
  ],
  "activities": [
    {
      "activityId": "string",
      "type": "enum",
      "duration": "integer",
      "materials": ["string"],
      "instructions": "string",
      "assessment": "string"
    }
  ],
  "differentiation": {
    "advanced": ["string"],
    "standard": ["string"],
    "remedial": ["string"]
  },
  "timestamp": "ISO8601"
}
```

### 6.2 Bloom's Taxonomy Levels

| Level | Code | Educational Goal |
|-------|------|------------------|
| **Remember** | `BL1` | Recall facts and basic concepts |
| **Understand** | `BL2` | Explain ideas or concepts |
| **Apply** | `BL3` | Use information in new situations |
| **Analyze** | `BL4` | Draw connections among ideas |
| **Evaluate** | `BL5` | Justify a decision or course of action |
| **Create** | `BL6` | Produce new or original work |

---

## 7. Validation Rules

### 7.1 Required Fields Validation
- All `robotId` values must follow pattern: `EDU-ROBOT-YYYY-XXX`
- All `studentId` values must be anonymized or pseudonymized
- All timestamps must be in ISO 8601 format
- All scores must be in range [0, 100]

### 7.2 Data Quality Rules
- Engagement scores must be recorded at minimum 1-minute intervals
- Session duration must not exceed 120 minutes
- Student age must match robot's certified age group
- Privacy consent must be verified before data collection

---

## 8. Examples

### 8.1 Complete Robot Profile Example

```json
{
  "@context": "https://wiastandards.com/contexts/educational-robot/v1",
  "@type": "EducationalRobotProfile",
  "robotId": "EDU-ROBOT-2025-001",
  "robotType": "stem-education",
  "manufacturer": {
    "name": "EduBots Inc.",
    "did": "did:wia:manufacturer:edubots-inc",
    "certification": "WIA-CERT-2025-123"
  },
  "specifications": {
    "ageGroup": "7-12",
    "subjectAreas": ["mathematics", "science", "programming"],
    "interactionModes": ["voice", "touch", "visual"],
    "languages": ["en", "es", "zh", "ko"]
  },
  "capabilities": {
    "teaching": [
      "adaptive-instruction",
      "personalized-feedback",
      "gamification"
    ],
    "interaction": [
      "natural-language-processing",
      "emotion-detection",
      "gesture-recognition"
    ],
    "assessment": [
      "real-time-quizzing",
      "progress-tracking",
      "skill-gap-analysis"
    ],
    "adaptation": [
      "difficulty-adjustment",
      "pace-modification",
      "learning-style-customization"
    ]
  },
  "safetyCompliance": {
    "certifications": ["COPPA", "GDPR", "ISO-13482"],
    "childSafe": true,
    "dataPrivacy": "COPPA-GDPR-compliant",
    "supervisionRequired": false
  },
  "version": "1.0.0",
  "timestamp": "2025-12-25T10:00:00Z"
}
```

---

## 9. Integration with WIA Ecosystem

### 9.1 Cross-Standard References

| WIA Standard | Integration Point | Purpose |
|--------------|-------------------|---------|
| **WIA-AAC-002** | Speech interface | Natural language interaction |
| **WIA-AI-006** | Emotion AI | Student emotion detection |
| **WIA-EDU-001** | LMS integration | Curriculum synchronization |
| **WIA-SEC-001** | Data privacy | Student data protection |
| **WIA-DATA-003** | Analytics | Learning analytics |

### 9.2 Verifiable Credentials
Educational achievements can be issued as W3C Verifiable Credentials (see Phase 4: Integration).

---

## 10. Implementation Guidelines

### 10.1 Data Storage
- Use encrypted storage for student data
- Implement data retention policies (max 90 days for inactive data)
- Support GDPR right-to-erasure

### 10.2 Data Exchange
- Use JSON-LD for semantic interoperability
- Implement RESTful APIs (see Phase 2)
- Support real-time streaming (see Phase 3)

### 10.3 Privacy by Design
- Anonymize student identifiers
- Obtain parental consent for children <13
- Implement role-based access control

---

## 11. Conformance

A system is conformant with WIA-EDU-007 Phase 1 if it:

✅ Implements all required fields in robot profile
✅ Records learning sessions in specified format
✅ Tracks student progress with defined metrics
✅ Maintains safety compliance metadata
✅ Integrates with standard curriculum frameworks
✅ Validates all data against specified rules
✅ Implements privacy-by-design principles

---

**弘益人間 (Benefit All Humanity)**

*WIA - World Certification Industry Association*
*© 2025 MIT License*


## Annex E — Implementation Notes for PHASE-1-DATA-FORMAT

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-1-DATA-FORMAT.

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
evidence for PHASE-1-DATA-FORMAT. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-1-data-format/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-1-DATA-FORMAT with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-1-DATA-FORMAT does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-1-DATA-FORMAT.
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
for PHASE-1-DATA-FORMAT. The profile mechanism is non-normative and exists so that
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
  advertises `WIA-P1-DATA-FORMAT-Edge-only/2` is asserting conformance with
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
  is published manually. Sufficient for PHASE-1-DATA-FORMAT validation when the
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
