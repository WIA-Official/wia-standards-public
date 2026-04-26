# WIA-EDU-014 Game-Based Learning Standard v1.0

## Phase 1: Data Format & Game Design

**Status:** ✅ Complete
**Version:** 1.0.0
**Date:** 2025-12-25
**Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

This specification defines the foundational data format requirements for WIA-compliant game-based learning systems. Phase 1 establishes JSON schemas for game metadata, player profiles, progress tracking, achievements, and assessment data.

## 2. Scope

Phase 1 covers:
- Game metadata and descriptive information
- Player profile and preference data
- Progress tracking and session data
- Achievement and badge systems
- Assessment and learning analytics data
- Content structure and challenges

## 3. Game Metadata Schema

### 3.1 Base Game Metadata

All educational games MUST include comprehensive metadata in JSON format:

```json
{
  "id": "game_unique_id",
  "standard": "WIA-EDU-014",
  "version": "1.0",
  "title": "Game Title",
  "type": "educational-game",
  "category": "puzzle|simulation|adventure|strategy|quiz",
  "subject": "mathematics|science|language|history|coding",
  "topics": ["topic1", "topic2"],
  "gradeLevel": {"min": 6, "max": 8},
  "difficulty": "beginner|intermediate|advanced",
  "targetAge": {"min": 11, "max": 14},
  "language": ["en", "es", "fr", "ko"],
  "platform": ["web", "ios", "android", "desktop"],
  "accessibility": {
    "wcagLevel": "AA",
    "features": ["screen-reader", "high-contrast", "subtitles", "keyboard-nav"]
  },
  "learningObjectives": [
    {
      "id": "obj1",
      "description": "Learning objective description",
      "standard": "CCSS.Math.7.EE.A.1"
    }
  ],
  "metadata": {
    "created": "2025-12-25T00:00:00Z",
    "updated": "2025-12-25T00:00:00Z",
    "author": "Developer Name",
    "publisher": "Publisher Name",
    "license": "MIT"
  }
}
```

### 3.2 Required Fields

- `id` (string): Unique identifier following pattern `game_[a-z0-9]+`
- `standard` (string): Must be "WIA-EDU-014"
- `version` (string): Semantic version (e.g., "1.0")
- `title` (string): Human-readable game title
- `type` (enum): Game classification
- `subject` (enum): Primary subject area
- `accessibility` (object): WCAG compliance information

## 4. Player Profile Schema

### 4.1 Player Profile Structure

```json
{
  "playerId": "player_unique_id",
  "username": "display_name",
  "demographics": {
    "age": 12,
    "grade": 7,
    "location": {"country": "US", "region": "CA"}
  },
  "preferences": {
    "difficulty": "adaptive|easy|medium|hard",
    "soundEnabled": true,
    "musicVolume": 0.7,
    "notificationsEnabled": true,
    "language": "en"
  },
  "accessibility": {
    "screenReader": false,
    "highContrast": false,
    "extendedTime": false,
    "alternativeInput": false,
    "fontSize": "medium|large|xlarge"
  },
  "learningProfile": {
    "style": "visual|auditory|kinesthetic|reading",
    "pace": "slow|moderate|fast",
    "strengths": ["skill1", "skill2"],
    "growthAreas": ["skill3", "skill4"]
  },
  "privacy": {
    "dataCollection": "consented|parent-consented|declined",
    "consentDate": "2025-12-25T00:00:00Z",
    "shareWithTeachers": true,
    "shareWithPeers": false
  }
}
```

## 5. Progress Tracking Schema

### 5.1 Session Progress

```json
{
  "sessionId": "session_unique_id",
  "playerId": "player_xyz789",
  "gameId": "game_abc123",
  "startTime": "2025-12-25T14:00:00Z",
  "endTime": "2025-12-25T14:45:00Z",
  "duration": "PT45M",
  "currentLevel": 15,
  "totalLevels": 50,
  "completionPercentage": 30,
  "performance": {
    "score": 450,
    "accuracy": 78,
    "problemsSolved": 15,
    "hintsUsed": 3,
    "attemptsAverage": 1.8
  },
  "skillProgress": [
    {
      "skillId": "linear-equations",
      "mastery": 90,
      "status": "mastered|developing|beginning",
      "lastPracticed": "2025-12-25T14:30:00Z"
    }
  ]
}
```

### 5.2 Cumulative Progress

```json
{
  "playerId": "player_xyz789",
  "gameId": "game_abc123",
  "totalPlayTime": "PT12H30M",
  "sessionsCompleted": 24,
  "lastPlayed": "2025-12-25T14:45:00Z",
  "achievements": ["badge1", "badge2"],
  "levelHistory": [
    {
      "level": 1,
      "completedAt": "2025-12-20T15:00:00Z",
      "attempts": 2,
      "score": 85
    }
  ]
}
```

## 6. Achievement System

### 6.1 Achievement Definition

```json
{
  "achievementId": "badge_unique_id",
  "type": "completion|mastery|performance|persistence|collaboration|innovation",
  "name": "Achievement Name",
  "description": "Achievement description",
  "iconUrl": "https://cdn.example.com/badges/badge.png",
  "criteria": {
    "skillsMastered": ["skill1", "skill2"],
    "minAccuracy": 85,
    "minProblems": 100,
    "minPlayTime": "PT10H"
  },
  "points": 100,
  "rarity": "common|uncommon|rare|legendary"
}
```

### 6.2 Achievement Award Record

```json
{
  "awardId": "award_unique_id",
  "achievementId": "badge_algebra_master",
  "playerId": "player_xyz789",
  "earnedDate": "2025-12-25T14:30:00Z",
  "evidence": {
    "score": 95,
    "challengesCompleted": 120,
    "accuracy": 92
  },
  "verifiable": true,
  "credentialUrl": "https://verify.wia.com/vc/abc123"
}
```

## 7. Assessment Data

### 7.1 Assessment Event

```json
{
  "assessmentId": "assess_unique_id",
  "type": "formative|summative|diagnostic|adaptive",
  "playerId": "player_xyz789",
  "challengeId": "ch_15_01",
  "timestamp": "2025-12-25T14:30:00Z",
  "response": "student answer or action",
  "correctAnswer": "expected answer",
  "isCorrect": true,
  "score": 10,
  "maxScore": 10,
  "timeSpent": 45,
  "hintsUsed": 1,
  "skillsAssessed": ["linear-equations", "algebraic-reasoning"]
}
```

## 8. Content Structure

### 8.1 Level Definition

```json
{
  "levelId": "level_15",
  "gameId": "game_abc123",
  "title": "Level Title",
  "difficulty": 6,
  "learningObjectives": ["obj1", "obj2"],
  "challenges": [
    {
      "challengeId": "ch_15_01",
      "type": "equation-solving|multiple-choice|drag-drop|simulation",
      "question": "Challenge prompt",
      "correctAnswer": "answer",
      "hints": ["hint1", "hint2"],
      "feedback": {
        "correct": "Positive feedback",
        "incorrect": "Corrective feedback"
      },
      "points": 10,
      "timeLimit": 120
    }
  ],
  "requiredScore": 80,
  "rewards": {
    "points": 100,
    "badge": "badge_id"
  }
}
```

## 9. Data Validation

All data structures MUST be validated against JSON Schema definitions. Implementations MUST:
- Reject invalid data with clear error messages
- Use ISO 8601 format for all dates and times
- Use ISO 8601 duration format (PT notation) for time periods
- Follow WIA naming conventions for all IDs (prefix_alphanumeric)
- Encrypt personally identifiable information at rest and in transit

## 10. Privacy Requirements

### 10.1 Data Minimization
Collect only data necessary for educational purposes. PII MUST be:
- Separated from learning analytics data
- Encrypted at rest
- Transmitted only over TLS 1.3
- Deleted upon request (right to erasure)

### 10.2 Consent Management
```json
{
  "playerId": "player_xyz789",
  "consents": [
    {
      "type": "data-collection",
      "granted": true,
      "grantedBy": "parent|student",
      "grantedAt": "2025-12-25T00:00:00Z",
      "scope": ["progress", "analytics"]
    }
  ]
}
```

## 11. Compliance

Implementations MUST comply with:
- FERPA (US educational records privacy)
- COPPA (US children's online privacy)
- GDPR (EU data protection)
- WCAG 2.1 Level AA (accessibility)

---

**Status:** This specification is complete and ready for implementation.

**Next Phase:** Phase 2 defines RESTful APIs for game management and player interaction.

弘益人間 · Benefit All Humanity
© 2025 WIA - MIT License


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
