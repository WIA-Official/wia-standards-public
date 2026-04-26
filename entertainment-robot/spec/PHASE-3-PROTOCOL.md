# WIA-EDU-025: Technical Specification

**Version:** 1.0.0
**Last Updated:** 2025-12-26

---

## 1. Data Formats

### 1.1 Interactive Story Format

Stories are defined in JSON-LD with semantic markup:

```json
{
  "@context": "https://wiastandards.com/contexts/entertainment-robot/v1",
  "@type": "InteractiveStory",
  "storyId": "STORY-2025-001",
  "title": "The Magic Forest Adventure",
  "genre": "fantasy",
  "targetAgeRange": "7-12",
  "language": "en",
  "narrator": {
    "robotId": "ENT-ROBOT-001",
    "voice": "friendly-narrator",
    "expressiveness": "high"
  },
  "chapters": [
    {
      "chapterId": "CH01",
      "title": "The Beginning",
      "duration": 180,
      "narrative": {
        "beats": [
          {
            "beatId": "B01",
            "type": "exposition",
            "text": "Once upon a time...",
            "emotionalTone": "mysterious",
            "characterId": "narrator"
          }
        ]
      },
      "choicePoints": [
        {
          "choiceId": "C01",
          "prompt": "Which path do you choose?",
          "options": [
            {
              "optionId": "O01",
              "text": "The sunny path",
              "consequence": "leads-to-meadow",
              "learningObjective": "decision-making"
            }
          ]
        }
      ]
    }
  ],
  "characters": [
    {
      "characterId": "protagonist",
      "name": "Alex",
      "traits": ["brave", "curious"],
      "emotionalRange": ["happy", "scared", "excited"],
      "voiceProfile": "child-friendly"
    }
  ],
  "learningObjectives": [
    {
      "objectiveId": "LO01",
      "standard": "CCSS.ELA-LITERACY.RL.3.3",
      "description": "Describe characters and their motivations",
      "assessmentPoints": ["C01", "C02", "C03"]
    }
  ]
}
```

### 1.2 Performance Program Format

```json
{
  "@context": "https://wiastandards.com/contexts/entertainment-robot/v1",
  "@type": "PerformanceProgram",
  "performanceId": "PERF-2025-001",
  "title": "Science Magic Show",
  "type": "theater",
  "duration": 900,
  "educationalFocus": "science",
  "acts": [
    {
      "actNumber": 1,
      "title": "Opening",
      "scenes": [
        {
          "sceneId": "S01",
          "duration": 120,
          "actions": [
            {
              "timestamp": 0,
              "type": "speech",
              "robotId": "main",
              "text": "Welcome to the amazing world of science!",
              "emotion": "excited"
            },
            {
              "timestamp": 5,
              "type": "movement",
              "choreography": "welcome-bow",
              "duration": 3
            }
          ],
          "interactionPoints": [
            {
              "timestamp": 60,
              "type": "audience-poll",
              "question": "Have you ever seen real magic?"
            }
          ]
        }
      ]
    }
  ]
}
```

### 1.3 Emotional State Format

```json
{
  "@type": "EmotionalState",
  "timestamp": "2025-12-26T10:30:00Z",
  "detectedEmotions": [
    {
      "emotion": "happy",
      "confidence": 0.87,
      "source": "facial-expression"
    },
    {
      "emotion": "excited",
      "confidence": 0.72,
      "source": "voice-tone"
    }
  ],
  "engagementLevel": 0.85,
  "attentionScore": 0.92,
  "overallSentiment": "positive"
}
```

### 1.4 Therapeutic Session Format

```json
{
  "@type": "TherapeuticSession",
  "sessionId": "THERAPY-2025-001",
  "therapeuticFocus": "autism-social-skills",
  "duration": 1800,
  "protocol": {
    "protocolId": "ASD-SST-01",
    "evidenceBase": "ABA-based social stories",
    "supervisionRequired": true
  },
  "phases": [
    {
      "phase": "warmup",
      "duration": 300,
      "activities": [
        {
          "activityId": "greeting-routine",
          "type": "structured-interaction",
          "objective": "greeting-skills"
        }
      ]
    },
    {
      "phase": "main-activity",
      "duration": 900,
      "activities": [
        {
          "activityId": "turn-taking-game",
          "type": "guided-play",
          "objective": "turn-taking"
        }
      ]
    },
    {
      "phase": "cooldown",
      "duration": 300,
      "activities": [
        {
          "activityId": "reflection",
          "type": "conversation",
          "objective": "self-awareness"
        }
      ]
    }
  ],
  "progressTracking": {
    "metrics": ["engagement", "skill-demonstration", "independence"],
    "dataRetention": "HIPAA-compliant",
    "reporting": "parent-therapist-shared"
  }
}
```

---

## 2. Emotion Recognition Standards

### 2.1 Multi-Modal Detection

Emotion detection combines:
- Facial expression analysis (Facial Action Coding System compatible)
- Voice tone and prosody analysis
- Body language and gesture recognition
- Conversational sentiment analysis

### 2.2 Emotion Categories

Standard emotion categories:
- **Primary:** happy, sad, angry, fearful, surprised, disgusted
- **Secondary:** excited, calm, frustrated, confused, proud, shy
- **Complex:** anxious, overwhelmed, content, bored

### 2.3 Privacy Protocols

- All emotion data processed locally when possible
- Cloud processing requires explicit opt-in consent
- Emotion data encrypted in transit and at rest
- Automatic deletion after session unless explicitly saved
- Parental access to all emotion data for minors

---

## 3. Safety Protocols

### 3.1 Child Safety Requirements

All robots must implement:
- Safe physical materials (non-toxic, rounded edges)
- Volume limits appropriate for age group
- Emergency stop mechanism
- Supervised mode for therapeutic applications
- Content filtering based on age appropriateness

### 3.2 Emotional Safety

- Detect and respond to distress signals
- Escalation protocols for concerning behavior
- Positive reinforcement prioritization
- Avoidance of shame, guilt, or fear-based motivation
- Support for emotional regulation

---

## 4. Performance Specifications

### 4.1 Latency Requirements

- Voice interaction response: < 500ms
- Emotion detection: < 200ms
- Story choice processing: < 1000ms
- Performance synchronization: < 50ms (multi-robot)

### 4.2 Reliability

- 99.9% uptime for cloud services
- Graceful degradation for offline operation
- Automatic error recovery
- Session state preservation

---

**© 2025 WIA - World Certification Industry Association**


## Annex E — Implementation Notes for PHASE-3-PROTOCOL

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-3-PROTOCOL.

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
evidence for PHASE-3-PROTOCOL. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-3-protocol/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-3-PROTOCOL with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-3-PROTOCOL does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-3-PROTOCOL.
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
