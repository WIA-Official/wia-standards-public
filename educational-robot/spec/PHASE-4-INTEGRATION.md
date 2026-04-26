# WIA-EDU-007: Educational Robot Standard
## Phase 4: WIA Ecosystem Integration

**Version:** 2.0.0
**Status:** ✅ Complete
**Last Updated:** 2025-12-25
**Category:** Education/Culture (EDU)

---

## 1. Overview

### 1.1 Purpose
This specification defines how educational robots integrate with the broader WIA ecosystem, enabling interoperability with other WIA standards, cross-standard data exchange, verifiable credentials, and certification requirements.

### 1.2 Philosophy
**弘益人間 (Benefit All Humanity)** - By connecting educational robots to the global WIA ecosystem, we create a unified, interoperable system that serves all of humanity's educational needs.

### 1.3 Integration Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    WIA Ecosystem Layer                       │
├─────────────────────────────────────────────────────────────┤
│  WIA-OMNI-API (Universal Integration Layer)                 │
├─────────────────────────────────────────────────────────────┤
│   │      │      │      │      │      │      │      │        │
│  AAC    AI    EDU   SEC   DATA  COMP   BIO   MED   ...     │
├─────────────────────────────────────────────────────────────┤
│  Educational Robot (WIA-EDU-007)                            │
└─────────────────────────────────────────────────────────────┘
```

---

## 2. WIA-OMNI-API Integration

### 2.1 Universal Connector

All WIA standards connect through the **WIA-OMNI-API** - the universal integration layer.

#### Base URL
```
https://omni.wiastandards.com/v1
```

#### Registration Endpoint
```http
POST /standards/register
Content-Type: application/json
Authorization: Bearer <wia_api_key>

{
  "standardId": "WIA-EDU-007",
  "version": "2.0.0",
  "robotId": "EDU-ROBOT-2025-001",
  "capabilities": [
    "teaching-assistant",
    "stem-education",
    "adaptive-learning"
  ],
  "integrations": [
    "WIA-AAC-002",
    "WIA-AI-006",
    "WIA-SEC-001"
  ],
  "endpoints": {
    "api": "https://api.edubot.example.com/v1",
    "stream": "wss://stream.edubot.example.com/v1",
    "webhook": "https://webhook.edubot.example.com/v1"
  }
}
```

### 2.2 Service Discovery

#### Query Available Integrations
```http
GET /standards/WIA-EDU-007/integrations
Authorization: Bearer <token>
```

#### Response
```json
{
  "@context": "https://wiastandards.com/contexts/integration/v1",
  "standardId": "WIA-EDU-007",
  "availableIntegrations": [
    {
      "standardId": "WIA-AAC-002",
      "name": "Speech-to-Speech Communication",
      "category": "Accessibility",
      "integrationPoints": ["voice-interface", "tts", "stt"],
      "compatible": true,
      "documentation": "https://wiastandards.com/docs/edu-007-aac-002"
    },
    {
      "standardId": "WIA-AI-006",
      "name": "Emotion AI",
      "category": "AI",
      "integrationPoints": ["emotion-detection", "adaptive-response"],
      "compatible": true,
      "documentation": "https://wiastandards.com/docs/edu-007-ai-006"
    },
    {
      "standardId": "WIA-EDU-001",
      "name": "Learning Management System",
      "category": "Education",
      "integrationPoints": ["curriculum-sync", "grade-book"],
      "compatible": true,
      "documentation": "https://wiastandards.com/docs/edu-007-edu-001"
    }
  ]
}
```

---

## 3. Cross-Standard Integration

### 3.1 WIA-AAC-002 (Speech Communication)

#### Integration Point: Voice Interface

```json
{
  "@context": [
    "https://wiastandards.com/contexts/edu-robot/v1",
    "https://wiastandards.com/contexts/speech/v1"
  ],
  "@type": "VoiceIntegration",
  "robotId": "EDU-ROBOT-2025-001",
  "speechStandard": "WIA-AAC-002",
  "configuration": {
    "inputLanguages": ["en-US", "es-ES", "zh-CN", "ko-KR"],
    "outputVoices": [
      {
        "language": "en-US",
        "gender": "neutral",
        "age": "child-friendly",
        "emotionSupport": true
      }
    ],
    "features": {
      "realTimeTranscription": true,
      "emotionIntonation": true,
      "multilingual": true,
      "backgroundNoiseReduction": true
    }
  },
  "dataFlow": {
    "studentSpeech": "WIA-AAC-002 → STT → WIA-EDU-007 → NLU",
    "robotResponse": "WIA-EDU-007 → Text → WIA-AAC-002 → TTS → Audio"
  }
}
```

### 3.2 WIA-AI-006 (Emotion AI)

#### Integration Point: Emotion Detection

```json
{
  "@context": [
    "https://wiastandards.com/contexts/edu-robot/v1",
    "https://wiastandards.com/contexts/emotion-ai/v1"
  ],
  "@type": "EmotionIntegration",
  "robotId": "EDU-ROBOT-2025-001",
  "emotionStandard": "WIA-AI-006",
  "configuration": {
    "detectionMethods": ["facial-analysis", "voice-analysis", "behavioral"],
    "emotionCategories": [
      "engaged", "confused", "frustrated", "excited",
      "bored", "anxious", "confident", "curious"
    ],
    "updateFrequency": "real-time",
    "privacyMode": "emotion-vectors-only"
  },
  "adaptiveResponse": {
    "frustration": {
      "action": "difficulty-reduction",
      "empathy": "supportive-encouragement",
      "paceAdjustment": "slower"
    },
    "boredom": {
      "action": "difficulty-increase",
      "engagement": "gamification",
      "varietyBoost": true
    },
    "confusion": {
      "action": "re-explanation",
      "modalityShift": "visual-aids",
      "checkUnderstanding": true
    }
  }
}
```

### 3.3 WIA-EDU-001 (Learning Management System)

#### Integration Point: Curriculum Synchronization

```json
{
  "@context": [
    "https://wiastandards.com/contexts/edu-robot/v1",
    "https://wiastandards.com/contexts/lms/v1"
  ],
  "@type": "LMSIntegration",
  "robotId": "EDU-ROBOT-2025-001",
  "lmsStandard": "WIA-EDU-001",
  "configuration": {
    "lmsProvider": "Canvas LMS",
    "syncDirection": "bidirectional",
    "syncFrequency": "real-time",
    "dataExchange": {
      "courses": true,
      "assignments": true,
      "grades": true,
      "attendance": true,
      "progress": true
    }
  },
  "mapping": {
    "robotSession": "lms.assignment_submission",
    "robotAssessment": "lms.grade_entry",
    "robotProgress": "lms.progress_update",
    "robotAttendance": "lms.attendance_mark"
  }
}
```

### 3.4 WIA-SEC-001 (Privacy & Security)

#### Integration Point: Student Data Protection

```json
{
  "@context": [
    "https://wiastandards.com/contexts/edu-robot/v1",
    "https://wiastandards.com/contexts/security/v1"
  ],
  "@type": "SecurityIntegration",
  "robotId": "EDU-ROBOT-2025-001",
  "securityStandard": "WIA-SEC-001",
  "configuration": {
    "encryption": {
      "atRest": "AES-256-GCM",
      "inTransit": "TLS 1.3",
      "keyManagement": "WIA-KMS"
    },
    "authentication": {
      "method": "OAuth 2.0 + DID",
      "mfa": true,
      "biometric": false
    },
    "authorization": {
      "model": "RBAC",
      "roles": ["student", "teacher", "parent", "admin"]
    },
    "privacy": {
      "compliance": ["COPPA", "GDPR", "FERPA"],
      "dataMinimization": true,
      "anonymization": true,
      "consentManagement": true,
      "rightToErasure": true
    }
  }
}
```

### 3.5 WIA-DATA-003 (Learning Analytics)

#### Integration Point: Analytics & Insights

```json
{
  "@context": [
    "https://wiastandards.com/contexts/edu-robot/v1",
    "https://wiastandards.com/contexts/analytics/v1"
  ],
  "@type": "AnalyticsIntegration",
  "robotId": "EDU-ROBOT-2025-001",
  "analyticsStandard": "WIA-DATA-003",
  "configuration": {
    "dataStreams": [
      "engagement-metrics",
      "learning-outcomes",
      "skill-progression",
      "emotional-states",
      "interaction-patterns"
    ],
    "aggregation": {
      "individual": true,
      "classroom": true,
      "school": true,
      "anonymizedCohort": true
    },
    "insights": {
      "predictiveAnalytics": true,
      "interventionSuggestions": true,
      "performanceTrends": true,
      "skillGapAnalysis": true
    }
  }
}
```

---

## 4. Verifiable Credentials

### 4.1 Educational Achievement Credentials

Educational robots can issue **W3C Verifiable Credentials** for student achievements.

#### Achievement Credential Schema

```json
{
  "@context": [
    "https://www.w3.org/2018/credentials/v1",
    "https://wiastandards.com/contexts/educational-achievement/v1"
  ],
  "type": ["VerifiableCredential", "EducationalAchievementCredential"],
  "issuer": {
    "id": "did:wia:robot:EDU-ROBOT-2025-001",
    "name": "SmartEdu Robot System",
    "certifiedBy": "WIA",
    "certificationId": "WIA-CERT-EDU-2025-123"
  },
  "issuanceDate": "2025-12-25T10:00:00Z",
  "expirationDate": null,
  "credentialSubject": {
    "id": "did:wia:student:anonymous-hash-12345",
    "achievement": {
      "achievementType": "skill-mastery",
      "subject": "mathematics",
      "skill": "multiplication-mastery",
      "level": "gold",
      "description": "Demonstrated mastery of single and double-digit multiplication",
      "dateEarned": "2025-12-25T10:00:00Z",
      "assessmentScore": 95,
      "bloomLevel": "BL4-Analyze",
      "metadata": {
        "sessionsCompleted": 24,
        "totalLearningTime": 720,
        "averageEngagement": 92
      }
    },
    "evidence": [
      {
        "type": "RobotAssessment",
        "assessmentId": "ASSESS-2025-789",
        "score": 95,
        "timestamp": "2025-12-25T10:00:00Z"
      }
    ]
  },
  "proof": {
    "type": "Ed25519Signature2020",
    "created": "2025-12-25T10:00:00Z",
    "proofPurpose": "assertionMethod",
    "verificationMethod": "did:wia:robot:EDU-ROBOT-2025-001#key-1",
    "proofValue": "z5g3Dz8N9KvJNqGhH3J2PqP8yBxFqJqJvN8hQ9RtVxWzK..."
  }
}
```

### 4.2 QR Code Generation

Achievement credentials can be encoded as QR codes for easy verification.

#### QR Data Format
```
wia://verify?credential=<base64_encoded_vc>&type=educational-achievement
```

#### Verification Endpoint
```http
POST https://verify.wiastandards.com/v1/credentials
Content-Type: application/json

{
  "credential": "<vc_json>",
  "verificationMethod": "blockchain"
}
```

#### Verification Response
```json
{
  "verified": true,
  "issuer": "did:wia:robot:EDU-ROBOT-2025-001",
  "subject": "did:wia:student:anonymous-hash-12345",
  "achievementType": "skill-mastery",
  "issuanceDate": "2025-12-25T10:00:00Z",
  "blockchainProof": {
    "chain": "WIA-Chain",
    "blockNumber": 12345678,
    "transactionHash": "0x123abc..."
  },
  "status": "valid",
  "timestamp": "2025-12-25T11:00:00Z"
}
```

---

## 5. WIA Certification

### 5.1 Certification Levels

| Level | Requirements | Badge |
|-------|--------------|-------|
| **WIA Compatible** | Implements Phases 1-2 | 🟢 |
| **WIA Certified** | Implements Phases 1-3 + Safety | 🔵 |
| **WIA Verified** | Implements all 4 phases + Audit | 🟣 |
| **WIA Premium** | Verified + Annual re-certification | ⭐ |

### 5.2 Certification Process

```
┌─────────────┐     ┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│  1. Apply   │────▶│ 2. Assess   │────▶│ 3. Audit    │────▶│ 4. Certify  │
└─────────────┘     └─────────────┘     └─────────────┘     └─────────────┘
                           │                    │                    │
                           │                    │                    ▼
                           │                    │            ┌─────────────┐
                           │                    └───────────▶│  5. Monitor │
                           │                                 └─────────────┘
                           ▼                                        │
                    ┌─────────────┐                                │
                    │  Re-submit  │◀───────────────────────────────┘
                    └─────────────┘
```

### 5.3 Certification Endpoint

```http
POST https://cert.wiastandards.com/v1/apply
Content-Type: application/json

{
  "standardId": "WIA-EDU-007",
  "version": "2.0.0",
  "applicant": {
    "organization": "EduBots Inc.",
    "did": "did:wia:manufacturer:edubots-inc",
    "contact": "cert@edubots.example.com"
  },
  "product": {
    "name": "SmartEdu Robot",
    "model": "EDU-2025-PRO",
    "robotId": "EDU-ROBOT-2025-001"
  },
  "compliance": {
    "phase1": true,
    "phase2": true,
    "phase3": true,
    "phase4": true,
    "safety": "ISO-13482",
    "privacy": ["COPPA", "GDPR"],
    "accessibility": "WCAG-2.1-AA"
  },
  "documentation": [
    "https://edubots.example.com/docs/phase1",
    "https://edubots.example.com/docs/phase2",
    "https://edubots.example.com/docs/phase3",
    "https://edubots.example.com/docs/phase4"
  ],
  "testResults": "https://edubots.example.com/test-results.pdf"
}
```

---

## 6. Global Registry

### 6.1 Robot Registration

All certified robots must register with the WIA Global Registry.

#### Registration Endpoint
```http
POST https://registry.wiastandards.com/v1/robots
Content-Type: application/json

{
  "robotId": "EDU-ROBOT-2025-001",
  "manufacturer": "did:wia:manufacturer:edubots-inc",
  "model": "SmartEdu Robot Pro",
  "certificationLevel": "WIA Verified",
  "certificationDate": "2025-12-01T00:00:00Z",
  "expirationDate": "2026-12-01T00:00:00Z",
  "capabilities": [
    "teaching-assistant",
    "stem-education",
    "adaptive-learning"
  ],
  "integrations": [
    "WIA-AAC-002",
    "WIA-AI-006",
    "WIA-EDU-001"
  ],
  "deployments": [
    {
      "location": "Lincoln Elementary School",
      "city": "San Francisco",
      "country": "USA",
      "activeDate": "2025-12-15T00:00:00Z"
    }
  ]
}
```

### 6.2 Registry Lookup

```http
GET https://registry.wiastandards.com/v1/robots/EDU-ROBOT-2025-001
```

#### Response
```json
{
  "@context": "https://wiastandards.com/contexts/registry/v1",
  "robotId": "EDU-ROBOT-2025-001",
  "status": "active",
  "certified": true,
  "certificationLevel": "WIA Verified",
  "manufacturer": {
    "name": "EduBots Inc.",
    "did": "did:wia:manufacturer:edubots-inc",
    "verified": true
  },
  "trustScore": 98,
  "deployments": 127,
  "totalStudents": 3845,
  "averageRating": 4.8,
  "lastAudit": "2025-11-15T00:00:00Z",
  "nextAudit": "2026-05-15T00:00:00Z"
}
```

---

## 7. Ecosystem Benefits

### 7.1 For Students
✅ Seamless learning across different robot platforms
✅ Verifiable credentials for achievements
✅ Personalized learning paths powered by WIA-AI
✅ Privacy-protected data sharing

### 7.2 For Teachers
✅ Unified dashboard across all robots (via WIA-EDU-001)
✅ Real-time analytics and insights
✅ Easy curriculum integration
✅ Safety compliance monitoring

### 7.3 For Schools
✅ Multi-vendor robot interoperability
✅ Centralized management via WIA-OMNI-API
✅ Compliance with education regulations
✅ ROI tracking and reporting

### 7.4 For Robot Manufacturers
✅ Access to global ecosystem
✅ WIA certification credibility
✅ Standardized integration patterns
✅ Reduced development time

---

## 8. Future Roadmap

### 8.1 Phase 5: AI Personalization (v3.0) - Q2 2026
- Advanced learning path optimization
- Multi-robot collaborative teaching
- Global knowledge graph integration

### 8.2 Phase 6: Metaverse Integration (v4.0) - Q4 2026
- Virtual classroom robots
- Mixed reality education
- Global virtual school network

---

## 9. Conformance

A system is conformant with WIA-EDU-007 Phase 4 if it:

✅ Registers with WIA-OMNI-API
✅ Implements at least 3 cross-standard integrations
✅ Issues W3C Verifiable Credentials for achievements
✅ Registers with WIA Global Registry
✅ Passes WIA certification audit
✅ Maintains certification compliance
✅ Implements privacy-by-design across integrations
✅ Supports QR code verification
✅ Publishes integration documentation

---

## 10. Complete Integration Example

### 10.1 Full-Stack Educational Robot

```json
{
  "@context": "https://wiastandards.com/contexts/integration/v1",
  "@type": "WIAIntegratedEducationalRobot",
  "robotId": "EDU-ROBOT-2025-001",
  "certificationLevel": "WIA Verified",
  "integrations": {
    "WIA-AAC-002": {
      "purpose": "Voice interaction",
      "bidirectional": true,
      "status": "active"
    },
    "WIA-AI-006": {
      "purpose": "Emotion detection & adaptive response",
      "bidirectional": true,
      "status": "active"
    },
    "WIA-EDU-001": {
      "purpose": "LMS synchronization",
      "bidirectional": true,
      "status": "active"
    },
    "WIA-SEC-001": {
      "purpose": "Data privacy & security",
      "bidirectional": false,
      "status": "active"
    },
    "WIA-DATA-003": {
      "purpose": "Learning analytics",
      "bidirectional": false,
      "status": "active"
    }
  },
  "capabilities": {
    "teaching": [
      "adaptive-instruction",
      "personalized-feedback",
      "multi-modal-content",
      "gamification"
    ],
    "assessment": [
      "real-time-quizzing",
      "skill-gap-analysis",
      "mastery-tracking",
      "credential-issuance"
    ],
    "interaction": [
      "natural-language",
      "emotion-aware",
      "gesture-recognition",
      "multilingual"
    ]
  },
  "deploymentStats": {
    "schools": 47,
    "students": 3845,
    "sessionsCompleted": 28934,
    "credentialsIssued": 1247,
    "averageEngagement": 89,
    "satisfactionScore": 4.8
  },
  "wiaEcosystemBenefits": {
    "interoperability": true,
    "verifiableCredentials": true,
    "globalRegistry": true,
    "crossStandardData": true,
    "certifiedSafety": true
  }
}
```

---

**弘益人間 (Benefit All Humanity)**

*Together, we build a global educational ecosystem that serves all students, everywhere.*

*WIA - World Certification Industry Association*
*© 2025 MIT License*


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
