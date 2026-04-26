# Phase 1: Data Format Specification - WIA-ROB-013

## WIA-ROB-013 Companion Robot Data Format Standard

**Version**: 1.0.0  
**Date**: 2025-01-15  
**Status**: Active  
**Standard ID**: WIA-ROB-013-PHASE1-001

---

## 1. Overview

The WIA-ROB-013 Companion Robot Standard defines comprehensive data formats for AI companions that provide emotional support, social interaction, and personal assistance. This phase establishes interoperable data structures enabling cross-platform compatibility and data portability.

### 1.1 Purpose

- Enable interoperability between companion robot platforms
- Standardize emotion representation and recognition
- Define personality trait encoding
- Establish conversation history formats
- Support data portability and user control

### 1.2 Scope

This specification covers:
- Companion robot profile data structures
- Interaction message formats
- Emotion recognition data
- Personality trait encoding
- Memory and context representations
- User preference formats

### 1.3 Philosophy

**홍익인간 (弘益人間)** - Benefit All Humanity

All data formats prioritize user well-being, privacy, autonomy, and safety.

---

## 2. Companion Profile Format

### 2.1 Basic Profile Structure

```typescript
interface CompanionProfile {
  id: string;                    // UUID v7 (time-sorted)
  version: string;               // Semantic version
  createdAt: ISO8601;
  lastUpdated: ISO8601;
  
  // Identity
  name: string;
  avatar?: string;               // Base64 or URI
  description: string;
  
  // Personality (OCEAN+ model)
  personality: PersonalityTraits;
  
  // Configuration
  primaryPurpose: CompanionPurpose;
  targetAgeRange: AgeRange;
  communicationStyle: CommunicationStyle;
  
  // Languages
  languagePrimary: string;       // ISO 639-1
  languagesSupported: string[];
  
  // Capabilities
  capabilities: Capability[];
  limitations: string[];
}
```

### 2.2 Personality Traits (OCEAN+)

```typescript
interface PersonalityTraits {
  openness: number;              // 0-100
  conscientiousness: number;     // 0-100
  extraversion: number;          // 0-100
  agreeableness: number;         // 0-100
  neuroticism: number;           // 0-100
  
  // Extended traits
  playfulness: number;           // 0-100
  formality: number;             // 0-100
  proactivity: number;           // 0-100
}

enum CompanionPurpose {
  EMOTIONAL_SUPPORT = "emotional_support",
  EDUCATION = "education",
  HEALTH_WELLNESS = "health_wellness",
  ENTERTAINMENT = "entertainment",
  DAILY_ASSISTANCE = "daily_assistance"
}

enum AgeRange {
  CHILD = "child",         // 5-12
  TEEN = "teen",           // 13-17
  ADULT = "adult",         // 18-64
  SENIOR = "senior"        // 65+
}
```

---

## 3. Interaction Message Format

### 3.1 Message Structure

```typescript
interface InteractionMessage {
  id: string;                    // UUID v7
  timestamp: ISO8601;
  sender: "user" | "companion";
  
  // Content
  content: MessageContent;
  
  // Context
  context: MessageContext;
  
  // Metadata
  metadata: MessageMetadata;
}

interface MessageContent {
  text: string;
  language: string;              // ISO 639-1
  
  emotionalTone?: EmotionalTone;
  
  // Optional multimodal content
  voice?: VoiceData;
  image?: ImageData;
}

interface MessageContext {
  conversationId: string;
  sessionId: string;
  threadId?: string;
  
  // User state
  userEmotion?: EmotionState;
  userActivity?: string;
  userLocation?: string;
  
  // Temporal context
  timeOfDay: "morning" | "afternoon" | "evening" | "night";
  dayOfWeek: string;
  specialOccasion?: string;
}
```

### 3.2 Emotional Tone

```typescript
interface EmotionalTone {
  valence: number;               // -1.0 (negative) to +1.0 (positive)
  arousal: number;               // 0.0 (calm) to 1.0 (excited)
  dominance: number;             // -1.0 (submissive) to +1.0 (dominant)
  intensity: number;             // 0.0 to 1.0
}
```

---

## 4. Emotion Recognition Format

### 4.1 Emotion State

```typescript
interface EmotionState {
  timestamp: ISO8601;
  
  // Discrete emotion model
  primaryEmotion: PrimaryEmotion;
  secondaryEmotions: EmotionProbability[];
  
  // Dimensional model
  dimensions: EmotionalDimensions;
  
  // Confidence and sources
  overallConfidence: number;     // 0.0 to 1.0
  sources: EmotionSources;
}

enum PrimaryEmotion {
  JOY = "joy",
  SADNESS = "sadness",
  ANGER = "anger",
  FEAR = "fear",
  SURPRISE = "surprise",
  DISGUST = "disgust",
  CONTEMPT = "contempt"
}

interface EmotionProbability {
  emotion: string;
  probability: number;           // 0.0 to 1.0
}

interface EmotionalDimensions {
  valence: number;               // -1.0 to +1.0
  arousal: number;               // 0.0 to 1.0
  dominance: number;             // -1.0 to +1.0
}

interface EmotionSources {
  text?: {
    emotion: PrimaryEmotion;
    confidence: number;
  };
  voice?: {
    emotion: PrimaryEmotion;
    confidence: number;
  };
  facial?: {
    emotion: PrimaryEmotion;
    confidence: number;
  };
}
```

---

## 5. Memory and Context Format

### 5.1 Long-Term Memory

```typescript
interface LongTermMemory {
  userId: string;
  companionId: string;
  
  // User profile
  userProfile: UserProfile;
  
  // Relationship history
  firstInteraction: ISO8601;
  totalInteractions: number;
  lastInteraction: ISO8601;
  
  // Learned preferences
  preferences: UserPreferences;
  
  // Important memories
  memories: Memory[];
}

interface UserProfile {
  personalInfo: {
    name?: string;
    interests: string[];
    goals: string[];
    challenges: string[];
  };
  
  communicationPreferences: {
    preferredLanguage: string;
    formalityLevel: number;      // 0-100
    verbosity: number;            // 0-100
    emojiUsage: number;           // 0-100
  };
  
  emotionalProfile: {
    baselineAffect: EmotionalTone;
    expressionStyle: "reserved" | "moderate" | "expressive";
    commonTriggers: {
      positive: string[];
      negative: string[];
    };
    copingStrategies: string[];
  };
}

interface Memory {
  id: string;
  timestamp: ISO8601;
  type: "event" | "fact" | "preference" | "goal" | "concern";
  content: string;
  importance: number;            // 0.0 to 1.0
  lastAccessed: ISO8601;
  accessCount: number;
}
```

---

## 6. Safety and Crisis Data

### 6.1 Crisis Detection

```typescript
interface CrisisDetection {
  timestamp: ISO8601;
  severity: "low" | "medium" | "high" | "critical";
  type: CrisisType;
  indicators: string[];
  confidence: number;            // 0.0 to 1.0
  
  // Required actions
  recommendedActions: string[];
  resourcesProvided: CrisisResource[];
  
  // Follow-up
  followUpRequired: boolean;
  followUpTiming?: string;
}

enum CrisisType {
  SUICIDAL_IDEATION = "suicidal_ideation",
  SELF_HARM = "self_harm",
  HARM_TO_OTHERS = "harm_to_others",
  SEVERE_DISTRESS = "severe_distress",
  ABUSE_DISCLOSURE = "abuse_disclosure",
  SUBSTANCE_CRISIS = "substance_crisis"
}

interface CrisisResource {
  type: "hotline" | "emergency" | "text_line" | "website";
  name: string;
  contact: string;
  availability: string;
  language: string[];
}
```

---

## 7. Data Validation

All data must pass validation before processing:

- **UUID Format**: Valid UUID v7 with proper timestamp encoding
- **Timestamp Format**: ISO 8601 with timezone (UTC preferred)
- **Range Validation**: All numeric scores within specified ranges
- **Language Codes**: Valid ISO 639-1 codes only
- **Text Length**: Maximum 10,000 characters per message
- **Enum Values**: Only defined enum values accepted

---

## 8. Data Portability

Users must be able to export their data in standard formats:

```typescript
interface DataExport {
  exportDate: ISO8601;
  version: string;
  
  companionProfile: CompanionProfile;
  userProfile: UserProfile;
  conversationHistory: InteractionMessage[];
  memories: Memory[];
  
  // Analytics (optional)
  analytics?: {
    totalInteractions: number;
    averageSatisfaction: number;
    topTopics: string[];
    emotionTrends: any;
  };
}
```

Export formats supported:
- JSON (required)
- CSV (for tabular data)
- PDF (human-readable report)

---

## 9. Privacy and Security

### 9.1 Data Protection Requirements

- All personal data encrypted at rest (AES-256)
- All transmissions use TLS 1.3+
- No data retention without explicit consent
- User-controlled data deletion
- Data minimization principles

### 9.2 Sensitive Data Handling

Sensitive categories requiring extra protection:
- Mental health information
- Crisis interactions
- Personal relationships
- Financial information
- Health conditions

---

## 10. Compliance Checklist

✓ Data formats follow standard specifications  
✓ All numeric values within valid ranges  
✓ Timestamps in ISO 8601 format  
✓ UUIDs properly generated and validated  
✓ Language codes are valid ISO 639-1  
✓ Emotion representations include confidence scores  
✓ Privacy and security requirements met  
✓ Data portability supported  
✓ Crisis detection properly structured  
✓ Documentation complete and accessible

---

**WIA-ROB-013 PHASE 1 - Data Format Specification**  
© 2025 World Certification Industry Association  
弘益人間 · Benefit All Humanity


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
