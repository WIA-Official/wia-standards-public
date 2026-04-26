# WIA-AUG-012 PHASE 1 — Data Format Specification

**Standard:** WIA-AUG-012
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 1 of 4)

---

# WIA-AUG-012: Augmentation Ethics Specification v1.0

> **Standard ID:** WIA-AUG-012
> **Version:** 1.0.0
> **Published:** 2025-12-27
> **Status:** Active
> **Authors:** WIA Human Augmentation Ethics Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Ethical Principles Framework](#2-ethical-principles-framework)
3. [Informed Consent Protocols](#3-informed-consent-protocols)
4. [Enhancement vs Therapy Distinction](#4-enhancement-vs-therapy-distinction)
5. [Equity and Access Considerations](#5-equity-and-access-considerations)
6. [Coercion Prevention](#6-coercion-prevention)
7. [Identity and Authenticity Preservation](#7-identity-and-authenticity-preservation)
8. [Reversibility Requirements](#8-reversibility-requirements)
9. [Vulnerable Population Protections](#9-vulnerable-population-protections)
10. [Implementation Guidelines](#10-implementation-guidelines)
11. [References](#11-references)

---

## 1. Introduction

### 1.1 Purpose

This specification establishes comprehensive ethical frameworks for human augmentation technologies, ensuring that the enhancement of human capabilities respects fundamental human rights, promotes individual autonomy, maintains human dignity, and advances social justice.

### 1.2 Scope

The standard covers:
- Ethical principles for augmentation design and deployment
- Informed consent frameworks and protocols
- Distinction between therapeutic and enhancement augmentation
- Equity, access, and justice considerations
- Prevention of coercion and undue influence
- Preservation of personal identity and authenticity
- Reversibility standards and requirements
- Special protections for vulnerable populations
- Societal impact assessment

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - Human augmentation should enhance human flourishing while respecting the inherent dignity, autonomy, and equality of all persons. Technology must serve humanity, not diminish it.

### 1.4 Terminology

- **Augmentation**: Technology that enhances human physical or cognitive capabilities
- **Therapeutic**: Treatment of disease or disability to restore normal function
- **Enhancement**: Improvement beyond species-typical functioning
- **Autonomy**: Capacity for self-determination and informed decision-making
- **Coercion**: Pressure that compromises voluntary decision-making
- **Vulnerable Population**: Groups requiring special ethical protections
- **Reversibility**: Ability to restore pre-augmentation state

---

## 2. Ethical Principles Framework

### 2.1 Six Core Ethical Principles

All human augmentation must be evaluated against six fundamental ethical principles:

#### 2.1.1 Autonomy

**Definition**: Respect for individual self-determination and decision-making capacity.

**Requirements**:
- Informed consent must be obtained
- Freedom from coercion or undue influence
- Right to accept or refuse augmentation
- Right to withdraw consent at any time
- Cultural and religious values respected

**Assessment Criteria**:
```
Autonomy Score = (Information × Comprehension × Voluntariness × Capacity) / 4

Where each factor is rated 0-10:
- Information: Completeness of disclosure
- Comprehension: Subject's understanding
- Voluntariness: Freedom from coercion
- Capacity: Decision-making ability
```

**Thresholds**:
- Score ≥ 8.0: Autonomy satisfied
- Score 6.0-7.9: Concerns requiring mitigation
- Score < 6.0: Autonomy not satisfied

#### 2.1.2 Beneficence

**Definition**: Actions must promote the well-being and best interests of the individual.

**Requirements**:
- Expected benefits must be significant
- Benefits must outweigh risks
- Quality of life improvement demonstrated
- Subject's own values and goals considered
- Long-term benefits evaluated

**Benefit Categories**:
1. Medical/Therapeutic: Improved health, function restoration
2. Functional: Enhanced capabilities, productivity
3. Psychological: Increased well-being, self-efficacy
4. Social: Improved relationships, opportunities

**Assessment**:
```
Beneficence Score = Σ(Benefit_i × Weight_i) - Risk_Burden

Where:
- Benefits are weighted by importance to subject
- Risk burden includes physical, psychological, social costs
```

#### 2.1.3 Non-Maleficence

**Definition**: "First, do no harm" - Obligation to avoid causing harm.

**Requirements**:
- Risks minimized to lowest feasible level
- Safety standards met (WIA-AUG-013)
- Harm prevention protocols in place
- Emergency procedures established
- Long-term safety monitored

**Harm Categories**:
1. Physical: Injury, disease, disability
2. Psychological: Mental distress, identity disruption
3. Social: Stigma, discrimination, isolation
4. Economic: Financial burden, employment impact

**Risk-Benefit Analysis**:
```
Acceptable Risk = (Expected Benefits / Potential Harms) ≥ Threshold

Thresholds by Augmentation Type:
- Therapeutic: 1.5
- Restorative: 2.0
- Enhancement: 3.0
- Experimental: 5.0
```

#### 2.1.4 Justice

**Definition**: Fair and equitable treatment, distribution, and access.

**Requirements**:
- No discrimination based on protected characteristics
- Fair selection criteria for access
- Equitable distribution of benefits and burdens
- Consideration of social determinants
- Remediation of existing inequities

**Justice Dimensions**:

**Distributive Justice**:
```
Access_Equity = (Actual_Access / Population_Need) across demographic groups

Target: Variance < 0.15 across groups
```

**Procedural Justice**:
- Fair decision-making processes
- Transparency in selection criteria
- Right to appeal decisions
- Stakeholder participation

**Compensatory Justice**:
- Priority for disadvantaged groups
- Subsidized access programs
- Accommodation for disabilities
- Remediation of past injustices

#### 2.1.5 Dignity

**Definition**: Respect for inherent human worth and inviolability.

**Requirements**:
- Human worth not contingent on capabilities
- Protection from degradation or objectification
- Respect for human embodiment
- Cultural and personal values honored
- Rights and personhood maintained

**Dignity Threats**:
1. Commodification: Treating persons as products
2. Instrumentalization: Using persons as means only
3. Dehumanization: Denying human status
4. Objectification: Reducing to functional capabilities

**Dignity Assessment**:
```
Questions to evaluate:
- Does augmentation respect human embodiment?
- Is person valued beyond augmented capabilities?
- Are human rights and personhood maintained?
- Is cultural/religious identity respected?
- Is there protection from degradation?
```

#### 2.1.6 Authenticity

**Definition**: Preservation of personal identity and genuine self.

**Requirements**:
- Changes aligned with person's values and goals
- Core identity elements preserved
- Narrative continuity maintained
- Alienation from self prevented
- Reversibility available when feasible

**Identity Dimensions**:
1. Psychological: Memories, personality, consciousness
2. Physical: Embodiment, sensorimotor experience
3. Narrative: Life story, personal history
4. Social: Relationships, roles, community
5. Values: Beliefs, commitments, goals

**Authenticity Assessment**:
```
Identity_Impact = Σ(Change_i × Centrality_i)

Where:
- Change: Degree of alteration (0-10)
- Centrality: Importance to core identity (0-10)

Thresholds:
- Impact < 25: Minimal identity change
- Impact 25-50: Moderate change, monitoring required
- Impact > 50: Substantial change, enhanced consent required
```

### 2.2 Principle Integration

All six principles must be satisfied for ethical compliance:

```typescript
interface EthicalAssessment {
  principleScores: {
    autonomy: number;
    beneficence: number;
    nonMaleficence: number;
    justice: number;
    dignity: number;
    authenticity: number;
  };

  overallCompliance: boolean;
  concerns: string[];
  recommendations: string[];
}

function assessCompliance(scores: PrincipleScores): boolean {
  return Object.values(scores).every(score => score >= THRESHOLD);
}
```

**Compliance Requirements**:
- All six principles score ≥ 7.0/10
- No principle scores < 5.0/10
- Overall average ≥ 8.0/10
- All critical concerns addressed

---

## 3. Informed Consent Protocols

### 3.1 Consent Level Framework

Four levels of informed consent based on augmentation type:

| Level | Type | Requirements | Duration |
|-------|------|-------------|----------|
| **BASIC** | Therapeutic | Standard medical consent | Single session |
| **ENHANCED** | Restorative | Detailed risks/benefits | Multiple sessions |
| **COMPREHENSIVE** | Enhancement | Full long-term implications | Extended process |
| **EXPERIMENTAL** | Experimental | Complete uncertainty disclosure | Ongoing |

### 3.2 Basic Consent (Therapeutic Augmentation)

**Applies to**: Medically necessary augmentation for disease/disability treatment

**Required Elements**:
1. Nature and purpose of augmentation
2. Expected benefits
3. Material risks and complications
4. Available alternatives
5. Right to refuse
6. Opportunity to ask questions

**Process**:
```
1. Information disclosure (written + verbal)
2. Comprehension assessment
3. Voluntary agreement
4. Documentation and signature
5. Cooling-off period (24 hours minimum)
```

### 3.3 Enhanced Consent (Restorative Augmentation)

**Applies to**: Restoration of normal function through augmentation

**Additional Requirements**:
- Detailed risk-benefit analysis
- Long-term maintenance requirements
- Lifestyle impact assessment
- Financial obligations
- Success rate statistics
- Failure scenarios and contingencies

**Process**:
```
Session 1: Initial information and assessment
Session 2: Detailed discussion of risks and benefits (72 hours later)
Session 3: Final consent and documentation (1 week later)
```

### 3.4 Comprehensive Consent (Enhancement Augmentation)

**Applies to**: Augmentation beyond normal capabilities

**Additional Requirements**:
- Philosophical and ethical implications
- Identity and authenticity impacts
- Social and occupational consequences
- Societal implications
- Reversibility options and limitations
- Long-term unknown risks
- Enhancement alternatives (training, tools)

**Process**:
```
Phase 1: Education (multiple sessions over 2+ weeks)
Phase 2: Psychological assessment
Phase 3: Ethics committee review
Phase 4: Trial period (if applicable)
Phase 5: Final comprehensive consent
Phase 6: Cooling-off period (30 days minimum)
```

**Documentation Requirements**:
```json
{
  "consentType": "COMPREHENSIVE",
  "subject": {
    "id": "anonymized",
    "age": 0,
    "capacityAssessment": "documented",
    "psychologicalEvaluation": "completed"
  },
  "augmentation": {
    "type": "enhancement",
    "category": "cognitive|physical|sensory",
    "description": "detailed",
    "reversibility": "score 0-1"
  },
  "disclosures": {
    "risks": ["comprehensive list"],
    "benefits": ["detailed benefits"],
    "alternatives": ["non-augmentation options"],
    "unknowns": ["long-term uncertainties"],
    "identity": ["potential impacts"],
    "social": ["societal implications"]
  },
  "sessions": [
    {
      "date": "ISO-8601",
      "topics": ["covered"],
      "comprehension": "verified",
      "questions": ["addressed"]
    }
  ],
  "signatures": {
    "subject": "signed with date",
    "witness": "independent witness",
    "ethicsOfficer": "ethics approval",
    "physician": "medical clearance"
  }
}
```

### 3.5 Experimental Consent

**Applies to**: Unproven or research-stage augmentation

**Additional Requirements**:
- Explicit acknowledgment of experimental nature
- Complete disclosure of uncertainties
- No guarantee of benefits
- Potential for unforeseen consequences
- Right to withdraw at any time
- Compensation for research injuries
- Independent oversight (IRB/Ethics Board)

**Special Provisions**:
- Ongoing consent (periodic renewal)
- Enhanced monitoring and reporting
- Immediate notification of new risks
- Community consultation (if applicable)
- Public registry of experimental augmentations

### 3.6 Capacity Assessment

All consent requires verified decision-making capacity:

```
Capacity Criteria:
1. Understanding: Comprehends information
2. Appreciation: Recognizes personal relevance
3. Reasoning: Weighs risks and benefits rationally
4. Expression: Communicates clear choice

Assessment Methods:
- Standardized capacity instruments
- Clinical evaluation by qualified professional
- Documentation of assessment results
- Periodic reassessment for experimental consent
```

**Incapacity Protocols**:
- Surrogate decision-makers (therapeutic only)
- Best interest standards
- Advance directives honored
- Legal guardianship requirements
- No enhancement for incapacitated persons

### 3.7 Consent Documentation

All consent must be documented with:

```
Required Documentation:
□ Consent form (signed and dated)
□ Information disclosure record
□ Comprehension assessment results
□ Capacity evaluation
□ Questions and answers log
□ Witness attestation (for enhanced/comprehensive)
□ Ethics committee approval (for comprehensive/experimental)
□ Cooling-off period confirmation
□ Right to withdraw notification
```

---

## 4. Enhancement vs Therapy Distinction

### 4.1 Classification Framework

Clear distinction between therapeutic and enhancement augmentation:

```
THERAPEUTIC:
- Treats disease or pathology
- Restores to normal functioning
- Medically indicated
- Standard medical ethics apply

RESTORATIVE:
- Repairs injury or deficit
- Returns to baseline capabilities
- Medically beneficial
- Enhanced disclosure required

ENHANCEMENT:
- Exceeds normal capabilities
- Improves beyond species-typical
- Elective choice
- Comprehensive ethics review required

EXPERIMENTAL:
- Unproven technology
- Research purposes
- Uncertain outcomes
- Full research ethics protocols
```

### 4.2 Classification Algorithm

```typescript
enum AugmentationType {
  THERAPEUTIC = 'THERAPEUTIC',


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
