# WIA-AUG-012 PHASE 3 — Protocol Specification

**Standard:** WIA-AUG-012
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 3 of 4)

---

**High-Risk Augmentations**:
- Cognitive enhancements affecting personality
- Memory modification or enhancement
- Emotion regulation implants
- Radical appearance alterations
- Consciousness-altering technologies
- Value-influencing interventions

**Risk Mitigation**:
```
1. Gradual implementation
   - Incremental changes over time
   - Adaptation periods between stages
   - Monitoring of identity impacts

2. Reversibility options
   - Removal or deactivation possible
   - Return to baseline capabilities
   - Identity preservation priority

3. Psychological support
   - Pre-augmentation counseling
   - Ongoing identity therapy
   - Support for adaptation
   - Crisis intervention available

4. Social support
   - Peer support groups
   - Family involvement (with consent)
   - Community resources

5. Monitoring and intervention
   - Regular identity assessments
   - Early detection of disruption
   - Intervention if severe impact
   - Option to reverse/modify
```

### 7.5 Self-Alienation Prevention

**Self-Alienation**: Feeling disconnected from augmented self or capabilities.

**Warning Signs**:
- "This isn't really me"
- Disowning augmented capabilities
- Distress about identity changes
- Loss of narrative coherence
- Relationship disruptions
- Feeling like "a different person"

**Interventions**:
```
1. Identity integration therapy
2. Narrative reconstruction
3. Gradual adjustment of augmentation
4. Consideration of reversal
5. Peer support connection
6. Meaning-making facilitation
```

### 7.6 Enhancement and Character

**Ethical Concern**: Does enhancement undermine character development?

**Considerations**:
- Value of effort and struggle in human flourishing
- Meaning of accomplishment with vs. without enhancement
- Development of virtues through challenge
- Authentic achievement and self-worth
- "Shortcut" vs. "earned" capabilities

**Framework**:
```
Character preservation requires:
1. Enhancement aligned with person's developmental goals
2. Continued opportunity for meaningful challenge
3. Recognition of effort (enhanced and non-enhanced)
4. Value not contingent solely on capabilities
5. Character virtues cultivatable with augmentation
```

---

## 8. Reversibility Requirements

### 8.1 Reversibility Principle

**Principle**: Augmentation should be reversible unless irreversibility is justified by substantial benefit and explicitly consented to.

**Rationale**:
- Respects autonomy (can change mind)
- Mitigates identity risks
- Reduces coercion concerns
- Allows adaptation and adjustment
- Protects against unforeseen consequences

### 8.2 Reversibility Classification

```typescript
enum ReversibilityLevel {
  FULLY_REVERSIBLE = 'FULLY_REVERSIBLE',      // 90-100% restoration
  LARGELY_REVERSIBLE = 'LARGELY_REVERSIBLE',   // 70-89% restoration
  PARTIALLY_REVERSIBLE = 'PARTIALLY_REVERSIBLE', // 40-69% restoration
  MINIMALLY_REVERSIBLE = 'MINIMALLY_REVERSIBLE', // 10-39% restoration
  IRREVERSIBLE = 'IRREVERSIBLE'                // 0-9% restoration
}

interface ReversibilityProfile {
  level: ReversibilityLevel;
  restorationPercentage: number;  // 0-100
  reversalProcess: {
    surgical: boolean;
    duration: number;              // days
    risk: 'low' | 'moderate' | 'high';
    cost: number;
  };
  permanentChanges: string[];
  recovery: {
    physical: number;              // days
    psychological: number;         // days
    functional: number;            // days
  };
}
```

### 8.3 Reversibility Requirements by Augmentation Type

#### Therapeutic Augmentation
- Reversibility encouraged but not required
- Medical necessity may justify irreversibility
- Risk-benefit analysis determines requirements
- Enhanced consent for irreversible procedures

#### Restorative Augmentation
- Reversibility highly encouraged
- Irreversibility requires justification
- Alternative reversible options explored
- Detailed disclosure of permanent changes

#### Enhancement Augmentation
- **Reversibility strongly preferred**
- Irreversible enhancement requires:
  - Compelling justification
  - Comprehensive consent
  - Ethics committee approval
  - Extended cooling-off period
  - Psychological assessment
  - Trial of reversible alternatives

#### Experimental Augmentation
- **Maximum reversibility required**
- Irreversible experiments prohibited except:
  - Therapeutic necessity
  - No reversible alternative
  - IRB approval
  - Extraordinary consent process

### 8.4 Reversibility Assessment

```
Reversibility Score = (Physical × 0.4) + (Functional × 0.3) + (Identity × 0.3)

Where each component (0-100):

Physical Reversibility:
- Percentage of anatomical restoration
- Removal of implanted components
- Healing of surgical modifications

Functional Reversibility:
- Recovery of pre-augmentation capabilities
- Restoration of normal function
- Absence of permanent deficits

Identity Reversibility:
- Return to pre-augmentation sense of self
- Reversal of identity impacts
- Restoration of psychological continuity

Thresholds:
- ≥90: Fully reversible
- 70-89: Largely reversible
- 40-69: Partially reversible
- 10-39: Minimally reversible
- <10: Irreversible
```

### 8.5 Justifications for Irreversibility

Irreversible augmentation may be ethically acceptable when:

1. **Medical Necessity**: Therapeutic benefit requires irreversibility
2. **Technical Impossibility**: Reversibility not technologically feasible
3. **Proportionate Benefit**: Benefits vastly outweigh reversibility loss
4. **Subject Preference**: Informed subject explicitly chooses irreversibility
5. **Risk Reduction**: Reversibility would increase overall risk

**Required Documentation**:
```json
{
  "justification": "medical_necessity",
  "rationale": "Detailed explanation of why irreversibility is required",
  "alternatives": {
    "reversibleOptions": ["List of reversible alternatives considered"],
    "whyNotFeasible": ["Explanation of why alternatives inadequate"]
  },
  "consent": {
    "type": "COMPREHENSIVE",
    "irreversibilityDisclosed": true,
    "subjectAcknowledgment": "Signed statement understanding irreversibility",
    "coolingOffPeriod": "30 days minimum",
    "psychologicalAssessment": "Completed",
    "ethicsApproval": "Committee approval reference"
  }
}
```

### 8.6 Reversal Protocols

For reversible augmentations, clear reversal protocols required:

```
Reversal Protocol Components:
1. Eligibility criteria for reversal
2. Reversal process description
3. Timeline and stages
4. Expected outcomes and limitations
5. Risks and complications
6. Recovery and rehabilitation
7. Support services
8. Cost and coverage
9. Follow-up care

Reversal Rights:
- Right to request reversal at any time
- No penalty for requesting reversal
- Support for reversal decision
- Financial arrangements clear upfront
- Access to reversal regardless of ability to pay (therapeutic)
```

### 8.7 Partial Reversibility Management

When full reversibility impossible, requirements include:

```
1. Clear disclosure
   - Specific permanent changes identified
   - Percentage reversibility quantified
   - Limitations explicitly described

2. Staged approach
   - Maximize reversible options first
   - Delay irreversible components
   - Trial periods before permanence

3. Enhanced consent
   - Multiple sessions
   - Independent review
   - Psychological assessment
   - Extended cooling-off period

4. Ongoing support
   - Long-term monitoring
   - Adaptation assistance
   - Intervention if needed
```

---

## 9. Vulnerable Population Protections

### 9.1 Vulnerable Populations Defined

Groups requiring special ethical protections:

1. **Children and Adolescents**
2. **Individuals with Cognitive Impairments**
3. **Economically Disadvantaged**
4. **Institutionalized Persons**
5. **Ethnic and Racial Minorities** (historical discrimination)
6. **Military Personnel** (context-dependent)
7. **Prisoners**
8. **Refugees and Displaced Persons**

### 9.2 Children and Adolescents

**General Principle**: **Therapeutic only** until age of majority.

#### 9.2.1 Therapeutic Augmentation in Children

**Permitted when**:
- Medically necessary
- Cannot safely wait until adulthood
- Expected benefit substantial
- Risks proportionate
- Best interest of child

**Requirements**:
```
1. Parental/Guardian consent
2. Child assent (if capable)
3. Independent medical evaluation
4. Ethics committee review
5. Least invasive option
6. Maximum reversibility
7. Delayed timing if possible
8. Child's future autonomy preserved
```

#### 9.2.2 Enhancement Augmentation in Children

**General Prohibition**: Enhancement of children prohibited.

**Rationale**:
- Cannot provide informed consent
- Identity still developing
- Cannot appreciate long-term implications
- Risk of parental coercion
- Permanent alteration of developing person
- Violates child's future autonomy

**Absolute Prohibitions**:
- Cognitive enhancement
- Cosmetic enhancement
- Performance enhancement (academic, athletic)
- Social enhancement
- Any irreversible enhancement

**Possible Exception** (case-by-case, extraordinary circumstances):
- Immediate and severe harm without enhancement
- No therapeutic alternative
- Independent advocacy for child
- Judicial review
- Maximum reversibility required
- Deferred until adolescence if possible

#### 9.2.3 Adolescent Augmentation

**Ages 16-18**: Transitional framework

```
Therapeutic:
- Standard pediatric protections
- Increasing weight to adolescent assent
- Preparation for adult decision-making

Restorative:
- May be considered case-by-case
- Ethics review required
- Adolescent assent essential
- Parental consent still required
- Prefer to delay until 18 if safe

Enhancement:
- Generally prohibited
- Extraordinary circumstances only
- Judicial review
- Independent advocacy
- Maximum protections
```

### 9.3 Cognitive Impairment

**Principle**: Therapeutic only, best interest standard, surrogate consent with safeguards.

#### 9.3.1 Decision-Making Capacity Assessment

```
Capacity Domains:
1. Understanding: Comprehends information
2. Appreciation: Recognizes personal relevance
3. Reasoning: Weighs options rationally
4. Expression: Communicates choice

Assessment:
- Standardized instruments
- Qualified evaluator
- Domain-specific (may have partial capacity)
- Periodic reassessment
- Documentation required
```

#### 9.3.2 Surrogate Decision-Making

**When capacity lacking**:

```
Surrogate Decision-Maker Selection:
1. Legal guardian (if appointed)
2. Healthcare proxy (if designated)
3. Family member (spouse, adult child, parent, sibling)
4. Close friend (if no family)
5. Public guardian (last resort)

Surrogate Standards:
1. Substituted judgment: What would person want?
2. Best interest: What serves person's well-being?

Requirements:
- Surrogate acts in person's best interest
- Person's known preferences considered
- Least restrictive alternative
- Preserves person's dignity
- Independent oversight
```

#### 9.3.3 Protections for Cognitively Impaired

```
1. Therapeutic priority
   - Enhancement prohibited
   - Medical necessity required
   - Improvement in function/well-being

2. Reversibility maximum
   - Fully reversible strongly preferred
   - Irreversibility requires extraordinary justification

3. Independent advocacy
   - Separate from surrogate
   - Represents person's interests
   - Can object to augmentation

4. Ethics committee review
   - All cases reviewed
   - Particular scrutiny for irreversible
   - Community representation

5. Judicial review
   - For contested cases
   - Irreversible augmentation
   - Substantial identity impact

6. Ongoing monitoring
   - Regular assessment
   - Early intervention
   - Right to discontinue
```

### 9.4 Economically Disadvantaged

**Concerns**:
- Exploitation through financial incentives
- Undue inducement to participate in experiments
- Pressure to augment for employment
- Inability to afford complications/maintenance
- Systematic exclusion from beneficial augmentation

**Protections**:

```
1. Fair compensation
   - Research: Reasonable compensation, not undue inducement
   - Employment: No cost to individual for job-required augmentation

2. Access programs
   - Sliding scale pricing
   - Charitable funding
   - Public subsidies (therapeutic)
   - Insurance mandates

3. Anti-exploitation measures
   - Independent oversight
   - Screening for economic coercion
   - Alternative options required
   - Right to withdraw without penalty

4. Long-term support
   - Maintenance coverage
   - Upgrade access
   - Complication treatment
   - Reversal funding
```

### 9.5 Institutionalized Persons

**Principle**: Highest scrutiny, therapeutic only, strong presumption against augmentation.

#### 9.5.1 Prisoners

```
Permitted:
- Medically necessary therapeutic augmentation
- Standard medical care

Prohibited:
- Enhancement augmentation
- Experimental augmentation (unless directly therapeutic)
- Behavior modification implants


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

## Annex I — Interoperability Profiles

This annex describes how implementations declare interoperability profiles
for PHASE-3-PROTOCOL. The profile mechanism is non-normative and exists so that
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
  advertises `WIA-P3-PROTOCOL-Edge-only/2` is asserting conformance with
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
  is published manually. Sufficient for PHASE-3-PROTOCOL validation when the
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
