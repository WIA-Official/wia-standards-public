# WIA-AUG-012 PHASE 2 — API Interface Specification

**Standard:** WIA-AUG-012
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 2 of 4)

---

RESTORATIVE = 'RESTORATIVE',
  ENHANCEMENT = 'ENHANCEMENT',
  EXPERIMENTAL = 'EXPERIMENTAL'
}

interface ClassificationInput {
  currentFunction: number;      // 0-100, population baseline = 50
  targetFunction: number;        // 0-100
  medicalNecessity: boolean;
  pathologyPresent: boolean;
  provenEffective: boolean;
}

function classifyAugmentation(input: ClassificationInput): AugmentationType {
  // Experimental if unproven
  if (!input.provenEffective) {
    return AugmentationType.EXPERIMENTAL;
  }

  // Therapeutic if treating pathology
  if (input.pathologyPresent && input.medicalNecessity) {
    return AugmentationType.THERAPEUTIC;
  }

  // Restorative if returning to baseline
  if (input.currentFunction < 50 && input.targetFunction <= 50) {
    return AugmentationType.RESTORATIVE;
  }

  // Enhancement if exceeding normal
  if (input.targetFunction > 50) {
    return AugmentationType.ENHANCEMENT;
  }

  return AugmentationType.RESTORATIVE;
}
```

### 4.3 Ethical Implications by Type

#### Therapeutic Augmentation
- Standard medical ethics
- Insurance coverage appropriate
- Physician discretion primary
- Basic informed consent sufficient
- Public health priority

#### Restorative Augmentation
- Enhanced ethical review
- Insurance consideration warranted
- Shared decision-making
- Enhanced informed consent
- Individual and social benefits

#### Enhancement Augmentation
- Comprehensive ethical review
- Private payment expected
- Individual autonomy primary
- Comprehensive informed consent
- Equity concerns heightened
- Identity impacts significant
- Societal implications major

#### Experimental Augmentation
- Research ethics protocols
- IRB/Ethics board required
- Compensation for participation
- Experimental consent
- Maximum oversight
- Public interest high
- Unknown risk profile

### 4.4 Gray Areas

Some augmentation defies clear classification:

**Example: Cognitive Enhancement for Age-Related Decline**
- Therapeutic? (treating aging as disease)
- Restorative? (returning to younger baseline)
- Enhancement? (exceeding current normal aging)

**Resolution Framework**:
1. Apply most stringent ethical requirements
2. Seek ethics committee guidance
3. Document rationale clearly
4. Apply comprehensive consent
5. Monitor outcomes carefully

---

## 5. Equity and Access Considerations

### 5.1 Justice Framework

Augmentation must not exacerbate existing inequities:

**Distributive Justice Principles**:
1. Equal access to therapeutic augmentation
2. Fair allocation of scarce resources
3. Priority to medical need over enhancement
4. Consideration of social determinants
5. Remediation of structural inequities

### 5.2 Access Barriers

Identified barriers requiring mitigation:

#### Economic Barriers
```
Cost Assessment:
- Procedure cost
- Maintenance/upgrades
- Time off work
- Travel and accommodations
- Lost opportunity costs

Mitigation:
- Sliding scale pricing
- Payment plans
- Charitable funding
- Public subsidies (therapeutic)
- Insurance coverage mandates
```

#### Geographic Barriers
- Rural/remote access
- Provider concentration in urban areas
- Travel requirements
- Telemedicine options

#### Social Barriers
- Stigma and discrimination
- Cultural appropriateness
- Language access
- Health literacy
- Trust in medical system

#### Systemic Barriers
- Insurance coverage gaps
- Regulatory restrictions
- Provider biases
- Institutional discrimination

### 5.3 Equity Assessment

```typescript
interface EquityAssessment {
  accessBarriers: {
    economic: number;      // 0-10
    geographic: number;
    social: number;
    systemic: number;
  };

  demographicDistribution: {
    income: DistributionMetric;
    race: DistributionMetric;
    geography: DistributionMetric;
    disability: DistributionMetric;
  };

  equityScore: number;      // 0-100
  concerns: string[];
  mitigations: string[];
}

interface DistributionMetric {
  giniCoefficient: number;  // 0 = perfect equality, 1 = perfect inequality
  representationRatio: number; // Actual / Expected distribution
}

// Equity score calculation
EquityScore = 100 - (Σ(Barrier_i × Weight_i) + Σ(Inequality_j × Weight_j))

// Thresholds:
// Score ≥ 80: Equitable access
// Score 60-79: Concerns, mitigation required
// Score < 60: Inequitable, major intervention needed
```

### 5.4 Equity Requirements

**Therapeutic Augmentation**:
- Universal access mandate
- Public health priority
- Insurance coverage required
- Sliding scale for uninsured
- Outreach to underserved

**Restorative Augmentation**:
- Broad access encouraged
- Insurance consideration
- Subsidies for low-income
- Anti-discrimination protections

**Enhancement Augmentation**:
- Market allocation acceptable
- Anti-discrimination laws apply
- Monitor for inequity
- Prevent coercive advantage
- Public education on risks

### 5.5 Enhancement Divide Prevention

**Concerns**:
- Widening gap between enhanced and non-enhanced
- Creation of genetic/augmentation aristocracy
- Entrenched social stratification
- Decreased social mobility
- Discrimination against non-enhanced

**Safeguards**:
```
1. Anti-discrimination laws
   - Prohibit enhancement-based discrimination
   - Protect non-enhanced individuals
   - Mandate reasonable accommodations

2. Enhancement-neutral opportunities
   - Education accessible to all
   - Employment based on competence
   - Merit not contingent on enhancement

3. Social safety net
   - Universal basic services
   - Healthcare access for all
   - Economic opportunity regardless of enhancement

4. Monitoring and regulation
   - Track enhancement distribution
   - Assess social impact
   - Intervene if inequity emerges
   - Adjust policy as needed
```

---

## 6. Coercion Prevention

### 6.1 Coercion Definition

**Coercion**: Pressure that compromises voluntary decision-making through threats, force, or manipulation.

**Forms of Coercion**:
1. Direct: Explicit threats or force
2. Indirect: Structural pressures or incentives
3. Subtle: Social pressure or manipulation
4. Internalized: Self-imposed from societal norms

### 6.2 Coercion Risk Factors

```typescript
interface CoercionRisk {
  context: 'occupational' | 'military' | 'educational' | 'social' | 'familial';

  indicators: {
    mandatoryRequirement: boolean;
    employmentConsequence: boolean;
    peerPressure: boolean;
    authorityPressure: boolean;
    financialIncentive: boolean;
    limitedAlternatives: boolean;
    powerImbalance: boolean;
    timeConstraint: boolean;
  };

  riskLevel: 'none' | 'low' | 'moderate' | 'high' | 'severe';
}

// Risk calculation
CoercionRisk = Σ(Indicator_i × Weight_i) × ContextMultiplier

Thresholds:
- None: Score 0-1
- Low: Score 2-4
- Moderate: Score 5-7
- High: Score 8-10
- Severe: Score > 10 (immediate intervention)
```

### 6.3 Prohibited Coercive Practices

**Absolutely Prohibited**:
1. Mandatory augmentation for employment (with exceptions*)
2. Augmentation as prerequisite for education
3. Financial penalties for non-augmentation
4. Denial of services based on augmentation status
5. Manipulation through misinformation
6. Exploitation of vulnerability

*Exceptions: Highly specific occupational requirements with no reasonable alternative (case-by-case ethics review required)

### 6.4 Occupational Context

**Ethical Framework**:
```
Question 1: Is augmentation genuinely necessary for job function?
  → If No: Prohibited

Question 2: Are reasonable accommodations available?
  → If Yes: Accommodation required, augmentation optional

Question 3: Is augmentation narrowly tailored to specific need?
  → If No: Overly broad, prohibited

Question 4: Are alternatives available (tools, training, reassignment)?
  → If Yes: Alternatives must be offered

Question 5: Is there independent ethics review?
  → If No: Cannot proceed
```

**Permitted Occupational Requirements** (with safeguards):
- Specific sensory augmentation for safety-critical roles
- Narrow therapeutic augmentation for medical fitness
- Time-limited enhancement with full consent
- Employer-funded with no cost to employee
- Removal/reversal upon job separation
- Alternative positions available

**Prohibited Occupational Requirements**:
- General enhancement for productivity
- Cosmetic augmentation for customer service
- Monitoring/surveillance implants
- Augmentation transferring risk from employer to employee
- Enhancement as substitute for training or tools

### 6.5 Military Context

**Special Considerations**:
- Chain of command creates inherent coercion risk
- National security interests vs. individual rights
- Voluntariness difficult in military hierarchy
- Long-term consequences for veterans

**Ethical Requirements**:
```
1. Voluntary augmentation only
   - Explicit opt-in required
   - No career consequences for refusal
   - Alternative assignments available

2. Enhanced informed consent
   - Independent counseling
   - Extended cooling-off period
   - Right to withdraw consent

3. Service member protections
   - Full medical support
   - Removal upon discharge
   - VA coverage for complications
   - Disability benefits if harmed

4. Civilian ethics oversight
   - Independent ethics board
   - External review of programs
   - Public transparency

5. Reversibility priority
   - Maximum reversibility required
   - Removal protocols established
   - Long-term monitoring
```

### 6.6 Educational Context

**Prohibited**:
- Augmentation as admission requirement
- Enhancement for academic performance
- Pressure from educational institutions

**Permitted** (with safeguards):
- Therapeutic augmentation for disabilities (voluntary)
- Accommodations without augmentation
- Student autonomy in all decisions

### 6.7 Coercion Detection and Intervention

```typescript
function detectCoercion(context: Context): CoercionAssessment {
  const indicators = assessIndicators(context);
  const riskLevel = calculateRisk(indicators);

  if (riskLevel >= 'moderate') {
    return {
      coercionDetected: true,
      interventionsRequired: [
        'Independent counseling',
        'Extended cooling-off period',
        'Alternative options required',
        'Ethics committee review',
        'External advocacy'
      ]
    };
  }
}

// Intervention escalation
Moderate Risk → Enhanced consent + counseling
High Risk → Ethics review + alternative required
Severe Risk → Halt augmentation + investigation
```

---

## 7. Identity and Authenticity Preservation

### 7.1 Personal Identity Framework

Human augmentation can impact multiple dimensions of personal identity:

```typescript
interface IdentityDimensions {
  psychological: {
    memory: number;          // Impact on memories (0-10)
    personality: number;     // Changes to personality traits
    consciousness: number;   // Alterations to subjective experience
    emotions: number;        // Effects on emotional life
  };

  physical: {
    embodiment: number;      // Changes to bodily experience
    appearance: number;      // Visible physical changes
    capabilities: number;    // Functional changes
    sensorimotor: number;    // Perceptual/motor changes
  };

  narrative: {
    continuity: number;      // Disruption to life story
    meaning: number;         // Changes to life meaning
    autobiography: number;   // Alterations to personal history
  };

  social: {
    relationships: number;   // Impact on relationships
    roles: number;           // Changes to social roles
    community: number;       // Effects on community belonging
    identity: number;        // Social identity changes
  };

  values: {
    beliefs: number;         // Changes to core beliefs
    commitments: number;     // Shifts in commitments
    goals: number;           // Alterations to life goals
    worldview: number;       // Changes to worldview
  };
}
```

### 7.2 Identity Impact Assessment

```
IdentityImpact = Σ(Dimension_i × Centrality_i × Magnitude_i)

Where:
- Dimension: Psychological, Physical, Narrative, Social, Values
- Centrality: Importance to core identity (1-10)
- Magnitude: Degree of change (0-10)

Interpretation:
- Impact 0-25: Minimal (standard consent)
- Impact 26-50: Moderate (enhanced consent, monitoring)
- Impact 51-75: Substantial (comprehensive consent, support)
- Impact 76-100: Severe (may be unethical, extensive review required)
```

### 7.3 Authenticity Criteria

Augmentation respects authenticity when:

1. **Value Alignment**: Changes align with person's core values and goals
2. **Narrative Coherence**: Life story remains comprehensible and meaningful
3. **Psychological Continuity**: Connection to past self maintained
4. **Relational Identity**: Key relationships sustained
5. **Self-Recognition**: Person recognizes self in augmented state

**Assessment Questions**:
```
□ Does augmentation reflect subject's authentic goals?
□ Is change continuous with personal history?
□ Does person identify with augmented capabilities?
□ Are core values and commitments preserved?
□ Do important others still recognize person?
□ Can person narrate coherent life story?
□ Is augmentation chosen, not imposed?
□ Does person feel "like themselves"?
```

### 7.4 Identity Disruption Risks


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
