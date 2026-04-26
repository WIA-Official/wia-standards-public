# WIA-AUG-012 PHASE 4 — Integration Specification

**Standard:** WIA-AUG-012
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 4 of 4)

---

- Augmentation as condition of release
- Monitoring/surveillance devices

Requirements (if therapeutic):
- Independent medical necessity determination
- Separate from correctional authority
- Enhanced informed consent
- External ethics review
- Judicial oversight
- Community standard of care
```

#### 9.5.2 Psychiatric Institutions

```
Protections:
- Capacity assessment required
- Voluntary admission vs. involuntary (higher scrutiny)
- No coercion from institution
- Independent advocacy
- Right to refuse
- Periodic review
- Court approval (for involuntary patients)

Prohibited:
- Involuntary enhancement
- Augmentation for institutional convenience
- Experimental augmentation (generally)
```

### 9.6 Ethnic and Racial Minorities

**Historical Context**: Medical experimentation and exploitation.

**Protections**:

```
1. Trust building
   - Community engagement
   - Representation in research
   - Transparency in processes
   - Accountability mechanisms

2. Cultural sensitivity
   - Culturally appropriate consent
   - Language access
   - Cultural values respected
   - Community input

3. Anti-discrimination
   - Equal access to therapeutic augmentation
   - No targeting for experiments
   - Fair distribution of benefits
   - Monitoring for disparities

4. Community oversight
   - Community advisory boards
   - Meaningful participation
   - Veto power over research
   - Benefit sharing
```

### 9.7 Refugees and Displaced Persons

```
Concerns:
- Vulnerability to exploitation
- Pressure to augment for employment/asylum
- Limited alternatives
- Language and cultural barriers
- Lack of long-term support

Protections:
- Enhanced consent (language appropriate)
- Independent advocacy
- No augmentation linked to asylum/status
- Long-term support commitments
- Cultural sensitivity
- Community involvement
```

### 9.8 General Vulnerable Population Principles

```
1. Therapeutic priority: Enhancement generally prohibited
2. Maximum reversibility: Irreversibility requires extraordinary justification
3. Enhanced consent: Additional protections and oversight
4. Independent advocacy: Separate from interested parties
5. Ethics review: Mandatory for all cases
6. Judicial oversight: For high-risk or contested cases
7. Community involvement: Representation and input
8. Long-term support: Commitment beyond procedure
9. Right to withdraw: Can discontinue without penalty
10. Regular monitoring: Ongoing assessment and intervention
```

---

## 10. Implementation Guidelines

### 10.1 Ethical Review Structure

#### 10.1.1 Ethics Committee Composition

```
Required Members:
- Bioethicist (chair)
- Physician (augmentation specialist)
- Psychologist/Psychiatrist
- Legal expert
- Community representative
- Patient advocate
- Ethicist (second)
- Technical expert (as needed)

Desired Diversity:
- Gender balance
- Racial/ethnic diversity
- Disability representation
- Range of perspectives
```

#### 10.1.2 Review Process

```
Standard Review (Therapeutic):
1. Application submission
2. Committee review (2-4 weeks)
3. Decision: Approve, Deny, Request Modifications
4. Appeal process available

Enhanced Review (Restorative/Enhancement):
1. Pre-application consultation
2. Full application submission
3. Committee review (4-8 weeks)
4. Subject interview (if needed)
5. Decision with detailed rationale
6. Appeal to higher ethics board

Expedited Review:
- Therapeutic necessity
- Time-sensitive
- Minimal risk
- Single reviewer + chair
- Full committee notification

Ongoing Review (Experimental):
- Initial approval
- Periodic progress reports
- Adverse event reporting
- Annual renewal
- Modification approval required
```

### 10.2 Certification Requirements

To achieve WIA-AUG-012 certification:

```
Required Elements:
□ Ethics committee established
□ Consent protocols implemented (all 4 levels)
□ Augmentation classification system
□ Equity assessment process
□ Coercion screening procedures
□ Identity impact assessment
□ Reversibility evaluation
□ Vulnerable population protections
□ Documentation system
□ Training program
□ Audit mechanism
□ Public transparency

Documentation:
□ Ethics policy manual
□ Consent form templates
□ Assessment instruments
□ Committee meeting minutes
□ Decisions and rationale
□ Adverse event reports
□ Annual compliance reports
```

### 10.3 API Interface

```typescript
// Core ethical assessment function
interface EthicalAssessmentRequest {
  augmentation: AugmentationDetails;
  subject: SubjectProfile;
  context: DecisionContext;
}

interface EthicalAssessmentResult {
  compliant: boolean;
  principlesSatisfied: EthicalPrinciple[];
  concerns: EthicalConcern[];
  recommendations: string[];
  requiredActions: RequiredAction[];
  approvalLevel: 'automatic' | 'standard' | 'enhanced' | 'prohibited';
}

function assessEthicalCompliance(
  request: EthicalAssessmentRequest
): EthicalAssessmentResult;

// Consent validation
interface ConsentValidationRequest {
  subjectId: string;
  augmentationType: AugmentationType;
  requiredLevel: ConsentLevel;
  providedConsent: ConsentDocumentation;
}

interface ConsentValidationResult {
  valid: boolean;
  gaps: string[];
  recommendations: string[];
}

function validateConsent(
  request: ConsentValidationRequest
): ConsentValidationResult;

// Coercion check
interface CoercionCheckRequest {
  context: CoercionContext;
  indicators: CoercionIndicator[];
}

interface CoercionCheckResult {
  coercionDetected: boolean;
  riskLevel: 'none' | 'low' | 'moderate' | 'high' | 'severe';
  concerns: string[];
  interventions: string[];
}

function checkCoercion(
  request: CoercionCheckRequest
): CoercionCheckResult;

// Additional functions
function evaluateEquity(context: EquityContext): EquityAssessment;
function assessIdentityImpact(augmentation: AugmentationDetails): IdentityImpact;
function reviewReversibility(augmentation: AugmentationDetails): ReversibilityProfile;
function protectVulnerable(subject: SubjectProfile): VulnerableProtections;
```

### 10.4 Training Requirements

```
Ethics Training Program:
1. Foundational Ethics (8 hours)
   - Ethical principles
   - Bioethics frameworks
   - Augmentation-specific ethics

2. Consent Procedures (4 hours)
   - Informed consent elements
   - Capacity assessment
   - Documentation

3. Equity and Justice (4 hours)
   - Distributive justice
   - Access barriers
   - Anti-discrimination

4. Vulnerable Populations (4 hours)
   - Special protections
   - Surrogate consent
   - Community engagement

5. Practical Application (4 hours)
   - Case studies
   - Ethical dilemmas
   - Decision-making process

Certification:
- Written examination
- Practical assessment
- Annual recertification
- Continuing education (8 hours/year)
```

### 10.5 Monitoring and Compliance

```
Ongoing Monitoring:
- Quarterly ethics committee reviews
- Annual compliance audits
- Adverse event reporting
- Equity assessments
- Subject satisfaction surveys
- Community feedback

Compliance Metrics:
- Consent documentation completeness
- Ethics review timeliness
- Coercion incident rate
- Equity in access
- Adverse event frequency
- Complaint resolution
- Stakeholder satisfaction

Corrective Actions:
- Process improvements
- Additional training
- Policy revisions
- Individual counseling
- Certification suspension
- Public disclosure
```

---

## 11. References

### 11.1 Ethical Frameworks

1. Beauchamp, T.L. & Childress, J.F. (2019). *Principles of Biomedical Ethics* (8th ed.)
2. Habermas, J. (2003). *The Future of Human Nature*
3. Sandel, M.J. (2007). *The Case Against Perfection*
4. Fukuyama, F. (2002). *Our Posthuman Future*
5. Bostrom, N. & Savulescu, J. (2009). *Human Enhancement*

### 11.2 International Standards

1. UNESCO Universal Declaration on Bioethics and Human Rights (2005)
2. Council of Europe Oviedo Convention on Human Rights and Biomedicine (1997)
3. World Medical Association Declaration of Helsinki (2013)
4. Nuremberg Code (1947)
5. Belmont Report (1979)

### 11.3 Specific Issues

**Informed Consent**:
- Faden, R.R. & Beauchamp, T.L. (1986). *A History and Theory of Informed Consent*
- Appelbaum, P.S. (2007). Assessment of patients' competence to consent to treatment

**Enhancement Ethics**:
- President's Council on Bioethics (2003). *Beyond Therapy*
- Buchanan, A. (2011). *Better Than Human*

**Justice and Equity**:
- Daniels, N. (2008). *Just Health: Meeting Health Needs Fairly*
- Powers, M. & Faden, R. (2006). *Social Justice*

**Identity and Authenticity**:
- DeGrazia, D. (2005). *Human Identity and Bioethics*
- Elliott, C. (2003). *Better Than Well*

### 11.4 WIA Standards

- WIA-AUG-001: Human Augmentation General Standards
- WIA-AUG-013: Augmentation Safety
- WIA-AUG-014: Human-Machine Interface
- WIA-MED: Medical Device Standards
- WIA-DATA: Data Privacy and Rights

---

## Appendix A: Ethical Assessment Worksheet

```
Device/Augmentation: _______________
Date: _______________
Assessor: _______________

ETHICAL PRINCIPLES ASSESSMENT (Score 0-10 each)

1. Autonomy: ___
   □ Informed consent obtained
   □ Subject has capacity
   □ Free from coercion
   □ Right to withdraw respected

2. Beneficence: ___
   □ Expected benefits significant
   □ Benefits outweigh risks
   □ Quality of life improved
   □ Subject's values considered

3. Non-Maleficence: ___
   □ Risks minimized
   □ Safety standards met
   □ Harm prevention protocols
   □ Emergency procedures established

4. Justice: ___
   □ Fair access
   □ No discrimination
   □ Equitable distribution
   □ Social determinants considered

5. Dignity: ___
   □ Human worth respected
   □ No degradation/objectification
   □ Rights maintained
   □ Cultural values honored

6. Authenticity: ___
   □ Aligned with subject's values
   □ Identity preserved
   □ Narrative continuity maintained
   □ Self-alienation prevented

OVERALL ASSESSMENT:
Average Score: ___
Compliant (all ≥7.0): □ Yes □ No

CONCERNS:
1. _______________
2. _______________
3. _______________

RECOMMENDATIONS:
1. _______________
2. _______________
3. _______________

DECISION: □ Approve □ Conditional □ Deny

Signature: _______________ Date: _______________
```

## Appendix B: Consent Checklist

```
COMPREHENSIVE CONSENT CHECKLIST

Subject: _______________
Augmentation: _______________
Type: □ Therapeutic □ Restorative □ Enhancement □ Experimental

INFORMATION DISCLOSURE:
□ Nature and purpose of augmentation
□ Expected benefits
□ Material risks and complications
□ Long-term implications
□ Identity and authenticity impacts
□ Social and occupational consequences
□ Reversibility options and limitations
□ Alternatives (including no augmentation)
□ Costs and financial obligations
□ Right to refuse and withdraw

COMPREHENSION ASSESSMENT:
□ Capacity evaluation completed
□ Understanding verified
□ Appreciation confirmed
□ Reasoning demonstrated
□ Questions answered

VOLUNTARINESS:
□ Coercion screening completed
□ No undue influence detected
□ Decision freely made
□ Adequate time for decision
□ Independent counseling offered

SPECIAL CONSIDERATIONS:
□ Vulnerable population protections (if applicable)
□ Cultural/religious considerations addressed
□ Language access provided
□ Independent advocacy (if required)

DOCUMENTATION:
□ Consent form signed and dated
□ Information disclosure documented
□ Comprehension assessment recorded
□ Capacity evaluation on file
□ Cooling-off period confirmed
□ Witness attestation (if required)
□ Ethics committee approval (if required)

SIGNATURES:
Subject: _______________ Date: _______________
Witness: _______________ Date: _______________
Provider: _______________ Date: _______________
Ethics Officer: _______________ Date: _______________ (if required)
```

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA-AUG-012 Specification v1.0*
*© 2025 SmileStory Inc. / WIA*
*MIT License*


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
