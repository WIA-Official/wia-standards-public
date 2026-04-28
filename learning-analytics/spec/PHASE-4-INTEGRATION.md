# WIA-learning-analytics PHASE 4 — Integration Specification

**Standard:** WIA-learning-analytics
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable

This PHASE describes how WIA-learning-analytics
integrates with adjacent ecosystems (LMS platforms,
SIS systems, ministry data warehouses, AI fairness
tooling, accessibility evaluators, and downstream
WIA standards), how conformance evidence is
produced, and how deployments are governed.

References (CITATION-POLICY ALLOW only):
- ISO/IEC 17065:2012, ISO/IEC 27001:2022, ISO/IEC 27701:2019
- ISO/IEC 20748-1..-4 (Learning Analytics Interoperability)
- ISO/IEC 23053 (Framework for AI systems using ML), ISO/IEC 23894 (AI risk)
- ISO/IEC 42001 (AI management system)
- IEEE 1484.12.1 LOM, IEEE 1484.20.1 xAPI
- IMS Caliper 1.2, IMS LTI 1.3, IMS OneRoster 1.2, IMS QTI 3.0
- IMS Open Badges 3.0, W3C VC Data Model 2.0
- W3C WCAG 2.2 (Web Content Accessibility Guidelines)
- EU AI Act, EU GDPR, US FERPA, US PPRA, K-PIPA, COPPA

---

## §1 Scope

This PHASE covers integration points outside the
PHASE-1..3 scope: how WIA-learning-analytics consumes
upstream specifications (xAPI, Caliper, LOM, LTI),
how downstream WIA standards reference learning-
analytics records, and how conformance is assessed,
recorded, and audited.

## §2 Upstream integration

### 2.1 IEEE LTSC

xAPI 1.0.3 and LOM are consumed at the latest stable
release. Profile updates are absorbed editorially.

### 2.2 IMS Global / 1EdTech

Caliper 1.2, LTI 1.3 / Advantage, OneRoster 1.2 are
consumed at their published versions. Caliper Profile
revisions trigger a profile-record amendment in the
registry.

### 2.3 ISO

ISO/IEC 20748 is the architectural reference. ISO/IEC
23053 / 23894 / 42001 frame the ML and AI management
expectations applied to model records (PHASE-1 §7).

### 2.4 W3C

W3C VC 2.0, DID 1.0, SHACL, WCAG 2.2 are consumed at
their latest Recommendation status.

## §3 LMS / SIS integration

LMS deployments emit Caliper events at the platform
level. SIS systems contribute enrolment context via
OneRoster 1.2 nightly sync. The registry reconciles
both into a single actor / activity / context graph.

## §4 Ministry warehouse integration

Sovereign education ministries ingest aggregate
metrics through the registry's bulk export endpoint
(PHASE-2 Annex F). Aggregate exports honour
k-anonymity (default k=10) and l-diversity
constraints declared in the deployment's privacy
policy.

## §5 Conformance

### 5.1 Evidence package

Every conformant deployment publishes:

- declared xAPI / Caliper versions and supported
  profiles;
- declared model registry contents with model card
  URLs and fairness audits;
- declared dashboard registry with WCAG 2.2 AA
  conformance evidence;
- the test-vector matrix per Annex G of each PHASE;
- the JWS signing key set used for statements,
  profiles, models, and dashboards.

### 5.2 Auditor responsibilities

Under ISO/IEC 17065:2012:

1. Statements satisfy the templates and patterns of
   their declared profile.
2. Models pass the fairness audits referenced from
   their model card.
3. Interventions of `auto-block` type carry a
   human review reference where required.
4. Dashboards present the explainability surface
   declared in their record.
5. Consent withdrawal is honoured in subsequent
   statements and aggregate analyses.

### 5.3 Tier promotion

| Tier            | Auditor                              | Cadence      |
|-----------------|--------------------------------------|--------------|
| Self-declared   | none                                 | annual       |
| Verified        | independent third-party              | 24 months    |
| Anchored        | accredited body (ISO/IEC 17065)      | 12 months    |

## §6 Cross-domain references (normative)

| Standard                 | Integration point                       |
|--------------------------|-----------------------------------------|
| WIA-language-learning    | proficiency analytics                   |
| WIA-lms                  | Caliper / xAPI source at the platform   |
| WIA-prompts              | LLM-mediated tutoring analytics         |
| WIA-multiverse-interface | XR learning analytics                   |

## §7 Privacy

Personal data flows through the deployment's privacy
regime. Pseudonymised actor records carry
re-identification keys held under the regime's
authority. Subject access requests follow GDPR
Article 15 / FERPA / K-PIPA Article 35.

## §8 Security

The registry operates a vulnerability-disclosure
programme aligned with ISO/IEC 30111. Statement
signing key compromise triggers a JWKS rotation.

## §9 EU AI Act conformity

Deployments classified as high-risk under EU AI Act
Annex III §3 (education and vocational training)
publish a conformity assessment per Article 43. The
assessment is referenced from each model record's
`eulaAttestationRef` field. Post-market monitoring
results are appended to the model card on a
quarterly cadence.

## §10 Accessibility

Dashboards conform to WCAG 2.2 AA. Caliper sensors
emit accessibility events (e.g. `Screen Reader Used`,
`Caption Toggled`) for analytics consumers
inspecting accessibility-equivalent learning paths.

## §11 Localisation

Dashboard cards and intervention narratives are
localised in BCP 47 form. Profile concept labels
follow xAPI Profile localisation conventions.

## §12 Data subject rights

Learners exercise the following rights via the
deployment's privacy portal:

- right of access (PHASE-2 Annex L);
- right to opt out of analytics-driven interventions;
- right to challenge an automated decision (GDPR
  Article 22);
- right to be forgotten (subject to the deployment's
  legal obligations to retain certain records).

## Annex A — Conformance disclosure

Implementations link to the conformance evidence URL
from their landing page and from the README under a
`## Conformance` heading.

## Annex B — Worked dashboard policy (informative)

```json
{
  "dashboardRef": "https://la.example.org/dash/at-risk",
  "audience": "instructor",
  "dataScope": "class",
  "explainabilityRef": "https://la.example.org/dash/at-risk/explain.json",
  "optOutPath": "https://la.example.org/privacy/opt-out",
  "wcagEvidence": "https://la.example.org/wcag/at-risk-2026-04-28.json"
}
```

## Annex C — Versioning

Field additions are minor; semantic redefinition
requires a major bump synchronised with upstream
xAPI / Caliper / LOM major releases.

## Annex D — Open governance

Issues and proposals are tracked at
`github.com/WIA-Official/wia-standards/issues` with
the `learning-analytics` label.

## Annex E — Withdrawal procedure

A deployment withdraws conformance by tombstoning
its evidence package. Tombstones are immutable.

## Annex F — Reproducibility

Evidence is reproducible from publicly available
inputs: profile registry, model card URLs, xAPI /
Caliper schema, signing key set, and dashboard
WCAG evidence URLs.

## Annex G — Test vectors

Every normative requirement in PHASE-1..3 has at
least one positive vector and one negative vector
under `tests/phase-vectors/`. Vector matrices are
versioned with Semantic Versioning 2.0.0.

## Annex H — Sustainability

Predictive-model deployments SHOULD declare their
estimated training carbon intensity per parameter
update and inference latency cost per prediction,
using the methodology of ISO 14064 or the Green
Software Foundation SCI specification.

## Annex I — Risk register

| Risk                                  | Mitigation                |
|---------------------------------------|---------------------------|
| Unprofiled verb leakage               | Reject; quarantine        |
| Model fairness drift                  | Quarterly audit gating    |
| `auto-block` without human review     | Reject at intervention    |
| PII in `result.response`              | Truncate + redact         |
| Aggregate re-identification           | k-anon / l-diversity      |
| Statement replay                      | `iat`/`exp` ≤ 24h         |
| Dashboard accessibility regression    | WCAG evidence required    |

## Annex J — Sovereign-scale deployment

Ministry deployments operate as federations of
authority-internal LRSs. Cross-authority federation
is gated by bilateral data-sharing agreements that
the registry catalogues at
`/v1/registry/agreements`. Aggregate exports across
authorities apply k-anonymity at the union of the
sharing agreements' floors.

## Annex K — Model card schema

Model cards reference the model's intended use, the
training-data scope, fairness audit summary,
performance metrics, and post-market monitoring plan.
The schema is JSON-Schema-validated at registration
under `application/vnd.wia.model-card+json`. Cards
that fail validation are rejected before model
publication.

## Annex L — Continuous fairness audit

Models classified as high-risk produce monthly
fairness audit reports. Each report references the
model card, the post-deployment statement sample, and
the metrics evaluated (equal opportunity, demographic
parity, calibration). The reports are signed by the
auditor and published at the model record's
`fairnessAuditRef` URL.

## Annex M — Researcher access programme

Educational researchers may request access to
de-identified statement extracts via the registry's
researcher access programme. Each request is reviewed
by an institutional IRB / REC and a data-use
agreement that is catalogued in
`/v1/registry/researcher-access`. Researchers commit
to publishing replication code and aggregate results.

## Annex N — Educational technology certification

Educational-technology vendors that embed analytics
sensors carry a certification declaration referencing
their declared profiles, model cards (where
applicable), and dashboard accessibility evidence.
Certifications are renewed on the tier cadence
(annually for self-declared, every 24 months for
verified, every 12 months for anchored).

## Annex O — Course design feedback loop

Aggregate analytics feed back into course design via
LOM record annotations. The annotation surface is
informative; instructors retain authority over
curriculum changes derived from analytics signals.

## Annex P — Federated learning across institutions

Where direct data sharing is restricted, model
training MAY proceed via federated learning. Each
participating institution runs a local trainer; the
registry coordinates parameter aggregation under the
data-sharing agreement. Model updates are signed by
each institution; aggregated weights are signed by
the registry.

The federated-learning aggregator is itself audited
for parameter-leakage risk per ISO/IEC 23894.

## Annex Q — Privacy-preserving analytics primitives

Where the deployment's privacy regime constrains
direct access to micro-data, analytics may operate
via privacy-preserving primitives:

- differential privacy (with declared epsilon, delta);
- secure multi-party computation;
- trusted-execution-environment enclaves with
  attested code measurement.

The chosen primitive is declared in the deployment's
evidence package and verified by the auditor against
the published parameter set.

## Annex R — Continuous improvement programme

Each conformant deployment publishes an annual
improvement plan addressing fairness drift,
accessibility regressions, model deprecations, and
profile maintenance. The plan is reviewed at the
auditor's next visit and the deltas surface in the
audit feed.

## Annex S — Common-cartridge interop

When a course is delivered in a Common Cartridge 1.3
package, the analytics emitted from the cartridge
content are bound to the cartridge identifier so
that platform migrations preserve the analytics
trail.

## Annex T — Sovereign-scale aggregate dashboards

Ministry-level aggregate dashboards consume the
cross-authority federation feed. Aggregation rules
(rounding, suppression of small cells, top-coding)
are declared in the dashboard record and verified
against the deployment's privacy policy.

## Annex U — Annual ecosystem report

The registry publishes an annual ecosystem report
summarising profile uptake, model card coverage,
fairness audit completion rates, accessibility
evidence freshness, and data subject access volumes.
The report is informative.

弘益人間 (Hongik Ingan) — Benefit All Humanity
