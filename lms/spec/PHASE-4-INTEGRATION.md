# WIA-lms PHASE 4 — Integration Specification

**Standard:** WIA-lms
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable

This PHASE describes how WIA-lms integrates with
adjacent ecosystems (SIS systems, LTI tools,
content publishers, credential issuers, identity
providers, and downstream WIA standards), how
conformance evidence is produced, and how
deployments are governed.

References (CITATION-POLICY ALLOW only):
- ISO/IEC 17065:2012 (Conformity assessment)
- ISO/IEC 27001:2022, ISO/IEC 27701:2019, ISO/IEC 23988
- IETF RFC 7515 (JWS), RFC 9421 (HTTP Message Signatures)
- IMS LTI 1.3 / Advantage, IMS Caliper 1.2, IMS QTI 3.0
- IMS Common Cartridge 1.3, IMS OneRoster 1.2
- IMS Open Badges 3.0, W3C VC 2.0, W3C DID 1.0
- IEEE 1484.12.1 LOM, IEEE 1484.20.1 xAPI
- ADL SCORM 2004 4th Edition (legacy interop)
- W3C WCAG 2.2, ISO 14289-1 PDF/UA-1

---

## §1 Scope

This PHASE covers integration points outside the
PHASE-1..3 scope: how WIA-lms consumes upstream
specifications (LTI / Caliper / OneRoster / QTI /
LOM), how downstream WIA standards reference LMS
artefacts, and how conformance is assessed,
recorded, and audited.

## §2 Upstream integration

### 2.1 IMS Global / 1EdTech

LTI 1.3 / Advantage, Common Cartridge 1.3, QTI 3.0,
Caliper 1.2, OneRoster 1.2 are consumed at their
published versions. Errata are absorbed editorially.

### 2.2 IEEE LTSC

LOM (1484.12.1), xAPI (1484.20.1), and CMI
(1484.11.2) are consumed at their latest stable
revisions.

### 2.3 ADL

SCORM 2004 4th Edition packages are accepted as
legacy content; cmi5 bridges legacy AICC into the
xAPI ecosystem.

### 2.4 W3C

VC 2.0, DID 1.0, BitstringStatusList, WCAG 2.2 are
consumed at the latest Recommendation.

## §3 SIS integration

Student information systems integrate via
OneRoster 1.2 (REST or CSV) for nightly sync. The
LMS reconciles OneRoster `users`, `classes`,
`enrollments`, and `academicSessions` into the
PHASE-1 records.

## §4 LTI tool integration

LTI 1.3 tools register via the tool record (PHASE-1
§10). Deep Linking 2.0, NRPS 2.0, AGS 2.0, and
Proctoring Services 1.0 are advertised in the
discovery document.

## §5 Conformance

### 5.1 Evidence package

Every conformant deployment publishes:

- declared LTI / Caliper / OneRoster / QTI / LOM
  versions in use;
- registered LTI tool catalogue with their
  Advantage service support;
- declared accessibility evidence (WCAG 2.2 AA);
- the test-vector matrix per Annex G of each
  PHASE;
- the JWS signing key set used to sign content
  packages and credentials.

### 5.2 Auditor responsibilities

Under ISO/IEC 17065:2012:

1. LTI 1.3 launches validate against the platform
   JWKS.
2. AGS scores post correctly to gradebook line
   items.
3. NRPS membership reflects OneRoster sync.
4. Credentials verify against issuer DIDs and
   revocation lists.
5. Accessibility evidence covers all content
   categories.

### 5.3 Tier promotion

| Tier            | Auditor                              | Cadence      |
|-----------------|--------------------------------------|--------------|
| Self-declared   | none                                 | annual       |
| Verified        | independent third-party              | 24 months    |
| Anchored        | accredited body (ISO/IEC 17065)      | 12 months    |

## §6 Cross-domain references (normative)

| Standard                 | Integration point                       |
|--------------------------|-----------------------------------------|
| WIA-language-learning    | LTI 1.3 hand-off                        |
| WIA-learning-analytics   | Caliper / xAPI source                   |
| WIA-prompts              | LLM tutoring activity                   |
| WIA-multiverse-interface | XR learning activity                    |
| WIA-language-bridge      | classroom interpretation                |

## §7 Privacy

Personal data flows under the institution's privacy
regime (FERPA, GDPR, K-PIPA, COPPA). Subject access
requests follow GDPR Article 15 / FERPA / K-PIPA
Article 35.

## §8 Security

Vulnerability disclosure follows ISO/IEC 30111.
LTI tool credential compromise triggers immediate
rotation and a tool-record amendment with
`keyRevocation` populated.

## §9 Identity providers

The LMS federates identity via OIDC, SAML 2.0, or
sovereign-equivalent identity protocols. SAML
attribute mapping to OneRoster roles is documented
in the deployment's identity-provider configuration.

## §10 Localisation

Course catalogues are localised in BCP 47 form. LMS
UI is localised at runtime per the learner's
preferred locale.

## §11 Accessibility

WCAG 2.2 AA conformance is mandatory for the
learner journey. Course-author tooling SHOULD
present accessibility hints (alt text, heading
structure, colour contrast) at authoring time.

## §12 Open Badges and Verifiable Credentials

Achievements are issued as Open Badges 3.0 (which
is itself a W3C VC 2.0 profile). Sovereign
authorities may issue parallel VC 2.0 credentials
for high-stakes outcomes.

## Annex A — Conformance disclosure

Implementations link to the conformance evidence
URL from their landing page.

## Annex B — Worked LTI tool record (informative)

```json
{
  "toolRef": "https://lms.example.org/v1/tools/calc",
  "clientId": "calc-tool-1",
  "loginInitiationUrl": "https://calc.example.org/lti/login",
  "targetLinkUri": "https://calc.example.org/lti/launch",
  "keySetUrl": "https://calc.example.org/.well-known/jwks.json",
  "services": ["deep-linking", "nrps", "ags"]
}
```

## Annex C — Versioning

Field additions are minor; field removals or
semantic redefinition require a major bump
synchronised with the corresponding upstream
specification major release.

## Annex D — Open governance

Issues and proposals at
`github.com/WIA-Official/wia-standards/issues` with
the `lms` label.

## Annex E — Withdrawal procedure

Tombstone the evidence package; tombstones are
immutable.

## Annex F — Reproducibility

Evidence is reproducible from publicly available
inputs: platform JWKS, registered tool catalogue,
OneRoster sync logs, Caliper sensor identities,
and content-package digests.

## Annex G — Test vectors

Every normative requirement in PHASE-1..3 has at
least one positive vector and one negative vector
under `tests/phase-vectors/`.

## Annex H — Sustainability

Deployments SHOULD declare their estimated energy
cost per active learner-month using the methodology
of ISO 14064 or the Green Software Foundation SCI
specification.

## Annex I — Risk register

| Risk                                  | Mitigation                |
|---------------------------------------|---------------------------|
| LTI tool key compromise               | Immediate rotation         |
| OneRoster sync drift                  | Nightly reconciliation     |
| Gradebook back-fill anomaly           | Audit-feed surfacing       |
| Caliper firehose backpressure         | HTTP/2 stream throttling   |
| Credential replay                     | Status list + `iat`/`exp`  |
| Content accessibility regression      | WCAG evidence required     |

## Annex J — Sovereign-scale deployments

Ministry-scale LMS deployments operate as
federations of authority-internal LMSs. Federation
peers are catalogued at
`/v1/registry/federation` with their accepted
operation groups and trust anchor JWKS URLs.

## Annex K — Vendor migration kit

Vendors migrating between platforms package the
deployment's records as a tarball: courses,
enrolments, gradebooks, credentials, and content
packages. The kit is signed by the source vendor;
the destination vendor verifies the signature
before importing.

## Annex L — Continuous improvement programme

Each deployment publishes an annual improvement
plan addressing accessibility regressions, LTI
tool maintenance, and SCORM-to-cmi5 migration
progress.

## Annex M — CASE competency frameworks

Mastery-based programmes consume CASE 1.0 frameworks
for competency definitions. Frameworks are mirrored
into the LMS at the institution's chosen cadence;
each framework version is recorded with the source
authority signature for audit reproducibility.

## Annex N — SIS reconciliation diagnostics

The LMS publishes a nightly OneRoster reconciliation
report listing drift between SIS-of-record and the
LMS-internal copy: missing enrolments, stale
enrolments, role mismatches, and unknown courses.
Reports surface in the audit feed under
`oneroster.reconciliation` events.

## Annex O — Cohort-based licensing

When licenses for content packages are scoped to
cohorts (e.g. honours-section access only),
licensing checks reference the cohort SHACL shape.
Licensing failures emit `license.cohort-mismatch`
events and block the launch with a learner-facing
notice.

## Annex P — Sovereign-scale pseudonymisation

Ministry deployments operate at scales where
pseudonymisation across institutions is necessary.
The LMS supports a federated pseudonymisation
service that maps institution-internal identifiers
to ministry-scoped pseudonyms without exposing the
mapping to either side.

## Annex Q — Course retirement and archival

Retired courses transition to an archive tier with
read-only access. Archive transitions emit
`course.archived` events and reduce the deployment's
active-course quota footprint. Archive content is
preserved for the institution's stated retention
window (typically 7 years for HE, varies for K-12).

## Annex R — Cross-institution course sharing

Sister institutions may share a course as a single
canonical instance with parallel rosters. Sharing is
governed by a bilateral data-sharing agreement
catalogued at `/v1/registry/agreements` and gated
by the institutions' identity providers.

## Annex S — Vendor extensibility manifest

Vendors expose extension points via a manifest
declaring custom fields, custom Caliper events, and
custom LTI launch parameters. The manifest is
namespaced under the vendor's domain so that
analytics consumers can reason about vendor-specific
extensions without colliding with WIA-standardised
records.

## Annex T — Disaster recovery

Deployments declare an RPO (Recovery Point
Objective) and an RTO (Recovery Time Objective) in
their evidence package. Default targets: RPO ≤ 24h
for HE, ≤ 1h for high-stakes assessment windows;
RTO ≤ 8h. DR drills are run annually with the
results catalogued in the audit feed.

## Annex U — Open-source reference implementation

A reference WIA-lms implementation is published at
the WIA Standards GitHub umbrella under the
`wia-lms-reference` repository. The implementation
covers the full PHASE contract and is licensed
Apache-2.0.

## Annex V — Vendor certification programme

Vendors apply to the certification programme by
submitting their evidence package and undergoing
the auditor review at their declared tier. Certified
vendors are catalogued at
`/v1/registry/certified-vendors` with their tier,
audit window, and contact information.

## Annex W — Annual ecosystem report

The registry publishes an annual ecosystem report
summarising LTI tool adoption, OneRoster sync
quality, content-package accessibility coverage,
credential issuance volume, and the SCORM-to-cmi5
migration progress. The report is informative.

## Annex X — Industry binding catalogue

| Industry segment       | Bound standard                       |
|------------------------|--------------------------------------|
| K-12 (US)              | OneRoster CSV, NIEM Education        |
| K-12 (KR)              | NEIS interop, MoE national assessmt. |
| HE                     | OneRoster REST, IMS Caliper          |
| Corporate L&D          | xAPI cmi5, SCORM 2004 4ed            |
| Government             | sovereign LMS profile                |
| Vocational             | competency-based mastery transcript  |

## Annex Y — Vendor neutrality

WIA-lms does not endorse a particular LMS vendor.
The conformance programme is open to commercial,
open-source, and institutional implementations on
identical terms. Reference materials are licensed
under permissive open-source licenses (Apache-2.0
or equivalent) to ensure that vendor lock-in is
minimised and migration between platforms is
feasible.

弘益人間 (Hongik Ingan) — Benefit All Humanity
