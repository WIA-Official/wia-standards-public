# WIA-medical-data-privacy PHASE 1 — Data Format Specification

**Standard:** WIA-medical-data-privacy
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable

This document defines the canonical data format for medical data privacy
artefacts: consent receipts, purpose-of-use declarations, minimum-necessary
filters, de-identification job records, breach notifications, and the
audit trail that binds them together. The shape is interoperable with
HL7 FHIR R5 Consent and AuditEvent resources so that an implementation
already publishing FHIR can adopt this PHASE without inventing parallel
data models.

References (CITATION-POLICY ALLOW only):
- HL7 FHIR R5 — Consent (R5/consent.html), AuditEvent (R5/auditevent.html), Provenance (R5/provenance.html)
- ISO/IEC 27001:2022 — information security management
- ISO/IEC 27018:2019 — PII protection in public cloud (processor)
- ISO/IEC 29100:2011 — privacy framework (controller/processor/data subject roles)
- ISO/IEC 29184:2020 — online privacy notices and consent
- ISO/IEC 27701:2019 — privacy information management (extends 27001)
- IETF RFC 8259 (JSON), RFC 9457 (Problem Details), RFC 7515 (JWS), RFC 7519 (JWT)
- W3C XML Signature 1.1 (for legacy CDA / interoperability with prior-era data)
- Kantara Initiative Consent Receipt Specification 1.1 (prior-art mapping only)

---

## §1 Scope

This PHASE applies to systems that handle Protected Health Information
(PHI) or its national equivalents (KR 민감정보 / 건강정보 under PIPA;
EU "data concerning health" under GDPR Art. 9; US PHI under 45 CFR §160.103).
It standardises the *shape* of privacy-relevant artefacts. Storage,
key management, and transport are addressed in PHASE 3 (Protocol)
and PHASE 4 (Integration).

The standard is jurisdiction-aware: a single implementation MUST be
able to declare which legal regime its records fall under (one of
HIPAA / GDPR / K-PIPA / LGPD / PIPL / PDPA-SG / NDB-AU) so that
downstream auditors can select the correct interpretation of
"minimum necessary", "purpose limitation", and "retention". A record
without a declared regime MUST be rejected at the edge.

## §2 Subject identifier model

PHI records carry a *pseudonymous* subject identifier (`subjectRef`).
Direct identifiers (name, contact, government ID) are stored separately
under a key that the privacy boundary controls. The cross-reference
between `subjectRef` and direct identifier is the *re-identification
key* and is itself in scope as PHI.

`subjectRef` is a URN of form `urn:wia:mdp:subject:<opaque>` where the
opaque part is at least 128 bits of entropy. Implementations MUST NOT
encode personal data into the opaque segment (no birth-year prefix,
no initials, no MRN). The mapping between MRN/EHR identifier and
`subjectRef` is held in a separate domain (the *identity broker*) and
is rotated per PHASE 3 §6.

Subject identifiers are *per-purpose* by default: a research dataset
and a clinical-care dataset MUST use different `subjectRef` values
for the same individual unless an active linkage authorisation
(see §5) is in force. This forces re-identification attempts to
go through the identity broker, which logs every linkage as an
AuditEvent.

## §3 Consent receipt (FHIR-aligned)

Every disclosure of PHI is gated by a consent record. Consent receipts
follow HL7 FHIR R5 Consent profile with the following fixed bindings:

| FHIR field          | Binding                                                                |
|---------------------|------------------------------------------------------------------------|
| `status`            | active / inactive / draft / entered-in-error                           |
| `category`          | from FHIR ConsentCategoryCodes (RESEARCH, NORMAL, etc.)                |
| `subject`           | Reference to the pseudonymous patient (`subjectRef`)                   |
| `dateTime`          | RFC 3339 timestamp; MUST include offset                                |
| `grantee`           | the recipient organisation (Practitioner / Organization)               |
| `controller`        | the data controller declaring jurisdiction (per §1)                    |
| `provision.purpose` | from v3-PurposeOfUse value set (TREAT, HRESCH, HPAYMT, etc.)           |
| `provision.action`  | one or more of access, collect, disclose, correct, delete              |
| `provision.period`  | start MUST be present; end SHOULD be present (open-ended is auditable) |
| `policy.uri`        | URL pointing to the human-readable privacy notice (per ISO 29184)      |
| `verification`      | identity-verification evidence (idType, verifiedWith, verifiedBy)      |

The receipt is signed using JSON Web Signature (RFC 7515) with the
controller's signing key. The detached signature is stored alongside
the receipt to keep the FHIR JSON canonical for downstream FHIR-aware
consumers.

## §4 Purpose-of-use vocabulary

The `provision.purpose` field draws from a closed value set so that
"minimum necessary" enforcement is deterministic. The closed set is
the v3-PurposeOfUse from the HL7 v3 vocabulary:

- `TREAT` — direct patient care
- `HPAYMT` — payment for healthcare
- `HOPERAT` — healthcare operations (quality assurance, training, audit)
- `HRESCH` — research, with secondary scope (`CLINTRCH`, `BIORCH`, `POPHLTH`)
- `PUBHLTH` — public health surveillance and reporting
- `EDU` — education
- `ETREAT` — emergency treatment override (gated by §6)

Purposes outside this list are rejected. Mapping a domain-specific
purpose to one of these eight values is the integration team's
responsibility (PHASE 4).

## §5 Linkage authorisation

Re-identification or cross-purpose linkage requires a separate
*linkage authorisation* record, distinct from the disclosure consent.
Linkage authorisations name the two `subjectRef` values being
linked, the purpose of the linkage, the requesting party, the
broker that will mint the linked identifier, and the period for
which the link is valid. A linkage authorisation can authorise
*one* link only — it does not extend to subsequent links derived
from the linked identifier.

Every minted linkage emits a Provenance resource (FHIR R5 Provenance)
so that auditors can see, for any record downstream, which linkage
authorisation produced its `subjectRef`.

## §6 Emergency-treatment override (break-glass)

When a clinician asserts `ETREAT` purpose on a disclosure that
lacks active consent, the system accepts the disclosure but
records:

- the asserting clinician's authenticated identity
- the patient's `subjectRef`
- the data scope disclosed (FHIR resource types and IDs)
- the timestamp and elapsed time since assertion
- the reason text (mandatory free-text, ≥20 characters)

The override is *post-hoc auditable* — the clinician's organisation
MUST review every break-glass event within 72 hours. A break-glass
event without a recorded review at +72h escalates to the data
protection officer per the deployment policy.

## §7 De-identification job record

When PHI is transformed into a de-identified extract (for research,
secondary use, or external publication), the job emits a record
containing:

- input scope (FHIR resource types, date range, cohort selector)
- transformation pipeline (k-anonymity, l-diversity, differential
  privacy ε, date-shift offsets, generalisation hierarchies)
- residual re-identification risk band (very low / low / moderate /
  not assessed; numeric scoring is out of scope because no method
  is universally agreed)
- output dataset hash (SHA-256 of the manifest)
- expiry date (after which the dataset MUST be re-evaluated or
  destroyed)

The job record is signed using the same JWS mechanism as consent
receipts. Datasets without a current job record cannot be retained
beyond a 90-day grace window.

## §8 Breach notification record

A suspected or confirmed breach of PHI emits a structured
notification with the fields required by the controlling
jurisdiction:

| Jurisdiction | Required field set                                                       |
|--------------|--------------------------------------------------------------------------|
| HIPAA        | breach description, PHI types affected, individuals affected, date range |
| GDPR Art. 33 | nature, categories of data subjects, likely consequences, mitigation     |
| K-PIPA       | item types, count, cause, response, contact for affected subject         |

The record is created within 24 hours of detection. Notification to
the regulator follows the jurisdiction's clock (HIPAA ≤60 days; GDPR
≤72 hours; K-PIPA 사실 인지 후 지체 없이). The record carries
provenance back to the AuditEvent that first surfaced the suspicion.

## §9 AuditEvent envelope

Every read, write, disclosure, override, linkage, de-identification
job, and breach event emits a FHIR AuditEvent. AuditEvents are
append-only, hash-chained (each event references the prior event's
hash), and signed using the controller's signing key on a daily
rollup. The chain root is published to a transparency log if and
only if the deployment policy enables it; otherwise the chain is
held in a controller-internal append-only store (PHASE 3 §4).

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Field-level mapping table (informative)

The following mapping documents how concrete record fields used by
common EHR vendors map onto this PHASE's data model. This annex is
informative; the normative shape is defined in §2–§9 above.

| EHR field                         | PHASE 1 §       | Notes                                            |
|-----------------------------------|-----------------|--------------------------------------------------|
| MRN / patient ID                  | §2 (subjectRef) | held only at the identity broker; never on record|
| Encounter / visit number          | §3 (Consent)    | encounter is the resource subject of `provision` |
| Reason-for-visit text             | §6 (ETREAT)     | only when override is asserted; otherwise out of scope |
| Allergy list                      | §3 (Consent)    | minimum-necessary scope under TREAT              |
| Insurance member ID               | §3 (HPAYMT)     | scoped to billing purpose                        |
| Research-cohort flag              | §5 (Linkage)    | provenance back to linkage authorisation         |
| De-identified extract identifier  | §7 (Job record) | extracted dataset hash, not subject identity     |

The mapping is not exhaustive; vendor field names differ. The
deployment SHOULD maintain a mapping document under change control
so that auditors can reproduce the field-to-§ binding without
reading vendor-specific schema.

## Annex B — Jurisdiction-specific extensions (informative)

Some jurisdictions require fields beyond the minimum set defined here.
Deployments add fields under a controller-scoped extension namespace
(`urn:wia:mdp:ext:<controller>:<field>`) so that the core record
remains portable across jurisdictions.

- KR PIPA — 본인확인 결과(CI) is held at the identity broker, never
  on disclosure-side records; CI rotation policy follows
  §3.6 (identity rotation).
- EU GDPR — special-category-data flag (`Art9Basis`) is required
  on every record where the FHIR resource carries health data;
  permitted bases are enumerated in deployment policy.
- US HIPAA — the 18 HIPAA Safe Harbor identifiers are removed
  from de-identified extracts per PHASE 1 §7; the residual
  re-identification risk band reflects what remains after the
  Safe Harbor pass.

## Annex C — Worked example: research consent receipt (informative)

The following is a fully populated FHIR R5 Consent resource that
satisfies this PHASE for a research disclosure. The example is
illustrative; production deployments substitute their own URNs,
controller IDs, and signing keys.

```json
{
  "resourceType": "Consent",
  "status": "active",
  "category": [{"coding": [{"system": "http://terminology.hl7.org/CodeSystem/consentcategorycodes", "code": "RESEARCH"}]}],
  "subject": {"reference": "urn:wia:mdp:subject:f4c2-9bd1-7a05-3e8e"},
  "dateTime": "2026-04-27T09:30:00+09:00",
  "grantee": [{"reference": "Organization/research-consortium-A"}],
  "controller": [{"reference": "Organization/hospital-K-controller"}],
  "policy": [{"uri": "https://hospital-k.example/privacy/research-2026"}],
  "provision": {
    "type": "permit",
    "period": {"start": "2026-04-27", "end": "2029-04-27"},
    "purpose": [{"system": "http://terminology.hl7.org/CodeSystem/v3-PurposeOfUse", "code": "HRESCH"}],
    "action": [{"coding": [{"code": "access"}, {"code": "disclose"}]}],
    "data": [{"meaning": "related", "reference": {"reference": "Group/research-cohort-2026Q2"}}]
  },
  "verification": [{"verified": true, "verifiedWith": "Practitioner/research-coordinator-12", "verificationDate": "2026-04-27T09:25:00+09:00"}],
  "extension": [
    {"url": "urn:wia:mdp:ext:jurisdiction", "valueCode": "K-PIPA"},
    {"url": "urn:wia:mdp:ext:dataMinimisationPlan", "valueUri": "Document/dmp-2026Q2-v3"}
  ]
}
```

The detached JWS signature is held alongside this resource at the
controller's signing endpoint and is verifiable against the JWKS at
`https://hospital-k.example/.well-known/jwks.json` (kid in the JWS
header matches a current `use: sig` key in the JWKS).

## Annex D — Conformance disclosure

Implementations declare per-section conformance in their published
Capability Statement (PHASE 2 Annex A). Sections marked `partial`
reference the deployment policy explaining the gap. Sections marked
`excluded` carry a reason citing the controlling jurisdiction's
allowance for that exclusion. An implementation that is `partial`
or `excluded` on §3 (Consent receipt), §4 (Purpose-of-use), §6
(Break-glass), or §9 (AuditEvent envelope) is non-conformant overall
and cannot claim Deep v3 status.
