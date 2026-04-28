# WIA-language-learning PHASE 4 — Integration Specification

**Standard:** WIA-language-learning
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable

This PHASE describes how WIA-language-learning
integrates with adjacent ecosystems (sovereign
testing authorities, learning management systems,
employers, immigration authorities, accommodation
authorities, and downstream WIA standards), how
conformance evidence is produced, and how deployments
are governed.

References (CITATION-POLICY ALLOW only):
- ISO/IEC 17065:2012 (Conformity assessment)
- ISO/IEC 27001:2022, ISO/IEC 27701:2019
- ISO/IEC 23988:2007 (Assessment delivery), ISO 11669
- IETF RFC 7515 (JWS), RFC 9421
- IEEE 1484.12.1 (LOM), IEEE 1484.20.1 (xAPI)
- IMS Caliper 1.2, IMS LTI 1.3 / Advantage, IMS QTI 3.0
- IMS OneRoster 1.2, IMS Common Cartridge 1.3, IMS Open Badges 3.0
- W3C Verifiable Credentials Data Model 2.0
- W3C Decentralized Identifiers 1.0
- Council of Europe CEFR Companion Volume

---

## §1 Scope

This PHASE covers integration points outside the
PHASE-1..3 scope: how WIA-language-learning consumes
upstream framework specifications (CEFR, ACTFL,
ALTE, ICAO Doc 9835), how downstream WIA standards
reference language-learning artefacts, and how
conformance is assessed, recorded, and audited.

## §2 Upstream integration

### 2.1 Council of Europe / ACTFL / ALTE / ICAO

The framework versions in use are recorded in every
score record. Companion Volume revisions are
absorbed editorially; framework alignment tables
(PHASE-1 Annex B) are advisory and do not override
the framework's authoritative scoring guide.

### 2.2 IMS Global / 1EdTech

IMS LTI 1.3 / Advantage, OneRoster 1.2, QTI 3.0,
Caliper 1.2, and Common Cartridge 1.3 are consumed
at the latest stable release.

### 2.3 IEEE LTSC

IEEE 1484.12.1 LOM and 1484.20.1 xAPI are consumed
at the latest stable revision; new metadata fields
are absorbed into the LOM record without a major
bump unless they redefine existing semantics.

### 2.4 W3C

W3C VC 2.0 and DID 1.0 are consumed at their latest
Recommendation status.

## §3 LMS integration

LMS deployments integrate as LTI 1.3 platforms or
tools. Course launches use Deep Linking 2.0; class
roster sync uses NRPS 2.0 and OneRoster 1.2; gradebook
write-back uses AGS 2.0.

## §4 Sovereign authority integration

Sovereign testing authorities (e.g. Cambridge
Assessment, ETS, IELTS Consortium, Goethe Institute,
JLPT, KLAT) integrate at the score and credential
endpoints with their own signing key set. Their
credentials carry `issuerRef` resolving to a DID
controlled by the authority. The WIA registry does
not re-sign sovereign credentials.

## §5 Conformance

### 5.1 Evidence package

Every conformant deployment publishes:

- declared framework set with version (CEFR
  Companion Volume edition; ACTFL guidelines year);
- the LOM/QTI versions in use;
- the xAPI / Caliper LRS contract;
- the rater pool with sovereign certifications;
- the test-vector matrix per Annex G of each PHASE;
- the JWS signing key set used for scores and
  credentials.

### 5.2 Auditor responsibilities

Under ISO/IEC 17065:2012:

1. Cut-score tables match the published reference at
   the date of issuance.
2. Inter-rater agreement (κ) meets the PHASE-1 §10
   Annex H targets for the declared stakes class.
3. Accommodations are honoured throughout the
   assessment record.
4. xAPI / Caliper streams reproduce the session-level
   timeline in the audit replay.
5. Credential signatures verify against the issuer
   DID's published key.

### 5.3 Tier promotion

| Tier            | Auditor                              | Cadence      |
|-----------------|--------------------------------------|--------------|
| Self-declared   | none                                 | annual       |
| Verified        | independent third-party              | 24 months    |
| Anchored        | accredited body (ISO/IEC 17065)      | 12 months    |

## §6 Cross-domain references (normative)

| Standard                 | Integration point                       |
|--------------------------|-----------------------------------------|
| WIA-learning-analytics   | xAPI / Caliper dashboards               |
| WIA-lms                  | LTI 1.3 hand-off                        |
| WIA-language-bridge      | classroom interpretation                |
| WIA-prompts              | LLM tutor prompt provenance             |
| WIA-multiverse-interface | XR / immersive language environments    |

## §7 Employer / immigration integration

Employers and immigration authorities verify
language-proficiency credentials via the W3C VC 2.0
verification flow. The verifier presents a
Verifiable Presentation; the holder (learner)
discloses the minimum claims required. Selective
disclosure is supported via BBS+ signatures where
the issuer supports it.

## §8 Privacy

Learner records are processed under the deployment's
privacy regime. Score and credential records are
held under the issuer's retention rules; revocation
list members are retained for at least the
credential's `validUntil` plus 7 years to support
historical verification.

## §9 Security

Sovereign authority key compromise (loss, theft,
post-quantum migration event) triggers immediate
JWKS rotation and a public revocation list update.

## §10 AI and automated assessment

Automated scoring engines for speaking and writing
declare their model identity (model card URL),
training-data scope, and fairness audit summary on
each score record (`aiAssistMethod`, `modelCardRef`).
EU AI Act high-risk classification (Annex III §3)
applies to high-stakes language-proficiency scoring;
deployments meeting this threshold publish a
conformity assessment under EU AI Act Article 43.

## §11 Localisation (of the platform itself)

Platform UI is localised in BCP 47 form. Framework
descriptions are localised to the framework's
authoritative translations.

## §12 Accessibility

Platforms conform to WCAG 2.2 AA for the candidate
journey. Computer-based assessments support
screen-reader interaction with QTI 3.0 items
properly labelled. Captioning for listening items
is provided in the listed accommodation.

## Annex A — Conformance disclosure

Implementations link to the conformance evidence URL
from their landing page and from the README under a
`## Conformance` heading.

## Annex B — Worked credential (informative)

```json
{
  "@context": ["https://www.w3.org/ns/credentials/v2"],
  "type": ["VerifiableCredential", "LanguageProficiencyCredential"],
  "issuer": "did:web:cefr.example.org",
  "validFrom": "2026-04-28T00:00:00Z",
  "credentialSubject": {
    "id": "did:key:z6Mk...",
    "framework": "CEFR",
    "level": "C1",
    "skill": "overall",
    "evidence": ["https://ll.example.org/scores/..."]
  }
}
```

## Annex C — Versioning

Field additions are minor; framework alignment table
revisions track upstream release cycles editorially.

## Annex D — Open governance

Issues and proposals are tracked at
`github.com/WIA-Official/wia-standards/issues` with
the `language-learning` label.

## Annex E — Withdrawal procedure

A deployment withdraws conformance by tombstoning its
evidence package. Tombstones are immutable.

## Annex F — Reproducibility

Evidence is reproducible from publicly available
inputs: framework version, LOM/QTI versions, xAPI /
Caliper schema, rater pool, and the score signing
key set.

## Annex G — Test vectors

Every normative requirement in PHASE-1..3 has at
least one positive vector and one negative vector
under `tests/phase-vectors/`.

## Annex H — Sovereign authority binding catalogue

| Region   | Authority                              | Framework  |
|----------|----------------------------------------|------------|
| EU       | Goethe-Institut, Instituto Cervantes   | CEFR       |
| US       | ACTFL, OPI                             | ACTFL      |
| UK       | Cambridge English, BC IELTS            | CEFR/ALTE  |
| KR       | KICE (TOPIK)                           | CEFR-aligned |
| JP       | JEES (JLPT)                            | JLPT-CEFR  |
| Aviation | ICAO ELP TRAINAIR                      | ICAO 1..6  |

## Annex I — Risk register

| Risk                                  | Mitigation                |
|---------------------------------------|---------------------------|
| Cut-score table drift                 | Retention + cite-by-URL   |
| Inter-rater κ below target            | Block issuance            |
| Accommodation not honoured            | Audit incident            |
| Automated scoring bias                | Fairness audit on record  |
| Credential replay                     | Status list + `iat`/`exp` |
| Issuer DID compromise                 | Immediate rotation        |

## Annex J — Sustainability

Online assessment deployments SHOULD declare their
estimated cold-start energy cost per candidate-hour
and publish the methodology. The declaration is
informative.

## Annex K — Multi-issuer cross-walks

When a learner holds credentials from multiple
sovereign authorities (e.g. CEFR via Goethe-Institut
and ACTFL via OPI), employers may request a cross-walk
that aligns the credentials. Cross-walks are
informative and produced by an authority listed in
the registry's `/v1/registry/crosswalks` directory.
The cross-walk itself is signed by its issuer and
holds an explicit non-substitutability disclaimer:
cross-walks do not replace the underlying credentials.

## Annex L — Sovereign accommodation authority registry

Accommodation authorities (e.g. UK JCQ, US ETS
Disability Services, Korean NIIED, Japan AAAD) are
catalogued at `/v1/registry/accommodations/authorities`
with their accepted evidence types, lead times, and
appeal procedures.

## Annex M — Conformity assessment under EU AI Act

Deployments classified as high-risk AI systems under
EU AI Act Annex III §3 publish a conformity assessment
declaring the model card, training-data scope, fairness
audit summary, and post-market monitoring plan. The
declaration URL is published at
`/v1/registry/eu-ai-act` for each deployment that
falls within EU territorial scope.

## Annex N — National qualification frameworks

National Qualification Frameworks (NQF) bind language
levels to wider qualification ladders. The registry
catalogues the bindings at `/v1/registry/nqf` so that
employers and education ministries can resolve a
language credential against an NQF level (e.g. EQF
Level 6 for CEFR C1 in some member states, KQF Level
5 for ICAO Level 4 aviation English).

## Annex O — Continuous professional development

Instructor records carry a CPD log: each entry
references a CPD provider, a CEFR Companion Volume
chapter or ACTFL guideline section, and a duration
in hours. The CPD log is signed by the issuing
provider and surfaces in the instructor's public
profile when the instructor authorises disclosure.

## Annex O2 — Course catalogue alignment

Course catalogues from member providers are
harmonised so that a learner discovering a course at
provider A can resolve the equivalent CEFR / ACTFL
band at provider B. Harmonisation is performed by an
alignment service catalogued at
`/v1/registry/alignment` with each alignment record
signed by both providers and the registry.

The alignment record is informative; the issuing
authority's score remains authoritative. Where
alignments diverge from authority cut-scores the
registry records the divergence in the audit feed.

## Annex P — Cross-border credential portability

A learner moving across jurisdictions presents their
Verifiable Credentials at a destination registry's
`/v1/credentials/verify` endpoint. The destination
registry resolves the issuer DID, verifies the proof,
and checks the issuer against its accepted-authority
catalogue. Credentials from authorities not on the
catalogue are surfaced as `informational` rather than
authoritative.

弘益人間 (Hongik Ingan) — Benefit All Humanity
