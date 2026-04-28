# WIA-language-bridge PHASE 4 — Integration Specification

**Standard:** WIA-language-bridge
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable

This PHASE describes how WIA-language-bridge integrates
with adjacent ecosystems (translation tools, content-
management systems, healthcare and legal systems,
conference platforms, sign-language relay services,
and downstream WIA standards), how conformance evidence
is produced, and how deployments are governed.

References (CITATION-POLICY ALLOW only):
- ISO/IEC 17065:2012 (Conformity assessment)
- ISO 17100:2015, ISO 18587:2017, ISO 18841:2018, ISO 13611:2014
- ISO/IEC 27001:2022, ISO/IEC 27701:2019
- IETF RFC 7515 (JWS), RFC 9421 (HTTP Message Signatures)
- ASTM F2575-14, ASTM F2089-15
- HL7 FHIR R5 Communication, CommunicationRequest
- W3C ITS 2.0, W3C TTML2, W3C WebVTT
- ICAO Doc 9835 (Language Proficiency Requirements)

---

## §1 Scope

This PHASE covers integration points outside the
PHASE-1..3 scope: how WIA-language-bridge participants
consume upstream specifications (BCP 47, ISO 17100,
ITS 2.0), how downstream WIA standards reference
language-bridge artifacts, and how conformance is
assessed, recorded, and audited.

## §2 Upstream integration

### 2.1 IETF / Unicode

BCP 47 subtag updates are tracked editorially. CLDR
releases are consumed at the latest release; the
release identifier is recorded in every signed segment.

### 2.2 ISO

Translation production processes follow ISO 17100;
post-editing follows ISO 18587; community interpreting
follows ISO 13611; conference interpreting follows
ISO 18841. Each process declares its applicable ISO
standard in the job record.

### 2.3 OASIS / W3C

XLIFF 2.1 is the canonical interchange format for
written translation jobs. ITS 2.0 data categories are
honoured at every record. WebVTT and TTML2 are accepted
captioning formats.

## §3 Authoring tool integration

WIA-language-bridge does not mandate a CAT tool. The
integration contract is XLIFF 2.1 round-trip plus
LSP/HTTPS submission of segment records. Reference
plugins are published for memoQ, Trados, Phrase, and
OmegaT under the WIA Standards GitHub umbrella.

## §4 CMS / DAM integration

Content management systems integrate via the segment
endpoint with `documentRef` pointing to the CMS asset.
Round-tripping preserves the CMS asset identifier so
that translated content is reinjected without manual
re-binding.

## §5 Conformance

### 5.1 Evidence package

Every conformant deployment publishes:

- declared ISO 17100 / 18587 / 13611 / 18841 process
  scope;
- the practitioner register with sovereign certificate
  references;
- the test-vector matrix per Annex G of each PHASE;
- the BCP 47 tag set the deployment supports;
- the JWS signing key set used to sign segments and
  TM/TBX documents.

### 5.2 Auditor responsibilities

Under ISO/IEC 17065:2012 the auditor verifies:

1. Practitioner certificates resolve at the issuing
   authority where authoritative APIs exist.
2. Rotation rules are enforced in stored session
   records.
3. TBX-mandated terms are reflected in target segments
   for medical, legal, and safety-critical domains.
4. Quality measurements are produced by reviewers
   distinct from the producing practitioner.
5. Audio retention windows match declared values.

### 5.3 Tier promotion

| Tier            | Auditor                              | Cadence      |
|-----------------|--------------------------------------|--------------|
| Self-declared   | none                                 | annual       |
| Verified        | independent third-party              | 24 months    |
| Anchored        | accredited body (ISO/IEC 17065)      | 12 months    |

## §6 Cross-domain references (normative)

| Standard                     | Integration point                      |
|------------------------------|----------------------------------------|
| WIA-js                       | ECMA-402 locale negotiation in clients |
| WIA-language-learning        | proficiency assessment results         |
| WIA-pubscript                | publication-time localisation          |
| WIA-prompts                  | LLM prompt translation provenance      |
| WIA-emergency-medical-data   | bedside interpretation hand-off        |

## §7 Healthcare integration (FHIR)

Healthcare deployments bind language-bridge jobs to
FHIR R5 `CommunicationRequest` with `medium` set to the
relevant interpretation modality and `language` set to
the patient's preferred BCP 47 tag. The completion of
the job is reflected by a `Communication` resource
referencing the segment or session record.

Patient and practitioner identity flow through the
HL7 FHIR `Patient` and `Practitioner` resources; PHI
boundaries follow the deployment's HIPAA / GDPR /
K-PIPA configuration.

## §8 Legal-system integration

Court interpretation deployments bind sessions to a
court case identifier (Akoma Ntoso `<num>`-style
references or the local court's case-number scheme).
Audio retention follows the court rules; deletion is
recorded with the case identifier and a SHA-512 of the
deleted audio for tamper-evident traceability.

## §9 Privacy

Personal data in segments (names, dates of birth,
addresses, medical information) is handled under the
deployment's privacy regime. Translation memory
publication MUST NOT include segments that carry PII
unless an explicit consent record references the TM
inclusion.

## §10 Security

The WIA registry operates a vulnerability-disclosure
programme aligned with ISO/IEC 30111. Practitioner
credential compromise (key theft, private key leak)
triggers an immediate JWS key revocation and a
practitioner-record amendment with `keyRevocation`
populated.

## §11 Localisation (of the bridge itself)

The bridge's own user-facing surfaces (consent forms,
quality reports, session prompts) are localised in BCP
47 form. Locale negotiation follows RFC 4647 lookup;
the canonical record of available locales is exposed at
`/v1/registry/locales`.

## §12 Accessibility

Sign-language relay sessions MUST conform to the WCAG
2.2 Level AA captioning expectations. Captions emitted
by simultaneous interpretation MUST be timed to within
500 ms of the floor speaker's media timeline.

## Annex A — Conformance disclosure

Implementations link to the conformance evidence URL
from their landing page and from the README under a
`## Conformance` heading.

## Annex B — Worked evidence package (informative)

```json
{
  "deployment": "lb.example.org",
  "iso17100": "audited 2026-04-01",
  "practitionerCount": 482,
  "supportedTags": 312,
  "testMatrix": "https://lb.example.org/tests/matrix-2026-04-28.json",
  "manifestSig": "eyJhbGciOiJFUzI1NiIs..."
}
```

## Annex C — Versioning

Field additions are minor; field removals or semantic
redefinition require a major bump synchronised with
the corresponding ISO 17100 process review cycle.

## Annex D — Open governance

Issues, errata, and proposals are tracked at
`github.com/WIA-Official/wia-standards/issues` with the
`language-bridge` label. The WIA Standards working
group reviews open issues at the start of every minor
release cycle.

## Annex E — Withdrawal procedure

A deployment withdraws conformance by submitting a
tombstone for its declared evidence package. The
registry retains the tombstone indefinitely so that
historical audits remain reproducible.

## Annex F — Reproducibility

Conformance evidence is reproducible from the publicly
available inputs: the registered practitioner records,
the BCP 47 tag set, the TM/TBX corpora, and the
deployment's runtime profile. Reproduction harnesses
are published at `/v1/conformance/repro`.

## Annex G — Test vectors

Every normative requirement in PHASE-1..3 has at least
one positive vector and one negative vector under
`tests/phase-vectors/`. Vector matrices are versioned
with Semantic Versioning 2.0.0 so auditors can pin a
specific release.

## Annex H — Conference platform integration

Major conference platforms (Zoom, Teams, Jitsi,
Whereby) integrate via the RSI signalling described in
PHASE-3 §5. Each platform publishes a binding profile
declaring its supported codecs and rotation event
delivery semantics. Profiles are catalogued at
`/v1/registry/platforms`.

## Annex I — Sustainability

Real-time interpretation deployments SHOULD declare
their estimated carbon intensity per session-hour,
using the methodology of ISO 14064 or the Green
Software Foundation Software Carbon Intensity (SCI)
specification. The declaration is informative.

## Annex J — Industry binding catalogue

| Industry segment          | Bound standard                         |
|---------------------------|----------------------------------------|
| Aviation                  | ICAO Doc 9835 English proficiency      |
| Maritime                  | IMO STCW table A-II/1 communication    |
| Healthcare                | HL7 FHIR R5 Communication              |
| Court / Tribunal          | Akoma Ntoso citation IDs               |
| Asylum / Refugee          | UNHCR interview interpretation guide   |
| Public broadcasting       | EBU Tech 3370 captioning workflow      |

## Annex K — Sovereign certificate registers

The following certificate registers are recognised at
the registry without further attestation:

- ATA (American Translators Association)
- NAATI (National Accreditation Authority for Translators and Interpreters, AU)
- CIOL DipTrans (Chartered Institute of Linguists, UK)
- KICE Korean Translation Examination
- JTF (Japan Translation Federation)
- AAEC Korean Court Interpreter Examination
- CCHI / NBCMI medical interpreter certifications

Other registers may be accepted under a deployment-
specific addendum.

## Annex L — Translation memory privacy review

Before a TM is published outside the originating
deployment, a privacy review redacts segments matching
PII patterns. The default review consults the
ICO/EDPB anonymisation guidelines and the K-PIPA
Anonymisation Decision Framework. Redacted segments
remain in the deployment-internal TM but are excluded
from federated indices.

## Annex M — Style-guide binding

Each domain may bind a style guide (Microsoft Style,
Apple Style, IBM Editorial Style, Korean National
Institute of the Korean Language, Plain English
Campaign). The bound style guide identifier is recorded
in the job record so that quality reviewers apply the
correct rules.

## Annex N — Termbase round-trip with public glossaries

Public terminology bases (UNTERM, IATE, FAO AGROVOC,
WHO MeSH, Codex Alimentarius) are mirrored as TBX
extracts in the registry. Mirror cadence is monthly
for slow-moving bases and weekly for IATE; the mirror
URI carries the upstream snapshot date and a SHA-512
content digest for tamper-evident traceability.

## Annex O — Risk register

| Risk                                  | Mitigation                |
|---------------------------------------|---------------------------|
| Unsigned segment accepted             | Reject; require JWS sig   |
| Rotation rule violation               | Conformance event         |
| TBX preferred term ignored            | Quality warning           |
| Audio retention exceeded              | Auto-delete on schedule   |
| Practitioner key compromise           | Immediate JWKS rotation   |
| Sign-language frame loss              | Codec fallback + alert    |
| Session replay                        | `iat`/`exp` ≤ 1h          |

弘益人間 (Hongik Ingan) — Benefit All Humanity
