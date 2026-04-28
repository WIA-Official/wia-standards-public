# WIA-micro-credential PHASE 4 — Integration Specification

**Standard:** WIA-micro-credential
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable

This PHASE specifies how WIA-micro-credential
integrates with adjacent identity, learning, employer,
and regulator systems: digital-identity wallets (EUDI
Wallet, Apple / Google wallet via OID4VCI bridges),
learning management systems, MOOC platforms, employer
HRIS recruitment pipelines, public-employment-service
skills-matching, EU EBSI, EU Europass, national NQF
authorities, ESCO / O*NET-SOC skills frameworks,
academic transcript exchange (ELMO XML, EDI TS 130 / 138),
proctoring services, and sponsor-internal data-warehouse
pipelines. It also specifies the operational binding
to companion WIA standards.

References (CITATION-POLICY ALLOW only):
- W3C VC Data Model 2.0; W3C DID 1.0; W3C Status List 2021
- 1EdTech Open Badges 3.0; 1EdTech CLR 2.0; 1EdTech Caliper Analytics
- Europass Credentials specification; European Learning Model (ELM)
- ESCO classification; O*NET-SOC; ISCO-08
- ENQA ESG 2015 — Standards and Guidelines for QA in EHEA
- Bologna Process / Lisbon Recognition Convention
- EU eIDAS Regulation (910/2014, EUDI Wallet Reg 2024)
- EU EBSI specification
- KR DTAB / KISA Digital Trust Framework
- ELMO (European Learner Mobility Object) XML
- HR-XML / HR Open Standards (recruitment data exchange)
- ISO 21001 — educational organisations management system
- IETF RFC 9110 (HTTP), RFC 7515 (JWS), RFC 8259 (JSON), RFC 8785 (JCS)
- ISO 8601, ISO 3166, BCP 47

---

## §1 Wallet and identity-framework integration

| Wallet / framework      | Binding                                       |
|-------------------------|-----------------------------------------------|
| EU EUDI Wallet          | per Reg 2024/... (eIDAS 2.0)                  |
| Apple / Google wallet   | OID4VCI bridge with platform-issued profile    |
| EBSI Verifiable         | EBSI accreditation chain                      |
| KR mobile ID / 모바일신분증 | KR DTAB profile                               |
| Sponsor / employer wallet| sponsor's DID method + trust list             |

The wallet integration's `holder` claim binds to the
recipient DID; per-wallet onboarding records the
wallet implementation reference and the controller-key
attestation.

## §2 LMS / MOOC platform integration

| Platform            | Integration profile                              |
|---------------------|--------------------------------------------------|
| LTI-compliant LMS   | IMS LTI 1.3 + LTI Advantage; Caliper Analytics   |
| Caliper consumers   | ingest learning events to CLR                    |
| MOOC platforms      | Open Badges 3.0 / CLR; xAPI fall-back            |
| Bespoke LMS         | OAuth-protected REST per OID4VCI / OID4VP        |

Learning events emitted by the LMS / MOOC platform
populate the evidence record (PHASE 1 §6); credential
classes catalogued in the platform map to PHASE 1 §4
classes via a lightweight publish/subscribe contract.

## §3 Employer HRIS integration

| HRIS pattern           | Integration                                    |
|------------------------|------------------------------------------------|
| HR-XML staffing        | HR Open Standards messages                      |
| Bespoke ATS API        | OID4VP request profile                         |
| O*NET / ESCO matching  | skill-binding query                            |
| Recruiter dashboard    | OID4VP request + selective disclosure           |

Recruiters request only the claims they need
(selective disclosure); recipients consent at
presentation-time so the recruiter never sees
unrelated credentials.

## §4 Public-employment-service / skills-matching

| Service                 | Binding                                         |
|-------------------------|-------------------------------------------------|
| EU EURES                | Europass + ELM credential exchange               |
| KR Worknet              | National NQF binding                            |
| Public skills-match API | ESCO / O*NET-SOC skill predicate match           |

Public services consume the framework-mapping records
(PHASE 1 §8) to suggest training, employment, or
recognition pathways.

## §5 EBSI / eIDAS 2.0 integration

EBSI / EUDI Wallet binding profile:

```
issuer-accreditation-on-EBSI →  trust-anchor-published →
  issuer-DID-resolved-on-chain →  status-list-fetched →
  proof-validated →  outcome-published-to-EBSI
```

The implementation may operate without EBSI (sponsor-
internal trust list) or with EBSI (broader European
verifier acceptance). EBSI-bound issuances cite the
EBSI accreditation event identifier.

## §6 Europass and EU learning frameworks

Europass Credentials format complements W3C VC:
issuers may dual-issue (W3C VC + Europass XML) so EU-
legacy verifiers and W3C-aware verifiers both work.
The cross-format binding records the same issuance in
both forms with a shared content hash.

| Format                | Operator                                      |
|-----------------------|-----------------------------------------------|
| Europass Credentials  | European Commission                            |
| ELM XML               | European Commission                            |
| W3C VC + Open Badges  | open-source / industry                         |

## §7 Proctoring and identity-verification integration

Proctored assessments integrate with proctoring
services per the issuer's evidence policy:

- live human proctoring (web-camera supervision)
- AI-assisted proctoring (anomaly detection)
- proctoring-centre (in-person)

Proctoring artefacts carry their own provenance hash;
evidence records reference the artefact hash so a
recognising party can request the artefact under
recipient consent.

## §8 Cross-domain WIA bindings

| Companion standard           | Binding purpose                               |
|------------------------------|-----------------------------------------------|
| WIA-digital-credential       | parallel-track digital-id binding              |
| WIA-learning-analytics       | outcome / evidence pipeline                   |
| WIA-virtual-classroom        | instructor / proctor identity                 |
| WIA-content-ai               | AI-generated evidence governance              |
| WIA-mooc                     | platform-specific learning events             |
| WIA-data-portability         | recipient export / migration                  |

Each binding identifies the consumed PHASE.

## §9 Long-term archival

| Authority / context           | Retention                                |
|-------------------------------|------------------------------------------|
| Issuer (regulated education)  | per national rules; typically ≥ 25 years |
| Issuer (vocational)           | per national rules; ≥ 10 years           |
| Recipient wallet              | recipient-controlled                      |
| Verifier audit                | per regulator; typically ≥ 5 years        |
| Status-list publication       | indefinite (per W3C SL recommendation)    |

Personal-data erasure tombstones the issuance payload
while preserving the audit-chain hash.

## §10 Conformance test suite

The reference test suite covers:

- OID4VCI pre-authorised flow round-trip
- OID4VP presentation_definition match with selective
  disclosure
- W3C Status List 2021 bit lookup with cache control
- EBSI trust-list resolution against an EBSI test
  registry
- Europass dual-issuance binding
- selective disclosure of a single claim from a
  multi-claim VC
- holder-binding proof preventing presentation replay
- revocation propagation across verifier cache
- EQF / ESCO mapping retrieval
- cross-wallet credential transfer

## §11 Internationalisation

Localised attributes (class names, achievement
narratives, evidence narratives, framework labels)
carry BCP 47 language tags. Country-specific NQF
binding is recorded per ISO 3166-1 alpha-3.

## §12 Security and privacy posture

- Transport: TLS 1.3 with mutual TLS for issuer ↔
  regulator transmissions
- Authentication: OID4VCI / OID4VP for credential flow;
  client_credentials with key attestation for sponsor
  back-end integrations
- At-rest: AES-256-GCM with sponsor-controlled KMS;
  per-issuer key wrapping per ISO/IEC 27002 §8.24
- Audit: tamper-evident chain (PHASE 3 §9) exportable
  per ISO/IEC 27037 forensic-evidence guidance
- Privacy: recipient holds credentials in a wallet
  under their control; selective disclosure minimises
  data shared with verifier; recipient rights honoured
  per PHASE 3 §10
- Key management: issuer signing keys rotate on a
  policy clock (typically annual); old keys remain
  resolvable via the DID Document key-history

## §13 Operational metrics

Sponsors / issuers report (informationally) on the
WIA registry:

- credential classes published
- issuances per class
- revocation rate (by reason)
- presentation-acceptance rate at recognising parties
- trust-list updates honoured
- accreditation status

## §14 Recovery and continuity

- API outage — wallet caches issuances offline;
  status-list refresh is best-effort
- DID-resolution outage — cached DID Documents permit
  short-window verification with degraded confidence
- status-list outage — verifiers fall back to last-
  known status (cache-control honoured) and surface a
  warning
- KMS outage — sealed back-up keys per sponsor's BCP

## Annex A — Worked end-to-end example (informative)

A national vocational authority accredits an issuer at
NQF level 4. The issuer publishes a "Solar PV
Installer" micro-credential class with a 60-hour
notional learning load and an EQF level 4 mapping. A
learner completes the course; assessor evidence is
captured per PHASE 3 §3; the issuer issues a credential
via OID4VCI. The learner stores the credential in an
EU EUDI Wallet. An employer in another EU member state
runs an OID4VP request asking only for credential
class, EQF level, and validity. The wallet constructs
a selectively-disclosed presentation; the employer's
verification policy accepts the credential. The
employer hires the learner; the learner's NQF level
4 binding satisfies the local regulator's installer
licensing prerequisite.

## Annex B — Conformance disclosure

Implementations declare the wallet integrations
supported, the OID4VCI / OID4VP profiles, the W3C VC
formats, the trust frameworks consumed, the Europass /
ELM binding, and the framework-mapping registries
indexed. Disclosure is machine-readable at
`/.well-known/wia-mc-conformance.json`.

## Annex C — Versioning

Adding a new wallet integration is minor; changing
the W3C VC binding format is major.

## Annex D — Recipient data-portability export

Recipient wallets request a portable archive:

```
GET /v1/recipients/{ref}/portability-export
```

Output is a signed bundle (W3C VC list + audit-chain
proof) suitable for import into another wallet. The
bundle's container format follows the OCI Distribution
content-addressing convention so any tooling that
understands content-addressed bundles can carry it.

## Annex E — Cross-jurisdiction recognition

Recognition across jurisdictions consumes the
applicable convention or directive:

| Convention / directive    | Scope                                  |
|---------------------------|----------------------------------------|
| Lisbon Recognition        | Council of Europe / UNESCO higher-      |
| Convention                 | education recognition                  |
| Bologna Process            | EHEA (European Higher Education Area)   |
| EU Directive 2005/36       | regulated-profession recognition       |
| UNESCO Global Convention   | global higher-education recognition    |
|                            | (in force 2023)                         |
| Mutual recognition         | bilateral / sector-specific            |
| arrangements               |                                         |

Recognition events bind to the convention identifier
so recognising parties resolve the applicable
provision automatically.

## Annex F — ELMO / Europass dual-issuance

Issuers operating in EU contexts may dual-issue:

```
W3C VC issuance ──────┐
                      ├── shared content hash
Europass / ELM XML ───┘
```

The dual-issuance event records both forms with a
shared `contentDigest` so a verifier consuming either
format can cross-verify against the other.
