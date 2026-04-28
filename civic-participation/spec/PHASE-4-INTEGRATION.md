# WIA-civic-participation PHASE 4 — INTEGRATION Specification

**Standard:** WIA-civic-participation
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how a public-administration
body operating a citizen-engagement platform
integrates with the systems that surround
deliberative democracy: the EU Single Digital
Gateway operating the cross-border citizen
service one-stop shop; the Interoperable Europe
Portal under EU Regulation 2024/903; the smart-
city KPI dashboard ingesting ITU-T Y.4900 and
ISO 37120/37122 indicators; the national
identity provider issuing the citizen's W3C
Verifiable Credential; the open-data portal
under W3C DCAT v3; the parliament secretariat's
written-contribution intake; the OECD Open
Government Partnership clearinghouse; the UN
DESA biennial e-government survey; the
transparency observer running the audit-of-
record; the supervisory data-protection
authority overseeing GDPR Article 9 processing;
and the public-procurement authority running an
inclusive-engagement programme.

References (CITATION-POLICY ALLOW only):

- ITU-T Y.4900/L.1600
- ISO 37120:2018, ISO 37122:2019, ISO 37123:2019,
  ISO 37100:2016, ISO 37101:2016
- ISO 18091:2019, ISO 9001:2015, ISO 37001:2016
- W3C ODRL 2.2, W3C DCAT v3, W3C DID v1.0, W3C
  VC Data Model v2.0
- OECD Recommendation of the Council on Open
  Government (OECD/LEGAL/0438), OECD Citizen
  Engagement Guide
- UN DESA E-Government Survey
- World Bank GovTech Maturity Index
- ISO/IEC 27001:2022, ISO/IEC 17021-1:2015,
  ISO/IEC 17065:2012
- IETF RFC 8259, RFC 9457, RFC 8615, RFC 9421,
  RFC 6962
- W3C Verifiable Credentials Data Model v2.0
- EU Single Digital Gateway Regulation (EU)
  2018/1724
- EU Interoperable Europe Act Regulation (EU)
  2024/903
- EU Regulation (EU) 2018/1724 Implementing
  Regulation (EU) 2022/1463 (technical
  specifications)
- EU GDPR Articles 6, 9, 12-22, 24-30, 32-34
- KR 행정기본법, KR 정보공개법, KR 전자정부법
  (e-Government Act)

---

## §1 EU Single Digital Gateway Integration

### §1.1 SDG operator registry binding

A consultation in scope of the EU SDG
Regulation (EU) 2018/1724 binds the operator's
SDG identifier to the consultation record. The
SDG implementing-regulation (EU) 2022/1463
technical specifications govern the cross-
border-citizen-service interaction.

### §1.2 Multi-language summary publication

The operator publishes a per-consultation
summary in each EU official language declared
in the operator's SDG profile. The summary's
multi-language envelope is signed by the
operator's public-key set so that the SDG-side
aggregator can validate the summary independent
of the operator.

## §2 EU Interoperable Europe Portal Integration

EU Regulation 2024/903 (Interoperable Europe
Act) establishes the Interoperable Europe
Portal as the catalogue of cross-border
interoperable services. The operator's API
publishes its per-service interoperability
descriptor (the data shapes carried, the
authentication mechanism, the rights expression)
through the portal's intake endpoint so that a
downstream consumer can discover the operator's
service alongside other Member-State services.

## §3 National Identity Provider Integration

### §3.1 Verified-citizen submission

A submission carrying a verified-citizen
identity references a W3C Verifiable Credential
issued by the national identity provider (the
EU eIDAS-2 European Digital Identity Wallet
under EU Regulation (EU) 2024/1183, the KR 본인
확인기관 issuing 본인확인서비스, the US state-
level digital-driver-license trust framework).
The credential's issuer signature is verified
against the issuer's public-key set published
on the issuer's discovery endpoint.

### §3.2 Credential revocation check

The operator's API queries the issuer's
revocation list on each verified-citizen
submission and refuses a submission whose
underlying credential has been revoked. The
revocation status is cached for the TTL
declared in the issuer's response headers.

## §4 Smart-City Dashboard Integration

A smart-city dashboard consuming the operator's
ITU-T Y.4900 / ISO 37120 / ISO 37122 KPI bundle
binds the per-period indicator value to the
city-level dashboard. The KPI envelope is
streamed to the dashboard via the webhook
endpoint declared in PHASE-2 §12; the
dashboard's signature verifier validates the
operator's signing key against the public-key
set.

## §5 Open-Data Portal Integration

The operator publishes the consultation envelope
and the post-consultation aggregated submission
corpus as a W3C DCAT v3 dataset on the
operator's open-data portal (or the national
open-data portal where the operator
participates in a federated catalogue). The
DCAT envelope carries the dataset's title in
multiple languages, the rights expression
(W3C ODRL), the publication-date, and the
contact-point.

## §6 Parliament-Secretariat Integration

A consultation related to a parliamentary
proposal binds the consultation envelope to the
parliament secretariat's written-contribution
intake. The secretariat's API publishes the per-
proposal contribution summary that the operator
ingests and binds to the consultation's
deliberation record.

## §7 OECD Open Government Partnership

The OECD Open Government Partnership maintains
a clearinghouse of open-government commitments
and reviews. An operator participating in the
OGP National Action Plan binds the per-
consultation outcome to the relevant OGP
commitment so that the OGP review-of-record
can audit the operator's progress against the
commitment.

## §8 UN DESA Biennial Survey Integration

The UN DESA E-Government Survey is conducted
biennially and publishes the per-Member-State
E-Government Development Index and the E-
Participation Index. The operator's API
publishes the per-period KPI bundle in the
form expected by the UN DESA secretariat so
that the operator's contribution to the
survey is preserved.

## §9 Transparency-Observer Integration

A transparency observer (a civil-society
watchdog, an academic researcher running a
democratic-audit, a journalist running a public-
interest investigation) integrates with the
operator's API through the audit-scope
authorisation declared in the operator's open-
data publication policy. The observer's audit
findings are published on the observer's
endpoint and may be referenced by the
operator's nonconformity register under PHASE-3
§11.

## §10 Supervisory Data-Protection Authority Integration

A supervisory data-protection authority
overseeing the operator's processing under
GDPR Article 9 audits the operator's processing
records (Article 30) on demand. The operator's
API publishes the GDPR Article 30 records of
processing activities to the authority's
endpoint so that the authority's audit trail
is preserved.

## §11 KR-Jurisdiction Integration

### §11.1 KR 전자정부법 binding

A KR-jurisdiction operator declares the KR
전자정부법 (e-Government Act) and the KR
국가정보화 기본법 (Framework Act on National
Informatization) reference in the programme
record's `governingFrameworks`. The KR Ministry
of the Interior and Safety operates the e-
government service register; the operator's
API queries the register on each retrieval
after the caching TTL.

### §11.2 KR 국민신문고 / 국민제안 integration

The KR 국민신문고 / 국민제안 platform operated
by the Anti-Corruption and Civil Rights
Commission ingests cross-jurisdictional citizen
petitions. A KR municipal operator publishing
a local petition binds the local petition to
the 국민신문고 cross-jurisdictional petition
where the petitioner requests the cross-
jurisdictional escalation.

### §11.3 KR 정보공개법 disclosure register

A KR-jurisdiction operator handling an
information-disclosure request publishes the
per-request decision in the KR 정보공개포털
operated by the Ministry of the Interior and
Safety so that the request's decision is
discoverable.

## §12 Public Retrieval and Re-Issuance

### §12.1 Public consultation retrieval

A public consumer (a citizen reading the
consultation, a journalist reporting on the
consultation, a researcher referencing the
consultation in a publication) retrieves the
consultation record at
`/v1/consultations/{consultationId}` without
authentication; the response carries the
public fields and the underlying outcome
summary.

### §12.2 Verifiable-credentials re-issuance

A consultation outcome (the per-citizen
acknowledgement, the per-citizen response with
rationale, the per-deliberation decision-
record) is re-issuable as a W3C Verifiable
Credential signed by the operator's public-
key set so that a downstream consumer (a
follow-up consultation, an external register,
an academic researcher) can validate the
outcome without contacting the operator
directly.

## §13 Audit and Conformity-Assessment Integration

### §13.1 ISO/IEC 17021-1 management-system audit

The operator's quality-management system
declared in PHASE-3 §10 is audited under
ISO/IEC 17021-1 by an accredited certification
body. The audit result is stored in the
operator's audit envelope and is referenced
from the programme record's
`engagementMaturity`. The certification body's
public-key set is published at the certification
body's endpoint; the operator's API verifies
the audit certificate's signature against the
certification body's public key on retrieval.

### §13.2 ISO/IEC 17065 conformity body
       discipline

Where the operator's certification (for
example the OECD OURdata Index sub-component
attestation, the World Bank GovTech Maturity
Index attestation) is issued by an external
conformity-assessment body, the body's
ISO/IEC 17065:2012 accreditation is bound to
the certification reference declared in the
programme record.

## §14 References (consolidated)

The references list across PHASE-1 to PHASE-4
is the canonical citation set for the WIA-
civic-participation standard. Implementations
cite the standards by their issuing
organisation (ITU-T, ISO, IEC, IETF, W3C, OECD,
UN, World Bank, EU regulatory text, KR
regulatory text) and the publication year so
that a downstream consumer can locate the
authoritative text. Updates to a cited standard
(for example, a new edition of ISO 37122)
trigger an internal review cycle in the
operator's quality-management discipline
declared in PHASE-3 §10 before the new revision
is bound into the operator's enumeration set.
