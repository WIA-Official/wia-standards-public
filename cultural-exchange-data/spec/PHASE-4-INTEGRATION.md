# WIA-cultural-exchange-data PHASE 4 — INTEGRATION Specification

**Standard:** WIA-cultural-exchange-data
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how a memory-institution
operator integrates with the systems that
surround international cultural-data exchange:
the federated-aggregator portal (Europeana, the
Digital Public Library of America, the
Internet Archive, the operator's national
aggregator); the IIIF Consortium publishing the
IIIF specifications; the international
authority files (VIAF, LCNAF, ULAN, Wikidata);
the W3C Working Groups publishing the linked-
data specifications; the ICOM, IFLA, ICA
international organisations; the UNESCO
secretariat operating the 2005 / 1972 / 2003
Conventions; the supervisory cultural-heritage
authority overseeing the operator's
accreditation; the regulator overseeing the
operator's GDPR Article 89 archival
processing; and the public-procurement
authority running a cultural-data interop
programme.

References (CITATION-POLICY ALLOW only):

- UNESCO 2005 / 1972 / 2003 / 1970 Conventions
  and Operational Guidelines / Directives
- UNIDROIT 1995 Convention
- ISO 15836-1/-2, ISO 21127, ISO 23081-1/-2/-3,
  ISO 25964-1/-2, ISO 23950, ISO 9001
- IIIF Image API 3.0, IIIF Presentation API 3.0,
  IIIF Authentication API 2.0, IIIF Search API
  2.0
- EAD3, RDA Toolkit, METS, MODS, LIDO, EDM
- W3C ODRL 2.2, W3C SKOS, W3C LDP 1.0, W3C VC
  Data Model v2.0
- OAI-PMH 2.0
- ISO/IEC 27001:2022, ISO/IEC 17021-1:2015,
  ISO/IEC 17065:2012
- IETF RFC 8259, RFC 9457, RFC 8615, RFC 9421,
  RFC 6962
- W3C Trace Context, W3C SPARQL 1.1 Protocol
- EU Directive 2019/790 (Articles 14, 17)
- EU GDPR Articles 6, 9, 89
- EU Single Digital Gateway Reg (EU) 2018/1724
  and EU Interoperable Europe Act Reg (EU)
  2024/903
- KR 박물관및미술관진흥법, KR 문화재보호법, KR
  도서관법, KR 공공기록물 관리에 관한 법률

---

## §1 Federated-Aggregator Integration

### §1.1 Europeana integration

A European-jurisdiction operator publishes the
catalogue under the Europeana Data Model (EDM)
through Europeana's harvest endpoint. The
Europeana Aggregation Hub binds the operator's
collection to the Europeana federated portal so
that the operator's records are discoverable
alongside the records of other Member-State
operators.

### §1.2 DPLA integration

A US-jurisdiction operator binds the catalogue
to the Digital Public Library of America hub
under the DPLA Metadata Application Profile.
The operator publishes the harvest endpoint
that the DPLA hub ingests on a declared
cadence.

### §1.3 National aggregator integration

A national aggregator (e.g., the operator's
national digital library, a national heritage
portal) ingests the operator's records under
the national aggregation discipline. The
operator publishes the per-aggregator harvest
endpoint and the per-aggregator metadata
profile.

## §2 IIIF Consortium Integration

The IIIF Consortium publishes the IIIF Image
API and Presentation API specifications. The
operator subscribes to the IIIF specification-
update channel and triggers an internal review
cycle when a specification revision is
published. A specification version that is
deprecated by the IIIF Consortium is marked
deprecated in the operator's enumeration set
but is not removed for the duration of the
operator's record-retention period.

## §3 Authority-File Integration

### §3.1 VIAF / LCNAF / ULAN integration

The operator's per-creator authority records
are bound to a public authority file (VIAF,
LCNAF, ULAN). The operator's API queries the
public authority file to verify the per-
identifier persistence and refuses an
attribution that references a non-resolvable
authority identifier.

### §3.2 Wikidata integration

The operator may additionally bind the per-
creator and per-subject records to Wikidata
items. The operator's API publishes the per-
record Wikidata QID so that a downstream
linked-data consumer can pivot to the
Wikidata graph.

## §4 W3C Working Group Integration

The W3C Linked Data Platform, SKOS, ODRL, and
DCAT specifications are reviewed by W3C
Working Groups on a multi-year cycle. The
operator subscribes to the W3C specification-
publishing endpoint and triggers an internal
review cycle when a new edition is published.

## §5 International Organisation Integration

### §5.1 ICOM membership

A museum operator is bound to its ICOM
(International Council of Museums) membership.
The ICOM Code of Ethics for Museums applies to
the operator's collection-management discipline.

### §5.2 IFLA membership

A library operator is bound to its IFLA
(International Federation of Library
Associations) membership. The IFLA Statement
on Indigenous Traditional Knowledge applies
where the operator's collection includes
indigenous-knowledge materials.

### §5.3 ICA membership

An archive operator is bound to its ICA
(International Council on Archives)
membership. The ICA Code of Ethics applies to
the operator's archival processing discipline.

## §6 UNESCO Secretariat Integration

### §6.1 2005 Convention quadrennial reporting

The UNESCO 2005 Convention quadrennial
reporting cycle is operated by the UNESCO
Diversity of Cultural Expressions Section. The
operator's API publishes the per-cycle
Member-State report through the
secretariat's intake endpoint.

### §6.2 World Heritage Centre integration

The UNESCO World Heritage Centre operates the
state-of-conservation reporting cycle for
World Heritage properties. The operator's API
publishes the per-property periodic report
through the Centre's intake endpoint.

### §6.3 Intangible Cultural Heritage Section

The UNESCO Intangible Cultural Heritage
Section operates the periodic state-of-the-
element reporting for inscribed elements. The
operator's API publishes the per-element
periodic report.

## §7 Restitution-Authority Integration

A claimant State requesting restitution under
the UNESCO 1970 Convention or the UNIDROIT
1995 Convention queries the operator's API for
the catalogue record's provenance trail. The
operator's API publishes the trail and the
underlying acquisition documentation per the
operator's documented restitution-handling
discipline.

## §8 Public Retrieval and Re-Issuance

### §8.1 Public catalogue retrieval

A public consumer (a researcher, an educator,
a journalist, a curator) retrieves the
catalogue record at
`/v1/catalogue-records/{itemId}` without
authentication; the response carries the
public fields and the per-item rights
statement.

### §8.2 IIIF cross-institutional viewer

A IIIF-compliant viewer (Mirador, Universal
Viewer) loads the operator's IIIF Presentation
manifest alongside manifests from other
institutions for cross-institutional
comparison. The viewer's sign-in flow uses the
IIIF Authentication API 2.0 where the operator
restricts access.

### §8.3 Verifiable-credentials re-issuance

An attestation about an item (the public-
domain mark, the per-item provenance summary,
the per-item rights statement) is re-issuable
as a W3C Verifiable Credential signed by the
operator's public-key set so that a downstream
consumer can validate the attestation without
contacting the operator directly.

## §9 GDPR Supervisory Authority Integration

The supervisory data-protection authority
overseeing the operator's GDPR Article 89
archival processing audits the operator's
records of processing activities (Article 30)
on demand. The operator's API publishes the
records to the authority's endpoint so that
the authority's audit trail is preserved.

## §10 KR-Jurisdiction Integration

### §10.1 KR 문화재청 register integration

A KR-jurisdiction operator binds the
catalogue record to the KR Cultural Heritage
Administration's heritage register. The
register is queried on each retrieval after
the caching TTL.

### §10.2 KR 국립중앙도서관 integration

A KR-jurisdiction library operator binds the
catalogue record to the KR National Library
of Korea's union catalogue. The operator
publishes the harvest endpoint that the
National Library ingests.

### §10.3 KR 국가기록원 integration

A KR-jurisdiction archive operator binds the
finding aid to the KR National Archives'
union finding-aid catalogue.

### §10.4 KR 도서관법 binding

The operator declares the KR 도서관법 (Library
Act) reference in the programme record's
`governingFrameworks` set where the operator
is in scope.

### §10.5 KR 공공기록물 관리에 관한 법률 binding

A KR-jurisdiction archive operator declares
the KR 공공기록물 관리에 관한 법률 (Public
Records Management Act) reference in the
programme record's `governingFrameworks` set.

## §11 Audit and Conformity-Assessment
       Integration

### §11.1 ISO/IEC 17021-1 management-system audit

The operator's quality-management system
declared in PHASE-3 §9 is audited under
ISO/IEC 17021-1 by an accredited certification
body. The audit result is stored in the
operator's audit envelope.

### §11.2 ISO/IEC 17065 product certification

Where the operator's collection-care
certification (a museum collection-management
certification, a library preservation
certification) is issued by an external
conformity-assessment body, the body's
ISO/IEC 17065:2012 accreditation is bound to
the certification reference.

## §12 References (consolidated)

The references list across PHASE-1 to PHASE-4
is the canonical citation set for the WIA-
cultural-exchange-data standard. Implementations
cite the standards by their issuing
organisation (UNESCO, UNIDROIT, ISO, IEC, IETF,
W3C, IIIF Consortium, Library of Congress,
SAA, ICOM CIDOC, EU regulatory text, KR
regulatory text) and the publication year so
that a downstream consumer can locate the
authoritative text. Updates to a cited
standard (for example, an amendment to ISO
21127) trigger an internal review cycle in the
operator's quality-management discipline
declared in PHASE-3 §9 before the new
revision is bound into the operator's
enumeration set.
