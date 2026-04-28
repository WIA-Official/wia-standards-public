# WIA-cultural-exchange-data PHASE 3 — PROTOCOL Specification

**Standard:** WIA-cultural-exchange-data
**Phase:** 3 — PROTOCOL
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern
a memory-institution operator across the
operator-to-aggregator-to-researcher value
chain: the cataloguing-scheme discipline that
ties the catalogue record to its declared
description scheme, the IIIF API discipline
that gates the per-image and per-presentation
publication, the EAD3 finding-aid discipline
that gates the archival arrangement-and-
description envelope, the linked-data
discipline that aligns the catalogue record
with the CIDOC CRM ontology, the rights-
expression discipline that governs the per-item
re-use, the provenance-and-attribution
discipline that preserves the chain-of-custody
of every catalogue assertion, the cross-
jurisdictional restitution discipline that
handles an inter-state claim, the privacy
discipline that protects sensitive personal
data carried by archival records, and the
quality-management discipline that maintains
the operator's documented process governance.

References (CITATION-POLICY ALLOW only):

- UNESCO 2005 Convention on the Protection and
  Promotion of the Diversity of Cultural
  Expressions and its Operational Guidelines
- UNESCO 1972 World Heritage Convention,
  Operational Guidelines for the
  Implementation of the World Heritage
  Convention
- UNESCO 2003 Convention for the Safeguarding
  of the Intangible Cultural Heritage,
  Operational Directives
- UNESCO 1970 Convention on the Means of
  Prohibiting and Preventing the Illicit
  Import, Export and Transfer of Ownership of
  Cultural Property
- UNIDROIT Convention on Stolen or Illegally
  Exported Cultural Objects (1995)
- ISO 15836-1:2017, ISO 15836-2:2019 (Dublin
  Core)
- ISO 21127:2014 (CIDOC CRM)
- ISO 23081-1/-2/-3 (records-metadata
  management)
- ISO 25964-1/-2 (thesauri and
  interoperability)
- ISO 23950:1998 (Z39.50)
- IIIF Image API 3.0, IIIF Presentation API
  3.0, IIIF Authentication API 2.0, IIIF Search
  API 2.0
- EAD3, RDA Toolkit, METS, MODS, LIDO, EDM
- W3C ODRL 2.2, W3C SKOS, W3C LDP 1.0, W3C VC
  v2.0
- ISO 9001:2015 (quality management systems)
- ISO/IEC 27001:2022, ISO/IEC 17021-1:2015
- IETF RFC 9110, RFC 9421, RFC 9457, RFC 8615,
  RFC 6962
- W3C Trace Context
- EU Directive 2019/790 on copyright in the
  Digital Single Market (Articles 14, 17)
- EU GDPR Articles 6, 9, 89 (processing for
  archiving in the public interest, scientific
  or historical research, or statistical
  purposes)
- KR 박물관 및 미술관 진흥법 (Museum and Art
  Gallery Promotion Act) and KR 문화재보호법
  (Cultural Heritage Protection Act)

---

## §1 Cataloguing-Scheme Discipline

### §1.1 Scheme-to-item-type binding

Every catalogue record carries the
`cataloguingScheme` enumeration declared in
PHASE-1 §4. The operator's API enforces the
scheme-to-item-type mapping table:

- `LIDO` — for `physical-object` and
  `monument` records.
- `MODS` — for `text` and `manuscript`
  bibliographic records.
- `EAD3` — for `archival-collection` records.
- `Dublin-Core-15836-1/-2` — for any item type
  as a baseline interoperable set.
- `EDM` — for records contributed to Europeana
  or a comparable aggregator.
- `CIDOC-CRM` — for the linked-data graph
  representation of any item type.
- `RDA-Toolkit` — for cataloguing rules
  applied to the bibliographic records.

### §1.2 Per-scheme schema validation

The operator's API validates the catalogue
record against the declared scheme's schema —
the LIDO XSD, the MODS XSD, the EAD3 XSD, the
EDM RDF profile — and refuses a record that
does not validate.

## §2 IIIF API Discipline

### §2.1 Image API conformance level

Every IIIF Image API endpoint declares its
conformance level (Level 0, Level 1, Level 2)
per the IIIF Image API 3.0 specification §5.
The operator's discovery document publishes the
declared level so that a downstream consumer
(a viewer, an annotation tool) can adapt its
request envelope accordingly.

### §2.2 Presentation API manifest validation

The operator's API validates a published
Presentation API manifest against the IIIF
Presentation API 3.0 schema and refuses a
publication where the manifest does not
validate (a `Range` referencing a missing
`Canvas`, a `Service` referencing a non-IIIF
endpoint).

### §2.3 Authentication API discipline

Where the operator restricts access to a
manifest (a copyright-restricted image, a
sensitive-cultural-content item under indigenous-
data-sovereignty discipline), the operator
publishes the IIIF Authentication API 2.0
service URI in the manifest. The viewer
authenticates against the operator's
authentication endpoint before receiving the
restricted resource.

## §3 Cross-Jurisdictional Restitution Discipline

### §3.1 1970 UNESCO Convention binding

A restitution claim under the UNESCO 1970
Convention on the Means of Prohibiting and
Preventing the Illicit Import, Export and
Transfer of Ownership of Cultural Property
binds the operator's catalogue record to the
claim record. The operator publishes the per-
item provenance trail so that a claimant State
can verify the item's origin and chain of
ownership.

### §3.2 UNIDROIT 1995 Convention binding

Where the claim is in scope of the UNIDROIT
1995 Convention on Stolen or Illegally Exported
Cultural Objects, the operator binds the claim
to the UNIDROIT discipline so that the
restitution request is processed under the
applicable transnational framework.

### §3.3 Provenance research record

The operator's API publishes a provenance
research record for every item whose
acquisition predates the operator's modern
acquisition discipline. The provenance record
carries the item's known ownership history
between 1933 and 1945 (the Washington
Principles on Nazi-Confiscated Art declaration)
and any post-colonial provenance research
applicable to colonial-era acquisitions.

## §4 Rights-Expression Discipline

### §4.1 W3C ODRL profile

Every catalogue record carries a W3C ODRL 2.2
policy in `rightsExpression`. The operator's
API parses the policy and serves the per-item
rights statement on the public retrieval
endpoint so that a downstream re-user can
verify the permitted scope of re-use.

### §4.2 Public-domain and out-of-copyright
       discipline

A record marked as public-domain (the public-
domain mark per the Creative Commons rights-
statement profile) carries the basis for the
public-domain determination — the per-
jurisdiction copyright term has expired, the
work was authored before the per-jurisdiction
copyright threshold, or the rights have been
formally dedicated.

### §4.3 EU Directive 2019/790 Article 14
       discipline

Under EU Directive 2019/790 Article 14, a
faithful reproduction of a public-domain
visual artwork is itself public-domain in EU
Member States. The operator's API enforces this
discipline by refusing a copyright claim on a
faithful reproduction of a public-domain work
in the EU jurisdiction.

## §5 Provenance-and-Attribution Discipline

### §5.1 Per-assertion attribution

Every catalogue assertion (a creator
attribution, a date, a place of creation, a
subject classification) carries the source of
the assertion (the operator's curator, the
acquisition record, an external reference) so
that the assertion's basis can be audited.

### §5.2 Authority-record binding

Per-creator authority records are bound to a
public authority file — the Virtual
International Authority File (VIAF), the
Library of Congress Name Authority File
(LCNAF), the Getty Union List of Artist Names
(ULAN), the operator's own local authority. A
creator attribution that is not bound to a
public authority is annotated as an
unverified attribution in the catalogue
record.

## §6 Privacy Discipline for Archival Records

### §6.1 GDPR Article 89 archiving basis

An archival record containing personal data is
processed under GDPR Article 89(1) processing
for archiving in the public interest. The
operator's API records the per-record Article
89 processing basis and applies the safeguards
required by the operator's national archival
law (closure periods, redaction-on-access,
researcher-access agreement).

### §6.2 Sensitive-cultural-content discipline

A record carrying sensitive cultural content
(indigenous ceremonial knowledge, secret-
sacred materials, materials subject to
descendant-community-decided access controls)
is bound to a per-record access policy
consistent with the relevant indigenous-data-
sovereignty principles (the CARE Principles
for Indigenous Data Governance — Collective
benefit, Authority to control, Responsibility,
Ethics).

## §7 Linked-Data Graph Discipline

### §7.1 CIDOC CRM ontology binding

Every linked-data graph record published under
PHASE-1 §7 binds its entities and properties to
the CIDOC CRM (ISO 21127) ontology version
declared in `ontologySet`. The operator's API
validates the binding against the CRM's class-
and-property hierarchy.

### §7.2 SKOS vocabulary alignment

Subject classifications are published as W3C
SKOS concept schemes. The operator publishes the
per-scheme concept hierarchy with `skos:broader`
and `skos:narrower` relations so that a
downstream query can traverse the hierarchy.

## §8 Chain-of-Custody Anchoring Discipline

### §8.1 Per-event transparency log

Every chain-of-custody event carried by PHASE-1
§8 is appended to a per-operator transparency
log modelled on the IETF RFC 6962 Certificate
Transparency append-only-log structure. The log
publishes a signed tree-head every signed-tree-
head period (default 24 h, configurable per
programme).

### §8.2 Mutation prevention

A custody event cannot be retroactively edited;
an amendment is recorded as a new event with
`previousEventRef` pointing at the event being
amended.

## §9 Quality-Management Discipline

The operator runs an ISO 9001:2015 quality
management system covering the cataloguing,
IIIF publication, finding-aid publication,
graph publication, OAI-PMH harvest, and chain-
of-custody processes. Internal audits run on a
frequency declared in the quality manual; the
nonconformity register is reviewed in the ISO
9001 §9.3 management-review cycle.

## §10 OAI-PMH Harvest Discipline

### §10.1 Selective harvest under set

The operator's OAI-PMH endpoint exposes
selective harvest under `set` (per-collection
sub-trees, per-rights-statement sub-trees, per-
language sub-trees). A harvester pulls only the
records matching its declared interest.

### §10.2 Deletion policy

The operator declares its deletion policy in
the OAI-PMH `Identify` response (no, persistent,
or transient). A persistent-deletion operator
records the per-item deletion timestamp in the
harvest envelope so that a downstream cache can
be reconciled.

## §11 UNESCO Programme Discipline

### §11.1 Quadrennial reporting cadence

The UNESCO 2005 Convention quadrennial
periodic-reporting cadence binds the operator's
programme record to the per-Member-State
report deadline. The operator's API records
the per-cycle report identifier and the
underlying preparation evidence.

### §11.2 World Heritage state-of-conservation

Where the operator is bound to a UNESCO World
Heritage property, the periodic state-of-
conservation report is published under the
UNESCO 1972 Convention's Operational
Guidelines. The operator's API records the
per-cycle report's submission to the UNESCO
World Heritage Centre.

### §11.3 Intangible Cultural Heritage state-of-
       the-element

Where the operator is bound to an Intangible
Cultural Heritage element, the periodic state-
of-the-element report is published under the
UNESCO 2003 Convention's Operational
Directives.

## §12 KR-Jurisdiction Discipline

### §12.1 KR 문화재보호법 binding

A KR-jurisdiction operator binds the catalogue
record to the relevant article of KR 문화재
보호법 (Cultural Heritage Protection Act) — the
국가지정문화재 (state-designated heritage), the
시·도지정문화재, the 등록문화재, or the 무형
문화재. The KR Cultural Heritage Administration
operates the heritage register; the operator's
API queries the register on each retrieval
after the caching TTL.

### §12.2 KR 박물관및미술관진흥법 binding

The operator's accreditation under KR 박물관
및 미술관 진흥법 (Museum and Art Gallery
Promotion Act) is recorded in the programme
record's `accreditationStatus`. The KR Ministry
of Culture, Sports and Tourism operates the
accreditation register.
